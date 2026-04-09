// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <asm-generic/qspinlock.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/bpf_komb.h>

struct shadow_stack {
	union {
		struct {
			void *lock_addr;
			void *ptr;
			uint32_t curr_cs_cpu;
			uint32_t prev_cs_cpu;
			uint64_t counter_val;
			struct komb_node *next_node_ptr;
			void *local_shadow_stack_ptr;
			struct komb_node *local_queue_head;
			struct komb_node *local_queue_tail;
		};
		char dummy[128];
	};
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct komb_node, komb_nodes[MAX_NODES]);
static DEFINE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

static inline __pure u32 encode_tail(int cpu, int idx)
{
	u32 tail;

	tail = (cpu + 1) << _Q_TAIL_CPU_OFFSET;
	tail |= idx << _Q_TAIL_IDX_OFFSET; /* assume < 4 */

	return tail;
}

static inline __pure struct komb_node *decode_tail(u32 tail)
{
	int cpu = (tail >> _Q_TAIL_CPU_OFFSET) - 1;
	int idx = (tail & _Q_TAIL_IDX_MASK) >> _Q_TAIL_IDX_OFFSET;

	return per_cpu_ptr(&komb_nodes[idx], cpu);
}

inline __pure u32 get_cpu_from_tail(u32 tail)
{
	return ((tail >> _Q_TAIL_CPU_OFFSET) - 1);
}

__always_inline void clear_locked_set_completed(struct komb_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

__always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked_pending, _Q_LOCKED_VAL);
}

static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	return ((u32)xchg(&lock->tail, tail >> _Q_TAIL_OFFSET))
	       << _Q_TAIL_OFFSET;
}

__always_inline u32 cmpxchg_tail(struct qspinlock *lock, u32 tail, u32 new_tail)
{
	return ((u32)cmpxchg(&lock->tail, tail >> _Q_TAIL_OFFSET,
			     new_tail >> _Q_TAIL_OFFSET))
	       << _Q_TAIL_OFFSET;
}

static __always_inline void clear_pending(struct qspinlock *lock)
{
	atomic_andnot(_Q_PENDING_VAL, &lock->val);
}

static __always_inline void set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked, _Q_LOCKED_VAL);
}

static __always_inline void check_and_set_combiner(struct qspinlock *lock)
{
	u32 val, new_val;

	if (lock->locked > 0) {
		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_COMBINER_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) ==
			    val)
				return;
		}
	}
	WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
}

static __always_inline u32 komb_fetch_set_pending_acquire(struct qspinlock *lock)
{
	return atomic_fetch_or_acquire(_Q_PENDING_VAL, &lock->val);
}

__always_inline static void add_to_local_queue(struct komb_node *node)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (ptr->local_queue_head == NULL) {
		ptr->local_queue_head = node;
		ptr->local_queue_tail = node;
	} else {
		ptr->local_queue_tail->next = node;
		ptr->local_queue_tail = node;
	}
}

__always_inline static struct komb_node *
get_next_node(struct komb_node *my_node)
{
	int i;
	struct komb_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->next == NULL)
			goto next_node_null;

		if (next_node->socket_id == numa_node_id()) {
			void *rsp_ptr = (void *)(per_cpu_ptr(&komb_nodes[0],
							     next_node->cpuid)
							 ->rsp);
			prefetchw(rsp_ptr);
			for (i = 1; i < NUM_PREFETCH_LINES; i++)
				prefetchw(rsp_ptr + (64 * i));

			prefetch(next_node->next);
			return next_node;
		}

		add_to_local_queue(next_node);
		curr_node = next_node;
		next_node = curr_node->next;
	}

next_node_null:
	return next_node;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
execute_cs(struct qspinlock *lock, struct komb_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct komb_node *next_node = NULL;

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &(ptr->local_shadow_stack_ptr);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
}
#pragma GCC pop_options

__always_inline static void run_combiner(struct qspinlock *lock,
					 struct komb_node *curr_node)
{
	struct shadow_stack *ptr;
	struct komb_node *next_node = curr_node->next;

	if (next_node == NULL) {
		set_locked(lock);
		curr_node->locked = false;
		smp_mb();
		return;
	}

	ptr = this_cpu_ptr(&local_shadow_stack);
	curr_node->rsp = NULL;
	ptr->counter_val = 0;
	ptr->next_node_ptr = NULL;
	ptr->local_queue_head = NULL;
	ptr->local_queue_tail = NULL;

	ptr->curr_cs_cpu = curr_node->cpuid;
	ptr->lock_addr = lock;
	ptr->counter_val = 0;

	execute_cs(lock, curr_node);

	if (ptr->prev_cs_cpu != -1) {
		clear_locked_set_completed(
			per_cpu_ptr(&komb_nodes[0], ptr->prev_cs_cpu));
		ptr->prev_cs_cpu = -1;
	}

	next_node = ptr->next_node_ptr;

	if (ptr->local_queue_head != NULL) {
		ptr->local_queue_tail->next = next_node;
		next_node = ptr->local_queue_head;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
	}

	set_locked(lock);
	next_node->locked = false;

	ptr->lock_addr = NULL;
	ptr->curr_cs_cpu = -1;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static int
__komb_spin_lock_longjmp(struct qspinlock *lock)
{
	register struct komb_node *curr_node;
	struct komb_node *prev_node = NULL, *next_node = NULL;
	struct qspinlock *parent_lock;
	int old_tail, val, j;
	int i;

	int tail, idx;

	curr_node = this_cpu_ptr(&komb_nodes[0]);
	idx = curr_node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	curr_node->locked = true;
	curr_node->completed = false;
	curr_node->next = NULL;
	curr_node->tail = tail;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		prev_node->next = curr_node;
		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

		if (curr_node->completed) {
			curr_node->count--;
			for (i = 0; i < NUM_PREFETCH_LINES; i++)
				prefetchw(((char *)curr_node->rsp) + (64 * i));
			return 0;
		}
	}

	val = atomic_cond_read_acquire(&lock->val,
				       !(VAL & _Q_LOCKED_PENDING_MASK));

	if (((val & _Q_TAIL_MASK) == tail) &&
	    atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL))
		goto release;

	check_and_set_combiner(lock);

	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	next_node = curr_node->next;

	curr_node->count--;
	run_combiner(lock, next_node);
	return 0;

release:
	curr_node->count--;
	return 0;
}
#pragma GCC pop_options

void komb_init(void)
{
	int i, j;
	for_each_possible_cpu(i) {
		void *stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		struct shadow_stack *ptr = per_cpu_ptr(&local_shadow_stack, i);

		ptr->ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		ptr->lock_addr = NULL;
		ptr->local_shadow_stack_ptr =
			stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = -1;
		ptr->counter_val = 0;
		ptr->next_node_ptr = NULL;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
	}
}

void komb_spin_lock_init(struct qspinlock *lock)
{
	atomic_set(&lock->val, 0);
}

__attribute__((noipa)) noinline notrace static void *get_shadow_stack_ptr(void)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
	return &(ptr->local_shadow_stack_ptr);
}

__attribute__((noipa)) noinline notrace static struct komb_node *
get_komb_node(void)
{
	return this_cpu_ptr(&komb_nodes[0]);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace void
komb_spin_lock_slowpath(struct qspinlock *lock)
{
	register int ret_val;
	asm volatile("pushq %%rbp\n"
		     "pushq %%rbx\n"
		     "pushq %%r12\n"
		     "pushq %%r13\n"
		     "pushq %%r14\n"
		     "pushq %%r15\n"
		     :
		     :
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq %%rsp, %c1(%%rax)\n"
		     :
		     : "i"(get_komb_node), "i"(offsetof(struct komb_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");

	__komb_spin_lock_slowpath(lock);

	asm volatile("callq %P0\n"
		     "movq %%rsp, (%%rax)\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq %c1(%%rax), %%rsp\n"
		     :
		     : "i"(get_komb_node), "i"(offsetof(struct komb_node, rsp))
		     : "memory");
	asm volatile("popq %%r15\n"
		     "popq %%r14\n"
		     "popq %%r13\n"
		     "popq %%r12\n"
		     "popq %%rbx\n"
		     "popq %%rbp\n"
		     "retq\n"
		     :
		     :
		     : "memory");
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace void
komb_spin_lock(struct qspinlock *lock)
{
	u32 val, cnt;
	struct komb_node *curr_node = NULL;
	struct shadow_stack *ptr;
	uint32_t curr_cpuid, prev_cpuid;
	struct komb_node *prev_node, *next_node;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		return;

	if (val == _Q_PENDING_VAL) {
		cnt = _Q_PENDING_LOOPS;
		val = atomic_cond_read_relaxed(
			&lock->val, (VAL != _Q_PENDING_VAL) || !cnt--);
	}

	if (val & ~_Q_LOCKED_MASK)
		goto queue;

	val = komb_fetch_set_pending_acquire(lock);

	if (unlikely(val & ~_Q_LOCKED_MASK)) {
		if (!(val & _Q_PENDING_MASK))
			clear_pending(lock);

		goto queue;
	}

	if (val & _Q_LOCKED_MASK)
		atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_MASK));

	clear_pending_set_locked(lock);
	return;

queue:

	curr_node = this_cpu_ptr(&komb_nodes[0]);
	komb_spin_lock_slowpath(lock);

	ptr = this_cpu_ptr(&local_shadow_stack);
	curr_cpuid = ptr->curr_cs_cpu;
	if (curr_cpuid != -1) {
		curr_node = per_cpu_ptr(&komb_nodes[0], curr_cpuid);
		if (curr_node->lock == lock) {
			next_node = get_next_node(curr_node);
			ptr->next_node_ptr = next_node;
		}

		prev_cpuid = ptr->prev_cs_cpu;
		if (prev_cpuid != -1) {
			prev_node = per_cpu_ptr(&komb_nodes[0], prev_cpuid);

			clear_locked_set_completed(prev_node);
			ptr->prev_cs_cpu = -1;
		}
	}
}
EXPORT_SYMBOL_GPL(komb_spin_lock);

__attribute__((noipa)) noinline notrace void
komb_spin_unlock(struct qspinlock *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
	int from_cpuid = ptr->curr_cs_cpu;
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct komb_node *curr_node;
	struct komb_node *next_node;
	uint64_t counter;
	if (ptr->lock_addr == NULL) {
		lock->locked = false;
		return;
	}

	curr_node = per_cpu_ptr(&komb_nodes[0], from_cpuid);
	next_node = ptr->next_node_ptr;

	counter = ptr->counter_val;

	if (next_node == NULL || next_node->next == NULL ||
	    counter >= komb_batch_size || need_resched()) {
		incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = curr_node->cpuid;

	} else {
		ptr->curr_cs_cpu = next_node->cpuid;
		ptr->prev_cs_cpu = curr_node->cpuid;
		incoming_rsp_ptr = &(next_node->rsp);
		ptr->counter_val = counter + 1;
	}
	outgoing_rsp_ptr = &(curr_node->rsp);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	return;
}
EXPORT_SYMBOL_GPL(komb_spin_unlock);
