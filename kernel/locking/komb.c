// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

/*
 * TODO: (Performance optimiztion)
 * Fix the index part when same is acquired multiple times.
 * Currently nested locking becomes TTAS lock.
 * irqs_disabled() will hurt performance when running in VM.
 */
#include <asm-generic/qspinlock.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

//#define DSM_DEBUG 1
#ifdef DSM_DEBUG
#define print_debug(fmt, ...)                                              \
	({                                                                 \
		printk(KERN_EMERG "[%d] komb (%s) lock(%px): " fmt,        \
		       smp_processor_id(), __func__, lock, ##__VA_ARGS__); \
	})
#else
#define print_debug(fmt, ...)
#endif

#define SIZE_OF_SHADOW_STACK 8192L
#define IRQ_NUMA_NODE 255

//#define DEBUG_KOMB 1

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#define UINT64_MAX 0xffffffffffffffffL

#define smp_cond_load_relaxed_sched(ptr, cond_expr) \
	({                                          \
		typeof(ptr) __PTR = (ptr);          \
		__unqual_scalar_typeof(*ptr) VAL;   \
		for (;;) {                          \
			VAL = READ_ONCE(*__PTR);    \
			if (cond_expr)              \
				break;              \
			cpu_relax();                \
		}                                   \
		(typeof(*ptr))VAL;                  \
	})

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                 \
	({                                                          \
		__unqual_scalar_typeof(*ptr) _val;                  \
		_val = smp_cond_load_relaxed_sched(ptr, cond_expr); \
		smp_acquire__after_ctrl_dep();                      \
		(typeof(*ptr))_val;                                 \
	})
#endif

#define atomic_cond_read_acquire_sched(v, c) \
	smp_cond_load_acquire_sched(&(v)->counter, (c))

struct shadow_stack {
	uint64_t lock_addr[8];
	void *ptr;
	uint32_t curr_cs_cpu;
	uint32_t prev_cs_cpu;
	uint64_t counter_val;
	struct komb_node *next_node_ptr;
	uint64_t local_shadow_stack_ptr;
	struct komb_node *local_queue_head;
	struct komb_node *local_queue_tail;
	int irqs_disabled;

} __cacheline_aligned_in_smp;

#ifdef KOMB_STATS
DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, waiter_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, ooo_combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, ooo_waiter_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, ooo_unlocks);
#endif

extern bool do_fds_fallback;
extern bool do_tas_fallback;

static DEFINE_PER_CPU_SHARED_ALIGNED(struct komb_node, komb_nodes[MAX_NODES]);
static DEFINE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)
#define _Q_LOCKED_COMBINER_VAL 3
#define _Q_UNLOCKED_OOO_VAL 7 //Unlocked a lock out-of-order
#define _Q_LOCKED_IRQ_VAL 15 // Lock Stealing by IRQ

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

static inline bool check_irq_node(struct komb_node *node)
{
	return (node->socket_id == IRQ_NUMA_NODE || node->rsp == 0xdeadbeef);
}

__always_inline void clear_locked_set_completed(struct komb_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

__always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	KOMB_BUG_ON(lock->locked != 0);
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

	KOMB_BUG_ON(lock->locked == _Q_LOCKED_VAL);

	if (lock->locked > 0) {
		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));
			KOMB_BUG_ON(lock->locked != 0);
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
	struct komb_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->next == NULL)
			goto next_node_null;
		else if (check_irq_node(next_node) ||
			 check_irq_node(next_node->next))
			goto next_node_null;

		if (next_node->socket_id == numa_node_id()) {
			void *rsp_ptr =
				(per_cpu_ptr(&komb_nodes[0], next_node->cpuid)
					 ->rsp);
			prefetchw(rsp_ptr);
			int i;
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

	ptr->curr_cs_cpu = curr_node->cpuid;

	KOMB_BUG_ON(curr_node->cpuid == smp_processor_id());
	KOMB_BUG_ON((ptr->ptr - (ptr->local_shadow_stack_ptr)) >
		    SIZE_OF_SHADOW_STACK);

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &(ptr->local_shadow_stack_ptr);

	KOMB_BUG_ON(irqs_disabled());
	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == NULL);
	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == NULL);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	KOMB_BUG_ON((ptr->ptr - (ptr->local_shadow_stack_ptr)) >
		    SIZE_OF_SHADOW_STACK);

	if (lock->locked == _Q_UNLOCKED_OOO_VAL) {
		print_debug("Combiner got control back OOO unlock\n");

#ifdef KOMB_STATS
		this_cpu_add(ooo_waiter_combined, ptr->counter_val);
		this_cpu_inc(ooo_combiner_count);
#endif

		if (ptr->curr_cs_cpu != -1) {
			print_debug("OOO waking up %d\n", ptr->curr_cs_cpu);
			curr_node =
				per_cpu_ptr(&komb_nodes[0], ptr->curr_cs_cpu);
			curr_node->rsp = this_cpu_ptr(&komb_nodes[0])->rsp;
			clear_locked_set_completed(curr_node);
			KOMB_BUG_ON(ptr->prev_cs_cpu != -1);
		}
		ptr->prev_cs_cpu = -1;
		ptr->curr_cs_cpu = -1;
		lock->locked = _Q_LOCKED_COMBINER_VAL;

		next_node = ptr->next_node_ptr;

		if (next_node != NULL && next_node->next != NULL &&
		    !check_irq_node(next_node) &&
		    !check_irq_node(next_node->next)) {
			execute_cs(lock, ptr->next_node_ptr);
		}
	}
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
run_combiner(struct qspinlock *lock, struct komb_node *curr_node)
{
	KOMB_BUG_ON(curr_node == NULL);
	struct komb_node *next_node = curr_node->next;
	int counter = 0;

	if (next_node == NULL || check_irq_node(curr_node) ||
	    check_irq_node(next_node)) {
		set_locked(lock);
		curr_node->locked = false;
		smp_mb();
		return;
	}

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->counter_val = 0;

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	execute_cs(lock, curr_node);

	print_debug(
		"Combiner got the control back: %d counter: %d last_waiter: %d\n",
		smp_processor_id(), ptr->counter_val, ptr->curr_cs_cpu);

#ifdef KOMB_STATS
	this_cpu_add(waiter_combined, ptr->counter_val);
	this_cpu_inc(combiner_count);
#endif

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

	print_debug("After combiner %d, next node: %d\n", smp_processor_id(),
		    next_node->cpuid);

	KOMB_BUG_ON(next_node == NULL);

	set_locked(lock);

	ptr->curr_cs_cpu = -1;
	next_node->locked = false;
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static int
__komb_spin_lock_longjmp(struct qspinlock *lock, int tail,
			 register struct komb_node *curr_node)
{
	register struct komb_node *prev_node = NULL, *next_node = NULL;
	struct qspinlock *parent_lock;
	int old_tail, val, j;

	uint32_t prev_cs_cpu;
	bool prev_locked_val;
	uint64_t prev_rsp;
	uint64_t prev_counter_val;
	struct komb_node *prev_next_node_ptr = NULL;
	struct komb_node *prev_local_queue_head;
	struct komb_node *prev_local_queue_tail;

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		prev_node->next = curr_node;
		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));
		struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

		if (curr_node->completed) {
			if (curr_node->irqs_disabled)
				ptr->irqs_disabled = curr_node->irqs_disabled;

			for (j = 7; j >= 0; j--)
				if (ptr->lock_addr[j] != NULL)
					break;

			curr_node->count--;

			if (j >= 0) {
				parent_lock = ptr->lock_addr[j];
				KOMB_BUG_ON(parent_lock == lock);
				if (parent_lock->locked ==
				    _Q_UNLOCKED_OOO_VAL) {
					print_debug("Waiter unlocked OOO\n");
					return 1;
				}
			}

			prefetchw(curr_node->rsp);
			prefetchw(curr_node->rsp + 64);
			prefetchw(curr_node->rsp + 128);
			prefetchw(curr_node->rsp + 192);
			prefetchw(curr_node->rsp + 256);
			prefetchw(curr_node->rsp + 320);
			return 0;
		}
	}

	val = atomic_cond_read_acquire(&lock->val,
				       !(VAL & _Q_LOCKED_PENDING_MASK));

	if (((val & _Q_TAIL_MASK) == tail) &&
	    atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL))
		goto release; /* No contention */

	check_and_set_combiner(lock);

	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	next_node = curr_node->next;
	KOMB_BUG_ON(next_node == NULL);

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	prev_cs_cpu = ptr->curr_cs_cpu;
	ptr->curr_cs_cpu = -1;
	ptr->irqs_disabled = false;
	KOMB_BUG_ON(ptr->prev_cs_cpu != -1);

	curr_node->count--;
	prev_locked_val = lock->locked;
	KOMB_BUG_ON(prev_locked_val >= _Q_LOCKED_COMBINER_VAL);

	lock->locked = _Q_LOCKED_COMBINER_VAL;
	prev_rsp = curr_node->rsp;
	curr_node->rsp = NULL;

	prev_counter_val = ptr->counter_val;
	ptr->counter_val = 0;

	prev_next_node_ptr = ptr->next_node_ptr;
	ptr->next_node_ptr = NULL;

	prev_local_queue_head = ptr->local_queue_head;
	prev_local_queue_tail = ptr->local_queue_tail;

	ptr->local_queue_head = NULL;
	ptr->local_queue_tail = NULL;

	j = 7;
	for (j = 7; j >= 0; j--)
		if (ptr->lock_addr[j] != NULL)
			break;
	j += 1;
	KOMB_BUG_ON(j >= 8 || j < 0);
	ptr->lock_addr[j] = lock;

	run_combiner(lock, next_node);

	KOMB_BUG_ON(this_cpu_ptr(&local_shadow_stack)->lock_addr[j] != lock);

	ptr->lock_addr[j] = NULL;

	ptr->next_node_ptr = prev_next_node_ptr;
	ptr->counter_val = prev_counter_val;
	ptr->local_queue_head = prev_local_queue_head;
	ptr->local_queue_tail = prev_local_queue_tail;

	curr_node->rsp = prev_rsp;

	if (lock->locked == _Q_UNLOCKED_OOO_VAL) {
		if (prev_cs_cpu != -1) {
			print_debug("Waking up %d\n", prev_cs_cpu);
			clear_locked_set_completed(
				per_cpu_ptr(&komb_nodes[0], prev_cs_cpu));
		}
		ptr->curr_cs_cpu = -1;
	} else {
		ptr->curr_cs_cpu = prev_cs_cpu;
	}
	lock->locked = prev_locked_val;

	return 0;
release:
	curr_node->count--;
	return 0;
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace static int
__komb_spin_lock_slowpath(struct qspinlock *lock)
{
	struct komb_node *curr_node;
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
	curr_node->irqs_disabled = false;
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;

	return __komb_spin_lock_longjmp(lock, tail, curr_node);
}

void komb_init(void)
{
	int i, j;
	for_each_possible_cpu(i) {
		void *stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		struct shadow_stack *ptr = per_cpu_ptr(&local_shadow_stack, i);
		KOMB_BUG_ON(stack_ptr == NULL);

		ptr->ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		for (j = 0; j < 8; j++)
			ptr->lock_addr[j] = 0;
		ptr->local_shadow_stack_ptr =
			stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = -1;
		ptr->counter_val = 0;
		ptr->next_node_ptr = NULL;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;

		ptr->irqs_disabled = false;
	}
}

void komb_free(void)
{
	int i;
	for_each_possible_cpu(i) {
		vfree(per_cpu_ptr(&local_shadow_stack, i)->ptr -
		      SIZE_OF_SHADOW_STACK);
	}
}

void komb_spin_lock_init(struct qspinlock *lock)
{
	atomic_set(&lock->val, 0);
}

__attribute__((noipa)) noinline notrace static uint64_t
get_shadow_stack_ptr(void)
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
	KOMB_BUG_ON(irqs_disabled());

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

	ret_val = __komb_spin_lock_slowpath(lock);

	if (ret_val) {
		asm volatile("callq %P0\n"
			     "movq (%%rax), %%rsp\n"
			     "popq %%r15\n"
			     "popq %%r14\n"
			     "popq %%r13\n"
			     "popq %%r12\n"
			     "popq %%rbx\n"
			     "popq %%rbp\n"
			     "retq\n"
			     :
			     : "i"(get_shadow_stack_ptr)
			     : "memory");
	} else {
		asm volatile("callq %P0\n"
			     "movq %%rsp, (%%rax)\n"
			     :
			     : "i"(get_shadow_stack_ptr)
			     : "memory");
		asm volatile("callq %P0\n"
			     "movq %c1(%%rax), %%rsp\n"
			     :
			     : "i"(get_komb_node),
			       "i"(offsetof(struct komb_node, rsp))
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
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace void
komb_spin_lock(struct qspinlock *lock)
{
	u32 val, cnt;
	struct komb_node *curr_node = NULL;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		return;

	if (val == _Q_PENDING_VAL) {
		cnt = _Q_PENDING_LOOPS;
		val = atomic_cond_read_relaxed(
			&lock->val, (VAL != _Q_PENDING_VAL) || !cnt--);
	}

	if (val & ~_Q_LOCKED_MASK ||
	    (val & _Q_LOCKED_MASK) == _Q_LOCKED_IRQ_VAL)
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
	KOMB_BUG_ON(curr_node == NULL);

	if (curr_node->count > 0 || !in_task() || irqs_disabled() ||
	    current->migration_disabled || do_fds_fallback) {
		struct komb_node *prev_node, *next_node;
		u32 tail, idx;

		idx = curr_node->count++;

		if (do_tas_fallback || unlikely(idx >= MAX_NODES)) {
			while (!komb_spin_trylock(lock))
				cpu_relax();
			goto irq_release;
		}

		curr_node += idx;
		tail = encode_tail(smp_processor_id(), idx);

		barrier();

		curr_node->locked = true;
		curr_node->completed = false;
		curr_node->next = NULL;
		curr_node->tail = tail;
		curr_node->socket_id = IRQ_NUMA_NODE;
		curr_node->cpuid = smp_processor_id();
		curr_node->irqs_disabled = false;
		curr_node->lock = lock;
		curr_node->task_struct_ptr = current;

		uint64_t prev_rsp = curr_node->rsp;
		curr_node->rsp = 0xdeadbeef;

		smp_wmb();

		u32 old_tail = xchg_tail(lock, tail);

		if (old_tail & _Q_TAIL_MASK) {
			prev_node = decode_tail(old_tail);
			WRITE_ONCE(prev_node->next, curr_node);

			print_debug("IRQ going to waiting for lock\n");
			smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));
		}

		curr_node->rsp = prev_rsp;

		u32 val, new_val;

		print_debug("IRQ spinning on the locked field\n");

		val = atomic_cond_read_acquire(&lock->val,
					       !(VAL & _Q_LOCKED_PENDING_MASK));

		if (((val & _Q_TAIL_MASK) == tail) &&
		    atomic_try_cmpxchg_relaxed(&lock->val, &val,
					       _Q_LOCKED_IRQ_VAL)) {
			print_debug("IRQ only one in the queue unlocked\n");
			goto irq_release;
		}

		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			KOMB_BUG_ON(lock->locked != 0);

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_IRQ_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) ==
			    val)
				break;
		}

		print_debug("IRQ got the lock\n");

		smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
		next_node = curr_node->next;

		KOMB_BUG_ON(next_node == NULL);

		WRITE_ONCE(next_node->locked, false);

		print_debug("IRQ passing lock next node: %d\n",
			    next_node->cpuid);

irq_release:
		curr_node = this_cpu_ptr(&komb_nodes[0]);
		curr_node->count--;
		return;
	} else {
		komb_spin_lock_slowpath(lock);

		struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

		uint32_t curr_cpuid = ptr->curr_cs_cpu;
		if (curr_cpuid != -1) {
			struct komb_node *curr_node =
				per_cpu_ptr(&komb_nodes[0], curr_cpuid);
			if (curr_node->lock == lock) {
				KOMB_BUG_ON(lock->locked !=
					    _Q_LOCKED_COMBINER_VAL);
				struct komb_node *next_node =
					get_next_node(curr_node);
				ptr->next_node_ptr = next_node;
			}

			uint32_t prev_cpuid = ptr->prev_cs_cpu;
			if (prev_cpuid != -1) {
				struct komb_node *prev_node =
					per_cpu_ptr(&komb_nodes[0], prev_cpuid);

				KOMB_BUG_ON(prev_node->lock != lock);

				print_debug("Waking up prev waiter: %d\n",
					    prev_cpuid);
				clear_locked_set_completed(prev_node);
				ptr->prev_cs_cpu = -1;
			}
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

	int j, max_idx, my_idx;

	uint64_t temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	if (lock->locked == _Q_LOCKED_VAL ||
	    lock->locked == _Q_LOCKED_IRQ_VAL) {
		lock->locked = false;
		return;
	}

	for (j = 0; j < 8; j++) {
		temp_lock_addr = ptr->lock_addr[j];
		if (temp_lock_addr != NULL)
			max_idx = j;
		if (temp_lock_addr == lock)
			my_idx = j;
		if (temp_lock_addr == NULL)
			break;
	}

	if (my_idx == -1) {
		if (lock->locked == _Q_LOCKED_VAL ||
		    lock->locked == _Q_LOCKED_IRQ_VAL)
			lock->locked = false;
		else if (lock->locked == _Q_LOCKED_COMBINER_VAL) {
#ifdef KOMB_STATS
			this_cpu_inc(ooo_unlocks);
#endif
			lock->locked = _Q_UNLOCKED_OOO_VAL;
			print_debug("OOO unlock\n");
		} else
			BUG_ON(true);
		return;
	}

	KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
	KOMB_BUG_ON(from_cpuid == -1);
	KOMB_BUG_ON(max_idx < 0);

	if (my_idx < max_idx) {
#ifdef KOMB_STATS
		this_cpu_inc(ooo_unlocks);
#endif
		lock->locked = _Q_UNLOCKED_OOO_VAL;
		return;
	}

	struct komb_node *curr_node = per_cpu_ptr(&komb_nodes[0], from_cpuid);
	struct komb_node *next_node = ptr->next_node_ptr;

	uint64_t counter = ptr->counter_val;

	if (next_node == NULL || next_node->next == NULL ||
	    check_irq_node(next_node) || check_irq_node(next_node->next) ||
	    counter >= komb_batch_size || need_resched()) {
		incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = curr_node->cpuid;

	} else {
		ptr->curr_cs_cpu = next_node->cpuid;
		ptr->prev_cs_cpu = curr_node->cpuid;
		incoming_rsp_ptr = &(next_node->rsp);
		ptr->counter_val = counter + 1;
		print_debug("Jumping to the next waiter: %d\n",
			    next_node->cpuid);
	}

	outgoing_rsp_ptr = &(curr_node->rsp);

	KOMB_BUG_ON(incoming_rsp_ptr == NULL);
	KOMB_BUG_ON(outgoing_rsp_ptr == NULL);
	KOMB_BUG_ON(incoming_rsp_ptr == 0xdeadbeef);
	KOMB_BUG_ON(outgoing_rsp_ptr == 0xdeadbeef);

	curr_node->irqs_disabled = irqs_disabled();
	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	ptr = this_cpu_ptr(&local_shadow_stack);
	if (ptr->irqs_disabled) {
		ptr->irqs_disabled = false;
		local_irq_disable();
	}
	return;
}
EXPORT_SYMBOL_GPL(komb_spin_unlock);

__always_inline int komb_spin_trylock(struct qspinlock *lock)
{
	u32 val;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		return true;

	return false;
}

void komb_spin_lock_nested(struct qspinlock *lock, int level)
{
	komb_spin_lock(lock);
}
EXPORT_SYMBOL_GPL(komb_spin_lock_nested);

void komb_assert_spin_locked(struct qspinlock *lock)
{
	BUG_ON(!komb_spin_is_locked(lock));
}

__always_inline int komb_spin_is_locked(struct qspinlock *lock)
{
	return atomic_read(&lock->val);
}
EXPORT_SYMBOL(komb_spin_is_locked);

__always_inline int komb_spin_is_contended(struct qspinlock *lock)
{
	return atomic_read(&lock->val) & ~_Q_LOCKED_MASK;
}

__always_inline int komb_spin_value_unlocked(struct qspinlock lock)
{
	return !atomic_read(&lock.val);
}

struct task_struct *komb_get_current(spinlock_t *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	int j, my_idx;

	j = 0;
	my_idx = -1;

	for (j = 0; j < 8; j++) {
		if (ptr->lock_addr[j] == lock) {
			KOMB_BUG_ON(ptr->curr_cs_cpu < 0);
			return per_cpu_ptr(&komb_nodes[0], ptr->curr_cs_cpu)
				->task_struct_ptr;
		}
	}

	return current;
}

void komb_set_current_state(spinlock_t *lock, unsigned int state)
{
	smp_store_mb(komb_get_current(lock)->__state, state);
}
