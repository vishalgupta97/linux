// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <asm-generic/qspinlock.h>
#include <linux/sched.h>
#include <linux/komb_delegation.h>
#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/combiner.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#define DEBUG_KOMB 1
#define DSM_DEBUG 0

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#if DSM_DEBUG
#define print_debug(fmt, ...)                                                \
	({                                                                   \
		printk(KERN_EMERG "[%d] {%p} (%s) " fmt, smp_processor_id(), \
		       lock, __func__, ##__VA_ARGS__);                       \
	})
#else
#define print_debug(fmt, ...)
#endif

#define SIZE_OF_SHADOW_STACK 8192L
#define IRQ_NUMA_NODE 255

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

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr) \
	({                                                     \
		typeof(ptr) __PTR = (ptr);                     \
		__unqual_scalar_typeof(*ptr) VAL;              \
		for (;;) {                                     \
			VAL = READ_ONCE(*__PTR);               \
			if (cond_expr)                         \
				break;                         \
			cpu_relax();                           \
			if (need_resched()) {                  \
				preempt_enable();              \
				cond_resched();                \
				preempt_disable();             \
			}                                      \
		}                                              \
		(typeof(*ptr))VAL;                             \
	})

struct shadow_stack {
	void *lock_addr[8];
	void *ptr;
	uint32_t curr_cs_cpu;
	uint32_t prev_cs_cpu;
	uint64_t counter_val;
	struct kd_node *next_node_ptr;
	void *local_shadow_stack_ptr;
	struct kd_node *local_queue_head;
	struct kd_node *local_queue_tail;

	bool is_local_queue_tail_last;

	int irqs_disabled;

	char dummy[128];
};

static struct task_struct **dthreads;
static int num_delegation_threads = 1;
static int num_cores_per_socket = 1;

#ifdef KOMB_STATS
DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, waiter_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, ooo_combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, ooo_waiter_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, qspinlock_fallback);
#endif

static DEFINE_PER_CPU_ALIGNED(struct kd_node, *lock_rq_tail);
static DEFINE_PER_CPU_SHARED_ALIGNED(struct kd_node, kd_nodes[MAX_NODES]);
static DEFINE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

extern enum system_states system_state;

#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)
#define _Q_LOCKED_COMBINER_VAL 7
#define _Q_UNLOCKED_OOO_VAL 15 //Unlocked a lock out-of-order
#define _Q_LOCKED_IRQ_VAL 31 // Lock Stealing by IRQ

static inline __pure u32 encode_tail(int cpu, int idx)
{
	u32 tail;

	tail = (cpu + 1) << _Q_TAIL_CPU_OFFSET;
	tail |= idx << _Q_TAIL_IDX_OFFSET; /* assume < 4 */

	return tail;
}

static inline __pure struct kd_node *decode_tail(u32 tail)
{
	int cpu = (tail >> _Q_TAIL_CPU_OFFSET) - 1;
	int idx = (tail & _Q_TAIL_IDX_MASK) >> _Q_TAIL_IDX_OFFSET;

	//KOMB_BUG_ON(idx > 1); //TODO: Fix for nested case

	return per_cpu_ptr(&kd_nodes[idx], cpu);
}

static inline __pure u32 get_cpu_from_tail(u32 tail)
{
	return ((tail >> _Q_TAIL_CPU_OFFSET) - 1);
}

static __always_inline void clear_locked_set_completed(struct kd_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	return (u32)xchg(&lock->tail, tail >> _Q_TAIL_OFFSET) << _Q_TAIL_OFFSET;
}

static __always_inline u32 cmpxchg_tail(struct qspinlock *lock, u32 tail,
					u32 new_tail)
{
	return (u32)cmpxchg(&lock->tail, tail >> _Q_TAIL_OFFSET,
			    new_tail >> _Q_TAIL_OFFSET)
	       << _Q_TAIL_OFFSET;
}

__always_inline static void add_to_local_queue(struct qspinlock *lock,
					       struct kd_node *node)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (ptr->local_queue_head == NULL) {
		print_debug("Adding first node %d to local queue\n",
			    node->cpuid);
		ptr->local_queue_head = node;
		ptr->local_queue_tail = node;
	} else {
		print_debug(
			"Adding node %d to local queue. Head: %d tail: %d\n",
			node->cpuid, ptr->local_queue_head->cpuid,
			ptr->local_queue_tail->cpuid);
		ptr->local_queue_tail->next = node;
		ptr->local_queue_tail = node;
	}

	ptr->is_local_queue_tail_last = true;
}

static __always_inline struct kd_node *get_next_node(struct kd_node *my_node)
{
	struct kd_node *curr_node, *next_node;
	int i;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->socket_id == IRQ_NUMA_NODE)
			goto next_node_null;

		if (next_node->socket_id == -1) {
			curr_node->next = NULL;
			curr_node = next_node;
			next_node = curr_node->next;
			continue;
		}

		if (next_node->socket_id == numa_node_id()) {
			void *rsp_ptr = (void *)(per_cpu_ptr(&kd_nodes[0],
							     next_node->cpuid)
							 ->rsp);
			prefetchw(rsp_ptr);
			for (i = 1; i < NUM_PREFETCH_LINES; i++)
				prefetchw(rsp_ptr + (64 * i));

			prefetch(next_node->next);
			return next_node;
		}

		curr_node->next = NULL;
		add_to_local_queue(my_node->lock, next_node);
		curr_node = next_node;
		next_node = curr_node->next;
	}

next_node_null:
	return NULL;
}

#pragma GCC push_options
#pragma GCC optimize("O3")

__attribute__((noipa)) noinline notrace static void
execute_cs(struct kd_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	KOMB_BUG_ON(curr_node->cpuid == smp_processor_id());
	KOMB_BUG_ON((ptr->ptr - (ptr->local_shadow_stack_ptr)) >
		    SIZE_OF_SHADOW_STACK);

	ptr->is_local_queue_tail_last = false;

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &(ptr->local_shadow_stack_ptr);

	KOMB_BUG_ON(irqs_disabled());
	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == 0);
	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == 0);
	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == 0xdeadbeef);
	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == 0xdeadbeef);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	KOMB_BUG_ON((ptr->ptr - (ptr->local_shadow_stack_ptr)) >
		    SIZE_OF_SHADOW_STACK);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static struct kd_node *
run_combiner(struct kd_node *curr_node)
{
	struct shadow_stack *ptr;
	struct qspinlock *lock;

	KOMB_BUG_ON(curr_node == NULL);
	KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);
	ptr = this_cpu_ptr(&local_shadow_stack);
	lock = curr_node->lock;

	ptr->counter_val = 0;
	ptr->prev_cs_cpu = -1;
	ptr->next_node_ptr = NULL;
	ptr->curr_cs_cpu = curr_node->cpuid;

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	execute_cs(curr_node);

	print_debug(
		"Combiner got the control back: %d counter: %lld last_waiter: %d\n",
		smp_processor_id(), ptr->counter_val, ptr->curr_cs_cpu);

#if KOMB_STATS
	this_cpu_add(waiter_combined, ptr->counter_val);
	this_cpu_inc(combiner_count);
#endif

	KOMB_BUG_ON(ptr->prev_cs_cpu == -1 || ptr->prev_cs_cpu == 0);

	ptr->curr_cs_cpu = -1;
	return per_cpu_ptr(&kd_nodes[0], ptr->prev_cs_cpu);
}
#pragma GCC pop_options

static inline __pure u32 select_delegation_cpu(struct qspinlock *lock)
{
	return (num_cores_per_socket * numa_node_id());
	// 				+ ((u64)lock % (num_delegation_threads / num_online_nodes())));
}

__attribute__((noipa)) noinline notrace static int
__komb_spin_lock_slowpath(struct qspinlock *lock)
{
	struct kd_node *curr_node;
	register struct kd_node *prev_node;
	struct kd_node **rq_tail;
	u32 tail, old_tail, i, val, new_val, idx;

	curr_node = this_cpu_ptr(&kd_nodes[0]);
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
	curr_node->pos = 0;

	print_debug("my_tail: %x\n", tail);

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		curr_node->pos = prev_node->pos + 1;
		WRITE_ONCE(prev_node->next, curr_node);
		smp_mb();
		print_debug("prev_node: %d my_pos: %d\n", prev_node->cpuid,
			    curr_node->pos);
	} else {
head_of_queue:
		print_debug("Head of queue\n");
		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			KOMB_BUG_ON(lock->locked != 0);

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) ==
			    val)
				break;
		}

		print_debug("Got the lock\n");

		rq_tail =
			per_cpu_ptr(&lock_rq_tail, select_delegation_cpu(lock));
		if (READ_ONCE(*rq_tail) == 0xdeadbeef ||
		    !task_is_running(dthreads[numa_node_id()]) ||
		    cmpxchg(rq_tail, NULL, curr_node) != NULL) {
			// Fallback to qspinlock
			this_cpu_inc(qspinlock_fallback);
			print_debug("Delegation %d running something else\n",
				    select_delegation_cpu(lock));
			curr_node->locked = false;
			curr_node->completed = true;

			val = atomic_read(&lock->val);

			if (((val & _Q_TAIL_MASK) == tail) &&
			    atomic_try_cmpxchg_relaxed(&lock->val, &val,
						       _Q_LOCKED_VAL)) {
				print_debug(
					"IRQ only one in the queue unlocked\n");
				goto continue_with_cs_execution;
			} else {
				print_debug("Someone else joined the queue\n");
			}

			smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
			KOMB_BUG_ON(curr_node->next == NULL);
			print_debug("Next node now head of queue: %d\n",
				    curr_node->next->cpuid);
			WRITE_ONCE(curr_node->next->locked, false);

		} else {
			KOMB_BUG_ON(!task_is_running(dthreads[numa_node_id()]));
			print_debug("Added to the delegation thread: %d\n",
				    select_delegation_cpu(lock));
		}
	}

	smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (!curr_node->completed) {
		print_debug("Head of queue but not completed\n");
		curr_node->locked = true;
		curr_node->completed = false;
		goto head_of_queue;
	}

continue_with_cs_execution:
	if (curr_node->irqs_disabled) {
		ptr->irqs_disabled = curr_node->irqs_disabled;
	}
	curr_node->count--;
	KOMB_BUG_ON((char *)curr_node->rsp == 0);
	for (i = 0; i < NUM_PREFETCH_LINES; i++)
		prefetchw(((char *)curr_node->rsp) + (64 * i));

	return 0;
}

int komb_thread(void *args)
{
	struct qspinlock *lock;
	struct shadow_stack *ptr;
	struct kd_node *prev_node, *next_node;
	struct kd_node **rq_tail;
	int j, prev_preempt_count;

	ptr = this_cpu_ptr(&local_shadow_stack);
	rq_tail = this_cpu_ptr(&lock_rq_tail);
	lock = NULL;

	while (true) {
		if (READ_ONCE(*rq_tail) != 0xdeadbeef)
			break;
		cpu_relax();
		cond_resched();
	}

	preempt_disable();

	while (true) {
		print_debug("komb thread waiting for lock\n");
		smp_cond_load_relaxed_sched_delegation(rq_tail, (VAL));

		next_node = *rq_tail;
		KOMB_BUG_ON(next_node == NULL);
		KOMB_BUG_ON(next_node->cpuid == smp_processor_id());
		lock = next_node->lock;
		KOMB_BUG_ON(lock->locked ==
			    0); //Lock should already be acquired.
		WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
		print_debug("Running combiner with node from: %d\n",
			    next_node->cpuid);

		j = 0;
		for (j = 0; j < 8; j++)
			if (ptr->lock_addr[j] == NULL)
				break;

		KOMB_BUG_ON(j !=
			    0); //TODO: Update this condition nested delegation

		ptr->lock_addr[j] = lock;

		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
		ptr->is_local_queue_tail_last = false;

		KOMB_BUG_ON(!in_task());
		KOMB_BUG_ON(irqs_disabled());

		prev_preempt_count = preempt_count();

		prev_node = run_combiner(next_node);

		KOMB_BUG_ON(prev_preempt_count != preempt_count());

		KOMB_BUG_ON(ptr->lock_addr[j] != lock);
		ptr->lock_addr[j] = NULL;

		print_debug("Got control back prev node: %d\n",
			    prev_node->cpuid);
		KOMB_BUG_ON(prev_node == NULL);
		next_node = NULL;
		WRITE_ONCE(*rq_tail, NULL); //Combining done

		//lock = prev_node->lock; //TODO: Check why this is needed.
		if (READ_ONCE(prev_node->next) == NULL) {
			if (ptr->local_queue_head != NULL) {
				if (!ptr->is_local_queue_tail_last) {
					ptr->local_queue_tail->next = NULL;
					if (cmpxchg_tail(lock, prev_node->tail,
							 ptr->local_queue_tail
								 ->tail) !=
					    prev_node->tail) {
						print_debug(
							"local_queue_tail exists Someone else joined the queue: %d\n",
							prev_node->cpuid);
						smp_cond_load_relaxed_sched(
							&prev_node->next,
							(VAL));
						print_debug(
							"local_queue_tail exists Got the node: %d\n",
							READ_ONCE(
								prev_node->next
									->cpuid));

						ptr->local_queue_tail->next =
							prev_node->next;
					}
				}
				next_node = ptr->local_queue_head;
				print_debug(
					"prev_node next null, local_queue_head: %d local_queue_tail: %d\n",
					ptr->local_queue_head->cpuid,
					ptr->local_queue_tail->cpuid);
			} else {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				if (cmpxchg_tail(lock, prev_node->tail, 0) !=
				    prev_node->tail) {
					print_debug(
						"local_queue_tail exists Someone else joined the queue: %d\n",
						prev_node->cpuid);
					smp_cond_load_relaxed_sched(
						&prev_node->next, (VAL));
					print_debug(
						"local_queue_tail exists Got the node: %d\n",
						READ_ONCE(
							prev_node->next->cpuid));
					next_node = prev_node->next;
				}
			}
		} else {
			if (ptr->local_queue_head != NULL) {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				ptr->local_queue_tail->next = prev_node->next;
				next_node = ptr->local_queue_head;
				print_debug(
					"prev_node next %d, local_queue_head: %d local_queue_tail: %d\n",
					prev_node->next->cpuid,
					ptr->local_queue_head->cpuid,
					ptr->local_queue_tail->cpuid);
			} else {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				next_node = prev_node->next;
				print_debug(
					"prev_node next %d, local_queue_head NULL\n",
					prev_node->next->cpuid);
			}
		}

		if (READ_ONCE(next_node) != NULL) {
			next_node->locked = false;
		}

		clear_locked_set_completed(prev_node);

		KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
		print_debug("Releasing the lock from combiner\n");
		WRITE_ONCE(lock->locked, 0);
	}

	BUG_ON(true);
	preempt_enable();

	return 0;
}

void kd_init(void)
{
	int i, j;
	struct kd_node *komb_node;

	for_each_possible_cpu(i) {
		void *stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		struct shadow_stack *ptr = per_cpu_ptr(&local_shadow_stack, i);

		KOMB_BUG_ON(stack_ptr == NULL);

		ptr->ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		for (j = 0; j < 8; j++)
			ptr->lock_addr[j] = NULL;
		ptr->local_shadow_stack_ptr =
			stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = -1;
		ptr->counter_val = 0;
		ptr->next_node_ptr = NULL;
		ptr->irqs_disabled = false;
		*per_cpu_ptr(&lock_rq_tail, i) = 0xdeadbeef;
	}

	komb_node = per_cpu_ptr(&kd_nodes[0], 0);
	komb_node->next = NULL;
	komb_node->rsp = (void *)0xdeadbeef;
	komb_node->locked_completed = 0;
	komb_node->pos = 0;
	komb_node->socket_id = -1;

	num_delegation_threads = num_online_nodes();
	num_cores_per_socket = num_online_cpus() / num_online_nodes();

	dthreads =
		vzalloc(num_delegation_threads * sizeof(struct task_struct *));
	for (i = 0; i < num_delegation_threads; i++) {
		dthreads[i] = kthread_create(komb_thread, NULL, "komb_thread");
		kthread_bind(dthreads[i], i * num_cores_per_socket);
		if (dthreads[i])
			wake_up_process(dthreads[i]);
		else
			printk(KERN_ALERT
			       "failed to create komb delegation threads\n");
	}

	printk(KERN_ALERT "starting delegation threads\n");
}

void kd_free(void)
{
}

void kd_spin_lock_init(struct qspinlock *lock)
{
	atomic_set(&lock->val, 0);
}

__attribute__((noipa)) noinline notrace static void *get_shadow_stack_ptr(void)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	return &(ptr->local_shadow_stack_ptr);
}

__attribute__((noipa)) noinline notrace static struct kd_node *get_kd_node(void)
{
	return this_cpu_ptr(&kd_nodes[0]);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static __attribute__((noipa)) noinline notrace void
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
		     : "i"(get_kd_node), "i"(offsetof(struct kd_node, rsp))
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
			     : "i"(get_kd_node),
			       "i"(offsetof(struct kd_node, rsp))
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

static __always_inline u32 komb_fetch_set_pending_acquire(struct qspinlock *lock)
{
	return atomic_fetch_or_acquire(_Q_PENDING_VAL, &lock->val);
}

static __always_inline void clear_pending(struct qspinlock *lock)
{
	atomic_andnot(_Q_PENDING_VAL, &lock->val);
}

static __always_inline void set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked, _Q_LOCKED_VAL);
}

__always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	KOMB_BUG_ON(lock->locked != 0);
	WRITE_ONCE(lock->locked_pending, _Q_LOCKED_VAL);
}

__attribute__((noipa)) noinline notrace void
kd_spin_lock(struct qspinlock *lock)
{
	struct kd_node *curr_node = NULL;
	struct shadow_stack *ptr;
	uint32_t curr_cpuid, prev_cpuid;
	struct kd_node *prev_node, *next_node;
	u32 val, cnt;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		return;

	KOMB_BUG_ON(!in_atomic());

queue:
	if (unlikely(system_state != SYSTEM_RUNNING)) {
		while (!kd_spin_trylock(lock))
			cpu_relax();
		return;
	}

	curr_node = this_cpu_ptr(&kd_nodes[0]);
	KOMB_BUG_ON(curr_node == NULL);

	if (curr_node->count > 0 || !in_task() || irqs_disabled() ||
	    current->migration_disabled ||
	    ((smp_processor_id() % num_cores_per_socket) == 0) ||
	    !task_is_running(dthreads[numa_node_id()])) {
		struct kd_node *prev_node, *next_node;
		u32 tail, idx;

		idx = curr_node->count++;
		if (unlikely(idx >= MAX_NODES)) {
			while (!kd_spin_trylock(lock))
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
		curr_node = this_cpu_ptr(&kd_nodes[0]);
		curr_node->count--;
		return;
	} else {
		KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) == 0);

		print_debug("Going on slowpath\n");

		komb_spin_lock_slowpath(lock);

		ptr = this_cpu_ptr(&local_shadow_stack);
		curr_cpuid = ptr->curr_cs_cpu;
		if (curr_cpuid != -1) {
			curr_node = per_cpu_ptr(&kd_nodes[0], curr_cpuid);
			next_node = get_next_node(curr_node);
			ptr->next_node_ptr = next_node;

			prev_cpuid = ptr->prev_cs_cpu;
			if (prev_cpuid != -1) {
				prev_node =
					per_cpu_ptr(&kd_nodes[0], prev_cpuid);
				print_debug("Waking up prev waiter: %d\n",
					    prev_cpuid);
				clear_locked_set_completed(prev_node);
				ptr->prev_cs_cpu = -1;
			}
		}
	}
}
EXPORT_SYMBOL_GPL(kd_spin_lock);

inline bool kd_spin_trylock(struct qspinlock *lock)
{
	return (atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL) == 0);
}

__attribute__((noipa)) noinline notrace void
kd_spin_unlock(struct qspinlock *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
	int from_cpuid = ptr->curr_cs_cpu;
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct kd_node *curr_node;
	struct kd_node *next_node;
	uint64_t counter;
	int j, max_idx, my_idx;
	void *temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

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
		    lock->locked == _Q_LOCKED_IRQ_VAL) {
			lock->locked = false;
		} else {
			KOMB_BUG_ON(true);
		}
		return;
	}

	KOMB_BUG_ON(my_idx < max_idx);

#if DEBUG_KOMB
	//Delegation thread should be on CPU 0 on each socket
	if ((smp_processor_id() % num_cores_per_socket) != 0) {
		printk(KERN_ALERT "curr_cpu: %d\n", smp_processor_id());
		BUG_ON(true);
	}
	BUG_ON(from_cpuid == -1);
#endif

	curr_node = per_cpu_ptr(&kd_nodes[0], from_cpuid);

	next_node = ptr->next_node_ptr;

	counter = ptr->counter_val;

	if (next_node == NULL || need_resched()) {
		incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = curr_node->cpuid;

	} else {
		ptr->is_local_queue_tail_last = false;
		ptr->curr_cs_cpu = next_node->cpuid;
		ptr->prev_cs_cpu = curr_node->cpuid;
		incoming_rsp_ptr = &(next_node->rsp);
		ptr->counter_val = counter + 1;
		print_debug("Jumping to the next waiter: %d\n",
			    next_node->cpuid);
	}

	outgoing_rsp_ptr = &(curr_node->rsp);

	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == 0);
	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == 0);
	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == 0xdeadbeef);
	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == 0xdeadbeef);

	curr_node->irqs_disabled = irqs_disabled();

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	ptr = this_cpu_ptr(&local_shadow_stack);
	if (ptr->irqs_disabled) {
		ptr->irqs_disabled = false;
		local_irq_disable();
	}
	return;
}
EXPORT_SYMBOL_GPL(kd_spin_unlock);

__always_inline int kd_spin_is_locked(struct qspinlock *lock)
{
	return atomic_read(&lock->val);
}
EXPORT_SYMBOL(kd_spin_is_locked);

__always_inline int kd_spin_is_contended(struct qspinlock *lock)
{
	return atomic_read(&lock->val) & ~_Q_LOCKED_MASK;
}

__always_inline int kd_spin_value_unlocked(struct qspinlock lock)
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
			return per_cpu_ptr(&kd_nodes[0], ptr->curr_cs_cpu)
				->task_struct_ptr;
		}
	}

	return current;
}

void komb_set_current_state(spinlock_t *lock, unsigned int state)
{
	smp_store_mb(komb_get_current(lock)->__state, state);
}

SYSCALL_DEFINE0(komb_start_delegation)
{
	printk(KERN_ALERT "======== KOMB starting delegation ========\n");
	int i;
	for_each_online_cpu(i) {
		*per_cpu_ptr(&lock_rq_tail, i) = 0;
	}
	return 0;
}