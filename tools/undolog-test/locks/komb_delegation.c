// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

/*
 * TODO: (Performance optimiztion)
 * Fix the index part when same is acquired multiple times.
 * Currently nested locking becomes TTAS lock.
 * irqs_disabled() will hurt performance when running in VM.
 */
#if KERNEL_SYNCSTRESS
#include "spinlock/komb_delegation.h"
#include "timing_stats.h"
#else
#include <asm-generic/qspinlock.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#define LOCK_START_TIMING_PER_CPU(combiner_loop)
#define LOCK_END_TIMING_PER_CPU(combiner_loop)
#endif
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

#if DSM_DEBUG
#define print_debug(fmt, ...)                                                                      \
	({                                                                                               \
		printk(KERN_EMERG "[%d] komb (%s) lock(%px): " fmt, smp_processor_id(), __func__, lock,        \
					 ##__VA_ARGS__);                                                                         \
	})
#define print_debug_without_lock(fmt, ...)                                                         \
	({ printk(KERN_EMERG "[%d] komb (%s): " fmt, smp_processor_id(), __func__, ##__VA_ARGS__); })
#else
#define print_debug(fmt, ...)
#define print_debug_without_lock(fmt, ...)
#endif

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#if KERNEL_SYNCSTRESS
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
	({                                                                                               \
		typeof(ptr) __PTR = (ptr);                                                                     \
		__unqual_scalar_typeof(*ptr) VAL;                                                              \
		for (;;) {                                                                                     \
			VAL = READ_ONCE(*__PTR);                                                                     \
			if (cond_expr)                                                                               \
				break;                                                                                     \
			cpu_relax();                                                                                 \
			if (need_resched()) {                                                                        \
				cond_resched();                                                                            \
			}                                                                                            \
		}                                                                                              \
		(typeof(*ptr))VAL;                                                                             \
	})

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr)                                     \
	({                                                                                               \
		typeof(ptr) __PTR = (ptr);                                                                     \
		__unqual_scalar_typeof(*ptr) VAL;                                                              \
		for (;;) {                                                                                     \
			VAL = READ_ONCE(*__PTR);                                                                     \
			if (cond_expr || kthread_should_stop())                                                      \
				break;                                                                                     \
			cpu_relax();                                                                                 \
			if (need_resched()) {                                                                        \
				cond_resched();                                                                            \
			}                                                                                            \
		}                                                                                              \
		(typeof(*ptr))VAL;                                                                             \
	})

#else
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
	({                                                                                               \
		typeof(ptr) __PTR = (ptr);                                                                     \
		__unqual_scalar_typeof(*ptr) VAL;                                                              \
		for (;;) {                                                                                     \
			VAL = READ_ONCE(*__PTR);                                                                     \
			if (cond_expr)                                                                               \
				break;                                                                                     \
			cpu_relax();                                                                                 \
		}                                                                                              \
		(typeof(*ptr))VAL;                                                                             \
	})

//if (need_resched()) {
//	schedule_preempt_disabled();
//}
#endif

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                                                \
	({                                                                                               \
		__unqual_scalar_typeof(*ptr) _val;                                                             \
		_val = smp_cond_load_relaxed_sched(ptr, cond_expr);                                            \
		smp_acquire__after_ctrl_dep();                                                                 \
		(typeof(*ptr))_val;                                                                            \
	})
#endif

#define atomic_cond_read_acquire_sched(v, c) smp_cond_load_acquire_sched(&(v)->counter, (c))

struct shadow_stack {
	/* 
	 * lock_addr represents the lock addresses acquired by the current CPU.
	 * when acquiring multiple locks on the same CPU (level locking). Each
	 * lock address is stored in the lock_addr array. This helps when locks
	 * are released out of order [Example : Acquire(A), Acquire(B),
	 * Release(A), Release(B)]
	 *
	 * TODO (Space Optimization): Fix this to be at the base of the shadow 
	 * stack.
	 */
	void *lock_addr[8];

	/*
	 * ptr points to the base of the shadow stack.
	 */
	void *ptr;
	/*
 * Used by the combiner to identify for which CPU is the critical section 
 * currently executing.
 */
	uint32_t curr_cs_cpu;
	uint32_t prev_cs_cpu;
	uint64_t counter_val;
	struct komb_delegation_node *next_node_ptr;
	void *local_shadow_stack_ptr;
#if NUMA_AWARE
	/*
 * Create a queue which is used by the combiner to store nodes which belong to 
 * different socket.
 */

	struct komb_delegation_node *local_queue_head;
	struct komb_delegation_node *local_queue_tail;

	bool is_local_queue_tail_last;
#endif


	int irqs_disabled;

	char dummy[128];
};

static struct task_struct **dthreads;
static int num_delegation_threads;
static int num_delegation_threads_per_socket;

#if LOCK_MEASURE_TIME
//static DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_loop);
//static DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_loop_lockfn);
//static DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_loop_unlockfn);
/*static DEFINE_PER_CPU_ALIGNED(uint64_t, lock_stack_switch);
static DEFINE_PER_CPU_ALIGNED(uint64_t, unlock_stack_switch);*/
#endif

static DEFINE_PER_CPU_ALIGNED(struct komb_delegation_node, *lock_rq_tail);

/*
 * Used by all threads to add itself to the queue on the slowpath.
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct komb_delegation_node, komb_delegation_nodes[MAX_NODES]);

/*
 * 8KB shadow stack used by all the threads. For waiter threads, switch to 
 * shadow stack, because of IRQs. For combiner thread, switch to shadow stack 
 * to handle nesting
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)
#define _Q_LOCKED_COMBINER_VAL 7

/*
 * We must be able to distinguish between no-tail and the tail at 0:0,
 * therefore increment the cpu number by one.
 */
static inline __pure u32 encode_tail(int cpu, int idx)
{
	u32 tail;

	tail = (cpu + 1) << _Q_TAIL_CPU_OFFSET;
	tail |= idx << _Q_TAIL_IDX_OFFSET; /* assume < 4 */

	return tail;
}

static inline __pure struct komb_delegation_node *decode_tail(u32 tail)
{
	int cpu = (tail >> _Q_TAIL_CPU_OFFSET) - 1;
	int idx = (tail & _Q_TAIL_IDX_MASK) >> _Q_TAIL_IDX_OFFSET;

	KOMB_BUG_ON(idx > 1); //TODO: Fix for the kernel

	return per_cpu_ptr(&komb_delegation_nodes[idx], cpu);
}

static inline __pure u32 get_cpu_from_tail(u32 tail)
{
	return ((tail >> _Q_TAIL_CPU_OFFSET) - 1);
}

static __always_inline void clear_locked_set_completed(struct komb_delegation_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	return ((u32)xchg(&lock->tail, tail >> _Q_TAIL_OFFSET)) << _Q_TAIL_OFFSET;
}

static __always_inline u32 cmpxchg_tail(struct qspinlock *lock, u32 tail, u32 new_tail)
{
	return ((u32)cmpxchg(&lock->tail, tail >> _Q_TAIL_OFFSET, new_tail >> _Q_TAIL_OFFSET))
				 << _Q_TAIL_OFFSET;
}

#if NUMA_AWARE
__always_inline static void add_to_local_queue(struct komb_delegation_node *node)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (ptr->local_queue_head == NULL) {
		print_debug_without_lock("Adding first node %d to local queue\n", node->cpuid);
		ptr->local_queue_head = node;
		ptr->local_queue_tail = node;
	} else {
		print_debug_without_lock("Adding node %d to local queue. Head: %d tail: %d\n", node->cpuid,
														 ptr->local_queue_head->cpuid, ptr->local_queue_tail->cpuid);
		ptr->local_queue_tail->next = node;
		ptr->local_queue_tail = node;
	}

	ptr->is_local_queue_tail_last = true;
}
#endif

static __always_inline struct komb_delegation_node *
get_next_node(struct komb_delegation_node *my_node)
{
#if NUMA_AWARE
	struct komb_delegation_node *curr_node, *next_node;
#if PREFETCHING
	int i;
#endif

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
#if PREFETCHING
			void *rsp_ptr = (void *)(per_cpu_ptr(&komb_delegation_nodes[0], next_node->cpuid)->rsp);
			prefetchw(rsp_ptr);
			for (i = 1; i < NUM_PREFETCH_LINES; i++)
				prefetchw(rsp_ptr + (64 * i));

			prefetch(next_node->next);
#endif
			return next_node;
		}

		curr_node->next = NULL;
		add_to_local_queue(next_node);
		curr_node = next_node;
		next_node = curr_node->next;
	}

next_node_null:
	return NULL;
#else
	return my_node->next;
#endif
}

#pragma GCC push_options
#pragma GCC optimize("O3")

__attribute__((noipa)) noinline notrace static void
execute_cs(struct komb_delegation_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	KOMB_BUG_ON(curr_node->cpuid == smp_processor_id());
	KOMB_BUG_ON((ptr->ptr - (ptr->local_shadow_stack_ptr)) > SIZE_OF_SHADOW_STACK);

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &(ptr->local_shadow_stack_ptr);

	/*
	 * Make the actual switch, the pushed return address is after this
	 * function call, which we will resume execution at using the switch
	 * in unlock.
	 */
	//KOMB_BUG_ON(*(char *)incoming_rsp_ptr == 0);
	//KOMB_BUG_ON(*(char *)outgoing_rsp_ptr == 0);
	//KOMB_BUG_ON(incoming_rsp_ptr == (void *)0xdeadbeef);
	//KOMB_BUG_ON(outgoing_rsp_ptr == (void *)0xdeadbeef);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	KOMB_BUG_ON((ptr->ptr - (ptr->local_shadow_stack_ptr)) > SIZE_OF_SHADOW_STACK);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static struct komb_delegation_node *
run_combiner(struct qspinlock *lock, struct komb_delegation_node *curr_node)
{
	struct shadow_stack *ptr;
#if WWJUMP == 0
	struct komb_delegation_node *next_node, *prev_node;
	uint64_t counter;
#endif
	KOMB_BUG_ON(curr_node == NULL);
	KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);

	ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->counter_val = 0;
	ptr->prev_cs_cpu = -1;
	ptr->next_node_ptr = NULL;
	ptr->curr_cs_cpu = curr_node->cpuid;

#if LOCK_MEASURE_TIME
//	*this_cpu_ptr(&combiner_loop) = UINT64_MAX;
#endif

#if WWJUMP == 0
	prev_node = NULL;
	counter = 0;
	while(curr_node) {
#if LOCK_MEASURE_TIME
	LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop);
	LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop);
//	LOCK_END_TIMING_PER_CPU(combiner_loop);
//	LOCK_START_TIMING_PER_CPU(combiner_loop);
#endif

		counter++;
		
#if NUMA_AWARE
		ptr->is_local_queue_tail_last = false;
#endif
		next_node = get_next_node(curr_node);
		if(prev_node) {
			clear_locked_set_completed(prev_node);
			prev_node = NULL;
		}
		ptr->curr_cs_cpu = curr_node->cpuid;
		print_debug("Combiner jumping to waiter: %d\n", curr_node->cpuid);
		execute_cs(curr_node);

		if(next_node == NULL || next_node->socket_id == IRQ_NUMA_NODE || counter >= komb_batch_size ||
				need_resched())
			break;
	
		prev_node = curr_node;
		curr_node = next_node;
	}
	ptr->curr_cs_cpu = -1;
	ptr->prev_cs_cpu = curr_node->cpuid;
	ptr->counter_val = counter;
#else //WWJUMP

	//print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
	//	    curr_node->cpuid);

#if NUMA_AWARE
	ptr->is_local_queue_tail_last = false;
#endif
	execute_cs(curr_node);

	//print_debug(
	//	"Combiner got the control back: %d counter: %lld last_waiter: %d\n",
	//	smp_processor_id(), ptr->counter_val, ptr->curr_cs_cpu);

#endif //WWJUMP
#if KOMB_STATS
	this_cpu_add(waiter_combined, ptr->counter_val);
	this_cpu_inc(combiner_count);
#endif

	KOMB_BUG_ON(ptr->prev_cs_cpu == -1 || ptr->prev_cs_cpu == KOMB_CPU);
	ptr->curr_cs_cpu = -1;
	return per_cpu_ptr(&komb_delegation_nodes[0], ptr->prev_cs_cpu);
}
#pragma GCC pop_options

static inline __pure u32 select_delegation_cpu(struct qspinlock *lock)
{
#if NUMA_AWARE
	return ((num_cores_per_socket * numa_node_id()) +
					((u64)lock % num_delegation_threads_per_socket));
#else
	return 0;
#endif
}

__attribute__((noipa)) noinline notrace static int __komb_spin_lock_slowpath(struct qspinlock *lock)
{
	struct komb_delegation_node *curr_node;
	struct komb_delegation_node **rq_tail;
	register struct komb_delegation_node *prev_node;
	u32 tail, old_tail, val, new_val;
#if PREFETCHING
	u32 i;
#endif

	LOCK_END_TIMING_PER_CPU_DISABLE(lock_stack_switch);

	curr_node = this_cpu_ptr(&komb_delegation_nodes[0]);
	tail = encode_tail(smp_processor_id(), 0);

	/*
	 * Initialize curr_node
	 */
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

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		curr_node->pos = prev_node->pos + 1;
		WRITE_ONCE(prev_node->next, curr_node);
		smp_mb();
		print_debug("prev_node: %d my_pos: %d\n", prev_node->cpuid, curr_node->pos);
	} else {
	head_of_queue:
		print_debug("Head of queue\n");
		while (true) {
			val = atomic_cond_read_relaxed(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			KOMB_BUG_ON(lock->locked != 0);

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) == val)
				break;
		}

		print_debug("Got the lock\n");

		rq_tail = per_cpu_ptr(&lock_rq_tail, select_delegation_cpu(lock));
		if (cmpxchg(rq_tail, NULL, curr_node) != NULL) {
			// Fallback to qspinlock
			print_debug("Delegation %d running something else\n", select_delegation_cpu(lock));
			curr_node->locked = false;
			curr_node->completed = true;

			val = atomic_read(&lock->val);

			if (((val & _Q_TAIL_MASK) == tail) &&
					atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL)) {
				print_debug("IRQ only one in the queue unlocked\n");
				goto continue_with_cs_execution;
			} else {
				print_debug("Someone else joined the queue\n");
			}

			smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
			KOMB_BUG_ON(curr_node->next == NULL);
			print_debug("Next node now head of queue: %d\n", curr_node->next->cpuid);
			WRITE_ONCE(curr_node->next->locked, false);

		} else {
			print_debug("Added to the delegation thread: %d\n", select_delegation_cpu(lock));
		}
	}

	smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

	if (!curr_node->completed) {
		print_debug("Head of queue but not completed\n");
		curr_node->locked = true;
		curr_node->completed = false;
		goto head_of_queue;
	}

continue_with_cs_execution:
#if PREFETCHING
	for (i = 0; i < NUM_PREFETCH_LINES; i++)
		prefetchw(((char *)curr_node->rsp) + (64 * i));
#endif

	LOCK_START_TIMING_PER_CPU_DISABLE(unlock_stack_switch);
	return 0;
}

int komb_thread(void *args)
{
	struct qspinlock *lock;
	struct shadow_stack *ptr;
	struct komb_delegation_node *prev_node, *next_node;
	struct komb_delegation_node **rq_tail;
	int j;

	ptr = this_cpu_ptr(&local_shadow_stack);
	rq_tail = this_cpu_ptr(&lock_rq_tail);
	lock = NULL;

	while (!kthread_should_stop()) {
		smp_cond_load_relaxed_sched_delegation(rq_tail, (VAL));
		if (kthread_should_stop()) {
			KOMB_BUG_ON(*rq_tail != NULL);
			break;
		}

		next_node = *rq_tail;
		KOMB_BUG_ON(next_node == NULL);
		KOMB_BUG_ON(next_node->cpuid == smp_processor_id());
		lock = next_node->lock;
		KOMB_BUG_ON(lock->locked == 0); //Lock should already be acquired.
		WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
		print_debug("Running combiner with node from: %d\n", next_node->cpuid);

		j = 0;
		for (j = 0; j < 8; j++)
			if (ptr->lock_addr[j] == NULL)
				break;

		KOMB_BUG_ON(j != 0); //TODO: Update this condition nested delegation

		ptr->lock_addr[j] = lock;
#if NUMA_AWARE
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
		ptr->is_local_queue_tail_last = false;
#endif
		prev_node = run_combiner(lock, next_node);

		KOMB_BUG_ON(ptr->lock_addr[j] != lock);
		ptr->lock_addr[j] = NULL;

		print_debug("Got control back prev node: %d\n", prev_node->cpuid);
		KOMB_BUG_ON(prev_node == NULL);
		next_node = NULL;
		WRITE_ONCE(*rq_tail, NULL); //Combining done

#if NUMA_AWARE
		if (READ_ONCE(prev_node->next) == NULL) {
			if (ptr->local_queue_head != NULL) {
				if (!ptr->is_local_queue_tail_last) {
					ptr->local_queue_tail->next = NULL;
					if (cmpxchg_tail(lock, prev_node->tail, ptr->local_queue_tail->tail) != prev_node->tail) {
						print_debug("local_queue_tail exists Someone else joined the queue: %d\n",
												prev_node->cpuid);
						smp_cond_load_relaxed_sched(&prev_node->next, (VAL));
						print_debug("local_queue_tail exists Got the node: %d\n",
												READ_ONCE(prev_node->next->cpuid));

						ptr->local_queue_tail->next = prev_node->next;
					}
				}
				next_node = ptr->local_queue_head;
				print_debug("prev_node next null, local_queue_head: %d local_queue_tail: %d\n",
										ptr->local_queue_head->cpuid, ptr->local_queue_tail->cpuid);
			} else {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				print_debug("Setting lock tail to zero\n");
				if (cmpxchg_tail(lock, prev_node->tail, 0) != prev_node->tail) {
					print_debug("local_queue_tail exists Someone else joined the queue: %d\n",
											prev_node->cpuid);
					smp_cond_load_relaxed_sched(&prev_node->next, (VAL));
					print_debug("local_queue_tail exists Got the node: %d\n",
											READ_ONCE(prev_node->next->cpuid));
					next_node = prev_node->next;
				}
			}
		} else {
			if (ptr->local_queue_head != NULL) {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				ptr->local_queue_tail->next = prev_node->next;
				next_node = ptr->local_queue_head;
				print_debug("prev_node next %d, local_queue_head: %d local_queue_tail: %d\n",
										prev_node->next->cpuid, ptr->local_queue_head->cpuid,
										ptr->local_queue_tail->cpuid);
			} else {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				next_node = prev_node->next;
				print_debug("prev_node next %d, local_queue_head NULL\n", prev_node->next->cpuid);
			}
		}

#else //NUMA_AWARE

		if (READ_ONCE(prev_node->next) == NULL) {
				if (cmpxchg_tail(lock, prev_node->tail, 0) != prev_node->tail) {
					smp_cond_load_relaxed_sched(&prev_node->next, (VAL));
					next_node = prev_node->next;
				}
		} else {
				next_node = READ_ONCE(prev_node->next);
		}

#endif //NUMA_AWARE

		if (READ_ONCE(next_node) != NULL) {
			print_debug("Transferring lock to another socket %d cpuid %d\n", next_node->socket_id,
									next_node->cpuid);

			next_node->locked = false;
		} else {
			print_debug("My node next NULL\n");
		}


		clear_locked_set_completed(prev_node);

		//Release the lock
		KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
		print_debug("Releasing the lock from combiner\n");
		WRITE_ONCE(lock->locked, 0);

		if (need_resched())
			cond_resched();
	}

	cond_resched();

	__set_current_state(TASK_RUNNING);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);

	return 0;
}

/*
 * Public API
 */

void komb_delegation_init(int __num_delegation_threads)
{
	int i, j;
	struct komb_delegation_node *komb_node;

	for_each_possible_cpu (i) {
		void *stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		struct shadow_stack *ptr = per_cpu_ptr(&local_shadow_stack, i);

		KOMB_BUG_ON(stack_ptr == NULL);

		ptr->ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		for (j = 0; j < 8; j++)
			ptr->lock_addr[j] = NULL;
		ptr->local_shadow_stack_ptr = stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = -1;
		ptr->counter_val = 0;
		ptr->next_node_ptr = NULL;
		ptr->irqs_disabled = false;

		*per_cpu_ptr(&lock_rq_tail, i) = NULL;
#if LOCK_MEASURE_TIME
//		*per_cpu_ptr(&combiner_loop, i) = UINT64_MAX;
#endif
	}

#if LOCK_MEASURE_TIME
	*per_cpu_ptr(&do_timing, KOMB_CPU) = true;
#endif

	// Initialize komb node
	komb_node = per_cpu_ptr(&komb_delegation_nodes[0], 0);
	komb_node->next = NULL;
	komb_node->rsp = (void *)0xdeadbeef;
	komb_node->locked_completed = 0;
	komb_node->pos = 0;
	komb_node->socket_id = -1;

#if NUMA_AWARE
	num_delegation_threads = __num_delegation_threads;
	num_delegation_threads_per_socket = num_delegation_threads / online_sockets;
	KOMB_BUG_ON(num_delegation_threads != online_sockets);
#else
	num_delegation_threads = 1;
	num_delegation_threads_per_socket = 1;
#endif

	dthreads = vzalloc(num_delegation_threads * sizeof(struct task_struct *));
	for (i = 0; i < num_delegation_threads; i++) {
		dthreads[i] = kthread_create(komb_thread, NULL, "komb_thread");
		kthread_bind(dthreads[i], i * num_cores_per_socket);
		if (dthreads[i])
			wake_up_process(dthreads[i]);
		else
			printk(KERN_ALERT "failed to create komb delegation threads\n");
	}
}

void komb_delegation_free(void)
{
	int ret;
	uint32_t i;

	for (i = 0; i < num_delegation_threads; i++) {
		ret = kthread_stop(dthreads[i]);
		if (ret)
			printk(KERN_ALERT "komb delegate thread returned error %d\n", ret);
	}

	for_each_possible_cpu (i) {
		vfree(per_cpu_ptr(&local_shadow_stack, i)->ptr - SIZE_OF_SHADOW_STACK);
	}
}

void komb_delegation_spin_lock_init(struct qspinlock *lock)
{
	atomic_set(&lock->val, 0);
}

__attribute__((noipa)) noinline notrace static void *get_shadow_stack_ptr(void)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	return &(ptr->local_shadow_stack_ptr);
}

__attribute__((noipa)) noinline notrace static struct komb_delegation_node *
get_komb_delegation_node(void)
{
	return this_cpu_ptr(&komb_delegation_nodes[0]);
}

/* 
 * When the following function is called, the compiler in caller frame does
 * this sequence:
 *
 * push caller-saved-regs			(rdi, rsi, rdx, rcx, ...)
 * ...<prepare arguments>...
 * .- push eip+1				(eip + 1 == pop insn)
 * callq |
 * `- jmp komb_spin_lock
 * pop caller-saved-regs
 *
 * This means that the following function must in no way modify the the rsp
 * before it is written to the task_frame, otherwise the top of the stack
 * (rsp) would no longer point to the return address. To do this, emit a
 * call in assembly and pass the top of the stack as an argument so that
 * we can use the stack while preserving the semantics of the combiner.
 *
 * NOTE: One advantage of offloading the caller-saved register spill to the
 * compiler is that it only saves the caller-saved registers in use in the
 * caller frame. If we were to inline this whole logic in assembly, we wouldn't
 * know the registers in use and would have to save all possible caller-saved
 * registers.
 */

#pragma GCC push_options
#pragma GCC optimize("O3")
static __attribute__((noipa)) noinline notrace void komb_spin_lock_slowpath(struct qspinlock *lock)
{
	register int ret_val;
	/* 
	 * We must save the callee-saved registers now, since we will resume
	 * execution at the return address currently at top of stack. If we
	 * defer saving to when context switch happens later, the C function
	 * may have saved callee-saved registers and will restore them before
	 * returning.
	 *
	 * Due to unstructured control flow, do the saving now.
	 *
	 * NOTE: When we touch the stack inside this function, so a 16-byte
	 * stack's alignment will change to 8-byte alignment when callq pushes
	 * our return address, but Linux only assumes 8-byte stack alignment,
	 * so it isn't a problem for us.
	 */
#if ENABLE_IRQS_CHECK
	KOMB_BUG_ON(irqs_disabled());
#endif
	//local_irq_disable();
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
							 : "i"(get_komb_delegation_node), "i"(offsetof(struct komb_delegation_node, rsp))
							 : "memory");
	asm volatile("callq %P0\n"
							 "movq (%%rax), %%rsp\n"
							 :
							 : "i"(get_shadow_stack_ptr)
							 : "memory");
	//local_irq_enable();

	ret_val = __komb_spin_lock_slowpath(lock);

	//local_irq_disable();
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
								 : "i"(get_komb_delegation_node), "i"(offsetof(struct komb_delegation_node, rsp))
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
	//local_irq_enable();
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace void komb_delegation_spin_lock(struct qspinlock *lock)
{
	struct komb_delegation_node *curr_node = NULL;
	struct shadow_stack *ptr;
#if WWJUMP
	uint32_t curr_cpuid, prev_cpuid;
	struct komb_delegation_node *prev_node, *next_node;
#endif
	u32 val;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0) {
		print_debug("Got lock on fast path\n");
		return;
	}

	curr_node = this_cpu_ptr(&komb_delegation_nodes[0]);
	ptr = this_cpu_ptr(&local_shadow_stack);
	KOMB_BUG_ON(curr_node == NULL);
	if((smp_processor_id() % num_cores_per_socket) == 0) {
		int j = 0;
		for(j = 0; j < 8; j++)
			if(ptr->lock_addr[j] != NULL)
				break;
		print_debug("lock addr index: %d\n", j);
		KOMB_BUG_ON(true);
	}

#if LOCK_MEASURE_TIME
//		*this_cpu_ptr(&lock_stack_switch) = UINT64_MAX;
//		*this_cpu_ptr(&unlock_stack_switch) = UINT64_MAX;
#endif
	LOCK_START_TIMING_PER_CPU_DISABLE(lock_stack_switch);

	komb_spin_lock_slowpath(lock);

#if WWJUMP
	curr_cpuid = ptr->curr_cs_cpu;
	if (curr_cpuid != -1) {
		curr_node =
			per_cpu_ptr(&komb_delegation_nodes[0], curr_cpuid); // TODO: Check if this needs to be fixed

#if LOCK_MEASURE_TIME
		LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop_lockfn);
#endif
		next_node = get_next_node(curr_node);
#if LOCK_MEASURE_TIME
		LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop_lockfn);
#endif
		ptr->next_node_ptr = next_node;

		prev_cpuid = ptr->prev_cs_cpu;
		if (prev_cpuid != -1) {
			prev_node = per_cpu_ptr(&komb_delegation_nodes[0], prev_cpuid);
			print_debug("Waking up prev waiter: %d\n", prev_cpuid);
			clear_locked_set_completed(prev_node);
			ptr->prev_cs_cpu = -1;
		}
	}
#endif

}

void komb_delegation_spin_lock_nested(struct qspinlock *lock, int level)
{
	u32 val;
	struct komb_delegation_node *curr_node, *prev_node, *next_node;
	u32 tail, idx, old_tail, new_val;

	print_debug("Nested lock acquiring the lock\n");

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0) {
		print_debug("Nested lock got the lock\n");
		return;
	}

	curr_node = this_cpu_ptr(&komb_delegation_nodes[0]);
	KOMB_BUG_ON(curr_node == NULL);
	idx = 1;
	if (unlikely(idx >= MAX_NODES)) {
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

	curr_node->rsp = (void *)0xdeadbeef;

	smp_wmb();

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		WRITE_ONCE(prev_node->next, curr_node);

		print_debug("IRQ going to waiting for lock\n");
		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));
	}

	print_debug("IRQ spinning on the locked field\n");

	val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

	if (((val & _Q_TAIL_MASK) == tail) &&
			atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL)) {
		print_debug("IRQ only one in the queue unlocked\n");
		goto irq_release;
	}

	//set_locked(lock);

	while (true) {
		val = atomic_cond_read_relaxed(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

		KOMB_BUG_ON(lock->locked != 0);

		new_val = val >> _Q_LOCKED_BITS;
		new_val <<= _Q_LOCKED_BITS;
		new_val |= _Q_LOCKED_VAL;

		if (atomic_cmpxchg_acquire(&lock->val, val, new_val) == val)
			break;
	}

	print_debug("IRQ got the lock\n");

	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	next_node = curr_node->next;

	KOMB_BUG_ON(next_node == NULL);

	WRITE_ONCE(next_node->locked, false);

	print_debug("IRQ passing lock next node: %d\n", next_node->cpuid);

irq_release:
	//curr_node = this_cpu_ptr(&komb_delegation_nodes[0]);
	//curr_node->count--;
	return;
}

/*void komb_delegation_spin_unlock_nested(struct qspinlock *lock)
{
	KOMB_BUG_ON(lock->locked != _Q_LOCKED_VAL);
	print_debug("Releasing the lock nested case\n");
	WRITE_ONCE(lock->locked, false);
}*/

/* 
 * No IPA is even more relevant here, because even if assembly is removed,
 * compiler shouldn't assume which caller-saved registers are in use inside
 * the function. When we jump from the spin loop to the return address after
 * the unlock function, we have caller-saved registers that are completely
 * inconsistent, because we only restored the callee-saved registers.
 */
__attribute__((noipa)) noinline notrace void komb_delegation_spin_unlock(struct qspinlock *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
	int from_cpuid = ptr->curr_cs_cpu;
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct komb_delegation_node *curr_node;
#if WWJUMP
	struct komb_delegation_node *next_node;
	uint64_t counter;
#endif
	int j, max_idx, my_idx;
	void *temp_lock_addr;

#if LOCK_MEASURE_TIME
	LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop_unlockfn);
#endif

#if WWJUMP
#if LOCK_MEASURE_TIME
	LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop);
	LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop);
//	LOCK_END_TIMING_PER_CPU(combiner_loop);
//	LOCK_START_TIMING_PER_CPU(combiner_loop);
#endif
#endif

	/*if(lock->locked != _Q_LOCKED_COMBINER_VAL) {
		print_debug("releasing lock locked value: %d\n", lock->locked);
		KOMB_BUG_ON(lock->locked != _Q_LOCKED_VAL);
		WRITE_ONCE(lock->locked, 0);
		return;
	}*/

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
		KOMB_BUG_ON(lock->locked != _Q_LOCKED_VAL);
		WRITE_ONCE(lock->locked, 0);
		print_debug("Unlocked the qspinlock\n");
		return;
	}

	//Delegation thread should be on CPU 0 on each socket
	KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);
	KOMB_BUG_ON(from_cpuid == -1);

	curr_node = per_cpu_ptr(&komb_delegation_nodes[0], from_cpuid);

#if WWJUMP
	next_node = ptr->next_node_ptr;

	counter = ptr->counter_val;

	if (next_node == NULL || counter >= komb_batch_size || need_resched()) {
		incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = curr_node->cpuid;

	} else {
#if NUMA_AWARE
		ptr->is_local_queue_tail_last = false;
#endif
		ptr->curr_cs_cpu = next_node->cpuid;
		ptr->prev_cs_cpu = curr_node->cpuid;
		incoming_rsp_ptr = &(next_node->rsp);
		ptr->counter_val = counter + 1;
		//prefetchw(((char*)curr_node) + 195);
		print_debug("Jumping to the next waiter: %d\n", next_node->cpuid);
	}
#else // WWJUMP
	incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
	print_debug("Jumping back to combiner\n");
#endif
	/*
	 * Komb node still active here, because cpu (from_cpuid) still spinning.
	 */
	outgoing_rsp_ptr = &(curr_node->rsp);

	//KOMB_BUG_ON(*(char *)incoming_rsp_ptr == 0);
	//KOMB_BUG_ON(*(char *)outgoing_rsp_ptr == 0);
	//KOMB_BUG_ON(incoming_rsp_ptr == (void *)0xdeadbeef);
	//KOMB_BUG_ON(outgoing_rsp_ptr == (void *)0xdeadbeef);

#if LOCK_MEASURE_TIME
	LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop_unlockfn);
#endif

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	LOCK_END_TIMING_PER_CPU_DISABLE(unlock_stack_switch);
	return;
}

#if KOMB_STATS
void komb_delegation_print_stats(void)
{
	int i;
	uint64_t total_counters[5] = { 0 };
	printk(KERN_ALERT "======== KOMB spinlock stats ========\n");
	for_each_online_cpu (i) {
		total_counters[0] += per_cpu(combiner_count, i);
		total_counters[1] += per_cpu(waiter_combined, i);
		total_counters[2] += per_cpu(ooo_unlocks, i);
		total_counters[3] += per_cpu(ooo_combiner_count, i);
		total_counters[4] += per_cpu(ooo_waiter_combined, i);
	}

	printk(KERN_ALERT "Combiner_count: %lld\n", total_counters[0]);
	printk(KERN_ALERT "waiter_combined: %lld\n", total_counters[1]);
	printk(KERN_ALERT "ooo_unlocks: %lld\n", total_counters[2]);
	printk(KERN_ALERT "ooo_combiner_count: %lld\n", total_counters[3]);
	printk(KERN_ALERT "ooo_waiter_combined: %lld\n", total_counters[4]);
}
#endif
