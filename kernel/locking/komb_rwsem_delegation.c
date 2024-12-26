// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

/*
 * TODO: (Performance optimiztion)
 * Fix the index part when same is acquired multiple times.
 * Currently nested locking becomes TTAS lock.
 * irqs_disabled() will hurt performance when running in VM.
 */
#if KERNEL_SYNCSTRESS
#include "mutex/komb_mutex_delegation.h"
#include "rwsem/komb_rwsem_delegation.h"
#include "timing_stats.h"
#else
#include <linux/timing_stats.h>
#include <linux/komb_rwsem_delegation.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#define LOCK_START_TIMING_PER_CPU(combiner_loop)
#define LOCK_END_TIMING_PER_CPU(combiner_loop)
#endif
#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <linux/sched/stat.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

#define DSM_DEBUG 1
#define DEBUG_KOMB 1

#if DSM_DEBUG
#define print_debug(fmt, ...)                                              \
	({                                                                 \
		printk(KERN_EMERG "[%d] komb (%s) lock(%px): " fmt,        \
		       smp_processor_id(), __func__, lock, ##__VA_ARGS__); \
	})
#define print_debug_without_lock(fmt, ...)                                    \
	({                                                                    \
		printk(KERN_EMERG "[%d] komb (%s): " fmt, smp_processor_id(), \
		       __func__, ##__VA_ARGS__);                              \
	})
#else
#define print_debug(fmt, ...)
#define print_debug_without_lock(fmt, ...)
#endif

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#define READ_STATE 0
#define WRITE_STATE 1

#define KOMB_WAITER_UNPROCESSED 0
#define KOMB_WAITER_PARKED 1
#define KOMB_WAITER_PROCESSING 2
#define KOMB_WAITER_PROCESSED 4
#define KOMB_WAITER_WAKER 8

#define KOMB_WAKER_COUNT_SHIFT 16
#define KOMB_WAKER_COUNT (1U << KOMB_WAKER_COUNT_SHIFT)

#define smp_cond_load_relaxed_sleep(curr_node, ptr, cond_expr)                 \
	({                                                                     \
		typeof(ptr) __PTR = (ptr);                                     \
		__unqual_scalar_typeof(*ptr) VAL;                              \
		for (;;) {                                                     \
			VAL = READ_ONCE(*__PTR);                               \
			if (cond_expr)                                         \
				break;                                         \
			cpu_relax();                                           \
			if (need_resched()) {                                  \
				if (single_task_running())                     \
					schedule_out_curr_task();              \
				else {                                         \
					if (READ_ONCE(curr_node->completed) == \
					    KOMB_WAITER_UNPROCESSED)           \
						park_waiter(curr_node);        \
					else                                   \
						schedule_out_curr_task();      \
				}                                              \
			}                                                      \
		}                                                              \
		(typeof(*ptr))VAL;                                             \
	})

#define smp_cond_load_relaxed_sched(ptr, cond_expr) \
	({                                          \
		typeof(ptr) __PTR = (ptr);          \
		__unqual_scalar_typeof(*ptr) VAL;   \
		for (;;) {                          \
			VAL = READ_ONCE(*__PTR);    \
			if (cond_expr)              \
				break;              \
			cpu_relax();                \
			if (need_resched()) {       \
				schedule_out_curr_task();     \
			}                           \
		}                                   \
		(typeof(*ptr))VAL;                  \
	})

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr)  \
	({                                                      \
		typeof(ptr) __PTR = (ptr);                      \
		__unqual_scalar_typeof(*ptr) VAL;               \
		for (;;) {                                      \
			VAL = READ_ONCE(*__PTR);                \
			if (cond_expr || kthread_should_stop()) \
				break;                          \
			cpu_relax();                            \
			if (need_resched()) {                   \
				cond_resched();                 \
			}                                       \
		}                                               \
		(typeof(*ptr))VAL;                              \
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

static struct task_struct **dthreads;
static int num_rwsemd_threads = 1;
static int rwsemd_num_cores_per_socket = 1;

#if LOCK_MEASURE_TIME
static DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_loop);
#endif

static DEFINE_PER_CPU_ALIGNED(struct kombd_mutex_node, *lock_rq_tail);

#define _Q_UNLOCKED_OOO_VAL 7 //Unlocked a lock out-of-order

#define KOMB_WAITER_UNPROCESSED 0
#define KOMB_WAITER_PARKED 1
#define KOMB_WAITER_PROCESSING 2
#define KOMB_WAITER_PROCESSED 4

static inline void schedule_out_curr_task(void)
{
	preempt_enable();
	schedule();
	preempt_disable();
}

static inline void park_waiter(struct kombd_mutex_node *node)
{
	__set_current_state(TASK_INTERRUPTIBLE);

	if (cmpxchg(&node->completed, KOMB_WAITER_UNPROCESSED,
		    KOMB_WAITER_PARKED) != KOMB_WAITER_UNPROCESSED) {
		__set_current_state(TASK_RUNNING);
		return;
	}
	schedule_out_curr_task();
	__set_current_state(TASK_RUNNING);
}

static inline void wake_up_waiter(struct kombd_mutex_node *node)
{
	u8 old_val = xchg(&node->completed, KOMB_WAITER_PROCESSING);

	if (old_val == KOMB_WAITER_PARKED) {
		struct task_struct *task = node->task_struct_ptr;
		get_task_struct(task);
		wake_up_process(task);
		put_task_struct(task);
	}
}

__attribute__((noipa)) noinline notrace static uint64_t
get_shadow_stack_ptr(struct kombd_rwsem *lock)
{
	return &current->komb_stack_curr_ptr;
}

__attribute__((noipa)) noinline notrace static struct kombd_mutex_node *
get_kombd_mutex_node(struct kombd_rwsem *lock)
{
	return ((struct kombd_mutex_node *)(current->komb_mutex_node));
}

static __always_inline void
clear_locked_set_completed(struct kombd_mutex_node *node)
{
	WRITE_ONCE(node->completed, KOMB_WAITER_PROCESSED);
	WRITE_ONCE(node->locked, 0);
}

#if NUMA_AWARE
__always_inline static void add_to_local_queue(struct kombd_mutex_node *node)
{
	struct kombd_mutex_node **head, **tail;

	head = (struct kombd_mutex_node **)(&current->komb_local_queue_head);
	tail = (struct kombd_mutex_node **)(&current->komb_local_queue_tail);

	if (*head == NULL) {
		*head = node;
		*tail = node;
	} else {
		(*tail)->next = node;
		*tail = node;
	}

	current->komb_lock_addr[7] = 0xdeadbeef;
}
#endif

static __always_inline struct kombd_mutex_node *
get_next_node(struct kombd_mutex_node *my_node)
{
#if NUMA_AWARE
	struct kombd_mutex_node *curr_node, *next_node;
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
			void *rsp_ptr = next_node->rsp;
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

static inline void kombd_read_lock_slowpath(struct kombd_rwsem *lock)
{
	//print_debug("Reader waiting for spinlock\n");
	aqm_lock(&lock->reader_wait_lock);
	atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS, &lock->cnts);
	//print_debug(
	//	"Reader slowpath got wait lock, waiting for writer to go away\n");
	atomic_long_cond_read_acquire(&lock->cnts,
				      !(VAL & _KOMB_RWSEM_W_WMASK));
	//print_debug("Reader slowpath got the lock\n");
	aqm_unlock(&lock->reader_wait_lock);
	return;
}

void kombd_rwsem_down_read(struct kombd_rwsem *lock)
{
	u64 cnts;

	cnts = atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS, &lock->cnts);

	if (likely(!(cnts & _KOMB_RWSEM_W_WMASK))) {
		return;
	}

	(void)atomic_long_sub_return_release(_KOMB_RWSEM_R_BIAS, &lock->cnts);

	//preempt_disable();
	kombd_read_lock_slowpath(lock);
	//preempt_enable();
}
EXPORT_SYMBOL_GPL(kombd_rwsem_down_read);

#pragma GCC push_options
#pragma GCC optimize("O3")

__attribute__((noipa)) noinline notrace static void
execute_cs(struct kombd_mutex_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;

	WRITE_ONCE(current->komb_curr_waiter_task, curr_node->task_struct_ptr);
	KOMB_BUG_ON(curr_node->cpuid == smp_processor_id());

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = get_shadow_stack_ptr(NULL);

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
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
run_combiner(struct kombd_rwsem *lock, struct kombd_mutex_node *curr_node)
{
	KOMB_BUG_ON(curr_node == NULL);
	KOMB_BUG_ON((smp_processor_id() % rwsemd_num_cores_per_socket) != 0);

#if LOCK_MEASURE_TIME
	*this_cpu_ptr(&combiner_loop) = UINT64_MAX;
#endif

#if NUMA_AWARE
	current->komb_lock_addr[7] =
		NULL; //TODO: fix this to is_local_queue_tail_last
#endif

	current->counter_val = 0;

	execute_cs(curr_node);

#if KOMB_STATS
	this_cpu_add(waiter_combined, current->counter_val);
	this_cpu_inc(combiner_count);
#endif

	KOMB_BUG_ON(current->komb_prev_waiter_task == NULL);
	current->komb_curr_waiter_task = NULL;
}
#pragma GCC pop_options

static inline __pure u32 select_delegation_cpu(struct kombd_rwsem *lock)
{
#if NUMA_AWARE
	return (rwsemd_num_cores_per_socket * numa_node_id());
//		       	+((u64)lock % num_rwsemd_threads_per_socket));
#else
	return 0;
#endif
}

__attribute__((noipa)) noinline notrace static int
__kombd_write_lock_slowpath(struct kombd_rwsem *lock)
{
	struct kombd_mutex_node *curr_node, *next_node;
	struct kombd_mutex_node **rq_tail;
	register struct kombd_mutex_node *prev_node;
#if PREFETCHING
	u32 i;
#endif

	LOCK_END_TIMING_PER_CPU_DISABLE(lock_stack_switch);

	curr_node = get_kombd_mutex_node(lock);

	/*
	 * Initialize curr_node
	 */
	curr_node->locked = true;
	curr_node->completed = KOMB_WAITER_UNPROCESSED;
	curr_node->next = NULL;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;

	prev_node = xchg(&lock->writer_tail, curr_node);

	if (prev_node) {
		WRITE_ONCE(prev_node->next, curr_node);
		smp_mb();
		print_debug("prev_node: %d\n", prev_node->cpuid);
	} else {
head_of_queue:
		print_debug("Head of queue\n");

		print_debug("Writer owner on slowpath\n");
		aqm_lock(&lock->reader_wait_lock);

		print_debug(
			"Writer got the mutex lock. waiting for pending readers\n");

		if (!atomic_long_read(&lock->cnts) &&
		    (atomic_long_cmpxchg_relaxed(&lock->cnts, 0,
						 _KOMB_RWSEM_W_LOCKED) == 0)) {
			print_debug("No pending readers\n");
			goto unlock;
		}

		atomic_long_add_return_acquire(_KOMB_RWSEM_W_WAITING,
					       &lock->cnts);
		print_debug("Writer set the pending bit\n");
		do {
			atomic_long_cond_read_acquire(
				&lock->cnts, VAL == _KOMB_RWSEM_W_WAITING);
		} while (atomic_long_cmpxchg_relaxed(&lock->cnts,
						     _KOMB_RWSEM_W_WAITING,
						     _KOMB_RWSEM_W_LOCKED) !=
			 _KOMB_RWSEM_W_WAITING);
unlock:
		print_debug("Writer got the lock slowpath\n");
		aqm_unlock(&lock->reader_wait_lock);

		rq_tail =
			per_cpu_ptr(&lock_rq_tail, select_delegation_cpu(lock));
		if (READ_ONCE(*rq_tail) == 0xdeadbeef ||
				!task_is_running(dthreads[numa_node_id()]) ||
				cmpxchg(rq_tail, NULL, curr_node) != NULL) {
			// Fallback to qspinlock
			print_debug("Delegation %d running something else\n",
				    select_delegation_cpu(lock));
			curr_node->locked = false;
			curr_node->completed = KOMB_WAITER_PROCESSED;

			if (cmpxchg(&lock->writer_tail, curr_node, NULL) ==
			    curr_node) {
				print_debug(
					"IRQ only one in the queue unlocked\n");
				goto continue_with_cs_execution;
			} else {
				print_debug("Someone else joined the queue\n");
			}

			next_node = READ_ONCE(curr_node->next);

			while (!next_node) {
				next_node = READ_ONCE(curr_node->next);

				cpu_relax();
				if (need_resched())
					schedule_out_curr_task();
			}

			KOMB_BUG_ON(curr_node->next == NULL);
			print_debug("Next node now head of queue: %d\n",
				    curr_node->next->cpuid);
			wake_up_waiter(curr_node->next);
			WRITE_ONCE(curr_node->next->locked, false);

		} else {
			print_debug("Added to the delegation thread: %d\n",
				    select_delegation_cpu(lock));
		}
	}

	smp_cond_load_relaxed_sleep(curr_node, &curr_node->locked, VAL == 0);

	if (READ_ONCE(curr_node->completed) != KOMB_WAITER_PROCESSED) {
		print_debug("Head of queue but not completed\n");
		curr_node->locked = true;
		curr_node->completed = KOMB_WAITER_UNPROCESSED;
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

int komb_rwd_thread(void *args)
{
	struct kombd_rwsem *lock;
	struct kombd_mutex_node *prev_node, *next_node;
	struct kombd_mutex_node **rq_tail;
	struct kombd_mutex_node **head, **tail;
	int j;

	rq_tail = this_cpu_ptr(&lock_rq_tail);
	lock = NULL;

	while (true) {
		if (READ_ONCE(*rq_tail) != 0xdeadbeef)
			break;
		cpu_relax();
		cond_resched();
	}

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
		KOMB_BUG_ON(lock->wlocked ==
			    0); //Lock should already be acquired.
		lock->wlocked = _KOMB_RWSEM_W_COMBINER;
		print_debug("Running combiner with node from: %d\n",
			    next_node->cpuid);

		j = 0;
		for (j = 0; j < 7; j++)
			if (current->komb_lock_addr[j] == NULL)
				break;

		KOMB_BUG_ON(j !=
			    0); //TODO: Update this condition nested delegation

		current->komb_lock_addr[j] = lock;
#if NUMA_AWARE
		current->komb_local_queue_head = NULL;
		current->komb_local_queue_tail = NULL;
		current->komb_lock_addr[7] =
			NULL; //->is_local_queue_tail_last = false;
#endif
		current->komb_curr_waiter_task = NULL;
		current->komb_prev_waiter_task = NULL;
		current->komb_next_waiter_task = NULL;
		current->counter_val = 0;

		preempt_disable();

		run_combiner(lock, next_node);

		preempt_enable();

		prev_node =
			(((struct task_struct *)current->komb_prev_waiter_task)
				 ->komb_mutex_node);

		KOMB_BUG_ON(current->komb_lock_addr[j] != lock);
		current->komb_lock_addr[j] = NULL;

		print_debug("Got control back prev node: %d\n",
			    prev_node->cpuid);
		KOMB_BUG_ON(prev_node == NULL);
		next_node = NULL;
		WRITE_ONCE(*rq_tail, NULL); //Combining done

		head = (struct kombd_mutex_node *
				*)(&current->komb_local_queue_head);
		tail = (struct kombd_mutex_node *
				*)(&current->komb_local_queue_tail);

#if NUMA_AWARE
		if (READ_ONCE(prev_node->next) == NULL) {
			if (*head != NULL) {
				if (current->komb_lock_addr[7] ==
				    NULL) { //if (!ptr->is_local_queue_tail_last) {
					(*tail)->next = NULL;
					if (cmpxchg(&lock->writer_tail,
						    prev_node,
						    (*tail)) != prev_node) {
						smp_cond_load_relaxed_sched(
							&prev_node->next,
							(VAL));
						(*tail)->next = prev_node->next;
					}
				}
				next_node = *head;
			} else {
				KOMB_BUG_ON(
					current->komb_lock_addr[7] ==
					0xdeadbeef); //KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				if (cmpxchg(&lock->writer_tail, prev_node,
					    NULL) != prev_node) {
					smp_cond_load_relaxed_sched(
						&prev_node->next, (VAL));
					next_node = prev_node->next;
				}
			}
		} else {
			KOMB_BUG_ON(
				current->komb_lock_addr[7] ==
				0xdeadbeef); //KOMB_BUG_ON(ptr->is_local_queue_tail_last);
			if (*head != NULL) {
				(*tail)->next = prev_node->next;
				next_node = *head;
			} else {
				next_node = prev_node->next;
			}
		}

#else //NUMA_AWARE
		if (READ_ONCE(prev_node->next) == NULL) {
			if (cmpxchg(&lock->writer_tail, prev_node, NULL) !=
			    prev_node) {
				smp_cond_load_relaxed_sched(&prev_node->next,
							    (VAL));
				next_node = prev_node->next;
			}
		} else {
			next_node = READ_ONCE(prev_node->next);
		}

#endif //NUMA_AWARE

		if (READ_ONCE(next_node) != NULL) {
			print_debug(
				"Transferring lock to another socket %d cpuid %d\n",
				next_node->socket_id, next_node->cpuid);

			next_node->locked = false;
		} else {
			print_debug("My node next NULL\n");
		}

		wake_up_waiter(prev_node);
		clear_locked_set_completed(prev_node);

		//Release the lock
		KOMB_BUG_ON(!(lock->wlocked == _KOMB_RWSEM_W_COMBINER || lock->wlocked == _KOMB_RWSEM_W_DOWNGRADE));
		print_debug("Releasing the lock from combiner\n");
		WRITE_ONCE(lock->wlocked, 0);

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

void kombd_rwsem_init(void)
{
	int i;

	for_each_possible_cpu(i) {
		*per_cpu_ptr(&lock_rq_tail, i) = 0xdeadbeef;
	}

#if LOCK_MEASURE_TIME
	*per_cpu_ptr(&do_timing, KOMB_CPU) = true;
#endif

	rwsemd_num_cores_per_socket = num_online_cpus() / num_online_nodes();

#if NUMA_AWARE
	num_rwsemd_threads = num_online_nodes();
#else
	num_rwsemd_threads = 1;
#endif

	dthreads =
		vzalloc(num_rwsemd_threads * sizeof(struct task_struct *));
	for (i = 0; i < num_rwsemd_threads; i++) {
		dthreads[i] =
			kthread_create(komb_rwd_thread, NULL, "komb_rw_thread");
		kthread_bind(dthreads[i], i * rwsemd_num_cores_per_socket);
		if (dthreads[i])
			wake_up_process(dthreads[i]);
		else
			printk(KERN_ALERT
			       "failed to create komb delegation threads\n");
	}
}

void kombd_rwsem_free(void)
{
	int ret;
	uint32_t i;

	for (i = 0; i < num_rwsemd_threads; i++) {
		ret = kthread_stop(dthreads[i]);
		if (ret)
			printk(KERN_ALERT
			       "komb delegate thread returned error %d\n",
			       ret);
	}
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
static __attribute__((noipa)) noinline notrace void
kombd_write_lock_slowpath(struct kombd_rwsem *lock)
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
		     : "i"(get_kombd_mutex_node),
		       "i"(offsetof(struct kombd_mutex_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");
	//local_irq_enable();

	ret_val = __kombd_write_lock_slowpath(lock);

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
			     : "i"(get_kombd_mutex_node),
			       "i"(offsetof(struct kombd_mutex_node, rsp))
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

void kombd_rwsem_down_write(struct kombd_rwsem *lock)
{
	struct kombd_mutex_node *curr_node = NULL;
	struct kombd_mutex_node **rq_tail;
	u64 val;
	val = atomic_long_cmpxchg_acquire(&lock->cnts, 0, _KOMB_RWSEM_W_LOCKED);
	if (val == 0)
		return;

	preempt_disable();

	curr_node = get_kombd_mutex_node(lock);
	KOMB_BUG_ON(curr_node == NULL);
	/*if ((smp_processor_id() % rwsemd_num_cores_per_socket) == 0) {
		int j = 0;
		for (j = 0; j < 7; j++)
			if (current->komb_lock_addr[j] != NULL)
				break;
		print_debug("lock addr index: %d\n", j);
		KOMB_BUG_ON(true);
	}*/

#if LOCK_MEASURE_TIME
//		*this_cpu_ptr(&lock_stack_switch) = UINT64_MAX;
//		*this_cpu_ptr(&unlock_stack_switch) = UINT64_MAX;
#endif
	LOCK_START_TIMING_PER_CPU_DISABLE(lock_stack_switch);

	
	rq_tail = per_cpu_ptr(&lock_rq_tail, select_delegation_cpu(lock));

	if(READ_ONCE(*rq_tail) == 0xdeadbeef || (smp_processor_id() % rwsemd_num_cores_per_socket) == 0)
		kombd_rwsem_down_write_nested(lock, 0);
	else
		kombd_write_lock_slowpath(lock);

#if WWJUMP
	if (current->komb_curr_waiter_task) {
		struct kombd_mutex_node *curr_node =
			((struct task_struct *)current->komb_curr_waiter_task)
				->komb_mutex_node;

		if ((struct kombd_rwsem *)curr_node->lock == lock) {
			KOMB_BUG_ON(lock->wlocked != _KOMB_RWSEM_W_COMBINER);
			struct kombd_mutex_node *next_node =
				get_next_node(curr_node);
			if (next_node == NULL)
				current->komb_next_waiter_task = NULL;
			else
				current->komb_next_waiter_task =
					next_node->task_struct_ptr;
		}

		wake_up_waiter(curr_node);

		if (current->komb_prev_waiter_task) {
			struct kombd_mutex_node *prev_node =
				((struct task_struct *)
					 current->komb_prev_waiter_task)
					->komb_mutex_node;

			KOMB_BUG_ON(prev_node->lock != lock);
			print_debug("Waking up prev waiter: %d\n",
				    prev_node->cpuid);
			wake_up_waiter(prev_node);
			clear_locked_set_completed(prev_node);
			current->komb_prev_waiter_task = NULL;
		}
	}
#endif

	preempt_enable();
}

void kombd_rwsem_up_read(struct kombd_rwsem *lock)
{
	int j, max_idx, my_idx;
	uint64_t temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 7; j++) {
		temp_lock_addr = current->komb_lock_addr[j];
		if (temp_lock_addr)
			max_idx = j;
		if (temp_lock_addr == lock)
			my_idx = j;
		if (temp_lock_addr == NULL)
			break;
	}

	if (my_idx == -1) {
		KOMB_BUG_ON(lock->wlocked != 0);
		atomic_long_sub_return_release(_KOMB_RWSEM_R_BIAS, &lock->cnts);
	} else {
		if (my_idx == max_idx) {
			KOMB_BUG_ON(lock->wlocked != _KOMB_RWSEM_W_DOWNGRADE);
			kombd_rwsem_up_write(lock);
		} else {
			BUG_ON(true);
		}
	}
}
EXPORT_SYMBOL_GPL(kombd_rwsem_up_read);

__attribute__((noipa)) noinline notrace void
kombd_rwsem_up_write(struct kombd_rwsem *lock)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct kombd_mutex_node *curr_node;
#if WWJUMP
	struct kombd_mutex_node *next_node;
	uint64_t counter;
#endif
	int j, max_idx, my_idx;
	void *temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 7; j++) {
		temp_lock_addr = current->komb_lock_addr[j];
		if (temp_lock_addr != NULL)
			max_idx = j;
		if (temp_lock_addr == lock)
			my_idx = j;
		if (temp_lock_addr == NULL)
			break;
	}

	if (my_idx == -1) {
		KOMB_BUG_ON(lock->wlocked != _KOMB_RWSEM_W_LOCKED);
		WRITE_ONCE(lock->wlocked, 0);
		//print_debug("Unlocked the qspinlock\n");
		return;
	}

#if LOCK_MEASURE_TIME
	LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop);
	LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop);
#endif

	//Delegation thread should be on CPU 0 on each socket
	KOMB_BUG_ON((smp_processor_id() % rwsemd_num_cores_per_socket) != 0);

	curr_node = ((struct task_struct *)current->komb_curr_waiter_task)
			    ->komb_mutex_node;

#if WWJUMP
	if (current->komb_next_waiter_task)
		next_node =
			((struct task_struct *)current->komb_next_waiter_task)
				->komb_mutex_node;
	else
		next_node = NULL;

	counter = current->counter_val;

	if (next_node == NULL || counter >= komb_batch_size || need_resched()) {
		incoming_rsp_ptr = get_shadow_stack_ptr(lock);
		current->komb_prev_waiter_task = current->komb_curr_waiter_task;
		current->komb_curr_waiter_task = NULL;
	} else {
#if NUMA_AWARE
		current->komb_lock_addr[7] =
			NULL; //ptr->is_local_queue_tail_last = false;
#endif
		current->komb_prev_waiter_task = current->komb_curr_waiter_task;
		current->komb_curr_waiter_task = current->komb_next_waiter_task;
		incoming_rsp_ptr = &(next_node->rsp);
		current->counter_val = counter + 1;
		print_debug("Jumping to the next waiter: %d\n",
			    next_node->cpuid);
	}
#else // WWJUMP
	incoming_rsp_ptr = get_shadow_stack_ptr(lock);
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

	preempt_disable();
	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	preempt_enable();
	return;
}

void kombd_init_rwsem(struct kombd_rwsem *sem)
{
	sem->writer_tail = NULL;
	atomic_set(&sem->reader_wait_lock.val, 0);
	sem->reader_wait_lock.tail = NULL;
	atomic_long_set(&sem->cnts, 0);
}

bool kombd_rwsem_down_read_trylock(struct kombd_rwsem *lock)
{
	u64 cnts;

	cnts = atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS, &lock->cnts);

	if (likely(!(cnts & _KOMB_RWSEM_W_WMASK))) {
		return true;
	}

	(void)atomic_long_sub_return_release(_KOMB_RWSEM_R_BIAS, &lock->cnts);
	return false;
}

void kombd_rwsem_down_write_nested(struct kombd_rwsem *lock, int subclass)
{
	u64 val;
	val = atomic_long_cmpxchg_acquire(&lock->cnts, 0, _KOMB_RWSEM_W_LOCKED);
	if (val == 0)
		return;

	struct kombd_mutex_node *curr_node, *next_node;
	register struct kombd_mutex_node *prev_node;

	curr_node = get_kombd_mutex_node(lock);

	curr_node->locked = true;
	curr_node->completed = KOMB_WAITER_UNPROCESSED;
	curr_node->next = NULL;
	curr_node->socket_id = IRQ_NUMA_NODE;
	curr_node->cpuid = smp_processor_id();
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;

	prev_node = xchg(&lock->writer_tail, curr_node);

	if (prev_node) {
		WRITE_ONCE(prev_node->next, curr_node);
		smp_mb();
		//print_debug("prev_node: %d\n", prev_node->cpuid);

                smp_cond_load_relaxed_sleep(curr_node, &curr_node->locked, VAL == 0);

                KOMB_BUG_ON(READ_ONCE(curr_node->completed) == KOMB_WAITER_PROCESSED);
	}

head_of_queue:
        //print_debug("Head of queue\n");

        //print_debug("Writer owner on slowpath\n");
        aqm_lock(&lock->reader_wait_lock);

        //print_debug(
        //        "Writer got the mutex lock. waiting for pending readers\n");

        if (!atomic_long_read(&lock->cnts) &&
                (atomic_long_cmpxchg_relaxed(&lock->cnts, 0,
                                                _KOMB_RWSEM_W_LOCKED) == 0)) {
                //print_debug("No pending readers\n");
                goto unlock;
        }

        atomic_long_add_return_acquire(_KOMB_RWSEM_W_WAITING,
                                        &lock->cnts);
        //print_debug("Writer set the pending bit\n");
        do {
                atomic_long_cond_read_acquire(
                        &lock->cnts, VAL == _KOMB_RWSEM_W_WAITING);
        } while (atomic_long_cmpxchg_relaxed(&lock->cnts,
                                                _KOMB_RWSEM_W_WAITING,
                                                _KOMB_RWSEM_W_LOCKED) !=
                        _KOMB_RWSEM_W_WAITING);
unlock:
        //print_debug("Writer got the lock slowpath\n");
        aqm_unlock(&lock->reader_wait_lock);

        if (cmpxchg(&lock->writer_tail, curr_node, NULL) ==
                curr_node) {
                //print_debug(
                //        "IRQ only one in the queue unlocked\n");
		return;
        } else {
                //print_debug("Someone else joined the queue\n");
        }

        next_node = READ_ONCE(curr_node->next);

        while (!next_node) {
                next_node = READ_ONCE(curr_node->next);

                cpu_relax();
                if (need_resched())
                        schedule_out_curr_task();
        }

        KOMB_BUG_ON(curr_node->next == NULL);
        //print_debug("Next node now head of queue: %d\n",
        //                curr_node->next->cpuid);
        wake_up_waiter(curr_node->next);
        WRITE_ONCE(curr_node->next->locked, false);
}

void kombd_rwsem_downgrade_write(struct kombd_rwsem *lock)
{
	int j, max_idx, my_idx;
	uint64_t temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 7; j++) {
		temp_lock_addr = current->komb_lock_addr[j];
		if (temp_lock_addr)
			max_idx = j;
		if (temp_lock_addr == lock)
			my_idx = j;
		if (temp_lock_addr == NULL)
			break;
	}

	if (my_idx == -1) {
		if (lock->wlocked == _KOMB_RWSEM_W_LOCKED) {
			atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS,
						       &lock->cnts);
			WRITE_ONCE(lock->wlocked, 0);
			return;
		}
		BUG_ON(true);
		return;
	}

	if (my_idx == max_idx) {
#ifdef KOMB_STATS
		this_cpu_inc(rwsem_downgrade);
#endif
		print_debug("Downgrade with combinining\n");
		lock->wlocked = _KOMB_RWSEM_W_DOWNGRADE;
		return;
	}
	BUG_ON(true);
}

void kombd_rwsem_up_read_non_owner(struct kombd_rwsem *sem) {
        kombd_rwsem_up_read(sem);
}

bool kombd_rwsem_is_contended(struct kombd_rwsem *sem) {
        return (sem->writer_tail != NULL);
}

SYSCALL_DEFINE0(komb_start_rwsem_delegation)
{
	printk(KERN_ALERT "======== KOMB starting RWSEM delegation ========\n");
	int i;
	for_each_online_cpu(i) {
		*per_cpu_ptr(&lock_rq_tail, i) = 0;
	}

	return 0;
}
