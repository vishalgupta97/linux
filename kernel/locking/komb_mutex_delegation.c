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
#include "timing_stats.h"
#else
#include <linux/komb_mutex_delegation.h>
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

#if DSM_DEBUG
#define print_debug(fmt, ...)                                                                      \
		({                                                                                         \
				printk(KERN_EMERG "[%d] komb (%s) lock(%px): " fmt, smp_processor_id(), __func__,  \
					   lock, ##__VA_ARGS__);                                                       \
		})
#define print_debug_without_lock(fmt, ...)                                                         \
		({                                                                                         \
				printk(KERN_EMERG "[%d] komb (%s): " fmt, smp_processor_id(), __func__,            \
					   ##__VA_ARGS__);                                                             \
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

#if KERNEL_SYNCSTRESS
#define smp_cond_load_relaxed_sleep(curr_node, ptr, cond_expr)                                     \
		({                                                                                         \
				typeof(ptr) __PTR = (ptr);                                                         \
				__unqual_scalar_typeof(*ptr) VAL;                                                  \
				for (;;) {                                                                         \
						VAL = READ_ONCE(*__PTR);                                                   \
						if (cond_expr)                                                             \
								break;                                                             \
						cpu_relax();                                                               \
						if (need_resched()) {                                                      \
								if (single_task_running())                                         \
										schedule_out_curr_task();                                  \
								else {                                                             \
										if (READ_ONCE(curr_node->completed) ==                     \
											KOMB_WAITER_UNPROCESSED)                               \
												park_waiter(curr_node);                            \
										else                                                       \
												schedule_out_curr_task();                          \
								}                                                                  \
						}                                                                          \
				}                                                                                  \
				(typeof(*ptr)) VAL;                                                                \
		})


#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
		({                                                                                         \
				typeof(ptr) __PTR = (ptr);                                                         \
				__unqual_scalar_typeof(*ptr) VAL;                                                  \
				for (;;) {                                                                         \
						VAL = READ_ONCE(*__PTR);                                                   \
						if (cond_expr)                                                             \
								break;                                                             \
						cpu_relax();                                                               \
						if (need_resched()) {                                                      \
								cond_resched();                                                    \
						}                                                                          \
				}                                                                                  \
				(typeof(*ptr)) VAL;                                                                \
		})

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr)                                     \
		({                                                                                         \
				typeof(ptr) __PTR = (ptr);                                                         \
				__unqual_scalar_typeof(*ptr) VAL;                                                  \
				for (;;) {                                                                         \
						VAL = READ_ONCE(*__PTR);                                                   \
						if (cond_expr || kthread_should_stop())                                    \
								break;                                                             \
						cpu_relax();                                                               \
						if (need_resched()) {                                                      \
								cond_resched();                                                    \
						}                                                                          \
				}                                                                                  \
				(typeof(*ptr)) VAL;                                                                \
		})

#else
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
		({                                                                                         \
				typeof(ptr) __PTR = (ptr);                                                         \
				__unqual_scalar_typeof(*ptr) VAL;                                                  \
				for (;;) {                                                                         \
						VAL = READ_ONCE(*__PTR);                                                   \
						if (cond_expr)                                                             \
								break;                                                             \
						cpu_relax();                                                               \
				}                                                                                  \
				(typeof(*ptr)) VAL;                                                                \
		})

//if (need_resched()) {
//	schedule_preempt_disabled();
//}
#endif

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                                                \
		({                                                                                         \
				__unqual_scalar_typeof(*ptr) _val;                                                 \
				_val = smp_cond_load_relaxed_sched(ptr, cond_expr);                                \
				smp_acquire__after_ctrl_dep();                                                     \
				(typeof(*ptr)) _val;                                                               \
		})
#endif

#define atomic_cond_read_acquire_sched(v, c) smp_cond_load_acquire_sched(&(v)->counter, (c))

static struct task_struct **dthreads;
static int num_delegation_threads;
static int num_delegation_threads_per_socket;

#if LOCK_MEASURE_TIME
static DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_loop);
#endif

static DEFINE_PER_CPU_ALIGNED(struct kombd_mutex_node, *lock_rq_tail);

#define _Q_LOCKED_COMBINER_VAL 3
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

		if (cmpxchg(&node->completed, KOMB_WAITER_UNPROCESSED, KOMB_WAITER_PARKED) !=
			KOMB_WAITER_UNPROCESSED) {
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

void kombd_mutex_lock_init(struct kombd_mutex *lock)
{
		atomic_set(&lock->state, 0);
        lock->tail = NULL;
        lock->combiner_task = NULL;
}

__attribute__((noipa)) noinline notrace static uint64_t
get_shadow_stack_ptr(struct kombd_mutex *lock)
{
		return &current->komb_stack_curr_ptr;
}

__attribute__((noipa)) noinline notrace static struct kombd_mutex_node *
get_kombd_mutex_node(struct kombd_mutex *lock)
{
		return ((struct kombd_mutex_node *)(current->komb_mutex_node));
}


static __always_inline void clear_locked_set_completed(struct kombd_mutex_node *node)
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
run_combiner(struct kombd_mutex *lock, struct kombd_mutex_node *curr_node)
{
		KOMB_BUG_ON(curr_node == NULL);
		KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);
		struct kombd_mutex_node *next_node = curr_node->next;

#if LOCK_MEASURE_TIME
		*this_cpu_ptr(&combiner_loop) = UINT64_MAX;
#endif


#if NUMA_AWARE
		current->komb_lock_addr[7] = NULL; //TODO: fix this to is_local_queue_tail_last
#endif

        WRITE_ONCE(lock->combiner_task, current);
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

static inline __pure u32 select_delegation_cpu(struct kombd_mutex *lock)
{
#if NUMA_AWARE
		return ((num_cores_per_socket * numa_node_id()) +
				((u64)lock % num_delegation_threads_per_socket));
#else
		return 0;
#endif
}

__attribute__((noipa)) noinline notrace static int __kombd_mutex_lock_slowpath(struct kombd_mutex *lock)
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

        prev_node = xchg(&lock->tail, curr_node);

		if (prev_node) {
				WRITE_ONCE(prev_node->next, curr_node);
				smp_mb();
				print_debug("prev_node: %d my_pos: %d\n", prev_node->cpuid, curr_node->pos);
		} else {
		head_of_queue:
				print_debug("Head of queue\n");

                for (;;) {
                        while (READ_ONCE(lock->locked)) {
                                cpu_relax();
                                if (need_resched())
                                        schedule_out_curr_task();
                        }

                        if (cmpxchg(&lock->locked, 0, 1) == 0)
                                break;
                }

				print_debug("Got the lock\n");

				rq_tail = per_cpu_ptr(&lock_rq_tail, select_delegation_cpu(lock));
				if (cmpxchg(rq_tail, NULL, curr_node) != NULL) {
						// Fallback to qspinlock
						print_debug("Delegation %d running something else\n",
									select_delegation_cpu(lock));
						curr_node->locked = false;
						curr_node->completed = KOMB_WAITER_PROCESSED;

						if (cmpxchg(&lock->tail, curr_node, NULL) == curr_node) {
								print_debug("IRQ only one in the queue unlocked\n");
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
						print_debug("Next node now head of queue: %d\n", curr_node->next->cpuid);
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

int komb_md_thread(void *args)
{
		struct kombd_mutex *lock;
		struct shadow_stack *ptr;
		struct kombd_mutex_node *prev_node, *next_node;
		struct kombd_mutex_node **rq_tail;
        struct kombd_mutex_node **head, **tail;
		int j;

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
				for (j = 0; j < 7; j++)
						if (current->komb_lock_addr[j] == NULL)
								break;

				KOMB_BUG_ON(j != 0); //TODO: Update this condition nested delegation

				current->komb_lock_addr[j] = lock;
#if NUMA_AWARE
				current->komb_local_queue_head = NULL;
				current->komb_local_queue_tail = NULL;
				current->komb_lock_addr[7] = NULL; //->is_local_queue_tail_last = false;
#endif
                current->komb_curr_waiter_task = NULL;
                current->komb_prev_waiter_task = NULL;
                current->komb_next_waiter_task = NULL;
                current->counter_val = 0;

                preempt_disable();

				run_combiner(lock, next_node);

                preempt_enable();

                prev_node = (((struct task_struct *)current->komb_prev_waiter_task)->komb_mutex_node);

				KOMB_BUG_ON(current->komb_lock_addr[j] != lock);
				current->komb_lock_addr[j] = NULL;

				print_debug("Got control back prev node: %d\n", prev_node->cpuid);
				KOMB_BUG_ON(prev_node == NULL);
				next_node = NULL;
				WRITE_ONCE(*rq_tail, NULL); //Combining done

                head = (struct kombd_mutex_node **)(&current->komb_local_queue_head);
                tail = (struct kombd_mutex_node **)(&current->komb_local_queue_tail);

#if NUMA_AWARE
				if (READ_ONCE(prev_node->next) == NULL) {
						if (*head != NULL) {
								if(current->komb_lock_addr[7] == NULL) { //if (!ptr->is_local_queue_tail_last) {
										(*tail)->next = NULL;
										if (cmpxchg(&lock->tail, prev_node,
														 (*tail)) != prev_node) {
												smp_cond_load_relaxed_sched(&prev_node->next,
																			(VAL));
												(*tail)->next = prev_node->next;
										}
								}
								next_node = *head;
						} else {
								KOMB_BUG_ON(current->komb_lock_addr[7] == 0xdeadbeef); //KOMB_BUG_ON(ptr->is_local_queue_tail_last);
								if (cmpxchg(&lock->tail, prev_node, NULL) != prev_node) {
										smp_cond_load_relaxed_sched(&prev_node->next, (VAL));
										next_node = prev_node->next;
								}
						}
				} else {
                        KOMB_BUG_ON(current->komb_lock_addr[7] == 0xdeadbeef); //KOMB_BUG_ON(ptr->is_local_queue_tail_last);
						if (*head != NULL) {
								(*tail)->next = prev_node->next;
								next_node = *head; 
						} else {
								next_node = prev_node->next;
						}
				}

#else //NUMA_AWARE
				if (READ_ONCE(prev_node->next) == NULL) {
						if (cmpxchg(lock->tail, prev_node, NULL) != prev_node) {
								smp_cond_load_relaxed_sched(&prev_node->next, (VAL));
								next_node = prev_node->next;
						}
				} else {
						next_node = READ_ONCE(prev_node->next);
				}

#endif //NUMA_AWARE

				if (READ_ONCE(next_node) != NULL) {
						print_debug("Transferring lock to another socket %d cpuid %d\n",
									next_node->socket_id, next_node->cpuid);

						next_node->locked = false;
				} else {
						print_debug("My node next NULL\n");
				}

                wake_up_waiter(prev_node);
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

void kombd_mutex_init(int __num_delegation_threads)
{
		int i, j;
		struct kombd_mutex_node *komb_node;

		for_each_possible_cpu (i) {
				*per_cpu_ptr(&lock_rq_tail, i) = NULL;
		}

#if LOCK_MEASURE_TIME
		*per_cpu_ptr(&do_timing, KOMB_CPU) = true;
#endif

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
				dthreads[i] = kthread_create(komb_md_thread, NULL, "komb_thread");
				kthread_bind(dthreads[i], i * num_cores_per_socket);
				if (dthreads[i])
						wake_up_process(dthreads[i]);
				else
						printk(KERN_ALERT "failed to create komb delegation threads\n");
		}
}

void kombd_mutex_free(void)
{
		int ret;
		uint32_t i;

		for (i = 0; i < num_delegation_threads; i++) {
				ret = kthread_stop(dthreads[i]);
				if (ret)
						printk(KERN_ALERT "komb delegate thread returned error %d\n", ret);
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
static __attribute__((noipa)) noinline notrace void kombd_mutex_lock_slowpath(struct kombd_mutex *lock)
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

		ret_val = __kombd_mutex_lock_slowpath(lock);

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

__attribute__((noipa)) noinline notrace void kombd_mutex_lock(struct kombd_mutex *lock)
{
		struct kombd_mutex_node *curr_node = NULL;
#if WWJUMP
		uint32_t curr_cpuid, prev_cpuid;
		struct kombd_mutex_node *prev_node, *next_node;
#endif
		int ret;

		ret = cmpxchg(&lock->locked, 0, 1);
		if (likely(ret == 0)) {
				/*if (lock->tail != NULL)
			print_debug("Lock stealing\n");*/
				return;
		}

        preempt_disable();

		curr_node = get_kombd_mutex_node(lock);
		KOMB_BUG_ON(curr_node == NULL);
		if ((smp_processor_id() % num_cores_per_socket) == 0) {
				int j = 0;
				for (j = 0; j < 7; j++)
						if (current->komb_lock_addr[j] != NULL)
								break;
				print_debug("lock addr index: %d\n", j);
				KOMB_BUG_ON(true);
		}

#if LOCK_MEASURE_TIME
//		*this_cpu_ptr(&lock_stack_switch) = UINT64_MAX;
//		*this_cpu_ptr(&unlock_stack_switch) = UINT64_MAX;
#endif
		LOCK_START_TIMING_PER_CPU_DISABLE(lock_stack_switch);

		kombd_mutex_lock_slowpath(lock);

#if WWJUMP
		if (current->komb_curr_waiter_task) {
				struct kombd_mutex_node *curr_node =
						((struct task_struct *)current->komb_curr_waiter_task)->komb_mutex_node;

				if ((struct kombd_mutex *)curr_node->lock == lock) {
						KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
						struct kombd_mutex_node *next_node = get_next_node(curr_node);
						if (next_node == NULL)
								current->komb_next_waiter_task = NULL;
						else
								current->komb_next_waiter_task = next_node->task_struct_ptr;
				}

				wake_up_waiter(curr_node);

				if (current->komb_prev_waiter_task) {
						struct kombd_mutex_node *prev_node =
								((struct task_struct *)current->komb_prev_waiter_task)
										->komb_mutex_node;

						KOMB_BUG_ON(prev_node->lock != lock);
						print_debug("Waking up prev waiter: %d\n", prev_node->cpuid);
						wake_up_waiter(prev_node);
						clear_locked_set_completed(prev_node);
						current->komb_prev_waiter_task = NULL;
				}
		}
#endif

        preempt_enable();
}

__attribute__((noipa)) noinline notrace void kombd_mutex_unlock(struct kombd_mutex *lock)
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
				KOMB_BUG_ON(lock->locked != _Q_LOCKED_VAL);
				WRITE_ONCE(lock->locked, 0);
				print_debug("Unlocked the qspinlock\n");
				return;
		}

#if LOCK_MEASURE_TIME
		LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop);
		LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop);
#endif

		//Delegation thread should be on CPU 0 on each socket
		KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);

		curr_node = ((struct task_struct *)current->komb_curr_waiter_task)->komb_mutex_node;

#if WWJUMP
        if (current->komb_next_waiter_task)
		    next_node = ((struct task_struct *)current->komb_next_waiter_task)->komb_mutex_node;
        else
            next_node = NULL;

		counter = current->counter_val;

		if (next_node == NULL || counter >= komb_batch_size || need_resched()) {
				incoming_rsp_ptr = get_shadow_stack_ptr(lock);
				current->komb_prev_waiter_task = current->komb_curr_waiter_task;
				current->komb_curr_waiter_task = NULL;
		} else {
#if NUMA_AWARE
				current->komb_lock_addr[7] = NULL; //ptr->is_local_queue_tail_last = false;
#endif
                current->komb_prev_waiter_task = current->komb_curr_waiter_task;
				current->komb_curr_waiter_task = current->komb_next_waiter_task;
				incoming_rsp_ptr = &(next_node->rsp);
				current->counter_val = counter + 1;
				print_debug("Jumping to the next waiter: %d\n", next_node->cpuid);
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
