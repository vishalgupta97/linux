// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <linux/file.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kthread.h>

#include "mutex.h"

static struct task_struct **dthreads;
static int num_delegation_threads = 1;
static int num_cores_per_socket = 1;

static DEFINE_PER_CPU_ALIGNED(struct mutex_node, *mutex_rq_tail);

extern enum system_states system_state;

void park_komb_mutex_thread(void)
{
	struct mutex_node **rq_tail = this_cpu_ptr(&mutex_rq_tail);
	//Park
	__set_current_state(TASK_INTERRUPTIBLE);
	if (READ_ONCE(*rq_tail) == NULL && cmpxchg(rq_tail, NULL, NULL) == NULL)
		schedule_preempt_disabled();
	__set_current_state(TASK_RUNNING);
}

static inline __pure u32 select_delegation_cpu(struct mutex *lock)
{
	return (num_cores_per_socket * numa_node_id());
	// 				+ ((u64)lock % (num_delegation_threads / num_online_nodes())));
}

__attribute__((noipa)) noinline notrace static int
__md_mutex_lock_slowpath(struct mutex *lock)
{
	register struct mutex_node *prev_node, *next_node;
	struct mutex_node **rq_tail;
	struct mutex_node *curr_node = get_komb_mutex_node(lock);

	curr_node->locked = true;
	curr_node->completed = KOMB_WAITER_UNPROCESSED;
	curr_node->next = NULL;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;
	curr_node->lockm = FDS_TDLOCK;

	prev_node = xchg(&lock->tail, curr_node);

	if (prev_node) {
		WRITE_ONCE(prev_node->next, curr_node);
		smp_mb();
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

		rq_tail = per_cpu_ptr(&mutex_rq_tail,
				      select_delegation_cpu(lock));
		if (READ_ONCE(*rq_tail) == 0xdeadbeef ||
		    cmpxchg(rq_tail, NULL, curr_node) != NULL) {
			print_debug("Delegation %d running something else\n",
				    select_delegation_cpu(lock));

			if (cmpxchg(&lock->tail, curr_node, NULL) ==
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
			KOMB_BUG_ON(next_node == NULL);
			print_debug("Next node now head of queue: %d\n",
				    next_node->cpuid);
			wake_up_waiter(next_node);
			WRITE_ONCE(next_node->locked, false);
			goto continue_with_cs_execution;
		} else {
			//KOMB_BUG_ON(!task_is_running(dthreads[numa_node_id()]));
			print_debug("Added to the delegation thread: %d\n",
				    select_delegation_cpu(lock));
			wake_up_process(dthreads[numa_node_id()]);
		}
	}

	smp_cond_load_relaxed_sleep(curr_node, &curr_node->locked, VAL == 0);

	if (READ_ONCE(curr_node->completed) == KOMB_WAITER_PROCESSED) {
		return 0;
	} else {
		print_debug("Head of queue but not completed\n");
		curr_node->locked = true;
		curr_node->completed = false;
		goto head_of_queue;
	}

continue_with_cs_execution:
	return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static __attribute__((noipa)) noinline notrace void
md_mutex_lock_slowpath(struct mutex *lock)
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
		     : "i"(get_komb_mutex_node),
		       "i"(offsetof(struct mutex_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     :
		     : "i"(mutex_get_shadow_stack_ptr)
		     : "memory");

	ret_val = __md_mutex_lock_slowpath(lock);

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
			     : "i"(mutex_get_shadow_stack_ptr)
			     : "memory");
	} else {
		asm volatile("callq %P0\n"
			     "movq %%rsp, (%%rax)\n"
			     :
			     : "i"(mutex_get_shadow_stack_ptr)
			     : "memory");
		asm volatile("callq %P0\n"
			     "movq %c1(%%rax), %%rsp\n"
			     :
			     : "i"(get_komb_mutex_node),
			       "i"(offsetof(struct mutex_node, rsp))
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

__attribute__((noipa)) noinline notrace void md_mutex_lock(struct mutex *lock)
{
	if (((smp_processor_id() % num_cores_per_socket) == 0)) {
		__mutex_lock(lock, FDS_QSPINLOCK);
		return;
	} else {
		print_debug("Going on slowpath\n");

		preempt_disable();
		md_mutex_lock_slowpath(lock);

		if (current->komb_curr_waiter_task) {
			struct mutex_node *curr_node =
				((struct task_struct *)
					 current->komb_curr_waiter_task)
					->komb_mutex_node;

			if ((struct mutex *)curr_node->lock == lock) {
				KOMB_BUG_ON(lock->locked !=
					    _Q_LOCKED_COMBINER_VAL);
				struct mutex_node *next_node =
					mutex_get_next_node(curr_node);
				if (next_node == NULL)
					current->komb_next_waiter_task = NULL;
				else
					current->komb_next_waiter_task =
						next_node->task_struct_ptr;
			}

			wake_up_waiter(curr_node);

			if (current->komb_prev_waiter_task) {
				struct mutex_node *prev_node =
					((struct task_struct *)
						 current->komb_prev_waiter_task)
						->komb_mutex_node;

				KOMB_BUG_ON(prev_node->lock != lock);
				print_debug("Waking up prev waiter: %d\n",
					    prev_node->cpuid);
				wake_up_waiter(prev_node);
				mutex_clear_locked_set_completed(prev_node);
				current->komb_prev_waiter_task = NULL;
			}
		}
		preempt_enable();
		mutex_stat_lock_acquire(lock->key);
	}
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static struct mutex_node *
tdlock_mutex_run_combiner(struct mutex *lock, struct mutex_node *curr_node)
{
	int j;

	KOMB_BUG_ON(curr_node == NULL);
	KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);

	WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
	current->counter_val = 0;

	BUG_ON(current->komb_curr_waiter_task !=
	       NULL); //TODO: Needed only for nested combining

	current->komb_curr_waiter_task = NULL;
	current->komb_next_waiter_task = NULL;
	current->komb_local_queue_head = NULL;
	current->komb_local_queue_tail = NULL;
	current->komb_is_local_queue_tail_last = false;

	j = 0;
	for (j = 0; j < 8; j++)
		if (current->komb_lock_addr[j] == NULL)
			break;

	KOMB_BUG_ON(j >= 8);

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	mutex_execute_cs(lock, curr_node);

	KOMB_BUG_ON(current->komb_lock_addr[j] != lock);
	current->komb_lock_addr[j] = NULL;

	print_debug(
		"Combiner got the control back: %d counter: %lld last_waiter: \n",
		smp_processor_id(), current->counter_val);

#if KOMB_STATS
	this_cpu_add(waiter_combined, current->counter_val);
	this_cpu_inc(combiner_count);
#endif

	KOMB_BUG_ON(current->komb_prev_waiter_task == NULL);
	current->komb_curr_waiter_task = NULL;

	return current->komb_prev_waiter_task;
}
#pragma GCC pop_options

int komb_mutex_thread(void *args)
{
	struct mutex *lock;
	struct mutex_node *prev_node, *next_node;
	struct mutex_node **local_head, **local_tail;
	struct mutex_node **rq_tail;

	rq_tail = this_cpu_ptr(&mutex_rq_tail);
	lock = NULL;

	while (true) {
		if (READ_ONCE(*rq_tail) != 0xdeadbeef)
			break;
		cpu_relax();
		cond_resched();
	}

	preempt_disable();

	while (true) {
		print_debug("komb mutex thread waiting for lock\n");
		smp_cond_load_relaxed_sched_delegation(rq_tail, (VAL));

		next_node = *rq_tail;
		KOMB_BUG_ON(next_node == NULL);
		//KOMB_BUG_ON(next_node->cpuid == smp_processor_id());
		lock = next_node->lock;
		//Lock should already be acquired.
		KOMB_BUG_ON(lock->locked == 0);
		print_debug("Running combiner with node from: %d\n",
			    next_node->cpuid);

		prev_node = tdlock_mutex_run_combiner(lock, next_node);
		next_node = NULL;
		WRITE_ONCE(*rq_tail, NULL); //Combining done

		local_head =
			(struct mutex_node **)(&current->komb_local_queue_head);
		local_tail =
			(struct mutex_node **)(&current->komb_local_queue_tail);

		//lock = prev_node->lock; //TODO: Check why this is needed.
		if (READ_ONCE(prev_node->next) == NULL) {
			if (*local_head) {
				if (!current->komb_is_local_queue_tail_last) {
					(*local_tail)->next = NULL;
					if (cmpxchg(&lock->tail, prev_node,
						    *local_tail) != prev_node) {
						next_node = READ_ONCE(
							prev_node->next);
						while (!next_node) {
							next_node = READ_ONCE(
								prev_node->next);

							cpu_relax();
							if (need_resched())
								schedule_out_curr_task();
						}
						(*local_tail)->next =
							prev_node->next;
					}
				}
				next_node = (*local_head);
			} else {
				KOMB_BUG_ON(
					current->komb_is_local_queue_tail_last);
				if (cmpxchg(&lock->tail, prev_node, NULL) !=
				    prev_node) {
					next_node = READ_ONCE(prev_node->next);
					while (!next_node) {
						next_node = READ_ONCE(
							prev_node->next);

						cpu_relax();
						if (need_resched())
							schedule_out_curr_task();
					}
					next_node = prev_node->next;
				}
			}
		} else {
			if (*local_head) {
				KOMB_BUG_ON(
					current->komb_is_local_queue_tail_last);
				(*local_tail)->next = prev_node->next;
				next_node = *local_head;
			} else {
				KOMB_BUG_ON(
					current->komb_is_local_queue_tail_last);
				next_node = prev_node->next;
			}
		}

		if (next_node != NULL) {
			wake_up_waiter(next_node);
			WRITE_ONCE(next_node->locked, 0);
		}

		wake_up_waiter(prev_node);
		mutex_clear_locked_set_completed(prev_node);

		KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
		print_debug("Releasing the lock from combiner\n");
		WRITE_ONCE(lock->locked, 0);
	}

	BUG_ON(true);
	preempt_enable();

	return 0;
}

static int __init md_init(void)
{
	int i;

	for_each_possible_cpu(i) {
		*per_cpu_ptr(&mutex_rq_tail, i) = 0;
	}

	num_delegation_threads = num_online_nodes();
	num_cores_per_socket = num_online_cpus() / num_online_nodes();

	printk(KERN_ALERT "======== KOMB MUTEX starting delegation ========\n");

	dthreads =
		vzalloc(num_delegation_threads * sizeof(struct task_struct *));
	for (i = 0; i < num_delegation_threads; i++) {
		dthreads[i] = kthread_create(komb_mutex_thread, NULL,
					     "komb_mutex_thread");
		kthread_bind(dthreads[i], i * num_cores_per_socket);
		if (dthreads[i])
			wake_up_process(dthreads[i]);
		else
			printk(KERN_ALERT
			       "failed to create komb mutex delegation threads\n");
	}

	printk(KERN_ALERT "Created mutex delegation threads\n");

	return 0;
}

module_init(md_init)
