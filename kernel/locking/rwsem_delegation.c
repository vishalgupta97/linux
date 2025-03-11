// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include "rwsem.h"

static struct task_struct **dthreads;
static int num_rwsemd_threads = 1;
static int rwsemd_num_cores_per_socket = 1;

static DEFINE_PER_CPU_ALIGNED(struct mutex_node, *rwsem_rq_tail);

void park_komb_rwsem_thread(void)
{
	struct mutex_node **rq_tail = this_cpu_ptr(&rwsem_rq_tail);
	//Park
	__set_current_state(TASK_INTERRUPTIBLE);
	if (READ_ONCE(*rq_tail) == NULL && cmpxchg(rq_tail, NULL, NULL) == NULL) {
		schedule_preempt_disabled();
		//schedule_out_curr_task();
	}
	__set_current_state(TASK_RUNNING);
}

static inline void park_waiter(struct mutex_node *node)
{
	__set_current_state(TASK_INTERRUPTIBLE);

	if (cmpxchg(&node->completed, KOMB_WAITER_UNPROCESSED,
		    KOMB_WAITER_PARKED) != KOMB_WAITER_UNPROCESSED) {
		__set_current_state(TASK_RUNNING);
		return;
	}
	schedule_preempt_disabled();
	//schedule_out_curr_task();
	__set_current_state(TASK_RUNNING);
}

static inline void wake_up_waiter(struct mutex_node *node)
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
rwsemd_get_shadow_stack_ptr(struct rw_semaphore *lock)
{
	return &current->komb_stack_curr_ptr;
}

__attribute__((noipa)) noinline notrace static struct mutex_node *
rwsemd_get_mutex_node(struct rw_semaphore *lock)
{
	return ((struct mutex_node *)(current->komb_mutex_node));
}

static __always_inline void
rwsemd_clear_locked_set_completed(struct mutex_node *node)
{
	WRITE_ONCE(node->completed, KOMB_WAITER_PROCESSED);
	WRITE_ONCE(node->locked, 0);
}

__always_inline static void add_to_local_queue(struct mutex_node *node)
{
	struct mutex_node **head, **tail;

	head = (struct mutex_node **)(&current->komb_local_queue_head);
	tail = (struct mutex_node **)(&current->komb_local_queue_tail);

	if (*head == NULL) {
		*head = node;
		*tail = node;
	} else {
		(*tail)->next = node;
		*tail = node;
	}

	current->komb_is_local_queue_tail_last = true;
}

static __always_inline struct mutex_node *
get_next_node(struct mutex_node *my_node)
{
	struct mutex_node *curr_node, *next_node;
	int i;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->socket_id == IRQ_NUMA_NODE)
			goto next_node_null;
	
		prefetch(next_node->next);

		if (next_node->socket_id == numa_node_id()) {
			void *rsp_ptr = next_node->rsp;
			prefetchw(rsp_ptr);
			for (i = 1; i < KOMBD_NUM_PREFETCH_LINES; i++)
				prefetchw(rsp_ptr + (64 * i));
			return next_node;
		}

		curr_node->next = NULL;
		add_to_local_queue(next_node);
		curr_node = next_node;
		next_node = curr_node->next;
	}

next_node_null:
	return NULL;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
rwsemd_execute_cs(struct mutex_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;

	WRITE_ONCE(current->komb_curr_waiter_task, curr_node->task_struct_ptr);
	KOMB_BUG_ON(curr_node->cpuid == smp_processor_id());

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = rwsemd_get_shadow_stack_ptr(NULL);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
rwsemd_run_combiner(struct rw_semaphore *lock, struct mutex_node *curr_node)
{
	KOMB_BUG_ON(curr_node == NULL);
	KOMB_BUG_ON((smp_processor_id() % rwsemd_num_cores_per_socket) != 0);

	current->komb_is_local_queue_tail_last = false;

	current->counter_val = 0;
	rwsemd_execute_cs(curr_node);

#if KOMB_STATS
	this_cpu_add(waiter_combined, current->counter_val);
	this_cpu_inc(combiner_count);
	//printk(KERN_ALERT "KOMB RWSEMD combiner_count: %d\n", combiner_count);
#endif

	KOMB_BUG_ON(current->komb_prev_waiter_task == NULL);
	current->komb_curr_waiter_task = NULL;
}
#pragma GCC pop_options

static inline __pure u32 select_delegation_cpu(struct rw_semaphore *lock)
{
	return (rwsemd_num_cores_per_socket * numa_node_id());
	//		       	+((u64)lock % num_rwsemd_threads_per_socket));
}

__attribute__((noipa)) noinline notrace static int
__kombd_write_lock_slowpath(struct rw_semaphore *lock)
{
	struct mutex_node *curr_node, *next_node;
	struct mutex_node **rq_tail;
	register struct mutex_node *prev_node;
	u32 i;

	curr_node = rwsemd_get_mutex_node(lock);

	curr_node->locked = true;
	curr_node->completed = KOMB_WAITER_UNPROCESSED;
	curr_node->next = NULL;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;
	curr_node->lockm = FDS_TDLOCK;

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

		wait_for_visible_readers(lock);

		rq_tail =
			per_cpu_ptr(&rwsem_rq_tail, select_delegation_cpu(lock));
		if (READ_ONCE(*rq_tail) == 0xdeadbeef ||
		    cmpxchg(rq_tail, NULL, curr_node) != NULL) {
#if KOMB_STATS
			//this_cpu_inc(rwsem_qspinlock_fallback);
#endif
			
			BUG_ON(true);
	
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
			wake_up_process(dthreads[numa_node_id()]);
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
	for (i = 0; i < KOMBD_NUM_PREFETCH_LINES; i++)
		prefetchw(((char *)curr_node->rsp) + (64 * i));

	return 0;
}

int komb_rwsemd_thread(void *args)
{
	struct rw_semaphore *lock;
	struct mutex_node *prev_node, *next_node;
	struct mutex_node **rq_tail;
	struct mutex_node **head, **tail;
	int j;

	rq_tail = this_cpu_ptr(&rwsem_rq_tail);
	lock = NULL;

	while (true) {
		if (READ_ONCE(*rq_tail) != 0xdeadbeef)
			break;
		cpu_relax();
		cond_resched();
	}

	preempt_disable();

	while (true) {
		smp_cond_load_relaxed_sched_delegation(rq_tail, (VAL));

		next_node = *rq_tail;
		KOMB_BUG_ON(next_node == NULL);
		KOMB_BUG_ON(next_node->cpuid == smp_processor_id());
		lock = next_node->lock;
		KOMB_BUG_ON(lock->wlocked ==
			    0); //Lock should already be acquired.
		lock->wlocked = _KOMB_RWSEM_W_COMBINER;
		print_debug("Running combiner with node from: %d\n",
			    next_node->cpuid);

		BUG_ON(current->mm != NULL); //For mmap_lock
		current->mm = next_node->task_struct_ptr->mm;

		j = 0;
		for (j = 0; j < 8; j++)
			if (current->komb_lock_addr[j] == NULL)
				break;

		//TODO: Update this condition nested delegation
		KOMB_BUG_ON(j != 0);

		current->komb_lock_addr[j] = lock;
		current->komb_local_queue_head = NULL;
		current->komb_local_queue_tail = NULL;
		current->komb_is_local_queue_tail_last = false;
		current->komb_curr_waiter_task = NULL;
		current->komb_prev_waiter_task = NULL;
		current->komb_next_waiter_task = NULL;
		current->counter_val = 0;

		rwsemd_run_combiner(lock, next_node);
	
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

		current->mm = NULL;

		head = (struct mutex_node *
				*)(&current->komb_local_queue_head);
		tail = (struct mutex_node *
				*)(&current->komb_local_queue_tail);

		if (READ_ONCE(prev_node->next) == NULL) {
			if (*head != NULL) {
				if (!current->komb_is_local_queue_tail_last) {
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
					current->komb_is_local_queue_tail_last);
				if (cmpxchg(&lock->writer_tail, prev_node,
					    NULL) != prev_node) {
					smp_cond_load_relaxed_sched(
						&prev_node->next, (VAL));
					next_node = prev_node->next;
				}
			}
		} else {
			KOMB_BUG_ON(current->komb_is_local_queue_tail_last);
			if (*head != NULL) {
				(*tail)->next = prev_node->next;
				next_node = *head;
			} else {
				next_node = prev_node->next;
			}
		}

		if (READ_ONCE(next_node) != NULL) {
			print_debug(
				"Transferring lock to another socket %d cpuid %d\n",
				next_node->socket_id, next_node->cpuid);

			wake_up_waiter(next_node);
			next_node->locked = false;
		} else {
			print_debug("My node next NULL\n");
		}

		wake_up_waiter(prev_node);
		rwsemd_clear_locked_set_completed(prev_node);

		//Release the lock
		KOMB_BUG_ON(!(lock->wlocked == _KOMB_RWSEM_W_COMBINER ||
			      lock->wlocked == _KOMB_RWSEM_W_DOWNGRADE));
		print_debug("Releasing the lock from combiner\n");
		WRITE_ONCE(lock->wlocked, 0);
	}

	BUG_ON(true);
	preempt_enable();

	return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static __attribute__((noipa)) noinline notrace void
kombd_write_lock_slowpath(struct rw_semaphore *lock)
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
		     : "i"(rwsemd_get_mutex_node),
		       "i"(offsetof(struct mutex_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     :
		     : "i"(rwsemd_get_shadow_stack_ptr)
		     : "memory");

	ret_val = __kombd_write_lock_slowpath(lock);

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
			     : "i"(rwsemd_get_shadow_stack_ptr)
			     : "memory");
	} else {
		asm volatile("callq %P0\n"
			     "movq %%rsp, (%%rax)\n"
			     :
			     : "i"(rwsemd_get_shadow_stack_ptr)
			     : "memory");
		asm volatile("callq %P0\n"
			     "movq %c1(%%rax), %%rsp\n"
			     :
			     : "i"(rwsemd_get_mutex_node),
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

void komb_rwsemd_down_write(struct rw_semaphore *lock)
{
	struct mutex_node *curr_node = NULL;
	u64 val;
	val = atomic_long_cmpxchg_acquire(&lock->cnts, 0, _KOMB_RWSEM_W_LOCKED);
	if (val == 0) {
		wait_for_visible_readers(lock);
		return;
	}

	curr_node = rwsemd_get_mutex_node(lock);
	KOMB_BUG_ON(curr_node == NULL);

	preempt_disable();
	kombd_write_lock_slowpath(lock);

	if (current->komb_curr_waiter_task) {
		struct mutex_node *curr_node =
			((struct task_struct *)current->komb_curr_waiter_task)
				->komb_mutex_node;

		if ((struct rw_semaphore *)curr_node->lock == lock) {
			KOMB_BUG_ON(
				!(lock->wlocked == _KOMB_RWSEM_W_COMBINER ||
				  lock->wlocked == _KOMB_RWSEM_W_DOWNGRADE));
			struct mutex_node *next_node =
				get_next_node(curr_node);
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
			rwsemd_clear_locked_set_completed(prev_node);
			current->komb_prev_waiter_task = NULL;
		}
	}
	
	preempt_enable();
}

static int __init rwsemd_init(void)
{
	int i;

	for_each_possible_cpu(i) {
		*per_cpu_ptr(&rwsem_rq_tail, i) = 0;
	}

	num_rwsemd_threads = num_online_nodes();
	rwsemd_num_cores_per_socket = num_online_cpus() / num_online_nodes();

	printk(KERN_ALERT "======== KOMB RWSEM starting delegation ========\n");

	dthreads =
		vzalloc(num_rwsemd_threads * sizeof(struct task_struct *));
	for (i = 0; i < num_rwsemd_threads; i++) {
		dthreads[i] = kthread_create(komb_rwsemd_thread, NULL,
					     "komb_rwsem_thread");
		kthread_bind(dthreads[i], i * rwsemd_num_cores_per_socket);
		if (dthreads[i])
			wake_up_process(dthreads[i]);
		else
			printk(KERN_ALERT
			       "failed to create komb rwsem delegation threads\n");
	}

	printk(KERN_ALERT "Created rwsem delegation threads\n");

	return 0;
}

module_init(rwsemd_init)
