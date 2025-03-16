// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi


#include "komb_rwsem.h"

#include <linux/rwsem.h>
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/bug.h>
#include <linux/hardirq.h>
#include <linux/prefetch.h>
#include <linux/atomic.h>
#include <asm/byteorder.h>
#include <linux/vmalloc.h>
#include <linux/sched/stat.h>
#include <linux/sched/task.h>
#include <linux/sched.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>


#ifdef KOMB_STATS
DEFINE_PER_CPU_ALIGNED(uint64_t, rwsem_combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, rwsem_waiter_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, rwsem_reads);
DEFINE_PER_CPU_ALIGNED(uint64_t, rwsem_writes);
DEFINE_PER_CPU_ALIGNED(uint64_t, rwsem_downgrade);
#endif

__attribute__((noipa)) noinline notrace static uint64_t
get_shadow_stack_ptr(struct komb_rwsem *lock)
{
	return &current->komb_stack_curr_ptr;
}

__attribute__((noipa)) noinline notrace static struct komb_mutex_node *
get_komb_mutex_node(struct komb_rwsem *lock)
{
	return ((struct komb_mutex_node *)(current->komb_mutex_node));
}

static __always_inline void set_locked(struct komb_rwsem *lock)
{
	WRITE_ONCE(lock->wlocked, _KOMB_RWSEM_W_LOCKED);
}

__always_inline static void add_to_local_queue(struct komb_mutex_node *node)
{
	struct komb_mutex_node **head, **tail;

	head = (struct komb_mutex_node **)(&current->komb_local_queue_head);
	tail = (struct komb_mutex_node **)(&current->komb_local_queue_tail);

	if (*head == NULL) {
		*head = node;
		*tail = node;
	} else {
		(*tail)->next = node;
		*tail = node;
	}
}

static inline bool check_irq_node(struct komb_mutex_node *node)
{
	return (node->socket_id == IRQ_NUMA_NODE || node->rsp == 0xdeadbeef);
}

__always_inline bool check_rwsem_exit_condition(struct komb_mutex_node *my_node) {
	return (my_node == NULL || check_irq_node(my_node) ||
	 		my_node->next == NULL || check_irq_node(my_node->next));			
}


__always_inline static struct komb_mutex_node *
get_next_node(struct komb_mutex_node *my_node)
{
	struct komb_mutex_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (check_rwsem_exit_condition(next_node))
			goto next_node_null;

		prefetch(next_node->next);

		if (next_node->socket_id == numa_node_id()) {
			void *rsp_ptr = (next_node->rsp);
			prefetchw(rsp_ptr);
			prefetchw(rsp_ptr + 64);
			prefetchw(rsp_ptr + 128);
			prefetchw(rsp_ptr + 192);
			prefetchw(rsp_ptr + 256);
			prefetchw(rsp_ptr + 320);

			return next_node;
		}

		add_to_local_queue(next_node);
		curr_node = next_node;
		next_node = curr_node->next;
	}

next_node_null:
	return next_node;
}

static inline void park_waiter(struct komb_mutex_node *node)
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

static inline void wake_up_waiter(struct komb_mutex_node *node)
{
	u8 old_val = xchg(&node->completed, KOMB_WAITER_PROCESSING);

	if (old_val == KOMB_WAITER_PARKED) {
		struct task_struct *task = node->task_struct_ptr;
		get_task_struct(task);
		wake_up_process(task);
		put_task_struct(task);
	}
}

static __always_inline void clear_locked_set_completed(struct komb_mutex_node *node)
{
	WRITE_ONCE(node->completed, KOMB_WAITER_PROCESSED);
	WRITE_ONCE(node->locked, 0);
}

static inline u64 komb_read_lock_slowpath(struct komb_rwsem *lock)
{
	u64 cnts;
	print_debug("Reader waiting for spinlock\n");
	aqm_lock(&lock->reader_wait_lock);
	cnts = atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS, &lock->cnts);
	print_debug(
		"Reader slowpath got wait lock, waiting for writer to go away\n");
	atomic_long_cond_read_acquire_sched(&lock->cnts,
					    !(VAL & _KOMB_RWSEM_W_WMASK));
	print_debug("Reader slowpath got the lock\n");
	aqm_unlock(&lock->reader_wait_lock);
	return cnts;
}

void komb_down_read(struct komb_rwsem *lock)
{
	u64 cnts;

	cnts = atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS, &lock->cnts);

	if (likely(!(cnts & _KOMB_RWSEM_W_WMASK))) {
		print_debug("Read acquired on fastpath\n");
		goto read_exit; 
	}

	(void)atomic_long_sub_return_release(_KOMB_RWSEM_R_BIAS, &lock->cnts);

	preempt_disable();
	cnts = komb_read_lock_slowpath(lock);
	preempt_enable();

read_exit:
	this_cpu_inc(rwsem_reads);
	return;
}
EXPORT_SYMBOL(komb_down_read);

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
execute_cs(struct komb_rwsem *lock, struct komb_mutex_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	WRITE_ONCE(current->komb_curr_waiter_task, curr_node->task_struct_ptr);
	struct komb_mutex_node *next_node, *my_node;

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &current->komb_stack_curr_ptr;

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	KOMB_BUG_ON(current->komb_stack_base_ptr -
			    (current->komb_stack_curr_ptr) >
		    8192);

	if (lock->wlocked == _KOMB_RWSEM_W_OOO) {
		BUG_ON(true); //TODO: Check OOO

		print_debug("Combiner got control back OOO unlock\n");

#ifdef KOMB_STATS
		// this_cpu_add(rwsem_ooo_waiter_combined, current->counter_val);
		// this_cpu_inc(rwsem_ooo_combiner_count);
#endif

		KOMB_BUG_ON(current->komb_curr_waiter_task == NULL);

		curr_node =
			((struct task_struct *)current->komb_curr_waiter_task)
				->komb_mutex_node;
		my_node = current->komb_mutex_node;

		if (curr_node) {
			print_debug("OOO waking up \n");
			curr_node->rsp = my_node->rsp;
			wake_up_waiter(curr_node);
			clear_locked_set_completed(curr_node);
			KOMB_BUG_ON(current->komb_prev_waiter_task != NULL);
		}
		current->komb_prev_waiter_task = NULL;
		current->komb_curr_waiter_task = NULL;
		lock->wlocked = _KOMB_RWSEM_W_COMBINER;

		next_node = NULL;
		if (current->komb_next_waiter_task)
			next_node = ((struct task_struct *)
					     current->komb_next_waiter_task)
					    ->komb_mutex_node;

		if (next_node && next_node->next &&
		    !check_irq_node(next_node) &&
		    !check_irq_node(next_node->next))
			execute_cs(lock, next_node);
	}
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
run_combiner(struct komb_rwsem *lock, struct komb_mutex_node *curr_node)
{
	struct komb_mutex_node **local_head, **local_tail;
	struct komb_mutex_node *next_node = curr_node->next, *waker = curr_node;
	int counter = 0;

	if (next_node == NULL || check_irq_node(curr_node) ||
	    check_irq_node(next_node)) {
		set_locked(lock);

		wake_up_waiter(curr_node);
		WRITE_ONCE(curr_node->locked, 0);
		return;
	}

	current->counter_val = 0;

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	execute_cs(lock, curr_node);

	print_debug(
		"Combiner got the control back: %d counter: %d prev_waiter: %px next_waiter: %px\n",
		smp_processor_id(), current->counter_val,
		current->komb_prev_waiter_task, current->komb_next_waiter_task);

#ifdef KOMB_STATS
	this_cpu_add(rwsem_waiter_combined, current->counter_val);
	this_cpu_inc(rwsem_combiner_count);
#endif

	if (current->komb_prev_waiter_task) {
		struct komb_mutex_node *prev_node =
			((struct task_struct *)current->komb_prev_waiter_task)
				->komb_mutex_node;
		wake_up_waiter(prev_node);
		clear_locked_set_completed(prev_node);
		current->komb_prev_waiter_task = NULL;
	}

	next_node = NULL;
	if (current->komb_next_waiter_task) {
		next_node =
			((struct task_struct *)current->komb_next_waiter_task)
				->komb_mutex_node;
		current->komb_next_waiter_task = NULL;
	}

	local_head = (struct komb_mutex_node **)(&current->komb_local_queue_head);
	local_tail = (struct komb_mutex_node **)(&current->komb_local_queue_tail);

	if (*local_head) {
		(*local_tail)->next = next_node;
		next_node = *local_head;
		*local_head = NULL;
		*local_tail = NULL;
	}

	set_locked(lock);

	KOMB_BUG_ON(next_node == NULL);

	current->komb_curr_waiter_task = NULL;
	wake_up_waiter(next_node);
	WRITE_ONCE(next_node->locked, 0);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static int
__komb_write_lock_slowpath(register struct komb_rwsem *lock)
{
	struct komb_mutex_node *prev_local_queue_head;
	struct komb_mutex_node *prev_local_queue_tail;
	register struct komb_mutex_node *prev, *next,
		*curr_node = get_komb_mutex_node(lock);
	u8 prev_locked_val;
	int j;

	prev = xchg(&lock->writer_tail, curr_node);
	next = NULL;

	if (prev) {
		WRITE_ONCE(prev->next, curr_node);

		smp_cond_load_relaxed_sleep(curr_node, &curr_node->locked,
					    VAL == 0);

		if (READ_ONCE(curr_node->completed) == KOMB_WAITER_PROCESSED) {
			for (j = 7; j >= 0; j--)
				if (current->komb_lock_addr[j])
					break;

			if (j >= 0) {
				struct komb_rwsem *parent_lock =
					current->komb_lock_addr[j];
				KOMB_BUG_ON(parent_lock == lock);
				if (parent_lock->wlocked == _KOMB_RWSEM_W_OOO) {
					BUG_ON(true); // TODO: Check OOO
					print_debug("Waiter unlocked OOO\n");
					return 1;
				}
			}
			return 0;
		}
	}

	print_debug("Writer owner on slowpath\n");
	aqm_lock(&lock->reader_wait_lock);

	print_debug("Writer got the mutex lock. waiting for pending readers\n");

	if (!atomic_long_read(&lock->cnts) &&
	    (atomic_long_cmpxchg_relaxed(&lock->cnts, 0,
					 _KOMB_RWSEM_W_LOCKED) == 0)) {
		print_debug("No pending readers\n");
		goto unlock;
	}

	atomic_long_add_return_acquire(_KOMB_RWSEM_W_WAITING, &lock->cnts);
	print_debug("Writer set the pending bit\n");
	do {
		atomic_long_cond_read_acquire_sched(
			&lock->cnts, VAL == _KOMB_RWSEM_W_WAITING);
	} while (atomic_long_cmpxchg_relaxed(&lock->cnts, _KOMB_RWSEM_W_WAITING,
					     _KOMB_RWSEM_W_LOCKED) !=
		 _KOMB_RWSEM_W_WAITING);
unlock:
	print_debug("Writer got the lock slowpath\n");
	aqm_unlock(&lock->reader_wait_lock);

	if (cmpxchg(&lock->writer_tail, curr_node, NULL) == curr_node)
		goto release;

	while (!next) {
		next = READ_ONCE(curr_node->next);

		cpu_relax();
		if (need_resched())
			schedule_out_curr_task();
	}

	struct task_struct *prev_curr_waiter_task =
		current->komb_curr_waiter_task;
	current->komb_curr_waiter_task = NULL;

	KOMB_BUG_ON(current->komb_prev_waiter_task != NULL);

	prev_locked_val = lock->wlocked;
	KOMB_BUG_ON(prev_locked_val == _KOMB_RWSEM_W_COMBINER);

	lock->wlocked = _KOMB_RWSEM_W_COMBINER;

	struct task_struct *prev_task_struct_ptr = curr_node->task_struct_ptr;
	uint64_t prev_rsp = curr_node->rsp;
	curr_node->rsp = NULL;
	uint64_t prev_counter_val = current->counter_val;
	current->counter_val = 0;

	struct task_struct *prev_next_waiter_task =
		current->komb_next_waiter_task;
	current->komb_next_waiter_task = NULL;

	uint64_t prev_stack_curr_ptr = current->komb_stack_curr_ptr;

	prev_local_queue_head =
		(struct komb_mutex_node *)current->komb_local_queue_head;
	prev_local_queue_tail =
		(struct komb_mutex_node *)current->komb_local_queue_tail;

	current->komb_local_queue_head = NULL;
	current->komb_local_queue_tail = NULL;

	j = 7;
	for (j = 7; j >= 0; j--)
		if (current->komb_lock_addr[j])
			break;
	j += 1;

	KOMB_BUG_ON(j >= 8 || j < 0);
	current->komb_lock_addr[j] = lock;

	run_combiner(lock, next);

	KOMB_BUG_ON(current->komb_lock_addr[j] != lock);

	current->komb_lock_addr[j] = NULL;

	current->komb_next_waiter_task = prev_next_waiter_task;
	current->counter_val = prev_counter_val;
	current->komb_local_queue_head = prev_local_queue_head;
	current->komb_local_queue_tail = prev_local_queue_tail;
	current->komb_stack_curr_ptr = prev_stack_curr_ptr;
	current->komb_curr_waiter_task = prev_curr_waiter_task;
	curr_node->rsp = prev_rsp;
	curr_node->task_struct_ptr = prev_task_struct_ptr;

	if (lock->wlocked == _KOMB_RWSEM_W_OOO) {
		BUG_ON(true); //TODO: Check OOO
		if (prev_curr_waiter_task) {
			print_debug("Waking up \n");
			wake_up_waiter(
				((struct task_struct *)prev_curr_waiter_task)
					->komb_mutex_node);
			clear_locked_set_completed(
				((struct task_struct *)prev_curr_waiter_task)
					->komb_mutex_node);
		}
		current->komb_curr_waiter_task = NULL;
	}
	WRITE_ONCE(lock->wlocked, prev_locked_val);

release:
	return 0;
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace static int
komb_write_lock_slowpath(struct komb_rwsem *lock)
{
	struct komb_mutex_node *curr_node = get_komb_mutex_node(lock);

	curr_node->locked = true;
	curr_node->completed = KOMB_WAITER_UNPROCESSED;
	curr_node->next = NULL;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->task_struct_ptr = current;
	curr_node->lock = lock;

	smp_wmb();

	return __komb_write_lock_slowpath(lock);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace void
__komb_write_stack_switch(struct komb_rwsem *lock)
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
		       "i"(offsetof(struct komb_mutex_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     "pushq %%rdi\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");

	ret_val = komb_write_lock_slowpath(lock);

	if (ret_val) {
		asm volatile("popq %%rdi\n"
			     "callq %P0\n"
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
		asm volatile("popq %%rdi\n"
			     "callq %P0\n"
			     "movq %%rsp, (%%rax)\n"
			     :
			     : "i"(get_shadow_stack_ptr)
			     : "memory");
		asm volatile("callq %P0\n"
			     "movq %c1(%%rax), %%rsp\n"
			     :
			     : "i"(get_komb_mutex_node),
			       "i"(offsetof(struct komb_mutex_node, rsp))
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

void __down_write(struct komb_rwsem *lock)
{
	preempt_disable();
	__komb_write_stack_switch(lock);

	KOMB_BUG_ON(lock->wlocked == _KOMB_RWSEM_W_COMBINER &&
		    current->komb_curr_waiter_task == NULL);

	if (READ_ONCE(current->komb_curr_waiter_task)) {
		struct komb_mutex_node *curr_node =
			((struct task_struct *)current->komb_curr_waiter_task)
				->komb_mutex_node;

		print_debug("komb-curr-waiter_tsk\n");

		if ((struct komb_rwsem *)curr_node->lock == lock) {
			KOMB_BUG_ON(lock->wlocked != _KOMB_RWSEM_W_COMBINER);
			struct komb_mutex_node *next_node = get_next_node(curr_node);
			print_debug("get_next_node called\n");
			if (next_node == NULL)
				current->komb_next_waiter_task = NULL;
			else
				current->komb_next_waiter_task =
					next_node->task_struct_ptr;
		}

		wake_up_waiter(curr_node);

		if (current->komb_prev_waiter_task) {
			struct komb_mutex_node *prev_node =
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
	preempt_enable();
write_exit:
	this_cpu_inc(rwsem_writes);
	return;
}

void komb_down_write(struct komb_rwsem *lock)
{
	u64 val, cnt;
	val = atomic_long_cmpxchg_acquire(&lock->cnts, 0, _KOMB_RWSEM_W_LOCKED);
	if (val == 0) {
		print_debug("Writer got the lock fastpath\n");
		goto write_exit;
	}

	__down_write(lock);

write_exit:
	this_cpu_inc(rwsem_writes);
	return;
}
EXPORT_SYMBOL(komb_down_write);

void komb_up_read(struct komb_rwsem *lock)
{
	int j, max_idx, my_idx;
	uint64_t temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 8; j++) {
		temp_lock_addr = current->komb_lock_addr[j];
		if (temp_lock_addr)
			max_idx = j;
		if (temp_lock_addr == lock)
			my_idx = j;
		if (temp_lock_addr == NULL)
			break;
	}

	if (my_idx == -1) {
		atomic_long_sub_return_release(_KOMB_RWSEM_R_BIAS, &lock->cnts);
		print_debug("Read lock released\n");
	} else {
		if (my_idx == max_idx) {
			KOMB_BUG_ON(lock->wlocked != _KOMB_RWSEM_W_DOWNGRADE);
			lock->wlocked = _KOMB_RWSEM_W_COMBINER;
			komb_up_write(lock);
		} else {
			BUG_ON(true);
		}
	}
}
EXPORT_SYMBOL(komb_up_read);

__attribute__((noipa)) noinline notrace void komb_up_write(struct komb_rwsem *lock)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct task_struct *curr_task;
	struct komb_mutex_node *curr_node;

	int j, max_idx, my_idx;

	uint64_t temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 8; j++) {
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
			print_debug("Writer releasing on fastpath\n");			
			WRITE_ONCE(lock->wlocked, 0);
		} else if (lock->wlocked == _KOMB_RWSEM_W_COMBINER) {
#ifdef KOMB_STATS
			// this_cpu_inc(rwsem_ooo_unlocks);
#endif
			printk(KERN_ALERT "OOO KOMBD RWSEM not supported\n");
			BUG_ON(true);
			lock->wlocked = _KOMB_RWSEM_W_OOO;
			print_debug("OOO unlock\n");
		} else
			BUG_ON(true);
		return;
	}

	KOMB_BUG_ON(!(lock->wlocked == _KOMB_RWSEM_W_COMBINER ||
		      lock->wlocked == _KOMB_RWSEM_W_DOWNGRADE));
	KOMB_BUG_ON(current->komb_curr_waiter_task == NULL);
	//KOMB_BUG_ON(current->komb_next_waiter_task == NULL);
	KOMB_BUG_ON(max_idx < 0);

	if (my_idx < max_idx) {
#ifdef KOMB_STATS
		// this_cpu_inc(rwsem_ooo_unlocks);
#endif
		printk(KERN_ALERT "OOO KOMBD RWSEM not supported\n");
		BUG_ON(true);
		lock->wlocked = _KOMB_RWSEM_W_OOO;
		return;
	}

	curr_node = ((struct task_struct *)current->komb_curr_waiter_task)
			    ->komb_mutex_node;

	struct komb_mutex_node *next_node = NULL;
	if (current->komb_next_waiter_task)
		next_node =
			((struct task_struct *)current->komb_next_waiter_task)
				->komb_mutex_node;

	uint64_t counter = current->counter_val;

	if (check_rwsem_exit_condition(next_node) ||
	    counter >= komb_batch_size || need_resched()) {
		incoming_rsp_ptr = &(current->komb_stack_curr_ptr);
		current->komb_prev_waiter_task = current->komb_curr_waiter_task;
		current->komb_curr_waiter_task = NULL;

	} else {
		current->komb_prev_waiter_task = current->komb_curr_waiter_task;
		current->komb_curr_waiter_task = current->komb_next_waiter_task;
		current->komb_is_local_queue_tail_last = false;
		incoming_rsp_ptr = &(next_node->rsp);
		current->counter_val = counter + 1;
		print_debug("Jumping to the next waiter: %d\n",
			    next_node->cpuid);
	}

	outgoing_rsp_ptr = &(curr_node->rsp);

	preempt_disable();
	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	preempt_enable();
	return;
}
EXPORT_SYMBOL(komb_up_write);

void komb_init_rwsem(struct komb_rwsem *lock)
{
	atomic_long_set(&lock->cnts, 0);
	atomic_set(&lock->reader_wait_lock.val, 0);
	lock->reader_wait_lock.tail = NULL;
	lock->writer_tail = NULL;
}
EXPORT_SYMBOL(komb_init_rwsem);

int __must_check komb_down_read_killable(struct komb_rwsem *lock)
{
	komb_down_read(lock);
	return 0;
}
EXPORT_SYMBOL(komb_down_read_killable);

int __must_check komb_down_read_interruptible(struct komb_rwsem *lock)
{
	komb_down_read(lock);
	return 0;
}
EXPORT_SYMBOL(komb_down_read_interruptible);

int komb_down_read_trylock(struct komb_rwsem *lock)
{
	u64 cnts =
		atomic_long_add_return_acquire(_KOMB_RWSEM_R_BIAS, &lock->cnts);
	if (likely(!(cnts & _KOMB_RWSEM_W_WMASK))) {
		this_cpu_inc(rwsem_reads);
		print_debug("Reader got the lock\n");
		goto read_exit;
	}
	(void)atomic_long_sub_return_release(_KOMB_RWSEM_R_BIAS, &lock->cnts);

	return 0;

read_exit:
	this_cpu_inc(rwsem_reads);
	return 1;

}
EXPORT_SYMBOL(komb_down_read_trylock);

int __must_check komb_down_write_killable(struct komb_rwsem *lock)
{
	komb_down_write(lock);
	return 0;
}
EXPORT_SYMBOL(komb_down_write_killable);

int komb_down_write_trylock(struct komb_rwsem *lock)
{
	int val = (atomic_long_cmpxchg_acquire(&lock->cnts, 0,
					       _KOMB_RWSEM_W_LOCKED) == 0);

	if (val) {
		this_cpu_inc(rwsem_writes);
	}
	return val;
}
EXPORT_SYMBOL(komb_down_write_trylock);

void komb_downgrade_write(struct komb_rwsem *lock)
{
	int j, max_idx, my_idx;
	uint64_t temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 8; j++) {
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
			print_debug("downgrade to read\n");
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
		lock->wlocked = _KOMB_RWSEM_W_DOWNGRADE;
		return;
	}
	BUG_ON(true);
}
EXPORT_SYMBOL(komb_downgrade_write);
