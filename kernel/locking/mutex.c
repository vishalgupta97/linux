// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <linux/mutex.h>
#include <linux/sched/signal.h>
#include <linux/sched/rt.h>
#include <linux/sched/wake_q.h>
#include <linux/sched/debug.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/debug_locks.h>
#include <linux/osq_lock.h>

#include <linux/module.h>
#include <linux/smp.h>
#include <linux/bug.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/prefetch.h>
#include <linux/atomic.h>
#include <asm/byteorder.h>
#include <linux/vmalloc.h>
#include <linux/sched/stat.h>
#include <linux/sched/task.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>

#include "mutex.h"

#define CREATE_TRACE_POINTS
#include <trace/events/lock.h>

//#define DSM_DEBUG 1
#ifdef DSM_DEBUG
#define print_debug(fmt, ...)                                            \
	({                                                               \
		printk(KERN_ALERT "[%d] [%d] komb (%s) lock(%px): " fmt, \
		       smp_processor_id(), current->pid, __func__, lock, \
		       ##__VA_ARGS__);                                   \
	})
#else
#define print_debug(fmt, ...)
#endif

//#define DEBUG_KOMB 1
#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

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

#ifndef smp_cond_load_relaxed_sched
#define smp_cond_load_relaxed_sched(ptr, cond_expr)       \
	({                                                \
		typeof(ptr) __PTR = (ptr);                \
		__unqual_scalar_typeof(*ptr) VAL;         \
		for (;;) {                                \
			VAL = READ_ONCE(*__PTR);          \
			if (cond_expr)                    \
				break;                    \
			cpu_relax();                      \
			if (need_resched()) {             \
				schedule_out_curr_task(); \
			}                                 \
		}                                         \
		(typeof(*ptr))VAL;                        \
	})
#endif

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                 \
	({                                                          \
		__unqual_scalar_typeof(*ptr) _val;                  \
		_val = smp_cond_load_relaxed_sched(ptr, cond_expr); \
		smp_acquire__after_ctrl_dep();                      \
		(typeof(*ptr))_val;                                 \
	})
#endif

#define UINT64_MAX 0xffffffffffffffffL

#ifdef KOMB_STATS
DEFINE_PER_CPU_ALIGNED(uint64_t, mutex_combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, mutex_waiter_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, mutex_qspinlock);
DEFINE_PER_CPU_ALIGNED(uint64_t, mutex_tclock);
DEFINE_PER_CPU_ALIGNED(uint64_t, mutex_ooo_unlocks);
#endif

#define _Q_LOCKED_COMBINER_VAL 3
#define _Q_UNLOCKED_OOO_VAL 7 //Unlocked a lock out-of-order

#define KOMB_WAITER_UNPROCESSED 0
#define KOMB_WAITER_PARKED 1
#define KOMB_WAITER_PROCESSING 2
#define KOMB_WAITER_PROCESSED 4

static inline void schedule_out_curr_task(void)
{
	// preempt_enable();
	// schedule();
	// preempt_disable();
	__set_current_state(TASK_RUNNING);
	schedule_preempt_disabled();
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

static __always_inline void clear_locked_set_completed(struct mutex_node *node)
{
	WRITE_ONCE(node->completed, KOMB_WAITER_PROCESSED);
	WRITE_ONCE(node->locked, 0);
}

static __always_inline void set_locked(struct mutex *lock)
{
	WRITE_ONCE(lock->locked, 1);
}

__attribute__((noipa)) noinline notrace static uint64_t
get_shadow_stack_ptr(struct mutex *lock)
{
	return &current->komb_stack_curr_ptr;
}

__attribute__((noipa)) noinline notrace static struct mutex_node *
get_komb_mutex_node(struct mutex *lock)
{
	return ((struct mutex_node *)(current->komb_mutex_node));
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
}

static inline bool check_irq_node(struct mutex_node *node)
{
	return (node->socket_id == IRQ_NUMA_NODE || node->rsp == 0xdeadbeef);
}

__always_inline static struct mutex_node *
get_next_node(struct mutex_node *my_node)
{
	struct mutex_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->next == NULL)
			goto next_node_null;
		else if (check_irq_node(next_node) ||
			 check_irq_node(next_node->next))
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

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
execute_cs(struct mutex *lock, struct mutex_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct mutex_node *my_node, *next_node;

	WRITE_ONCE(current->komb_curr_waiter_task, curr_node->task_struct_ptr);

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = get_shadow_stack_ptr(lock);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

	KOMB_BUG_ON(current->komb_stack_base_ptr -
			    (current->komb_stack_curr_ptr) >
		    8192);

	if (lock->locked == _Q_UNLOCKED_OOO_VAL) {
		print_debug("Combiner got control back OOO unlock\n");

		KOMB_BUG_ON(current->komb_curr_waiter_task == NULL);

		curr_node =
			((struct task_struct *)current->komb_curr_waiter_task)
				->komb_mutex_node;
		my_node = current->komb_mutex_node;

		if (curr_node) {
			print_debug("OOO waking up\n");
			curr_node->rsp = my_node->rsp;
			wake_up_waiter(curr_node);
			clear_locked_set_completed(curr_node);
			KOMB_BUG_ON(current->komb_prev_waiter_task != NULL);
		}
		current->komb_prev_waiter_task = NULL;
		current->komb_curr_waiter_task = NULL;
		lock->locked = _Q_LOCKED_COMBINER_VAL;

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
run_combiner(struct mutex *lock, struct mutex_node *curr_node)
{
	KOMB_BUG_ON(curr_node == NULL);

	struct mutex_node **local_head, **local_tail;
	struct mutex_node *next_node = curr_node->next;
	int counter = 0;

	if (next_node == NULL || check_irq_node(curr_node) ||
	    check_irq_node(next_node)) {
		set_locked(lock);
		wake_up_waiter(curr_node);
		WRITE_ONCE(curr_node->locked, 0);
		return;
	}

	WRITE_ONCE(lock->combiner_task, current);
	current->counter_val = 0;

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	execute_cs(lock, curr_node);

	print_debug(
		"Combiner got the control back: %d counter: %d last_waiter: %d\n",
		smp_processor_id(), current->counter_val,
		current->komb_curr_waiter_task);

#ifdef KOMB_STATS
	this_cpu_add(mutex_waiter_combined, current->counter_val);
	this_cpu_inc(mutex_combiner_count);
#endif

	if (current->komb_prev_waiter_task) {
		struct mutex_node *prev_node =
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

	WRITE_ONCE(lock->combiner_task, NULL);
	local_head = (struct mutex_node **)(&current->komb_local_queue_head);
	local_tail = (struct mutex_node **)(&current->komb_local_queue_tail);

	if (*local_head) {
		(*local_tail)->next = next_node;
		next_node = *local_head;
		*local_head = NULL;
		*local_tail = NULL;
	}

	set_locked(lock);
	KOMB_BUG_ON(next_node == NULL);

	wake_up_waiter(next_node);
	WRITE_ONCE(next_node->locked, 0);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static int
__komb_mutex_lock_slowpath(struct mutex *lock,
			   register struct mutex_node *curr_node)
{
	register struct mutex_node *prev, *next;
	u8 prev_locked_val;
	int j;
	struct mutex *parent_lock;
	struct mutex_node *prev_local_queue_head;
	struct mutex_node *prev_local_queue_tail;

	prev = xchg(&lock->tail, curr_node);
	next = NULL;

	if (prev) {
		WRITE_ONCE(prev->next, curr_node);

		smp_mb();

		smp_cond_load_relaxed_sleep(curr_node, &curr_node->locked,
					    VAL == 0);

		if (READ_ONCE(curr_node->completed) == KOMB_WAITER_PROCESSED) {
			for (j = 7; j >= 0; j--)
				if (current->komb_lock_addr[j])
					break;

			if (j >= 0) {
				parent_lock = current->komb_lock_addr[j];
				KOMB_BUG_ON(parent_lock == lock);
				if (parent_lock->locked ==
				    _Q_UNLOCKED_OOO_VAL) {
					print_debug("Waiter unlocked OOO\n");
					return 1;
				}
			}
			return 0;
		}
	}

	for (;;) {
		while (READ_ONCE(lock->locked)) {
			cpu_relax();
			if (need_resched())
				schedule_out_curr_task();
		}

		if (cmpxchg(&lock->locked, 0, 1) == 0)
			break;
	}

	if (cmpxchg(&lock->tail, curr_node, NULL) == curr_node)
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

	prev_locked_val = lock->locked;
	KOMB_BUG_ON(prev_locked_val >= _Q_LOCKED_COMBINER_VAL);

	lock->locked = _Q_LOCKED_COMBINER_VAL;

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
		(struct mutex_node *)current->komb_local_queue_head;
	prev_local_queue_tail =
		(struct mutex_node *)current->komb_local_queue_tail;

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

	if (lock->locked == _Q_UNLOCKED_OOO_VAL) {
		if (prev_curr_waiter_task) {
			print_debug("Waking up OOO\n");
			wake_up_waiter(
				((struct task_struct *)prev_curr_waiter_task)
					->komb_mutex_node);
			clear_locked_set_completed(
				((struct task_struct *)prev_curr_waiter_task)
					->komb_mutex_node);
		}
		current->komb_curr_waiter_task = NULL;
	}
	lock->locked = prev_locked_val;

release:
	return 0;
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace static int
komb_mutex_lock_slowpath(struct mutex *lock)
{
	struct mutex_node *curr_node = get_komb_mutex_node(lock);

	curr_node->locked = true;
	curr_node->completed = KOMB_WAITER_UNPROCESSED;
	curr_node->next = NULL;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->task_struct_ptr = current;
	curr_node->lock = lock;

	return __komb_mutex_lock_slowpath(lock, curr_node);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace void
__komb_mutex_lock(struct mutex *lock)
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
		     "pushq %%rdi\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");

	ret_val = komb_mutex_lock_slowpath(lock);

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

__attribute__((noipa)) noinline notrace void mutex_lock(struct mutex *lock)
{
	int ret;

	ret = cmpxchg(&lock->locked, 0, 1);
	if (likely(ret == 0)) {
		// mutex_stat_lock_acquire(&lock->key);
		return;
	}

	might_sleep();

	if (lock->key.ptr == NULL || lock->key.ptr->lockm == FDS_QSPINLOCK) {
		//if (true) {
		preempt_disable();
		this_cpu_inc(mutex_qspinlock);

		struct mutex_node *prev, *next;
		struct mutex_node *curr_node = get_komb_mutex_node(lock);

		curr_node->locked = true;
		curr_node->completed = KOMB_WAITER_UNPROCESSED;
		curr_node->next = NULL;
		curr_node->socket_id = IRQ_NUMA_NODE;
		curr_node->cpuid = smp_processor_id();
		curr_node->task_struct_ptr = current;
		curr_node->lock = lock;

		prev = xchg(&lock->tail, curr_node);
		next = NULL;

		if (prev) {
			WRITE_ONCE(prev->next, curr_node);
			smp_mb();
			smp_cond_load_relaxed_sleep(
				curr_node, &curr_node->locked, VAL == 0);
			KOMB_BUG_ON(READ_ONCE(curr_node->completed) ==
				    KOMB_WAITER_PROCESSED);
		}

		for (;;) {
			while (READ_ONCE(lock->locked)) {
				cpu_relax();
				if (need_resched())
					schedule_out_curr_task();
			}

			if (cmpxchg(&lock->locked, 0, 1) == 0)
				break;
		}

		if (cmpxchg(&lock->tail, curr_node, NULL) == curr_node)
			goto irq_release;

		while (!next) {
			next = READ_ONCE(curr_node->next);

			cpu_relax();
			if (need_resched())
				schedule_out_curr_task();
		}
		wake_up_waiter(next);
		WRITE_ONCE(next->locked, 0);
irq_release:
		preempt_enable();
		goto write_exit;
	}

	preempt_disable();
	this_cpu_inc(mutex_tclock);
	trace_contention_begin(lock, LCB_F_MUTEX);
	__komb_mutex_lock(lock);

	if (current->komb_curr_waiter_task) {
		struct mutex_node *curr_node =
			((struct task_struct *)current->komb_curr_waiter_task)
				->komb_mutex_node;

		if ((struct mutex *)curr_node->lock == lock) {
			KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
			struct mutex_node *next_node = get_next_node(curr_node);
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
			clear_locked_set_completed(prev_node);
			current->komb_prev_waiter_task = NULL;
		}
	}
	trace_contention_end(lock, 0);
	preempt_enable();
write_exit:
	mutex_stat_lock_acquire(&lock->key);
}
EXPORT_SYMBOL(mutex_lock);

int __must_check mutex_lock_interruptible(struct mutex *lock)
{
	mutex_lock(lock);
	return 0;
}
EXPORT_SYMBOL(mutex_lock_interruptible);

int __must_check mutex_lock_killable(struct mutex *lock)
{
	mutex_lock(lock);
	return 0;
}
EXPORT_SYMBOL(mutex_lock_killable);

void mutex_lock_io(struct mutex *lock)
{
	int token;

	token = io_schedule_prepare();
	mutex_lock(lock);
	io_schedule_finish(token);
}
EXPORT_SYMBOL_GPL(mutex_lock_io);

bool mutex_is_locked(struct mutex *lock)
{
	return lock->locked != 0;
}
EXPORT_SYMBOL(mutex_is_locked);

int mutex_trylock(struct mutex *lock)
{
	if (!lock->locked && cmpxchg(&lock->locked, 0, 1) == 0) {
		mutex_stat_lock_acquire(&lock->key);
		return 1;
	}

	return 0;
}
EXPORT_SYMBOL(mutex_trylock);

__attribute__((noipa)) noinline notrace void mutex_unlock(struct mutex *lock)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct task_struct *curr_task;
	struct mutex_node *curr_node;

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
		if (lock->locked == _Q_LOCKED_VAL)
			lock->locked = false;
		else if (lock->locked == _Q_LOCKED_COMBINER_VAL) {
#ifdef KOMB_STATS
			this_cpu_inc(mutex_ooo_unlocks);
#endif

			lock->locked = _Q_UNLOCKED_OOO_VAL;
			print_debug("OOO unlock\n");
		} else
			BUG_ON(true);
		return;
	}

	KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
	KOMB_BUG_ON(current->komb_curr_waiter_task == NULL);
	KOMB_BUG_ON(max_idx < 0);

	if (my_idx < max_idx) {
#ifdef KOMB_STATS
		this_cpu_inc(mutex_ooo_unlocks);
#endif
		lock->locked = _Q_UNLOCKED_OOO_VAL;
		return;
	}

	curr_node = ((struct task_struct *)current->komb_curr_waiter_task)
			    ->komb_mutex_node;

	struct mutex_node *next_node = NULL;
	if (current->komb_next_waiter_task)
		next_node =
			((struct task_struct *)current->komb_next_waiter_task)
				->komb_mutex_node;

	uint64_t counter = current->counter_val;

	KOMB_BUG_ON(lock->combiner_task != current);

	if (next_node == NULL || next_node->next == NULL ||
	    check_irq_node(next_node) || check_irq_node(next_node->next) ||
	    counter >= komb_batch_size) {
		incoming_rsp_ptr = &(current->komb_stack_curr_ptr);
		current->komb_prev_waiter_task = current->komb_curr_waiter_task;
		current->komb_curr_waiter_task = NULL;

	} else {
		current->komb_prev_waiter_task = current->komb_curr_waiter_task;
		current->komb_curr_waiter_task = current->komb_next_waiter_task;
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
EXPORT_SYMBOL(mutex_unlock);

#include <linux/ww_mutex.h>

int ww_mutex_trylock(struct ww_mutex *lock, struct ww_acquire_ctx *ww_ctx)
{
	return mutex_trylock(&lock->base);
}
EXPORT_SYMBOL(ww_mutex_trylock);

int __sched ww_mutex_lock(struct ww_mutex *lock, struct ww_acquire_ctx *ctx)
{
	mutex_lock(&lock->base);
	return 0;
}
EXPORT_SYMBOL(ww_mutex_lock);

int __sched ww_mutex_lock_interruptible(struct ww_mutex *lock,
					struct ww_acquire_ctx *ctx)
{
	mutex_lock(&lock->base);
	return 0;
}
EXPORT_SYMBOL(ww_mutex_lock_interruptible);

void __sched ww_mutex_unlock(struct ww_mutex *lock)
{
	mutex_unlock(&lock->base);
}
EXPORT_SYMBOL(ww_mutex_unlock);

void ___mutex_init(struct mutex *lock, const char *name,
		   struct fds_lock_key *key)
{
	lock->tail = NULL;
	atomic_set(&lock->state, 0);
	lock->combiner_task = NULL;
	lock->key.name = name;
	lock->key.ptr = key;
	init_fds_lock_key(key, name, DEFAULT_FDS_LOCK);
}
EXPORT_SYMBOL(___mutex_init);

void __mutex_init(struct mutex *lock, const char *name,
		  struct lock_class_key *key)
{
	lock->tail = NULL;
	atomic_set(&lock->state, 0);
	lock->combiner_task = NULL;
}
EXPORT_SYMBOL(__mutex_init);

#define MUTEX_FLAG_WAITERS 0x01
#define MUTEX_FLAG_HANDOFF 0x02
#define MUTEX_FLAG_PICKUP 0x04

#define MUTEX_FLAGS 0x07

static __always_inline bool mutex_optimistic_spin(struct mutex *lock,
						  struct ww_acquire_ctx *ww_ctx,
						  struct mutex_waiter *waiter)
{
	return false;
}

int atomic_dec_and_mutex_lock(atomic_t *cnt, struct mutex *lock)
{
	if (atomic_add_unless(cnt, -1, 1))
		return 0;
	mutex_lock(lock);
	if (!atomic_dec_and_test(cnt)) {
		mutex_unlock(lock);
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(atomic_dec_and_mutex_lock);

EXPORT_TRACEPOINT_SYMBOL_GPL(contention_begin);
EXPORT_TRACEPOINT_SYMBOL_GPL(contention_end);
