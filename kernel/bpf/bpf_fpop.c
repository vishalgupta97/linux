// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <asm/qspinlock.h>
#include <linux/sched.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

static DEFINE_PER_CPU_SHARED_ALIGNED(struct fpop_node, fpop_nodes);

static long komb_batch_size = 16384;

static inline __pure u32 encode_tail(int cpu)
{
	u32 tail = (cpu + 1) << _Q_TAIL_CPU_OFFSET;
	return tail;
}

static inline __pure struct fpop_node *decode_tail(u32 tail)
{
	int cpu = (tail >> _Q_TAIL_CPU_OFFSET) - 1;
	return per_cpu_ptr(&fpop_nodes, cpu);
}

__always_inline void clear_locked_set_completed(struct fpop_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

__always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked_pending, _Q_LOCKED_VAL);
}

static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	return ((u32)xchg(&lock->tail, tail >> _Q_TAIL_OFFSET)) << _Q_TAIL_OFFSET;
}

__always_inline u32 cmpxchg_tail(struct qspinlock *lock, u32 tail, u32 new_tail)
{
	return ((u32)cmpxchg(&lock->tail, tail >> _Q_TAIL_OFFSET, new_tail >> _Q_TAIL_OFFSET)) << _Q_TAIL_OFFSET;
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
			val = atomic_cond_read_relaxed(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_COMBINER_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) == val)
				return;
		}
	}
	WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
}

__always_inline static void add_to_local_queue(struct fpop_node *node)
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

__always_inline static struct fpop_node *
get_next_node(struct fpop_node *my_node)
{
	int i;
	struct fpop_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->next == NULL)
			goto next_node_null;

		if (next_node->socket_id == numa_node_id()) {
			prefetch(next_node->cs_fn_ptr);
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

static void komb_spin_lock_slowpath(struct qspinlock *lock)
{
	register struct fpop_node *curr_node;
	struct fpop_node *prev_node = NULL, *next_node = NULL;
	int old_tail, val, i, tail;
	struct fpop_node* local_queue_head = NULL;
	struct fpop_node* local_queue_tail = NULL;
	uint64_t counter_val = 0;

	curr_node = this_cpu_ptr(&fpop_nodes[0]);
	tail = encode_tail(smp_processor_id(), 0);

	curr_node->locked = true;
	curr_node->completed = false;
	curr_node->next = NULL;
	curr_node->tail = tail;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		prev_node->next = curr_node;
		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

		if (curr_node->completed)
			return;
	}

	val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

	if (((val & _Q_TAIL_MASK) == tail) &&
	    atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL))
		return;

	check_and_set_combiner(lock);

	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	curr_node = curr_node->next; // Skip my node, will be executed at last

	if (curr_node->next == NULL) {
		set_locked(lock);
		curr_node->locked = false;
		smp_mb();
		return;
	}

	while(true) {
		counter_val++;
		next_node = get_next_node(curr_node);
		execute_op(curr_node);
		curr_node->locked_completed = 1;
		if(next_node == NULL || next_node->next == NULL || counter_val > komb_batch_size || need_resched())
			break;
		curr_node = next_node;
	}
	
	if (local_queue_head != NULL) {
		local_queue_tail->next = next_node;
		next_node = local_queue_head;
	}

	next_node->locked = false;	
	set_locked(lock);
}

void fpop_execute(struct qspinlock *lock)
{
	u32 val, cnt;
	struct fpop_node *curr_node = NULL;
	struct shadow_stack *ptr;
	uint32_t curr_cpuid, prev_cpuid;
	struct fpop_node *prev_node, *next_node;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		goto execute_and_unlock;

	if (val == _Q_PENDING_VAL) {
		cnt = _Q_PENDING_LOOPS;
		val = atomic_cond_read_relaxed(&lock->val, (VAL != _Q_PENDING_VAL) || !cnt--);
	}

	if (val & ~_Q_LOCKED_MASK)
		goto queue;

	val = atomic_fetch_or_acquire(_Q_PENDING_VAL, &lock->val);

	if (unlikely(val & ~_Q_LOCKED_MASK)) {
		if (!(val & _Q_PENDING_MASK))
			clear_pending(lock);

		goto queue;
	}

	if (val & _Q_LOCKED_MASK)
		atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_MASK));

	clear_pending_set_locked(lock);
	goto execute_and_unlock;

queue:
	komb_spin_lock_slowpath(lock);

execute_and_unlock:
	execute_op();
	lock->locked = false;
}
