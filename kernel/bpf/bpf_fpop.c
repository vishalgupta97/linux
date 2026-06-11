// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <linux/sched.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/bpf_fpop.h>

#ifdef CONFIG_BPF_TIMEOUT
extern void bpf_notify_lock_kthread(void);
extern void bpf_notify_unlock_kthread(void);
extern void tell_bpf_loop_to_terminate(void);
#endif /* CONFIG_BPF_TIMEOUT */


static DEFINE_PER_CPU_SHARED_ALIGNED(struct fpop_node, fpop_nodes);
static DEFINE_PER_CPU_ALIGNED(struct fpop_node*, local_queue_head);
static DEFINE_PER_CPU_ALIGNED(struct fpop_node*, local_queue_tail);

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

static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	return ((u32)xchg(&lock->tail, tail >> _Q_TAIL_OFFSET)) << _Q_TAIL_OFFSET;
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
	if (*this_cpu_ptr(&local_queue_head) == NULL) {
		*this_cpu_ptr(&local_queue_head) = node;
		*this_cpu_ptr(&local_queue_tail) = node;
	} else {
		(*this_cpu_ptr(&local_queue_tail))->next = node;
		*this_cpu_ptr(&local_queue_tail) = node;
	}
}

__always_inline static struct fpop_node *
get_next_node(struct fpop_node *my_node)
{
	struct fpop_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->next == NULL)
			goto next_node_null;

		if (next_node->socket_id == numa_node_id()) {
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

static void execute_op(bpf_callback_t callback, void* local_state)
{
	callback((u64)(long)local_state, 0, 0, 0, 0);
}

static void execute_op_and_unlock(struct qspinlock *lock, bpf_callback_t callback, void* local_state)
{
#ifdef CONFIG_BPF_TIMEOUT
	bpf_notify_lock_kthread(); // Only waiter. Notify kthread.
#endif
	callback((u64)(long)local_state, 0, 0, 0, 0);
	WRITE_ONCE(lock->locked, false);
#ifdef CONFIG_BPF_TIMEOUT
	bpf_notify_unlock_kthread(); // Only waiter. Notify kthread.
#endif

}


static void komb_spin_lock_slowpath(struct qspinlock *lock, bpf_callback_t callback, void* local_state)
{
	register struct fpop_node *curr_node;
	struct fpop_node *prev_node = NULL, *next_node = NULL;
	int old_tail, val, tail;
	uint64_t counter_val = 0;

	curr_node = this_cpu_ptr(&fpop_nodes);
	tail = encode_tail(smp_processor_id());

	curr_node->locked = true;
	curr_node->completed = false;
	curr_node->next = NULL;
	curr_node->tail = tail;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->callback = callback;
	curr_node->local_state = local_state;

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		prev_node->next = curr_node;
		//TODO: Add timeout here
		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

		if (curr_node->completed)
			return;
	}

	val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

	if (((val & _Q_TAIL_MASK) == tail) &&
	    atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL)) {
		execute_op_and_unlock(lock, callback, local_state);
		return;
	}

	check_and_set_combiner(lock);

	*this_cpu_ptr(&local_queue_head) = NULL;
	*this_cpu_ptr(&local_queue_tail) = NULL;

	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	curr_node = curr_node->next; // Skip my node, will be executed at last

	if (curr_node->next == NULL) {
		set_locked(lock);
		curr_node->locked = false;
		smp_mb();
		execute_op(callback, local_state); // No Timeout, next waiter exists.
		WRITE_ONCE(lock->locked, false);
		return;
	}

	while(true) {
		counter_val++;
		next_node = get_next_node(curr_node);
		execute_op(curr_node->callback, curr_node->local_state);
		WRITE_ONCE(curr_node->locked_completed, 1);
		if(next_node == NULL || next_node->next == NULL || counter_val > komb_batch_size || need_resched())
			break;
		curr_node = next_node;
	}
	
	if (*this_cpu_ptr(&local_queue_head) != NULL) {
		(*this_cpu_ptr(&local_queue_tail))->next = next_node;
		next_node = *this_cpu_ptr(&local_queue_head);
		*this_cpu_ptr(&local_queue_head) = NULL;
		*this_cpu_ptr(&local_queue_tail) = NULL;
	}

	set_locked(lock);
	next_node->locked = false;
	smp_mb();
	execute_op(callback, local_state); // No Timeout, next waiter exists.
	WRITE_ONCE(lock->locked, false);
	return;
}

void fpop_execute(struct qspinlock *lock, bpf_callback_t callback, void* local_state)
{
	u32 val, cnt;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
	{
		execute_op_and_unlock(lock, callback, local_state);
		return;
	}

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

	if (val & _Q_LOCKED_MASK) {
#ifdef CONFIG_BPF_TIMEOUT
		u64 timeout_ns = (u64)READ_ONCE(sysctl_bpf_spin_lock_timeout) * NSEC_PER_MSEC;
		u64 end_time = ktime_get_mono_fast_ns() + timeout_ns;
		bool already_told_to_terminate = false;

		while (true) {
			if (!already_told_to_terminate && ktime_get_mono_fast_ns() > end_time) {
				tell_bpf_loop_to_terminate();
				already_told_to_terminate = true;
			}
			if (!(READ_ONCE(lock->locked)))
				break;
			cpu_relax();
		}
#else
		atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_MASK));
#endif
	}

	WRITE_ONCE(lock->locked_pending, _Q_LOCKED_VAL);
	execute_op_and_unlock(lock, callback, local_state);
	return;

queue:
	komb_spin_lock_slowpath(lock, callback, local_state);
	return;
}
