// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <linux/sched.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/bpf.h>
#include <linux/bpf_fpop.h>

#ifdef CONFIG_BPF_SPINLOCK_HOOKS
extern void set_state_for_cs_timeout(void *lock, bool notify_watchdog);
extern void reset_state_for_cs_timeout(void *lock);

static __always_inline void fpop_enter_cs(struct qspinlock *lock,
					  bool notify_watchdog)
{
	set_state_for_cs_timeout(lock, notify_watchdog);
}

static __always_inline void fpop_exit_cs(struct qspinlock *lock)
{
	reset_state_for_cs_timeout(lock);
}
#else
static __always_inline void fpop_enter_cs(struct qspinlock *lock,
					  bool notify_watchdog)
{
}

static __always_inline void fpop_exit_cs(struct qspinlock *lock)
{
}
#endif /* CONFIG_BPF_SPINLOCK_HOOKS */

#ifdef CONFIG_BPF_TIMEOUT
extern void tell_bpf_loop_to_terminate(void *lock);
extern void bpf_throw(u64 cookie);
#endif /* CONFIG_BPF_TIMEOUT */

#ifdef CONFIG_BPF_TIMEOUT
DEFINE_PER_CPU(bool, fpop_running);
DEFINE_PER_CPU(struct qspinlock *, fpop_active_lock);
#endif

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

static void execute_op(struct qspinlock *lock, bpf_callback_t callback,
		       u64 v1, u64 v2, u64 v3)
{
#ifdef CONFIG_BPF_TIMEOUT
	this_cpu_write(fpop_active_lock, lock);
#endif
	fpop_enter_cs(lock, false);
	callback(v1, v2, v3, 0, 0);
	fpop_exit_cs(lock);
#ifdef CONFIG_BPF_TIMEOUT
	this_cpu_write(fpop_active_lock, NULL);
#endif
}

static void execute_op_and_unlock(struct qspinlock *lock, bpf_callback_t callback,
				  u64 v1, u64 v2, u64 v3,
				  bool notify_watchdog)
{
	fpop_enter_cs(lock, notify_watchdog);
	callback(v1, v2, v3, 0, 0);
	WRITE_ONCE(lock->locked, false);
	fpop_exit_cs(lock);
}

#ifdef CONFIG_BPF_TIMEOUT
static __always_inline void fpop_wait_for_node_reuse(struct fpop_node *node)
{
	while (READ_ONCE(node->completed) == (uint8_t)_FPOP_PRCSING)
		cpu_relax();
}
#else
static __always_inline void fpop_wait_for_node_reuse(struct fpop_node *node) { }
#endif

static void komb_spin_lock_slowpath(struct qspinlock *lock, bpf_callback_t callback,
				    u64 v1, u64 v2, u64 v3)
{
	register struct fpop_node *curr_node;
	struct fpop_node *prev_node = NULL, *next_node = NULL;
	int old_tail, val, tail;
	uint64_t counter_val = 0;

	curr_node = this_cpu_ptr(&fpop_nodes);
	fpop_wait_for_node_reuse(curr_node);
	tail = encode_tail(smp_processor_id());

	curr_node->locked = true;
	curr_node->completed = (uint8_t)_FPOP_UNPRCSD;
	curr_node->next = NULL;
	curr_node->tail = tail;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->callback = callback;
	curr_node->v1 = v1;
	curr_node->v2 = v2;
	curr_node->v3 = v3;

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		WRITE_ONCE(prev_node->next, curr_node);
		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

#ifdef CONFIG_BPF_TIMEOUT
		if (READ_ONCE(curr_node->completed) == (uint8_t)_FPOP_PRCSING) {
			u64 timeout_ns = (u64)READ_ONCE(sysctl_bpf_spin_lock_timeout) * NSEC_PER_MSEC;
			u64 end_time = ktime_get_mono_fast_ns() + timeout_ns;
			bool already_told_to_terminate = false;

			while(true) {
				if(!already_told_to_terminate && ktime_get_mono_fast_ns() > end_time) {
					tell_bpf_loop_to_terminate(lock);
					already_told_to_terminate = true;
					preempt_enable();
					bpf_throw(50); // Check if this will work.
					return;
				}
				if(READ_ONCE(curr_node->completed) == (uint8_t)_FPOP_PRCSD)
					return;
			}
		}
#endif
		if(READ_ONCE(curr_node->completed) == (uint8_t)_FPOP_PRCSD)
			return;
	}

#ifdef CONFIG_BPF_TIMEOUT
	{
		u64 timeout_ns = (u64)READ_ONCE(sysctl_bpf_spin_lock_timeout) * NSEC_PER_MSEC;
		u64 end_time = ktime_get_mono_fast_ns() + timeout_ns;
		bool already_told_to_terminate = false;

		while (true) {
			if (!already_told_to_terminate && ktime_get_mono_fast_ns() > end_time) {
				tell_bpf_loop_to_terminate(lock);
				already_told_to_terminate = true;
			}
			if (!(READ_ONCE(lock->val.counter) & _Q_LOCKED_PENDING_MASK))
				break;
			cpu_relax();
		}
		val = READ_ONCE(lock->val.counter);
	}
#else
	val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));
#endif

	if (((val & _Q_TAIL_MASK) == tail) &&
	    atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL)) {
		execute_op_and_unlock(lock, callback, v1, v2, v3, true);
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
		execute_op_and_unlock(lock, callback, v1, v2, v3, false);
		return;
	}

#ifdef CONFIG_BPF_TIMEOUT
	*this_cpu_ptr(&fpop_running) = true;
#endif

	while(true) {
		counter_val++;
		next_node = get_next_node(curr_node);
#ifdef CONFIG_BPF_TIMEOUT
		WRITE_ONCE(curr_node->locked_completed, (uint16_t)_FPOP_PRCSING);
#endif
		execute_op(lock, curr_node->callback, curr_node->v1, curr_node->v2, curr_node->v3);
		WRITE_ONCE(curr_node->locked_completed, (uint16_t)_FPOP_PRCSD);
		if(next_node == NULL || next_node->next == NULL || counter_val > komb_batch_size || need_resched())
			break;
		curr_node = next_node;
	}

#ifdef CONFIG_BPF_TIMEOUT
	*this_cpu_ptr(&fpop_running) = false;
#endif
	
	if (*this_cpu_ptr(&local_queue_head) != NULL) {
		(*this_cpu_ptr(&local_queue_tail))->next = next_node;
		next_node = *this_cpu_ptr(&local_queue_head);
		*this_cpu_ptr(&local_queue_head) = NULL;
		*this_cpu_ptr(&local_queue_tail) = NULL;
	}

	set_locked(lock);
	next_node->locked = false;
	smp_mb();
	execute_op_and_unlock(lock, callback, v1, v2, v3, false);
	return;
}

void fpop_execute(struct qspinlock *lock, bpf_callback_t callback,
		  u64 v1, u64 v2, u64 v3)
{
	u32 val, cnt;

	preempt_disable();

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
	{
		execute_op_and_unlock(lock, callback, v1, v2, v3, true);
		goto out;
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
				tell_bpf_loop_to_terminate(lock);
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
	execute_op_and_unlock(lock, callback, v1, v2, v3, true);
	goto out;

queue:
	komb_spin_lock_slowpath(lock, callback, v1, v2, v3);
out:
	preempt_enable();
	return;
}
