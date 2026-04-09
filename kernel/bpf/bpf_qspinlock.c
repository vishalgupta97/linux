// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BPF Custom QSpinlock Slow Path
 *
 * Provides a qspinlock-based lock acquisition function for BPF where the
 * waiter at the head of the MCS queue starts/cancels the hrtimer.  The
 * function acquires the lock with IRQs *enabled* (so the hrtimer can fire on
 * the waiting CPU), then disables IRQs only after acquisition.
 *
 * Modeled on kernel/bpf/rqspinlock.c but with the following differences:
 *  - No inline timeout / ktime polling.  Timeout via hrtimer only.
 *  - Never returns an error.  On timeout the *owner* is terminated by
 *    bpf_spin_lock_timeout_handler(); the waiter eventually acquires normally.
 *  - No deadlock detection.
 *  - Uses its own bpf_qnodes per-CPU MCS nodes (separate from rqnodes).
 */

#include <linux/smp.h>
#include <linux/bpf.h>
#include <linux/cpumask.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/prefetch.h>
#include <linux/hrtimer.h>
#include <linux/irqflags.h>
#include <asm/byteorder.h>
#include <asm/qspinlock.h>
#include <linux/bpf_lock_timer.h>

#include "../locking/qspinlock.h"
#include "../locking/mcs_spinlock.h"

#include <linux/bpf_qspinlock.h>

/*
 * Separate per-CPU MCS queue nodes for BPF — must not share nodes with the
 * generic kernel qspinlock or rqspinlock paths.
 */
static DEFINE_PER_CPU_ALIGNED(struct qnode, bpf_qnodes[_Q_MAX_NODES]);

/* ---------------------------------------------------------------------- */
/* Externs defined in helpers.c / syscall.c                               */
/* ---------------------------------------------------------------------- */
extern int ebpf_spinlock_timeout;
extern int sysctl_bpf_spin_lock_timeout;
extern const struct bpf_lock_timer_ops bpf_hrtimer_ops;
extern enum hrtimer_restart bpf_qspinlock_timer_cb(struct hrtimer *timer);
extern void bpf_notify_lock_kthread(void);

/* ---------------------------------------------------------------------- */
/* Per-CPU waiter timers                                                    */
/* ---------------------------------------------------------------------- */

/*
 * Per-CPU hrtimer used by the waiter at the head of the MCS queue.
 * Started with IRQs enabled so it can fire on the waiting CPU and set
 * ebpf_spinlock_timeout to terminate the owner.
 */
static DEFINE_PER_CPU(struct hrtimer, bpf_waiter_hrtimer);
static DEFINE_PER_CPU(struct bpf_lock_timer, bpf_waiter_timer);

/*
 * Per-CPU pointer to whichever bpf_lock_timer is currently active for this
 * CPU's lock session.  Set by:
 *   - the waiter slow path (contended case), or
 *   - bpf_notify_lock_kthread() (uncontended fast path).
 * Cleared (and the underlying timer cancelled) by bpf_spin_unlock via
 * bpf_lock_timer_cancel().
 */
DEFINE_PER_CPU(struct bpf_lock_timer *, bpf_active_timer);
EXPORT_PER_CPU_SYMBOL_GPL(bpf_active_timer);

/* ---------------------------------------------------------------------- */
/* Initialisation                                                           */
/* ---------------------------------------------------------------------- */

/**
 * bpf_qspinlock_init_timers - initialise per-CPU waiter hrtimers.
 * Called once from bpf_lock_kthread_init() (late_initcall in helpers.c).
 */
void __init bpf_qspinlock_init_timers(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct hrtimer *timer = per_cpu_ptr(&bpf_waiter_hrtimer, cpu);
		struct bpf_lock_timer *lt = per_cpu_ptr(&bpf_waiter_timer, cpu);

		hrtimer_setup(timer, bpf_qspinlock_timer_cb, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL | HRTIMER_MODE_HARD);
		lt->ops = &bpf_hrtimer_ops;
		lt->ctx = timer;
	}
}

extern void tell_bpf_loop_to_terminate(void);

/* ---------------------------------------------------------------------- */
/* Slow path                                                                */
/* ---------------------------------------------------------------------- */

/**
 * bpf_queued_spin_lock_slowpath - acquire a qspinlock for BPF (slow path).
 * @lock:      Pointer to the qspinlock to acquire.
 * @val:       Current value of lock->val as read by the fast-path attempt.
 * @flags_out: Receives the saved IRQ flags (written after acquisition).
 *
 * Returns 0 always.  IRQs are disabled after the lock is held; the caller
 * (bpf_qspinlock_lock) stores *flags_out.
 *
 * The hrtimer started here remains active while the new owner holds the lock.
 * It is cancelled by bpf_spin_unlock() via the per-CPU bpf_active_timer
 * pointer.
 *
 * Preemption MUST already be disabled by the caller.
 */
static void bpf_queued_spin_lock_slowpath(struct qspinlock *lock, u32 val)
{
	struct mcs_spinlock *prev, *next, *node;
	struct bpf_lock_timer *lt;
	u64 timeout_ns;
	int idx;
	u32 old, tail;

	BUILD_BUG_ON(CONFIG_NR_CPUS >= (1U << _Q_TAIL_CPU_BITS));

	/*
	 * Wait for in-progress pending->locked hand-overs with a bounded
	 * number of spins so that we guarantee forward progress.
	 *
	 * 0,1,0 -> 0,0,1
	 */
	if (val == _Q_PENDING_VAL) {
		int cnt = _Q_PENDING_LOOPS;

		val = atomic_cond_read_relaxed(&lock->val,
					       (VAL != _Q_PENDING_VAL) || !cnt--);
	}

	/*
	 * If we observe any contention; queue.
	 */
	if (val & ~_Q_LOCKED_MASK)
		goto queue;

	/*
	 * trylock || pending
	 *
	 * 0,0,* -> 0,1,* -> 0,0,1 pending, trylock
	 */
	val = queued_fetch_set_pending_acquire(lock);

	/*
	 * If we observe contention, there is a concurrent locker.
	 *
	 * Undo and queue; our setting of PENDING might have made the
	 * n,0,0 -> 0,0,0 transition fail and it will now be waiting
	 * on @next to become !NULL.
	 */
	if (unlikely(val & ~_Q_LOCKED_MASK)) {
		/* Undo PENDING if we set it. */
		if (!(val & _Q_PENDING_MASK))
			clear_pending(lock);
		goto queue;
	}

	/*
	 * We're pending, wait for the owner to go away.
	 *
	 * 0,1,1 -> *,1,0
	 *
	 * This must be a load-acquire so we match the store-release that
	 * clears the locked bit and create lock sequentiality.
	 * IRQs are enabled here so the hrtimer can fire on this CPU.
	 */
	if (val & _Q_LOCKED_MASK) {
    	//smp_cond_load_acquire(&lock->locked, !VAL);

        u64 end_time = ktime_get_mono_fast_ns() + timeout_ns;
        bool already_told_to_terminate = false;

        while(true) {
            if(!already_told_to_terminate && ktime_get_mono_fast_ns() >  end_time) {
                tell_bpf_loop_to_terminate();
                already_told_to_terminate = true;
            }

            if(!(READ_ONCE(lock->locked)))
                break;

            cpu_relax();
        }    
    }

	/*
	 * Take ownership and clear the pending bit: 0,1,0 -> 0,0,1
	 *
	 * Lock acquired.  Disable IRQs now.  The timer (if started) keeps
	 * running — it will be cancelled by bpf_spin_unlock via bpf_active_timer.
	 */
	clear_pending_set_locked(lock);
	bpf_notify_lock_kthread(); // Only waiter. Notify kthread.
	return;

queue:
	node = this_cpu_ptr(&bpf_qnodes[0].mcs);
	idx = node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	/*
	 * Fallback: if we somehow exceed _Q_MAX_NODES, spin with trylock.
	 * This is unexpected in normal operation (max nesting depth is 4).
	 */
	if (unlikely(idx >= _Q_MAX_NODES)) {
		node->count--;
		while (!queued_spin_trylock(lock))
			cpu_relax();
		bpf_notify_lock_kthread(); // Don't know the next waiter. Notify kthread.
		return;
	}

	node = grab_mcs_node(node, idx);

	/*
	 * Ensure node->count increment is visible before we initialise the
	 * node itself; prevents an IRQ from clobbering our assignments.
	 */
	barrier();

	node->locked = 0;
	node->next = NULL;

	/*
	 * Attempt a trylock once more before committing to the queue in case
	 * the lock was released while we were setting up.
	 */
	if (queued_spin_trylock(lock)) {
		__this_cpu_dec(bpf_qnodes[0].mcs.count);
		bpf_notify_lock_kthread(); // Don't know the next waiter. Notify kthread.
		return;
	}

	/*
	 * Ensure that the initialisation of @node is complete before we
	 * publish the updated tail via xchg_tail() and potentially link
	 * @node into the waitqueue via WRITE_ONCE(prev->next, node) below.
	 */
	smp_wmb();

	/*
	 * Publish the updated tail: p,*,* -> n,*,*
	 */
	old = xchg_tail(lock, tail);
	next = NULL;

	/*
	 * If there was a previous node, link into the wait queue and spin
	 * (with IRQs enabled so the hrtimer can fire) until we reach the head.
	 */
	if (old & _Q_TAIL_MASK) {
		prev = decode_tail(old, bpf_qnodes);

		/* Link @node into the waitqueue. */
		WRITE_ONCE(prev->next, node);

		/* Spin until the predecessor hands off to us. */
		arch_mcs_spin_lock_contended(&node->locked);

		next = READ_ONCE(node->next);
		if (next)
			prefetchw(next);
	}

	/*
	 * We are now at the head of the waitqueue.
	 * Start the hrtimer so it can fire on this CPU (IRQs are still
	 * enabled) and set ebpf_spinlock_timeout to terminate the owner.
	 */
	timeout_ns = (u64)READ_ONCE(sysctl_bpf_spin_lock_timeout) * NSEC_PER_MSEC;
	/*lt = this_cpu_ptr(&bpf_waiter_timer);
	if (timeout_ns > 0) {
		bpf_lock_timer_start(lt, timeout_ns);
		this_cpu_write(bpf_active_timer, lt);
	}*/

	/*
	 * Wait for the owner & pending to go away: *,x,y -> *,0,0
	 *
	 * This must be a load-acquire to create lock sequentiality.
	 * IRQs are enabled here so the hrtimer can fire on this CPU.
	 */
	//val = atomic_cond_read_acquire(&lock->val,
	//			       !(VAL & _Q_LOCKED_PENDING_MASK));

    	u64 end_time = ktime_get_mono_fast_ns() + timeout_ns;
    	bool already_told_to_terminate = false;

    	while(true) {
        	if(!already_told_to_terminate && ktime_get_mono_fast_ns() >  end_time) {
            		tell_bpf_loop_to_terminate();
            		already_told_to_terminate = true;
        	}

        	if(!(READ_ONCE(lock->val.counter) & _Q_LOCKED_PENDING_MASK))
            		break;

        	cpu_relax();
    	}    

	val = READ_ONCE(lock->val.counter);

	/*
	 * Claim the lock:
	 *
	 *   n,0,0 -> 0,0,1 : lock, uncontended
	 *   *,*,0 -> *,*,1 : lock, contended
	 *
	 * If we are the only one in the queue (val == tail) and nobody is
	 * pending, clear the tail code and grab the lock atomically.
	 * Otherwise just grab the lock.
	 */
	if ((val & _Q_TAIL_MASK) == tail) {
		if (atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL))
		{
			bpf_notify_lock_kthread(); //No next waiter. Notify kthread.
			goto release;
		}
	}

	set_locked(lock);

	/*
	 * Contended path: wait for next waiter to appear, then wake it.
	 */
	if (!next)
		next = smp_cond_load_relaxed(&node->next, (VAL));
	arch_mcs_spin_unlock_contended(&next->locked);

release:
	__this_cpu_dec(bpf_qnodes[0].mcs.count);
}

/* ---------------------------------------------------------------------- */
/* Top-level wrapper                                                        */
/* ---------------------------------------------------------------------- */

/**
 * bpf_qspinlock_lock - acquire a qspinlock for BPF.
 * @lock:      The qspinlock to acquire.
 * @flags_out: Receives the saved IRQ flags (written after acquisition).
 *
 * On success: lock is held, IRQs are disabled and saved to *flags_out.
 * Preemption MUST already be disabled by the caller.
 *
 * For the fast (uncontended) path, the kthread is notified to start a timer.
 * For the slow path, the waiter at the head of the MCS queue starts the timer
 * itself.
 */
void bpf_qspinlock_lock(struct qspinlock *lock)
{
	u32 val = 0;

	if (likely(atomic_try_cmpxchg_acquire(&lock->val, &val, _Q_LOCKED_VAL))) {
		/*
		 * Uncontended fast path — no waiter is at the head of the queue
		 * to start the timer.  Disable IRQs, then notify the kthread so
		 * it can start the watchdog timer for this CPU's critical section.
		 */
		bpf_notify_lock_kthread();
		return;
	}

	bpf_queued_spin_lock_slowpath(lock, val);
}
EXPORT_SYMBOL_GPL(bpf_qspinlock_lock);

void bpf_qspinlock_unlock(struct qspinlock *lock)
{
  smp_store_release(&lock->locked, 0);
}
EXPORT_SYMBOL_GPL(bpf_qspinlock_unlock);

