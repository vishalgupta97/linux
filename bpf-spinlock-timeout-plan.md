# BPF Spinlock: Waiter/Kthread-driven Timeout – Implementation Plan

Date: 2026-03-02

## Problem

The legacy `bpf_spin_lock` path in `kernel/bpf/helpers.c` starts an hrtimer
**after** the lock is acquired, with interrupts already disabled
(`local_irq_save` happens **before** `arch_spin_lock`). Since the hrtimer
interrupt cannot fire while IRQs are off, the timeout only triggers after
`bpf_spin_unlock` re-enables interrupts — defeating its purpose.

## Solution Overview

1. Switch from `arch_spin_lock` to a **custom qspinlock slow path** (when
   `CONFIG_QUEUED_SPINLOCKS` is set) so the lock can be acquired with IRQs
   enabled. Disable IRQs only **after** acquisition.
2. Move hrtimer responsibility from the lock **holder** to:
   - **(a) The waiter at the head of the MCS queue** (contended case), or
   - **(b) A global kthread** (uncontended case — no one is waiting to start
     the timer).
3. Abstract the timer start/cancel behind an ops struct so other backends
   (ktime polling, jiffies) can be plugged in later.

The existing enforcement path is **unchanged**: `bpf_loop()` and
`bpf_iter_num_next()` in `kernel/bpf/bpf_iter.c` check the global
`ebpf_spinlock_timeout` flag and call `bpf_spin_lock_timeout_handler()`
→ undo-log replay → lock release → `bpf_die()`.

---

## File Map (existing files to modify / new files to create)

| File | Action |
|------|--------|
| `include/linux/bpf_lock_timer.h` | **CREATE** — timer abstraction |
| `kernel/bpf/bpf_qspinlock.c` | **CREATE** — custom qspinlock slow path for BPF |
| `kernel/bpf/helpers.c` | **MODIFY** — restructure lock/unlock, add kthread, wire up new paths |
| `kernel/bpf/Makefile` | **MODIFY** — add `bpf_qspinlock.o` |
| `kernel/bpf/Kconfig` | **MODIFY** — (optional) add config for timeout value |

Files that are **NOT modified**:
- `kernel/bpf/bpf_iter.c` — the `ebpf_spinlock_timeout` check stays as-is
- `kernel/bpf/rqspinlock.c` — the resilient kfunc path is separate
- `include/asm-generic/rqspinlock.h` — no changes

---

## Step-by-Step Implementation

### Step 1: Create `include/linux/bpf_lock_timer.h` — Timer abstraction

**Purpose:** Abstract hrtimer start/cancel so alternative backends (ktime
polling, jiffies) can be swapped in without touching the lock logic.

Create this new header file with the following contents:

```c
/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_BPF_LOCK_TIMER_H
#define _LINUX_BPF_LOCK_TIMER_H

#include <linux/hrtimer.h>
#include <linux/types.h>

/**
 * struct bpf_lock_timer_ops - timer backend operations
 * @start: Start a timeout. @ctx is backend-specific state, @timeout_ns is
 *         the timeout duration in nanoseconds.
 * @cancel: Cancel a previously started timeout.
 */
struct bpf_lock_timer_ops {
	void (*start)(void *ctx, u64 timeout_ns);
	void (*cancel)(void *ctx);
};

/**
 * struct bpf_lock_timer - timer instance
 * @ops:  Pointer to the active backend operations.
 * @ctx:  Backend-specific context (e.g., pointer to an hrtimer).
 */
struct bpf_lock_timer {
	const struct bpf_lock_timer_ops *ops;
	void *ctx;
};

static inline void bpf_lock_timer_start(struct bpf_lock_timer *t, u64 timeout_ns)
{
	if (t && t->ops && t->ops->start)
		t->ops->start(t->ctx, timeout_ns);
}

static inline void bpf_lock_timer_cancel(struct bpf_lock_timer *t)
{
	if (t && t->ops && t->ops->cancel)
		t->ops->cancel(t->ctx);
}

#endif /* _LINUX_BPF_LOCK_TIMER_H */
```

**Key design points:**
- `struct bpf_lock_timer_ops` has only two operations: `start` and `cancel`.
- `struct bpf_lock_timer` holds an ops pointer and an opaque `ctx`.
- Inline wrappers do NULL checks for safety.
- Future ktime/jiffies backends just provide different ops + ctx.

---

### Step 2: Create `kernel/bpf/bpf_qspinlock.c` — Custom BPF qspinlock slow path

**Purpose:** Provide a qspinlock-based lock acquisition function for BPF where
the waiter at the head of the queue starts/cancels the hrtimer. The function
acquires the lock with IRQs enabled, then disables IRQs after acquisition.

Create a new file modeled closely on `kernel/bpf/rqspinlock.c` lines 300–663,
but with major differences noted below.

#### 2a. Includes and per-CPU data

```c
// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/smp.h>
#include <linux/bug.h>
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

static DEFINE_PER_CPU_ALIGNED(struct qnode, bpf_qnodes[_Q_MAX_NODES]);
```

Note: we use a **separate** per-CPU `bpf_qnodes` array from both the generic
kernel's qspinlock and rqspinlock's `rqnodes`, because the BPF lock path must
not share MCS queue nodes with other subsystems.

#### 2b. hrtimer backend implementation

Implement the hrtimer backend for `bpf_lock_timer_ops` in this file
(or in helpers.c — either works; keeping it here keeps the lock logic
self-contained):

```c
/*
 * Declared in helpers.c, the global flag checked by bpf_loop / bpf_iter_num_next.
 */
extern int ebpf_spinlock_timeout;

static void bpf_hrtimer_start(void *ctx, u64 timeout_ns)
{
	struct hrtimer *timer = ctx;
	hrtimer_start(timer, ns_to_ktime(timeout_ns), HRTIMER_MODE_REL | HRTIMER_MODE_HARD);
}

static void bpf_hrtimer_cancel(void *ctx)
{
	struct hrtimer *timer = ctx;
	hrtimer_cancel(timer);
}

static const struct bpf_lock_timer_ops bpf_hrtimer_ops = {
	.start  = bpf_hrtimer_start,
	.cancel = bpf_hrtimer_cancel,
};
```

Also define a per-CPU hrtimer and `bpf_lock_timer` instance that will be used
by the waiter and kthread:

```c
static DEFINE_PER_CPU(struct hrtimer, bpf_waiter_hrtimer);
static DEFINE_PER_CPU(struct bpf_lock_timer, bpf_waiter_timer);

/* Track which timer instance is active for the current CPU's lock session */
DEFINE_PER_CPU(struct bpf_lock_timer *, bpf_active_timer);
EXPORT_PER_CPU_SYMBOL_GPL(bpf_active_timer);

/*
 * hrtimer callback — identical behavior to helpers.c bpf_spin_lock_timer_cb.
 * Sets the global terminate flag.
 */
static enum hrtimer_restart bpf_qspinlock_timer_cb(struct hrtimer *timer)
{
	WRITE_ONCE(ebpf_spinlock_timeout, 1);
	return HRTIMER_NORESTART;
}
```

Add an init function (called from a `late_initcall` or from helpers.c init):

```c
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
```

#### 2c. The custom slow path function: `bpf_queued_spin_lock_slowpath()`

**Signature:**

```c
/*
 * Acquire a qspinlock for BPF. Returns with the lock held and
 * *flags_out set from local_irq_save (caller must store it).
 * IRQs are disabled only AFTER acquisition.
 */
int bpf_queued_spin_lock_slowpath(struct qspinlock *lock, u32 val,
                                  unsigned long *flags_out);
```

**Body structure — based on `resilient_queued_spin_lock_slowpath()` in
`kernel/bpf/rqspinlock.c` lines 338–663, with these key differences:**

| rqspinlock.c behavior | bpf_qspinlock.c behavior |
|---|---|
| Uses `rqspinlock_timeout` (ktime polling) for timeout | No inline timeout. Uses hrtimer only. |
| Returns `-ETIMEDOUT` / `-EDEADLK` to caller | Never returns an error — the timeout terminates the **owner** via `ebpf_spinlock_timeout` flag, and the waiter eventually gets the lock normally. |
| Calls `check_deadlock_AA()`, `check_deadlock_ABBA()` | No deadlock detection (relies on timeout → terminate). |
| Calls `grab_held_lock_entry()` / `release_held_lock_entry()` for held lock table | **Does not** use `rqspinlock_held_locks` — the BPF path uses its own `held_locks[]` in helpers.c. |
| Uses `rqnodes` per-CPU MCS nodes | Uses its own `bpf_qnodes` per-CPU MCS nodes. |
| IRQs are already disabled on entry | IRQs are **enabled** on entry; `local_irq_save()` after lock acquisition. |
| `RES_TIMEOUT_VAL` propagation through MCS queue | Not needed — the waiter at head starts/cancels hrtimer; no timeout propagation between queue nodes. |

**Pseudo-code for the three paths:**

**Path A — Pending bit (one waiter ahead, no queue):**
```
val = queued_fetch_set_pending_acquire(lock)
if contention observed:
    clear pending if we set it
    goto queue

// We are the pending waiter — effectively head of queue
// START the hrtimer
timeout_ns = sysctl_bpf_spin_lock_timeout * NSEC_PER_MSEC
lt = this_cpu_ptr(&bpf_waiter_timer)
bpf_lock_timer_start(lt, timeout_ns)
this_cpu_write(bpf_active_timer, lt)

// Spin for locked bit to clear (owner releases)
smp_cond_load_acquire(&lock->locked, !VAL)

// CANCEL the timer
bpf_lock_timer_cancel(lt)

// Acquired! Disable IRQs now.
local_irq_save(*flags_out)
clear_pending_set_locked(lock)
return 0
```

**Path B — MCS queue (multiple waiters):**
```
// Enqueue into MCS queue
node = this_cpu_ptr(&bpf_qnodes[0].mcs)
idx = node->count++
tail = encode_tail(smp_processor_id(), idx)
old = xchg_tail(lock, tail)

if old & _Q_TAIL_MASK:
    // There is a predecessor — link and wait
    prev = decode_tail(old, bpf_qnodes)
    WRITE_ONCE(prev->next, node)
    arch_mcs_spin_lock_contended(&node->locked)   // spin until predecessor signals us

// === We are now HEAD of the MCS queue ===
// START the hrtimer
timeout_ns = sysctl_bpf_spin_lock_timeout * NSEC_PER_MSEC
lt = this_cpu_ptr(&bpf_waiter_timer)
bpf_lock_timer_start(lt, timeout_ns)
this_cpu_write(bpf_active_timer, lt)

// Spin for owner + pending to clear
val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK))

// CANCEL the timer
bpf_lock_timer_cancel(lt)

// Acquired! Disable IRQs now.
local_irq_save(*flags_out)

// Claim the lock
if (val & _Q_TAIL_MASK) == tail:
    if atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL):
        goto release
set_locked(lock)

// Signal next waiter in MCS queue
if !next:
    next = smp_cond_load_relaxed(&node->next, VAL)
arch_mcs_spin_unlock_contended(&next->locked)

release:
    __this_cpu_dec(bpf_qnodes[0].mcs.count)
    return 0
```

**Path C — Uncontended (handled by the wrapper, not the slow path):**
See Step 3. The fast path in the wrapper does `queued_spin_trylock()`, and
if it succeeds, notifies the kthread instead.

**Important notes:**
- The `smp_cond_load_acquire` spins at the head of the queue will eventually
  succeed because when the hrtimer fires: `ebpf_spinlock_timeout` is set → the
  lock **owner** (running a BPF program with `bpf_loop`/`bpf_for`) checks the
  flag → calls `bpf_spin_lock_timeout_handler()` → releases the lock → the
  head-of-queue waiter's spin condition is satisfied.
- If the owner is stuck in a tight loop without `bpf_loop`/`bpf_for`, timeouts
  won't help (same limitation as before). This is enforced at the verifier
  level: BPF programs must use bounded loops.
- The `local_irq_save()` **after** the lock is acquired is critical — it means
  the hrtimer can fire on the waiter's CPU during the spin.

#### 2d. Wrapper function: `bpf_qspinlock_lock()`

```c
/*
 * Top-level BPF qspinlock acquisition. Tries the fast path, falls back
 * to the custom slow path.
 *
 * On success: lock is held, IRQs are saved to *flags_out, preemption
 *             was already disabled by caller.
 */
int bpf_qspinlock_lock(struct qspinlock *lock, unsigned long *flags_out)
{
	u32 val = 0;

	if (likely(atomic_try_cmpxchg_acquire(&lock->val, &val, _Q_LOCKED_VAL))) {
		/* Uncontended — no waiter to start the timer.
		 * Disable IRQs, then notify the kthread.
		 */
		local_irq_save(*flags_out);
		bpf_notify_lock_kthread();    /* defined in helpers.c, Step 4 */
		return 0;
	}

	return bpf_queued_spin_lock_slowpath(lock, val, flags_out);
}
EXPORT_SYMBOL_GPL(bpf_qspinlock_lock);
```

Note: `bpf_notify_lock_kthread()` is implemented in Step 4.

#### 2e. Export declarations

Add to a header (or declare extern in helpers.c):

```c
extern int bpf_qspinlock_lock(struct qspinlock *lock, unsigned long *flags_out);
extern void __init bpf_qspinlock_init_timers(void);
```

---

### Step 3: Modify `kernel/bpf/helpers.c` — Restructure lock/unlock

#### 3a. Add includes

At the top of `kernel/bpf/helpers.c`, add:

```c
#include <linux/bpf_lock_timer.h>
#include <linux/kthread.h>
#include <linux/wait.h>
```

#### 3b. Restructure `__bpf_spin_lock_irqsave()`

**Current code** (lines ~499-505):
```c
static inline void __bpf_spin_lock_irqsave(struct bpf_spin_lock *lock)
{
	unsigned long flags;
	local_irq_save(flags);
	__bpf_spin_lock(lock);
	__this_cpu_write(irqsave_flags, flags);
}
```

**New code:**
```c
static inline void __bpf_spin_lock_irqsave(struct bpf_spin_lock *lock)
{
	unsigned long flags;

#ifdef CONFIG_QUEUED_SPINLOCKS
	/*
	 * Use the custom qspinlock path: acquires the lock with IRQs
	 * enabled (so the waiter's hrtimer can fire), then disables
	 * IRQs after acquisition.
	 */
	preempt_disable();
	bpf_qspinlock_lock((struct qspinlock *)lock, &flags);
	/* lock is now held, IRQs just disabled by bpf_qspinlock_lock */
#else
	local_irq_save(flags);
	__bpf_spin_lock(lock);
#endif
	__this_cpu_write(irqsave_flags, flags);
}
```

**Note:** For `CONFIG_QUEUED_SPINLOCKS`, `preempt_disable()` is done here
(before the qspinlock path), while `local_irq_save` is done **inside**
`bpf_qspinlock_lock` after acquisition. For the non-qspinlock fallback, the
old behavior is preserved.

Also restructure the original `__bpf_spin_lock` to remove the redundant
`preempt_disable` when `CONFIG_QUEUED_SPINLOCKS` is set (since the irqsave
wrapper handles it):

```c
#if defined(CONFIG_QUEUED_SPINLOCKS) || defined(CONFIG_BPF_ARCH_SPINLOCK)
static inline void __bpf_spin_lock(struct bpf_spin_lock *lock)
{
	arch_spinlock_t *l = (void *)lock;
	/* ... same BUILD_BUG_ON checks ... */
	/* Note: preempt_disable is handled by the caller (__bpf_spin_lock_irqsave).
	 * This function is now only used by the non-qspinlock fallback path.
	 */
	preempt_disable();
	arch_spin_lock(l);
}
#endif
```

#### 3c. Remove hrtimer start from `bpf_spin_lock` helper

**Current code** (lines ~541-552):
```c
if (cnt == 1 && READ_ONCE(sysctl_bpf_spin_lock_timeout) > 0) {
    struct hrtimer *timer = this_cpu_ptr(&lock_watchdog_timer);
    ktime_t timeout_ms = ms_to_ktime(READ_ONCE(sysctl_bpf_spin_lock_timeout));

    /* Reset timeout flag */
    WRITE_ONCE(ebpf_spinlock_timeout, 0);

    hrtimer_setup(timer, bpf_spin_lock_timer_cb, CLOCK_MONOTONIC, HRTIMER_MODE_REL | HRTIMER_MODE_HARD);
    hrtimer_start(timer, timeout_ms, HRTIMER_MODE_REL | HRTIMER_MODE_HARD);
}
```

**Replace with:**
```c
if (cnt == 1) {
    /* Reset timeout flag for fresh critical section */
    WRITE_ONCE(ebpf_spinlock_timeout, 0);
    /* Timer is already started by:
     *  - The waiter at head of MCS queue (contended case), or
     *  - The kthread, notified from the fast path (uncontended case).
     * Nothing to do here.
     */
}
```

The existing `lock_watchdog_timer` per-CPU hrtimer and `bpf_spin_lock_timer_cb`
in helpers.c can be **removed** (they are superseded by `bpf_waiter_hrtimer` +
`bpf_qspinlock_timer_cb` in `bpf_qspinlock.c`).

#### 3d. Modify `bpf_spin_unlock` — cancel via active timer pointer

**Current code** (lines ~615-633):
```c
if (found && this_cpu_read(held_locks_cnt) == 0) {
    this_cpu_write(bpf_undo_log_cnt, 0);
    if (READ_ONCE(sysctl_bpf_spin_lock_timeout) > 0) {
        struct hrtimer *timer = this_cpu_ptr(&lock_watchdog_timer);
        hrtimer_cancel(timer);
        WRITE_ONCE(ebpf_spinlock_timeout, 0);
    }
}
```

**Replace with:**
```c
if (found && this_cpu_read(held_locks_cnt) == 0) {
    this_cpu_write(bpf_undo_log_cnt, 0);

    /* Cancel whichever timer is active for this CPU's lock session.
     * This handles both the waiter-started timer and the kthread-started timer.
     */
    struct bpf_lock_timer *active = this_cpu_read(bpf_active_timer);
    if (active) {
        bpf_lock_timer_cancel(active);
        this_cpu_write(bpf_active_timer, NULL);
    }
    WRITE_ONCE(ebpf_spinlock_timeout, 0);
}
```

This uses the per-CPU `bpf_active_timer` pointer (defined in `bpf_qspinlock.c`,
exported) to cancel whatever timer instance was started. The
`hrtimer_cancel()` is safe to call cross-CPU (when the kthread started the
timer on a different CPU).

#### 3e. Clean up `__bpf_spin_unlock_irqrestore`

No structural changes needed. The existing code:
```c
static inline void __bpf_spin_unlock_irqrestore(struct bpf_spin_lock *lock)
{
	unsigned long flags;
	flags = __this_cpu_read(irqsave_flags);
	__bpf_spin_unlock(lock);
	local_irq_restore(flags);
}
```

This still works: `__bpf_spin_unlock` calls `arch_spin_unlock` which is
`queued_spin_unlock` → `smp_store_release(&lock->locked, 0)`. Then
`local_irq_restore` re-enables IRQs. Order is correct: unlock while IRQs
are still off, then restore IRQs.

#### 3f. Remove debug printks

Remove all the `printk(KERN_ALERT ...)` calls in `bpf_spin_lock` and
`bpf_spin_unlock`. They are development artifacts.

---

### Step 4: Implement the global kthread — in `kernel/bpf/helpers.c`

The kthread handles timeout for the **uncontended** case where the lock is
acquired via the fast path (no waiter exists to start the timer).

#### 4a. Add kthread state variables

```c
static struct task_struct *bpf_lock_timeout_kthread;
static DECLARE_WAIT_QUEUE_HEAD(bpf_lock_timeout_wq);
static atomic_t bpf_lock_timeout_pending = ATOMIC_INIT(0);

/* The kthread's own timer instance */
static struct hrtimer bpf_kthread_hrtimer;
static struct bpf_lock_timer bpf_kthread_timer;
```

#### 4b. Implement `bpf_notify_lock_kthread()`

Called from `bpf_qspinlock_lock()` fast path (Step 2d) when lock is acquired
without contention:

```c
void bpf_notify_lock_kthread(void)
{
	if (READ_ONCE(sysctl_bpf_spin_lock_timeout) <= 0)
		return;

	/* Record that the kthread's timer is the active timer for this CPU */
	this_cpu_write(bpf_active_timer, &bpf_kthread_timer);

	/* One-way notification: set flag and wake kthread. No synchronization
	 * needed the other way — the kthread just starts the timer and sleeps.
	 */
	atomic_set(&bpf_lock_timeout_pending, 1);
	wake_up(&bpf_lock_timeout_wq);
}
EXPORT_SYMBOL_GPL(bpf_notify_lock_kthread);
```

#### 4c. Implement `bpf_lock_timeout_kthread_fn()`

```c
static int bpf_lock_timeout_kthread_fn(void *data)
{
	while (!kthread_should_stop()) {
		wait_event_interruptible(bpf_lock_timeout_wq,
					atomic_read(&bpf_lock_timeout_pending) ||
					kthread_should_stop());

		if (kthread_should_stop())
			break;

		if (atomic_cmpxchg(&bpf_lock_timeout_pending, 1, 0) == 1) {
			u64 timeout_ns = (u64)READ_ONCE(sysctl_bpf_spin_lock_timeout)
					 * NSEC_PER_MSEC;
			bpf_lock_timer_start(&bpf_kthread_timer, timeout_ns);
		}
	}
	return 0;
}
```

#### 4d. Kthread initialization

Add to a `late_initcall` function in helpers.c (or create a new one):

```c
static int __init bpf_lock_kthread_init(void)
{
	/* Initialize kthread's hrtimer */
	hrtimer_setup(&bpf_kthread_hrtimer, bpf_qspinlock_timer_cb,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL | HRTIMER_MODE_HARD);
	bpf_kthread_timer.ops = &bpf_hrtimer_ops;
	bpf_kthread_timer.ctx = &bpf_kthread_hrtimer;

	/* Initialize per-CPU waiter timers */
	bpf_qspinlock_init_timers();

	/* Start the kthread */
	bpf_lock_timeout_kthread = kthread_run(bpf_lock_timeout_kthread_fn,
					       NULL, "bpf_lock_wd");
	if (IS_ERR(bpf_lock_timeout_kthread)) {
		pr_err("Failed to create bpf_lock_wd kthread\n");
		bpf_lock_timeout_kthread = NULL;
		return PTR_ERR(bpf_lock_timeout_kthread);
	}
	return 0;
}
late_initcall(bpf_lock_kthread_init);
```

Note: `bpf_qspinlock_timer_cb` and `bpf_hrtimer_ops` are defined in
`bpf_qspinlock.c` and need to be either exported or moved to helpers.c.
The cleanest approach is to keep the hrtimer callback and ops in helpers.c
(since helpers.c already owns `ebpf_spinlock_timeout` and the kthread), and
have `bpf_qspinlock.c` reference them via extern. **Decision**: put the
hrtimer callback, ops struct, and kthread in `helpers.c`; put only the
qspinlock slow path logic in `bpf_qspinlock.c`.

---

### Step 5: Modify `kernel/bpf/Makefile`

Add `bpf_qspinlock.o` to the build, gated on `CONFIG_QUEUED_SPINLOCKS`:

**Current line 16:**
```makefile
obj-$(CONFIG_BPF_SYSCALL) += btf.o memalloc.o rqspinlock.o stream.o
```

**Changed to:**
```makefile
obj-$(CONFIG_BPF_SYSCALL) += btf.o memalloc.o rqspinlock.o stream.o
obj-$(CONFIG_QUEUED_SPINLOCKS) += bpf_qspinlock.o
```

Also add the ftrace removal (same as rqspinlock):
```makefile
CFLAGS_REMOVE_bpf_qspinlock.o = $(CC_FLAGS_FTRACE)
```

---

### Step 6: Modify `kernel/bpf/Kconfig` (optional)

No new Kconfig symbol is strictly required. The existing
`sysctl_bpf_spin_lock_timeout` sysctl controls the timeout value at runtime.
If desired, a compile-time default can be added later.

---

## Execution Order Summary

| Order | File | What to do |
|-------|------|------------|
| 1 | `include/linux/bpf_lock_timer.h` | Create the timer ops abstraction header |
| 2 | `kernel/bpf/bpf_qspinlock.c` | Create the custom qspinlock slow path with per-CPU hrtimer per waiter |
| 3 | `kernel/bpf/helpers.c` | Restructure `__bpf_spin_lock_irqsave` to use qspinlock path; remove old hrtimer start from `bpf_spin_lock`; update `bpf_spin_unlock` to cancel via `bpf_active_timer`; add kthread + notification; add init function; remove debug printks |
| 4 | `kernel/bpf/Makefile` | Add `bpf_qspinlock.o` |

---

## Data Flow Diagram

```
CONTENDED CASE (waiter exists):
=================================
CPU A (owner)                    CPU B (waiter, head of MCS queue)
─────────────                    ────────────────────────────────
bpf_spin_lock()                  bpf_spin_lock()
  └─ fast path succeeds           └─ fast path fails
  └─ local_irq_save()               └─ enters bpf_queued_spin_lock_slowpath()
  └─ enters BPF program               └─ enqueues in MCS queue
       (loops via bpf_loop)            └─ becomes head of queue
       ...                             └─ STARTS hrtimer (IRQs enabled!)
       ...                             └─ spins on lock->val
       ...                             ...
                                       [hrtimer fires on CPU B]
                                       └─ callback sets ebpf_spinlock_timeout = 1
       ...                             ...
  [bpf_loop iteration]                └─ still spinning...
  └─ checks ebpf_spinlock_timeout
  └─ calls bpf_spin_lock_timeout_handler()
     └─ undo log replay
     └─ releases all locks ───────────► lock->locked = 0
     └─ bpf_die()                      └─ spin condition satisfied!
                                       └─ CANCELS hrtimer
                                       └─ local_irq_save()
                                       └─ claims lock
                                       └─ returns to BPF program


UNCONTENDED CASE (no waiter):
==============================
CPU A (owner)                    bpf_lock_wd kthread (any CPU)
─────────────                    ──────────────────────────────
bpf_spin_lock()                  sleeping on bpf_lock_timeout_wq
  └─ fast path succeeds
  └─ local_irq_save()
  └─ bpf_notify_lock_kthread()
     └─ sets pending flag ────────────► wakes up
     └─ enters BPF program             └─ STARTS hrtimer
       (loops via bpf_loop)            └─ goes back to sleep
       ...
                                       [hrtimer fires]
                                       └─ callback sets ebpf_spinlock_timeout = 1
       ...
  [bpf_loop iteration]
  └─ checks ebpf_spinlock_timeout
  └─ calls bpf_spin_lock_timeout_handler()
     └─ undo log replay
     └─ releases all locks
     └─ bpf_die()
```

---

## Verification Plan

1. **Compile test:** `make -j$(nproc)` with `CONFIG_QUEUED_SPINLOCKS=y` and
   `CONFIG_BPF_SYSCALL=y`. Verify no build errors.

2. **Existing selftests:** Run `test_spin_lock_loop_timeout` — the timeout
   should now actually fire (previously silently defeated).

3. **Contended test:** Two CPUs contend on same BPF lock. Owner loops forever
   in `bpf_loop`. Waiter at queue head starts hrtimer. After
   `sysctl_bpf_spin_lock_timeout` ms → owner terminated → waiter acquires.

4. **Uncontended test:** Single CPU acquires lock and loops forever. Kthread
   starts hrtimer. After timeout → owner terminated.

5. **Normal path regression:** Lock acquired + released normally should
   work identically (timer started then cancelled before it fires).

6. **Full BPF selftests:** `./test_progs -t spin_lock` — no regressions.

---

## Open Questions / Future Work

- **Kthread latency:** there is inherent latency between `wake_up()` and the
  kthread actually running & starting the timer. For short timeouts this could
  be noticeable. Acceptable for now since the timeout is in the 50-250ms range.
- **Multiple CPUs uncontended:** if two CPUs both acquire BPF locks
  uncontended simultaneously, they both notify the same kthread. The kthread
  will start the timer once. The second notification may be lost. This is
  acceptable — it's best-effort for the uncommon case. The common case under
  contention is precisely handled by the waiter path.
- **ktime/jiffies backends:** the `bpf_lock_timer_ops` abstraction is ready
  for alternative backends but only the hrtimer backend is implemented now.
