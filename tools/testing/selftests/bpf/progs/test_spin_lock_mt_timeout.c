// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-threaded BPF spin_lock timeout + undo-log tests (BPF side).
 *
 * These programs are driven concurrently from multiple pinned userspace
 * threads (see prog_tests/spin_lock_mt_timeout.c) to exercise the genuinely
 * concurrent kernel paths that the single-threaded tests cannot reach:
 *
 *   - the contended qspinlock slow path (MCS queue, pending/queued waiters)
 *   - the contended-timeout path where the *waiter* at the head of the queue
 *     detects the timeout and terminates the *owner* running on another CPU
 *   - nested locking where the inner lock is contended and its holder times out
 *   - concurrent / repeated timeouts stressing the per-CPU timer + cancel logic
 *
 * Map layout note: lock entry 0 is "lock A", entry 1 is "lock B".  Each entry
 * carries an independent @value (written-then-rolled-back by holders) and an
 * independent @counter (incremented-and-committed by short critical sections).
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

struct lock_pair {
	struct bpf_spin_lock lock;
	__u64 value;
	__u64 counter;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 4);
	__type(key, int);
	__type(value, struct lock_pair);
} mt_locks SEC(".maps");

/* Distinct values written inside critical sections; userspace pre-fills the
 * map with different sentinels so a rollback is observable as value==sentinel.
 */
#define A_WRITE_VAL	0xDEADBEEFDEADBEEFULL
#define B_WRITE_VAL	0xFEEDFACEFEEDFACEULL

/* ------------------------------------------------------------------ */
/* Shared spin helpers                                                  */
/* ------------------------------------------------------------------ */

#define LOOP_CNT (1 << 10)

/* Nested loops -> ~1024^4 iterations: guaranteed to exceed any sane timeout.
 * Each bpf_loop iteration re-checks ebpf_spinlock_timeout, so the program is
 * terminated promptly once the flag is set.
 */
static int cb4(void *ctx) { return 0; }
static int cb3(void *ctx) { bpf_loop(LOOP_CNT, cb4, NULL, 0); return 0; }
static int cb2(void *ctx) { bpf_loop(LOOP_CNT, cb3, NULL, 0); return 0; }
static int cb1(void *ctx) { bpf_loop(LOOP_CNT, cb2, NULL, 0); return 0; }

/* ------------------------------------------------------------------ */
/* Long-running holders (expected to be terminated by the timeout)      */
/* ------------------------------------------------------------------ */

/* Hold lock A, dirty A.value, then spin past the timeout.  On timeout the
 * undo log rolls A.value back and lock A is released before termination.
 */
SEC("tc")
int holder_loop_write_a(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;

	a = bpf_map_lookup_elem(&mt_locks, &key);
	if (!a)
		return 0;

	bpf_spin_lock(&a->lock);
	a->value = A_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* trigger timeout */
	bpf_spin_unlock(&a->lock);		/* not reached on timeout */
	return 0;
}

/* Nested holder: acquire A (outer) then B (inner), dirty both, then spin past
 * the timeout.  Used to verify that a timeout while holding nested locks rolls
 * back writes under *both* locks and releases *both*.  When another thread is
 * contending lock B, this also exercises the inner-lock waiter path.
 */
SEC("tc")
int nested_inner_holder(struct __sk_buff *ctx)
{
	int ka = 0, kb = 1;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&mt_locks, &ka);
	if (!a)
		return 0;
	b = bpf_map_lookup_elem(&mt_locks, &kb);
	if (!b)
		return 0;

	bpf_spin_lock(&a->lock);
	bpf_spin_lock(&b->lock);
	a->value = A_WRITE_VAL;
	b->value = B_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* trigger timeout */
	bpf_spin_unlock(&b->lock);		/* not reached on timeout */
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Short critical sections (expected to commit)                         */
/* ------------------------------------------------------------------ */

/* Lock A, bump A.counter, unlock.  No loop -> never checks the timeout flag,
 * so these always commit; used as contenders/waiters on lock A.
 */
SEC("tc")
int short_cs_incr_a(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;

	a = bpf_map_lookup_elem(&mt_locks, &key);
	if (!a)
		return 0;

	bpf_spin_lock(&a->lock);
	a->counter++;
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* Lock B, bump B.counter, unlock.  Waiter/contender on lock B. */
SEC("tc")
int short_cs_incr_b(struct __sk_buff *ctx)
{
	int key = 1;
	struct lock_pair *b;

	b = bpf_map_lookup_elem(&mt_locks, &key);
	if (!b)
		return 0;

	bpf_spin_lock(&b->lock);
	b->counter++;
	bpf_spin_unlock(&b->lock);
	return 0;
}

/* Independent worker on lock B that DOES run a bounded loop.  The loop is far
 * shorter than the timeout so this program should always commit on its own.
 * But because it polls ebpf_spinlock_timeout each iteration, it is a victim of
 * the global-flag limitation: a timeout raised by an unrelated lock-A holder on
 * another CPU can falsely terminate it mid-loop and roll back its counter bump.
 */
SEC("tc")
int worker_b_short_loop(struct __sk_buff *ctx)
{
	int key = 1;
	struct lock_pair *b;

	b = bpf_map_lookup_elem(&mt_locks, &key);
	if (!b)
		return 0;

	bpf_spin_lock(&b->lock);
	b->counter++;
	bpf_loop(8192, cb4, NULL, 0);	/* short: checks the flag, won't self-timeout */
	bpf_spin_unlock(&b->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* AB-BA pair                                                           */
/* ------------------------------------------------------------------ */
/*
 * Each program grabs its first lock, dirties it, then spins past the timeout
 * *before* attempting the second lock.  The timeout therefore fires (and rolls
 * back the first write) before the cross-acquire can complete.  This is
 * deliberate: a fully-formed AB-BA deadlock parks both threads in the kernel
 * qspinlock slow path where neither runs bpf_loop, so the timeout machinery
 * (which only fires from bpf_loop/bpf_iter) could not break it.  Here we
 * validate concurrent rollback of two independent locks instead.
 */
SEC("tc")
int deadlock_ab(struct __sk_buff *ctx)
{
	int ka = 0, kb = 1;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&mt_locks, &ka);
	if (!a)
		return 0;
	b = bpf_map_lookup_elem(&mt_locks, &kb);
	if (!b)
		return 0;

	bpf_spin_lock(&a->lock);
	a->value = A_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* timeout fires here */
	bpf_spin_lock(&b->lock);		/* not reached on timeout */
	b->value = B_WRITE_VAL;
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

SEC("tc")
int deadlock_ba(struct __sk_buff *ctx)
{
	int ka = 0, kb = 1;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&mt_locks, &ka);
	if (!a)
		return 0;
	b = bpf_map_lookup_elem(&mt_locks, &kb);
	if (!b)
		return 0;

	bpf_spin_lock(&b->lock);
	b->value = B_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* timeout fires here */
	bpf_spin_lock(&a->lock);		/* not reached on timeout */
	a->value = A_WRITE_VAL;
	bpf_spin_unlock(&a->lock);
	bpf_spin_unlock(&b->lock);
	return 0;
}

/* ================================================================== */
/* bpf_lock_func() variants of all of the above                        */
/*                                                                      */
/* bpf_lock_func(&lock, cb, local_state) acquires @lock, runs cb() with */
/* it held, then releases it.  Writes inside cb are undo-logged exactly  */
/* like a plain bpf_spin_lock() critical section, so the timeout handler */
/* rolls them back on abort.  bpf_lock_func() cannot be nested, so the   */
/* nested/inner-lock cases take the inner lock with plain bpf_spin_lock()*/
/* inside the callback (a different lock, which the verifier permits).   */
/* ================================================================== */

/* ---- holder on lock A: dirty A.value then spin past the timeout ---- */
static int lf_holder_a_cb(void *ctx)
{
	int key = 0;
	struct lock_pair *a = bpf_map_lookup_elem(&mt_locks, &key);

	if (a) {
		a->value = A_WRITE_VAL;
		bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* trigger timeout */
	}
	return 0;
}

SEC("tc")
int lf_holder_loop_write_a(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;

	a = bpf_map_lookup_elem(&mt_locks, &key);
	if (!a)
		return 0;
	bpf_lock_func(&a->lock, lf_holder_a_cb, 0, 0, 0);
	return 0;
}

/* ---- short critical sections: bump a counter and commit ---- */
static int lf_incr_a_cb(void *ctx)
{
	int key = 0;
	struct lock_pair *a = bpf_map_lookup_elem(&mt_locks, &key);

	if (a)
		a->counter++;
	return 0;
}

SEC("tc")
int lf_short_cs_incr_a(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;

	a = bpf_map_lookup_elem(&mt_locks, &key);
	if (!a)
		return 0;
	bpf_lock_func(&a->lock, lf_incr_a_cb, 0, 0, 0);
	return 0;
}

static int lf_incr_b_cb(void *ctx)
{
	int key = 1;
	struct lock_pair *b = bpf_map_lookup_elem(&mt_locks, &key);

	if (b)
		b->counter++;
	return 0;
}

SEC("tc")
int lf_short_cs_incr_b(struct __sk_buff *ctx)
{
	int key = 1;
	struct lock_pair *b;

	b = bpf_map_lookup_elem(&mt_locks, &key);
	if (!b)
		return 0;
	bpf_lock_func(&b->lock, lf_incr_b_cb, 0, 0, 0);
	return 0;
}

/* ---- independent lock-B worker with a bounded (committing) loop ---- */
static int lf_workerb_cb(void *ctx)
{
	int key = 1;
	struct lock_pair *b = bpf_map_lookup_elem(&mt_locks, &key);

	if (b) {
		b->counter++;
		bpf_loop(8192, cb4, NULL, 0);	/* short: checks flag, won't self-timeout */
	}
	return 0;
}

SEC("tc")
int lf_worker_b_short_loop(struct __sk_buff *ctx)
{
	int key = 1;
	struct lock_pair *b;

	b = bpf_map_lookup_elem(&mt_locks, &key);
	if (!b)
		return 0;
	bpf_lock_func(&b->lock, lf_workerb_cb, 0, 0, 0);
	return 0;
}

/* ---- nested: outer lock via bpf_lock_func, inner via bpf_spin_lock ---- */
static int lf_nested_cb(void *ctx)
{
	int ka = 0, kb = 1;
	struct lock_pair *a = bpf_map_lookup_elem(&mt_locks, &ka);
	struct lock_pair *b;

	if (!a)
		return 0;
	b = bpf_map_lookup_elem(&mt_locks, &kb);
	if (!b)
		return 0;

	a->value = A_WRITE_VAL;			/* under outer lock A (lock_func) */
	bpf_spin_lock(&b->lock);		/* inner lock B */
	b->value = B_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* trigger timeout */
	bpf_spin_unlock(&b->lock);		/* not reached on timeout */
	return 0;
}

SEC("tc")
int lf_nested_inner_holder(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;

	a = bpf_map_lookup_elem(&mt_locks, &key);
	if (!a)
		return 0;
	bpf_lock_func(&a->lock, lf_nested_cb, 0, 0, 0);
	return 0;
}

/* ---- AB-BA pair: first lock via bpf_lock_func, would-be second via
 *      bpf_spin_lock after the (timing-out) loop (never reached) ----
 */
static int lf_dlab_cb(void *ctx)
{
	int ka = 0, kb = 1;
	struct lock_pair *a = bpf_map_lookup_elem(&mt_locks, &ka);
	struct lock_pair *b;

	if (!a)
		return 0;
	b = bpf_map_lookup_elem(&mt_locks, &kb);
	if (!b)
		return 0;

	a->value = A_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* timeout fires here */
	bpf_spin_lock(&b->lock);		/* not reached on timeout */
	b->value = B_WRITE_VAL;
	bpf_spin_unlock(&b->lock);
	return 0;
}

SEC("tc")
int lf_deadlock_ab(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;

	a = bpf_map_lookup_elem(&mt_locks, &key);
	if (!a)
		return 0;
	bpf_lock_func(&a->lock, lf_dlab_cb, 0, 0, 0);
	return 0;
}

static int lf_dlba_cb(void *ctx)
{
	int ka = 0, kb = 1;
	struct lock_pair *a = bpf_map_lookup_elem(&mt_locks, &ka);
	struct lock_pair *b;

	if (!a)
		return 0;
	b = bpf_map_lookup_elem(&mt_locks, &kb);
	if (!b)
		return 0;

	b->value = B_WRITE_VAL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* timeout fires here */
	bpf_spin_lock(&a->lock);		/* not reached on timeout */
	a->value = A_WRITE_VAL;
	bpf_spin_unlock(&a->lock);
	return 0;
}

SEC("tc")
int lf_deadlock_ba(struct __sk_buff *ctx)
{
	int key = 1;
	struct lock_pair *b;

	b = bpf_map_lookup_elem(&mt_locks, &key);
	if (!b)
		return 0;
	bpf_lock_func(&b->lock, lf_dlba_cb, 0, 0, 0);
	return 0;
}

char _license[] SEC("license") = "GPL";
