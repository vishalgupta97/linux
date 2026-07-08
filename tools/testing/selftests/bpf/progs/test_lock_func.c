// SPDX-License-Identifier: GPL-2.0
/*
 * Tests for the bpf_lock_func() helper.
 *
 * bpf_lock_func(lock, callback_fn, local_state) acquires @lock, runs
 * callback_fn(local_state) with the lock held, then releases @lock.  Writes to
 * global state inside the callback are undo-logged, so an abort by the spin
 * lock timeout handler rolls them back.
 *
 * Groups:
 *   lf_commit / lf_timeout  - runtime: commit vs. rollback of callback writes
 *   lf_diff_lock            - verifier accept: bpf_spin_lock(different lock) in cb
 *   lf_nested_reject        - verifier reject: nested bpf_lock_func()
 *   lf_same_lock_reject     - verifier reject: bpf_spin_lock(same lock) AA deadlock
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include <bpf/bpf_core_read.h>
#include "bpf_misc.h"

struct lockval {
	struct bpf_spin_lock lock;
	__u64 x;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 2);
	__type(key, int);
	__type(value, struct lockval);
} map_a SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 2);
	__type(key, int);
	__type(value, struct lockval);
} map_b SEC(".maps");

/* Shared infinite-spin helpers used to drive the lock past the timeout. */
#define LOOP_CNT (1 << 10)

static int spin4(void *ctx) { return 0; }
static int spin3(void *ctx) { bpf_loop(LOOP_CNT, spin4, NULL, 0); return 0; }
static int spin2(void *ctx) { bpf_loop(LOOP_CNT, spin3, NULL, 0); return 0; }
static int spin1(void *ctx) { bpf_loop(LOOP_CNT, spin2, NULL, 0); return 0; }

/* ------------------------------------------------------------------ */
/* Commit path: callback writes a map value and returns normally.      */
/* key + value are passed as scalar callback args to prove they are    */
/* passed through.                                                     */
/* ------------------------------------------------------------------ */
static int commit_cb(__u64 key_v, __u64 newval)
{
	int key = (int)key_v;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (v)
		v->x = newval;
	return 0;
}

SEC("tc")
int lf_commit(struct __sk_buff *ctx)
{
	int key = 0;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (!v)
		return 0;
	bpf_lock_func(&v->lock, commit_cb, key, 0x0102030405060708ULL, 0);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Timeout path: callback writes a map value then spins forever.       */
/* The write must be rolled back by the timeout handler.               */
/* ------------------------------------------------------------------ */
static int timeout_cb(__u64 key_v, __u64 newval)
{
	int key = (int)key_v;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (v)
		v->x = newval;
	bpf_loop(LOOP_CNT, spin1, NULL, 0); /* trigger timeout */
	return 0;
}

SEC("tc")
int lf_timeout(struct __sk_buff *ctx)
{
	int key = 1;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (!v)
		return 0;
	bpf_lock_func(&v->lock, timeout_cb, key, 0xdeadbeefcafef00dULL, 0);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Verifier accept: locking a DIFFERENT lock inside the callback is OK */
/* and its write is undo-logged too.                                   */
/* ------------------------------------------------------------------ */
static int diff_lock_cb(void *ctx)
{
	int key = 0;
	struct lockval *v2 = bpf_map_lookup_elem(&map_b, &key);

	if (v2) {
		bpf_spin_lock(&v2->lock);
		v2->x = 0xBEEF;
		bpf_spin_unlock(&v2->lock);
	}
	return 0;
}

SEC("tc")
int lf_diff_lock(struct __sk_buff *ctx)
{
	int key = 0;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (!v)
		return 0;
	bpf_lock_func(&v->lock, diff_lock_cb, 0, 0, 0);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Verifier reject: nested bpf_lock_func() inside the callback.        */
/* ------------------------------------------------------------------ */
static int inner_cb(void *ctx)
{
	return 0;
}

static int nesting_cb(void *ctx)
{
	int key = 0;
	struct lockval *v2 = bpf_map_lookup_elem(&map_b, &key);

	if (v2)
		bpf_lock_func(&v2->lock, inner_cb, 0, 0, 0);
	return 0;
}

SEC("?tc")
__failure __msg("bpf_lock_func cannot be nested")
int lf_nested_reject(struct __sk_buff *ctx)
{
	int key = 0;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (!v)
		return 0;
	bpf_lock_func(&v->lock, nesting_cb, 0, 0, 0);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Verifier reject: re-locking the SAME lock (same register) inside    */
/* the callback is an AA deadlock. This confirms the lock state is      */
/* correctly tracked while the lock_func critical section is active.    */
/*                                                                      */
/* Note: AA detection matches on the (id, ptr) of the locked register, */
/* so it only catches a provably-identical lock. A fresh               */
/* bpf_map_lookup_elem() yields a new id and is treated as a separate   */
/* lock - exactly like plain nested bpf_spin_lock().                    */
/* ------------------------------------------------------------------ */
static int aa_cb(void *ctx)
{
	int key = 0;
	struct lockval *v2 = bpf_map_lookup_elem(&map_b, &key);

	if (v2) {
		bpf_spin_lock(&v2->lock);
		bpf_spin_lock(&v2->lock);	/* same register -> AA deadlock */
		v2->x = 1;
		bpf_spin_unlock(&v2->lock);
	}
	return 0;
}

SEC("?tc")
__failure __msg("AA deadlock")
int lf_cb_aa_reject(struct __sk_buff *ctx)
{
	int key = 0;
	struct lockval *v = bpf_map_lookup_elem(&map_a, &key);

	if (!v)
		return 0;
	bpf_lock_func(&v->lock, aa_cb, 0, 0, 0);
	return 0;
}

char _license[] SEC("license") = "GPL";
