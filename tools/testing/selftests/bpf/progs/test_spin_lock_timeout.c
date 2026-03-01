// SPDX-License-Identifier: GPL-2.0
// Test BPF spin lock timeout and cancellation mechanism
//#include <linux/bpf.h>
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include "bpf_experimental.h"

/* Shared locks for AB-BA deadlock testing */
struct lock_pair {
	struct bpf_spin_lock lock;
	int value;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 2);
	__type(key, int);
	__type(value, struct lock_pair);
} shared_locks SEC(".maps");

/* Test 1: Nested locking (A -> B) */
SEC("tc")
int test_nested_locking(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&shared_locks, &key);
	if (!a)
		return 0;

	key = 1;
	b = bpf_map_lookup_elem(&shared_locks, &key);
	if (!b)
		return 0;

	/* Lock A, then B */
	bpf_spin_lock(&a->lock);
	bpf_spin_lock(&b->lock);
	a->value++;
	b->value++;
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* Test 2: OOO (Out-of-Order) unlocking */
SEC("tc")
int test_ooo_unlocking(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&shared_locks, &key);
	if (!a)
		return 0;

	key = 1;
	b = bpf_map_lookup_elem(&shared_locks, &key);
	if (!b)
		return 0;

	/* Lock A, then B */
	bpf_spin_lock(&a->lock);
	bpf_spin_lock(&b->lock);
	a->value++;
	b->value++;

	/* Unlock A first (out of order), then B */
	bpf_spin_unlock(&a->lock);
	bpf_spin_unlock(&b->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Shared infinite-spin helpers (trigger timeout)                      */
/* ------------------------------------------------------------------ */

#define LOOP_CNT (1 << 10)

static int cb4(void *ctx) { return 0; }
static int cb3(void *ctx) { bpf_loop(LOOP_CNT, cb4, NULL, 0); return 0; }
static int cb2(void *ctx) { bpf_loop(LOOP_CNT, cb3, NULL, 0); return 0; }
static int cb1(void *ctx) { bpf_loop(LOOP_CNT, cb2, NULL, 0); return 0; }

/* Test 3: Timeout trigger via bpf_loop */
SEC("tc")
int test_timeout_trigger(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a;
	volatile long sum = 0;

	a = bpf_map_lookup_elem(&shared_locks, &key);
	if (!a)
		return 0;

	bpf_spin_lock(&a->lock);	
	bpf_printk("Acquired the lock\n");
	sum++;
    bpf_loop(LOOP_CNT, cb1, NULL, 0); // Trigger Timeout
	bpf_spin_unlock(&a->lock);

	bpf_printk("Total sum: %ld\n", sum);

	return 0;
}

/* Test 4: AB-BA deadlock scenario - Program 1 (Lock A -> B) */
SEC("tc")
int test_deadlock_prog1(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&shared_locks, &key);
	if (!a)
		return 0;

	key = 1;
	b = bpf_map_lookup_elem(&shared_locks, &key);
	if (!b)
		return 0;

	/* Lock A, then B */
	bpf_spin_lock(&a->lock);
    bpf_loop(LOOP_CNT, cb2, NULL, 0);
    a->value++;
	bpf_spin_lock(&b->lock);

	a->value++;
	b->value++;

	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);

	return 0;
}

/* Test 5: AB-BA deadlock scenario - Program 2 (Lock B -> A) */
SEC("tc")
int test_deadlock_prog2(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *a, *b;

	a = bpf_map_lookup_elem(&shared_locks, &key);
	if (!a)
		return 0;

	key = 1;
	b = bpf_map_lookup_elem(&shared_locks, &key);
	if (!b)
		return 0;

	/* Lock B, then A */
	bpf_spin_lock(&b->lock);
    bpf_loop(LOOP_CNT, cb2, NULL, 0);
	b->value++;
	bpf_spin_lock(&a->lock);

	a->value++;
	b->value++;

	bpf_spin_unlock(&a->lock);
	bpf_spin_unlock(&b->lock);

	return 0;
}

char _license[] SEC("license") = "GPL";
