// SPDX-License-Identifier: GPL-2.0
// Test BPF spin lock timeout and cancellation mechanism
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

/* Shared locks for AB-BA deadlock testing */
struct lock_pair {
	struct bpf_spin_lock lock_a;
	struct bpf_spin_lock lock_b;
	int value;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct lock_pair);
} shared_locks SEC(".maps");

/* Test 1: Nested locking (A -> B) */
SEC("tc")
int test_nested_locking(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *locks;

	locks = bpf_map_lookup_elem(&shared_locks, &key);
	if (!locks)
		return 0;

	bpf_spin_lock(&locks->lock_a);
	bpf_spin_lock(&locks->lock_b);
	locks->value++;
	bpf_spin_unlock(&locks->lock_b);
	bpf_spin_unlock(&locks->lock_a);

	return 0;
}

/* Test 2: OOO (Out-of-Order) unlocking */
SEC("tc")
int test_ooo_unlocking(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *locks;

	locks = bpf_map_lookup_elem(&shared_locks, &key);
	if (!locks)
		return 0;

	/* Lock A, then B */
	bpf_spin_lock(&locks->lock_a);
	bpf_spin_lock(&locks->lock_b);
	
	/* Unlock A first (out of order), then B */
	bpf_spin_unlock(&locks->lock_a);
	bpf_spin_unlock(&locks->lock_b);

	return 0;
}

/* Test 3: Timeout trigger via bpf_loop */
SEC("tc")
int test_timeout_trigger(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *locks;
	unsigned long i;

	locks = bpf_map_lookup_elem(&shared_locks, &key);
	if (!locks)
		return 0;

	bpf_spin_lock(&locks->lock_a);
	
	/* Busy loop to trigger timeout */
	for (i = 0; i < 0xFFFFFFF; i++) {
		locks->value++;
	}
	
	bpf_spin_unlock(&locks->lock_a);

	return 0;
}

/* Test 4: AB-BA deadlock scenario - Program 1 (Lock A -> B) */
SEC("tc")
int test_deadlock_prog1(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *locks;
	unsigned long i;

	locks = bpf_map_lookup_elem(&shared_locks, &key);
	if (!locks)
		return 0;

	bpf_spin_lock(&locks->lock_a);
	
	/* Small delay */
	for (i = 0; i < 1000; i++)
		barrier();
	
	bpf_spin_lock(&locks->lock_b);
	locks->value++;
	bpf_spin_unlock(&locks->lock_b);
	bpf_spin_unlock(&locks->lock_a);

	return 0;
}

/* Test 5: AB-BA deadlock scenario - Program 2 (Lock B -> A) */
SEC("tc")
int test_deadlock_prog2(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_pair *locks;
	unsigned long i;

	locks = bpf_map_lookup_elem(&shared_locks, &key);
	if (!locks)
		return 0;

	bpf_spin_lock(&locks->lock_b);
	
	/* Small delay */
	for (i = 0; i < 1000; i++)
		barrier();
	
	bpf_spin_lock(&locks->lock_a);
	locks->value++;
	bpf_spin_unlock(&locks->lock_a);
	bpf_spin_unlock(&locks->lock_b);

	return 0;
}

char _license[] SEC("license") = "GPL";
