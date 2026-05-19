// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: ring buffer, undo_log variant.
 *
 * Per-slot locking: each ring slot has a bpf_spin_lock in a parallel
 * map.  The head index is advanced atomically outside the lock so
 * multiple producers contend on different slots.
 *
 * Writes per CS: 2  (slot.data, slot.valid)
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_arena_common.h"
#include "bench_spinlock_shared.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 4);
	__ulong(map_extra, 0x1ull << 44);
} arena SEC(".maps");

struct bench_ring_slot __arena ring_pool[BENCH_RING_SLOTS];
volatile long ring_head;

struct ring_lock_entry {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, BENCH_RING_SLOTS);
	__type(key, __u32);
	__type(value, struct ring_lock_entry);
} ring_locks SEC(".maps");

SEC("fentry/bench_undo_ring_init")
int BPF_PROG(ring_init, __u32 num_slots)
{
	__u32 i;

	ring_head = 0;
	bpf_for(i, 0, BENCH_RING_SLOTS) {
		ring_pool[i].data  = 0;
		ring_pool[i].valid = 0;
	}
	return 0;
}

SEC("fentry/bench_undo_ring_enqueue")
int BPF_PROG(ring_enqueue, __u64 val)
{
	struct ring_lock_entry *lk;
	__u32 slot;

	/* Claim a slot atomically outside the lock */
	slot = (__u32)(__sync_fetch_and_add(&ring_head, 1) % BENCH_RING_SLOTS);

	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);

	/* 2 arena writes → 2 undo-log entries */
	ring_pool[slot].data  = val;
	ring_pool[slot].valid = 1;

	bpf_spin_unlock(&lk->lock);
	return 0;
}

/* Read data from slot if valid. 0 undo-log writes. */
SEC("fentry/bench_undo_ring_lookup")
int BPF_PROG(ring_lookup, __u32 slot)
{
	/*struct ring_lock_entry *lk;
	int ret = -1;

	slot %= BENCH_RING_SLOTS;
	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return ret;

	bpf_spin_lock(&lk->lock);
	if (ring_pool[slot].valid)
		ret = ring_pool[slot].data;
	bpf_spin_unlock(&lk->lock);*/
	return 0;
}

/* Overwrite slot data. 1 undo-log write. */
SEC("fentry/bench_undo_ring_update")
int BPF_PROG(ring_update, __u32 slot, __u64 val)
{
	struct ring_lock_entry *lk;

	slot %= BENCH_RING_SLOTS;
	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);
	ring_pool[slot].data = val;	/* 1 undo-log entry */
	bpf_spin_unlock(&lk->lock);
	return 0;
}

/* Mark slot invalid (dequeue). 1 undo-log write. */
SEC("fentry/bench_undo_ring_dequeue")
int BPF_PROG(ring_dequeue_op, __u32 slot)
{
	struct ring_lock_entry *lk;

	slot %= BENCH_RING_SLOTS;
	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);
	ring_pool[slot].valid = 0;	/* 1 undo-log entry */
	bpf_spin_unlock(&lk->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
