// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: ring buffer, kmod_bpf variant.
 *
 * The kernel module allocates ring slots (vmalloc) and passes a direct
 * kernel pointer (PTR_TO_BTF_ID | MEM_WRITE) for the slot being operated on.
 * No arena map — uses the "undo-log only" JIT path.
 *
 * Writes per CS: enqueue=2, lookup=0, update=1, dequeue=1.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bench_spinlock_shared.h"

struct ring_lock_entry {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, BENCH_RING_SLOTS);
	__type(key, __u32);
	__type(value, struct ring_lock_entry);
} ring_locks SEC(".maps");

SEC("fentry/bench_kmod_bpf_ring_init")
int BPF_PROG(ring_init, __u32 num_slots)
{
	return 0;
}

SEC("fentry/bench_kmod_bpf_ring_enqueue")
int BPF_PROG(ring_enqueue, __u32 slot, struct bench_ring_slot *sp, __u64 val)
{
	struct ring_lock_entry *lk;

	if (!sp)
		return 0;
	slot %= BENCH_RING_SLOTS;

	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);
	sp->data  = val;	/* undo-log entry 1 */
	sp->valid = 1;		/* undo-log entry 2 */
	bpf_spin_unlock(&lk->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_ring_lookup")
int BPF_PROG(ring_lookup, __u32 slot, struct bench_ring_slot *sp)
{
//	struct ring_lock_entry *lk;
//	__u64 val = 0;
//
//	if (!sp)
//		return 0;
//	slot %= BENCH_RING_SLOTS;
//
//	lk = bpf_map_lookup_elem(&ring_locks, &slot);
//	if (!lk)
//		return 0;
//
//	bpf_spin_lock(&lk->lock);
//	if (sp->valid)
//		val = sp->data;	/* read only, no undo-log entry */
//	bpf_spin_unlock(&lk->lock);
//	lookup_sum += val;
	return 0;
}

SEC("fentry/bench_kmod_bpf_ring_update")
int BPF_PROG(ring_update, __u32 slot, struct bench_ring_slot *sp, __u64 val)
{
	struct ring_lock_entry *lk;

	if (!sp)
		return 0;
	slot %= BENCH_RING_SLOTS;

	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);
	sp->data = val;		/* undo-log entry 1 */
	bpf_spin_unlock(&lk->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_ring_dequeue")
int BPF_PROG(ring_dequeue_op, __u32 slot, struct bench_ring_slot *sp)
{
	struct ring_lock_entry *lk;

	if (!sp)
		return 0;
	slot %= BENCH_RING_SLOTS;

	lk = bpf_map_lookup_elem(&ring_locks, &slot);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);
	sp->valid = 0;		/* undo-log entry 1 */
	bpf_spin_unlock(&lk->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
