// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: ring buffer, arena variant.
 * arena_spinlock_t embedded in each slot; no undo-log overhead.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bench_spinlock_shared.h"
#include "bpf_arena_spin_lock.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 24);
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32);
#else
	__ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

#if defined(ENABLE_ATOMICS_TESTS) && defined(__BPF_FEATURE_ADDR_SPACE_CAST)

struct bench_arena_ring_slot {
	arena_spinlock_t lock;
	__u64 data;
	__u32 valid;
};

struct bench_arena_ring_slot __arena arena_ring_pool[BENCH_RING_SLOTS];
volatile long arena_ring_head;

int test_skip = 1;

SEC("fentry/bench_arena_ring_init")
int BPF_PROG(ring_init, __u32 num_slots)
{
	__u32 i;

	arena_ring_head = 0;
	bpf_for(i, 0, BENCH_RING_SLOTS) {
		arena_ring_pool[i].data  = 0;
		arena_ring_pool[i].valid = 0;
	}
	return 0;
}

SEC("fentry/bench_arena_ring_enqueue")
int BPF_PROG(ring_enqueue, __u64 val)
{
	unsigned long flags;
	int ret;
	__u32 slot;

	slot = (__u32)(__sync_fetch_and_add(&arena_ring_head, 1) % BENCH_RING_SLOTS);

	ret = arena_spin_lock_irqsave(&arena_ring_pool[slot].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}

	arena_ring_pool[slot].data  = val;
	arena_ring_pool[slot].valid = 1;

	arena_spin_unlock_irqrestore(&arena_ring_pool[slot].lock, flags);
	return 0;
}

SEC("fentry/bench_arena_ring_lookup")
int BPF_PROG(ring_lookup, __u32 slot)
{
	unsigned long flags;
	int ret;

	slot %= BENCH_RING_SLOTS;
	ret = arena_spin_lock_irqsave(&arena_ring_pool[slot].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	/* read-only: no writes */
	arena_spin_unlock_irqrestore(&arena_ring_pool[slot].lock, flags);
	return 0;
}

SEC("fentry/bench_arena_ring_update")
int BPF_PROG(ring_update, __u32 slot, __u64 val)
{
	unsigned long flags;
	int ret;

	slot %= BENCH_RING_SLOTS;
	ret = arena_spin_lock_irqsave(&arena_ring_pool[slot].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	arena_ring_pool[slot].data = val;
	arena_spin_unlock_irqrestore(&arena_ring_pool[slot].lock, flags);
	return 0;
}

SEC("fentry/bench_arena_ring_dequeue")
int BPF_PROG(ring_dequeue_op, __u32 slot)
{
	unsigned long flags;
	int ret;

	slot %= BENCH_RING_SLOTS;
	ret = arena_spin_lock_irqsave(&arena_ring_pool[slot].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	arena_ring_pool[slot].valid = 0;
	arena_spin_unlock_irqrestore(&arena_ring_pool[slot].lock, flags);
	return 0;
}

#else
int test_skip = 2;

SEC("fentry/bench_arena_ring_init")
int BPF_PROG(ring_init, __u32 num_slots) { return 0; }

SEC("fentry/bench_arena_ring_enqueue")
int BPF_PROG(ring_enqueue, __u64 val) { return -2; }

SEC("fentry/bench_arena_ring_lookup")
int BPF_PROG(ring_lookup, __u32 slot) { return -2; }

SEC("fentry/bench_arena_ring_update")
int BPF_PROG(ring_update, __u32 slot, __u64 val) { return -2; }

SEC("fentry/bench_arena_ring_dequeue")
int BPF_PROG(ring_dequeue_op, __u32 slot) { return -2; }
#endif

char _license[] SEC("license") = "GPL";
