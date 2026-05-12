// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: linked list, arena variant.
 *
 * arena_spinlock_t is embedded directly in each arena node.
 * No undo-log entries are injected because arena_spinlock_t is not a
 * BPF helper; the verifier never sets active_locks for it.
 *
 * Writes per CS: 3  (same operations as undo_log variant)
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

struct bench_arena_list_node {
	arena_spinlock_t lock;
	__u32 next_idx;
	__u64 data;
};

struct bench_arena_list_node __arena arena_list_pool[BENCH_MAX_POOL];
__u32 __arena arena_list_head_idx;

int test_skip = 1;

SEC("fentry/bench_arena_list_init")
int BPF_PROG(list_init, __u32 pool_size)
{
	__u32 i;

	arena_list_head_idx = (__u32)~0;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		arena_list_pool[i].next_idx = (__u32)~0;
		arena_list_pool[i].data = 0;
	}
	return 0;
}

SEC("fentry/bench_arena_list_insert")
int BPF_PROG(list_insert, __u32 new_idx, __u32 head_lock_idx)
{
	unsigned long flags;
	int ret;

	if (new_idx >= BENCH_MAX_POOL)
		return 0;

	ret = arena_spin_lock_irqsave(&arena_list_pool[head_lock_idx].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}

	/* No undo-log entries — arena_spinlock_t is not a BPF helper */
	arena_list_pool[new_idx].next_idx = arena_list_head_idx;
	arena_list_pool[new_idx].data     = bpf_ktime_get_ns();
	arena_list_head_idx               = new_idx;

	arena_spin_unlock_irqrestore(&arena_list_pool[head_lock_idx].lock, flags);
	return 0;
}

SEC("fentry/bench_arena_list_lookup")
int BPF_PROG(list_lookup, __u32 idx)
{
	unsigned long flags;
	int ret;
	__u32 cur;
	int i;

	ret = arena_spin_lock_irqsave(&arena_list_pool[0].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	cur = arena_list_head_idx;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		if (cur == (__u32)~0)
			break;
		if ((__u32)i == idx % BENCH_MAX_POOL)
			break;
		cur = arena_list_pool[cur].next_idx;
	}
	arena_spin_unlock_irqrestore(&arena_list_pool[0].lock, flags);
	return 0;
}

SEC("fentry/bench_arena_list_update")
int BPF_PROG(list_update, __u32 idx, __u64 val)
{
	unsigned long flags;
	int ret;
	__u32 cur;
	int i;

	ret = arena_spin_lock_irqsave(&arena_list_pool[0].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	cur = arena_list_head_idx;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		if (cur == (__u32)~0)
			break;
		if ((__u32)i == idx % BENCH_MAX_POOL) {
			arena_list_pool[cur].data = val;
			break;
		}
		cur = arena_list_pool[cur].next_idx;
	}
	arena_spin_unlock_irqrestore(&arena_list_pool[0].lock, flags);
	return 0;
}

SEC("fentry/bench_arena_list_delete")
int BPF_PROG(list_delete_op)
{
	unsigned long flags;
	int ret;
	__u32 head;

	ret = arena_spin_lock_irqsave(&arena_list_pool[0].lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	head = arena_list_head_idx;
	if (head != (__u32)~0)
		arena_list_head_idx = arena_list_pool[head].next_idx;
	arena_spin_unlock_irqrestore(&arena_list_pool[0].lock, flags);
	return 0;
}

#else
int test_skip = 2;

SEC("fentry/bench_arena_list_init")
int BPF_PROG(list_init, __u32 pool_size) { return 0; }

SEC("fentry/bench_arena_list_insert")
int BPF_PROG(list_insert, __u32 new_idx, __u32 head_lock_idx) { return -2; }

SEC("fentry/bench_arena_list_lookup")
int BPF_PROG(list_lookup, __u32 idx) { return -2; }

SEC("fentry/bench_arena_list_update")
int BPF_PROG(list_update, __u32 idx, __u64 val) { return -2; }

SEC("fentry/bench_arena_list_delete")
int BPF_PROG(list_delete_op) { return -2; }
#endif

char _license[] SEC("license") = "GPL";
