// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: binary trie, arena variant.
 * Single arena_spinlock_t global lock; no undo-log overhead.
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

struct bench_trie_node __arena arena_trie_pool[BENCH_MAX_POOL];
__u32 __arena arena_trie_root;
volatile long arena_trie_alloc_idx;
arena_spinlock_t __arena arena_trie_lock;

int test_skip = 1;

SEC("fentry/bench_arena_trie_init")
int BPF_PROG(trie_init, __u32 pool_size)
{
	__u32 i;

	arena_trie_root = 0;
	arena_trie_alloc_idx = 1;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		arena_trie_pool[i].child[0] = 0;
		arena_trie_pool[i].child[1] = 0;
		arena_trie_pool[i].key_bit   = 0;
		arena_trie_pool[i].val       = 0;
	}
	return 0;
}

SEC("fentry/bench_arena_trie_insert")
int BPF_PROG(trie_insert, __u64 key, __u64 val)
{
	unsigned long flags;
	int ret, depth;
	__u32 cur, parent, new_node;
	int dir = 0;

	ret = arena_spin_lock_irqsave(&arena_trie_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}

	cur    = arena_trie_root;
	parent = 0;

	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur) {
			new_node = (__u32)__sync_fetch_and_add(&arena_trie_alloc_idx, 1);
			if (new_node >= BENCH_MAX_POOL)
				goto out;

			arena_trie_pool[new_node].child[0] = 0;
			arena_trie_pool[new_node].child[1] = 0;
			arena_trie_pool[new_node].key_bit   = (__u32)(63 - depth);
			arena_trie_pool[new_node].val       = val;

			if (parent)
				arena_trie_pool[parent].child[dir] = new_node;
			else
				arena_trie_root = new_node;
			goto out;
		}
		parent = cur;
		dir    = bit;
		cur    = arena_trie_pool[cur].child[bit];
	}
out:
	arena_spin_unlock_irqrestore(&arena_trie_lock, flags);
	return 0;
}

SEC("fentry/bench_arena_trie_lookup")
int BPF_PROG(trie_lookup, __u64 key)
{
	unsigned long flags;
	int ret, depth;
	__u32 cur;

	ret = arena_spin_lock_irqsave(&arena_trie_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	cur = arena_trie_root;
	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur)
			break;
		if (!arena_trie_pool[cur].child[0] && !arena_trie_pool[cur].child[1])
			break;
		cur = arena_trie_pool[cur].child[bit];
	}
	arena_spin_unlock_irqrestore(&arena_trie_lock, flags);
	return 0;
}

SEC("fentry/bench_arena_trie_update")
int BPF_PROG(trie_update, __u64 key, __u64 val)
{
	unsigned long flags;
	int ret, depth;
	__u32 cur;

	ret = arena_spin_lock_irqsave(&arena_trie_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	cur = arena_trie_root;
	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur)
			break;
		if (!arena_trie_pool[cur].child[0] && !arena_trie_pool[cur].child[1]) {
			arena_trie_pool[cur].val = val;
			break;
		}
		cur = arena_trie_pool[cur].child[bit];
	}
	arena_spin_unlock_irqrestore(&arena_trie_lock, flags);
	return 0;
}

SEC("fentry/bench_arena_trie_delete")
int BPF_PROG(trie_delete_op, __u64 key)
{
	unsigned long flags;
	int ret, depth, dir = 0;
	__u32 cur, parent;

	ret = arena_spin_lock_irqsave(&arena_trie_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	cur    = arena_trie_root;
	parent = 0;
	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur)
			break;
		if (!arena_trie_pool[cur].child[0] && !arena_trie_pool[cur].child[1]) {
			arena_trie_pool[cur].key_bit = 0;
			arena_trie_pool[cur].val     = 0;
			if (parent)
				arena_trie_pool[parent].child[dir] = 0;
			else
				arena_trie_root = 0;
			break;
		}
		parent = cur;
		dir    = bit;
		cur    = arena_trie_pool[cur].child[bit];
	}
	arena_spin_unlock_irqrestore(&arena_trie_lock, flags);
	return 0;
}

#else
int test_skip = 2;

SEC("fentry/bench_arena_trie_init")
int BPF_PROG(trie_init, __u32 pool_size) { return 0; }

SEC("fentry/bench_arena_trie_insert")
int BPF_PROG(trie_insert, __u64 key, __u64 val) { return -2; }

SEC("fentry/bench_arena_trie_lookup")
int BPF_PROG(trie_lookup, __u64 key) { return -2; }

SEC("fentry/bench_arena_trie_update")
int BPF_PROG(trie_update, __u64 key, __u64 val) { return -2; }

SEC("fentry/bench_arena_trie_delete")
int BPF_PROG(trie_delete_op, __u64 key) { return -2; }
#endif

char _license[] SEC("license") = "GPL";
