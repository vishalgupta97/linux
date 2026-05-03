// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: binary trie on 64-bit keys, undo_log variant.
 *
 * Global lock.  Inserts one node per call, traversing from the MSB
 * down until a null child is found.  Each new node requires 4 writes
 * (child[0], child[1], key_bit, val) plus a parent pointer update.
 *
 * Writes per CS: ~5–20 depending on key depth.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_arena_common.h"
#include "bench_spinlock_shared.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 8);
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32);
#else
	__ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
struct bench_trie_node __arena trie_pool[BENCH_MAX_POOL];
__u32 __arena trie_root;
volatile long trie_alloc_idx;
#endif

struct trie_global_lock {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct trie_global_lock);
} trie_lock SEC(".maps");

SEC("fentry/bench_undo_trie_init")
int BPF_PROG(trie_init, __u32 pool_size)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	__u32 i;

	trie_root = 0;
	trie_alloc_idx = 1;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		trie_pool[i].child[0] = 0;
		trie_pool[i].child[1] = 0;
		trie_pool[i].key_bit   = 0;
		trie_pool[i].val       = 0;
	}
#endif
	return 0;
}

SEC("fentry/bench_undo_trie_insert")
int BPF_PROG(trie_insert, __u64 key, __u64 val)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	struct trie_global_lock *g;
	__u32 key0 = 0;
	__u32 cur, parent, new_node;
	int dir = 0, depth;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);

	cur    = trie_root;
	parent = 0;

	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur) {
			new_node = (__u32)__sync_fetch_and_add(&trie_alloc_idx, 1);
			if (new_node >= BENCH_MAX_POOL)
				goto out;

			/* 4 arena writes → 4 undo-log entries */
			trie_pool[new_node].child[0] = 0;
			trie_pool[new_node].child[1] = 0;
			trie_pool[new_node].key_bit   = (__u32)(63 - depth);
			trie_pool[new_node].val       = val;

			/* 1 more arena write: parent's child pointer */
			if (parent)
				trie_pool[parent].child[dir] = new_node;
			else
				trie_root = new_node; /* arena write */
			goto out;
		}
		parent = cur;
		dir    = bit;
		cur    = trie_pool[cur].child[bit];
	}
out:
	bpf_spin_unlock(&g->lock);
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
