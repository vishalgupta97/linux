// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: binary trie on 64-bit keys, lock_func variant.
 *
 * Same data structure and attach points as the undo_log variant, but each
 * critical section is wrapped in a callback passed to bpf_lock_func() instead
 * of inline bpf_spin_lock()/bpf_spin_unlock().  Global lock; inserts one node
 * per call, traversing from the MSB down until a null child is found.
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
	__ulong(map_extra, 0x1ull << 44);
} arena SEC(".maps");

struct bench_trie_node __arena trie_pool[BENCH_MAX_POOL];
__u32 __arena trie_root;
volatile long trie_alloc_idx;

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
	__u32 i;

	trie_root = 0;
	trie_alloc_idx = 1;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		trie_pool[i].child[0] = 0;
		trie_pool[i].child[1] = 0;
		trie_pool[i].key_bit   = 0;
		trie_pool[i].val       = 0;
	}
	return 0;
}

static int trie_insert_cb(__u64 key, __u64 val)
{
	__u32 cur, parent, new_node;
	int dir = 0, depth;

	cur    = trie_root;
	parent = 0;

	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur) {
			new_node = (__u32)__sync_fetch_and_add(&trie_alloc_idx, 1);
			if (new_node >= BENCH_MAX_POOL)
				return 0;

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
			return 0;
		}
		parent = cur;
		dir    = bit;
		cur    = trie_pool[cur].child[bit];
	}
	return 0;
}

SEC("fentry/bench_undo_trie_insert")
int BPF_PROG(trie_insert, __u64 key, __u64 val)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, trie_insert_cb, key, val, 0);
	return 0;
}

/* Follow bits of key from MSB; return val at leaf. 0 undo-log writes. */
static int trie_lookup_cb(__u64 key)
{
	__u32 cur = trie_root;
	int depth;

	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur)
			break;
		if (!trie_pool[cur].child[0] && !trie_pool[cur].child[1])
			break;
		cur = trie_pool[cur].child[bit];
	}
	return 0;
}

SEC("fentry/bench_undo_trie_lookup")
int BPF_PROG(trie_lookup, __u64 key)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, trie_lookup_cb, key, 0, 0);
	return 0;
}

/* Same traversal; write val at leaf. 1 undo-log write. */
static int trie_update_cb(__u64 key, __u64 val)
{
	__u32 cur = trie_root;
	int depth;

	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur)
			break;
		if (!trie_pool[cur].child[0] && !trie_pool[cur].child[1]) {
			trie_pool[cur].val = val;	/* 1 undo-log entry */
			break;
		}
		cur = trie_pool[cur].child[bit];
	}
	return 0;
}

SEC("fentry/bench_undo_trie_update")
int BPF_PROG(trie_update, __u64 key, __u64 val)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, trie_update_cb, key, val, 0);
	return 0;
}

/* Find the leaf and zero it + unlink from parent. 2–3 undo-log writes. */
static int trie_delete_cb(__u64 key)
{
	__u32 cur, parent;
	int dir = 0, depth;

	cur    = trie_root;
	parent = 0;
	bpf_for(depth, 0, 64) {
		int bit = (int)((key >> (63 - depth)) & 1);

		if (!cur)
			break;
		if (!trie_pool[cur].child[0] && !trie_pool[cur].child[1]) {
			trie_pool[cur].key_bit = 0;	/* 1 undo-log entry */
			trie_pool[cur].val     = 0;	/* 1 undo-log entry */
			if (parent)
				trie_pool[parent].child[dir] = 0;	/* 1 undo-log */
			else
				trie_root = 0;			/* 1 undo-log */
			break;
		}
		parent = cur;
		dir    = bit;
		cur    = trie_pool[cur].child[bit];
	}
	return 0;
}

SEC("fentry/bench_undo_trie_delete")
int BPF_PROG(trie_delete_op, __u64 key)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, trie_delete_cb, key, 0, 0);
	return 0;
}

char _license[] SEC("license") = "GPL";
