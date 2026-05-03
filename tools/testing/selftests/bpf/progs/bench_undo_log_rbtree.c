// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: red-black tree insert + rebalance, undo_log variant.
 *
 * Global lock.  Node 0 is the nil sentinel (black, all pointers 0).
 * Each insert: BST traversal + color fixup with up to 2 rotations.
 *
 * Writes per CS: 6 (node init) + up to ~24 (rotations/recolor) = ~30.
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
struct bench_rb_node __arena rb_pool[BENCH_MAX_POOL];
__u32 __arena rb_root;
volatile long rb_alloc_idx;
#endif

struct rb_global_lock {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct rb_global_lock);
} rb_lock SEC(".maps");

SEC("fentry/bench_undo_rbtree_init")
int BPF_PROG(rbtree_init, __u32 pool_size)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	__u32 i;

	/* node 0 = nil sentinel */
	rb_pool[0].left = rb_pool[0].right = rb_pool[0].parent = 0;
	rb_pool[0].color = BENCH_RB_BLACK;
	rb_pool[0].key = rb_pool[0].val = 0;
	rb_root = 0;
	rb_alloc_idx = 1;
	bpf_for(i, 1, BENCH_MAX_POOL) {
		rb_pool[i].left = rb_pool[i].right = rb_pool[i].parent = 0;
		rb_pool[i].color = BENCH_RB_BLACK;
		rb_pool[i].key = rb_pool[i].val = 0;
	}
#endif
	return 0;
}

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST

static __always_inline void rb_rotate_left(__u32 x)
{
	__u32 y = rb_pool[x].right;

	rb_pool[x].right = rb_pool[y].left;
	if (rb_pool[y].left)
		rb_pool[rb_pool[y].left].parent = x;
	rb_pool[y].parent = rb_pool[x].parent;
	if (!rb_pool[x].parent)
		rb_root = y;
	else if (x == rb_pool[rb_pool[x].parent].left)
		rb_pool[rb_pool[x].parent].left = y;
	else
		rb_pool[rb_pool[x].parent].right = y;
	rb_pool[y].left = x;
	rb_pool[x].parent = y;
}

static __always_inline void rb_rotate_right(__u32 x)
{
	__u32 y = rb_pool[x].left;

	rb_pool[x].left = rb_pool[y].right;
	if (rb_pool[y].right)
		rb_pool[rb_pool[y].right].parent = x;
	rb_pool[y].parent = rb_pool[x].parent;
	if (!rb_pool[x].parent)
		rb_root = y;
	else if (x == rb_pool[rb_pool[x].parent].right)
		rb_pool[rb_pool[x].parent].right = y;
	else
		rb_pool[rb_pool[x].parent].left = y;
	rb_pool[y].right = x;
	rb_pool[x].parent = y;
}

static __always_inline void rb_insert_fixup(__u32 z)
{
	__u32 y;
	int depth;

	bpf_for(depth, 0, 64) {
		__u32 p, g;

		if (!rb_pool[rb_pool[z].parent].color)
			break;
		p = rb_pool[z].parent;
		g = rb_pool[p].parent;
		if (!p || !g)
			break;

		if (p == rb_pool[g].left) {
			y = rb_pool[g].right;
			if (rb_pool[y].color == BENCH_RB_RED) {
				rb_pool[p].color = BENCH_RB_BLACK;
				rb_pool[y].color = BENCH_RB_BLACK;
				rb_pool[g].color = BENCH_RB_RED;
				z = g;
			} else {
				if (z == rb_pool[p].right) {
					z = p;
					rb_rotate_left(z);
					p = rb_pool[z].parent;
					g = rb_pool[p].parent;
				}
				rb_pool[p].color = BENCH_RB_BLACK;
				rb_pool[g].color = BENCH_RB_RED;
				rb_rotate_right(g);
			}
		} else {
			y = rb_pool[g].left;
			if (rb_pool[y].color == BENCH_RB_RED) {
				rb_pool[p].color = BENCH_RB_BLACK;
				rb_pool[y].color = BENCH_RB_BLACK;
				rb_pool[g].color = BENCH_RB_RED;
				z = g;
			} else {
				if (z == rb_pool[p].left) {
					z = p;
					rb_rotate_right(z);
					p = rb_pool[z].parent;
					g = rb_pool[p].parent;
				}
				rb_pool[p].color = BENCH_RB_BLACK;
				rb_pool[g].color = BENCH_RB_RED;
				rb_rotate_left(g);
			}
		}
	}
	rb_pool[rb_root].color = BENCH_RB_BLACK;
}

#endif /* __BPF_FEATURE_ADDR_SPACE_CAST */

SEC("fentry/bench_undo_rbtree_insert")
int BPF_PROG(rbtree_insert, __u64 key, __u64 val)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	struct rb_global_lock *g;
	__u32 key0 = 0;
	__u32 z, p, x;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);

	z = (__u32)__sync_fetch_and_add(&rb_alloc_idx, 1);
	if (z >= BENCH_MAX_POOL)
		goto out;

	/* 6 arena writes → 6 undo-log entries */
	rb_pool[z].key    = key;
	rb_pool[z].val    = val;
	rb_pool[z].color  = BENCH_RB_RED;
	rb_pool[z].left   = 0;
	rb_pool[z].right  = 0;
	rb_pool[z].parent = 0;

	p = 0;
	x = rb_root;
	{
		int _d;

		bpf_for(_d, 0, BENCH_MAX_POOL) {
			if (!x)
				break;
			p = x;
			if (key < rb_pool[x].key)
				x = rb_pool[x].left;
			else
				x = rb_pool[x].right;
		}
	}

	rb_pool[z].parent = p;
	if (!p)
		rb_root = z;
	else if (key < rb_pool[p].key)
		rb_pool[p].left = z;
	else
		rb_pool[p].right = z;

	rb_insert_fixup(z); /* up to ~24 more arena writes */
out:
	bpf_spin_unlock(&g->lock);
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
