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
	__ulong(map_extra, 0x1ull << 44);
} arena SEC(".maps");

struct bench_rb_node __arena rb_pool[BENCH_MAX_POOL];
__u32 __arena rb_root;
volatile long rb_alloc_idx;

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
	__u32 i;

	rb_alloc_idx = 1;
	/* Access each node via an explicit __arena * → * cast so LLVM emits
	 * a single addr_space_cast for the element pointer rather than
	 * generating separate ld_imm64 loads (without the cast) for zero-value
	 * field writes.  rb_root is set after the loop so no arena-scalar
	 * write precedes the loop's arena-array access. */
	bpf_for(i, 0, BENCH_MAX_POOL) {
		struct bench_rb_node *node = (struct bench_rb_node *)(rb_pool + i);

		node->left = 0;
		node->right = 0;
		node->parent = 0;
		node->color = BENCH_RB_BLACK;
		node->key = 0;
		node->val = 0;
	}
	rb_root = 0;
	return 0;
}

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

	bpf_for(depth, 0, 64) { //TODO: Fix this
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

SEC("fentry/bench_undo_rbtree_insert")
int BPF_PROG(rbtree_insert, __u64 key, __u64 val)
{
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
	return 0;
}

/* BST search for key; return val if found. 0 undo-log writes. */
SEC("fentry/bench_undo_rbtree_lookup")
int BPF_PROG(rbtree_lookup, __u64 key)
{
	struct rb_global_lock *g;
	__u32 key0 = 0;
	__u32 cur;
	int _d;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	cur = rb_root;
	bpf_for(_d, 0, BENCH_MAX_POOL) {
		if (!cur)
			break;
		if (key == rb_pool[cur].key)
			break;
		cur = (key < rb_pool[cur].key) ? rb_pool[cur].left : rb_pool[cur].right;
	}
	bpf_spin_unlock(&g->lock);
	return 0;
}

/* BST search; overwrite val at the matching node. 1 undo-log write. */
SEC("fentry/bench_undo_rbtree_update")
int BPF_PROG(rbtree_update, __u64 key, __u64 val)
{
	struct rb_global_lock *g;
	__u32 key0 = 0;
	__u32 cur;
	int _d;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	cur = rb_root;
	bpf_for(_d, 0, BENCH_MAX_POOL) {
		if (!cur)
			break;
		if (key == rb_pool[cur].key) {
			rb_pool[cur].val = val;		/* 1 undo-log entry */
			break;
		}
		cur = (key < rb_pool[cur].key) ? rb_pool[cur].left : rb_pool[cur].right;
	}
	bpf_spin_unlock(&g->lock);
	return 0;
}

/*
 * Lazy delete: find the node and zero key + val + color.
 * 3 undo-log writes (avoids full Cormen fixup complexity in BPF verifier).
 */
SEC("fentry/bench_undo_rbtree_delete")
int BPF_PROG(rbtree_delete_op, __u64 key)
{
	struct rb_global_lock *g;
	__u32 key0 = 0;
	__u32 cur;
	int _d;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	cur = rb_root;
	bpf_for(_d, 0, BENCH_MAX_POOL) {
		if (!cur)
			break;
		if (key == rb_pool[cur].key) {
			rb_pool[cur].key   = 0;		/* 1 undo-log entry */
			rb_pool[cur].val   = 0;		/* 1 undo-log entry */
			rb_pool[cur].color = BENCH_RB_BLACK;	/* 1 undo-log entry */
			break;
		}
		cur = (key < rb_pool[cur].key) ? rb_pool[cur].left : rb_pool[cur].right;
	}
	bpf_spin_unlock(&g->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
