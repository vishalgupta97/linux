// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: red-black tree, kmod_bpf variant.
 *
 * The kernel module allocates the RB node pool (vmalloc) and passes a direct
 * kernel pointer (PTR_TO_BTF_ID | MEM_WRITE) for the node being operated on.
 * No arena map — uses the "undo-log only" JIT path.
 *
 * Simplified: one new node per insert (no rebalancing — benchmark focuses on
 * undo-log overhead, not RB correctness).  rb_root in .bss tracks the root.
 *
 * Writes per CS: insert=6 (+1 if first node), lookup=0, update=1, delete=3.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bench_spinlock_shared.h"

struct rb_global_lock {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct rb_global_lock);
} rb_lock SEC(".maps");

__u32 rb_root;

SEC("fentry/bench_kmod_bpf_rbtree_init")
int BPF_PROG(rbtree_init, __u32 pool_size)
{
	rb_root = 0;
	return 0;
}

SEC("fentry/bench_kmod_bpf_rbtree_insert")
int BPF_PROG(rbtree_insert, __u32 new_idx,
	     struct bench_rb_node *node, __u64 key, __u64 val)
{
	struct rb_global_lock *g;
	__u32 key0 = 0;

	if (!node || new_idx >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);

	node->left   = 0;		/* undo-log entry 1 */
	node->right  = 0;		/* undo-log entry 2 */
	node->parent = 0;		/* undo-log entry 3 */
	node->color  = BENCH_RB_RED;	/* undo-log entry 4 */
	node->key    = key;		/* undo-log entry 5 */
	node->val    = val;		/* undo-log entry 6 */
	if (!rb_root)
		rb_root = new_idx;	/* undo-log entry 7 (.bss) */

	bpf_spin_unlock(&g->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_rbtree_lookup")
int BPF_PROG(rbtree_lookup, __u64 key, struct bench_rb_node *node)
{
	return 0;
//	struct rb_global_lock *g;
//	__u32 key0 = 0;
//	__u64 val = 0;
//
//	if (!node)
//		return 0;
//
//	g = bpf_map_lookup_elem(&rb_lock, &key0);
//	if (!g)
//		return 0;
//
//	bpf_spin_lock(&g->lock);
//	val = node->val;	/* read only, no undo-log entry */
//	bpf_spin_unlock(&g->lock);
//	return (__s32)val;
}

SEC("fentry/bench_kmod_bpf_rbtree_update")
int BPF_PROG(rbtree_update, __u64 key, struct bench_rb_node *node, __u64 val)
{
	struct rb_global_lock *g;
	__u32 key0 = 0;

	if (!node)
		return 0;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	node->val = val;	/* undo-log entry 1 */
	bpf_spin_unlock(&g->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_rbtree_delete")
int BPF_PROG(rbtree_delete_op, __u64 key, struct bench_rb_node *node)
{
	struct rb_global_lock *g;
	__u32 key0 = 0;

	if (!node)
		return 0;

	g = bpf_map_lookup_elem(&rb_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	node->key   = 0;		/* undo-log entry 1 */
	node->val   = 0;		/* undo-log entry 2 */
	node->color = BENCH_RB_BLACK;	/* undo-log entry 3 */
	bpf_spin_unlock(&g->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
