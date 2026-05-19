// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: binary trie, kmod_bpf variant.
 *
 * The kernel module allocates the trie pool (vmalloc) and passes a direct
 * kernel pointer (PTR_TO_BTF_ID | MEM_WRITE) for the node being operated on.
 * No arena map — uses the "undo-log only" JIT path.
 *
 * Simplified: one new node per insert.  The kthread pre-allocates a slot
 * index and passes the node pointer directly.  trie_root in .bss tracks
 * the current root.
 *
 * Writes per CS: insert=4 (+1 if first node), lookup=0, update=1, delete=2.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bench_spinlock_shared.h"

struct trie_global_lock {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct trie_global_lock);
} trie_lock SEC(".maps");

__u32 trie_root;

SEC("fentry/bench_kmod_bpf_trie_init")
int BPF_PROG(trie_init, __u32 pool_size)
{
	trie_root = 0;
	return 0;
}

SEC("fentry/bench_kmod_bpf_trie_insert")
int BPF_PROG(trie_insert, __u32 new_idx,
	     struct bench_trie_node *node, __u64 key, __u64 val)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	if (!node || new_idx >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);

	node->child[0] = 0;			/* undo-log entry 1 */
	node->child[1] = 0;			/* undo-log entry 2 */
	node->key_bit  = (__u32)(key & 63);	/* undo-log entry 3 */
	node->val      = val;			/* undo-log entry 4 */
	if (!trie_root)
		trie_root = new_idx;		/* undo-log entry 5 (.bss) */

	bpf_spin_unlock(&g->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_trie_lookup")
int BPF_PROG(trie_lookup, __u64 key, struct bench_trie_node *node)
{
	return 0;
//	struct trie_global_lock *g;
//	__u32 key0 = 0;
//	__u64 val = 0;
//
//	if (!node)
//		return 0;
//
//	g = bpf_map_lookup_elem(&trie_lock, &key0);
//	if (!g)
//		return 0;
//
//	bpf_spin_lock(&g->lock);
//	val = node->val;	/* read only, no undo-log entry */
//	bpf_spin_unlock(&g->lock);
//	return (__s32)val;
}

SEC("fentry/bench_kmod_bpf_trie_update")
int BPF_PROG(trie_update, __u64 key, struct bench_trie_node *node, __u64 val)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	if (!node)
		return 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	node->val = val;	/* undo-log entry 1 */
	bpf_spin_unlock(&g->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_trie_delete")
int BPF_PROG(trie_delete_op, __u64 key, struct bench_trie_node *node)
{
	struct trie_global_lock *g;
	__u32 key0 = 0;

	if (!node)
		return 0;

	g = bpf_map_lookup_elem(&trie_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	node->child[0] = 0;	/* undo-log entry 1 */
	node->child[1] = 0;	/* undo-log entry 2 */
	bpf_spin_unlock(&g->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
