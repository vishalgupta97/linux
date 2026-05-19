// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: graph (adjacency list), kmod_bpf variant.
 *
 * The kernel module allocates node + edge pools (vmalloc) and passes direct
 * kernel pointers (PTR_TO_BTF_ID | MEM_WRITE) for the nodes being operated on.
 * No arena map — uses the "undo-log only" JIT path.
 *
 * add_edge receives src_node and edge pointers directly; no traversal needed.
 *
 * Writes per CS: add_edge=5, lookup=0, update=1, delete=2.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bench_spinlock_shared.h"

struct graph_global_lock {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct graph_global_lock);
} graph_lock SEC(".maps");

SEC("fentry/bench_kmod_bpf_graph_init")
int BPF_PROG(graph_init, __u32 num_nodes, __u32 num_edges)
{
	return 0;
}

SEC("fentry/bench_kmod_bpf_graph_add_edge")
int BPF_PROG(graph_add_edge, __u32 src, __u32 dst,
	     struct bench_graph_node *src_node, __u32 edge_idx,
	     struct bench_graph_edge *edge, __u64 weight)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (!src_node || !edge || src >= BENCH_MAX_POOL || edge_idx >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);

	edge->src      = src;			/* undo-log entry 1 */
	edge->dst      = dst;			/* undo-log entry 2 */
	edge->weight   = weight;		/* undo-log entry 3 */
	edge->next_out = src_node->first_edge;	/* undo-log entry 4 */
	src_node->first_edge = edge_idx;	/* undo-log entry 5 */

	bpf_spin_unlock(&g->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_graph_lookup")
int BPF_PROG(graph_lookup, __u32 src, __u32 dst,
	     struct bench_graph_node *src_node)
{
	return 0;
//	struct graph_global_lock *g;
//	__u32 key0 = 0;
//	__u64 val = 0;
//
//	if (!src_node || src >= BENCH_MAX_POOL)
//		return 0;
//
//	g = bpf_map_lookup_elem(&graph_lock, &key0);
//	if (!g)
//		return 0;
//
//	bpf_spin_lock(&g->lock);
//	val = src_node->data;	/* read only, no undo-log entry */
//	bpf_spin_unlock(&g->lock);
//	return (__s32)val;
}

SEC("fentry/bench_kmod_bpf_graph_update")
int BPF_PROG(graph_update, __u32 src, __u32 dst,
	     struct bench_graph_edge *edge, __u64 weight)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (!edge)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	edge->weight = weight;	/* undo-log entry 1 */
	bpf_spin_unlock(&g->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_graph_delete")
int BPF_PROG(graph_delete_op, __u32 src, __u32 dst,
	     struct bench_graph_edge *edge)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (!edge)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);
	edge->src = 0;		/* undo-log entry 1 */
	edge->dst = 0;		/* undo-log entry 2 */
	bpf_spin_unlock(&g->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
