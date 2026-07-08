// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: graph (adjacency list, add edge), lock_func variant.
 *
 * Same data structure and attach points as the undo_log variant, but each
 * critical section is wrapped in a callback passed to bpf_lock_func() instead
 * of inline bpf_spin_lock()/bpf_spin_unlock().  Global lock; pre-allocated
 * node + edge pools in arena.
 *
 * Writes per CS: 5  (edge.src, edge.dst, edge.weight, edge.next_out,
 *                    graph_nodes[src].first_edge)
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

struct bench_graph_node __arena graph_nodes[BENCH_MAX_POOL];
struct bench_graph_edge __arena graph_edges[BENCH_MAX_POOL];
volatile long graph_edge_alloc;

struct graph_global_lock {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct graph_global_lock);
} graph_lock SEC(".maps");

SEC("fentry/bench_undo_graph_init")
int BPF_PROG(graph_init, __u32 num_nodes, __u32 num_edges)
{
	__u32 i;

	graph_edge_alloc = 0;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		graph_nodes[i].first_edge = (__u32)~0;
		graph_nodes[i].data = 0;
	}
	bpf_for(i, 0, BENCH_MAX_POOL) {
		graph_edges[i].src      = 0;
		graph_edges[i].dst      = 0;
		graph_edges[i].next_out = (__u32)~0;
		graph_edges[i].weight   = 0;
	}
	return 0;
}

static int graph_add_edge_cb(__u64 src_v, __u64 dst_v, __u64 weight)
{
	__u32 src = (__u32)src_v;
	__u32 dst = (__u32)dst_v;
	__u32 e;

	e = (__u32)__sync_fetch_and_add(&graph_edge_alloc, 1);
	if (e >= BENCH_MAX_POOL)
		return 0;

	/* 5 arena writes → 5 undo-log entries */
	graph_edges[e].src      = src;
	graph_edges[e].dst      = dst;
	graph_edges[e].weight   = weight;
	graph_edges[e].next_out = graph_nodes[src].first_edge;
	graph_nodes[src].first_edge = e;
	return 0;
}

SEC("fentry/bench_undo_graph_add_edge")
int BPF_PROG(graph_add_edge, __u32 src, __u32 dst, __u64 weight)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (src >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, graph_add_edge_cb, src, dst, weight);
	return 0;
}

/* Traverse adjacency list of src; return weight if dst found. 0 undo-log writes. */
static int graph_lookup_cb(__u64 src_v, __u64 dst_v)
{
	__u32 src = (__u32)src_v;
	__u32 dst = (__u32)dst_v;
	__u32 e;
	int depth;

	e = graph_nodes[src].first_edge;
	bpf_for(depth, 0, BENCH_MAX_POOL) {
		if (e >= BENCH_MAX_POOL)
			break;
		if (graph_edges[e].dst == dst)
			break;
		e = graph_edges[e].next_out;
	}
	return 0;
}

SEC("fentry/bench_undo_graph_lookup")
int BPF_PROG(graph_lookup, __u32 src, __u32 dst)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (src >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, graph_lookup_cb, src, dst, 0);
	return 0;
}

/* Find edge (src, dst); update its weight. 1 undo-log write. */
static int graph_update_cb(__u64 src_v, __u64 dst_v, __u64 weight)
{
	__u32 src = (__u32)src_v;
	__u32 dst = (__u32)dst_v;
	__u32 e;
	int depth;

	e = graph_nodes[src].first_edge;
	bpf_for(depth, 0, BENCH_MAX_POOL) {
		if (e >= BENCH_MAX_POOL)
			break;
		if (graph_edges[e].dst == dst) {
			graph_edges[e].weight = weight;	/* 1 undo-log entry */
			break;
		}
		e = graph_edges[e].next_out;
	}
	return 0;
}

SEC("fentry/bench_undo_graph_update")
int BPF_PROG(graph_update, __u32 src, __u32 dst, __u64 weight)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (src >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, graph_update_cb, src, dst, weight);
	return 0;
}

/* Unlink edge (src, dst) from src's adjacency list. 1–2 undo-log writes. */
static int graph_delete_cb(__u64 src_v, __u64 dst_v)
{
	__u32 src = (__u32)src_v;
	__u32 dst = (__u32)dst_v;
	__u32 e, prev;
	int depth;

	e    = graph_nodes[src].first_edge;
	prev = (__u32)~0;
	bpf_for(depth, 0, BENCH_MAX_POOL) {
		if (e >= BENCH_MAX_POOL)
			break;
		if (graph_edges[e].dst == dst) {
			if (prev == (__u32)~0)
				graph_nodes[src].first_edge = graph_edges[e].next_out; /* 1 undo-log */
			else
				graph_edges[prev].next_out  = graph_edges[e].next_out; /* 1 undo-log */
			break;
		}
		prev = e;
		e    = graph_edges[e].next_out;
	}
	return 0;
}

SEC("fentry/bench_undo_graph_delete")
int BPF_PROG(graph_delete_op, __u32 src, __u32 dst)
{
	struct graph_global_lock *g;
	__u32 key0 = 0;

	if (src >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_lock_func(&g->lock, graph_delete_cb, src, dst, 0);
	return 0;
}

char _license[] SEC("license") = "GPL";
