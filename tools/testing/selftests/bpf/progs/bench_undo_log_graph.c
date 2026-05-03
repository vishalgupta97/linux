// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: graph (adjacency list, add edge), undo_log variant.
 *
 * Global lock.  Pre-allocated node + edge pools in arena.
 * Each add-edge: allocate edge slot, set 4 fields, prepend to src's list.
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
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32);
#else
	__ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
struct bench_graph_node __arena graph_nodes[BENCH_MAX_POOL];
struct bench_graph_edge __arena graph_edges[BENCH_MAX_POOL];
volatile long graph_edge_alloc;
#endif

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
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
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
#endif
	return 0;
}

SEC("fentry/bench_undo_graph_add_edge")
int BPF_PROG(graph_add_edge, __u32 src, __u32 dst, __u64 weight)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	struct graph_global_lock *g;
	__u32 key0 = 0;
	__u32 e;

	if (src >= BENCH_MAX_POOL)
		return 0;

	g = bpf_map_lookup_elem(&graph_lock, &key0);
	if (!g)
		return 0;

	bpf_spin_lock(&g->lock);

	e = (__u32)__sync_fetch_and_add(&graph_edge_alloc, 1);
	if (e >= BENCH_MAX_POOL)
		goto out;

	/* 5 arena writes → 5 undo-log entries */
	graph_edges[e].src      = src;
	graph_edges[e].dst      = dst;
	graph_edges[e].weight   = weight;
	graph_edges[e].next_out = graph_nodes[src].first_edge;
	graph_nodes[src].first_edge = e;
out:
	bpf_spin_unlock(&g->lock);
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
