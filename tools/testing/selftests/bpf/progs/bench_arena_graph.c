// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: graph (adjacency list, add edge), arena variant.
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

struct bench_graph_node __arena arena_graph_nodes[BENCH_MAX_POOL];
struct bench_graph_edge __arena arena_graph_edges[BENCH_MAX_POOL];
volatile long arena_graph_edge_alloc;
arena_spinlock_t __arena arena_graph_lock;

int test_skip = 1;

SEC("fentry/bench_arena_graph_init")
int BPF_PROG(graph_init, __u32 num_nodes, __u32 num_edges)
{
	__u32 i;

	arena_graph_edge_alloc = 0;
	bpf_for(i, 0, BENCH_MAX_POOL) {
		arena_graph_nodes[i].first_edge = (__u32)~0;
		arena_graph_nodes[i].data = 0;
	}
	bpf_for(i, 0, BENCH_MAX_POOL) {
		arena_graph_edges[i].src      = 0;
		arena_graph_edges[i].dst      = 0;
		arena_graph_edges[i].next_out = (__u32)~0;
		arena_graph_edges[i].weight   = 0;
	}
	return 0;
}

SEC("fentry/bench_arena_graph_add_edge")
int BPF_PROG(graph_add_edge, __u32 src, __u32 dst, __u64 weight)
{
	unsigned long flags;
	int ret;
	__u32 e;

	if (src >= BENCH_MAX_POOL)
		return 0;

	ret = arena_spin_lock_irqsave(&arena_graph_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}

	e = (__u32)__sync_fetch_and_add(&arena_graph_edge_alloc, 1);
	if (e >= BENCH_MAX_POOL)
		goto out;

	arena_graph_edges[e].src      = src;
	arena_graph_edges[e].dst      = dst;
	arena_graph_edges[e].weight   = weight;
	arena_graph_edges[e].next_out = arena_graph_nodes[src].first_edge;
	arena_graph_nodes[src].first_edge = e;
out:
	arena_spin_unlock_irqrestore(&arena_graph_lock, flags);
	return 0;
}

SEC("fentry/bench_arena_graph_lookup")
int BPF_PROG(graph_lookup, __u32 src, __u32 dst)
{
	unsigned long flags;
	int ret, depth;
	__u32 e;

	if (src >= BENCH_MAX_POOL)
		return 0;

	ret = arena_spin_lock_irqsave(&arena_graph_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	e = arena_graph_nodes[src].first_edge;
	bpf_for(depth, 0, BENCH_MAX_POOL) {
		if (e >= BENCH_MAX_POOL)
			break;
		if (arena_graph_edges[e].dst == dst)
			break;
		e = arena_graph_edges[e].next_out;
	}
	arena_spin_unlock_irqrestore(&arena_graph_lock, flags);
	return 0;
}

SEC("fentry/bench_arena_graph_update")
int BPF_PROG(graph_update, __u32 src, __u32 dst, __u64 weight)
{
	unsigned long flags;
	int ret, depth;
	__u32 e;

	if (src >= BENCH_MAX_POOL)
		return 0;

	ret = arena_spin_lock_irqsave(&arena_graph_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	e = arena_graph_nodes[src].first_edge;
	bpf_for(depth, 0, BENCH_MAX_POOL) {
		if (e >= BENCH_MAX_POOL)
			break;
		if (arena_graph_edges[e].dst == dst) {
			arena_graph_edges[e].weight = weight;
			break;
		}
		e = arena_graph_edges[e].next_out;
	}
	arena_spin_unlock_irqrestore(&arena_graph_lock, flags);
	return 0;
}

SEC("fentry/bench_arena_graph_delete")
int BPF_PROG(graph_delete_op, __u32 src, __u32 dst)
{
	unsigned long flags;
	int ret, depth;
	__u32 e, prev;

	if (src >= BENCH_MAX_POOL)
		return 0;

	ret = arena_spin_lock_irqsave(&arena_graph_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}
	e    = arena_graph_nodes[src].first_edge;
	prev = (__u32)~0;
	bpf_for(depth, 0, BENCH_MAX_POOL) {
		if (e >= BENCH_MAX_POOL)
			break;
		if (arena_graph_edges[e].dst == dst) {
			if (prev == (__u32)~0)
				arena_graph_nodes[src].first_edge = arena_graph_edges[e].next_out;
			else
				arena_graph_edges[prev].next_out  = arena_graph_edges[e].next_out;
			break;
		}
		prev = e;
		e    = arena_graph_edges[e].next_out;
	}
	arena_spin_unlock_irqrestore(&arena_graph_lock, flags);
	return 0;
}

#else
int test_skip = 2;

SEC("fentry/bench_arena_graph_init")
int BPF_PROG(graph_init, __u32 num_nodes, __u32 num_edges) { return 0; }

SEC("fentry/bench_arena_graph_add_edge")
int BPF_PROG(graph_add_edge, __u32 src, __u32 dst, __u64 weight) { return -2; }

SEC("fentry/bench_arena_graph_lookup")
int BPF_PROG(graph_lookup, __u32 src, __u32 dst) { return -2; }

SEC("fentry/bench_arena_graph_update")
int BPF_PROG(graph_update, __u32 src, __u32 dst, __u64 weight) { return -2; }

SEC("fentry/bench_arena_graph_delete")
int BPF_PROG(graph_delete_op, __u32 src, __u32 dst) { return -2; }
#endif

char _license[] SEC("license") = "GPL";
