// SPDX-License-Identifier: GPL-2.0
/*
 * BPF spinlock undo-log benchmark kernel module.
 *
 * Provides:
 *  - noinline stub functions for the undo_log and arena BPF variants.
 *    BPF fentry programs attach to these stubs and perform the actual
 *    data structure operations in arena memory.
 *  - Full kernel-side implementations for the kmod baseline variant.
 *  - A kthread-based benchmark loop (warmup + measurement) that drives
 *    all three variants uniformly from the same execution context.
 *  - A miscdevice (/dev/bench_spinlock) with an ioctl interface for
 *    parameter setup, benchmark triggering, and result retrieval.
 *
 * Per-CPU stats are collected in bench_cpu_stats[].  Percentiles are
 * computed in-kernel from sampled latencies before the ioctl returns.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/percpu.h>
#include <linux/atomic.h>
#include <linux/sort.h>
#include <linux/cpumask.h>
#include <linux/sched.h>

/* ------------------------------------------------------------------ */
/* Public ioctl definitions — kept in sync with userspace header       */
/* ------------------------------------------------------------------ */

#define BENCH_DS_LIST    0
#define BENCH_DS_RING    1
#define BENCH_DS_TRIE    2
#define BENCH_DS_RBTREE  3
#define BENCH_DS_GRAPH   4

#define BENCH_VARIANT_UNDO_LOG  0
#define BENCH_VARIANT_ARENA     1
#define BENCH_VARIANT_KMOD      2
#define BENCH_VARIANT_KMOD_BPF  3
#define BENCH_VARIANT_LOCK_FUNC 4

#define BENCH_OP_INSERT  0
#define BENCH_OP_LOOKUP  1
#define BENCH_OP_UPDATE  2
#define BENCH_OP_DELETE  3

#define BENCH_IOC_MAGIC    'B'

struct bench_params {
	__u32 ds_type;
	__u32 variant;
	__u32 num_threads;
	__u32 pool_size;
	__u32 init_size;
	__u32 op_type;
	__u32 warmup_ms;
	__u32 bench_ms;
};

#define BENCH_MAX_CPUS 256

struct bench_percpu_result {
	__u64 ops;
	__u64 lat_sum_ns;
};

struct bench_all_results {
	__u32 num_cpus;
	__u64 total_ops;
	__u64 total_lat_sum_ns;
	__u64 p50_lat_ns;
	__u64 p95_lat_ns;
	__u64 p99_lat_ns;
	struct bench_percpu_result cpu[BENCH_MAX_CPUS];
};

#define BENCH_IOCTL_SET_PARAMS  _IOW(BENCH_IOC_MAGIC, 1, struct bench_params)
#define BENCH_IOCTL_EBPF_READY  _IO(BENCH_IOC_MAGIC, 2)
#define BENCH_IOCTL_RUN_KMOD    _IO(BENCH_IOC_MAGIC, 3)
#define BENCH_IOCTL_GET_RESULTS _IOR(BENCH_IOC_MAGIC, 4, struct bench_all_results)

/* ------------------------------------------------------------------ */
/* Per-CPU statistics                                                  */
/* ------------------------------------------------------------------ */

#define BENCH_LATENCY_SAMPLES  64   /* power-of-2; 64 KB per CPU */
#define BENCH_SAMPLE_RATE      1024     /* sample every N ops */

struct bench_cpu_stats {
	u64 ops;
	u64 lat_sum_ns;
	u32 sample_cnt;
	u64 samples[BENCH_LATENCY_SAMPLES];
} ____cacheline_aligned;

static DEFINE_PER_CPU(struct bench_cpu_stats, bench_cpu_stats);

/* ------------------------------------------------------------------ */
/* Benchmark state                                                     */
/* ------------------------------------------------------------------ */

static struct bench_params bench_cfg = {
	.pool_size  = 256,
	.num_threads = 1,
	.warmup_ms  = 5000,
	.bench_ms   = 15000,
};

static struct bench_all_results bench_results;
static atomic_t bench_ready_cnt;
static int bench_nthreads;

/* ------------------------------------------------------------------ */
/* Kernel-side data structures for the kmod variant                   */
/* ------------------------------------------------------------------ */

#define KMOD_MAX_POOL 256

/* Linked list */
struct kmod_list_node {
	spinlock_t lock;
	u32 next_idx;
	u64 data;
};

static struct kmod_list_node *kmod_list_pool;
static u32 kmod_list_head_idx;

/* Ring buffer */
struct kmod_ring_slot {
	spinlock_t lock;
	u64 data;
	u32 valid;
};

static struct kmod_ring_slot *kmod_ring_pool;
static atomic_t kmod_ring_head;
static u32 kmod_ring_size;

/* Trie (binary trie on 32-bit keys stored as 64-bit) */
struct kmod_trie_node {
	u32 child[2];
	u32 key_bit;
	u64 val;
};

static struct kmod_trie_node *kmod_trie_pool;
static u32 kmod_trie_root;
static atomic_t kmod_trie_alloc;
static spinlock_t kmod_trie_lock;

/* Red-black tree */
#define KMOD_RB_BLACK 0
#define KMOD_RB_RED   1
#define KMOD_NULL_IDX 0  /* sentinel index; node 0 is the nil sentinel */

struct kmod_rb_node {
	u32 left, right, parent;
	u32 color;
	u64 key;
	u64 val;
};

static struct kmod_rb_node *kmod_rb_pool;
static u32 kmod_rb_root;
static atomic_t kmod_rb_alloc;
static spinlock_t kmod_rb_lock;

/* Graph (adjacency list) */
struct kmod_graph_node {
	u32 first_edge;
	u64 data;
};

struct kmod_graph_edge {
	u32 src, dst, next_out;
	u64 weight;
};

static struct kmod_graph_node *kmod_graph_nodes;
static struct kmod_graph_edge *kmod_graph_edges;
static atomic_t kmod_graph_edge_alloc;
static u32 kmod_graph_num_nodes;
static spinlock_t kmod_graph_lock;

/* ------------------------------------------------------------------ */
/* kmod_bpf variant: plain node pools without embedded spinlock_t     */
/* Kernel module allocates these; BPF programs access via __writable_bpf pointers */
/* ------------------------------------------------------------------ */

struct bench_list_node  *kmod_bpf_list_pool;
struct bench_ring_slot  *kmod_bpf_ring_pool;
struct bench_trie_node  *kmod_bpf_trie_pool;
struct bench_rb_node    *kmod_bpf_rb_pool;
struct bench_graph_node *kmod_bpf_graph_nodes;
struct bench_graph_edge *kmod_bpf_graph_edges;
static atomic_t kmod_bpf_trie_alloc;
static atomic_t kmod_bpf_rb_alloc;
static atomic_t kmod_bpf_graph_edge_alloc;

/* ------------------------------------------------------------------ */
/* kmod data structure implementations                                 */
/* ------------------------------------------------------------------ */

/* ---- list ---- */

#include "list.h"

/* ---- ring ---- */

#include "ring.h"

/* ---- trie ---- */

#include "trie.h"

/* ---- rbtree ---- */

#include "rbtree.h"

/* ---- graph ---- */

#include "graph.h"

/* ---- kmod_bpf variant stubs ---- */
/*
 * Include declarations (struct types, pool externs, __writable_bpf).
 * Function bodies are defined below so Clang emits BTF_FUNC_GLOBAL for each,
 * which is required for BPF fentry attachment.
 */
#include "bench_kmod_bpf.h"

/* ------------------------------------------------------------------ */
/* List                                                                */
/* ------------------------------------------------------------------ */

noinline void bench_kmod_bpf_list_init(u32 pool_size)
{
	u32 i;

	if (!kmod_bpf_list_pool)
		return;
	for (i = 0; i < pool_size; i++) {
		kmod_bpf_list_pool[i].next_idx = (u32)~0;
		kmod_bpf_list_pool[i].data = 0;
	}
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_list_init);

noinline void bench_kmod_bpf_list_insert(u32 new_idx,
	bench_list_node_bpf_writable * new_node,
	u32 head_lock_idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_list_insert);

noinline u64 bench_kmod_bpf_list_lookup(u32 idx,
	bench_list_node_bpf_writable * node)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return node ? node->data : 0;
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_list_lookup);

noinline void bench_kmod_bpf_list_update(u32 idx,
	bench_list_node_bpf_writable * node,
	u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_list_update);

noinline void bench_kmod_bpf_list_delete(u32 head_idx,
	bench_list_node_bpf_writable * head_node)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_list_delete);

/* ------------------------------------------------------------------ */
/* Ring                                                                */
/* ------------------------------------------------------------------ */

noinline void bench_kmod_bpf_ring_init(u32 num_slots)
{
	u32 i;

	if (!kmod_bpf_ring_pool)
		return;
	for (i = 0; i < num_slots && i < BENCH_RING_SLOTS; i++) {
		kmod_bpf_ring_pool[i].data  = 0;
		kmod_bpf_ring_pool[i].valid = 0;
	}
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_ring_init);

noinline void bench_kmod_bpf_ring_enqueue(u32 slot,
	bench_ring_slot_bpf_writable * sp,
	u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_ring_enqueue);

noinline u64 bench_kmod_bpf_ring_lookup(u32 slot,
	bench_ring_slot_bpf_writable * sp)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return sp ? sp->data : 0;
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_ring_lookup);

noinline void bench_kmod_bpf_ring_update(u32 slot,
	bench_ring_slot_bpf_writable * sp,
	u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_ring_update);

noinline void bench_kmod_bpf_ring_dequeue(u32 slot,
	bench_ring_slot_bpf_writable * sp)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_ring_dequeue);

/* ------------------------------------------------------------------ */
/* Trie                                                                */
/* ------------------------------------------------------------------ */

noinline void bench_kmod_bpf_trie_init(u32 pool_size)
{
	u32 i;

	if (!kmod_bpf_trie_pool)
		return;
	for (i = 0; i < pool_size; i++) {
		kmod_bpf_trie_pool[i].child[0] = 0;
		kmod_bpf_trie_pool[i].child[1] = 0;
		kmod_bpf_trie_pool[i].key_bit  = 0;
		kmod_bpf_trie_pool[i].val      = 0;
	}
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_trie_init);

noinline void bench_kmod_bpf_trie_insert(u32 new_idx,
	bench_trie_node_bpf_writable * node,
	u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_trie_insert);

noinline u64 bench_kmod_bpf_trie_lookup(u64 key,
	bench_trie_node_bpf_writable * node)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return node ? node->val : 0;
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_trie_lookup);

noinline void bench_kmod_bpf_trie_update(u64 key,
	bench_trie_node_bpf_writable * node,
	u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_trie_update);

noinline void bench_kmod_bpf_trie_delete(u64 key,
	bench_trie_node_bpf_writable * node)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_trie_delete);

/* ------------------------------------------------------------------ */
/* RBTree                                                              */
/* ------------------------------------------------------------------ */

noinline void bench_kmod_bpf_rbtree_init(u32 pool_size)
{
	u32 i;

	if (!kmod_bpf_rb_pool)
		return;
	for (i = 0; i < pool_size; i++) {
		kmod_bpf_rb_pool[i].left   = 0;
		kmod_bpf_rb_pool[i].right  = 0;
		kmod_bpf_rb_pool[i].parent = 0;
		kmod_bpf_rb_pool[i].color  = BENCH_RB_BLACK;
		kmod_bpf_rb_pool[i].key    = 0;
		kmod_bpf_rb_pool[i].val    = 0;
	}
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_rbtree_init);

noinline void bench_kmod_bpf_rbtree_insert(u32 new_idx,
	bench_rb_node_bpf_writable * node,
	u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_rbtree_insert);

noinline u64 bench_kmod_bpf_rbtree_lookup(u64 key,
	bench_rb_node_bpf_writable * node)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return node ? node->val : 0;
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_rbtree_lookup);

noinline void bench_kmod_bpf_rbtree_update(u64 key,
	bench_rb_node_bpf_writable * node,
	u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_rbtree_update);

noinline void bench_kmod_bpf_rbtree_delete(u64 key,
	bench_rb_node_bpf_writable * node)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_rbtree_delete);

/* ------------------------------------------------------------------ */
/* Graph                                                               */
/* ------------------------------------------------------------------ */

noinline void bench_kmod_bpf_graph_init(u32 num_nodes, u32 num_edges)
{
	u32 i;

	if (!kmod_bpf_graph_nodes || !kmod_bpf_graph_edges)
		return;
	for (i = 0; i < num_nodes; i++) {
		kmod_bpf_graph_nodes[i].first_edge = (u32)~0;
		kmod_bpf_graph_nodes[i].data       = 0;
	}
	for (i = 0; i < num_edges; i++) {
		kmod_bpf_graph_edges[i].src      = 0;
		kmod_bpf_graph_edges[i].dst      = 0;
		kmod_bpf_graph_edges[i].next_out = (u32)~0;
		kmod_bpf_graph_edges[i].weight   = 0;
	}
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_graph_init);

noinline void bench_kmod_bpf_graph_add_edge(u32 src, u32 dst,
	bench_graph_node_bpf_writable * src_node,
	u32 edge_idx,
	bench_graph_edge_bpf_writable * edge,
	u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_graph_add_edge);

noinline u64 bench_kmod_bpf_graph_lookup(u32 src, u32 dst,
	bench_graph_node_bpf_writable * src_node)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return src_node ? src_node->data : 0;
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_graph_lookup);

noinline void bench_kmod_bpf_graph_update(u32 src, u32 dst,
	bench_graph_edge_bpf_writable * edge,
	u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_graph_update);

noinline void bench_kmod_bpf_graph_delete(u32 src, u32 dst,
	bench_graph_edge_bpf_writable * edge)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_kmod_bpf_graph_delete);

/* ------------------------------------------------------------------ */
/* call_init / call_op dispatchers                                     */
/* ------------------------------------------------------------------ */

static void call_init(int variant, int ds_type, u32 pool_size)
{
	switch (variant) {
	case BENCH_VARIANT_UNDO_LOG:
	case BENCH_VARIANT_LOCK_FUNC:
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_undo_list_init(pool_size);             break;
		case BENCH_DS_RING:   bench_undo_ring_init(pool_size);             break;
		case BENCH_DS_TRIE:   bench_undo_trie_init(pool_size);             break;
		case BENCH_DS_RBTREE: bench_undo_rbtree_init(pool_size);           break;
		case BENCH_DS_GRAPH:  bench_undo_graph_init(pool_size, pool_size); break;
		}
		break;
	case BENCH_VARIANT_ARENA:
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_arena_list_init(pool_size);             break;
		case BENCH_DS_RING:   bench_arena_ring_init(pool_size);             break;
		case BENCH_DS_TRIE:   bench_arena_trie_init(pool_size);             break;
		case BENCH_DS_RBTREE: bench_arena_rbtree_init(pool_size);           break;
		case BENCH_DS_GRAPH:  bench_arena_graph_init(pool_size, pool_size); break;
		}
		break;
	case BENCH_VARIANT_KMOD:
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_kmod_list_init(pool_size);             break;
		case BENCH_DS_RING:   bench_kmod_ring_init(pool_size);             break;
		case BENCH_DS_TRIE:   bench_kmod_trie_init(pool_size);             break;
		case BENCH_DS_RBTREE: bench_kmod_rbtree_init(pool_size);           break;
		case BENCH_DS_GRAPH:  bench_kmod_graph_init(pool_size, pool_size); break;
		}
		break;
	case BENCH_VARIANT_KMOD_BPF:
		atomic_set(&kmod_bpf_trie_alloc, 1);
		atomic_set(&kmod_bpf_rb_alloc, 1);
		atomic_set(&kmod_bpf_graph_edge_alloc, 0);
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_kmod_bpf_list_init(pool_size);             break;
		case BENCH_DS_RING:   bench_kmod_bpf_ring_init(pool_size);             break;
		case BENCH_DS_TRIE:   bench_kmod_bpf_trie_init(pool_size);             break;
		case BENCH_DS_RBTREE: bench_kmod_bpf_rbtree_init(pool_size);           break;
		case BENCH_DS_GRAPH:  bench_kmod_bpf_graph_init(pool_size, pool_size); break;
		}
		break;
	}
}

static void call_op(int variant, int ds_type, int op_type, u64 iter, u32 pool_size)
{
	u32 idx = (u32)(iter % (pool_size > 1 ? pool_size - 1 : 1)) + 1;
	u32 src = (u32)(iter % pool_size);
	u32 dst = (u32)((iter + 1) % pool_size);
	/* Multiplicative hash for key variety without modulo cost */
	u64 key = iter * 2654435761ULL;
	u64 val = ktime_get_mono_fast_ns();

	switch (op_type) {
	case BENCH_OP_INSERT:
		switch (variant) {
		case BENCH_VARIANT_UNDO_LOG:
		case BENCH_VARIANT_LOCK_FUNC:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_undo_list_insert(idx, 0);           break;
			case BENCH_DS_RING:   bench_undo_ring_enqueue(val);             break;
			case BENCH_DS_TRIE:   bench_undo_trie_insert(key, val);         break;
			case BENCH_DS_RBTREE: bench_undo_rbtree_insert(key, val);       break;
			case BENCH_DS_GRAPH:  bench_undo_graph_add_edge(src, dst, val); break;
			}
			break;
		case BENCH_VARIANT_ARENA:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_arena_list_insert(idx, 0);           break;
			case BENCH_DS_RING:   bench_arena_ring_enqueue(val);             break;
			case BENCH_DS_TRIE:   bench_arena_trie_insert(key, val);         break;
			case BENCH_DS_RBTREE: bench_arena_rbtree_insert(key, val);       break;
			case BENCH_DS_GRAPH:  bench_arena_graph_add_edge(src, dst, val); break;
			}
			break;
		case BENCH_VARIANT_KMOD:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_kmod_list_insert(idx, 0);           break;
			case BENCH_DS_RING:   bench_kmod_ring_enqueue(val);             break;
			case BENCH_DS_TRIE:   bench_kmod_trie_insert(key, val);         break;
			case BENCH_DS_RBTREE: bench_kmod_rbtree_insert(key, val);       break;
			case BENCH_DS_GRAPH:  bench_kmod_graph_add_edge(src, dst, val); break;
			}
			break;
		case BENCH_VARIANT_KMOD_BPF: {
			u32 nidx = idx < KMOD_MAX_POOL ? idx : 0;
			u32 eidx;

			switch (ds_type) {
			case BENCH_DS_LIST:
				bench_kmod_bpf_list_insert(nidx,
					&kmod_bpf_list_pool[nidx], 0);
				break;
			case BENCH_DS_RING:
				bench_kmod_bpf_ring_enqueue(src,
					&kmod_bpf_ring_pool[src], val);
				break;
			case BENCH_DS_TRIE: {
				u32 tidx = (u32)atomic_inc_return(&kmod_bpf_trie_alloc) %
					   KMOD_MAX_POOL;
				bench_kmod_bpf_trie_insert(tidx,
					&kmod_bpf_trie_pool[tidx], key, val);
				break;
			}
			case BENCH_DS_RBTREE: {
				u32 ridx = (u32)atomic_inc_return(&kmod_bpf_rb_alloc) %
					   KMOD_MAX_POOL;
				bench_kmod_bpf_rbtree_insert(ridx,
					&kmod_bpf_rb_pool[ridx], key, val);
				break;
			}
			case BENCH_DS_GRAPH:
				eidx = (u32)atomic_inc_return(&kmod_bpf_graph_edge_alloc) %
				       KMOD_MAX_POOL;
				bench_kmod_bpf_graph_add_edge(src, dst,
					&kmod_bpf_graph_nodes[src], eidx,
					&kmod_bpf_graph_edges[eidx], val);
				break;
			}
			break;
		}
		}
		break;

	case BENCH_OP_LOOKUP:
		switch (variant) {
		case BENCH_VARIANT_UNDO_LOG:
		case BENCH_VARIANT_LOCK_FUNC:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_undo_list_lookup(idx);           break;
			case BENCH_DS_RING:   bench_undo_ring_lookup(idx);           break;
			case BENCH_DS_TRIE:   bench_undo_trie_lookup(key);           break;
			case BENCH_DS_RBTREE: bench_undo_rbtree_lookup(key);         break;
			case BENCH_DS_GRAPH:  bench_undo_graph_lookup(src, dst);     break;
			}
			break;
		case BENCH_VARIANT_ARENA:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_arena_list_lookup(idx);          break;
			case BENCH_DS_RING:   bench_arena_ring_lookup(idx);          break;
			case BENCH_DS_TRIE:   bench_arena_trie_lookup(key);          break;
			case BENCH_DS_RBTREE: bench_arena_rbtree_lookup(key);        break;
			case BENCH_DS_GRAPH:  bench_arena_graph_lookup(src, dst);    break;
			}
			break;
		case BENCH_VARIANT_KMOD:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_kmod_list_lookup(idx);           break;
			case BENCH_DS_RING:   bench_kmod_ring_lookup(idx);           break;
			case BENCH_DS_TRIE:   bench_kmod_trie_lookup(key);           break;
			case BENCH_DS_RBTREE: bench_kmod_rbtree_lookup(key);         break;
			case BENCH_DS_GRAPH:  bench_kmod_graph_lookup(src, dst);     break;
			}
			break;
		case BENCH_VARIANT_KMOD_BPF: {
			u32 nidx = idx < KMOD_MAX_POOL ? idx : 0;

			switch (ds_type) {
			case BENCH_DS_LIST:
				bench_kmod_bpf_list_lookup(nidx,
					&kmod_bpf_list_pool[nidx]);
				break;
			case BENCH_DS_RING:
				bench_kmod_bpf_ring_lookup(src,
					&kmod_bpf_ring_pool[src]);
				break;
			case BENCH_DS_TRIE:
				bench_kmod_bpf_trie_lookup(key,
					&kmod_bpf_trie_pool[nidx]);
				break;
			case BENCH_DS_RBTREE:
				bench_kmod_bpf_rbtree_lookup(key,
					&kmod_bpf_rb_pool[nidx]);
				break;
			case BENCH_DS_GRAPH:
				bench_kmod_bpf_graph_lookup(src, dst,
					&kmod_bpf_graph_nodes[src]);
				break;
			}
			break;
		}
		}
		break;

	case BENCH_OP_UPDATE:
		switch (variant) {
		case BENCH_VARIANT_UNDO_LOG:
		case BENCH_VARIANT_LOCK_FUNC:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_undo_list_update(idx, val);          break;
			case BENCH_DS_RING:   bench_undo_ring_update(idx, val);          break;
			case BENCH_DS_TRIE:   bench_undo_trie_update(key, val);          break;
			case BENCH_DS_RBTREE: bench_undo_rbtree_update(key, val);        break;
			case BENCH_DS_GRAPH:  bench_undo_graph_update(src, dst, val);    break;
			}
			break;
		case BENCH_VARIANT_ARENA:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_arena_list_update(idx, val);         break;
			case BENCH_DS_RING:   bench_arena_ring_update(idx, val);         break;
			case BENCH_DS_TRIE:   bench_arena_trie_update(key, val);         break;
			case BENCH_DS_RBTREE: bench_arena_rbtree_update(key, val);       break;
			case BENCH_DS_GRAPH:  bench_arena_graph_update(src, dst, val);   break;
			}
			break;
		case BENCH_VARIANT_KMOD:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_kmod_list_update(idx, val);          break;
			case BENCH_DS_RING:   bench_kmod_ring_update(idx, val);          break;
			case BENCH_DS_TRIE:   bench_kmod_trie_update(key, val);          break;
			case BENCH_DS_RBTREE: bench_kmod_rbtree_update(key, val);        break;
			case BENCH_DS_GRAPH:  bench_kmod_graph_update(src, dst, val);    break;
			}
			break;
		case BENCH_VARIANT_KMOD_BPF: {
			u32 nidx = idx < KMOD_MAX_POOL ? idx : 0;
			u32 eidx = src < KMOD_MAX_POOL ? src : 0;

			switch (ds_type) {
			case BENCH_DS_LIST:
				bench_kmod_bpf_list_update(nidx,
					&kmod_bpf_list_pool[nidx], val);
				break;
			case BENCH_DS_RING:
				bench_kmod_bpf_ring_update(src,
					&kmod_bpf_ring_pool[src], val);
				break;
			case BENCH_DS_TRIE:
				bench_kmod_bpf_trie_update(key,
					&kmod_bpf_trie_pool[nidx], val);
				break;
			case BENCH_DS_RBTREE:
				bench_kmod_bpf_rbtree_update(key,
					&kmod_bpf_rb_pool[nidx], val);
				break;
			case BENCH_DS_GRAPH:
				bench_kmod_bpf_graph_update(src, dst,
					&kmod_bpf_graph_edges[eidx], val);
				break;
			}
			break;
		}
		}
		break;

	case BENCH_OP_DELETE:
		switch (variant) {
		case BENCH_VARIANT_UNDO_LOG:
		case BENCH_VARIANT_LOCK_FUNC:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_undo_list_delete();              break;
			case BENCH_DS_RING:   bench_undo_ring_dequeue(idx);         break;
			case BENCH_DS_TRIE:   bench_undo_trie_delete(key);          break;
			case BENCH_DS_RBTREE: bench_undo_rbtree_delete(key);        break;
			case BENCH_DS_GRAPH:  bench_undo_graph_delete(src, dst);    break;
			}
			break;
		case BENCH_VARIANT_ARENA:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_arena_list_delete();             break;
			case BENCH_DS_RING:   bench_arena_ring_dequeue(idx);        break;
			case BENCH_DS_TRIE:   bench_arena_trie_delete(key);         break;
			case BENCH_DS_RBTREE: bench_arena_rbtree_delete(key);       break;
			case BENCH_DS_GRAPH:  bench_arena_graph_delete(src, dst);   break;
			}
			break;
		case BENCH_VARIANT_KMOD:
			switch (ds_type) {
			case BENCH_DS_LIST:   bench_kmod_list_delete();              break;
			case BENCH_DS_RING:   bench_kmod_ring_dequeue(idx);         break;
			case BENCH_DS_TRIE:   bench_kmod_trie_delete(key);          break;
			case BENCH_DS_RBTREE: bench_kmod_rbtree_delete(key);        break;
			case BENCH_DS_GRAPH:  bench_kmod_graph_delete(src, dst);    break;
			}
			break;
		case BENCH_VARIANT_KMOD_BPF: {
			u32 nidx = idx < KMOD_MAX_POOL ? idx : 0;
			u32 eidx = src < KMOD_MAX_POOL ? src : 0;

			switch (ds_type) {
			case BENCH_DS_LIST:
				bench_kmod_bpf_list_delete(nidx,
					&kmod_bpf_list_pool[nidx]);
				break;
			case BENCH_DS_RING:
				bench_kmod_bpf_ring_dequeue(src,
					&kmod_bpf_ring_pool[src]);
				break;
			case BENCH_DS_TRIE:
				bench_kmod_bpf_trie_delete(key,
					&kmod_bpf_trie_pool[nidx]);
				break;
			case BENCH_DS_RBTREE:
				bench_kmod_bpf_rbtree_delete(key,
					&kmod_bpf_rb_pool[nidx]);
				break;
			case BENCH_DS_GRAPH:
				bench_kmod_bpf_graph_delete(src, dst,
					&kmod_bpf_graph_edges[eidx]);
				break;
			}
			break;
		}
		}
		break;
	}
}

/* Pre-populate the data structure with init_size insert operations. */
static void call_prefill(int variant, int ds_type, u32 init_size, u32 pool_size)
{
	u64 i;

	for (i = 0; i < init_size; i++)
		call_op(variant, ds_type, BENCH_OP_INSERT, i, pool_size);
}

/* ------------------------------------------------------------------ */
/* Benchmark kthread                                                   */
/* ------------------------------------------------------------------ */

struct bench_worker_ctx {
	int cpu;
	int variant;
	int ds_type;
	int op_type;
	u32 pool_size;
	u32 warmup_ms;
	u32 bench_ms;
	struct completion done;
};

static int bench_worker(void *arg)
{
	struct bench_worker_ctx *ctx = arg;
	struct bench_cpu_stats *s = per_cpu_ptr(&bench_cpu_stats, ctx->cpu);
	ktime_t t0;
	u64 iter = 0, ops = 0, lat_sum = 0, sample_cnt = 0;
	u32 pool_size = ctx->pool_size;

	/* Zero this CPU's stats */
	memset(s, 0, sizeof(*s));

	/* Spin until all kthreads are ready for a synchronized start */
	atomic_inc(&bench_ready_cnt);
	while (atomic_read(&bench_ready_cnt) < bench_nthreads)
		cpu_relax();

	/* Warmup: run ops but discard stats */
	t0 = ktime_get();
	while (ktime_ms_delta(ktime_get(), t0) < ctx->warmup_ms) {
		call_op(ctx->variant, ctx->ds_type, ctx->op_type, iter, pool_size);
		iter++;
	}

	/* Reset per-CPU counter that stubs may have bumped during warmup */
	s->ops = 0;
	ops = 0;

	/* Measurement window */
	t0 = ktime_get();
	while (ktime_ms_delta(ktime_get(), t0) < ctx->bench_ms) {
		u64 t_start = ktime_get_mono_fast_ns();

		call_op(ctx->variant, ctx->ds_type, ctx->op_type, iter, pool_size);

		u64 lat = ktime_get_mono_fast_ns() - t_start;
		iter++;
		ops++;
		lat_sum += lat;

		if (!(ops & (BENCH_SAMPLE_RATE - 1))) {
			s->samples[sample_cnt & (BENCH_LATENCY_SAMPLES - 1)] = lat;
			sample_cnt++;
		}
	}

	s->ops        = ops;
	s->lat_sum_ns = lat_sum;
	s->sample_cnt = (u32)min_t(u64, sample_cnt, BENCH_LATENCY_SAMPLES);

	complete(&ctx->done);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Percentile computation                                              */
/* ------------------------------------------------------------------ */

static int cmp_u64(const void *a, const void *b)
{
	u64 x = *(const u64 *)a;
	u64 y = *(const u64 *)b;

	if (x < y) return -1;
	if (x > y) return  1;
	return 0;
}

static u64 compute_percentile(u64 *arr, u32 n, int pct)
{
	if (!n)
		return 0;
	return arr[(u64)n * pct / 100];
}

/* ------------------------------------------------------------------ */
/* Run benchmark: spawn kthreads, wait, collect results               */
/* ------------------------------------------------------------------ */

static int run_benchmark(int variant)
{
	struct bench_worker_ctx *ctxs;
	struct task_struct **tasks;
	int n = bench_cfg.num_threads;
	int i, cpu, err = 0;
	u64 *merge_buf = NULL;
	u32 merge_cnt = 0;

	if (n > BENCH_MAX_CPUS || n < 1)
		return -EINVAL;

	ctxs  = kcalloc(n, sizeof(*ctxs),  GFP_KERNEL);
	tasks = kcalloc(n, sizeof(*tasks), GFP_KERNEL);
	if (!ctxs || !tasks) {
		err = -ENOMEM;
		goto out_free_arrays;
	}

	atomic_set(&bench_ready_cnt, 0);
	bench_nthreads = n;

	/* Call init on the main thread (BPF fentry fires here) */
	call_init(variant, bench_cfg.ds_type, bench_cfg.pool_size);

	/* Pre-populate with init_size elements so lookup/update/delete find data */
	if (bench_cfg.init_size)
		call_prefill(variant, bench_cfg.ds_type,
			     bench_cfg.init_size, bench_cfg.pool_size);

	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < n; i++) {
		init_completion(&ctxs[i].done);
		ctxs[i].cpu       = cpu;
		ctxs[i].variant   = variant;
		ctxs[i].ds_type   = bench_cfg.ds_type;
		ctxs[i].op_type   = bench_cfg.op_type;
		ctxs[i].pool_size = bench_cfg.pool_size;
		ctxs[i].warmup_ms = bench_cfg.warmup_ms;
		ctxs[i].bench_ms  = bench_cfg.bench_ms;

		tasks[i] = kthread_create(bench_worker, &ctxs[i],
					  "bench_worker/%d", i);
		if (IS_ERR(tasks[i])) {
			err = PTR_ERR(tasks[i]);
			n = i;
			goto wait_done;
		}
		kthread_bind(tasks[i], cpu);
		wake_up_process(tasks[i]);

		cpu = cpumask_next(cpu, cpu_online_mask);
		if (cpu >= nr_cpu_ids)
			cpu = cpumask_first(cpu_online_mask);
	}

wait_done:
	for (i = 0; i < n; i++)
		wait_for_completion(&ctxs[i].done);

	/* Aggregate per-CPU stats */
	memset(&bench_results, 0, sizeof(bench_results));
	bench_results.num_cpus = n;

	merge_buf = vmalloc(array_size(n * BENCH_LATENCY_SAMPLES, sizeof(u64)));
	if (!merge_buf) {
		err = -ENOMEM;
		goto out_free_arrays;
	}

	for (i = 0; i < n; i++) {
		struct bench_cpu_stats *s = per_cpu_ptr(&bench_cpu_stats, ctxs[i].cpu);
		u32 sc = s->sample_cnt;

		bench_results.total_ops       += s->ops;
		bench_results.total_lat_sum_ns += s->lat_sum_ns;

		if (i < BENCH_MAX_CPUS) {
			bench_results.cpu[i].ops        = s->ops;
			bench_results.cpu[i].lat_sum_ns = s->lat_sum_ns;
		}

		if (sc && merge_cnt + sc <= (u32)(n * BENCH_LATENCY_SAMPLES)) {
			memcpy(merge_buf + merge_cnt, s->samples, sc * sizeof(u64));
			merge_cnt += sc;
		}
	}

	if (merge_cnt) {
		sort(merge_buf, merge_cnt, sizeof(u64), cmp_u64, NULL);
		bench_results.p50_lat_ns = compute_percentile(merge_buf, merge_cnt, 50);
		bench_results.p95_lat_ns = compute_percentile(merge_buf, merge_cnt, 95);
		bench_results.p99_lat_ns = compute_percentile(merge_buf, merge_cnt, 99);
	}

	vfree(merge_buf);
out_free_arrays:
	kfree(ctxs);
	kfree(tasks);
	return err;
}

/* ------------------------------------------------------------------ */
/* Pool allocation / deallocation                                      */
/* ------------------------------------------------------------------ */

static int alloc_kmod_pools(u32 pool_size)
{
	kmod_list_pool    = kcalloc(pool_size, sizeof(*kmod_list_pool),    GFP_KERNEL);
	kmod_ring_pool    = kcalloc(pool_size, sizeof(*kmod_ring_pool),    GFP_KERNEL);
	kmod_trie_pool    = kcalloc(pool_size, sizeof(*kmod_trie_pool),    GFP_KERNEL);
	kmod_rb_pool      = kcalloc(pool_size, sizeof(*kmod_rb_pool),      GFP_KERNEL);
	kmod_graph_nodes  = kcalloc(pool_size, sizeof(*kmod_graph_nodes),  GFP_KERNEL);
	kmod_graph_edges  = kcalloc(pool_size, sizeof(*kmod_graph_edges),  GFP_KERNEL);

	/* kmod_bpf pools: same sizes, no embedded spinlock */
	kmod_bpf_list_pool    = kcalloc(pool_size, sizeof(*kmod_bpf_list_pool),    GFP_KERNEL);
	kmod_bpf_ring_pool    = kcalloc(pool_size, sizeof(*kmod_bpf_ring_pool),    GFP_KERNEL);
	kmod_bpf_trie_pool    = kcalloc(pool_size, sizeof(*kmod_bpf_trie_pool),    GFP_KERNEL);
	kmod_bpf_rb_pool      = kcalloc(pool_size, sizeof(*kmod_bpf_rb_pool),      GFP_KERNEL);
	kmod_bpf_graph_nodes  = kcalloc(pool_size, sizeof(*kmod_bpf_graph_nodes),  GFP_KERNEL);
	kmod_bpf_graph_edges  = kcalloc(pool_size, sizeof(*kmod_bpf_graph_edges),  GFP_KERNEL);

	return (kmod_list_pool && kmod_ring_pool && kmod_trie_pool &&
		kmod_rb_pool && kmod_graph_nodes && kmod_graph_edges &&
		kmod_bpf_list_pool && kmod_bpf_ring_pool && kmod_bpf_trie_pool &&
		kmod_bpf_rb_pool && kmod_bpf_graph_nodes && kmod_bpf_graph_edges) ? 0 : -ENOMEM;
}

static void free_kmod_pools(void)
{
	kfree(kmod_list_pool);    kmod_list_pool   = NULL;
	kfree(kmod_ring_pool);    kmod_ring_pool   = NULL;
	kfree(kmod_trie_pool);    kmod_trie_pool   = NULL;
	kfree(kmod_rb_pool);      kmod_rb_pool     = NULL;
	kfree(kmod_graph_nodes);  kmod_graph_nodes = NULL;
	kfree(kmod_graph_edges);  kmod_graph_edges = NULL;

	kfree(kmod_bpf_list_pool);    kmod_bpf_list_pool    = NULL;
	kfree(kmod_bpf_ring_pool);    kmod_bpf_ring_pool    = NULL;
	kfree(kmod_bpf_trie_pool);    kmod_bpf_trie_pool    = NULL;
	kfree(kmod_bpf_rb_pool);      kmod_bpf_rb_pool      = NULL;
	kfree(kmod_bpf_graph_nodes);  kmod_bpf_graph_nodes  = NULL;
	kfree(kmod_bpf_graph_edges);  kmod_bpf_graph_edges  = NULL;
}

/* ------------------------------------------------------------------ */
/* ioctl                                                               */
/* ------------------------------------------------------------------ */

static long bench_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	switch (cmd) {
	case BENCH_IOCTL_SET_PARAMS: {
		struct bench_params p;

		if (copy_from_user(&p, (void __user *)arg, sizeof(p)))
			return -EFAULT;
		if (p.ds_type > BENCH_DS_GRAPH || p.variant > BENCH_VARIANT_LOCK_FUNC)
			return -EINVAL;
		if (p.op_type > BENCH_OP_DELETE)
			return -EINVAL;
		if (!p.num_threads || p.num_threads > BENCH_MAX_CPUS)
			return -EINVAL;
		if (!p.pool_size || p.pool_size > KMOD_MAX_POOL)
			return -EINVAL;
		if (p.init_size > p.pool_size)
			return -EINVAL;
		bench_cfg = p;
		if (!bench_cfg.warmup_ms) bench_cfg.warmup_ms = 5000;
		if (!bench_cfg.bench_ms)  bench_cfg.bench_ms  = 15000;
		break;
	}

	case BENCH_IOCTL_EBPF_READY:
		err = run_benchmark(bench_cfg.variant);
		break;

	case BENCH_IOCTL_RUN_KMOD:
		err = run_benchmark(BENCH_VARIANT_KMOD);
		break;

	case BENCH_IOCTL_GET_RESULTS:
		if (copy_to_user((void __user *)arg, &bench_results,
				 sizeof(bench_results)))
			return -EFAULT;
		break;

	default:
		return -ENOTTY;
	}

	return err;
}

static const struct file_operations bench_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = bench_ioctl,
	.compat_ioctl   = bench_ioctl,
};

static struct miscdevice bench_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "bench_spinlock",
	.fops  = &bench_fops,
	.mode  = 0666,
};

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */

static int __init bench_spinlock_init(void)
{
	int err;

	err = alloc_kmod_pools(KMOD_MAX_POOL);
	if (err)
		return err;

	err = misc_register(&bench_miscdev);
	if (err) {
		free_kmod_pools();
		return err;
	}

	pr_info("bench_spinlock: registered /dev/bench_spinlock\n");
	return 0;
}

static void __exit bench_spinlock_exit(void)
{
	misc_deregister(&bench_miscdev);
	free_kmod_pools();
}

module_init(bench_spinlock_init);
module_exit(bench_spinlock_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BPF spinlock undo-log overhead benchmark");
