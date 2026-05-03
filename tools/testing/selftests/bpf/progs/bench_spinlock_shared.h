/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared definitions for the BPF spinlock undo-log benchmark suite.
 * Included by BPF programs (progs/), the kernel module (bench_kmod/),
 * and the userspace harness (benchs/).
 */
#ifndef BENCH_SPINLOCK_SHARED_H
#define BENCH_SPINLOCK_SHARED_H

/* ------------------------------------------------------------------ */
/* Pool / structure size limits                                        */
/* ------------------------------------------------------------------ */

#define BENCH_MAX_POOL   256   /* nodes per data structure pool */
#define BENCH_RING_SLOTS BENCH_MAX_POOL

/* ------------------------------------------------------------------ */
/* ioctl constants (kernel module and userspace)                       */
/* ------------------------------------------------------------------ */

#define BENCH_DS_LIST    0
#define BENCH_DS_RING    1
#define BENCH_DS_TRIE    2
#define BENCH_DS_RBTREE  3
#define BENCH_DS_GRAPH   4

#define BENCH_VARIANT_UNDO_LOG  0
#define BENCH_VARIANT_ARENA     1
#define BENCH_VARIANT_KMOD      2

#define BENCH_IOC_MAGIC    'B'
#define BENCH_MAX_CPUS     256
#define BENCH_LATENCY_SAMPLES 8192


struct bench_params {
	__u32 ds_type;
	__u32 variant;
	__u32 num_threads;
	__u32 pool_size;
	__u32 warmup_ms;
	__u32 bench_ms;
};

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

#if !defined(__KERNEL__) && !defined(__BPF__)
#include <sys/ioctl.h>
#define BENCH_IOCTL_SET_PARAMS  _IOW(BENCH_IOC_MAGIC, 1, struct bench_params)
#define BENCH_IOCTL_EBPF_READY  _IO(BENCH_IOC_MAGIC, 2)
#define BENCH_IOCTL_RUN_KMOD    _IO(BENCH_IOC_MAGIC, 3)
#define BENCH_IOCTL_GET_RESULTS _IOR(BENCH_IOC_MAGIC, 4, struct bench_all_results)
#endif /* !__KERNEL__ && !__BPF__ */

/* ------------------------------------------------------------------ */
/* Node types (plain, no lock embedded)                               */
/* Used by undo_log BPF programs; locks live in a parallel map.       */
/* Also needed in userspace for skeleton type compatibility.          */
/* ------------------------------------------------------------------ */

struct bench_list_node {
	__u32 next_idx;
	__u64 data;
};

struct bench_ring_slot {
	__u64 data;
	__u32 valid;
};

struct bench_trie_node {
	__u32 child[2];
	__u32 key_bit;
	__u64 val;
};

#define BENCH_RB_BLACK 0
#define BENCH_RB_RED   1

struct bench_rb_node {
	__u32 left, right, parent;
	__u32 color;
	__u64 key;
	__u64 val;
};

struct bench_graph_node {
	__u32 first_edge;
	__u64 data;
};

struct bench_graph_edge {
	__u32 src, dst, next_out;
	__u64 weight;
};

/* ------------------------------------------------------------------ */
/* CSV output header (userspace only)                                  */
/* ------------------------------------------------------------------ */

#define BENCH_CSV_HEADER \
	"variant,ds,threads,pool_size,total_ops,ops_per_sec," \
	"avg_lat_ns,p50_lat_ns,p95_lat_ns,p99_lat_ns\n"

#if !defined(__BPF__)
static const char * const bench_ds_names[] = {
	[BENCH_DS_LIST]   = "list",
	[BENCH_DS_RING]   = "ring",
	[BENCH_DS_TRIE]   = "trie",
	[BENCH_DS_RBTREE] = "rbtree",
	[BENCH_DS_GRAPH]  = "graph",
};

static const char * const bench_variant_names[] = {
	[BENCH_VARIANT_UNDO_LOG] = "undo_log",
	[BENCH_VARIANT_ARENA]    = "arena",
	[BENCH_VARIANT_KMOD]     = "kmod",
};
#endif /* !__BPF__ */

#endif /* BENCH_SPINLOCK_SHARED_H */
