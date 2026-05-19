// SPDX-License-Identifier: GPL-2.0
/*
 * Declarations for the kmod_bpf benchmark variant.
 *
 * Contains struct types, pool extern declarations, writable typedef aliases,
 * and the __writable_bpf macro (kept for Clang builds).
 *
 * Typedef names ending in "_bpf_writable" are recognised by the BPF verifier
 * (btf_ctx_access) and cause MEM_WRITE to be set on the register, allowing
 * writes inside bpf_spin_lock critical sections.  This convention works with
 * both GCC and Clang module builds (no Clang-specific attributes required).
 */

#define BENCH_MAX_POOL   256
#define BENCH_RING_SLOTS BENCH_MAX_POOL
#define BENCH_RB_BLACK   0
#define BENCH_RB_RED     1

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

/*
 * Writable typedef aliases — names ending in "_bpf_writable" are detected by
 * btf_ctx_access() and set MEM_WRITE on the corresponding fentry argument.
 */
typedef struct bench_list_node  bench_list_node_bpf_writable;
typedef struct bench_ring_slot  bench_ring_slot_bpf_writable;
typedef struct bench_trie_node  bench_trie_node_bpf_writable;
typedef struct bench_rb_node    bench_rb_node_bpf_writable;
typedef struct bench_graph_node bench_graph_node_bpf_writable;
typedef struct bench_graph_edge bench_graph_edge_bpf_writable;

#ifndef __writable_bpf
/*
 * Kept for Clang builds: btf_decl_tag on a function parameter emits a
 * BTF_KIND_DECL_TAG with component_idx set to the parameter's position.
 * The BPF verifier checks for this tag in btf_ctx_access() and sets
 * MEM_WRITE on the register.  GCC silently ignores this attribute; the
 * _bpf_writable typedef convention above covers GCC module builds.
 */
#ifdef __KERNEL__
#define __writable_bpf __attribute__((btf_decl_tag("writable_bpf")))
#else
#define __writable_bpf
#endif
#endif

/* Pools defined in bench_spinlock_kmod.c */
extern struct bench_list_node  *kmod_bpf_list_pool;
extern struct bench_ring_slot  *kmod_bpf_ring_pool;
extern struct bench_trie_node  *kmod_bpf_trie_pool;
extern struct bench_rb_node    *kmod_bpf_rb_pool;
extern struct bench_graph_node *kmod_bpf_graph_nodes;
extern struct bench_graph_edge *kmod_bpf_graph_edges;
