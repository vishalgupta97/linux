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

#define BENCH_IOC_MAGIC    'B'

struct bench_params {
	__u32 ds_type;
	__u32 variant;
	__u32 num_threads;
	__u32 pool_size;
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
/* kmod data structure implementations                                 */
/* ------------------------------------------------------------------ */

noinline void bench_kmod_list_init(u32 pool_size)
{
	u32 i;

	if (!kmod_list_pool)
		return;
	for (i = 0; i < pool_size; i++) {
		spin_lock_init(&kmod_list_pool[i].lock);
		kmod_list_pool[i].next_idx = (u32)~0;
		kmod_list_pool[i].data = 0;
	}
	kmod_list_head_idx = (u32)~0;
}

noinline void bench_kmod_list_insert(u32 new_idx, u32 head_lock_idx)
{
	if (new_idx >= KMOD_MAX_POOL)
		return;

	spin_lock(&kmod_list_pool[head_lock_idx].lock);
	spin_lock(&kmod_list_pool[new_idx].lock);

	kmod_list_pool[new_idx].next_idx = kmod_list_head_idx;
	kmod_list_pool[new_idx].data     = ktime_get_mono_fast_ns();
	kmod_list_head_idx               = new_idx;

	spin_unlock(&kmod_list_pool[new_idx].lock);
	spin_unlock(&kmod_list_pool[head_lock_idx].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

noinline void bench_kmod_ring_init(u32 num_slots)
{
	u32 i;

	if (!kmod_ring_pool)
		return;
	kmod_ring_size = num_slots;
	atomic_set(&kmod_ring_head, 0);
	for (i = 0; i < num_slots; i++) {
		spin_lock_init(&kmod_ring_pool[i].lock);
		kmod_ring_pool[i].valid = 0;
		kmod_ring_pool[i].data  = 0;
	}
}

noinline void bench_kmod_ring_enqueue(u64 val)
{
	u32 slot;

	if (!kmod_ring_pool || !kmod_ring_size)
		return;
	slot = (u32)(atomic_fetch_add(1, &kmod_ring_head) % kmod_ring_size);

	spin_lock(&kmod_ring_pool[slot].lock);
	kmod_ring_pool[slot].data  = val;
	kmod_ring_pool[slot].valid = 1;
	spin_unlock(&kmod_ring_pool[slot].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

noinline void bench_kmod_trie_init(u32 pool_size)
{
	if (!kmod_trie_pool)
		return;
	memset(kmod_trie_pool, 0, pool_size * sizeof(*kmod_trie_pool));
	kmod_trie_root = 0;
	atomic_set(&kmod_trie_alloc, 1);
	spin_lock_init(&kmod_trie_lock);
}

/* Binary trie insert on 64-bit key, one bit per level from MSB */
noinline void bench_kmod_trie_insert(u64 key, u64 val)
{
	u32 cur, parent, bit, new_node;
	int dir;

	spin_lock(&kmod_trie_lock);
	cur = kmod_trie_root;
	parent = 0;
	dir = 0;

	for (bit = 63; ; bit--) {
		int b = (key >> bit) & 1;

		if (!cur) {
			new_node = (u32)atomic_fetch_add(1, &kmod_trie_alloc);
			if (new_node >= KMOD_MAX_POOL)
				goto out;
			kmod_trie_pool[new_node].child[0] = 0;
			kmod_trie_pool[new_node].child[1] = 0;
			kmod_trie_pool[new_node].key_bit   = bit;
			kmod_trie_pool[new_node].val       = val;
			if (parent)
				kmod_trie_pool[parent].child[dir] = new_node;
			else
				kmod_trie_root = new_node;
			break;
		}
		parent = cur;
		dir = b;
		cur = kmod_trie_pool[cur].child[b];
		if (!bit)
			break;
	}
out:
	spin_unlock(&kmod_trie_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}

noinline void bench_kmod_rbtree_init(u32 pool_size)
{
	if (!kmod_rb_pool)
		return;
	memset(kmod_rb_pool, 0, pool_size * sizeof(*kmod_rb_pool));
	/* node 0 = nil sentinel (black) */
	kmod_rb_pool[0].color = KMOD_RB_BLACK;
	kmod_rb_pool[0].left = kmod_rb_pool[0].right = kmod_rb_pool[0].parent = 0;
	kmod_rb_root = 0;
	atomic_set(&kmod_rb_alloc, 1);
	spin_lock_init(&kmod_rb_lock);
}

static void kmod_rb_rotate_left(u32 x)
{
	u32 y = kmod_rb_pool[x].right;

	kmod_rb_pool[x].right = kmod_rb_pool[y].left;
	if (kmod_rb_pool[y].left)
		kmod_rb_pool[kmod_rb_pool[y].left].parent = x;
	kmod_rb_pool[y].parent = kmod_rb_pool[x].parent;
	if (!kmod_rb_pool[x].parent)
		kmod_rb_root = y;
	else if (x == kmod_rb_pool[kmod_rb_pool[x].parent].left)
		kmod_rb_pool[kmod_rb_pool[x].parent].left = y;
	else
		kmod_rb_pool[kmod_rb_pool[x].parent].right = y;
	kmod_rb_pool[y].left = x;
	kmod_rb_pool[x].parent = y;
}

static void kmod_rb_rotate_right(u32 x)
{
	u32 y = kmod_rb_pool[x].left;

	kmod_rb_pool[x].left = kmod_rb_pool[y].right;
	if (kmod_rb_pool[y].right)
		kmod_rb_pool[kmod_rb_pool[y].right].parent = x;
	kmod_rb_pool[y].parent = kmod_rb_pool[x].parent;
	if (!kmod_rb_pool[x].parent)
		kmod_rb_root = y;
	else if (x == kmod_rb_pool[kmod_rb_pool[x].parent].right)
		kmod_rb_pool[kmod_rb_pool[x].parent].right = y;
	else
		kmod_rb_pool[kmod_rb_pool[x].parent].left = y;
	kmod_rb_pool[y].right = x;
	kmod_rb_pool[x].parent = y;
}

static void kmod_rb_insert_fixup(u32 z)
{
	u32 y;
	int depth = 0;

	while (kmod_rb_pool[kmod_rb_pool[z].parent].color == KMOD_RB_RED &&
	       depth++ < 64) {
		u32 p = kmod_rb_pool[z].parent;
		u32 g = kmod_rb_pool[p].parent;

		if (p == kmod_rb_pool[g].left) {
			y = kmod_rb_pool[g].right;
			if (kmod_rb_pool[y].color == KMOD_RB_RED) {
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[y].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				z = g;
			} else {
				if (z == kmod_rb_pool[p].right) {
					z = p;
					kmod_rb_rotate_left(z);
					p = kmod_rb_pool[z].parent;
					g = kmod_rb_pool[p].parent;
				}
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				kmod_rb_rotate_right(g);
			}
		} else {
			y = kmod_rb_pool[g].left;
			if (kmod_rb_pool[y].color == KMOD_RB_RED) {
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[y].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				z = g;
			} else {
				if (z == kmod_rb_pool[p].left) {
					z = p;
					kmod_rb_rotate_right(z);
					p = kmod_rb_pool[z].parent;
					g = kmod_rb_pool[p].parent;
				}
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				kmod_rb_rotate_left(g);
			}
		}
	}
	kmod_rb_pool[kmod_rb_root].color = KMOD_RB_BLACK;
}

noinline void bench_kmod_rbtree_insert(u64 key, u64 val)
{
	u32 z, p, x;

	spin_lock(&kmod_rb_lock);
	z = (u32)atomic_fetch_add(1, &kmod_rb_alloc);
	if (z >= KMOD_MAX_POOL)
		goto out;

	kmod_rb_pool[z].key    = key;
	kmod_rb_pool[z].val    = val;
	kmod_rb_pool[z].color  = KMOD_RB_RED;
	kmod_rb_pool[z].left   = 0;
	kmod_rb_pool[z].right  = 0;
	kmod_rb_pool[z].parent = 0;

	p = 0;
	x = kmod_rb_root;
	while (x) {
		p = x;
		if (key < kmod_rb_pool[x].key)
			x = kmod_rb_pool[x].left;
		else
			x = kmod_rb_pool[x].right;
	}
	kmod_rb_pool[z].parent = p;
	if (!p) {
		kmod_rb_root = z;
	} else if (key < kmod_rb_pool[p].key) {
		kmod_rb_pool[p].left = z;
	} else {
		kmod_rb_pool[p].right = z;
	}
	kmod_rb_insert_fixup(z);
out:
	spin_unlock(&kmod_rb_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}

noinline void bench_kmod_graph_init(u32 num_nodes, u32 num_edges)
{
	u32 i;

	if (!kmod_graph_nodes || !kmod_graph_edges)
		return;
	kmod_graph_num_nodes = num_nodes;
	for (i = 0; i < num_nodes; i++) {
		kmod_graph_nodes[i].first_edge = (u32)~0;
		kmod_graph_nodes[i].data = 0;
	}
	atomic_set(&kmod_graph_edge_alloc, 0);
	spin_lock_init(&kmod_graph_lock);
}

noinline void bench_kmod_graph_add_edge(u32 src, u32 dst, u64 weight)
{
	u32 e;

	if (!kmod_graph_nodes || !kmod_graph_edges)
		return;

	spin_lock(&kmod_graph_lock);
	e = (u32)atomic_fetch_add(1, &kmod_graph_edge_alloc);
	if (e >= KMOD_MAX_POOL || src >= kmod_graph_num_nodes)
		goto out;

	kmod_graph_edges[e].src      = src;
	kmod_graph_edges[e].dst      = dst;
	kmod_graph_edges[e].weight   = weight;
	kmod_graph_edges[e].next_out = kmod_graph_nodes[src].first_edge;
	kmod_graph_nodes[src].first_edge = e;
out:
	spin_unlock(&kmod_graph_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}

/* ------------------------------------------------------------------ */
/* Stub functions for BPF fentry variants                              */
/* BPF programs attach to these; the body just counts the op.         */
/* ------------------------------------------------------------------ */

/* ---- undo_log stubs ---- */
noinline void bench_undo_list_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_undo_list_init);

noinline void bench_undo_list_insert(u32 new_idx, u32 head_lock_idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_list_insert);

noinline void bench_undo_ring_init(u32 num_slots) { }
EXPORT_SYMBOL_GPL(bench_undo_ring_init);

noinline void bench_undo_ring_enqueue(u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_ring_enqueue);

noinline void bench_undo_trie_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_undo_trie_init);

noinline void bench_undo_trie_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_trie_insert);

noinline void bench_undo_rbtree_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_undo_rbtree_init);

noinline void bench_undo_rbtree_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_rbtree_insert);

noinline void bench_undo_graph_init(u32 num_nodes, u32 num_edges) { }
EXPORT_SYMBOL_GPL(bench_undo_graph_init);

noinline void bench_undo_graph_add_edge(u32 src, u32 dst, u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_graph_add_edge);

/* ---- arena stubs ---- */
noinline void bench_arena_list_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_arena_list_init);

noinline void bench_arena_list_insert(u32 new_idx, u32 head_lock_idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_list_insert);

noinline void bench_arena_ring_init(u32 num_slots) { }
EXPORT_SYMBOL_GPL(bench_arena_ring_init);

noinline void bench_arena_ring_enqueue(u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_ring_enqueue);

noinline void bench_arena_trie_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_arena_trie_init);

noinline void bench_arena_trie_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_trie_insert);

noinline void bench_arena_rbtree_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_arena_rbtree_init);

noinline void bench_arena_rbtree_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_rbtree_insert);

noinline void bench_arena_graph_init(u32 num_nodes, u32 num_edges) { }
EXPORT_SYMBOL_GPL(bench_arena_graph_init);

noinline void bench_arena_graph_add_edge(u32 src, u32 dst, u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_graph_add_edge);

/* ------------------------------------------------------------------ */
/* call_init / call_op dispatchers                                     */
/* ------------------------------------------------------------------ */

static void call_init(int variant, int ds_type, u32 pool_size)
{
	switch (variant) {
	case BENCH_VARIANT_UNDO_LOG:
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
	}
}

static void call_op(int variant, int ds_type, u64 iter, u32 pool_size)
{
	u32 idx  = (u32)(iter % (pool_size > 1 ? pool_size - 1 : 1)) + 1;
	u32 src  = (u32)(iter % pool_size);
	u32 dst  = (u32)((iter + 1) % pool_size);
	/* Multiplicative hash for key variety without modulo cost */
	u64 key  = iter * 2654435761ULL;
	u64 val  = ktime_get_mono_fast_ns();

	switch (variant) {
	case BENCH_VARIANT_UNDO_LOG:
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_undo_list_insert(idx, 0);        break;
		case BENCH_DS_RING:   bench_undo_ring_enqueue(val);          break;
		case BENCH_DS_TRIE:   bench_undo_trie_insert(key, val);      break;
		case BENCH_DS_RBTREE: bench_undo_rbtree_insert(key, val);    break;
		case BENCH_DS_GRAPH:  bench_undo_graph_add_edge(src, dst, val); break;
		}
		break;
	case BENCH_VARIANT_ARENA:
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_arena_list_insert(idx, 0);       break;
		case BENCH_DS_RING:   bench_arena_ring_enqueue(val);         break;
		case BENCH_DS_TRIE:   bench_arena_trie_insert(key, val);     break;
		case BENCH_DS_RBTREE: bench_arena_rbtree_insert(key, val);   break;
		case BENCH_DS_GRAPH:  bench_arena_graph_add_edge(src, dst, val); break;
		}
		break;
	case BENCH_VARIANT_KMOD:
		switch (ds_type) {
		case BENCH_DS_LIST:   bench_kmod_list_insert(idx, 0);        break;
		case BENCH_DS_RING:   bench_kmod_ring_enqueue(val);          break;
		case BENCH_DS_TRIE:   bench_kmod_trie_insert(key, val);      break;
		case BENCH_DS_RBTREE: bench_kmod_rbtree_insert(key, val);    break;
		case BENCH_DS_GRAPH:  bench_kmod_graph_add_edge(src, dst, val); break;
		}
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Benchmark kthread                                                   */
/* ------------------------------------------------------------------ */

struct bench_worker_ctx {
	int cpu;
	int variant;
	int ds_type;
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
		call_op(ctx->variant, ctx->ds_type, iter, pool_size);
		iter++;
	}

	/* Reset per-CPU counter that stubs may have bumped during warmup */
	s->ops = 0;
	ops = 0;

	/* Measurement window */
	t0 = ktime_get();
	while (ktime_ms_delta(ktime_get(), t0) < ctx->bench_ms) {
		u64 t_start = ktime_get_mono_fast_ns();

		call_op(ctx->variant, ctx->ds_type, iter, pool_size);

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

	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < n; i++) {
		init_completion(&ctxs[i].done);
		ctxs[i].cpu       = cpu;
		ctxs[i].variant   = variant;
		ctxs[i].ds_type   = bench_cfg.ds_type;
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

	return (kmod_list_pool && kmod_ring_pool && kmod_trie_pool &&
		kmod_rb_pool && kmod_graph_nodes && kmod_graph_edges) ? 0 : -ENOMEM;
}

static void free_kmod_pools(void)
{
	kfree(kmod_list_pool);    kmod_list_pool   = NULL;
	kfree(kmod_ring_pool);    kmod_ring_pool   = NULL;
	kfree(kmod_trie_pool);    kmod_trie_pool   = NULL;
	kfree(kmod_rb_pool);      kmod_rb_pool     = NULL;
	kfree(kmod_graph_nodes);  kmod_graph_nodes = NULL;
	kfree(kmod_graph_edges);  kmod_graph_edges = NULL;
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
		if (p.ds_type > BENCH_DS_GRAPH || p.variant > BENCH_VARIANT_KMOD)
			return -EINVAL;
		if (!p.num_threads || p.num_threads > BENCH_MAX_CPUS)
			return -EINVAL;
		if (!p.pool_size || p.pool_size > KMOD_MAX_POOL)
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
