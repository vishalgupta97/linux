// SPDX-License-Identifier: GPL-2.0
/*
 * BPF spinlock undo-log overhead benchmark — userspace harness.
 *
 * Opens /dev/bench_spinlock (provided by bench_spinlock_kmod), optionally
 * loads and attaches BPF fentry programs, then triggers the in-kernel
 * benchmark loop via ioctl.  Results are read back and printed as CSV.
 *
 * Usage:
 *   bench_spinlock [OPTIONS]
 *
 * Options:
 *   --ds <list|ring|trie|rbtree|graph|all>          (default: all)
 *   --variant <undo_log|arena|kmod|kmod_bpf|all>    (default: all)
 *   --op <insert|lookup|update|delete|all>          (default: all)
 *   --threads <N>                                   (default: nproc)
 *   --pool <N>                                      (default: 256)
 *   --init-size <N>  elements pre-populated before  (default: 0 = no prefill)
 *   --warmup-ms <N>                                 (default: 5000)
 *   --bench-ms <N>                                  (default: 15000)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/sysinfo.h>
#include <getopt.h>

#include <bpf/libbpf.h>

#include "../progs/bench_spinlock_shared.h"

/*
 * Minimal userspace stubs for arena skeleton types.
 * Must match the in-kernel sizes (qnodes: 16 B each, __qspinlock: 4 B).
 */
struct __qspinlock { int val; };
struct arena_qnode { unsigned long next; int locked; int count; };

/* Arena node types that arena skeletons reference */
struct bench_arena_list_node { struct __qspinlock lock; __u32 next_idx; __u64 data; };
struct bench_arena_ring_slot { struct __qspinlock lock; __u64 data; __u32 valid; };

/* Generated BPF skeleton headers */
#include "bench_undo_log_list.skel.h"
#include "bench_undo_log_ring.skel.h"
#include "bench_undo_log_trie.skel.h"
#include "bench_undo_log_rbtree.skel.h"
#include "bench_undo_log_graph.skel.h"
#include "bench_arena_list.skel.h"
#include "bench_arena_ring.skel.h"
#include "bench_arena_trie.skel.h"
#include "bench_arena_rbtree.skel.h"
#include "bench_arena_graph.skel.h"
#include "bench_kmod_bpf_list.skel.h"
#include "bench_kmod_bpf_ring.skel.h"
#include "bench_kmod_bpf_trie.skel.h"
#include "bench_kmod_bpf_rbtree.skel.h"
#include "bench_kmod_bpf_graph.skel.h"

#define BENCH_DEV "/dev/bench_spinlock"
#define SYSCTL_TIMEOUT "/proc/sys/net/core/bpf_spin_lock_timeout"

/* ------------------------------------------------------------------ */
/* CLI configuration                                                   */
/* ------------------------------------------------------------------ */

static struct {
	int ds_mask;       /* bitmask of BENCH_DS_* values */
	int variant_mask;  /* bitmask of BENCH_VARIANT_* values */
	int op_mask;       /* bitmask of BENCH_OP_* values */
	__u32 num_threads;
	__u32 pool_size;
	__u32 init_size;   /* 0 = no prefill */
	__u32 warmup_ms;
	__u32 bench_ms;
} cfg = {
	.ds_mask      = (1 << 5) - 1,   /* all DS */
	.variant_mask = (1 << 4) - 1,   /* all variants */
	.op_mask      = (1 << 4) - 1,   /* all ops */
	.pool_size    = 256,
	.init_size    = 0,
	.warmup_ms    = 5000,
	.bench_ms     = 15000,
};

/* ------------------------------------------------------------------ */
/* sysctl helpers                                                      */
/* ------------------------------------------------------------------ */

static int read_sysctl(const char *path)
{
	int fd = open(path, O_RDONLY);
	char buf[32] = {};

	if (fd < 0) return 0;
	if (read(fd, buf, sizeof(buf) - 1) <= 0) {
		close(fd);
		return 0;
	}
	close(fd);
	return atoi(buf);
}

static void write_sysctl(const char *path, int val)
{
	int fd = open(path, O_WRONLY);
	char buf[32];

	if (fd < 0) return;
	snprintf(buf, sizeof(buf), "%d\n", val);
	if (write(fd, buf, strlen(buf)) < 0)
		fprintf(stderr, "write to %s failed\n", path);
	close(fd);
}

/* ------------------------------------------------------------------ */
/* BPF load + attach helpers                                           */
/* ------------------------------------------------------------------ */

/* Generic: load skeleton, attach, signal ioctl, detach, return 0/-errno */
#define RUN_UNDO_LOG_BENCH(name, DS_CONST, fd, cfg_ptr)			\
({									\
	struct bench_undo_log_##name *_skel;				\
	int _ret = 0;							\
									\
	_skel = bench_undo_log_##name##__open_and_load();		\
	if (!_skel) {							\
		fprintf(stderr, "failed to load bench_undo_log_" #name "\n"); \
		_ret = -1;						\
	} else {							\
		_ret = bench_undo_log_##name##__attach(_skel);		\
		if (_ret) {						\
			fprintf(stderr, "failed to attach " #name ": %d\n", _ret); \
		} else {						\
			struct bench_params _p = *(cfg_ptr);		\
			_p.variant = BENCH_VARIANT_UNDO_LOG;		\
			_p.ds_type = (DS_CONST);			\
			ioctl((fd), BENCH_IOCTL_SET_PARAMS, &_p);	\
			_ret = ioctl((fd), BENCH_IOCTL_EBPF_READY, 0); \
			if (_ret)					\
				fprintf(stderr, "ioctl EBPF_READY failed: %s\n", \
					strerror(errno));		\
		}							\
		bench_undo_log_##name##__destroy(_skel);		\
	}								\
	_ret;								\
})

#define RUN_ARENA_BENCH(name, DS_CONST, fd, cfg_ptr, skip_p)		\
({									\
	struct bench_arena_##name *_skel;				\
	int _ret = 0;							\
									\
	_skel = bench_arena_##name##__open_and_load();			\
	if (!_skel) {							\
		fprintf(stderr, "failed to load bench_arena_" #name "\n"); \
		_ret = -1;						\
	} else {							\
		if (_skel->data->test_skip != 1) {			\
			*(skip_p) = _skel->data->test_skip;		\
			_ret = -2;	/* skip */			\
		} else {						\
			_ret = bench_arena_##name##__attach(_skel);	\
			if (_ret) {					\
				fprintf(stderr, "failed to attach " #name ": %d\n", _ret); \
			} else {					\
				struct bench_params _p = *(cfg_ptr);	\
				_p.variant = BENCH_VARIANT_ARENA;	\
				_p.ds_type = (DS_CONST);		\
				ioctl((fd), BENCH_IOCTL_SET_PARAMS, &_p); \
				_ret = ioctl((fd), BENCH_IOCTL_EBPF_READY, 0); \
				if (_ret)				\
					fprintf(stderr, "ioctl EBPF_READY failed: %s\n", \
						strerror(errno));	\
			}						\
		}							\
		bench_arena_##name##__destroy(_skel);			\
	}								\
	_ret;								\
})

#define RUN_KMOD_BPF_BENCH(name, DS_CONST, fd, cfg_ptr)			\
({									\
	struct bench_kmod_bpf_##name *_skel;				\
	int _ret = 0;							\
									\
	_skel = bench_kmod_bpf_##name##__open_and_load();		\
	if (!_skel) {							\
		fprintf(stderr, "failed to load bench_kmod_bpf_" #name "\n"); \
		_ret = -1;						\
	} else {							\
		_ret = bench_kmod_bpf_##name##__attach(_skel);		\
		if (_ret) {						\
			fprintf(stderr, "failed to attach " #name ": %d\n", _ret); \
		} else {						\
			struct bench_params _p = *(cfg_ptr);		\
			_p.variant = BENCH_VARIANT_KMOD_BPF;		\
			_p.ds_type = (DS_CONST);			\
			ioctl((fd), BENCH_IOCTL_SET_PARAMS, &_p);	\
			_ret = ioctl((fd), BENCH_IOCTL_EBPF_READY, 0); \
			if (_ret)					\
				fprintf(stderr, "ioctl EBPF_READY failed: %s\n", \
					strerror(errno));		\
		}							\
		bench_kmod_bpf_##name##__destroy(_skel);		\
	}								\
	_ret;								\
})

/* ------------------------------------------------------------------ */
/* Result printing                                                     */
/* ------------------------------------------------------------------ */

static void print_csv_row(int variant, int ds, int op,
			  __u32 threads, __u32 pool, __u32 init_size,
			  __u32 bench_ms,
			  const struct bench_all_results *r)
{
	double ops_per_sec = (double)r->total_ops / ((double)bench_ms / 1000.0);
	double avg_lat_ns  = r->total_ops
		? (double)r->total_lat_sum_ns / (double)r->total_ops
		: 0.0;

	printf("%s,%s,%s,%u,%u,%u,%llu,%.0f,%.1f,%llu,%llu,%llu\n",
	       bench_variant_names[variant],
	       bench_ds_names[ds],
	       bench_op_names[op],
	       threads,
	       pool,
	       init_size,
	       (unsigned long long)r->total_ops,
	       ops_per_sec,
	       avg_lat_ns,
	       (unsigned long long)r->p50_lat_ns,
	       (unsigned long long)r->p95_lat_ns,
	       (unsigned long long)r->p99_lat_ns);
}

static int collect_and_print(int fd, int variant, int ds, int op,
			     __u32 threads, __u32 pool, __u32 init_size,
			     __u32 bench_ms)
{
	struct bench_all_results res = {};

	if (ioctl(fd, BENCH_IOCTL_GET_RESULTS, &res)) {
		fprintf(stderr, "GET_RESULTS failed: %s\n", strerror(errno));
		return -1;
	}
	print_csv_row(variant, ds, op, threads, pool, init_size, bench_ms, &res);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Run one (variant, ds) combination                                   */
/* ------------------------------------------------------------------ */

static void run_one(int fd, int variant, int ds, int op,
		    struct bench_params *base_p)
{
	struct bench_params p = *base_p;
	int skip = 0;
	int ret  = 0;

	p.variant  = (__u32)variant;
	p.ds_type  = (__u32)ds;
	p.op_type  = (__u32)op;
	p.init_size = base_p->init_size;

	switch (variant) {
	case BENCH_VARIANT_UNDO_LOG:
		switch (ds) {
		case BENCH_DS_LIST:
			ret = RUN_UNDO_LOG_BENCH(list,   BENCH_DS_LIST,   fd, &p); break;
		case BENCH_DS_RING:
			ret = RUN_UNDO_LOG_BENCH(ring,   BENCH_DS_RING,   fd, &p); break;
		case BENCH_DS_TRIE:
			ret = RUN_UNDO_LOG_BENCH(trie,   BENCH_DS_TRIE,   fd, &p); break;
		case BENCH_DS_RBTREE:
			ret = RUN_UNDO_LOG_BENCH(rbtree, BENCH_DS_RBTREE, fd, &p); break;
		case BENCH_DS_GRAPH:
			ret = RUN_UNDO_LOG_BENCH(graph,  BENCH_DS_GRAPH,  fd, &p); break;
		}
		break;

	case BENCH_VARIANT_ARENA:
		switch (ds) {
		case BENCH_DS_LIST:
			ret = RUN_ARENA_BENCH(list,   BENCH_DS_LIST,   fd, &p, &skip); break;
		case BENCH_DS_RING:
			ret = RUN_ARENA_BENCH(ring,   BENCH_DS_RING,   fd, &p, &skip); break;
		case BENCH_DS_TRIE:
			ret = RUN_ARENA_BENCH(trie,   BENCH_DS_TRIE,   fd, &p, &skip); break;
		case BENCH_DS_RBTREE:
			ret = RUN_ARENA_BENCH(rbtree, BENCH_DS_RBTREE, fd, &p, &skip); break;
		case BENCH_DS_GRAPH:
			ret = RUN_ARENA_BENCH(graph,  BENCH_DS_GRAPH,  fd, &p, &skip); break;
		}
		if (ret == -2) {
			fprintf(stderr, "SKIP: arena/%s — test_skip=%d (%s)\n",
				bench_ds_names[ds], skip,
				skip == 2 ? "no addr_space_cast" : "NR_CPUS too large");
			return;
		}
		break;

	case BENCH_VARIANT_KMOD:
		if (ioctl(fd, BENCH_IOCTL_SET_PARAMS, &p)) {
			fprintf(stderr, "SET_PARAMS failed: %s\n", strerror(errno));
			return;
		}
		ret = ioctl(fd, BENCH_IOCTL_RUN_KMOD, 0);
		if (ret) {
			fprintf(stderr, "RUN_KMOD failed: %s\n", strerror(errno));
			return;
		}
		break;

	case BENCH_VARIANT_KMOD_BPF:
		switch (ds) {
		case BENCH_DS_LIST:
			ret = RUN_KMOD_BPF_BENCH(list,   BENCH_DS_LIST,   fd, &p); break;
		case BENCH_DS_RING:
			ret = RUN_KMOD_BPF_BENCH(ring,   BENCH_DS_RING,   fd, &p); break;
		case BENCH_DS_TRIE:
			ret = RUN_KMOD_BPF_BENCH(trie,   BENCH_DS_TRIE,   fd, &p); break;
		case BENCH_DS_RBTREE:
			ret = RUN_KMOD_BPF_BENCH(rbtree, BENCH_DS_RBTREE, fd, &p); break;
		case BENCH_DS_GRAPH:
			ret = RUN_KMOD_BPF_BENCH(graph,  BENCH_DS_GRAPH,  fd, &p); break;
		}
		break;
	}

	if (!ret)
		collect_and_print(fd, variant, ds, op,
				  base_p->num_threads, base_p->pool_size,
				  base_p->init_size, base_p->bench_ms);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

static struct option long_opts[] = {
	{ "ds",         required_argument, NULL, 'd' },
	{ "variant",    required_argument, NULL, 'v' },
	{ "op",         required_argument, NULL, 'o' },
	{ "threads",    required_argument, NULL, 't' },
	{ "pool",       required_argument, NULL, 'p' },
	{ "init-size",  required_argument, NULL, 'i' },
	{ "warmup-ms",  required_argument, NULL, 'w' },
	{ "bench-ms",   required_argument, NULL, 'b' },
	{ "help",       no_argument,       NULL, 'h' },
	{ NULL, 0, NULL, 0 },
};

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [OPTIONS]\n"
		"  --ds <list|ring|trie|rbtree|graph|all>          (default: all)\n"
		"  --variant <undo_log|arena|kmod|kmod_bpf|all>    (default: all)\n"
		"  --op <insert|lookup|update|delete|all>          (default: all)\n"
		"  --threads <N>       (default: nproc)\n"
		"  --pool <N>          (default: 256)\n"
		"  --init-size <N>     elements pre-inserted       (default: 0)\n"
		"  --warmup-ms <N>     (default: 5000)\n"
		"  --bench-ms <N>      (default: 15000)\n",
		prog);
}

int main(int argc, char **argv)
{
	int fd, opt, old_timeout;
	struct bench_params base_p = {};

	cfg.num_threads = (__u32)get_nprocs();

	while ((opt = getopt_long(argc, argv, "", long_opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			if (!strcmp(optarg, "all")) {
				cfg.ds_mask = (1 << 5) - 1;
			} else if (!strcmp(optarg, "list")) {
				cfg.ds_mask = 1 << BENCH_DS_LIST;
			} else if (!strcmp(optarg, "ring")) {
				cfg.ds_mask = 1 << BENCH_DS_RING;
			} else if (!strcmp(optarg, "trie")) {
				cfg.ds_mask = 1 << BENCH_DS_TRIE;
			} else if (!strcmp(optarg, "rbtree")) {
				cfg.ds_mask = 1 << BENCH_DS_RBTREE;
			} else if (!strcmp(optarg, "graph")) {
				cfg.ds_mask = 1 << BENCH_DS_GRAPH;
			} else {
				fprintf(stderr, "unknown ds: %s\n", optarg);
				return 1;
			}
			break;
		case 'v':
			if (!strcmp(optarg, "all")) {
				cfg.variant_mask = (1 << 4) - 1;
			} else if (!strcmp(optarg, "undo_log")) {
				cfg.variant_mask = 1 << BENCH_VARIANT_UNDO_LOG;
			} else if (!strcmp(optarg, "arena")) {
				cfg.variant_mask = 1 << BENCH_VARIANT_ARENA;
			} else if (!strcmp(optarg, "kmod")) {
				cfg.variant_mask = 1 << BENCH_VARIANT_KMOD;
			} else if (!strcmp(optarg, "kmod_bpf")) {
				cfg.variant_mask = 1 << BENCH_VARIANT_KMOD_BPF;
			} else {
				fprintf(stderr, "unknown variant: %s\n", optarg);
				return 1;
			}
			break;
		case 'o':
			if (!strcmp(optarg, "all")) {
				cfg.op_mask = (1 << 4) - 1;
			} else if (!strcmp(optarg, "insert")) {
				cfg.op_mask = 1 << BENCH_OP_INSERT;
			} else if (!strcmp(optarg, "lookup")) {
				cfg.op_mask = 1 << BENCH_OP_LOOKUP;
			} else if (!strcmp(optarg, "update")) {
				cfg.op_mask = 1 << BENCH_OP_UPDATE;
			} else if (!strcmp(optarg, "delete")) {
				cfg.op_mask = 1 << BENCH_OP_DELETE;
			} else {
				fprintf(stderr, "unknown op: %s\n", optarg);
				return 1;
			}
			break;
		case 't': cfg.num_threads = (__u32)atoi(optarg); break;
		case 'p': cfg.pool_size   = (__u32)atoi(optarg); break;
		case 'i': cfg.init_size   = (__u32)atoi(optarg); break;
		case 'w': cfg.warmup_ms   = (__u32)atoi(optarg); break;
		case 'b': cfg.bench_ms    = (__u32)atoi(optarg); break;
		case 'h': usage(argv[0]); return 0;
		default:  usage(argv[0]); return 1;
		}
	}

	fd = open(BENCH_DEV, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Cannot open %s: %s\n"
			"Is bench_spinlock_kmod loaded?\n",
			BENCH_DEV, strerror(errno));
		return 1;
	}

	/* Disable BPF spinlock timeout for the duration of the benchmark */
	old_timeout = read_sysctl(SYSCTL_TIMEOUT);
	write_sysctl(SYSCTL_TIMEOUT, 0);

	/* Build the base params struct */
	base_p.num_threads = cfg.num_threads;
	base_p.pool_size   = cfg.pool_size;
	base_p.init_size   = cfg.init_size;
	base_p.warmup_ms   = cfg.warmup_ms;
	base_p.bench_ms    = cfg.bench_ms;

	/* Print CSV header */
	fputs(BENCH_CSV_HEADER, stdout);

	/* Run all requested (variant, ds, op) combinations */
	for (int v = 0; v < 4; v++) {
		if (!(cfg.variant_mask & (1 << v)))
			continue;
		for (int d = 0; d < 5; d++) {
			if (!(cfg.ds_mask & (1 << d)))
				continue;
			for (int o = 0; o < 4; o++) {
				if (!(cfg.op_mask & (1 << o)))
					continue;
				fprintf(stderr, "Running %s/%s/%s ...\n",
					bench_variant_names[v],
					bench_ds_names[d],
					bench_op_names[o]);
				run_one(fd, v, d, o, &base_p);
				fflush(stdout);
			}
		}
	}

	write_sysctl(SYSCTL_TIMEOUT, old_timeout);
	close(fd);
	return 0;
}
