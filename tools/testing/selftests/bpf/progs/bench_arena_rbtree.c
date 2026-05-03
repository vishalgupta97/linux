// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: red-black tree insert + rebalance, arena variant.
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

struct bench_rb_node __arena arena_rb_pool[BENCH_MAX_POOL];
__u32 __arena arena_rb_root;
volatile long arena_rb_alloc_idx;
arena_spinlock_t __arena arena_rb_lock;

int test_skip = 1;

static __always_inline void arena_rb_rotate_left(__u32 x)
{
	__u32 y = arena_rb_pool[x].right;

	arena_rb_pool[x].right = arena_rb_pool[y].left;
	if (arena_rb_pool[y].left)
		arena_rb_pool[arena_rb_pool[y].left].parent = x;
	arena_rb_pool[y].parent = arena_rb_pool[x].parent;
	if (!arena_rb_pool[x].parent)
		arena_rb_root = y;
	else if (x == arena_rb_pool[arena_rb_pool[x].parent].left)
		arena_rb_pool[arena_rb_pool[x].parent].left = y;
	else
		arena_rb_pool[arena_rb_pool[x].parent].right = y;
	arena_rb_pool[y].left = x;
	arena_rb_pool[x].parent = y;
}

static __always_inline void arena_rb_rotate_right(__u32 x)
{
	__u32 y = arena_rb_pool[x].left;

	arena_rb_pool[x].left = arena_rb_pool[y].right;
	if (arena_rb_pool[y].right)
		arena_rb_pool[arena_rb_pool[y].right].parent = x;
	arena_rb_pool[y].parent = arena_rb_pool[x].parent;
	if (!arena_rb_pool[x].parent)
		arena_rb_root = y;
	else if (x == arena_rb_pool[arena_rb_pool[x].parent].right)
		arena_rb_pool[arena_rb_pool[x].parent].right = y;
	else
		arena_rb_pool[arena_rb_pool[x].parent].left = y;
	arena_rb_pool[y].right = x;
	arena_rb_pool[x].parent = y;
}

static __always_inline void arena_rb_insert_fixup(__u32 z)
{
	__u32 y;
	int depth;

	bpf_for(depth, 0, 64) {
		__u32 p, g;

		if (!arena_rb_pool[arena_rb_pool[z].parent].color)
			break;
		p = arena_rb_pool[z].parent;
		g = arena_rb_pool[p].parent;
		if (!p || !g)
			break;

		if (p == arena_rb_pool[g].left) {
			y = arena_rb_pool[g].right;
			if (arena_rb_pool[y].color == BENCH_RB_RED) {
				arena_rb_pool[p].color = BENCH_RB_BLACK;
				arena_rb_pool[y].color = BENCH_RB_BLACK;
				arena_rb_pool[g].color = BENCH_RB_RED;
				z = g;
			} else {
				if (z == arena_rb_pool[p].right) {
					z = p;
					arena_rb_rotate_left(z);
					p = arena_rb_pool[z].parent;
					g = arena_rb_pool[p].parent;
				}
				arena_rb_pool[p].color = BENCH_RB_BLACK;
				arena_rb_pool[g].color = BENCH_RB_RED;
				arena_rb_rotate_right(g);
			}
		} else {
			y = arena_rb_pool[g].left;
			if (arena_rb_pool[y].color == BENCH_RB_RED) {
				arena_rb_pool[p].color = BENCH_RB_BLACK;
				arena_rb_pool[y].color = BENCH_RB_BLACK;
				arena_rb_pool[g].color = BENCH_RB_RED;
				z = g;
			} else {
				if (z == arena_rb_pool[p].left) {
					z = p;
					arena_rb_rotate_right(z);
					p = arena_rb_pool[z].parent;
					g = arena_rb_pool[p].parent;
				}
				arena_rb_pool[p].color = BENCH_RB_BLACK;
				arena_rb_pool[g].color = BENCH_RB_RED;
				arena_rb_rotate_left(g);
			}
		}
	}
	arena_rb_pool[arena_rb_root].color = BENCH_RB_BLACK;
}

SEC("fentry/bench_arena_rbtree_init")
int BPF_PROG(rbtree_init, __u32 pool_size)
{
	__u32 i;

	arena_rb_pool[0].left = arena_rb_pool[0].right = arena_rb_pool[0].parent = 0;
	arena_rb_pool[0].color = BENCH_RB_BLACK;
	arena_rb_pool[0].key = arena_rb_pool[0].val = 0;
	arena_rb_root = 0;
	arena_rb_alloc_idx = 1;
	bpf_for(i, 1, BENCH_MAX_POOL) {
		arena_rb_pool[i].left = arena_rb_pool[i].right = arena_rb_pool[i].parent = 0;
		arena_rb_pool[i].color = BENCH_RB_BLACK;
		arena_rb_pool[i].key = arena_rb_pool[i].val = 0;
	}
	return 0;
}

SEC("fentry/bench_arena_rbtree_insert")
int BPF_PROG(rbtree_insert, __u64 key, __u64 val)
{
	unsigned long flags;
	int ret, _d;
	__u32 z, p, x;

	ret = arena_spin_lock_irqsave(&arena_rb_lock, flags);
	if (ret) {
		if (ret == -EOPNOTSUPP)
			test_skip = 3;
		return ret;
	}

	z = (__u32)__sync_fetch_and_add(&arena_rb_alloc_idx, 1);
	if (z >= BENCH_MAX_POOL)
		goto out;

	arena_rb_pool[z].key    = key;
	arena_rb_pool[z].val    = val;
	arena_rb_pool[z].color  = BENCH_RB_RED;
	arena_rb_pool[z].left   = 0;
	arena_rb_pool[z].right  = 0;
	arena_rb_pool[z].parent = 0;

	p = 0;
	x = arena_rb_root;
	bpf_for(_d, 0, BENCH_MAX_POOL) {
		if (!x)
			break;
		p = x;
		if (key < arena_rb_pool[x].key)
			x = arena_rb_pool[x].left;
		else
			x = arena_rb_pool[x].right;
	}

	arena_rb_pool[z].parent = p;
	if (!p)
		arena_rb_root = z;
	else if (key < arena_rb_pool[p].key)
		arena_rb_pool[p].left = z;
	else
		arena_rb_pool[p].right = z;

	arena_rb_insert_fixup(z);
out:
	arena_spin_unlock_irqrestore(&arena_rb_lock, flags);
	return 0;
}

#else
int test_skip = 2;

SEC("fentry/bench_arena_rbtree_init")
int BPF_PROG(rbtree_init, __u32 pool_size) { return 0; }

SEC("fentry/bench_arena_rbtree_insert")
int BPF_PROG(rbtree_insert, __u64 key, __u64 val) { return -2; }
#endif

char _license[] SEC("license") = "GPL";
