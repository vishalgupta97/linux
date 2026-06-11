// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: linked list, lock_func variant.
 *
 * Same data structure and attach points as the undo_log variant, but each
 * critical section is wrapped in a callback passed to bpf_lock_func() instead
 * of using inline bpf_spin_lock()/bpf_spin_unlock().  bpf_lock_func() acquires
 * the lock, runs the callback with the lock held, then releases it; writes to
 * global state inside the callback are still undo-logged.
 *
 * Operation: insert-at-head, per-element locking.
 * Writes per CS: 3  (new_node.next_idx, new_node.data, list_head_idx)
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_arena_common.h"
#include "bench_spinlock_shared.h"

/* ---- Arena: node pool ---- */
struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 4); /* pages — 16 KB is plenty for 256 nodes */
	__ulong(map_extra, 0x1ull << 44);
} arena SEC(".maps");

struct bench_list_node __arena list_pool[BENCH_MAX_POOL];
__u32 __arena list_head_idx;

/* ---- Parallel lock map: one bpf_spin_lock per pool slot ---- */
struct list_lock_entry {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, BENCH_MAX_POOL);
	__type(key, __u32);
	__type(value, struct list_lock_entry);
} list_locks SEC(".maps");

/* ---- Init: zero the pool and reset head (no lock needed — no concurrency yet) ---- */
SEC("fentry/bench_undo_list_init")
int BPF_PROG(list_init, __u32 pool_size)
{
	__u32 i;

	bpf_for(i, 0, BENCH_MAX_POOL) {
		list_pool[i].next_idx = (__u32)~0;
		list_pool[i].data = 0;
	}
	list_head_idx = (__u32)~0;
	return 0;
}

/* ---- Insert-at-head with per-element bpf_spin_lock ---- */
static int list_insert_cb(__u64 new_idx_v)
{
	__u32 new_idx = (__u32)new_idx_v;
	struct list_lock_entry *nl;

	nl = bpf_map_lookup_elem(&list_locks, &new_idx);
	if (!nl)
		return 0;

	/* Lock the new node (a different lock) inside the callback. */
	bpf_spin_lock(&nl->lock);

	/* 3 arena writes → 3 undo-log entries injected by JIT */
	list_pool[new_idx].next_idx = list_head_idx;
	list_pool[new_idx].data     = bpf_ktime_get_ns();
	list_head_idx               = new_idx;

	bpf_spin_unlock(&nl->lock);
	return 0;
}

SEC("fentry/bench_undo_list_insert")
int BPF_PROG(list_insert, __u32 new_idx, __u32 head_lock_idx)
{
	struct list_lock_entry *hl;

	if (new_idx >= BENCH_MAX_POOL)
		return 0;

	hl = bpf_map_lookup_elem(&list_locks, &head_lock_idx);
	if (!hl)
		return 0;

	bpf_lock_func(&hl->lock, list_insert_cb, new_idx, 0, 0);
	return 0;
}

/* Traverse from head, read data at position (idx % pool_size). 0 undo-log writes. */
static int list_lookup_cb(__u64 idx_v)
{
	__u32 idx = (__u32)idx_v;
	__u32 cur = list_head_idx;
	int i;

	bpf_for(i, 0, BENCH_MAX_POOL) {
		if (cur == (__u32)~0)
			break;
		if ((__u32)i == idx % BENCH_MAX_POOL)
			break;
		cur = list_pool[cur].next_idx;
	}
	return 0;
}

SEC("fentry/bench_undo_list_lookup")
int BPF_PROG(list_lookup, __u32 idx)
{
	struct list_lock_entry *hl;
	__u32 key0 = 0;

	hl = bpf_map_lookup_elem(&list_locks, &key0);
	if (!hl)
		return 0;

	bpf_lock_func(&hl->lock, list_lookup_cb, idx, 0, 0);
	return 0;
}

/* Traverse to position (idx % pool_size) and write data. 1 undo-log write. */
static int list_update_cb(__u64 idx_v, __u64 val)
{
	__u32 idx = (__u32)idx_v;
	__u32 cur = list_head_idx;
	int i;

	bpf_for(i, 0, BENCH_MAX_POOL) {
		if (cur == (__u32)~0)
			break;
		if ((__u32)i == idx % BENCH_MAX_POOL) {
			list_pool[cur].data = val;	/* 1 undo-log entry */
			break;
		}
		cur = list_pool[cur].next_idx;
	}
	return 0;
}

SEC("fentry/bench_undo_list_update")
int BPF_PROG(list_update, __u32 idx, __u64 val)
{
	struct list_lock_entry *hl;
	__u32 key0 = 0;

	hl = bpf_map_lookup_elem(&list_locks, &key0);
	if (!hl)
		return 0;

	bpf_lock_func(&hl->lock, list_update_cb, idx, val, 0);
	return 0;
}

/* Remove the head node. 1 undo-log write. No-op when list is empty. */
static int list_delete_cb(void)
{
	__u32 head = list_head_idx;

	if (head != (__u32)~0)
		list_head_idx = list_pool[head].next_idx;	/* 1 undo-log entry */
	return 0;
}

SEC("fentry/bench_undo_list_delete")
int BPF_PROG(list_delete_op)
{
	struct list_lock_entry *hl;
	__u32 key0 = 0;

	hl = bpf_map_lookup_elem(&list_locks, &key0);
	if (!hl)
		return 0;

	bpf_lock_func(&hl->lock, list_delete_cb, 0, 0, 0);
	return 0;
}

char _license[] SEC("license") = "GPL";
