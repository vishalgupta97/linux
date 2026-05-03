// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: linked list, undo_log variant.
 *
 * Attaches via fentry to bench_undo_list_init / bench_undo_list_insert
 * in bench_spinlock_kmod.  Uses bpf_spin_lock (from a parallel BPF map)
 * to protect per-element arena writes so that the JIT injects undo-log
 * entries for each write inside the critical section.
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
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32);
#else
	__ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
struct bench_list_node __arena list_pool[BENCH_MAX_POOL];
__u32 __arena list_head_idx;
#endif

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
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	__u32 i;

	bpf_for(i, 0, BENCH_MAX_POOL) {
		list_pool[i].next_idx = (__u32)~0;
		list_pool[i].data = 0;
	}
	list_head_idx = (__u32)~0;
#endif
	return 0;
}

/* ---- Insert-at-head with per-element bpf_spin_lock ---- */
SEC("fentry/bench_undo_list_insert")
int BPF_PROG(list_insert, __u32 new_idx, __u32 head_lock_idx)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	struct list_lock_entry *hl, *nl;

	if (new_idx >= BENCH_MAX_POOL)
		return 0;

	hl = bpf_map_lookup_elem(&list_locks, &head_lock_idx);
	nl = bpf_map_lookup_elem(&list_locks, &new_idx);
	if (!hl || !nl)
		return 0;

	bpf_spin_lock(&hl->lock);
	bpf_spin_lock(&nl->lock);

	/* 3 arena writes → 3 undo-log entries injected by JIT */
	list_pool[new_idx].next_idx = list_head_idx;
	list_pool[new_idx].data     = bpf_ktime_get_ns();
	list_head_idx               = new_idx;

	bpf_spin_unlock(&nl->lock);
	bpf_spin_unlock(&hl->lock);
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
