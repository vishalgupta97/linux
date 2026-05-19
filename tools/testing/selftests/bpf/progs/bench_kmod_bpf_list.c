// SPDX-License-Identifier: GPL-2.0
/*
 * BPF benchmark: linked list, kmod_bpf variant.
 *
 * Attaches via fentry to bench_kmod_bpf_list_* stubs in the kernel module.
 * The kernel module allocates the node pool (vmalloc) and passes a direct
 * kernel pointer (PTR_TO_BTF_ID | MEM_WRITE) for the node being operated on.
 * No arena map — uses the "undo-log only" JIT path (R12 cursor, no push/pop).
 *
 * Locks live in a parallel BPF array map.  list_head_idx in .bss is the
 * list head; writes to it inside the CS also get undo-log entries.
 *
 * Writes per CS: insert=3, lookup=0, update=1, delete=1.
 */
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bench_spinlock_shared.h"

struct list_lock_entry {
	struct bpf_spin_lock lock;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, BENCH_MAX_POOL);
	__type(key, __u32);
	__type(value, struct list_lock_entry);
} list_locks SEC(".maps");

__u32 list_head_idx = (__u32)~0;

SEC("fentry/bench_kmod_bpf_list_init")
int BPF_PROG(list_init, __u32 pool_size)
{
	list_head_idx = (__u32)~0;
	return 0;
}

SEC("fentry/bench_kmod_bpf_list_insert")
int BPF_PROG(list_insert, __u32 new_idx,
	     struct bench_list_node *new_node, __u32 head_lock_idx)
{
	struct list_lock_entry *hl, *nl;

	if (!new_node || new_idx >= BENCH_MAX_POOL)
		return 0;

	hl = bpf_map_lookup_elem(&list_locks, &head_lock_idx);
	nl = bpf_map_lookup_elem(&list_locks, &new_idx);
	if (!hl || !nl)
		return 0;

	bpf_spin_lock(&hl->lock);
	bpf_spin_lock(&nl->lock);

	new_node->next_idx = list_head_idx;	/* undo-log entry 1 */
	new_node->data     = bpf_ktime_get_ns();/* undo-log entry 2 */
	list_head_idx      = new_idx;		/* undo-log entry 3 (.bss) */

	bpf_spin_unlock(&nl->lock);
	bpf_spin_unlock(&hl->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_list_lookup")
int BPF_PROG(list_lookup, __u32 idx, struct bench_list_node *node)
{
	return 0;
//	struct list_lock_entry *lk;
//	__u64 val = 0;
//
//	if (!node || idx >= BENCH_MAX_POOL)
//		return 0;
//
//	lk = bpf_map_lookup_elem(&list_locks, &idx);
//	if (!lk)
//		return 0;
//
//	bpf_spin_lock(&lk->lock);
//	val = node->data;	/* read only, no undo-log entry */
//	bpf_spin_unlock(&lk->lock);
//	return (__s32)val;
}

SEC("fentry/bench_kmod_bpf_list_update")
int BPF_PROG(list_update, __u32 idx, struct bench_list_node *node, __u64 val)
{
	struct list_lock_entry *lk;

	if (!node || idx >= BENCH_MAX_POOL)
		return 0;

	lk = bpf_map_lookup_elem(&list_locks, &idx);
	if (!lk)
		return 0;

	bpf_spin_lock(&lk->lock);
	node->data = val;	/* undo-log entry 1 */
	bpf_spin_unlock(&lk->lock);
	return 0;
}

SEC("fentry/bench_kmod_bpf_list_delete")
int BPF_PROG(list_delete_op, __u32 head_idx, struct bench_list_node *head_node)
{
	struct list_lock_entry *hl;
	__u32 key0 = 0;

	if (!head_node)
		return 0;

	hl = bpf_map_lookup_elem(&list_locks, &key0);
	if (!hl)
		return 0;

	bpf_spin_lock(&hl->lock);
	if (list_head_idx != (__u32)~0)
		list_head_idx = head_node->next_idx;	/* undo-log entry 1 */
	bpf_spin_unlock(&hl->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
