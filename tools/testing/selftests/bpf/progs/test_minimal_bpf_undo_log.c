// SPDX-License-Identifier: GPL-2.0
/*
 * BPF spinlock undo-log tests
 *
 * Exercises the per-CPU undo log that rolls back map writes when a
 * bpf_spin_lock critical section is terminated by the timeout handler.
 *
 * Test groups:
 *  TC01-TC12 : single lock, various write types / sizes – timeout rollback
 *  TC17-TC18 : allocated objects (bpf_obj_new) – timeout rollback
 *  TC20-TC22 : re-entrant writes (same address multiple times)
 *  TC23-TC26 : nested locks
 *  TC27-TC29 : normal unlock path (commit, no rollback)
 *  TC30-TC31 : verifier boundary (max writes / over-limit rejection)
 *  TC35-TC39 : edge-cases (zero writes, stack excluded, ctx excluded, …)
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include <bpf/bpf_core_read.h>
#include "bpf_misc.h"

/* ------------------------------------------------------------------ */
/* Map value types                                                      */
/* ------------------------------------------------------------------ */

struct undo_val {
	struct bpf_spin_lock lock;
	__u64 u64_a;
	__u64 u64_b;
	__u32 u32_a;
	__u32 u32_b;
	__u16 u16_a;
	__u16 u16_b;
	__u8  u8_a;
	__u8  u8_b;
	__u8  _pad[6];
};

/* ------------------------------------------------------------------ */
/* Maps                                                                 */
/* ------------------------------------------------------------------ */

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 4);
	__type(key, int);
	__type(value, struct undo_val);
} undo_map_a SEC(".maps");

/* ------------------------------------------------------------------ */
/* Shared infinite-spin helpers (trigger timeout)                      */
/* ------------------------------------------------------------------ */

#define LOOP_CNT (1 << 10)

static int cb4(void *ctx) { return 0; }
static int cb3(void *ctx) { bpf_loop(LOOP_CNT, cb4, NULL, 0); return 0; }
static int cb2(void *ctx) { bpf_loop(LOOP_CNT, cb3, NULL, 0); return 0; }
static int cb1(void *ctx) { bpf_loop(LOOP_CNT, cb2, NULL, 0); return 0; }

/* ------------------------------------------------------------------ */
/* TC01 – single u64 write → timeout → rollback                        */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_01_stx_u64(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0xdeadbeefcafe0001ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);   /* trigger timeout */
	bpf_spin_unlock(&v->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
