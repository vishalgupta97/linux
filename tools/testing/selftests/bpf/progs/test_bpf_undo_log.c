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

/* Separate map for the "second" lock needed by nested-lock tests. */
struct undo_val2 {
	struct bpf_spin_lock lock;
	__u64 value;
};

/*
 * Large struct used by the verifier boundary tests: it carries
 * CONFIG_BPF_UNDO_LOG_MAX_ENTRIES (default 64) + a few extra u64
 * padding fields so that we can precisely write exactly 64 or 65
 * distinct map fields inside a critical section.
 */
struct undo_big {
	struct bpf_spin_lock lock;
	__u64 pad[70]; /* indices 0-69 */
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

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 4);
	__type(key, int);
	__type(value, struct undo_val2);
} undo_map_b SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct undo_big);
} undo_map_big SEC(".maps");

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

/* ------------------------------------------------------------------ */
/* TC02 – single u32 write → timeout → rollback                        */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_02_stx_u32(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u32_a = 0xdeadc0deU;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC03 – single u16 write → timeout → rollback                        */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_03_stx_u16(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u16_a = 0xabcdU;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC04 – single u8 write → timeout → rollback                         */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_04_stx_u8(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u8_a = 0xffU;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC05 – multiple writes of different sizes → timeout → all rolled back */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_05_stx_multi(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0x1111111111111111ULL;
	v->u64_b = 0x2222222222222222ULL;
	v->u32_a = 0x33333333U;
	v->u32_b = 0x44444444U;
	v->u16_a = 0x5555U;
	v->u16_b = 0x6666U;
	v->u8_a  = 0x77U;
	v->u8_b  = 0x88U;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC06 – BPF_ADD atomic on u64 → timeout → rollback                   */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_06_atomic_add64(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	__sync_fetch_and_add(&v->u64_a, 1);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC07 – BPF_AND atomic on u64 → timeout → rollback                    */
/* TC08 – BPF_OR  atomic on u64 → timeout → rollback                    */
/* TC09 – BPF_XOR atomic on u64 → timeout → rollback                   */
/* TC10 – atomic XCHG (u64)    → timeout → rollback                     */
/* TC11 – several mixed atomics → timeout → all rolled back             */
/* These use non-ADD atomics which require cpuv3+ (ENABLE_ATOMICS_TESTS) */
/* ------------------------------------------------------------------ */
#ifdef ENABLE_ATOMICS_TESTS

SEC("tc")
int tc_07_atomic_and64(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	__sync_fetch_and_and(&v->u64_a, 0x0f0f0f0f0f0f0f0fULL);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

SEC("tc")
int tc_08_atomic_or64(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	__sync_fetch_and_or(&v->u64_a, 0xf0f0f0f0f0f0f0f0ULL);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

SEC("tc")
int tc_09_atomic_xor64(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	__sync_fetch_and_xor(&v->u64_a, 0xffffffffffffffffULL);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

SEC("tc")
int tc_10_atomic_xchg64(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	__sync_lock_test_and_set(&v->u64_a, 0xbadc0ffeeULL);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

SEC("tc")
int tc_11_multi_atomic(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	__sync_fetch_and_add(&v->u64_a, 100);
	__sync_fetch_and_or(&v->u64_b, 0xff00ULL);
	__sync_fetch_and_and(&v->u64_b, 0xffff00ffULL);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

#endif /* ENABLE_ATOMICS_TESTS */

/* ------------------------------------------------------------------ */
/* TC12 – mix of STX and atomic ADD writes → timeout → all rolled back */
/* Uses only 64-bit atomic ADD for cpuv2 compatibility                 */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_12_mixed_writes(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0xaaaabbbbccccddddULL;      /* STX DW */
	__sync_fetch_and_add(&v->u64_b, 42);   /* atomic ADD DW */
	v->u32_a = 0x12345678U;                /* STX W  */
	v->u32_b = 0x9abcdef0U;                /* STX W  (was 32-bit atomic OR) */
	v->u16_a = 0x9abcU;                    /* STX H  */
	v->u8_a  = 0xefU;                      /* STX B  */
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC17 – writes to bpf_obj_new() kptr → timeout → rollback           */
/* ------------------------------------------------------------------ */

struct kobj {
	struct bpf_spin_lock lock;
	__u64 value;
};

/* kptr map – stores a single allocated kobj reference */
struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__type(key, int);
	__type(value, struct kobj);
	__uint(max_entries, 2);
} kobj_map SEC(".maps");

SEC("tc")
int tc_17_kptr_write(struct __sk_buff *ctx)
{
	int key = 0;
	struct kobj *k = bpf_map_lookup_elem(&kobj_map, &key);

	if (!k)
		return 0;

	bpf_spin_lock(&k->lock);
	k->value = 0xdeaddeaddeadULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&k->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC18 – multiple writes to kptr → timeout → all rolled back         */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_18_kptr_multi_write(struct __sk_buff *ctx)
{
	int key = 0;
	struct kobj *k = bpf_map_lookup_elem(&kobj_map, &key);
	int key2 = 1;
	struct kobj *k2 = bpf_map_lookup_elem(&kobj_map, &key2);

	if (!k || !k2)
		return 0;

	bpf_spin_lock(&k->lock);
	k->value  = 0x1111111111111111ULL;
	k2->value = 0x2222222222222222ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&k->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC20 – same address written 3× inside CS → 3 log entries → rollback */
/* The log replays in reverse; the final net result must equal the     */
/* original value before the first write.                              */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_20_reentrant_same_addr(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0x1000000000000001ULL;
	v->u64_a = 0x2000000000000002ULL;
	v->u64_a = 0x3000000000000003ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC21 – same address written 5× across two different fields         */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_21_reentrant_two_fields(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0xaaaa0001ULL;
	v->u64_b = 0xbbbb0001ULL;
	v->u64_a = 0xaaaa0002ULL;
	v->u64_b = 0xbbbb0002ULL;
	v->u64_a = 0xaaaa0003ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC22 – same u32 field written 4× → rollback restores first snapshot */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_22_reentrant_u32(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u32_a = 0x11111111U;
	v->u32_a = 0x22222222U;
	v->u32_a = 0x33333333U;
	v->u32_a = 0x44444444U;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC23 – nested locks: writes under both → timeout → both rolled back  */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_23_nested_both_writes(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val  *a = bpf_map_lookup_elem(&undo_map_a, &key);
	struct undo_val2 *b = bpf_map_lookup_elem(&undo_map_b, &key);

	if (!a || !b)
		return 0;

	bpf_spin_lock(&a->lock);
	a->u64_a = 0xAAAA0000AAAA0000ULL;
	bpf_spin_lock(&b->lock);
	b->value = 0xBBBB0000BBBB0000ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC24 – nested locks: write only under inner lock → both checked     */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_24_nested_inner_only(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val  *a = bpf_map_lookup_elem(&undo_map_a, &key);
	struct undo_val2 *b = bpf_map_lookup_elem(&undo_map_b, &key);

	if (!a || !b)
		return 0;

	bpf_spin_lock(&a->lock);
	bpf_spin_lock(&b->lock);
	b->value = 0xCCCC1234CCCC5678ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC25 – nested: outer write only                                     */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_25_nested_outer_only(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val  *a = bpf_map_lookup_elem(&undo_map_a, &key);
	struct undo_val2 *b = bpf_map_lookup_elem(&undo_map_b, &key);

	if (!a || !b)
		return 0;

	bpf_spin_lock(&a->lock);
	a->u64_a = 0xDDDDDDDDDDDDDDDDULL;
	bpf_spin_lock(&b->lock);
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TC26 – three interleaved map writes across nested lock pair         */
/* ------------------------------------------------------------------ */
SEC("tc")
int tc_26_nested_interleaved(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val  *a = bpf_map_lookup_elem(&undo_map_a, &key);
	struct undo_val2 *b = bpf_map_lookup_elem(&undo_map_b, &key);

	if (!a || !b)
		return 0;

	bpf_spin_lock(&a->lock);
	a->u64_a = 0xEEEE0000EEEE0000ULL;   /* write 1 – outer CS */
	bpf_spin_lock(&b->lock);
	b->value = 0xFFFF0000FFFF0000ULL;   /* write 2 – inner CS */
	a->u64_b = 0x1234567812345678ULL;   /* write 3 – back to outer */
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* ================================================================== */
/* TC27-TC29  Normal unlock path – values must be COMMITTED           */
/* ================================================================== */

/* TC27 – single write + normal unlock → value committed */
SEC("tc")
int tc_27_normal_commit(struct __sk_buff *ctx)
{
	int key = 1;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0x0102030405060708ULL;
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* TC28 – multiple writes + normal unlock → all committed */
SEC("tc")
int tc_28_normal_multi_commit(struct __sk_buff *ctx)
{
	int key = 2;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 0xCAFECAFECAFECAFEULL;
	v->u64_b = 0xBABEBABEBABEBABEULL;
	v->u32_a = 0xDEADBEEFU;
	v->u16_a = 0xC0DEU;
	v->u8_a  = 0xF0U;
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* TC29 – write with atomics + normal unlock → committed */
SEC("tc")
int tc_29_normal_atomic_commit(struct __sk_buff *ctx)
{
	int key = 3;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a = 100ULL;
	__sync_fetch_and_add(&v->u64_a, 5);  /* should leave 105 */
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ================================================================== */
/* TC30 – verifier ACCEPTS exactly CONFIG_BPF_UNDO_LOG_MAX_ENTRIES    */
/*        (default 64) writes in a single CS.                         */
/* ================================================================== */
#define W64(v, n)   (v)->pad[(n)] = (__u64)(n) + 1;

SEC("tc")
int tc_30_limit_exact(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_big *v = bpf_map_lookup_elem(&undo_map_big, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	/* 64 individual stores – exactly at the limit */
	W64(v,  0) W64(v,  1) W64(v,  2) W64(v,  3)
	W64(v,  4) W64(v,  5) W64(v,  6) W64(v,  7)
	W64(v,  8) W64(v,  9) W64(v, 10) W64(v, 11)
	W64(v, 12) W64(v, 13) W64(v, 14) W64(v, 15)
	W64(v, 16) W64(v, 17) W64(v, 18) W64(v, 19)
	W64(v, 20) W64(v, 21) W64(v, 22) W64(v, 23)
	W64(v, 24) W64(v, 25) W64(v, 26) W64(v, 27)
	W64(v, 28) W64(v, 29) W64(v, 30) W64(v, 31)
	W64(v, 32) W64(v, 33) W64(v, 34) W64(v, 35)
	W64(v, 36) W64(v, 37) W64(v, 38) W64(v, 39)
	W64(v, 40) W64(v, 41) W64(v, 42) W64(v, 43)
	W64(v, 44) W64(v, 45) W64(v, 46) W64(v, 47)
	W64(v, 48) W64(v, 49) W64(v, 50) W64(v, 51)
	W64(v, 52) W64(v, 53) W64(v, 54) W64(v, 55)
	W64(v, 56) W64(v, 57) W64(v, 58) W64(v, 59)
	W64(v, 60) W64(v, 61) W64(v, 62) W64(v, 63)
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ================================================================== */
/* TC31 – verifier REJECTS one write over the limit (65 writes)       */
/* ================================================================== */
SEC("?tc")
__failure
__msg("BPF critical section write count")
int tc_31_reject_over_limit(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_big *v = bpf_map_lookup_elem(&undo_map_big, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	/* 65 individual stores – one over the default limit of 64 */
	W64(v,  0) W64(v,  1) W64(v,  2) W64(v,  3)
	W64(v,  4) W64(v,  5) W64(v,  6) W64(v,  7)
	W64(v,  8) W64(v,  9) W64(v, 10) W64(v, 11)
	W64(v, 12) W64(v, 13) W64(v, 14) W64(v, 15)
	W64(v, 16) W64(v, 17) W64(v, 18) W64(v, 19)
	W64(v, 20) W64(v, 21) W64(v, 22) W64(v, 23)
	W64(v, 24) W64(v, 25) W64(v, 26) W64(v, 27)
	W64(v, 28) W64(v, 29) W64(v, 30) W64(v, 31)
	W64(v, 32) W64(v, 33) W64(v, 34) W64(v, 35)
	W64(v, 36) W64(v, 37) W64(v, 38) W64(v, 39)
	W64(v, 40) W64(v, 41) W64(v, 42) W64(v, 43)
	W64(v, 44) W64(v, 45) W64(v, 46) W64(v, 47)
	W64(v, 48) W64(v, 49) W64(v, 50) W64(v, 51)
	W64(v, 52) W64(v, 53) W64(v, 54) W64(v, 55)
	W64(v, 56) W64(v, 57) W64(v, 58) W64(v, 59)
	W64(v, 60) W64(v, 61) W64(v, 62) W64(v, 63)
	W64(v, 64) /* write #65 – over limit */
	bpf_spin_unlock(&v->lock);
	return 0;
}

#undef W64

/* ================================================================== */
/* TC35 – no writes in CS → program loads and runs cleanly            */
/* ================================================================== */
SEC("tc")
int tc_35_zero_writes(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	/* intentionally no writes */
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ================================================================== */
/* TC36 – stack write inside CS is excluded from undo log tracking    */
/* (verifier accepts it; only 1 map write is tracked, not 2)          */
/* ================================================================== */
SEC("tc")
int tc_36_stack_write_excluded(struct __sk_buff *ctx)
{
	volatile __u64 local = 0;  /* stack variable – NOT in undo log */
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	local = 0xdeadULL;          /* stack write – excluded */
	v->u64_a = 0xbeefULL;       /* map write – counted    */
	bpf_spin_unlock(&v->lock);
	return (int)local;
}

/* ================================================================== */
/* TC37 – immediate (BPF_ST) store inside CS is tracked               */
/* Using an initialiser that the compiler emits as BPF_ST              */
/* ================================================================== */
SEC("tc")
int tc_37_immediate_store(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val *v = bpf_map_lookup_elem(&undo_map_a, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u32_a = 0;   /* compiler may emit BPF_ST imm=0 */
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v->lock);
	return 0;
}

/* ================================================================== */
/* TC38 – write exactly 1 before inner lock, exactly 1 inside: counts */
/*         shared in the undo log (nested locks share the log)        */
/* ================================================================== */
SEC("tc")
int tc_38_nested_shared_log(struct __sk_buff *ctx)
{
	int key = 0;
	struct undo_val  *a = bpf_map_lookup_elem(&undo_map_a, &key);
	struct undo_val2 *b = bpf_map_lookup_elem(&undo_map_b, &key);

	if (!a || !b)
		return 0;

	bpf_spin_lock(&a->lock);
	a->u64_a = 0xAA11ULL;
	bpf_spin_lock(&b->lock);
	b->value = 0xBB22ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&b->lock);
	bpf_spin_unlock(&a->lock);
	return 0;
}

/* ================================================================== */
/* TC39 – write, unlock, write again in a second CS (separate session) */
/* Only the second session's writes should be rolled back on timeout.  */
/* ================================================================== */
SEC("tc")
int tc_39_two_cs_sessions(struct __sk_buff *ctx)
{
	int key1 = 1, key2 = 2;
	struct undo_val *v1 = bpf_map_lookup_elem(&undo_map_a, &key1);
	struct undo_val *v2 = bpf_map_lookup_elem(&undo_map_a, &key2);

	if (!v1 || !v2)
		return 0;

	/* First CS – commit */
	bpf_spin_lock(&v1->lock);
	v1->u64_a = 0xF1F1F1F1F1F1F1F1ULL;
	bpf_spin_unlock(&v1->lock);

	/* Second CS – timeout; only v2 should be rolled back */
	bpf_spin_lock(&v2->lock);
	v2->u64_a = 0x2222222222222222ULL;
	bpf_loop(LOOP_CNT, cb1, NULL, 0);
	bpf_spin_unlock(&v2->lock);
	return 0;
}

char _license[] SEC("license") = "GPL";
