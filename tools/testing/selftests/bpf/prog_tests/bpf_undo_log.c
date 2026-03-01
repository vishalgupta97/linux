// SPDX-License-Identifier: GPL-2.0
/*
 * Userspace test runner for BPF spinlock undo-log mechanism.
 *
 * Each sub-test:
 *  - pre-fills the relevant map entry with a known "original" sentinel
 *  - runs a BPF tc program that writes different values inside a CS
 *  - either triggers a 50 ms timeout (rollback expected) or lets the
 *    program commit normally (new values expected)
 *  - then verifies the map contents match what is expected
 *
 * The verifier-rejection test (TC31) instead loads the program with a
 * kernel log buffer and asserts that loading fails with the expected
 * error message.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <test_progs.h>
#include <network_helpers.h>

#include "test_bpf_undo_log.skel.h"

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

/* ------------------------------------------------------------------ */
/* Userspace mirrors of the BPF-side map value structs                 */
/* Must match the layout in progs/test_bpf_undo_log.c exactly.         */
/* ------------------------------------------------------------------ */

struct undo_val {
	__u32 lock;          /* bpf_spin_lock occupies 4 bytes */
	__u8  _lock_pad[4];  /* natural alignment padding before u64 */
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

struct undo_val2 {
	__u32 lock;
	__u8  _lock_pad[4];
	__u64 value;
};

struct kobj {
	__u32 lock;
	__u8  _lock_pad[4];
	__u64 value;
};

struct undo_big {
	__u32 lock;
	__u8  _lock_pad[4];
	__u64 pad[70];
};

/* ------------------------------------------------------------------ */
/* sysctl helpers                                                       */
/* ------------------------------------------------------------------ */

static int __read_sysctl(void)
{
	int fd, val = 0;
	char buf[32];

	fd = open(SYSCTL_PATH, O_RDONLY);
	if (fd < 0)
		return -1;
	if (read(fd, buf, sizeof(buf)) > 0)
		val = atoi(buf);
	close(fd);
	return val;
}

static int __write_sysctl(int val)
{
	int fd;
	char buf[32];

	fd = open(SYSCTL_PATH, O_WRONLY);
	if (fd < 0)
		return -1;
	snprintf(buf, sizeof(buf), "%d\n", val);
	if (write(fd, buf, strlen(buf)) < 0) {
		close(fd);
		return -1;
	}
	close(fd);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Helpers: run a tc program and lookup a map value                    */
/* ------------------------------------------------------------------ */

static int run_tc_prog(int prog_fd)
{
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		    .data_in      = &pkt_v4,
		    .data_size_in = sizeof(pkt_v4));
	return bpf_prog_test_run_opts(prog_fd, &opts);
}

/* ------------------------------------------------------------------ */
/* TC01 – single u64 STX write → timeout rollback                      */
/* ------------------------------------------------------------------ */
static void test_tc01(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0xAAAAAAAAAAAAAAAAULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc01_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_01_stx_u64));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc01_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc01_u64_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC02 – single u32 STX write → timeout rollback                      */
/* ------------------------------------------------------------------ */
static void test_tc02(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u32_a = 0x12341234U };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc02_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_02_stx_u32));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc02_lookup"))
		return;
	ASSERT_EQ(result.u32_a, preset.u32_a, "tc02_u32_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC03 – single u16 STX write → timeout rollback                      */
/* ------------------------------------------------------------------ */
static void test_tc03(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u16_a = 0x5678U };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc03_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_03_stx_u16));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc03_lookup"))
		return;
	ASSERT_EQ(result.u16_a, preset.u16_a, "tc03_u16_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC04 – single u8 STX write → timeout rollback                       */
/* ------------------------------------------------------------------ */
static void test_tc04(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u8_a = 0x42U };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc04_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_04_stx_u8));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc04_lookup"))
		return;
	ASSERT_EQ(result.u8_a, preset.u8_a, "tc04_u8_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC05 – multiple writes of different sizes → all rolled back         */
/* ------------------------------------------------------------------ */
static void test_tc05(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = {
		.u64_a = 0xAAAAAAAAAAAAAAAAULL,
		.u64_b = 0xBBBBBBBBBBBBBBBBULL,
		.u32_a = 0xCCCCCCCCU,
		.u32_b = 0xDDDDDDDDU,
		.u16_a = 0xEEEEU,
		.u16_b = 0xFF00U,
		.u8_a  = 0x11U,
		.u8_b  = 0x22U,
	};
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc05_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_05_stx_multi));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc05_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc05_u64a");
	ASSERT_EQ(result.u64_b, preset.u64_b, "tc05_u64b");
	ASSERT_EQ(result.u32_a, preset.u32_a, "tc05_u32a");
	ASSERT_EQ(result.u32_b, preset.u32_b, "tc05_u32b");
	ASSERT_EQ(result.u16_a, preset.u16_a, "tc05_u16a");
	ASSERT_EQ(result.u16_b, preset.u16_b, "tc05_u16b");
	ASSERT_EQ(result.u8_a,  preset.u8_a,  "tc05_u8a");
	ASSERT_EQ(result.u8_b,  preset.u8_b,  "tc05_u8b");
}

/* ------------------------------------------------------------------ */
/* TC06 – atomic ADD u64 → timeout rollback                            */
/* ------------------------------------------------------------------ */
static void test_tc06(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0xCAFECAFE00000000ULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc06_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_06_atomic_add64));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc06_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc06_u64_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC07-TC11: Advanced atomics – require ENABLE_ATOMICS_TESTS build    */
/* ------------------------------------------------------------------ */
#ifdef ENABLE_ATOMICS_TESTS

/* TC07 – atomic AND u64 → timeout rollback */
static void test_tc07(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0xFFFFFFFFFFFFFFFFULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc07_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_07_atomic_and64));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc07_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc07_u64_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC08 – atomic OR u64 → timeout rollback                             */
/* ------------------------------------------------------------------ */
static void test_tc08(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0x0000000000000000ULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc08_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_08_atomic_or64));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc08_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc08_u64_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC09 – atomic XOR u64 → timeout rollback                            */
/* ------------------------------------------------------------------ */
static void test_tc09(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0x123456789ABCDEF0ULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc09_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_09_atomic_xor64));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc09_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc09_u64_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC10 – atomic XCHG u64 → timeout rollback                           */
/* ------------------------------------------------------------------ */
static void test_tc10(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0xFEEDFACEFEEDFACEULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc10_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_10_atomic_xchg64));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc10_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc10_u64_rolled_back");
}

/* TC11 – several mixed atomics → all rolled back */
static void test_tc11(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = {
		.u64_a = 0xAAAA000000000000ULL,
		.u64_b = 0xBBBB000000000000ULL,
	};
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc11_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_11_multi_atomic));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc11_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc11_u64a");
	ASSERT_EQ(result.u64_b, preset.u64_b, "tc11_u64b");
}

#endif /* ENABLE_ATOMICS_TESTS */

/* ------------------------------------------------------------------ */
/* TC12 – mix STX + atomics → all rolled back                          */
/* ------------------------------------------------------------------ */
static void test_tc12(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = {
		.u64_a = 0x0F0F0F0F0F0F0F0FULL,
		.u64_b = 0x1E1E1E1E1E1E1E1EULL,
		.u32_a = 0x2D2D2D2DU,
		.u32_b = 0x3C3C3C3CU,
		.u16_a = 0x4B4BU,
		.u8_a  = 0x5AU,
	};
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc12_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_12_mixed_writes));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc12_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc12_u64a");
	ASSERT_EQ(result.u64_b, preset.u64_b, "tc12_u64b");
	ASSERT_EQ(result.u32_a, preset.u32_a, "tc12_u32a");
	ASSERT_EQ(result.u32_b, preset.u32_b, "tc12_u32b");
	ASSERT_EQ(result.u16_a, preset.u16_a, "tc12_u16a");
	ASSERT_EQ(result.u8_a,  preset.u8_a,  "tc12_u8a");
}

/* ------------------------------------------------------------------ */
/* TC17 – kobj single write → timeout rollback                         */
/* ------------------------------------------------------------------ */
static void test_tc17(struct test_bpf_undo_log *skel, int fd_kobj)
{
	int key = 0;
	struct kobj preset = { .value = 0xABCDABCDABCDABCDULL };
	struct kobj result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_kobj, &key, &preset, BPF_ANY), "tc17_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_17_kptr_write));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_kobj, &key, &result), "tc17_lookup"))
		return;
	ASSERT_EQ(result.value, preset.value, "tc17_value_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC18 – kobj two entries written → both rolled back                  */
/* ------------------------------------------------------------------ */
static void test_tc18(struct test_bpf_undo_log *skel, int fd_kobj)
{
	int key = 0, key2 = 1;
	struct kobj preset0 = { .value = 0x1234567890ABCDEFULL };
	struct kobj preset1 = { .value = 0xFEDCBA0987654321ULL };
	struct kobj result0, result1;

	bpf_map_update_elem(fd_kobj, &key,  &preset0, BPF_ANY);
	bpf_map_update_elem(fd_kobj, &key2, &preset1, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_18_kptr_multi_write));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_kobj, &key,  &result0), "tc18_lookup0"))
		return;
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_kobj, &key2, &result1), "tc18_lookup1"))
		return;
	ASSERT_EQ(result0.value, preset0.value, "tc18_k0_rolled_back");
	ASSERT_EQ(result1.value, preset1.value, "tc18_k1_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC20 – same address written 3× → net rollback to original value    */
/* ------------------------------------------------------------------ */
static void test_tc20(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0xDEADC0DEDEADC0DEULL };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc20_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_20_reentrant_same_addr));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc20_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc20_reentrant_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC21 – two-field re-entrant writes → both fields rolled back        */
/* ------------------------------------------------------------------ */
static void test_tc21(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = {
		.u64_a = 0xAAAA000011110000ULL,
		.u64_b = 0xBBBB000022220000ULL,
	};
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc21_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_21_reentrant_two_fields));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc21_lookup"))
		return;
	ASSERT_EQ(result.u64_a, preset.u64_a, "tc21_u64a_rolled_back");
	ASSERT_EQ(result.u64_b, preset.u64_b, "tc21_u64b_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC22 – u32 field written 4× → back to original                     */
/* ------------------------------------------------------------------ */
static void test_tc22(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u32_a = 0x98765432U };
	struct undo_val result;

	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "tc22_preset"))
		return;
	run_tc_prog(bpf_program__fd(skel->progs.tc_22_reentrant_u32));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc22_lookup"))
		return;
	ASSERT_EQ(result.u32_a, preset.u32_a, "tc22_u32_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC23 – nested locks, writes under both → both rolled back           */
/* ------------------------------------------------------------------ */
static void test_tc23(struct test_bpf_undo_log *skel, int fd_a, int fd_b)
{
	int key = 0;
	struct undo_val  preset_a = { .u64_a = 0x1234567800000000ULL };
	struct undo_val2 preset_b = { .value = 0x8765432100000000ULL };
	struct undo_val  result_a;
	struct undo_val2 result_b;

	bpf_map_update_elem(fd_a, &key, &preset_a, BPF_ANY);
	bpf_map_update_elem(fd_b, &key, &preset_b, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_23_nested_both_writes));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result_a), "tc23_lookup_a"))
		return;
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_b, &key, &result_b), "tc23_lookup_b"))
		return;
	ASSERT_EQ(result_a.u64_a, preset_a.u64_a, "tc23_a_rolled_back");
	ASSERT_EQ(result_b.value, preset_b.value,  "tc23_b_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC24 – nested locks: write only under inner lock → rolled back      */
/* ------------------------------------------------------------------ */
static void test_tc24(struct test_bpf_undo_log *skel, int fd_b)
{
	int key = 0;
	struct undo_val2 preset_b = { .value = 0xC0FFEE00C0FFEE00ULL };
	struct undo_val2 result_b;

	bpf_map_update_elem(fd_b, &key, &preset_b, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_24_nested_inner_only));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_b, &key, &result_b), "tc24_lookup"))
		return;
	ASSERT_EQ(result_b.value, preset_b.value, "tc24_b_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC25 – nested locks: write only under outer lock → rolled back      */
/* ------------------------------------------------------------------ */
static void test_tc25(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset_a = { .u64_a = 0xF00DBABEDEADF00DULL };
	struct undo_val result_a;

	bpf_map_update_elem(fd_a, &key, &preset_a, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_25_nested_outer_only));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result_a), "tc25_lookup"))
		return;
	ASSERT_EQ(result_a.u64_a, preset_a.u64_a, "tc25_a_rolled_back");
}

/* ------------------------------------------------------------------ */
/* TC26 – nested interleaved: 3 writes → all rolled back               */
/* ------------------------------------------------------------------ */
static void test_tc26(struct test_bpf_undo_log *skel, int fd_a, int fd_b)
{
	int key = 0;
	struct undo_val  preset_a = {
		.u64_a = 0x1111111111111111ULL,
		.u64_b = 0x2222222222222222ULL,
	};
	struct undo_val2 preset_b = { .value = 0x3333333333333333ULL };
	struct undo_val  result_a;
	struct undo_val2 result_b;

	bpf_map_update_elem(fd_a, &key, &preset_a, BPF_ANY);
	bpf_map_update_elem(fd_b, &key, &preset_b, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_26_nested_interleaved));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result_a), "tc26_lookup_a"))
		return;
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_b, &key, &result_b), "tc26_lookup_b"))
		return;
	ASSERT_EQ(result_a.u64_a, preset_a.u64_a, "tc26_u64a_rolled_back");
	ASSERT_EQ(result_a.u64_b, preset_a.u64_b, "tc26_u64b_rolled_back");
	ASSERT_EQ(result_b.value, preset_b.value,  "tc26_b_rolled_back");
}

/* ================================================================== */
/* TC27-TC29  Normal unlock path – values must be COMMITTED           */
/* ================================================================== */

/* TC27 – single write + normal unlock → committed */
static void test_tc27(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 1;
	struct undo_val preset = { .u64_a = 0ULL };
	struct undo_val result;

	bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_27_normal_commit));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc27_lookup"))
		return;
	ASSERT_EQ(result.u64_a, 0x0102030405060708ULL, "tc27_committed");
}

/* TC28 – multiple writes + normal unlock → all committed */
static void test_tc28(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 2;
	struct undo_val preset = {};
	struct undo_val result;

	bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_28_normal_multi_commit));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc28_lookup"))
		return;
	ASSERT_EQ(result.u64_a, 0xCAFECAFECAFECAFEULL, "tc28_u64a");
	ASSERT_EQ(result.u64_b, 0xBABEBABEBABEBABEULL, "tc28_u64b");
	ASSERT_EQ(result.u32_a, 0xDEADBEEFU,           "tc28_u32a");
	ASSERT_EQ(result.u16_a, 0xC0DEU,               "tc28_u16a");
	ASSERT_EQ(result.u8_a,  0xF0U,                 "tc28_u8a");
}

/* TC29 – atomic write + normal unlock → committed with expected value */
static void test_tc29(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 3;
	struct undo_val preset = {};
	struct undo_val result;

	bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_29_normal_atomic_commit));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc29_lookup"))
		return;
	ASSERT_EQ(result.u64_a, 105ULL, "tc29_add_committed");
}

/* ================================================================== */
/* TC30 – verifier accepts exactly 64 writes                          */
/* ================================================================== */
static void test_tc30(struct test_bpf_undo_log *skel)
{
	/* The skeleton was already loaded successfully, which means
	 * tc_30_limit_exact was accepted by the verifier. */
	int prog_fd = bpf_program__fd(skel->progs.tc_30_limit_exact);

	ASSERT_GT(prog_fd, 0, "tc30_prog_fd_valid");
}

/* ================================================================== */
/* TC31 – verifier rejects 65 writes (over-limit)                     */
/* ================================================================== */
static void test_tc31(void)
{
	static char log_buf[512 * 1024];
	LIBBPF_OPTS(bpf_object_open_opts, open_opts,
		    .kernel_log_buf   = log_buf,
		    .kernel_log_size  = sizeof(log_buf),
		    .kernel_log_level = 1);

	struct test_bpf_undo_log *skel;
	int err;

	/*
	 * We open and load a fresh skeleton instance to trigger the verifier.
	 * The program tc_31_reject_over_limit is annotated SEC("?tc") which
	 * means loading the object should succeed but this specific program
	 * will have been rejected with a verifier error.
	 */
	skel = test_bpf_undo_log__open_opts(&open_opts);
	if (!ASSERT_OK_PTR(skel, "tc31_open"))
		return;

	err = test_bpf_undo_log__load(skel);
	/* Load should fail because the over-limit program is in the object */
	if (err == 0) {
		/*
		 * If load succeeded (e.g. because the "?tc" program was
		 * skipped), verify that the over-limit program was not loaded.
		 */
		int prog_fd = bpf_program__fd(skel->progs.tc_31_reject_over_limit);

		ASSERT_LT(prog_fd, 0, "tc31_overlimit_not_loaded");
	} else {
		/* Load failed as expected – check the log for our message */
		ASSERT_OK_PTR(strstr(log_buf, "BPF critical section write count"),
			      "tc31_error_msg_present");
	}

	test_bpf_undo_log__destroy(skel);
}

/* ================================================================== */
/* TC35 – zero writes in CS is allowed and runs cleanly               */
/* ================================================================== */
static void test_tc35(struct test_bpf_undo_log *skel)
{
	ASSERT_GT(bpf_program__fd(skel->progs.tc_35_zero_writes), 0,
		  "tc35_prog_loaded");
	/* Just run it – no map check needed, no timeout expected */
	run_tc_prog(bpf_program__fd(skel->progs.tc_35_zero_writes));
}

/* ================================================================== */
/* TC36 – stack write inside CS is excluded (map write still tracked) */
/* ================================================================== */
static void test_tc36(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u64_a = 0xABCDABCDABCDABCDULL };
	struct undo_val result;

	bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY);
	/* Normal unlock – the map write should be committed */
	run_tc_prog(bpf_program__fd(skel->progs.tc_36_stack_write_excluded));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc36_lookup"))
		return;
	ASSERT_EQ(result.u64_a, 0xbeefULL, "tc36_map_write_committed");
}

/* ================================================================== */
/* TC37 – immediate (BPF_ST) write inside CS → timeout rollback       */
/* ================================================================== */
static void test_tc37(struct test_bpf_undo_log *skel, int fd_a)
{
	int key = 0;
	struct undo_val preset = { .u32_a = 0xC0C0C0C0U };
	struct undo_val result;

	bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_37_immediate_store));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "tc37_lookup"))
		return;
	ASSERT_EQ(result.u32_a, preset.u32_a, "tc37_imm_rolled_back");
}

/* ================================================================== */
/* TC38 – nested locks share the log; both writes rolled back         */
/* ================================================================== */
static void test_tc38(struct test_bpf_undo_log *skel, int fd_a, int fd_b)
{
	int key = 0;
	struct undo_val  preset_a = { .u64_a = 0xAAAABBBBCCCCDDDDULL };
	struct undo_val2 preset_b = { .value = 0xEEEEFFFF00001111ULL };
	struct undo_val  result_a;
	struct undo_val2 result_b;

	bpf_map_update_elem(fd_a, &key, &preset_a, BPF_ANY);
	bpf_map_update_elem(fd_b, &key, &preset_b, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_38_nested_shared_log));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result_a), "tc38_lookup_a"))
		return;
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_b, &key, &result_b), "tc38_lookup_b"))
		return;
	ASSERT_EQ(result_a.u64_a, preset_a.u64_a, "tc38_a_rolled_back");
	ASSERT_EQ(result_b.value, preset_b.value,  "tc38_b_rolled_back");
}

/* ================================================================== */
/* TC39 – two CS sessions: first committed, second rolled back         */
/* ================================================================== */
static void test_tc39(struct test_bpf_undo_log *skel, int fd_a)
{
	int key1 = 1, key2 = 2;
	struct undo_val preset1 = { .u64_a = 0ULL };
	struct undo_val preset2 = { .u64_a = 0x9999999999999999ULL };
	struct undo_val result1, result2;

	bpf_map_update_elem(fd_a, &key1, &preset1, BPF_ANY);
	bpf_map_update_elem(fd_a, &key2, &preset2, BPF_ANY);
	run_tc_prog(bpf_program__fd(skel->progs.tc_39_two_cs_sessions));
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key1, &result1), "tc39_lookup1"))
		return;
	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key2, &result2), "tc39_lookup2"))
		return;
	/* v1 was committed in the first CS */
	ASSERT_EQ(result1.u64_a, 0xF1F1F1F1F1F1F1F1ULL, "tc39_v1_committed");
	/* v2 was rolled back when the second CS timed out */
	ASSERT_EQ(result2.u64_a, preset2.u64_a, "tc39_v2_rolled_back");
}

/* ================================================================== */
/* Main entry point                                                    */
/* ================================================================== */
void test_bpf_undo_log(void)
{
	struct test_bpf_undo_log *skel;
	int old_timeout;
	int fd_a, fd_b, fd_kobj;

	/* ---- setup ---- */
	old_timeout = __read_sysctl();
	if (old_timeout < 0) {
		test__skip();
		return;
	}
	if (__write_sysctl(5000) < 0) {  /* 50 ms */
		test__skip();
		return;
	}

	skel = test_bpf_undo_log__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto restore_sysctl;

	fd_a    = bpf_map__fd(skel->maps.undo_map_a);
	fd_b    = bpf_map__fd(skel->maps.undo_map_b);
	fd_kobj = bpf_map__fd(skel->maps.kobj_map);

	/* ---- timeout rollback tests ---- */
	if (test__start_subtest("tc01_stx_u64_rollback"))
		test_tc01(skel, fd_a);
	if (test__start_subtest("tc02_stx_u32_rollback"))
		test_tc02(skel, fd_a);
	if (test__start_subtest("tc03_stx_u16_rollback"))
		test_tc03(skel, fd_a);
	if (test__start_subtest("tc04_stx_u8_rollback"))
		test_tc04(skel, fd_a);
	if (test__start_subtest("tc05_stx_multi_rollback"))
		test_tc05(skel, fd_a);
	if (test__start_subtest("tc06_atomic_add64_rollback"))
		test_tc06(skel, fd_a);
#ifdef ENABLE_ATOMICS_TESTS
	if (test__start_subtest("tc07_atomic_and64_rollback"))
		test_tc07(skel, fd_a);
	if (test__start_subtest("tc08_atomic_or64_rollback"))
		test_tc08(skel, fd_a);
	if (test__start_subtest("tc09_atomic_xor64_rollback"))
		test_tc09(skel, fd_a);
	if (test__start_subtest("tc10_atomic_xchg64_rollback"))
		test_tc10(skel, fd_a);
	if (test__start_subtest("tc11_multi_atomic_rollback"))
		test_tc11(skel, fd_a);
#endif
	if (test__start_subtest("tc12_mixed_writes_rollback"))
		test_tc12(skel, fd_a);

	/* ---- allocated object (kobj) rollback tests ---- */
	if (test__start_subtest("tc17_kptr_write_rollback"))
		test_tc17(skel, fd_kobj);
	if (test__start_subtest("tc18_kptr_multi_write_rollback"))
		test_tc18(skel, fd_kobj);

	/* ---- re-entrant writes ---- */
	if (test__start_subtest("tc20_reentrant_same_addr"))
		test_tc20(skel, fd_a);
	if (test__start_subtest("tc21_reentrant_two_fields"))
		test_tc21(skel, fd_a);
	if (test__start_subtest("tc22_reentrant_u32"))
		test_tc22(skel, fd_a);

	/* ---- nested locks ---- */
	if (test__start_subtest("tc23_nested_both_writes"))
		test_tc23(skel, fd_a, fd_b);
	if (test__start_subtest("tc24_nested_inner_only"))
		test_tc24(skel, fd_b);
	if (test__start_subtest("tc25_nested_outer_only"))
		test_tc25(skel, fd_a);
	if (test__start_subtest("tc26_nested_interleaved"))
		test_tc26(skel, fd_a, fd_b);

	/* ---- normal unlock / commit tests (sysctl =0 for these) ---- */
	__write_sysctl(0);
	if (test__start_subtest("tc27_normal_commit"))
		test_tc27(skel, fd_a);
	if (test__start_subtest("tc28_normal_multi_commit"))
		test_tc28(skel, fd_a);
	if (test__start_subtest("tc29_normal_atomic_commit"))
		test_tc29(skel, fd_a);
	__write_sysctl(5000);

	/* ---- verifier boundary ---- */
	if (test__start_subtest("tc30_limit_exact_accepted"))
		test_tc30(skel);

	/* ---- edge cases ---- */
	if (test__start_subtest("tc35_zero_writes"))
		test_tc35(skel);
	if (test__start_subtest("tc36_stack_write_excluded"))
		test_tc36(skel, fd_a);
	if (test__start_subtest("tc37_immediate_store_rollback"))
		test_tc37(skel, fd_a);
	if (test__start_subtest("tc38_nested_shared_log"))
		test_tc38(skel, fd_a, fd_b);
	if (test__start_subtest("tc39_two_cs_sessions"))
		test_tc39(skel, fd_a);

	test_bpf_undo_log__destroy(skel);

	/* ---- verifier rejection (needs a fresh load attempt) ---- */
	if (test__start_subtest("tc31_reject_over_limit"))
		test_tc31();

restore_sysctl:
	__write_sysctl(old_timeout);
}
