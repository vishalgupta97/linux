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

#include "test_minimal_bpf_undo_log.skel.h"

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
static void test_tc01(struct test_minimal_bpf_undo_log *skel, int fd_a)
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

/* ================================================================== */
/* Main entry point                                                    */
/* ================================================================== */
void test_minimal_bpf_undo_log(void)
{
	struct test_minimal_bpf_undo_log *skel;
	int old_timeout;
	int fd_a;

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

	skel = test_minimal_bpf_undo_log__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto restore_sysctl;

	fd_a    = bpf_map__fd(skel->maps.undo_map_a);

	/* ---- timeout rollback tests ---- */
	if (test__start_subtest("tc01_stx_u64_rollback"))
		test_tc01(skel, fd_a);

	test_minimal_bpf_undo_log__destroy(skel);

restore_sysctl:
	__write_sysctl(old_timeout);
}
