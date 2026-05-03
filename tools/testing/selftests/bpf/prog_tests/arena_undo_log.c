// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side test runner for the combined arena + spinlock undo-log feature.
 *
 * test_arena_undo_combined
 *   Loads a BPF program that uses both an arena and the spinlock undo-log.
 *   Prior to this feature the verifier/JIT rejected such programs with
 *   -EOPNOTSUPP because both uses of R12 were incompatible.  Now arena
 *   occupies the lower 2 GB of the 4 GB window (bit 31 = 0) and the undo-log
 *   pages are pre-allocated in the upper 2 GB.  The program writes to both a
 *   regular map value and an arena global inside a spinlock critical section,
 *   then spins forever to trigger the timeout.  After rollback both writes
 *   must be back to their pre-test sentinel values.
 *
 * test_arena_bounds
 *   Verifies that creating an arena with more than SZ_2G / PAGE_SIZE pages
 *   fails with ETOOBIG (E2BIG) now that the upper half of the 4 GB window is
 *   reserved for undo-log pages.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <test_progs.h>
#include <network_helpers.h>

#include "test_arena_undo_log.skel.h"

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

/* ------------------------------------------------------------------ */
/* sysctl helpers (same pattern as minimal_bpf_undo_log)              */
/* ------------------------------------------------------------------ */

static int aul_read_sysctl(void)
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

static int aul_write_sysctl(int val)
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
/* test_arena_undo_combined                                            */
/* ------------------------------------------------------------------ */

#define MAP_SENTINEL  0xAAAAAAAAAAAAAAAAULL
#define ARENA_SENTINEL 0xBBBBBBBBBBBBBBBBULL

static void test_arena_undo_combined(void)
{
	struct test_arena_undo_log *skel;
	struct combined_val {
		__u32 lock;
		__u8  _pad[4];
		__u64 u64_a;
	} preset = { .u64_a = MAP_SENTINEL }, result;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		    .data_in      = &pkt_v4,
		    .data_size_in = sizeof(pkt_v4));
	int old_timeout, key = 0;

	old_timeout = aul_read_sysctl();
	if (old_timeout < 0) {
		test__skip();
		return;
	}
	/* 5-second timeout in milliseconds */
	if (aul_write_sysctl(5000) < 0) {
		test__skip();
		goto restore;
	}

	skel = test_arena_undo_log__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto restore;

	/* Pre-fill the map with a known sentinel value. */
	if (!ASSERT_OK(bpf_map_update_elem(bpf_map__fd(skel->maps.combined_map),
					   &key, &preset, BPF_ANY), "preset_map"))
		goto destroy;

	/*
	 * Touch the arena page that holds arena_val from user space to
	 * ensure the physical page is backed before the BPF program's
	 * undo-log push reads the old value from the kernel arena VA.
	 * Writing the sentinel also gives us a known value to compare
	 * against after the rollback.
	 */
	if (skel->arena) {
		skel->arena->arena_val = ARENA_SENTINEL;
	} else {
		/* addr_space_cast not supported by the compiler; skip arena check */
		test__skip();
		goto destroy;
	}

	/* Run the combined program; it spins until the timeout fires. */
	bpf_prog_test_run_opts(bpf_program__fd(skel->progs.tc_arena_combined),
			       &opts);

	/* Verify the map write was rolled back. */
	if (!ASSERT_OK(bpf_map_lookup_elem(bpf_map__fd(skel->maps.combined_map),
					   &key, &result), "lookup_map"))
		goto destroy;
	ASSERT_EQ(result.u64_a, MAP_SENTINEL, "map_u64_rolled_back");

	/* Verify the arena write was rolled back. */
	ASSERT_EQ(skel->arena->arena_val, ARENA_SENTINEL, "arena_val_rolled_back");

destroy:
	test_arena_undo_log__destroy(skel);
restore:
	aul_write_sysctl(old_timeout);
}

/* ------------------------------------------------------------------ */
/* test_arena_bounds                                                   */
/* ------------------------------------------------------------------ */

static void test_arena_bounds(void)
{
	LIBBPF_OPTS(bpf_map_create_opts, opts,
		    .map_flags = BPF_F_MMAPABLE,
#ifdef __TARGET_ARCH_arm64
		    .map_extra = 0x1ull << 32
#else
		    .map_extra = 0x1ull << 44
#endif
	);
	long page_sz = sysconf(_SC_PAGESIZE);
	/* max_entries that exceeds the 2 GB limit by one page */
	__u32 too_many = (__u32)(0x80000000u / page_sz) + 1;
	int fd;

	/* Use a tiny 1-page arena to verify basic arena support on this kernel. */
	fd = bpf_map_create(BPF_MAP_TYPE_ARENA, NULL, 0, 0, 1, &opts);
	if (fd < 0) {
		/* Arena not supported (no JIT, no CONFIG_BPF, etc.) — skip. */
		test__skip();
		return;
	}
	close(fd);

	/* An arena exceeding 2 GB must fail with E2BIG now that the upper
	 * 2 GB of the 4 GB window is reserved for undo-log pages. */
	fd = bpf_map_create(BPF_MAP_TYPE_ARENA, NULL, 0, 0, too_many, &opts);
	if (ASSERT_LT(fd, 0, "oversized_arena_must_fail"))
		ASSERT_EQ(errno, E2BIG, "arena_too_large_errno");
	if (fd >= 0)
		close(fd);
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */

void test_arena_undo_log(void)
{
	if (test__start_subtest("combined"))
		test_arena_undo_combined();
	if (test__start_subtest("arena_bounds"))
		test_arena_bounds();
}
