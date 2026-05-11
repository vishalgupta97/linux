// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side test runner for bpf_spin_lock / bpf_spin_unlock with a lock
 * stored inside a struct in BPF arena memory (PTR_TO_ARENA spinlock feature).
 *
 * arena_bpf_spin_lock/basic
 *   Loads the BPF program, runs tc_basic which acquires an arena-resident
 *   spinlock, writes 42 to arena_obj.data, and releases.  Verifies
 *   arena_obj.data reads back as 42 from user space.
 *
 * arena_bpf_spin_lock/timeout_rollback
 *   Writes a sentinel to arena_obj.data, sets a 5-second timeout via
 *   sysctl, then runs tc_timeout which overwrites arena_obj.data inside the
 *   critical section and spins forever.  After the timeout fires and the
 *   undo-log rolls back the write, arena_obj.data must equal the sentinel.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <test_progs.h>
#include <network_helpers.h>

/*
 * Must be defined before the skeleton include so that the arena member type
 * is complete.  Layout must match struct arena_obj in the BPF program:
 *   struct bpf_spin_lock lock  (4 bytes at offset 0)
 *   4 bytes natural padding    (offset 4-7)
 *   __u64 data                 (8 bytes at offset 8)
 * Total: 16 bytes.
 */
struct arena_obj {
	__u32 lock;   /* struct bpf_spin_lock is { __u32 val; } */
	__u32 _pad;
	__u64 data;
};

#include "test_arena_bpf_spin_lock.skel.h"

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

/* ------------------------------------------------------------------ */
/* sysctl helpers (prefixed to avoid conflict with test_progs.h)      */
/* ------------------------------------------------------------------ */

static int absl_read_sysctl(void)
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

static int absl_write_sysctl(int val)
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
/* basic: arena struct lock acquire / write / release                  */
/* ------------------------------------------------------------------ */

static void test_basic(void)
{
	struct test_arena_bpf_spin_lock *skel;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		    .data_in      = &pkt_v4,
		    .data_size_in = sizeof(pkt_v4));

	skel = test_arena_bpf_spin_lock__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		return;

	if (skel->bss->test_skip) {
		test__skip();
		goto destroy;
	}

	if (!skel->arena) {
		/* Arena mmap unavailable on this kernel; skip. */
		test__skip();
		goto destroy;
	}

	/*
	 * Write to arena_obj.data from user space to back the arena page that
	 * holds the entire struct (lock + data).  arena_obj.lock is
	 * zero-initialised by the anonymous mmap (= unlocked qspinlock).
	 */
	skel->arena->arena_obj.data = 0;

	if (!ASSERT_OK(bpf_prog_test_run_opts(
			bpf_program__fd(skel->progs.tc_basic), &opts),
		       "test_run_basic"))
		goto destroy;

	ASSERT_EQ(skel->arena->arena_obj.data, 42ULL, "arena_data_written");

destroy:
	test_arena_bpf_spin_lock__destroy(skel);
}

/* ------------------------------------------------------------------ */
/* timeout_rollback: dirty write in CS spun forever → undo-log restores */
/* ------------------------------------------------------------------ */

#define ARENA_SENTINEL 0xBBBBBBBBBBBBBBBBULL

static void test_timeout_rollback(void)
{
	struct test_arena_bpf_spin_lock *skel;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		    .data_in      = &pkt_v4,
		    .data_size_in = sizeof(pkt_v4));
	int old_timeout;

	old_timeout = absl_read_sysctl();
	if (old_timeout < 0) {
		/* Timeout sysctl absent: feature not compiled in; skip. */
		test__skip();
		return;
	}
	/* 5-second timeout in milliseconds */
	if (absl_write_sysctl(5000) < 0) {
		test__skip();
		goto restore;
	}

	skel = test_arena_bpf_spin_lock__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto restore;

	if (skel->bss->test_skip) {
		test__skip();
		goto destroy;
	}

	if (!skel->arena) {
		test__skip();
		goto destroy;
	}

	/*
	 * Pre-fill arena_obj.data with a sentinel.  Writing it from user space
	 * also backs the arena page so arena_obj.lock is physically mapped and
	 * zeroed (= unlocked).  The BPF program overwrites data with a dirty
	 * value inside the CS, then spins until the timeout fires.  The
	 * undo-log replay should restore the sentinel.
	 */
	skel->arena->arena_obj.data = ARENA_SENTINEL;

	/* Run the program; it is terminated by the timeout. */
	bpf_prog_test_run_opts(bpf_program__fd(skel->progs.tc_timeout), &opts);

	ASSERT_EQ(skel->arena->arena_obj.data, ARENA_SENTINEL,
		  "arena_data_rolled_back");

destroy:
	test_arena_bpf_spin_lock__destroy(skel);
restore:
	absl_write_sysctl(old_timeout);
}

/* ------------------------------------------------------------------ */
/* entry point                                                         */
/* ------------------------------------------------------------------ */

void test_arena_bpf_spin_lock(void)
{
	if (test__start_subtest("basic"))
		test_basic();
	if (test__start_subtest("timeout_rollback"))
		test_timeout_rollback();
}
