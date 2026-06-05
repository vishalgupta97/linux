// SPDX-License-Identifier: GPL-2.0
/*
 * Userspace test runner for the bpf_lock_func() helper.
 *
 *  - verifier subtests (RUN_TESTS): accept locking a different lock inside the
 *    callback, reject nested bpf_lock_func(), reject AA deadlock on the same
 *    lock.
 *  - runtime subtests: a callback write commits on the normal path and is
 *    rolled back when the critical section is aborted by the timeout handler.
 */
#include <fcntl.h>
#include <unistd.h>

#include <test_progs.h>
#include <network_helpers.h>

#include "test_lock_func.skel.h"

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

/* Userspace mirror of struct lockval in progs/test_lock_func.c. */
struct lockval {
	__u32 lock;          /* bpf_spin_lock occupies 4 bytes */
	__u8  _lock_pad[4];  /* alignment padding before __u64  */
	__u64 x;
};

static bool has_bpf_timeout(void)
{
	int fd = open(SYSCTL_PATH, O_RDONLY);

	if (fd < 0)
		return false;
	close(fd);
	return true;
}

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

static int run_tc_prog(int prog_fd)
{
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		    .data_in      = &pkt_v4,
		    .data_size_in = sizeof(pkt_v4));
	return bpf_prog_test_run_opts(prog_fd, &opts);
}

/* Normal path: the callback write to map_a[0] is committed. */
static void test_commit(void)
{
	const __u64 sentinel = 0xAAAAAAAAAAAAAAAAULL;
	struct test_lock_func *skel;
	struct lockval preset = { .x = sentinel };
	struct lockval result;
	int key = 0, fd_a, err;

	/* Disable the timeout so the callback always runs to completion. */
	__write_sysctl(0);

	skel = test_lock_func__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		return;

	fd_a = bpf_map__fd(skel->maps.map_a);
	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "preset"))
		goto out;

	err = run_tc_prog(bpf_program__fd(skel->progs.lf_commit));
	if (!ASSERT_OK(err, "run lf_commit"))
		goto out;

	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "lookup"))
		goto out;
	ASSERT_EQ(result.x, 0x0102030405060708ULL, "committed");
out:
	test_lock_func__destroy(skel);
}

/* Timeout path: the callback write to map_a[1] is rolled back. */
static void test_timeout_rollback(void)
{
	const __u64 sentinel = 0x1234567812345678ULL;
	struct test_lock_func *skel;
	struct lockval preset = { .x = sentinel };
	struct lockval result;
	int key = 1, fd_a, old_timeout;

	if (!has_bpf_timeout()) {
		test__skip();
		return;
	}

	old_timeout = __read_sysctl();
	if (old_timeout < 0 || __write_sysctl(1000) < 0) {  /* 1s timeout */
		test__skip();
		return;
	}

	skel = test_lock_func__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto restore;

	fd_a = bpf_map__fd(skel->maps.map_a);
	if (!ASSERT_OK(bpf_map_update_elem(fd_a, &key, &preset, BPF_ANY), "preset"))
		goto out;

	/* lf_timeout spins forever; the timeout handler rolls back and aborts. */
	run_tc_prog(bpf_program__fd(skel->progs.lf_timeout));

	if (!ASSERT_OK(bpf_map_lookup_elem(fd_a, &key, &result), "lookup"))
		goto out;
	ASSERT_EQ(result.x, sentinel, "rolled_back");
out:
	test_lock_func__destroy(skel);
restore:
	__write_sysctl(old_timeout);
}

void test_lock_func(void)
{
	/* Verifier accept/reject subtests. */
	RUN_TESTS(test_lock_func);

	if (test__start_subtest("commit"))
		test_commit();
	if (test__start_subtest("timeout_rollback"))
		test_timeout_rollback();
}
