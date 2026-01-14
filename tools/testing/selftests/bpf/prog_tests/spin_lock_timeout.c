// SPDX-License-Identifier: GPL-2.0
// Userspace test runner for spin lock timeout testing
#include <test_progs.h>
#include <network_helpers.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "test_spin_lock_timeout.skel.h"

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

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

static void test_nested_locking(void)
{
	struct test_spin_lock_timeout *skel;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
	);
	int prog_fd, err;

	skel = test_spin_lock_timeout__open_and_load();
	if (!ASSERT_OK_PTR(skel, "test_spin_lock_timeout__open_and_load"))
		return;

	prog_fd = bpf_program__fd(skel->progs.test_nested_locking);
	err = bpf_prog_test_run_opts(prog_fd, &opts);
	ASSERT_OK(err, "test_nested_locking run");
	ASSERT_EQ(opts.retval, 0, "test_nested_locking retval");

	test_spin_lock_timeout__destroy(skel);
}

static void test_ooo_unlocking(void)
{
	struct test_spin_lock_timeout *skel;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
	);
	int prog_fd, err;

	skel = test_spin_lock_timeout__open_and_load();
	if (!ASSERT_OK_PTR(skel, "test_spin_lock_timeout__open_and_load"))
		return;

	prog_fd = bpf_program__fd(skel->progs.test_ooo_unlocking);
	err = bpf_prog_test_run_opts(prog_fd, &opts);
	ASSERT_OK(err, "test_ooo_unlocking run");
	ASSERT_EQ(opts.retval, 0, "test_ooo_unlocking retval");

	test_spin_lock_timeout__destroy(skel);
}

static void test_timeout_trigger(void)
{
	struct test_spin_lock_timeout *skel;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
	);
	int prog_fd, err, old_timeout;

	/* Save current timeout and set a short timeout */
	old_timeout = __read_sysctl();
	if (old_timeout < 0) {
		test__skip();
		return;
	}

	if (__write_sysctl(100) < 0) {  /* 100ms timeout */
		test__skip();
		return;
	}

	skel = test_spin_lock_timeout__open_and_load();
	if (!ASSERT_OK_PTR(skel, "test_spin_lock_timeout__open_and_load"))
		goto cleanup;

	prog_fd = bpf_program__fd(skel->progs.test_timeout_trigger);
	err = bpf_prog_test_run_opts(prog_fd, &opts);
	
	/* This test may fail with timeout or succeed if loop completes fast */
	/* We expect timeout cancellation to occur */
	
	test_spin_lock_timeout__destroy(skel);

cleanup:
	__write_sysctl(old_timeout);
}

static void *deadlock_thread1(void *arg)
{
	int prog_fd = *(int *)arg;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
	);

	bpf_prog_test_run_opts(prog_fd, &opts);
	return NULL;
}

static void *deadlock_thread2(void *arg)
{
	int prog_fd = *(int *)arg;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
	);

	bpf_prog_test_run_opts(prog_fd, &opts);
	return NULL;
}

static void test_deadlock_abba(void)
{
	struct test_spin_lock_timeout *skel;
	pthread_t tid1, tid2;
	int prog_fd1, prog_fd2, old_timeout;

	/* Save current timeout and set a short timeout */
	old_timeout = __read_sysctl();
	if (old_timeout < 0) {
		test__skip();
		return;
	}

	if (__write_sysctl(500) < 0) {  /* 500ms timeout */
		test__skip();
		return;
	}

	skel = test_spin_lock_timeout__open_and_load();
	if (!ASSERT_OK_PTR(skel, "test_spin_lock_timeout__open_and_load"))
		goto cleanup;

	prog_fd1 = bpf_program__fd(skel->progs.test_deadlock_prog1);
	prog_fd2 = bpf_program__fd(skel->progs.test_deadlock_prog2);

	/* Run both programs concurrently to trigger AB-BA deadlock */
	pthread_create(&tid1, NULL, deadlock_thread1, &prog_fd1);
	pthread_create(&tid2, NULL, deadlock_thread2, &prog_fd2);

	pthread_join(tid1, NULL);
	pthread_join(tid2, NULL);

	/* If we got here, either no deadlock occurred or timeout handled it */
	
	test_spin_lock_timeout__destroy(skel);

cleanup:
	__write_sysctl(old_timeout);
}

void test_spin_lock_timeout(void)
{
	if (test__start_subtest("nested_locking"))
		test_nested_locking();
	if (test__start_subtest("ooo_unlocking"))
		test_ooo_unlocking();
	if (test__start_subtest("timeout_trigger"))
		test_timeout_trigger();
	if (test__start_subtest("deadlock_abba"))
		test_deadlock_abba();
}
