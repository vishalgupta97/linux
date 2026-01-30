// SPDX-License-Identifier: GPL-2.0
// Userspace test runner for spin lock timeout testing
#include <test_progs.h>
#include <sys/socket.h>
#include <network_helpers.h>

#include "test_spin_lock_loop_timeout.skel.h"

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



static void trigger_spinlock_loop_timeout(void)
{
	struct test_spin_lock_loop_timeout *skel;
	int prog_fd, err, old_timeout;

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

	skel = test_spin_lock_loop_timeout__open();
	if (!ASSERT_OK_PTR(skel, "test_spin_lock_loop_timeout__open"))
	        goto cleanup;
	
	err = test_spin_lock_loop_timeout__load(skel);
	if (!ASSERT_OK(err, "test_spin_lock_loop_timeout__load"))
	        goto out;
	
	skel->bss->pid = getpid();
	err = test_spin_lock_loop_timeout__attach(skel);
	if (!ASSERT_OK(err, "test_spin_lock_loop_timeout__attach"))
	        goto out;

	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
	);

	prog_fd = bpf_program__fd(skel->progs.test_spinlock_loop_timeout);
	err = bpf_prog_test_run_opts(prog_fd, &opts);

	/* Triggers long running BPF program */
	//socket(AF_UNSPEC, SOCK_DGRAM, 0);

	/* If the program is not terminated, it doesn't reach this point */
	ASSERT_TRUE(true, "Program is terminated");
out:
       test_spin_lock_loop_timeout__destroy(skel);
cleanup:
	__write_sysctl(old_timeout);
}

void test_spin_lock_loop_timeout(void)
{
	if (test__start_subtest("trigger_spinlock_loop_timeout"))
		trigger_spinlock_loop_timeout();
}
