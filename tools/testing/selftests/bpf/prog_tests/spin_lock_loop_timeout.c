// SPDX-License-Identifier: GPL-2.0
// Userspace test runner for spin lock timeout testing
#include <test_progs.h>
#include <sys/socket.h>
#include <network_helpers.h>

#include "test_spin_lock_loop_timeout.skel.h"

static void trigger_spinlock_loop_timeout(void)
{
	struct test_spin_lock_loop_timeout *skel;
	int prog_fd, err;

	skel = test_spin_lock_loop_timeout__open();
	if (!ASSERT_OK_PTR(skel, "test_spin_lock_loop_timeout__open"))
	        return;
	
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
}

void test_spin_lock_loop_timeout(void)
{
	if (test__start_subtest("trigger_spinlock_loop_timeout"))
		trigger_spinlock_loop_timeout();
}
