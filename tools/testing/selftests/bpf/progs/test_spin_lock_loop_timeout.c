// SPDX-License-Identifier: GPL-2.0
/*
 * Test BPF spinlock timeout mechanism with bpf_loop
 *
 * This test verifies that when a BPF program holds a spinlock and
 * enters a long-running bpf_loop, the timeout mechanism correctly:
 * 1. Sets the ebpf_spinlock_timeout flag via hrtimer
 * 2. Detects the flag in bpf_loop iterations
 * 3. Calls bpf_spin_lock_timeout_handler to release locks
 * 4. Terminates the program via bpf_die
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

struct lock_data {
	struct bpf_spin_lock lock;
	int counter;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct lock_data);
} lock_map SEC(".maps");

static long loop_callback(u32 index, void *ctx)
{
	volatile int *counter = ctx;

	/* Do some work each iteration */
	(*counter)++;

	/* Never return 1, so loop continues until timeout or nr_loops */
	return 0;
}

/*
 * Test: Acquire spinlock and run a very long bpf_loop.
 * The spinlock timeout should trigger and terminate the program
 * before the loop completes naturally.
 *
 * Expected behavior:
 * - Lock is acquired
 * - Loop starts running
 * - hrtimer fires after sysctl_bpf_spin_lock_timeout ms
 * - ebpf_spinlock_timeout flag is set
 * - bpf_loop detects flag and calls timeout handler
 * - Lock is released and program is terminated
 */
SEC("tc")
int test_spinlock_loop_timeout(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_data *data;
	volatile int counter = 0;

	data = bpf_map_lookup_elem(&lock_map, &key);
	if (!data)
		return 0;

	/* Acquire the spinlock - this starts the timeout watchdog */
	bpf_spin_lock(&data->lock);

	bpf_printk("Lock acquired, starting long loop\n");

	/*
	 * Run a very large number of iterations.
	 * With the timeout mechanism, this should be interrupted
	 * before completing all iterations.
	 * Using 0xFFFFFFFF (max u32) to ensure we hit the timeout.
	 */
	bpf_loop(0xFFFFFFFF, loop_callback, (void *)&counter, 0);

	/*
	 * If we reach here without the timeout firing, unlock normally.
	 * In the expected timeout case, bpf_die() is called and we
	 * never reach this point.
	 */
	bpf_spin_unlock(&data->lock);

	bpf_printk("Loop completed with counter: %d\n", counter);

	return 0;
}

char _license[] SEC("license") = "GPL";
