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
//#include "bpf_misc.h"

#define IS_BPF_LOOP_ENABLED 1

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

int pid;

#if IS_BPF_LOOP_ENABLED 
#define LOOPS_CNT 1 << 10

static int callback_fn4(void *ctx) {
	return 0;
}

static int callback_fn3(void *ctx) {
	bpf_loop(LOOPS_CNT, callback_fn4, NULL, 0);
	return 0;
}

static int callback_fn2(void *ctx) {
	bpf_loop(LOOPS_CNT, callback_fn3, NULL, 0);
	return 0;
}

static int callback_fn(void *ctx) {
	bpf_loop(LOOPS_CNT, callback_fn2, NULL, 0);
	return 0;
}
#endif

/*
 * Test: Acquire spinlock and run a very long bpf_loop.
 * The spinlock timeout should trigger and terminate the program
 * before the loop completes naturally.
 */
//SEC("tp/syscalls/sys_enter_socket")
SEC("tc")
int test_spinlock_loop_timeout(struct __sk_buff *ctx)
{
	int key = 0;
	struct lock_data *data;
	volatile int counter = 0;

	if ((bpf_get_current_pid_tgid() >> 32) != pid)
		return 0;

	data = bpf_map_lookup_elem(&lock_map, &key);
	if (!data)
		return 0;

	bpf_printk("Acquiring lock\n");

	/* Acquire the spinlock - this starts the timeout watchdog */
	bpf_spin_lock(&data->lock);

	bpf_printk("Lock acquired, starting long loop\n");

	counter++;
#if IS_BPF_LOOP_ENABLED
	bpf_loop(LOOPS_CNT, callback_fn, NULL, 0);
#endif
	counter++;

	bpf_printk("This should not be reached: %d\n", counter);

	bpf_spin_unlock(&data->lock);

	bpf_printk("Loop completed with counter: %d\n", counter);

	return 0;
}

char _license[] SEC("license") = "GPL";
