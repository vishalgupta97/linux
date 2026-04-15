// SPDX-License-Identifier: GPL-2.0
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>

struct shared_val {
	struct bpf_spin_lock lock;
	int cnt;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct shared_val);
} shared_map SEC(".maps");

volatile int null_cb_seen;
volatile int failures;

extern int bpf_execute_fn_with_lock(struct bpf_spin_lock *lock,
				   int (*callback_fn)(void *shared_data),
				   void *shared_data__nullable) __ksym;

static __always_inline int cb_inc(void *shared_data)
{
	int *cnt = shared_data;

	(*cnt)++;
	return 1;
}

static __always_inline int cb_ignore_null(void *shared_data)
{
	null_cb_seen++;
	return 1;
}

SEC("tc")
int execute_fn_with_lock_test(struct __sk_buff *skb)
{
	struct shared_val *v;
	int key = 0;
	int ret;

	v = bpf_map_lookup_elem(&shared_map, &key);
	if (!v)
		return 1;

	ret = bpf_execute_fn_with_lock(&v->lock, cb_inc, &v->cnt);
	if (ret != 1)
		failures++;

	ret = bpf_execute_fn_with_lock(&v->lock, cb_ignore_null, NULL);
	if (ret != 1)
		failures++;

	return 0;
}

char _license[] SEC("license") = "GPL";
