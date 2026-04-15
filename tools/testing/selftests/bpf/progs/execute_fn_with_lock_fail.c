// SPDX-License-Identifier: GPL-2.0
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

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

extern int bpf_execute_fn_with_lock(struct bpf_spin_lock *lock,
				   int (*callback_fn)(void *shared_data),
				   void *shared_data__nullable) __ksym;

static __always_inline int cb_null_deref(void *shared_data)
{
	return *(int *)shared_data;
}

static __always_inline int cb_invalid_ret(void *shared_data)
{
	return 2;
}

SEC("?tc")
__failure __msg("invalid mem access 'scalar'")
int execute_fn_with_lock_null_deref(void *ctx)
{
	struct shared_val *v;
	int key = 0;

	v = bpf_map_lookup_elem(&shared_map, &key);
	if (!v)
		return 0;

	return bpf_execute_fn_with_lock(&v->lock, cb_null_deref, NULL);
}

SEC("?tc")
__failure __msg("At callback return the register R0 has")
int execute_fn_with_lock_bad_ret(void *ctx)
{
	struct shared_val *v;
	int key = 0;

	v = bpf_map_lookup_elem(&shared_map, &key);
	if (!v)
		return 0;

	return bpf_execute_fn_with_lock(&v->lock, cb_invalid_ret, &v->cnt);
}

char _license[] SEC("license") = "GPL";
