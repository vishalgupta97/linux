// SPDX-License-Identifier: GPL-2.0
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

struct val {
	int cnt;
	struct bpf_spin_lock l;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 2);
	__type(key, int);
	__type(value, struct val);
} map_s SEC(".maps");

SEC("cgroup/skb")
__description("nested lock: lock A then lock B (success)")
__success __failure_unpriv __msg_unpriv("")
__naked void nested_lock_success(void)
{
	asm volatile ("					\
	r1 = 0;						\
	*(u32*)(r10 - 4) = r1;				\
	r2 = r10;					\
	r2 += -4;					\
	r1 = %[map_s] ll;			\
	call %[bpf_map_lookup_elem];			\
	if r0 != 0 goto l0_%=;				\
	exit;						\
l0_%=:	r6 = r0;					\
	r7 = 1;						\
	*(u32*)(r10 - 4) = r7;				\
	r2 = r10;					\
	r2 += -4;					\
	r1 = %[map_s] ll;			\
	call %[bpf_map_lookup_elem];			\
	if r0 != 0 goto l1_%=;				\
	exit;						\
l1_%=:	r7 = r0;					\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_lock];				\
	r1 = r7;					\
	r1 += 4;					\
	call %[bpf_spin_lock];				\
	r1 = r7;					\
	r1 += 4;					\
	call %[bpf_spin_unlock];			\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_unlock];			\
	r0 = 0;						\
	exit;						\
"	:
	: __imm(bpf_map_lookup_elem),
	  __imm(bpf_spin_lock),
	  __imm(bpf_spin_unlock),
	  __imm_addr(map_s)
	: __clobber_all);
}

SEC("cgroup/skb")
__description("nested lock: out of order unlock (success)")
__success __failure_unpriv __msg_unpriv("")
__naked void nested_lock_ooo_unlock(void)
{
	asm volatile ("					\
	r1 = 0;						\
	*(u32*)(r10 - 4) = r1;				\
	r2 = r10;					\
	r2 += -4;					\
	r1 = %[map_s] ll;			\
	call %[bpf_map_lookup_elem];			\
	if r0 != 0 goto l0_%=;				\
	exit;						\
l0_%=:	r6 = r0;					\
	r7 = 1;						\
	*(u32*)(r10 - 4) = r7;				\
	r2 = r10;					\
	r2 += -4;					\
	r1 = %[map_s] ll;			\
	call %[bpf_map_lookup_elem];			\
	if r0 != 0 goto l1_%=;				\
	exit;						\
l1_%=:	r7 = r0;					\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_lock];				\
	r1 = r7;					\
	r1 += 4;					\
	call %[bpf_spin_lock];				\
	r1 = r6;					\
	r1 += 4;					\
	/* unlock A first - allowed now */      \
	call %[bpf_spin_unlock];			\
	r1 = r7;					\
	r1 += 4;					\
	call %[bpf_spin_unlock];			\
	r0 = 0;						\
	exit;						\
"	:
	: __imm(bpf_map_lookup_elem),
	  __imm(bpf_spin_lock),
	  __imm(bpf_spin_unlock),
	  __imm_addr(map_s)
	: __clobber_all);
}

SEC("cgroup/skb")
__description("nested lock: AA deadlock (fail)")
__failure __msg("Acquiring the same lock again")
__failure_unpriv __msg_unpriv("")
__naked void nested_lock_aa_fail(void)
{
	asm volatile ("					\
	r1 = 0;						\
	*(u32*)(r10 - 4) = r1;				\
	r2 = r10;					\
	r2 += -4;					\
	r1 = %[map_s] ll;			\
	call %[bpf_map_lookup_elem];			\
	if r0 != 0 goto l0_%=;				\
	exit;						\
l0_%=:	r6 = r0;					\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_lock];				\
	r1 = r6;					\
	r1 += 4;					\
	/* Lock A again */              \
	call %[bpf_spin_lock];				\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_unlock];			\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_unlock];			\
	r0 = 0;						\
	exit;						\
"	:
	: __imm(bpf_map_lookup_elem),
	  __imm(bpf_spin_lock),
	  __imm(bpf_spin_unlock),
	  __imm_addr(map_s)
	: __clobber_all);
}

SEC("cgroup/skb")
__description("helper call in lock (success)")
__success __failure_unpriv __msg_unpriv("")
__naked void helper_call_in_lock(void)
{
	asm volatile ("					\
	r1 = 0;						\
	*(u32*)(r10 - 4) = r1;				\
	r2 = r10;					\
	r2 += -4;					\
	r1 = %[map_s] ll;			\
	call %[bpf_map_lookup_elem];			\
	if r0 != 0 goto l0_%=;				\
	exit;						\
l0_%=:	r6 = r0;					\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_lock];				\
	call %[bpf_get_prandom_u32];			\
	r1 = r6;					\
	r1 += 4;					\
	call %[bpf_spin_unlock];			\
	r0 = 0;						\
	exit;						\
"	:
	: __imm(bpf_map_lookup_elem),
	  __imm(bpf_spin_lock),
	  __imm(bpf_spin_unlock),
	  __imm(bpf_get_prandom_u32),
	  __imm_addr(map_s)
	: __clobber_all);
}

char _license[] SEC("license") = "GPL";
