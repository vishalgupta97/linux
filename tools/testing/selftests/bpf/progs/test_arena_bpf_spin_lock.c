// SPDX-License-Identifier: GPL-2.0
/*
 * BPF program exercising bpf_spin_lock / bpf_spin_unlock with a lock
 * resident in BPF arena memory (PTR_TO_ARENA spinlock feature).
 *
 * The spinlock and its protected payload live together in a single struct
 * stored in the arena, mirroring how map-value spin locks are typically used.
 *
 * tc_basic: acquire the arena-resident lock, write arena_obj.data = 42,
 *   release.  The write should be visible from user space after the program.
 *
 * tc_timeout: acquire the arena-resident lock, overwrite arena_obj.data with
 *   a dirty value, then spin via nested bpf_loop until the timeout fires.
 *   The undo-log replay rolls back the write; arena_obj.data must equal the
 *   sentinel written by user space before the run.
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_arena_common.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 2);
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32);
#else
	__ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

/*
 * Lock and payload in one struct — mirrors the map-value pattern but lives
 * entirely in arena memory.  The skeleton exposes it via skel->arena->arena_obj
 * so the test runner can inspect lock state and data from user space.
 */
struct arena_obj {
	struct bpf_spin_lock lock;
	__u64 data;
};

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
struct arena_obj __arena arena_obj;
int test_skip = 0;
#else
struct arena_obj arena_obj SEC(".addr_space.1");
int test_skip = 1;  /* addr_space_cast unsupported; runner will skip */
#endif

/* ---- spin helper for timeout test ---- */
#define LOOP_CNT (1 << 10)

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
static int cb4(void *ctx) { return 0; }
static int cb3(void *ctx) { bpf_loop(LOOP_CNT, cb4, NULL, 0); return 0; }
static int cb2(void *ctx) { bpf_loop(LOOP_CNT, cb3, NULL, 0); return 0; }
static int cb1(void *ctx) { bpf_loop(LOOP_CNT, cb2, NULL, 0); return 0; }
#endif

/* ---- basic correctness test ---- */

SEC("tc")
int tc_basic(struct __sk_buff *ctx)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	/*
	 * Cast &arena_obj.lock from address_space(1) to address_space(0).
	 * LLVM emits addr_space_cast; the verifier marks R1 PTR_TO_ARENA and
	 * our modified process_spin_lock() accepts it.  do_misc_fixups()
	 * injects "R0 = kern_vm_start; R1 += R0" before the call so the
	 * helper receives a valid kernel VA.
	 */
	struct bpf_spin_lock *lock = (struct bpf_spin_lock *)&arena_obj.lock;

	bpf_spin_lock(lock);
	arena_obj.data = 42ULL;
	bpf_spin_unlock(lock);
#endif
	return 0;
}

/* ---- timeout / undo-log rollback test ---- */

SEC("tc")
int tc_timeout(struct __sk_buff *ctx)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	struct bpf_spin_lock *lock = (struct bpf_spin_lock *)&arena_obj.lock;

	bpf_spin_lock(lock);
	arena_obj.data = 0xdeadbeefcafe0001ULL;  /* undo-log records old value */
	bpf_loop(LOOP_CNT, cb1, NULL, 0);        /* spin until timeout fires   */
	bpf_spin_unlock(lock);
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
