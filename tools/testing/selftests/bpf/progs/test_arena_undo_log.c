// SPDX-License-Identifier: GPL-2.0
/*
 * BPF program for combined arena + spinlock undo-log test.
 *
 * Exercises the new "combined" code path where a program uses both an arena
 * (R12 = kern_vm_start) and the spinlock undo-log.  Inside the critical
 * section the program writes to both a regular map value and to an arena
 * global, then spins forever to trigger the timeout.  Both writes should be
 * rolled back by the undo-log replay.
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_misc.h"
#include "bpf_arena_common.h"

/* ---------- combined map: spinlock + payload ---------------------- */

struct combined_val {
	struct bpf_spin_lock lock;
	__u64 u64_a;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct combined_val);
} combined_map SEC(".maps");

/* ---------- arena ------------------------------------------------- */

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 4);
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32);
#else
	__ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

/*
 * Arena global written inside the critical section.  The compiler places this
 * at a fixed offset in arena address-space 1; the skeleton exposes it as
 * skel->arena->arena_val so the test runner can read it from user space.
 */
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
__u64 __arena arena_val;
#else
__u64 arena_val SEC(".addr_space.1");
#endif

/* ---------- timeout helpers --------------------------------------- */

#define LOOP_CNT (1 << 10)

static int cb4(void *ctx) { return 0; }
static int cb3(void *ctx) { bpf_loop(LOOP_CNT, cb4, NULL, 0); return 0; }
static int cb2(void *ctx) { bpf_loop(LOOP_CNT, cb3, NULL, 0); return 0; }
static int cb1(void *ctx) { bpf_loop(LOOP_CNT, cb2, NULL, 0); return 0; }

/* ---------- combined test ----------------------------------------- */

/*
 * Write to both a regular map value and an arena global inside a spinlock
 * critical section, then spin until the timeout fires.  Both writes must
 * be rolled back by the undo-log replay.
 */
SEC("tc")
int tc_arena_combined(struct __sk_buff *ctx)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	int key = 0;
	struct combined_val *v = bpf_map_lookup_elem(&combined_map, &key);

	if (!v)
		return 0;

	bpf_spin_lock(&v->lock);
	v->u64_a  = 0xdeadbeefcafe0001ULL;	/* map write  → undo-log entry */
	arena_val = 0xcafecafe00000002ULL;	/* arena write → combined path */
	bpf_loop(LOOP_CNT, cb1, NULL, 0);	/* spin until timeout fires    */
	bpf_spin_unlock(&v->lock);
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
