# PTR_TO_ARENA Spin Lock Support — Change Summary

## Overview

This change extends the BPF verifier and selftest suite to allow
`bpf_spin_lock` / `bpf_spin_unlock` to be called with a lock pointer stored
in BPF arena memory.  Previously the verifier rejected such programs because
`ARG_PTR_TO_SPIN_LOCK` only accepted `PTR_TO_MAP_VALUE` and
`PTR_TO_BTF_ID | MEM_ALLOC`.  Arena pointers carry type `PTR_TO_ARENA`
(a 32-bit user-space offset, not a kernel VA), so two additional problems had
to be solved:

1. **Calling convention**: the helper `bpf_spin_lock` expects a kernel VA in
   R1, but the BPF program holds a 32-bit arena offset.  The verifier must
   inject `R0 = kern_vm_start; R1 += R0` before each such call.

2. **Undo-log coverage**: arena store instructions are converted from
   `BPF_MEM` to `BPF_PROBE_MEM32` by `convert_ctx_accesses()` before
   `do_misc_fixups()` runs.  The undo-log injection previously matched only
   `BPF_MEM | BPF_ATOMIC`, so arena writes inside a critical section were
   silently un-logged.

---

## Files Changed

### 1. `include/linux/bpf_verifier.h`

**What**: Added one field to `struct bpf_insn_aux_data`.

```c
/* R1 of this bpf_spin_{lock,unlock} call is PTR_TO_ARENA; do_misc_fixups()
 * must inject the arena-offset to kernel-VA conversion before the call. */
bool spin_lock_arena_arg;
```

Placed after the existing `in_critical_section` bool (~line 590).

**Why**: `do_misc_fixups()` processes instructions in a second pass after
verification.  It needs a per-instruction flag to know which `bpf_spin_lock`/
`bpf_spin_unlock` call sites require the kern_vm_start fixup, because by
`do_misc_fixups` time all register type information from the verifier pass is
gone.  The `bpf_insn_aux_data` array survives instruction patching correctly
because `adjust_insn_aux_data()` uses `memmove` to shift entries whenever
`bpf_patch_insn_data` inserts new instructions.

---

### 2. `kernel/bpf/verifier.c`

Four change sites:

#### 2a. `spin_lock_types` (~line 9470)

Added `PTR_TO_ARENA` to the set of register types accepted for
`ARG_PTR_TO_SPIN_LOCK`:

```c
static const struct bpf_reg_types spin_lock_types = {
    .types = {
        PTR_TO_MAP_VALUE,
        PTR_TO_BTF_ID | MEM_ALLOC,
        PTR_TO_ARENA,           /* NEW */
    }
};
```

#### 2b. `process_spin_lock()` (~line 8640)

Four sub-changes inside the existing function body:

**Skip `is_const` check for arena** — arena offsets are runtime values with
no static struct layout:
```c
if (!is_const && reg->type != PTR_TO_ARENA) {
    verbose(env, "R%d doesn't have constant offset...\n", ...);
    return -EINVAL;
}
```

**Arena branch** — reject `bpf_res_spin_lock` (requires BTF), skip BTF
struct-layout verification:
```c
} else if (reg->type == PTR_TO_ARENA) {
    if (is_res_lock) {
        verbose(env, "bpf_res_spin_lock is not supported in arena memory\n");
        return -EINVAL;
    }
    /* Skip struct-layout verification (no BTF record for arena). */
}
```

**BTF record check guarded for arena**:
```c
if (reg->type != PTR_TO_ARENA) {
    rec = reg_btf_record(reg);
    if (!btf_record_has_field(rec, ...)) { ... return -EINVAL; }
    spin_lock_off = ...;
    if (spin_lock_off != val + reg->off) { ... return -EINVAL; }
}
```

**Set `spin_lock_arena_arg` flag and use arena as the `ptr` key** for lock
state tracking in both the lock and unlock branches:
```c
if (reg->type == PTR_TO_ARENA)
    env->insn_aux_data[env->insn_idx].spin_lock_arena_arg = true;

/* lock path: */
if (map)                        ptr = map;
else if (reg->type == PTR_TO_ARENA) ptr = env->prog->aux->arena;
else                            ptr = btf;

/* unlock path: same three-way selection */
```

**AA deadlock detection skipped for arena** — all `PTR_TO_ARENA` registers
have `id = 0` after `mark_reg_unknown` (set by the `BPF_ADDR_SPACE_CAST`
handler), so `find_lock_state(..., id=0)` would produce false positives when
multiple arena locks are used:
```c
if (reg->type != PTR_TO_ARENA &&
    find_lock_state(env->cur_state, REF_TYPE_LOCK, reg->id, ptr)) {
    verbose(env, "Acquiring the same lock again, AA deadlock detected\n");
    return -EINVAL;
}
```

#### 2c. `do_misc_fixups()` — undo-log injection (~line 23572)

Extended the store-mode match to also cover `BPF_PROBE_MEM32`, the mode that
`convert_ctx_accesses()` assigns to arena stores:

```c
if ((cls == BPF_STX || cls == BPF_ST) &&
    (mode == BPF_MEM || mode == BPF_ATOMIC ||
     mode == BPF_PROBE_MEM32)) {          /* NEW: arena stores */
```

The x86 JIT's `emit_undo_log_push_combined()` already handles
`BPF_PROBE_MEM32` markers via `is_arena = (BPF_MODE(next->code) == BPF_PROBE_MEM32)`,
so no JIT changes were needed.

#### 2d. `do_misc_fixups()` — arena spin lock call fixup (~line 23593)

Inserted just before the existing `BPF_ADDR_SPACE_CAST` fixup.  When a
`bpf_spin_lock`/`bpf_spin_unlock` CALL instruction has `spin_lock_arena_arg`
set, injects three instructions before the call to convert the 32-bit arena
offset in R1 into a full kernel VA:

```c
if (insn->code == (BPF_JMP | BPF_CALL) && !insn->src_reg &&
    (insn->imm == BPF_FUNC_spin_lock ||
     insn->imm == BPF_FUNC_spin_unlock) &&
    env->insn_aux_data[i + delta].spin_lock_arena_arg) {

    u64 kern_vm_start = bpf_arena_get_kern_vm_start(
            (struct bpf_arena *)env->prog->aux->arena);
    struct bpf_insn kva[2] = { BPF_LD_IMM64(BPF_REG_0, kern_vm_start) };

    cnt = 0;
    insn_buf[cnt++] = kva[0];      /* LD_IMM64 R0, kern_vm_start (low)  */
    insn_buf[cnt++] = kva[1];      /* LD_IMM64 R0, kern_vm_start (high) */
    insn_buf[cnt++] = BPF_ALU64_REG(BPF_ADD, BPF_REG_1, BPF_REG_0); /* R1 += R0 */
    insn_buf[cnt++] = *insn;       /* original bpf_spin_{lock,unlock}   */
    ...
}
```

R0 is caller-saved and `bpf_spin_lock` returns void, so clobbering R0 here
is safe.

---

### 3. `tools/testing/selftests/bpf/progs/test_arena_bpf_spin_lock.c` (new)

BPF-side program exercising the new feature.  Key design points:

- Declares `struct arena_obj { struct bpf_spin_lock lock; __u64 data; } __arena arena_obj` — a single struct in arena memory combining the spinlock and its protected payload (mirrors the map-value pattern).
- Guards with `#ifdef __BPF_FEATURE_ADDR_SPACE_CAST`; falls back to `SEC(".addr_space.1")` for older compilers and sets `test_skip = 1` for the runner.
- Two programs:
  - `tc_basic`: acquire arena lock, write `arena_obj.data = 42`, release.  Proves the feature works without timeout.
  - `tc_timeout`: acquire arena lock, overwrite `arena_obj.data` with a dirty sentinel, spin via nested `bpf_loop` until the timeout fires.  The undo-log rolls back the write.
- The lock pointer is obtained as `(struct bpf_spin_lock *)&arena_obj.lock`, which causes LLVM to emit `addr_space_cast` (AS1→AS0); the verifier sees `PTR_TO_ARENA`.

---

### 4. `tools/testing/selftests/bpf/prog_tests/arena_bpf_spin_lock.c` (new)

Userspace test runner.  Key design points:

- Defines `struct arena_obj` before `#include "test_arena_bpf_spin_lock.skel.h"` to resolve the incomplete-type error that arises because bpftool emits the BTF-derived field name without the struct definition.  The layout (`__u32 lock; __u32 _pad; __u64 data;`) matches the BPF-side struct exactly.
- Sysctl helpers are prefixed `absl_` to avoid colliding with `write_sysctl(const char *, const char *)` declared in `test_progs.h`.
- `test_basic`: loads skeleton, writes 0 to `arena_obj.data` (backs the arena page so `arena_obj.lock` is mapped and zeroed), runs `tc_basic`, asserts `arena_obj.data == 42`.
- `test_timeout_rollback`: saves sysctl, sets 5000 ms timeout, writes `ARENA_SENTINEL` to `arena_obj.data` (backing the page + setting known value), runs `tc_timeout`, asserts `arena_obj.data == ARENA_SENTINEL` after rollback.  Skips if sysctl is absent (timeout feature not compiled in).

---

## Runtime Flow

```
BPF program: bpf_spin_lock((struct bpf_spin_lock *)&arena_obj.lock)
  LLVM emits:  addr_space_cast R1, AS0←AS1  → R1 = PTR_TO_ARENA
  Verifier:    accepts PTR_TO_ARENA for ARG_PTR_TO_SPIN_LOCK
               sets spin_lock_arena_arg = true at call-site aux entry
  do_misc_fixups injects:
               R0 = kern_vm_start          (LD_IMM64)
               R1 += R0                    (ALU64 ADD)
               call bpf_spin_lock          (original instruction)
  bpf_spin_lock receives valid kernel VA of arena_obj.lock

Writes inside CS (e.g., arena_obj.data = X):
  convert_ctx_accesses:  arena store → BPF_PROBE_MEM32 mode
  do_misc_fixups:        in_critical_section=true + BPF_PROBE_MEM32 matched
                         → inject BPF_EMIT_CALL(bpf_undo_log_push) before store
  JIT:                   emit_undo_log_push_combined with is_arena=true
                         reads old value, records {kernel_VA, old_val, size}

Timeout fires:
  bpf_undo_log_replay:   restores old values from log (uses kernel VAs) ✓
  bpf_spin_unlock:       finds lock by kernel VA in held_locks[] ✓
```

---

## Known Limitations

- **AA deadlock detection unavailable for arena locks**: all `PTR_TO_ARENA`
  registers share `id = 0` (set by `mark_reg_unknown` in the
  `BPF_ADDR_SPACE_CAST` handler), making `find_lock_state` unusable.  The
  32-active-locks depth limit still applies.
- **`bpf_res_spin_lock` not supported**: resilient spin lock requires a BTF
  record; arena memory is untyped.
- **One arena per program**: the fixup uses `env->prog->aux->arena` which
  holds the single arena associated with the program.  Programs with multiple
  arena maps are not currently supported by the BPF subsystem regardless.
