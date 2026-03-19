# BPF Spinlock Undo Log – Change Summary

Date: 2026-03-01  
Branch: `copilot-opus-undo-log`  
Base: `bpf-next`

---

## Overview

This change adds a **per-CPU undo log** for BPF spinlock critical sections.
Every memory write made inside a `bpf_spin_lock` / `bpf_spin_unlock` pair is
recorded before it happens.  When the spinlock timeout watchdog fires
(`bpf_spin_lock_timeout_handler`), the log is replayed in reverse order to
restore all modified locations to their original values before the locks are
released.  This prevents other CPUs from ever observing a partially-written
shared data structure after an aborted critical section.

---

## Motivation

The pre-existing spinlock timeout mechanism (`bpf_spin_lock_timeout_handler`)
forcibly releases all held BPF spinlocks when a program exceeds the configured
timeout (`/proc/sys/net/core/bpf_spin_lock_timeout`).  Without this change,
any writes already committed to shared maps during the timed-out critical
section would remain visible, leaving data in an inconsistent state.

The undo log provides atomicity-on-abort: either all writes in a critical
section become visible (normal `bpf_spin_unlock`) or none do (timeout).

---

## Files Changed

### `kernel/bpf/Kconfig`

Added new compile-time tunable:

```kconfig
config BPF_UNDO_LOG_MAX_ENTRIES
    int "Maximum undo log entries per BPF spinlock critical section"
    default 64
    depends on BPF_SYSCALL
    range 1 1024
```

- Controls the per-CPU undo log array size.
- Any BPF program whose critical section contains **more** write instructions
  than this limit on any execution path is **rejected by the verifier** at load
  time with `-E2BIG`.
- Valid range: 1–1024.  Default: 64.

---

### `include/linux/bpf.h`

Added the declaration for the internal helper injected by the verifier:

```c
u64 bpf_undo_log_push(u64 addr, u64 size, u64 r3, u64 r4, u64 r5);
```

This function is **not** callable from BPF programs directly; it is emitted
as a `BPF_EMIT_CALL` injection by `do_misc_fixups()` in the verifier.

---

### `include/linux/bpf_verifier.h`

Three additions:

1. **`u32 cs_write_count`** in `struct bpf_verifier_state`  
   Counts writes tracked so far on the current path inside a spinlock CS.
   Reset to 0 on the `active_locks` 0→1 transition (first lock) and again
   on the back-to-0 transition (last unlock).  Used to enforce the
   `CONFIG_BPF_UNDO_LOG_MAX_ENTRIES` limit at verify time.

2. **`bool in_critical_section`** in `struct bpf_insn_aux_data`  
   Set on any write instruction (STX, ST-immediate, atomic RMW) that the
   verifier determines lies inside a spinlock critical section on all paths
   that reach it.  Consumed by `do_misc_fixups()` to decide which
   instructions need the undo-log prefix.

3. **`#define BPF_UNDO_LOG_SPILL_SIZE  48`**  
   Stack space (6 registers × 8 bytes) reserved per-subprogram by
   `do_misc_fixups()` to hold the R0–R5 spill/restore code wrapped around
   each `bpf_undo_log_push` call.

---

### `include/linux/filter.h`

Added the two-argument variant of the notrace BPF helper macro:

```c
#define NOTRACE_BPF_CALL_2(name, ...)  BPF_CALL_x(2, notrace, name, __VA_ARGS__)
```

Used to declare `bpf_undo_log_push` as a notrace function (required because
it runs with IRQs disabled inside a spinlock).

---

### `kernel/bpf/helpers.c`

#### New data structures

```c
struct bpf_undo_log_entry {
    void *addr;       /* write destination */
    u64   old_value;  /* value before the write */
    u8    size;       /* 1, 2, 4, or 8 bytes */
};

DEFINE_PER_CPU(struct bpf_undo_log_entry[CONFIG_BPF_UNDO_LOG_MAX_ENTRIES],
               bpf_undo_log);
DEFINE_PER_CPU(int, bpf_undo_log_cnt);
```

#### `bpf_undo_log_push` (new helper)

```c
NOTRACE_BPF_CALL_2(bpf_undo_log_push, unsigned long addr, u64 size)
```

- Reads the current value at `addr` (1/2/4/8 bytes per `size`).
- Appends `(addr, old_value, size)` to the per-CPU log.
- Returns 0 on success, `-ENOSPC`/`-EINVAL` on error (defensive; verifier
  guarantees these are never hit in practice).
- Exported as `EXPORT_SYMBOL_GPL`.

#### `bpf_undo_log_replay` (new static helper)

```c
static void bpf_undo_log_replay(void)
```

- Iterates the per-CPU log **in reverse** (entry N-1 down to 0).
- Restores each entry using `WRITE_ONCE`, correctly handling the case where
  the same address was written multiple times (most-recent write undone first,
  eventually recovering the original value).
- Resets `bpf_undo_log_cnt` to 0 afterward.

#### Modifications to `bpf_spin_lock_timeout_handler`

Calls `bpf_undo_log_replay()` **before** releasing any held locks so that
other CPUs never observe the inconsistent state.

#### Modifications to `bpf_spin_lock`

Resets `bpf_undo_log_cnt = 0` when `active_locks` transitions from 0 to 1
(i.e., on the very first lock acquisition), giving each critical section a
clean log.

#### Modifications to `bpf_spin_unlock`

Resets `bpf_undo_log_cnt = 0` when the last lock is released via the normal
path, discarding the committed writes (no rollback needed).

---

### `kernel/bpf/verifier.c`

Six separate modifications:

#### 1. `check_store_reg()` — STX write tracking

After a successful `check_mem_access(BPF_WRITE)`, if a lock is held and the
destination is not `PTR_TO_STACK`:

```c
if (env->cur_state->active_locks > 0 &&
    base_type(dst_reg_type) != PTR_TO_STACK) {
    if (env->cur_state->cs_write_count >= CONFIG_BPF_UNDO_LOG_MAX_ENTRIES)
        // verbose error + return -E2BIG
    env->insn_aux_data[env->insn_idx].in_critical_section = true;
    env->cur_state->cs_write_count++;
}
```

Stack writes are excluded: they are private to the program invocation and
do not affect shared state visible to other CPUs.

Context register (`PTR_TO_CTX`) writes are also excluded (classified as
`PTR_TO_STACK` equivalent in BPF context; inaccessible via the undo log
address).

#### 2. `check_atomic_rmw()` — atomic write tracking

Same guard added after the atomic write's `check_mem_access`, covering
`BPF_ADD`, `BPF_AND`, `BPF_OR`, `BPF_XOR`, `BPF_XCHG`, and `BPF_CMPXCHG`.

#### 3. `BPF_ST` handler in `do_check()` — immediate store tracking

Same guard added for `BPF_ST` class instructions (immediate-to-memory stores
like `*(u32 *)(r1+8) = 0`).

#### 4. `process_spin_lock()`

- On lock: resets `cs_write_count = 0` when `active_locks` transitions 0→1.
- On unlock: resets `cs_write_count = 0` when `active_locks` returns to 0.

#### 5. `states_equal()`

Added a merge constraint:

```c
if (old->cs_write_count < cur->cs_write_count)
    return false;
```

Prevents the verifier from merging a state that has seen fewer CS writes into
one that has seen more, which would cause the injection loop in
`do_misc_fixups()` to miss writes on paths that follow the merge.

#### 6. `do_misc_fixups()` — undo-log injection

**Pre-pass** (per-subprogram):  
For each subprogram that contains at least one `in_critical_section` write,
extends the frame size by `BPF_UNDO_LOG_SPILL_SIZE` (48 bytes) to hold the
R0–R5 spill slots.  Rejects with an error if the resulting stack would exceed
`MAX_BPF_STACK`.

**Per-instruction injection**:  
For every instruction with `insn_aux_data[i].in_critical_section == true`,
the rewrite now uses one of two paths:

- **x86 JIT no-spill path** (`prog->jit_requested` and JIT advertises
  `bpf_jit_supports_undo_log_nospill()`):

```
CALL bpf_undo_log_push          ; marker call, no BPF arg setup
<original STX / ST / atomic>    ; unchanged write
```

  The x86 JIT recognizes this marker and derives `(addr, size)` from the
  following write instruction while preserving BPF R0-R5 in native code.

- **generic fallback path** (unchanged behavior):

```
; --- spill R0-R5 to reserved stack slots ---
STX  FP[-usd+ 0]  R0
STX  FP[-usd+ 8]  R1
STX  FP[-usd+16]  R2
STX  FP[-usd+24]  R3
STX  FP[-usd+32]  R4
STX  FP[-usd+40]  R5
; --- load arguments for bpf_undo_log_push ---
MOV  R1, <dst_reg>
ADD  R1, <insn_off>      ; effective address
MOV  R2, <write_size>    ; 1 / 2 / 4 / 8
CALL bpf_undo_log_push
; --- restore R0-R5 ---
LDX  R0  FP[-usd+ 0]
LDX  R1  FP[-usd+ 8]
LDX  R2  FP[-usd+16]
LDX  R3  FP[-usd+24]
LDX  R4  FP[-usd+32]
LDX  R5  FP[-usd+40]
; --- original write instruction (unchanged) ---
<original STX / ST / atomic>
```

The `delta` tracking already present in `do_misc_fixups()` correctly accounts
for the inserted instructions so that branch/call offsets are patched up.

---

## Test Cases

Two new selftest files (no Makefile changes needed; auto-discovered):

### `tools/testing/selftests/bpf/progs/test_bpf_undo_log.c`

BPF-side programs organised into the groups below.  Programs requiring
non-ADD atomics are guarded by `#ifdef ENABLE_ATOMICS_TESTS` for cpuv2
(`no_alu32` build) compatibility.

| Test | Description |
|------|-------------|
| TC01–TC04 | Single STX write of each width (u64/u32/u16/u8) inside a CS with a timeout trigger; expect rollback |
| TC05 | All four widths written in one CS; all rolled back |
| TC06 | Atomic ADD u64; rolled back |
| TC07–TC11 | Non-ADD atomics: AND, OR, XOR, XCHG (u64), mixed; rolled back (cpuv3+ only) |
| TC12 | Mix of STX and atomic ADD; all rolled back |
| TC17–TC18 | Writes via an array map used as a kobj store; rolled back |
| TC20–TC22 | Same address written 2–5× in one CS; log replays in reverse, restoring the first snapshot |
| TC23–TC26 | Nested lock pairs: both locks / inner-only / outer-only / interleaved writes; all rolled back |
| TC27–TC29 | Normal `bpf_spin_unlock` (no timeout); written values **committed** |
| TC30 | Exactly `CONFIG_BPF_UNDO_LOG_MAX_ENTRIES` (64) writes — verifier **accepts** |
| TC31 | 65 writes — verifier **rejects** (`SEC("?tc") __failure __msg("BPF critical section write count")`) |
| TC35 | Zero writes in CS — loads and runs cleanly |
| TC36 | Stack write inside CS excluded from undo log; only the map write is tracked |
| TC37 | BPF_ST (immediate) write inside CS; rolled back |
| TC38 | Nested locks share the single log; both writes rolled back |
| TC39 | Two sequential CS sessions: first committed, second rolled back |

### `tools/testing/selftests/bpf/prog_tests/bpf_undo_log.c`

Userspace runner (`void test_bpf_undo_log()`).

- Sets sysctl `bpf_spin_lock_timeout` to 50 ms before the rollback tests;
  saves and restores the previous value.
- For each rollback test: pre-fills the relevant map entry with a known
  sentinel, runs the BPF program via `bpf_prog_test_run_opts`, reads back
  the map entry, and asserts the value matches the original sentinel.
- For commit tests (TC27–TC29): sets sysctl to 0 (timeout disabled) and
  asserts the new values are present.
- For the verifier rejection test (TC31): opens a fresh skeleton with a
  kernel log buffer, attempts to load, and checks either that the load
  failed or that the loaded program fd is invalid, plus asserts the
  expected error string is present in the log.
- Advanced atomic tests (TC07–TC11) are conditionally compiled with
  `#ifdef ENABLE_ATOMICS_TESTS` to match the BPF program guards.

---

## Design Decisions

| Decision | Rationale |
|----------|-----------|
| Single shared log for nested locks | Simplest implementation; nested locks share one timeline of writes, so a single reverse-replay restores all of them correctly |
| Log cleared on **first** lock, not every lock | Outer-lock writes must be in the log too; clearing on every `bpf_spin_lock` would lose writes made between the outer and inner lock |
| Log cleared on **last** unlock (commit path) | Once all locks are released normally the writes are durable; keeping stale entries would contaminate the next CS |
| `PTR_TO_STACK` writes excluded | Stack is per-invocation and not shared across CPUs; rolling it back would break program semantics |
| Injection via `do_misc_fixups()` not JIT | Architecture-neutral; works for all supported BPF JIT backends without per-arch changes |
| `cs_write_count` merge constraint in `states_equal()` | Prevents the verifier from merging states with different write counts, which would cause the injection loop to miss writes on later paths |
| `notrace` on `bpf_undo_log_push` | The function runs inside a spinlock with IRQs disabled; tracing would deadlock |

---

## Limitations / Future Work

- The undo log does not cover writes through `bpf_probe_write_user` or
  `bpf_dynptr` (not yet analysed for CS membership by the verifier).
- Arena (PROBE_MEM32) pointer writes are marked `in_critical_section` by
  the same verifier paths but the undo-log helper address arithmetic
  currently assumes a direct pointer dereference; arena accesses may need
  a separate code path.
- The per-CPU log arrays are statically allocated at boot based on
  `CONFIG_BPF_UNDO_LOG_MAX_ENTRIES`; a dynamic allocation strategy would
  reduce memory on systems that run BPF programs with small CSes.
