# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Repository Is

This is a modified Linux kernel (v7.0, branch `v7.0-timeout-undo-log`) focused on a custom BPF spinlock timeout + undo-log mechanism. It is **not** a standard kernel tree — it contains experimental BPF infrastructure built on top of `bpf-next`.

The primary research goal: guarantee atomicity-on-abort for BPF spinlock critical sections. When a BPF program holding a spinlock exceeds the timeout (`/proc/sys/net/core/bpf_spin_lock_timeout` in ms), all writes made inside the critical section are rolled back via a per-CPU undo log before locks are released and the program is terminated via `bpf_die()`.

## Build Commands

```bash
# Build the kernel image (use the project config)
make KCONFIG_CONFIG=bpftest-config -j100 bzImage

# Build BPF selftests (from kernel root)
make -j100 -C tools/testing/selftests/bpf test_progs

# Run a single BPF selftest
./tools/testing/selftests/bpf/test_progs -t spin_lock_timeout
./tools/testing/selftests/bpf/test_progs -t spin_lock_loop_timeout
./tools/testing/selftests/bpf/test_progs -t spin_lock

# Run all BPF selftests
./tools/testing/selftests/bpf/test_progs
```

### Load-check a new BPF program before running it

After building, **always do a verifier load-check before running a newly written or
modified BPF program**. Loading a program runs it through the verifier (and the JIT)
without executing it, so it surfaces verifier rejections, undo-log marker / JIT issues,
and bad lock nesting cheaply and without the risk of a buggy program hanging or
spinning the machine at run time.

```bash
# Load every program in the compiled object through the verifier (does NOT run them).
# The .bpf.o is produced by the test_progs build above.
cd tools/testing/selftests/bpf
sudo ./tools/sbin/bpftool prog loadall <name>.bpf.o /sys/fs/bpf/<name>
# exit status 0 == all programs passed the verifier
sudo ./tools/sbin/bpftool prog show pinned /sys/fs/bpf/<name>/<prog>   # inspect xlated/jited
sudo rm -rf /sys/fs/bpf/<name>                                         # clean up the pins
```

Only after the load-check passes should you run the program via `test_progs`.

The build config is `bpftest-config` (not `.config`). Key config options relevant to this work:
- `CONFIG_BPF_UNDO_LOG=y`, `CONFIG_BPF_UNDO_LOG_MAX_ENTRIES=64`
- `CONFIG_QUEUED_SPINLOCKS=y` (required for the custom qspinlock path)

## Architecture of the BPF Spinlock Timeout + Undo Log System

### Conceptual flow

```
BPF program acquires lock
  → writes inside critical section are preceded by undo-log marker instructions (injected by verifier)
  → x86 JIT recognizes markers and emits inline undo-log entry writes using R12 as cursor
  → timeout hrtimer is started (by waiter or kthread)
  → if timeout fires: ebpf_spinlock_timeout flag is set
  → bpf_loop / bpf_iter_num_next checks the flag each iteration
  → calls bpf_spin_lock_timeout_handler() → undo-log replay → lock release → bpf_die()
```

### Key files

| File | Role |
|------|------|
| `kernel/bpf/helpers.c` | `bpf_spin_lock` / `bpf_spin_unlock` helpers; per-CPU `held_locks[]`; undo-log storage (`bpf_undo_log`, `bpf_undo_log_cnt`); `bpf_spin_lock_timeout_handler()`; kthread for uncontended timeout |
| `kernel/bpf/bpf_qspinlock.c` | Custom BPF qspinlock slow path: acquires with IRQs *enabled* so hrtimer can fire on the waiting CPU; per-CPU waiter hrtimers; `bpf_active_timer` per-CPU pointer |
| `kernel/bpf/rqspinlock.c` | Resilient qspinlock (upstream `bpf-next` path, separate from bpf_qspinlock) — not the active lock path for timeout work |
| `arch/x86/net/bpf_jit_comp.c` | Marker detection (`bpf_insn_is_undo_log_marker`); inline undo-log emission (`emit_undo_log_push_call`) using R12 as cursor; R12 push/pop in prologue/epilogue gated on `use_r12` |
| `kernel/bpf/verifier.c` | `do_misc_fixups()`: pre-scan to detect CS writes → sets `prog->aux->undo_log_requires_jit`; injects `bpf_undo_log_push` marker before each CS write; rejects arena + undo-log combination |
| `kernel/bpf/core.c` | `bpf_prog_select_runtime()`: forces JIT when `undo_log_requires_jit` is set |
| `kernel/bpf/bpf_iter.c` | `bpf_loop()` and `bpf_iter_num_next()`: check `ebpf_spinlock_timeout` per iteration and call timeout handler |
| `include/linux/bpf.h` | `struct bpf_undo_log_entry`; `undo_log_requires_jit` in `bpf_prog_aux`; extern declarations for per-CPU undo log |
| `include/linux/bpf_qspinlock.h` | Declaration of `bpf_qspinlock_lock` / `bpf_qspinlock_unlock` |
| `include/linux/bpf_lock_timer.h` | `bpf_lock_timer_ops` abstraction (start/cancel); `bpf_lock_timer` struct |

### Undo log mechanics

- The verifier (`do_misc_fixups`) injects a `BPF_EMIT_CALL(bpf_undo_log_push)` marker before every write instruction inside a spinlock critical section.
- The x86 JIT (`bpf_insn_is_undo_log_marker`) recognizes this call and replaces it with inline code: load old value by width, store `{addr, old_value, size}` into the current undo-log entry at the R12 cursor, then advance R12 by `sizeof(struct bpf_undo_log_entry)`.
- R12 is saved/restored in the JIT prologue/epilogue when `use_r12 = has_undo_log_markers || arena_vm_start`.
- On timeout, `bpf_undo_log_replay()` in helpers.c iterates the log in reverse and restores old values.
- Arena + undo-log combination is rejected by the verifier (R12 conflict).
- Programs with CS writes that require undo logging cannot run in the interpreter (`undo_log_requires_jit = true`).

### Lock timeout path (two cases)

**Contended** (waiter exists): `bpf_qspinlock.c` slow path — the waiter at the head of the MCS queue starts the per-CPU hrtimer with IRQs enabled. The timer fires, sets `ebpf_spinlock_timeout`, the owner's `bpf_loop` detects it and releases. The waiter then cancels the timer and acquires.

**Uncontended** (fast path): `bpf_qspinlock_lock()` notifies the kthread (`bpf_notify_lock_kthread()`), which starts the shared `bpf_kthread_hrtimer`. On unlock, the active timer pointer (`bpf_active_timer`) is used to cancel whichever timer is running.

### BPF selftests for this work

Located in `tools/testing/selftests/bpf/`:
- `prog_tests/spin_lock_loop_timeout.c` — verifies timeout fires and terminates a `bpf_loop` holding a lock
- `prog_tests/spin_lock_timeout.c` — nested locking, out-of-order unlock, timeout trigger, AB-BA deadlock
- `progs/test_spin_lock_loop_timeout.c` — BPF-side program for the loop timeout test
- `progs/test_bpf_undo_log.c`, `progs/test_minimal_bpf_undo_log.c` — undo-log specific BPF programs

The timeout value for tests is controlled by writing to `/proc/sys/net/core/bpf_spin_lock_timeout` (milliseconds).
