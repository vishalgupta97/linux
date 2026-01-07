# eBPF Spin Lock Timeout and Cancellation

The goal is to prevent deadlocks and indefinite spins in eBPF programs by introducing a timeout mechanism for `bpf_spin_lock`. This involves adding a sysctl knob, tracking held locks at runtime, and implementing a cancellation mechanism that releases locks and aborts the program if a timeout occurs.

## User Review Required

> [!IMPORTANT]
> This implementation introduces a `bp_spin_lock` timeout check inside the spin loop. This changes the behavior of `bpf_spin_lock` from an indefinite spin (or raw hardware spin) to a timed spin.
> The default timeout will be 0 (disabled) to preserve existing behavior unless configured via `sysctl_bpf_spin_lock_timeout`.

> [!WARNING]
> Cancellation uses `bpf_throw(0)`. Since `bpf_throw` generally handles resource cleanup for verifier-tracked objects, but our dynamic lock tracking `held_locks` is custom, we must manually release locks from our `held_locks` list in the timeout handler *before* calling `bpf_throw`. This ensures all nested locks are released.

## Proposed Changes

### Kernel Core

#### [MODIFY] [syscall.c](file:///home/vigupta/project/linux/kernel/bpf/syscall.c)
- Add `sysctl_bpf_spin_lock_timeout` integer variable.
- Register it via `register_sysctl` (or ensure it's exported if defined elsewhere).

#### [MODIFY] [helpers.c](file:///home/vigupta/project/linux/kernel/bpf/helpers.c)
- Define per-CPU data structures:
  ```c
  struct bpf_lock_entry {
      struct bpf_spin_lock *lock;
  };
  #define MAX_HELD_LOCKS 32
  DEFINE_PER_CPU(struct bpf_lock_entry[MAX_HELD_LOCKS], held_locks);
  DEFINE_PER_CPU(int, held_locks_cnt);
  DEFINE_PER_CPU(struct hrtimer, lock_watchdog_timer); // For Critical Section timeout
  ```
- Modify `__bpf_spin_lock`:
  - Check `sysctl_bpf_spin_lock_timeout`.
  - **Acquisition Phase**:
    - If `held_locks_cnt == 0`, determine deadline.
    - Spin wait with timeout check (as before).
    - If timeout -> Cleanup & Throw.
  - **Critical Section Phase** (On Acquisition Success):
    - Add lock to `held_locks`.
    - If `held_locks_cnt == 1` (Outermost lock):
       - Start `lock_watchdog_timer` (hrtimer) with `sysctl` timeout.

#### [NEW] [bpf_spin_lock_timeout_handler] (in `helpers.c`)
- **Location**: Defined in `kernel/bpf/helpers.c` to have access to `held_locks`.
- **Context**: Runs in hard IRQ context (hrtimer callback).
- **Logic**:
  1. Access per-CPU `held_locks`.
  2. Iterate and release all held locks (`arch_spin_unlock`).
  3. Reset `held_locks_cnt`.
  4. **Cancellation**: Invoke `bpf_throw(0)` (or suitable low-level stack unwinder/trampoline injection if direct call is unsafe from IRQ).

- Modify `__bpf_spin_unlock`:

  - Call `arch_spin_unlock`.
  - Remove lock from `held_locks`.
  - If `held_locks_cnt == 0` (Outermost unlock):
    - Cancel `lock_watchdog_timer`.


### Selftests

#### [NEW] [progs/test_spin_lock_timeout.c](file:///home/vigupta/project/linux/tools/testing/selftests/bpf/progs/test_spin_lock_timeout.c)
- BPF program with:
  - Nested locking (A -> B).
  - OOO unlocking (Lock A, Lock B, Unlock A, Unlock B).
  - **Timeout triggers**:
    - Use `bpf_for` loop inside critical section to exhaust the watchdog timer.
    - Check if program is cancelled/unwinds.
- **AB-BA Deadlock**:
  - Two BPF programs running in parallel (e.g., attached to different hooks or same hook triggered concurrently).
  - Prog 1: Lock A, Delay, Lock B.
  - Prog 2: Lock B, Delay, Lock A.
  - Verify one or both get cancelled.

#### [NEW] [prog_tests/spin_lock_timeout.c](file:///home/vigupta/project/linux/tools/testing/selftests/bpf/prog_tests/spin_lock_timeout.c)
- Userspace runner to:
  - Set `sysctl_bpf_spin_lock_timeout`.
  - Load and attach BPF programs.
  - Trigger events.
  - Assert that "Deadlock" case results in cancellation (error return or exception).
  - Assert that "Valid" cases pass.

## Verification Plan

### Automated Tests
Run the new selftest:
```bash
cd tools/testing/selftests/bpf
./test_progs -t spin_lock_timeout
```
