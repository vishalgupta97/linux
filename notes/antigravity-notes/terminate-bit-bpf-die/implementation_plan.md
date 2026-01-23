# eBPF Spinlock Timeout Implementation

This plan implements a mechanism to terminate eBPF programs if a spinlock timeout occurs, specifically targeting loops (`bpf_loop` and `bpf_for`) that might hold locks for too long.

## Proposed Changes

### Kernel

#### `kernel/bpf/helpers.c`
*   **Add per-CPU variable**: `DEFINE_PER_CPU(int, ebpf_spinlock_timeout);`
*   **Modify `bpf_spin_lock`**:
    *   Reset `ebpf_spinlock_timeout` to 0.
    *   Change hrtimer callback to a new function `bpf_spin_lock_timer_cb` which strictly sets the timeout variable.
*   **Implement `bpf_spin_lock_timer_cb`**:
    *   Sets `this_cpu_write(ebpf_spinlock_timeout, 1)`.
*   **Modify `bpf_spin_lock_timeout_handler`**:
    *   Expose this function (remove static or add friend declaration).
    *   Remove `WARN_ON_ONCE` and `BUG_ON`.
    *   Call `bpf_die(NULL)` (or appropriate arg) at the end. *Refined: bpf_die takes prog, need to retrieve it or handle context.*
    *   *Correction*: `bpf_die` requires `struct bpf_prog *`. The timeout handler might not have direct access if called asynchronously, but if called from `bpf_loop` (synchronous context), current task/prog is running.
    *   We will assume `bpf_die(current->bpf_ctx)` or similar if reachable, or just `bpf_die(NULL)` if it handles it (it doesn't).
    *   Actually, `bpf_die` uses `prog->bpf_func`. We need the current program.
    *   *Alternative*: We can get the program from `bpf_loop` context or `current`.
*   **Modify `bpf_spin_unlock`**:
    *   Reset `ebpf_spinlock_timeout` to 0.

#### `kernel/bpf/bpf_iter.c`
*   **Modify `BPF_CALL_4(bpf_loop, ...)`**:
    *   Update `BPF_MAX_LOOPS` check to allow `UINT64_MAX` (or remove check).
    *   In the loop check `this_cpu_read(ebpf_spinlock_timeout)`.
    *   If set, call `bpf_spin_lock_timeout_handler` (need to export/expose).
*   **Modify `bpf_iter_num_next` (for `bpf_for`)**:
    *   Add similar check for `ebpf_spinlock_timeout`.

#### `include/linux/bpf_verifier.h` or `kernel/bpf/bpf.h` (Internal)
*   Expose `bpf_spin_lock_timeout_handler`.

### Verification Plan

#### Automated Tests
*   Create a new test file `tools/testing/selftests/bpf/progs/test_spin_lock_loop_timeout.c`.
*   The test will:
    *   Acquire a spinlock.
    *   Enter a `bpf_loop` that runs for a long time (simulating hang/timeout).
    *   Expect the program to be terminated (not complete successfully).
    *   We can use `test_progs` runner to verify it returns error or is killed.

#### Manual Verification
*   Compile kernel: `make -j$(nproc)`
*   Compile tests: `make -C tools/testing/selftests/bpf`
*   Run specific test: `./test_progs -t spin_lock_loop_timeout`
