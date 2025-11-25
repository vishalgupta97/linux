# AA Deadlock Detection Walkthrough

This document describes the implementation of AA deadlock detection in the eBPF verifier and how to verify it.

## Changes Implemented

### 1. Global Lock Registry
- Added `struct bpf_spin_lock_usage` to track lock usage (Map ID + Offset) and context (Task vs NMI).
- Added `bpf_spin_lock_registry` (global list) and `bpf_spin_lock_mutex` in `kernel/bpf/verifier.c`.

### 2. Lock Usage Tracking
- Modified `process_spin_lock` in `verifier.c` to record every `bpf_spin_lock` acquisition in `env->used_spin_locks`.
- Added `used_spin_locks` to `struct bpf_prog_aux` to persist usage information after verification.

### 3. Deadlock Detection Logic
- In `bpf_check` (upon successful verification), the verifier now checks `env->used_spin_locks` against the global `bpf_spin_lock_registry`.
- **Condition**: If a lock is used in both Task context and NMI context (by different programs), the load is rejected with `AA deadlock detected`.
- If no conflict is found, the new lock usage is added to the global registry.

### 4. Cleanup
- Implemented `bpf_free_used_spin_locks` to remove lock usage from the registry when a program is unloaded.
- Hooked this into `bpf_prog_free_deferred` in `kernel/bpf/core.c`.

### 5. NMI Context Support
- Relaxed the restriction in `verifier.c` to allow `BPF_PROG_TYPE_PERF_EVENT` (NMI context) to use `bpf_spin_lock`.

## Verification

### Test Case: `deadlock_test.c`

A C test case has been created to verify the detection logic.

#### Steps to Run
1.  Compile the kernel with the applied changes.
2.  Compile `deadlock_test.c` against `libbpf`.
3.  Run `./deadlock_test`.

#### Expected Result
- The test attempts to load two programs:
    1.  `prog_task` (Task context) -> Uses `my_map` lock.
    2.  `prog_nmi` (NMI context) -> Uses `my_map` lock.
- The first load should succeed.
- The second load should **FAIL** with an error indicating the deadlock.
- The test program prints "Load failed as expected!" if the mechanism works.

### Manual Verification
You can also verify by checking the kernel log (`dmesg`) after running the test. You should see:
```
AA deadlock detected: lock used in Task and NMI contexts
```
