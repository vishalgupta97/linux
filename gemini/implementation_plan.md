# AA Deadlock Detection in eBPF Verifier

The goal is to prevent AA deadlocks where the same lock is acquired in both Task and NMI contexts by eBPF programs. This requires tracking lock usage across all loaded programs and enforcing that a lock used in Task context cannot be used in NMI context, and vice versa.

## User Review Required

> [!IMPORTANT]
> This change enables `bpf_spin_lock` for `BPF_PROG_TYPE_PERF_EVENT` programs, which were previously restricted. This is necessary to allow NMI context programs to use locks, subject to the new deadlock detection.

## Proposed Changes

### Kernel Core

#### [MODIFY] [bpf.h](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/include/linux/bpf.h)
- Add `struct list_head used_spin_locks;` to `struct bpf_prog_aux` to track locks used by a program for cleanup.

#### [MODIFY] [core.c](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/kernel/bpf/core.c)
- In `bpf_prog_free_deferred`, call `bpf_free_used_spin_locks(aux)` to release lock usage.

### BPF Verifier

#### [MODIFY] [bpf_verifier.h](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/include/linux/bpf_verifier.h)
- Add `struct list_head used_spin_locks;` to `struct bpf_verifier_env`.
- Declare `void bpf_free_used_spin_locks(struct bpf_prog_aux *aux);`.

#### [MODIFY] [verifier.c](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/kernel/bpf/verifier.c)
- Define `struct bpf_spin_lock_usage` for the global registry.
- Define `struct bpf_verifier_lock_usage` for local tracking in `bpf_verifier_env`.
- Add global `bpf_spin_lock_registry` list and `bpf_spin_lock_mutex`.
- In `process_spin_lock`, record the lock (map_id + offset) in `env->used_spin_locks`.
- In `bpf_check` (after verification success):
    - Check for conflicts against the global registry.
    - If valid, register the new program's lock usage.
    - Copy used locks to `prog->aux->used_spin_locks`.
- Implement `bpf_free_used_spin_locks` to remove lock usage from the registry.
- Relax the check in `check_map_access` (or `check_helper_call` path) to allow `bpf_spin_lock` for `BPF_PROG_TYPE_PERF_EVENT`.

## Verification Plan

### Automated Tests
- Create a C test file (using libbpf if available, or raw syscalls) to:
    1. Load a program of type `BPF_PROG_TYPE_SCHED_CLS` (Task context) that uses a specific map lock.
    2. Attempt to load a program of type `BPF_PROG_TYPE_PERF_EVENT` (NMI context) that uses the **same** map lock.
    3. Assert that the second load fails with a specific error message (e.g., "AA deadlock detected").
    4. Verify that loading an NMI program with a **different** lock succeeds.
    5. Verify that unloading the Task program allows the NMI program to load (if implemented).

### Manual Verification
- Compile the kernel with changes.
- Run the test case.
