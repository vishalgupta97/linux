# eBPF Spinlock Timeout Implementation

- [x] Explore codebase
- [x] Create Implementation Plan
- [x] Implement Kernel Changes
    - [x] Add per-CPU `ebpf_spinlock_timeout` variable
    - [x] Update `bpf_spin_lock` to reset timeout variable
    - [x] Implement new hrtimer callback to set timeout variable
    - [x] Update `bpf_spin_unlock` to reset timeout variable
    - [x] Modify `bpf_spin_lock_timeout_handler` (remove WARN/BUG, call `bpf_die`)
    - [x] Update `bpf_loop` to check timeout and call handler
    - [x] Update `bpf_iter_num_next` to check timeout
    - [x] Remove `BPF_MAX_LOOPS` limit check
- [x] Implement Selftest
    - [x] Create `test_spin_lock_loop_timeout.c`
- [x] Finalize
    - [x] Provide commands to build/test
