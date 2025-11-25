# Task: Add AA Deadlock Detection in eBPF Verifier

- [x] Research existing eBPF verifier lock handling <!-- id: 0 -->
# Task: Add AA Deadlock Detection in eBPF Verifier

- [x] Research existing eBPF verifier lock handling <!-- id: 0 -->
    - [x] Locate `kernel/bpf/verifier.c` and related files <!-- id: 1 -->
    - [x] Understand how `bpf_spin_lock` is currently verified <!-- id: 2 -->
    - [x] Understand how program type/context is determined <!-- id: 3 -->
- [x] Design Deadlock Detection Mechanism <!-- id: 4 -->
    - [x] Define data structure to track lock usage across programs <!-- id: 5 -->
    - [x] Determine where to hook into the verification process <!-- id: 6 -->
- [x] Implement Deadlock Detection <!-- id: 7 -->
    - [x] Add global lock registry <!-- id: 8 -->
    - [x] Implement check logic during verification <!-- id: 9 -->
    - [x] Implement registration logic on successful load <!-- id: 10 -->
- [x] Verification <!-- id: 11 -->
    - [x] Create test cases (Task context prog, NMI context prog, same lock) <!-- id: 12 -->
    - [x] Verify rejection of conflicting programs <!-- id: 13 -->
