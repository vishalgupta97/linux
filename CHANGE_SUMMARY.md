# Change Summary

## Scope
This summary covers all currently modified files in the repository working tree.

## 1) Core eBPF undo-log implementation changes

### `kernel/bpf/verifier.c`
- Added verifier rejection for undo-logged writes smaller than 8 bytes in critical sections (regular store, atomic RMW, and immediate-store paths).
- Added rejection for arena + spin-lock undo-log combination (x86 undo-log optimization incompatibility).
- Kept existing critical-section write-count accounting and limits, but added the new width and arena constraints.

### `kernel/bpf/helpers.c`
- Refactored undo-log bookkeeping from **count-based** to **pointer-based** state:
  - Replaced `bpf_undo_log_cnt` usage with per-CPU current pointer `bpf_undo_log_cur`.
  - Replay now walks backward from current pointer to base.
- Removed per-entry size tracking from runtime behavior; helper now snapshots/restores 8-byte old values.
- Updated lock acquire/release reset points to reset pointer state instead of count.

### `include/linux/bpf.h`
- Added `struct bpf_undo_log_entry` declaration for shared visibility.
- Added per-CPU declarations for:
  - `bpf_undo_log`
  - `bpf_undo_log_cur`

### `arch/x86/net/bpf_jit_comp.c`
- Reworked undo-log marker expansion in x86 JIT to avoid helper-call register spill/restore sequence.
- Added direct emission path that:
  - Resolves per-CPU `bpf_undo_log_cur`.
  - Uses base+offset stores for logged address and old value.
  - Advances and writes back current undo-log pointer.
- Added local forward declarations required by function ordering in this file.