# Arena + Undo-Log Unification: Change Summary

## Problem

Arena maps and the spinlock undo-log both used R12 as their dedicated register:
- **Arena**: R12 = `kern_vm_start` (fixed 64-bit base, initialized once in prologue)
- **Undo-log**: R12 = per-CPU cursor (advances by 24 bytes per CS write)

Both uses were mutually exclusive. The verifier and JIT rejected any program
that used both, returning `-EOPNOTSUPP`.

## Solution: 4 GB Address-Space Partition

The 4 GB arena window is split in half:
```
R12 = kern_vm_start  (unchanged)

[R12 + 0,    R12 + 2 GB):   ARENA pages  (bit 31 = 0)
[R12 + 2 GB, R12 + 4 GB):   UNDO-LOG pages (bit 31 = 1, one page per CPU)
```

For combined programs R12 stays fixed (`kern_vm_start`). Each undo-log push
temporarily borrows R12 as cursor via `push %r12` / `pop %r12`, trading
~5 extra instructions per CS write for zero spill/reload around calls.

---

## Files Changed

### `arch/x86/net/bpf_jit_comp.c`

**Removed** the 3-line rejection block that returned `-EOPNOTSUPP` when both
`has_undo_log_markers` and `arena_vm_start` were set.

**Added** `emit_undo_log_push_combined()` — the new inline emission helper for
combined programs. Instead of advancing R12 directly, it:
1. Computes `r11 = &target` (effective address of the CS write)
2. `push %r12` to save `vm_start`
3. `movq %gs:bpf_undo_log_cursor, %r12` to load the cursor
4. Writes `{addr, old_value, size}` into the undo-log entry at `[r12]`
5. `addq $24, %r12` to advance the cursor
6. `movq %r12, %gs:bpf_undo_log_cursor` to store the updated cursor
7. `pop %r12` to restore `vm_start`

**Updated** the undo-log marker handler (`bpf_insn_is_undo_log_marker`) to
dispatch based on mode: calls `emit_undo_log_push_combined` when
`arena_vm_start != 0`, otherwise uses the existing R12-cursor path.

**Fixed** `BPF_JMP | BPF_CALL` spill/reload: the R12 spill/reload
around calls is now conditioned on `has_undo_log_markers && !arena_vm_start`.
Combined programs skip the spill/reload because R12 = `vm_start` is a
callee-saved constant. Also fixed the `ip` accounting: `ip += 9` is only
emitted for undo-log-only programs (where the 9-byte spill shifts the
`callq` position).

**Bug fix** (from previous session): Added `if (has_undo_log_markers) ip += 9`
after `emit_undo_log_spill_r12` to fix a kernel panic caused by `emit_call`
computing a wrong relative displacement when the spill preceded the call.

### `include/linux/bpf.h`

Added `DECLARE_PER_CPU(struct bpf_undo_log_entry *, bpf_undo_log_base)` inside
`#ifdef CONFIG_BPF_UNDO_LOG`. This new per-CPU variable holds the base address
of the undo-log for the current session:
- **Undo-log-only programs**: points to the static kernel per-CPU array
  (`this_cpu_ptr(bpf_undo_log)`)
- **Combined arena+undo-log programs**: points to the arena-mapped page at
  `kern_vm_start + SZ_2G + cpu * PAGE_SIZE`

This allows `bpf_spin_lock` / `bpf_spin_unlock` to reset the cursor correctly
regardless of which storage backend is active.

### `kernel/bpf/helpers.c`

**Added** `DEFINE_PER_CPU(struct bpf_undo_log_entry *, bpf_undo_log_base)` and
its `EXPORT_PER_CPU_SYMBOL_GPL`.

**Updated** `bpf_undo_log_init`: initializes both `bpf_undo_log_cursor` and
`bpf_undo_log_base` to `this_cpu_ptr(bpf_undo_log)` at boot.

**Updated** `bpf_undo_log_replay`: uses `bpf_undo_log_base` as the base for
replay iteration (not the hardcoded static array), so that arena-mapped entries
are replayed correctly.

**Updated** cursor-reset calls in `__internal__bpf_spin_lock` and
`__internal__bpf_spin_unlock` to use `this_cpu_read(bpf_undo_log_base)` instead
of `this_cpu_ptr(bpf_undo_log)`.

### `kernel/bpf/arena.c`

**Restricted** arena max size: changed `vm_range > SZ_4G` to `vm_range > SZ_2G`,
returning `-E2BIG`. The upper 2 GB of the 4 GB window is reserved for undo-log
pages.

**Added** `struct page **undo_log_pages` field to `struct bpf_arena` (under
`#ifdef CONFIG_BPF_UNDO_LOG`) for lifetime management of the pre-allocated
undo-log physical pages.

**Added** `bpf_arena_alloc_undo_log_pages(arena)`: called from `arena_map_alloc`
after page-table setup. For each possible CPU it:
1. Allocates one physical page via `alloc_page(GFP_KERNEL | __GFP_ZERO)`
2. Maps it at `kern_vm_start + SZ_2G + cpu * PAGE_SIZE` via `apply_to_page_range`
3. Sets `per_cpu(bpf_undo_log_base, cpu)` and `per_cpu(bpf_undo_log_cursor, cpu)`
   to the mapped address

The `#else` stub (no `CONFIG_BPF_UNDO_LOG`) returns 0 immediately.

**Added** `bpf_arena_free_undo_log_pages(arena)`: called from `arena_map_free`
before the VM area is torn down. Resets per-CPU base/cursor to the static kernel
arrays. Physical pages are freed by the subsequent `apply_to_existing_page_range`
call in `arena_map_free` (which covers the entire arena kernel VM range including
the upper 2 GB undo-log area).

**Added** a forward declaration of `existing_page_cb` before the
`#ifdef CONFIG_BPF_UNDO_LOG` block to allow `bpf_arena_alloc_undo_log_pages`'s
error-cleanup path to call it.

### `kernel/bpf/verifier.c`

**Removed** the mutual-exclusion block in `do_misc_fixups` that returned
`-EOPNOTSUPP` for programs with both `undo_log_requires_jit` and `arena`.

**Added** a bounds check in `check_mem_access` for `PTR_TO_ARENA`: if
`reg->umin_value >= SZ_2G` the access is rejected with `-EACCES` and a
`"reaches undo-log region"` message. This guards the rare case where the
verifier can statically prove the pointer is guaranteed to be in the undo-log
partition (bit 31 = 1).

---

## New Test Files

### `tools/testing/selftests/bpf/progs/test_arena_undo_log.c`

BPF-side program with:
- `combined_map` (array map with `bpf_spin_lock` + `u64` field)
- `arena` map (`BPF_MAP_TYPE_ARENA`, 4 pages, `BPF_F_MMAPABLE`)
- `__arena __u64 arena_val` global
- `tc_arena_combined`: inside a spinlock CS, writes to both `v->u64_a` (map)
  and `arena_val` (arena), then spins in `bpf_loop` to trigger the timeout

### `tools/testing/selftests/bpf/prog_tests/arena_undo_log.c`

Host-side test runner with two sub-tests:

**`test_arena_undo_combined`**: verifies the end-to-end combined path.
1. Sets a 5-second spinlock timeout
2. Pre-fills the map value and the arena global with known sentinels
3. Runs `tc_arena_combined` (spins until timeout fires + undo-log replay)
4. Asserts both the map write and the arena write were rolled back

**`test_arena_bounds`**: verifies the 2 GB arena size limit.
1. Creates a 1-page arena to check basic support; skips if unsupported
2. Attempts to create an arena with `SZ_2G/PAGE_SIZE + 1` pages
3. Asserts the creation fails with `errno == E2BIG`

---

## Kernel Build

Build #33: `make KCONFIG_CONFIG=bpftest-config -j$(nproc) bzImage` — **passes**.

## Selftest Build

`make -C tools/testing/selftests/bpf test_progs` — **passes**.
`test_arena_undo_log.skel.h` generated with `skel->arena->arena_val` member.
`test_arena_undo_log` registered in `test_progs` binary.
