# BPF Spinlock Timeout + Undo-Log: Design and Implementation

This document describes all the kernel and BPF-verifier changes made on the
`v7.0-timeout-undo-log` branch (57 commits on top of the `028ef9c96e "Linux
7.0"` base). It was produced by reviewing the cumulative diff of the whole
branch against that base. The focus is the core kernel and verifier mechanism:
spinlock timeout, write undo-logging, and program termination/rollback. The
benchmark harness, plotting scripts, and result CSVs that make up most of the
line count are intentionally not covered here.

The branch turns BPF `bpf_spin_lock` critical sections from *unbounded,
unabortable* regions into *bounded, abortable, atomic-on-abort* regions. Three
properties are added:

1. **Bounded** — a watchdog times out a critical section that runs too long.
2. **Abortable** — on timeout the program is forcibly unwound (`bpf_throw`),
   and every lock it holds is released.
3. **Atomic on abort** — every memory write done inside the critical section is
   rolled back before the locks are dropped, so no other CPU ever observes a
   half-finished critical section.

---

## 1. High-level design decisions

### 1.1 Why this is hard in stock BPF

Upstream BPF deliberately makes `bpf_spin_lock` regions trivially safe by
forbidding almost everything inside them: no function calls, no loops that can
spin unboundedly, no nested locks, strict in-order unlock. The lock is taken
with IRQs disabled and preemption disabled, so the critical section is by
construction short and uninterruptible. There is nothing to time out and
nothing to roll back, because the verifier already guarantees termination.

This work intentionally **relaxes** those restrictions (nested locking, helper
and global-function calls, `bpf_loop` inside the lock) to support real
concurrent data structures. That reintroduces the possibility of a critical
section running arbitrarily long, which is why a timeout + rollback safety net
becomes necessary.

### 1.2 Key design choices

**Lock with IRQs enabled, not disabled.** The custom BPF qspinlock
(`bpf_qspinlock.c`) acquires with interrupts *enabled* and only disables
preemption. This is the linchpin: an hrtimer can only fire on a CPU that is
spinning for or holding the lock if that CPU still takes interrupts. Stock
`bpf_spin_lock` disables IRQs, which would deadlock the watchdog.

**Cooperative termination, not asynchronous cancellation.** The timer does not
kill the program from interrupt context. It only sets a global flag
(`ebpf_spinlock_timeout`). The program notices the flag at the next loop
iteration (`bpf_loop` / `bpf_iter_num_next`) and *itself* calls the timeout
handler. This keeps termination at a well-defined point where the stack and
registers are consistent, which is required for `bpf_throw` to unwind cleanly.
The cost is that a critical section with no loop in it cannot self-terminate;
the design targets loop-bearing data-structure operations.

**Undo log instead of shadow copies or transactional memory.** Rollback is done
with a per-CPU, fixed-capacity, append-only log of `{addr, old_value, size}`
records. Before each write inside a critical section, the *old* value at the
destination is captured. On abort the log is replayed in reverse. This is
cheap (one extra load + a few stores per CS write), needs no allocation on the
hot path, and is naturally per-CPU because a critical section runs on a single
CPU with preemption disabled.

**Push the cost onto the JIT, not the interpreter.** The verifier only *marks*
which writes are inside a critical section and injects a call to a marker stub
(`bpf_undo_log_push`). The x86-64 JIT recognizes that marker and replaces it
with a handful of inline instructions that use **R12 as the log cursor**. There
is no real function call per write. Programs that need undo logging are forced
to be JIT-compiled (`undo_log_requires_jit`); the interpreter path is rejected
because it cannot emit the inline cursor code.

**Static, verifier-enforced capacity.** The undo log is a fixed per-CPU array
(`CONFIG_BPF_UNDO_LOG_MAX_ENTRIES`, default 64). The verifier counts CS writes
per path and rejects any program that could exceed the capacity, so the log can
never overflow at run time and needs no bounds check in the JITted fast path.

**Two timeout paths for contended vs. uncontended.** If a critical section is
contended, the waiter at the head of the MCS queue is already spinning with
IRQs enabled and starts the timeout itself. If it is uncontended, there is no
waiter, so the fast path notifies a watchdog **kthread** which runs the timeout
for it.

**Feature-gated and composable.** `CONFIG_BPF_TIMEOUT` and
`CONFIG_BPF_UNDO_LOG` are independent. Timeout without undo-log gives
abort-without-rollback; undo-log without timeout instruments writes but never
replays them. `CONFIG_BPF_SPINLOCK_HOOKS` is auto-selected when either is on
and gates the lock-tracking machinery.

---

## 2. Configuration (`kernel/bpf/Kconfig`)

Four new options:

| Option | Default | Meaning |
|--------|---------|---------|
| `CONFIG_BPF_TIMEOUT` | n | Per-CPU hrtimer + kthread watchdog; on timeout terminate the program via `bpf_throw()` and release held locks. |
| `CONFIG_BPF_UNDO_LOG` | n | Track CS writes and roll them back on timeout. Verifier injects markers; JIT inlines them. |
| `CONFIG_BPF_UNDO_LOG_MAX_ENTRIES` | 64 (range 1–1024) | Per-CS write capacity; programs exceeding it are rejected at load. |
| `CONFIG_BPF_SPINLOCK_USE_KOMB` | n | Use the KOMB combining lock instead of the plain qspinlock. |

`CONFIG_BPF_SPINLOCK_HOOKS` is a hidden `def_bool (BPF_TIMEOUT || BPF_UNDO_LOG)`
that gates the held-lock tracking and the `bpf_qspinlock.o / bpf_komb.o /
bpf_fpop.o` build (`kernel/bpf/Makefile`). `CFLAGS_REMOVE_bpf_qspinlock.o`
strips ftrace instrumentation from the lock slow path.

---

## 3. Verifier changes (`kernel/bpf/verifier.c`)

The verifier is responsible for (a) relaxing the in-CS restrictions safely,
(b) deciding which writes need undo logging and marking them, (c) enforcing the
undo-log capacity, and (d) wiring up the new `bpf_lock_func` helper.

### 3.1 Relaxing critical-section restrictions

- **Nested locking allowed.** `process_spin_lock()` no longer rejects a second
  `bpf_spin_lock`. Instead it rejects re-acquiring *the same* lock
  (`find_lock_state(... reg->id, ptr)` → "AA deadlock") and caps nesting at 32
  (`active_locks >= 32` → "Locking depth limit reached").
- **Out-of-order unlock allowed.** The old strict check
  (`reg->id != active_lock_id || ptr != active_lock_ptr` → "cannot be out of
  order") is replaced by a membership test `find_lock_state(cur, type, reg->id,
  ptr)`. Any currently-held lock may be unlocked in any order. (The runtime
  `held_locks[]` removal logic in `helpers.c` is written to match, shifting
  entries down.)
- **Calls allowed while holding a lock.** Both the global-function-call check
  in `check_func_call()` and the per-instruction call check in
  `do_check_insn()` are relaxed so helper calls, kfunc calls, and global
  function calls are permitted inside a lock. (Previously: "function calls are
  not allowed while holding a lock".)
- **Sleepable callees still rejected.** New guards reject sleepable helpers
  (`fn->might_sleep` in `check_helper_call`) and sleepable kfuncs
  (`check_kfunc_call`) inside a lock region — a sleeping critical section would
  break the IRQs-enabled-but-non-preemptible model and could never time out.

### 3.2 Detecting and counting critical-section writes

A new field `cs_write_count` is added to `struct bpf_verifier_state`, and a new
per-insn flag `in_critical_section` to `struct bpf_insn_aux_data`
(`include/linux/bpf_verifier.h`).

Whenever `active_locks > 0`, three write sites mark the instruction and bump the
counter:

- register stores `STX` (`check_store_reg`)
- immediate stores `ST` (`do_check_insn`)
- atomic RMW (`check_atomic_rmw`)

In all three, **`PTR_TO_STACK` writes are excluded** — BPF stack is private to
the current invocation, discarded on return, and never observed by another CPU,
so it needs no rollback. Each marked site:

```c
if (cs_write_count >= CONFIG_BPF_UNDO_LOG_MAX_ENTRIES)
        return -E2BIG;   // "write count exceeds undo log capacity"
insn_aux_data[insn_idx].in_critical_section = true;
cs_write_count++;
```

The counter is reset to 0 on the outermost lock acquire (`!active_locks` before
acquiring) and again when the last lock is released, so counting is per
top-level critical section.

### 3.3 Capacity enforcement is safe under state pruning

State pruning could let an overflowing path slip through if a cached state had
counted fewer writes. `states_equal()` adds:

```c
if (old->cs_write_count < cur->cs_write_count)
        return false;
```

i.e. a cached state may only prune the current one if it already explored at
least as many CS writes. This guarantees the `-E2BIG` check is sound across all
paths.

### 3.4 Injecting the undo-log marker (`do_misc_fixups`)

Under `CONFIG_BPF_UNDO_LOG`, the fixup loop scans for `in_critical_section`
instructions. For each store-class instruction (`BPF_STX`/`BPF_ST` with mode
`BPF_MEM`, `BPF_ATOMIC`, or `BPF_PROBE_MEM32`) it patches in a marker call
*before* the write:

```c
insn_buf[0] = BPF_EMIT_CALL(bpf_undo_log_push);
insn_buf[1] = *insn;          // the original store
bpf_patch_insn_data(...);
prog->aux->undo_log_requires_jit = true;
```

`bpf_undo_log_push` is never actually called as a function (see §4.3); it is a
recognizable marker the JIT rewrites. Setting `undo_log_requires_jit` forces
JIT compilation and forbids the interpreter (`core.c`, §6).

### 3.5 Arena + undo-log are unified, not rejected

Rather than rejecting the combination, arena access is restricted to the lower
2 GB so the upper 2 GB of the arena window can hold the undo-log pages
(`check_mem_access` for `PTR_TO_ARENA`):

```c
if (reg->umin_value >= SZ_2G)   // reaches reserved undo-log region
        return -EACCES;
```

This dovetails with the arena allocator reserving the top 2 GB (§9) and lets the
JIT reuse R12 for both arena base and undo-log cursor (§5.3).

### 3.6 `bpf_lock_func`: a lock-scoped callback helper

`bpf_lock_func(lock, callback_fn, v1, v2, v3)` acquires `lock`, runs the
callback with the lock held (so its writes are undo-logged), and releases it.
The verifier models this so the callback body is verified *inside* the
critical section:

- It is registered as a sync-callback-calling function
  (`is_sync_callback_calling_function`).
- In `check_func_arg`, the lock argument is processed with
  `process_spin_lock(..., PROCESS_SPIN_LOCK)` so the lock is acquired before the
  callback is pushed.
- `set_lock_func_callback_state()` forwards `v1,v2,v3` as the callback's first
  three args (the other two are zeroed, matching `execute_op()`), marks the
  callee frame `in_lock_func_cb`, and records the lock id/ptr to release on
  callback exit.
- New `struct bpf_func_state` fields `in_lock_func_cb`, `lock_func_lock_id`,
  `lock_func_lock_ptr` (`include/linux/bpf_verifier.h`).
- **Nesting is forbidden.** `is_in_lock_func_cb()` walks all frames; a
  `bpf_lock_func` reached while any frame is already a lock-func callback is
  rejected ("bpf_lock_func cannot be nested"). Locking a *different* lock with
  plain `bpf_spin_lock` inside the callback is still allowed.
- **Lock release on both branches.** On the main continuation,
  `check_helper_call` releases the lock immediately (`process_spin_lock(...,
  BPF_REG_1, 0)`). On the callback-return branch, `prepare_func_exit()` releases
  the recorded lock and returns to `callsite + 1` (the callback runs exactly
  once, unlike `bpf_loop` which reschedules to `callsite`).

The UAPI helper id is `BPF_FUNC_lock_func = 212` (`include/uapi/linux/bpf.h`),
with a proto in `helpers.c` and registration in `bpf_base_func_proto`.

### 3.7 Other verifier changes

- **`MEM_WRITE` BTF pointers** may be written through: `check_ptr_to_btf_access`
  allows non-read access when `reg->type & MEM_WRITE` (see BTF change in §10).
- **`bpf_loop` inlining disabled.** `optimize_bpf_loop()` is short-circuited
  (`&& 0`) and the call in `bpf_check()` is commented out. Inlining would bypass
  the per-iteration timeout check that lives in the real `bpf_loop` helper.
- **`seen_exception` forced true.** `bpf_check()` sets `env->seen_exception =
  true` so the exception/throw frame infrastructure is always built — the
  timeout handler relies on `bpf_throw` being able to unwind any program.

---

## 4. Undo-log storage and replay (`kernel/bpf/helpers.c`)

### 4.1 Per-CPU storage

```c
struct bpf_undo_log_entry { void *addr; u64 old_value; u8 size; };

DEFINE_PER_CPU(struct bpf_undo_log_entry[CONFIG_BPF_UNDO_LOG_MAX_ENTRIES], bpf_undo_log);
DEFINE_PER_CPU(struct bpf_undo_log_entry *, bpf_undo_log_cursor);
DEFINE_PER_CPU(struct bpf_undo_log_entry *, bpf_undo_log_base);
```

`cursor` is the append pointer; `base` is where it resets to. For plain
(non-arena) programs both point at the static `bpf_undo_log` array
(`bpf_undo_log_init`, `core_initcall`). For arena programs they are repointed at
the arena-mapped pages (§9). The struct definition and `DECLARE_PER_CPU`s live
in `include/linux/bpf.h`.

### 4.2 Cursor lifecycle

The cursor is reset to `base` (logically emptying the log) at three points:

- Outermost `bpf_spin_lock` acquire (`cnt == 1` in `__internal__bpf_spin_lock`).
- Last `bpf_spin_unlock` release — a *successful* commit, so the captured old
  values must be discarded and not replayed by a later, unrelated timeout.
- Before each `bpf_lock_func` callback (`fpop_reset_undo_log`, §8), because that
  path acquires the lock inline and bypasses the helper.

### 4.3 The marker stub

```c
NOTRACE_BPF_CALL_2(bpf_undo_log_push, unsigned long, addr, u64, size)
{
        WARN_ONCE(1, "bpf_undo_log_push reached: JIT inlining missing\n");
        return -EOPNOTSUPP;
}
```

This body should never execute; reaching it means the JIT failed to inline a
marker. New `NOTRACE_BPF_CALL_2/3/5` macros are added in
`include/linux/filter.h` to declare it (and `bpf_lock_func`) without ftrace.

### 4.4 Replay

```c
static void bpf_undo_log_replay(void)
{
        base = this_cpu_read(bpf_undo_log_base);
        cursor = this_cpu_read(bpf_undo_log_cursor);
        for (i = (cursor - base) - 1; i >= 0; i--)
                WRITE_ONCE(*(uN *)base[i].addr, (uN)base[i].old_value);  // by size
        this_cpu_write(bpf_undo_log_cursor, base);
}
```

Reverse order matters: if the same location was written twice in the CS, the
*first* (oldest) recorded value is the one restored last and therefore wins.

---

## 5. x86-64 JIT: inline undo-log emission (`arch/x86/net/bpf_jit_comp.c`)

### 5.1 Marker recognition

`bpf_insn_is_undo_log_marker()` matches a `BPF_JMP|BPF_CALL` whose target is
`bpf_undo_log_push`. `bpf_prog_has_undo_log_markers()` scans the whole program
once; the result (`has_undo_log_markers`) drives R12 setup.

### 5.2 R12 as the log cursor

When a program has markers (and is *not* an arena program), R12 is dedicated to
the undo-log cursor:

- **Prologue:** `push_r12` is emitted whenever `arena_vm_start ||
  has_undo_log_markers`. For the undo-log-only case R12 is loaded with
  `this_cpu_ptr(&bpf_undo_log)` via `movabsq $&bpf_undo_log, %r12` plus a
  GS-segment `add %gs:this_cpu_off, %r12` (SMP).
- **Epilogue / tail-call exits:** `pop_r12` is emitted under the same condition.

### 5.3 Per-write inline sequence

At each marker the JIT inspects the *following* store instruction (dst reg,
`off`, size) and emits, with no call:

```
r11 = dst_breg
r11 += off                         ; effective write address
[r12 + off_addr] = r11             ; entry.addr
r11 = *(size *)(r11)               ; load OLD value (zero-extended)
[r12 + off_old]  = r11             ; entry.old_value
r11d = size; [r12 + off_size] = r11b   ; entry.size
r12 += sizeof(struct bpf_undo_log_entry)  ; advance cursor
```

`emit_stx_r12base()` is a dedicated helper because using R12 as a memory base
always requires a SIB byte (`rm=4`, `SIB=0x24`). AUX_REG (R11) is the scratch
register. After emitting the sequence the JIT falls through to emit the
original store, then `goto emit_insn_done`.

### 5.4 Combined arena + undo-log

For arena programs, R12 is already pinned to `arena_vm_start`, so it cannot also
be the cursor. `emit_undo_log_push_combined()` temporarily **borrows** R12:
compute the real kernel VA in R11 (adding R12 first for `PROBE_MEM32` arena
stores), `push %r12`, load the cursor from `%gs:bpf_undo_log_cursor` into R12,
write the entry, advance, store the cursor back, `pop %r12` to restore the arena
base. The cursor lives in the per-CPU `bpf_undo_log_cursor`, which for arena
programs points into the mapped arena pages (§9).

### 5.5 Cursor spill/reload around calls and tail calls

For undo-log-only programs R12 (the live cursor) must be kept coherent with the
per-CPU `bpf_undo_log_cursor` across calls, because callees (e.g.
`bpf_spin_lock`) reset that per-CPU cursor:

- Before a `BPF_CALL`: `emit_undo_log_spill_r12` → `movq %r12,
  %gs:bpf_undo_log_cursor` (and `ip += 9` to keep call-offset accounting
  correct). After the call: `emit_undo_log_reload_r12` reloads R12.
- Around `BPF_TAIL_CALL`: spill then reload unconditionally when markers exist.

For combined arena programs R12 is the constant arena base, so no spill/reload is
needed (only the per-CPU cursor is touched, inside the combined sequence).

---

## 6. Forcing JIT, forbidding interpreter (`kernel/bpf/core.c`)

```c
if (fp->aux->undo_log_requires_jit && !fp->jited) {
        *err = -EOPNOTSUPP;
        return fp;
}
```

A program with injected markers that did not get JIT-compiled is rejected,
because the interpreter would actually *call* the `bpf_undo_log_push` stub
(which WARNs and returns `-EOPNOTSUPP`) instead of logging.

---

## 7. Timeout: watchdog, detection, and termination

### 7.1 The global flag and timer callback (`helpers.c`)

`int ebpf_spinlock_timeout;` is the single global signal. The hrtimer callback
just sets it:

```c
enum hrtimer_restart bpf_qspinlock_timer_cb(struct hrtimer *timer)
{ WRITE_ONCE(ebpf_spinlock_timeout, 1); return HRTIMER_NORESTART; }
```

A `bpf_lock_timer_ops` abstraction (`include/linux/bpf_lock_timer.h`) wraps
`start`/`cancel` over an hrtimer, so both the per-CPU waiter timers and the
kthread timer share one backend (`bpf_hrtimer_ops`).

### 7.2 Per-iteration detection (`kernel/bpf/bpf_iter.c`)

Both loop primitives check the flag every iteration and, if set, call the
handler (which does not return — it `bpf_throw`s):

- `bpf_loop`: checks at the top of each iteration; also drops the
  `nr_loops > BPF_MAX_LOOPS` cap under `CONFIG_BPF_TIMEOUT` so genuinely long
  loops are allowed (the timeout bounds them instead).
- `bpf_iter_num_next` (the `bpf_for` primitive): same check.

Because inlining is disabled (§3.7), the check always runs in the real helper.

### 7.3 The timeout handler (`helpers.c`)

```c
void bpf_spin_lock_timeout_handler(void)
{
        bpf_undo_log_replay();              // (1) roll back writes first
        for (i = cnt-1; i >= 0; i--) {      // (2) release held locks, reverse
                bpf_qspinlock_unlock(locks[i].lock);
                preempt_enable();           //     one enable per acquire
        }
        held_locks_cnt = 0;
        ebpf_spinlock_timeout = 0;
        bpf_throw(100);                     // (3) unwind out of the program
}
```

Ordering is critical: **rollback happens before unlock**, so no other CPU can
acquire the lock and observe partially-written state. `bpf_throw(100)` performs
the actual non-local exit out of the BPF program using the exception
infrastructure (`seen_exception` forced on at verify time). The held-lock set is
tracked per-CPU in `held_locks[MAX_HELD_LOCKS=32]` / `held_locks_cnt`.

> Note: the CLAUDE.md narrative refers to `bpf_die()`; the current code uses
> `bpf_throw(100)` (the `bpf_die` call is commented out).

### 7.4 Held-lock tracking (`helpers.c`)

`bpf_spin_lock`/`bpf_spin_unlock` are rerouted under
`CONFIG_BPF_SPINLOCK_HOOKS` to `__internal__bpf_spin_lock` /
`__internal__bpf_spin_unlock`, which:

- disable/enable **preemption** (not IRQs — IRQs stay enabled for the timer),
- call the custom `bpf_qspinlock_lock/unlock` (or KOMB variant),
- push/remove the lock in the per-CPU `held_locks[]` (unlock supports
  out-of-order removal by shifting entries down),
- on the outermost acquire, reset the undo-log cursor and clear the timeout
  flag; on the final release, reset the cursor and clear the active timer.

### 7.5 Two timeout paths

**Contended (waiter present).** In `bpf_qspinlock.c`, the waiter spinning at the
head of the MCS queue (or in the pending state) runs a bounded spin loop:
compute `end_time` from `sysctl_bpf_spin_lock_timeout`, and once exceeded call
`tell_bpf_loop_to_terminate()` (which sets `ebpf_spinlock_timeout`). The current
owner then notices the flag in its loop and self-terminates; the waiter
subsequently acquires normally. IRQs are enabled during this spin specifically
so timers/interrupts work.

**Uncontended (fast path).** With no waiter, `bpf_qspinlock_lock()` calls
`bpf_notify_lock_kthread()`, which records the kthread's timer as this CPU's
`bpf_active_timer` and wakes the `bpf_lock_wd` kthread. The kthread
(`bpf_lock_timeout_kthread_fn`, a `late_initcall`) computes the timeout and runs
`bpf_lock_timeout_rdtsc()`: it busy-waits until either the deadline passes (then
sets the terminate flag) or the lock session ends (`bpf_active_timer` cleared).
`bpf_notify_unlock_kthread()` clears the active-timer pointer on release.

---

## 8. The custom BPF qspinlock (`kernel/bpf/bpf_qspinlock.c`)

A new ~390-line lock modeled on `rqspinlock.c` but specialized for this work:

- **Acquires with IRQs enabled**, disabling them only conceptually after
  acquisition — the whole point is to let the hrtimer fire on a spinning/owning
  CPU. (Preemption is disabled by the helper wrapper.)
- **Never returns an error and has no deadlock detection** — on timeout the
  *owner* is terminated by the handler, and the waiter just acquires afterward.
- Uses its **own** per-CPU MCS nodes (`bpf_qnodes`), separate from the kernel
  qspinlock and rqspinlock node pools, so the BPF lock path can't corrupt
  generic locking state.
- The pending-wait and head-of-queue waits embed the bounded timeout spin
  described in §7.5; the non-timeout build falls back to the standard
  `smp_cond_load_acquire` / `atomic_cond_read_acquire`.
- `bpf_qspinlock_unlock()` is a plain `smp_store_release(&lock->locked, 0)`.
- `bpf_qspinlock_init_timers()` (called from the kthread init) sets up the
  per-CPU waiter hrtimers; `bpf_active_timer` is defined and exported here.

Declarations live in `include/linux/bpf_qspinlock.h`.

---

## 9. Arena integration (`kernel/bpf/arena.c`)

To let arena programs *also* undo-log, the arena VM window is split:

- The usable arena range is capped at `SZ_2G` (was `SZ_4G`) in
  `arena_map_alloc`. The verifier enforces the matching `< SZ_2G` bound (§3.5).
- `bpf_arena_alloc_undo_log_pages()` allocates **one zeroed page per possible
  CPU** and maps each at `kern_vm_start + SZ_2G + cpu * PAGE_SIZE`, then
  repoints that CPU's `bpf_undo_log_base`/`cursor` at the mapped page. So the
  JIT's combined sequence (§5.4) and the C replay code write to the same arena
  pages.
- `bpf_arena_free_undo_log_pages()` (called from `arena_map_free`) resets the
  per-CPU pointers back to the static kernel arrays before the VM window is torn
  down; the physical pages are freed with the rest of the arena.
- A `struct page **undo_log_pages` field is added to `struct bpf_arena`. The
  non-`CONFIG_BPF_UNDO_LOG` build stubs both functions to no-ops.

---

## 10. Writable kernel-memory BTF tags (`kernel/bpf/btf.c`)

For data structures stored in *kernel* memory (not map values or arena), the
verifier must allow writes through `PTR_TO_BTF_ID`. `btf_ctx_access()` now marks
a pointer `MEM_WRITE` when any of three annotations is present, so it works
across toolchains:

1. `btf_decl_tag("writable_bpf")` type tag (Clang BPF target / vmlinux BTF).
2. A typedef whose name ends in `_bpf_writable` (GCC-friendly: GCC drops
   `btf_decl_tag` but preserves typedef names in BTF).
3. A `BTF_KIND_DECL_TAG "writable_bpf"` on the function parameter
   (`btf_find_next_decl_tag`), for Clang-native kernel-module builds.

`MEM_WRITE` then unlocks the relaxed write check in `check_ptr_to_btf_access`
(§3.7), and those writes get undo-logged like any other CS write.

---

## 11. `bpf_lock_func` runtime + the FPOP/KOMB combining lock

`bpf_lock_func` (`helpers.c`) calls `fpop_execute(lock, callback, v1,v2,v3)`
(`kernel/bpf/bpf_fpop.c`). FPOP is a **flat/combining** lock: under contention a
single "combiner" CPU executes the queued callbacks of other CPUs on their
behalf, improving cache locality for hot shared data structures. Key points
relevant to the timeout/undo-log core:

- Each callback is treated as an **independent outermost critical section**, so
  `fpop_reset_undo_log()` resets the per-CPU cursor to base immediately before
  every callback (`execute_op` / `execute_op_and_unlock`). Without this the
  cursor would advance forever across successive `lock_func` calls — and across
  every node a combiner runs — until it overflows the per-CPU page and faults.
- It integrates the same timeout hooks (`bpf_notify_lock_kthread`,
  `tell_bpf_loop_to_terminate`, the bounded IRQ-enabled spin) as the plain
  qspinlock.
- `bpf_komb.c` provides an alternative combining lock selected by
  `CONFIG_BPF_SPINLOCK_USE_KOMB`; the helper layer calls `komb_spin_lock/unlock`
  in that configuration.

---

## 12. End-to-end flow (timeout + rollback)

```
bpf_lock_func / bpf_spin_lock
  └─ preempt_disable; bpf_qspinlock_lock (IRQs enabled)
        ├─ uncontended → notify watchdog kthread to arm timer
        └─ contended   → waiter spins (IRQs on) and arms its own timeout
  └─ record lock in per-CPU held_locks[]; reset undo-log cursor

  ... program runs a bpf_loop inside the critical section ...
        each CS write:  JIT-inlined → log {addr, old_value, size}; advance R12

  timeout deadline passes
  └─ timer/waiter/kthread sets ebpf_spinlock_timeout = 1

  next bpf_loop / bpf_for iteration sees the flag
  └─ bpf_spin_lock_timeout_handler():
        1. bpf_undo_log_replay()  — restore writes in reverse  (BEFORE unlock)
        2. release every lock in held_locks[]  + matching preempt_enable()
        3. bpf_throw(100)         — unwind out of the BPF program
```

On the **normal** (non-timeout) path the final `bpf_spin_unlock` resets the
cursor (discarding the captured old values), cancels/clears the active timer,
and clears the timeout flag — committing the critical section.

---

## 13. Caveats and work-in-progress notes

These are visible in the code and worth flagging for any reviewer:

- **Loop required to self-terminate.** A critical section with no
  `bpf_loop`/`bpf_for` cannot observe the timeout flag and will not abort on its
  own; bounding such sections relies on the contended waiter terminating the
  owner only at its loop points.
- **`ebpf_spinlock_timeout` is a single global, not per-CPU**, despite some
  comments saying "per-CPU". Concurrent critical sections on different CPUs
  share one flag, so a timeout on one CPU can be observed by another; the
  held-lock/undo-log state that the handler acts on is per-CPU, which bounds the
  blast radius, but the flag semantics are coarse.
- **`bpf_lock_timeout_rdtsc` busy-waits** in the kthread rather than sleeping on
  the hrtimer (the `bpf_lock_timer_start` call is commented out), burning a CPU
  for the duration of an uncontended timeout window.
- **Debug residue**: a `printk(KERN_ALERT ...)` remains in
  `bpf_iter_num_next`'s timeout branch, and a `BUG_ON(true)` guards the KOMB
  unlock path in the timeout handler.
- **`bpf_throw(100)`** uses a fixed cookie; the `bpf_die` path referenced in the
  design notes is not the one currently wired up.

---

## Appendix A — Files changed (core mechanism)

| File | Role |
|------|------|
| `kernel/bpf/Kconfig`, `kernel/bpf/Makefile` | New config options; build the lock/fpop/komb objects. |
| `kernel/bpf/verifier.c` | CS-write detection & counting, capacity enforcement, marker injection, relaxed in-CS rules, `bpf_lock_func` modeling, pruning safety, arena bound, forced exception setup. |
| `kernel/bpf/helpers.c` | Undo-log storage/replay, marker stub, held-lock tracking, timeout handler, hrtimer ops, watchdog kthread, `bpf_spin_lock/unlock` reroute, `bpf_lock_func` proto. |
| `arch/x86/net/bpf_jit_comp.c` | Marker recognition; inline undo-log emission (plain + combined arena); R12 cursor setup, spill/reload. |
| `kernel/bpf/bpf_qspinlock.c` | Custom IRQ-enabled qspinlock slow path with bounded timeout spin; per-CPU waiter timers; `bpf_active_timer`. |
| `kernel/bpf/bpf_fpop.c`, `kernel/bpf/bpf_komb.c` | Combining-lock backends for `bpf_lock_func`. |
| `kernel/bpf/bpf_iter.c` | Per-iteration timeout checks in `bpf_loop` / `bpf_iter_num_next`. |
| `kernel/bpf/core.c` | Force JIT / reject interpreter when undo-logging. |
| `kernel/bpf/arena.c` | Reserve top 2 GB for per-CPU undo-log pages; map/free them. |
| `kernel/bpf/btf.c` | `writable_bpf` / `_bpf_writable` annotations → `MEM_WRITE`. |
| `kernel/bpf/syscall.c`, `net/core/sysctl_net_core.c` | `sysctl_bpf_spin_lock_timeout` definition + `/proc/sys/net/core/bpf_spin_lock_timeout`. |
| `include/linux/bpf.h`, `bpf_verifier.h`, `filter.h`, `bpf_lock_timer.h`, `bpf_qspinlock.h`, `bpf_fpop.h`, `bpf_komb.h`, `include/uapi/linux/bpf.h` | Structs, per-CPU declarations, helper id 212, NOTRACE call macros, timer/lock abstractions. |
