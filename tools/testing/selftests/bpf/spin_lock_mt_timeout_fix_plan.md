# spin_lock_mt_timeout / bpf_lock_func Fix Plan

This document tracks the fixes needed to make the multi-threaded BPF
spin-lock timeout tests reliable with `bpf_lock_func()` / fpop.

## 1. Fix mixed queue implementations

Problem:

`bpf_lock_func()` uses the fpop queue, while `bpf_spin_lock()` uses the BPF
qspinlock/MCS queue. Both publish into the same `qspinlock->tail` field, but
decode that tail into different per-CPU node families:

- fpop decodes tail as `struct fpop_node`
- qspinlock decodes tail as `struct qnode` / `struct mcs_spinlock`

If one API queues behind the other on the same lock, the predecessor link is
written into the wrong node object. The successor can then spin forever waiting
for a handoff that the predecessor will never observe.

Required fix:

- Do not mix `bpf_lock_func()` and `bpf_spin_lock()` on the same lock unless
  both paths share one compatible queue/handoff protocol.
- For the current selftest, avoid using `lf_short_cs_incr_b()` as the contender
  for `lf_nested_inner_holder()`, because the holder takes inner lock B with
  `bpf_spin_lock()`. Use the plain `short_cs_incr_b()` contender for that
  specific lockfunc nested test, or split the test so each lock is owned by a
  single queue implementation.

Non-fix:

- Do not let non-head MCS waiters trigger timeout. Only the queue head should
  terminate the current lock holder.

## 2. Keep timeout set while fpop combiner unwinds

Problem:

When a waiter times out while the combiner is executing that waiter's callback,
the timeout is delivered through `bpf_loop()`. If the fpop-running timeout
handler clears `ebpf_spinlock_timeout`, only the innermost `bpf_loop()` returns
early. Outer nested loops can continue for a very long time, making the combiner
look stuck.

Required fix:

- In `bpf_spin_lock_timeout_handler()`, when `fpop_running` is true, do not clear
  `ebpf_spinlock_timeout`.
- Replay the undo log, then return so nested `bpf_loop()` frames keep observing
  timeout and unwind.
- Let the next critical-section entry clear timeout through
  `set_state_for_cs_timeout()` or `execute_op()`.

Status:

- Implemented in the working tree.

## 3. Release inner spin locks on fpop timeout

Problem:

An offloaded `bpf_lock_func()` callback can take regular inner
`bpf_spin_lock()` locks. If timeout happens while `fpop_running`, replaying the
undo log is not enough; any tracked inner locks must also be released or future
waiters can hang.

Required fix:

- In the `fpop_running` timeout-handler path, replay undo and release all locks
  recorded in `held_locks[]`.
- Keep the outer fpop lock handoff owned by `bpf_fpop.c`.

Status:

- Implemented in the working tree.

## 4. Protect per-CPU fpop node reuse

Problem:

When a waiter observes `_FPOP_PRCSING`, it can time out and `bpf_throw(50)`
before the combiner stores `_FPOP_PRCSD`. The waiter returns to user context,
and the same CPU can re-enter `bpf_lock_func()` and reuse its per-CPU
`fpop_node` while the combiner still owns it.

Required fix:

- Before initializing this CPU's `fpop_node`, wait while `completed ==
  _FPOP_PRCSING`.
- The combiner must always store `_FPOP_PRCSD` after the callback unwinds.

Status:

- Implemented in the working tree.

## 5. Track combiner-owned critical sections

Problem:

Some fpop paths ran the combiner's own callback with `execute_op()` and then
manually unlocked. That bypassed `set_state_for_cs_timeout()`, so timeout state,
undo-log state, and held-lock tracking were not set up like a normal critical
section.

Required fix:

- Use `execute_op_and_unlock()` for combiner-owned callbacks.
- This calls `set_state_for_cs_timeout()` before the callback and
  `reset_state_for_cs_timeout()` after unlock.

Status:

- Implemented in the working tree.

## 6. Fix preemption accounting on waiter-side throw

Problem:

`fpop_execute()` disables preemption before entering the fpop path. If a waiter
times out in `_FPOP_PRCSING` wait and calls `bpf_throw(50)`, it must not escape
with preemption still disabled.

Required fix:

- Balance the `preempt_disable()` before `bpf_throw(50)` in the waiter timeout
  path.

Status:

- Implemented in the working tree.

## 7. Fix timeout slowpath value use

Problem:

In the fpop timeout-enabled wait for `lock->val` to clear locked/pending, `val`
must be refreshed after the polling loop. Otherwise the following cmpxchg can use
an uninitialized or stale value.

Required fix:

- Assign `val = READ_ONCE(lock->val.counter)` after the timeout polling loop.

Status:

- Implemented in the working tree.

## 8. Fix watchdog active-session test

Problem:

The no-contention watchdog kthread checked whether `per_cpu_ptr()` returned
`NULL`. That checks the address of the per-CPU slot, not the active timer value,
so normal unlock could be ignored and a stale timeout could be raised.

Required fix:

- Test the value stored in the per-CPU slot:
  `READ_ONCE(*per_cpu_ptr(&bpf_active_timer, cpu))`.

Status:

- Implemented in the working tree.

## 9. Replace global timeout state with scoped timeout state

Problem:

`ebpf_spinlock_timeout` used to be global. A timeout for lock A could be
observed by BPF programs running under unrelated lock B on another CPU, causing
false rollbacks.

Required fix:

- Replace the global timeout bit with state scoped at least per CPU.
- Associate timeout events with a generation/token so stale events cannot abort
  a new critical section.
- `bpf_loop()` / `bpf_iter_num_next()` should check whether the timeout applies
  to the current BPF lock session, not just the current CPU.

Status:

- Per-CPU timeout state is implemented in the working tree.
- Per-session generation/token validation is not implemented.

## 10. Replace single kthread watchdog session

Problem:

The kthread watchdog uses one global pending bit and one global
`bpf_kthread_timer.bpf_cpuid`. Concurrent no-contention critical sections on
different CPUs can overwrite/coalesce sessions.

Required fix:

- Use per-CPU watchdog state, or a queue of timeout sessions.
- Include a generation/token in each session.
- Unlock should cancel only the matching active session.

Status:

- Not implemented.

## 11. Update helper ABI documentation

Problem:

The `bpf_lock_func` helper declaration and UAPI comments must match the current
five-argument helper shape:

`bpf_lock_func(lock, callback_fn, v1, v2, v3)`

Required fix:

- Keep `include/uapi/linux/bpf.h` and `tools/include/uapi/linux/bpf.h` in sync
  with the five-argument helper and callback ABI.

Status:

- Implemented in the working tree.

## 12. Expected subtest impact

- `lockfunc_contended_holder_timeout`: should be covered by the fpop timeout
  unwind fixes.
- `lockfunc_nested_inner_contended_timeout`: unsafe until queue mixing on lock B
  is removed or the two lock APIs share a queue protocol.
- `lockfunc_deadlock_abba_rollback`: should not hit queue mixing because timeout
  occurs before the later plain `bpf_spin_lock()`, but it still depends on the
  global watchdog/session behavior.
- `lockfunc_concurrent_timeouts_stress`: pure fpop on one lock; this is the main
  stress case for waiter timeout while combiner is running.
- `lockfunc_independent_locks_isolation`: expected to fail until timeout state is
  scoped instead of global.
- `lockfunc_mcs_queue_stress`: pure fpop under long timeout; should not be run as
  a qspinlock/MCS correctness test for the lockfunc path.
