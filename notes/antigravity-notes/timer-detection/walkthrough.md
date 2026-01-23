# eBPF Spin Lock Timeout and Cancellation - Implementation Walkthrough

## Summary

Successfully implemented a timeout and cancellation mechanism for eBPF `bpf_spin_lock` to prevent deadlocks. The implementation adds:
- A `sysctl` knob (`/proc/sys/net/core/bpf_spin_lock_timeout`) for configurable timeout in milliseconds
- Runtime lock tracking with a per-CPU lock table (max 32 locks)
- An hrtimer watchdog that monitors critical sections
- Automatic cleanup and cancellation on timeout
- Support for nested locking and out-of-order (OOO) unlocking

## Changes Made

### Core Kernel Changes

#### 1. [bpf.h](file:///home/vigupta/project/linux/include/linux/bpf.h)
- Added `extern int sysctl_bpf_spin_lock_timeout;` declaration

#### 2. [syscall.c](file:///home/vigupta/project/linux/kernel/bpf/syscall.c#L67-L71)
- Defined `sysctl_bpf_spin_lock_timeout` with default value 0 (disabled)
- Exported symbol for use in helpers.c

#### 3. [sysctl_net_core.c](file:///home/vigupta/project/linux/net/core/sysctl_net_core.c#L483-L490)
- Registered sysctl entry `bpf_spin_lock_timeout` in net_core_table
- Allows dynamic configuration via `/proc/sys/net/core/bpf_spin_lock_timeout`

#### 4. [helpers.c](file:///home/vigupta/project/linux/kernel/bpf/helpers.c#L285-L410)

**Per-CPU Data Structures:**
```c
struct bpf_lock_entry {
    struct bpf_spin_lock *lock;
};

#define MAX_HELD_LOCKS 32
static DEFINE_PER_CPU(struct bpf_lock_entry[MAX_HELD_LOCKS], held_locks);
static DEFINE_PER_CPU(int, held_locks_cnt);
static DEFINE_PER_CPU(struct hrtimer, lock_watchdog_timer);
```

**Timeout Handler:**
- `bpf_spin_lock_timeout_handler()`: Releases all held locks in reverse order and triggers cleanup

**Modified `bpf_spin_lock()`:**
- Tracks acquired locks in `held_locks` array
- Starts hrtimer watchdog when acquiring outermost lock (cnt == 1)
- Timer duration set from `sysctl_bpf_spin_lock_timeout`

**Modified `bpf_spin_unlock()`:**
- Removes lock from tracking (supports OOO unlocking via linear search)
- Cancels watchdog timer when last lock is released (cnt == 0)

### Selftests

#### 1. [test_spin_lock_timeout.c](file:///home/vigupta/project/linux/tools/testing/selftests/bpf/progs/test_spin_lock_timeout.c)

BPF programs testing:
- `test_nested_locking`: Validates nested lock acquisition (A → B)
- `test_ooo_unlocking`: Tests out-of-order unlock (Lock A, B → Unlock A, B)
- `test_timeout_trigger`: Forces timeout with busy loop in critical section
- `test_deadlock_prog1` & `test_deadlock_prog2`: AB-BA deadlock scenario

#### 2. [spin_lock_timeout.c](file:///home/vigupta/project/linux/tools/testing/selftests/bpf/prog_tests/spin_lock_timeout.c)

Userspace test runner:
- Saves/restores sysctl values
- Runs nested locking and OOO tests
- Triggers timeout scenarios with short timeout values
- Concurrent execution for AB-BA deadlock testing

## Testing Instructions

### Manual Testing

1. **Set timeout value:**
   ```bash
   echo 1000 > /proc/sys/net/core/bpf_spin_lock_timeout  # 1 second timeout
   ```

2. **Run selftests:**
   ```bash
   cd tools/testing/selftests/bpf
   make
   ./test_progs -t spin_lock_timeout
   ```

3. **Check for timeout warnings:**
   ```bash
   dmesg | grep "BPF spin lock timeout"
   ```

4. **Disable timeout (default):**
   ```bash
   echo 0 > /proc/sys/net/core/bpf_spin_lock_timeout
   ```

### Test Scenarios

**Nested Locking:**
- Validates that nested locks are properly tracked
- Both locks should be in `held_locks` array during critical section

**OOO Unlocking:**
- Tests that locks can be released in any order
- Lock table correctly removes entries via linear search

**Timeout Trigger:**
- Busy loop inside critical section exceeds timeout
- Watchdog triggers, releases all locks, logs warning

**AB-BA Deadlock:**
- Two BPF programs acquire locks in opposite order
- Timeout mechanism should detect and resolve deadlock

## Known Limitations

1. **bpf_throw Integration:**
   - Current implementation uses `WARN_ONCE()` instead of `bpf_throw(0)`
   - Full cancellation requires integration with BPF exception handling framework
   - TODO: Hook into `bpf_exception_cb` for proper stack unwinding

2. **Verification:**
   - Verifier does not enforce MAX_HELD_LOCKS limit
   - Runtime check in place but should be verifier-enforced

3. **Default Disabled:**
   - Timeout is 0 by default to preserve backward compatibility
   - Must be explicitly enabled via sysctl

## Future Enhancements

1. Integrate full `bpf_throw()` cancellation
2. Add verifier checks for lock depth limits
3. Per-program timeout configuration
4. Enhanced telemetry (timeout stats, lock contention metrics)
