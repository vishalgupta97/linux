# eBPF Spinlock Timeout Implementation - Walkthrough

## Summary

Implemented a mechanism to terminate eBPF programs that timeout while holding spinlocks, using loop iteration checkpoints instead of direct hrtimer callback termination.

## Files Modified

### [helpers.c](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/kernel/bpf/helpers.c)

**Changes:**
- Added per-CPU `ebpf_spinlock_timeout` variable with `EXPORT_PER_CPU_SYMBOL_GPL`
- Created `bpf_spin_lock_timer_cb()` - new hrtimer callback that only sets the timeout flag
- Refactored `bpf_spin_lock_timeout_handler()`:
  - Changed from static hrtimer callback to exported function
  - Removed `WARN_ONCE` and `BUG_ON`
  - Added call to `bpf_die(NULL)` for program termination
- Modified `bpf_spin_lock()` to reset timeout flag when starting timer
- Modified `bpf_spin_unlock()` to reset timeout flag when cancelling timer

---

### [bpf_iter.c](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/kernel/bpf/bpf_iter.c)

**Changes:**
- Added extern declarations for `ebpf_spinlock_timeout` and `bpf_spin_lock_timeout_handler()`
- Modified `bpf_loop()`:
  - Removed `BPF_MAX_LOOPS` limit check
  - Added timeout flag check in each iteration
  - Calls handler and returns `-ETIMEDOUT` if timeout detected
- Modified `bpf_iter_num_next()`:
  - Added timeout flag check at start of function
  - Returns NULL after calling handler if timeout detected

---

### [NEW] [test_spin_lock_loop_timeout.c](file:///wsl.localhost/Ubuntu-24.04/home/vishal/linux/tools/testing/selftests/bpf/progs/test_spin_lock_loop_timeout.c)

New selftest that:
- Acquires a spinlock
- Runs `bpf_loop` with max iterations (0xFFFFFFFF)
- Expects timeout mechanism to terminate the program

## Build & Test Commands

```bash
# Build kernel
cd /home/vishal/linux
make -j$(nproc)

# Build BPF selftests
make -C tools/testing/selftests/bpf

# Run specific test (after booting new kernel)
./tools/testing/selftests/bpf/test_progs -t spin_lock_loop_timeout
```
