# eBPF vs Kernel Spinlock Performance Benchmark

A comprehensive benchmark suite comparing the performance of eBPF spinlocks with kernel qspinlock implementations. Tests multiple data structures with various locking strategies under configurable thread counts and workloads.

## Overview

This benchmark compares performance characteristics in two implementations:

### eBPF Implementation (`tools/testing/selftests/bpf/progs/spinlock_bench_bpf.c`)
- Uses BPF arena memory for data structure allocation
- Per-CPU local counters for operation tracking (no atomic overhead)
- Supports nested locking via hand-over-hand tree traversal
- Tests verify BPF verifier's nested lock support capabilities

### Kernel Module Implementation (`modules/spinlock_bench/spinlock_bench_qspinlock.c`)
- Uses kernel qspinlock for synchronization
- Per-CPU operation counters for thread-safe measurement
- Leverages kthread workers for concurrent thread simulation
- Demonstrates kernel-level spinlock optimization techniques

## Project Structure

```
modules/spinlock_bench/
├── operations.h                    # Shared data structure definitions
├── spinlock_bench_qspinlock.c      # Kernel module with qspinlock
└── Makefile                        # Kernel module build configuration

tools/spinlock_bench/
├── benchmark.py                    # Python test orchestrator
└── config.yaml                     # Test scenario configuration

tools/testing/selftests/bpf/progs/
└── spinlock_bench_bpf.c            # eBPF benchmark program (uses arena memory)
```

## Data Structures & Locking Strategies

### Linked List
- **Global Lock**: Single spinlock protecting entire list
- **Per-Element Lock**: Lock held per node during traversal (demonstrates lock contention scaling)

### Hash Table
- **Global Lock**: Single spinlock protecting all buckets
- **Per-Bucket Lock**: Independent lock per hash bucket (better scalability)
- Hash function: `(key × 2654435761) mod 256 buckets`

### Balanced Binary Tree (AVL)
- **Global Lock**: Single spinlock at root
- **Per-Node Lock**: Hand-over-hand locking during traversal
  - Acquires child lock before releasing parent (nested locks)
  - Tests eBPF nested lock support and verifier approval
  - Demonstrates tree traversal optimizations

## Compilation

### Build Kernel Module

```bash
cd /home/vishal/project/linux/modules/spinlock_bench
make
# Output: spinlock_bench.ko
```

**Requirements:**
- Linux kernel headers matching running kernel
- GCC compiler compatible with kernel build

### Build eBPF Program

The BPF program source is located at:
```
/home/vishal/project/linux/tools/testing/selftests/bpf/progs/spinlock_bench_bpf.c
```

Build via the selftests framework:
```bash
cd /home/vishal/project/linux/tools/testing/selftests/bpf
make spinlock_bench_bpf.skel.h  # Generates skeleton for loading
```

**Requirements:**
- LLVM/Clang for BPF code generation
- libbpf development headers

## Usage

### Manual Module Testing

```bash
# 1. Build the module
cd /home/vishal/project/linux/modules/spinlock_bench
make

# 2. Load with custom parameters
sudo insmod spinlock_bench.ko \
    data_struct=0 \           # 0=list, 1=hash, 2=tree
    lock_strategy=0 \         # 0=global, 1=per-elem, 2=per-bucket, 3=per-node
    num_threads=4 \           # 1-256
    time_to_run_sec=5 \       # Duration in seconds
    num_test_keys=100         # Number of keys for update phase

# 3. View results in kernel log
dmesg | tail -20
# Output includes:
# - Total operations performed
# - Elapsed time
# - Throughput (ops/sec)
```

### Automated Testing with Python Harness

```bash
cd /home/vishal/project/linux/tools/spinlock_bench

# Run specific test
python3 benchmark.py \
    --data-struct list \
    --lock-strategy global \
    --threads 4 \
    --time-to-run 5 \
    --num-keys 100 \
    --output-csv results.csv \
    --output-json results.json

# Run all scenarios from config
python3 benchmark.py \
    --config config.yaml \
    --output-csv benchmarks.csv

# Skip compilation and reuse built modules
python3 benchmark.py --skip-build --output-csv results.csv
```

## Module Parameters

All parameters are configurable at load time:

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `num_threads` | int | 1-256 | 1 | Number of concurrent threads |
| `time_to_run_sec` | int | 1-3600 | 5 | Duration of benchmark in seconds |
| `data_struct` | int | 0-2 | 0 | 0=list, 1=hash, 2=tree |
| `lock_strategy` | int | 0-3 | 0 | 0=global, 1=per-elem, 2=per-bucket, 3=per-node |
| `num_test_keys` | int | 1-10000 | 100 | Keys pre-inserted before timed operations |

### Valid Combinations

Not all data_struct/lock_strategy pairs are valid:

| Data Structure | Lock Strategies |
|---|---|
| Linked List | global, per-element |
| Hash Table | global, per-bucket |
| Tree | global, per-node |

## Test Workflow (Per Benchmark Run)

1. **Bulk Insert Phase**
   - Insert `num_test_keys` key-value pairs
   - Populates data structure before timing begins
   - Insertion not included in throughput measurement

2. **Timed Update Phase**
   - Run for exactly `time_to_run_sec` seconds
   - Repeatedly update pre-existing keys
   - Operations per second measured during this phase
   - This phase determines the throughput metric

3. **Bulk Delete Phase**
   - Clean up data structure
   - Free all allocated memory
   - Delete timing not included in measurement

## Expected Behavior & Interpretation

### Throughput Measurements
- **Higher throughput** = More lock operations completed per second = Better performance
- Measured in **operations per second (ops/sec)**

### Performance Characteristics by Configuration

**Global Lock:**
- Lower throughput at high thread counts (contention bottleneck)
- Simple implementation, good for low contention
- Baseline comparison point

**Per-Element/Per-Bucket Locks:**
- Better scalability with increasing threads
- More complex lock management
- Per-bucket superior for hash tables at high thread counts

**Per-Node (Tree with Hand-Over-Hand):**
- Tests nested lock capabilities
- More complex locking protocol
- eBPF demonstrates nested lock verification
- Throughput reflects tree traversal efficiency

### eBPF vs Kernel qspinlock

**eBPF Spinlocks:**
- Typically lower throughput (verification overhead, BPF interpreter overhead)
- Demonstrates capability and correctness
- Arena memory allocation may have different characteristics
- No kernel boundary crossing

**Kernel qspinlock:**
- Generally higher throughput (optimized, compiled code)
- Benefits from kernel scheduler integration
- MCS lock algorithm efficiency
- Direct hardware access

## Performance Optimization Tips

1. **Choosing Lock Strategy**
   - Use `global` for baseline
   - Use `per-bucket` for hash tables at >16 threads
   - Use `per-node` for tree traversal workloads

2. **Thread Count Selection**
   - Start with low threads (1-4) to establish baseline
   - Increase to CPU count for saturation
   - Go beyond CPU count for oversubscription tests

3. **Workload Tuning**
   - `num_test_keys`: More keys = longer traversals, more contention
   - Lower keys (10-100) test lock overhead dominance
   - Higher keys (1000+) test data structure traversal

4. **Runtime Duration**
   - 5-10 seconds adequate for stable measurements
   - Longer runs (20+s) better if high variance observed
   - Very short runs (<2s) may have larger measurement artifacts

## Configuration File (`config.yaml`)

Pre-defined test scenarios for common use cases:

- **lightweight**: 1-8 threads, 5s runtime, 100 keys (single socket test)
- **medium**: 16-64 threads, 10s runtime, 500 keys (multi-core)
- **heavy**: 128-256 threads, 20s runtime, 1000 keys (high contention)

Run all scenarios:
```bash
python3 benchmark.py --config config.yaml
```

## Output Formats

### CSV Output
```
data_struct,lock_strategy,threads,time_to_run_sec,num_test_keys,kernel_ops_per_sec,ebpf_ops_per_sec,speedup
list,global,4,5,100,150000.5,123456.2,1.22
...
```

### JSON Output
```json
[
  {
    "data_struct": "list",
    "lock_strategy": "global",
    "threads": 4,
    "time_to_run_sec": 5,
    "num_test_keys": 100,
    "kernel_ops_per_sec": 150000.5,
    "ebpf_ops_per_sec": 123456.2,
    "speedup": 1.22
  }
]
```

## Key Features

### ✓ Configurable Concurrency
- 1-256 threads supported
- Per-CPU binding for isolation
- Workqueue-based thread management

### ✓ Multiple Data Structures
- Linked list with pointer traversal
- Hash table with buckets
- Balanced AVL tree with nested locking

### ✓ Flexible Locking Strategies
- Global spinlock
- Per-element/per-bucket protection
- Hand-over-hand locking for nested tests

### ✓ Accurate Measurement
- Per-CPU counters eliminate atomic overhead
- Kernel timing via `ktime_get()`
- Clear separation of setup/measurement/cleanup phases

### ✓ eBPF Nested Lock Testing
- Tree traversal demonstrates nested lock capability
- Hand-over-hand lock acquisition pattern
- Verifier approval captured in compilation

## Troubleshooting

### Module Compilation Errors
- Ensure kernel headers match running kernel: `uname -r`
- Try: `sudo apt install linux-headers-$(uname -r)`

### Permission Denied Errors
- Benchmark requires root: `sudo insmod` module
- Alternative: Run Python harness with sudo if using automated loading

### High Variance in Results
- Increase `time_to_run_sec` (20+ seconds recommended)
- Reduce system load: close unnecessary applications
- Use CPU isolation/pinning for more consistent results

### Invalid Combination Warnings
- Only these are supported:
  - list: [global, per-element]
  - hashtbl: [global, per-bucket]
  - tree: [global, per-node]

## Nested Lock Implementation Details (eBPF)

The tree data structure specifically tests eBPF's nested locking capability:

```c
// Hand-over-hand locking during tree traversal
while (current) {
    bpf_spin_lock(&current->lock);
    
    if (key == current->key) {
        current->value = value;
        bpf_spin_unlock(&current->lock);
        if (parent) bpf_spin_unlock(&parent->lock);
        break;
    }
    
    if (parent)
        bpf_spin_unlock(&parent->lock);
    parent = current;
    current = next_node;  // Traverses with nested lock held
}
```

This demonstrates:
- **Nested Locks**: Parent lock held while acquiring child lock
- **OOO Unlocking**: Parent unlocked before all children
- **Verifier Approval**: Program passes BPF verifier checks
- **Correctness**: No deadlocks, safe lock ordering

## Related Kernel Code References

- **eBPF Spinlock Support**: `kernel/bpf/helpers.c` (bpf_spin_lock/unlock)
- **Kernel qspinlock**: `kernel/locking/qspinlock.c`
- **Arena Memory**: `kernel/bpf/arena.c`
- **Spinlock Torture Tests**: `kernel/locking/locktorture.c`
- **BPF Verifier**: `kernel/bpf/verifier.c` (nested lock tracking)

## Future Enhancements

Possible improvements:
- [ ] Adaptive test duration based on variance
- [ ] Memory consumption profiling
- [ ] Lock contention statistics per strategy
- [ ] Comparison with Reader-Writer locks
- [ ] GPU/SIMD-accelerated benchmark runner
- [ ] Real-time latency measurements (p50, p99)
- [ ] Integration with `perf` for flame graphs

## License

GPL v2 (kernel module requirement)

## Author

Linux Kernel Spinlock Benchmark Suite
