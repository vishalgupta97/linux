#!/bin/bash
# Quick start script for spinlock benchmark
# Demonstrates common usage patterns

set -e

BENCHMARK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LINUX_ROOT="$(dirname "$(dirname "${BENCHMARK_DIR}")")"
MODULE_DIR="${LINUX_ROOT}/modules/spinlock_bench"

echo "================================"
echo "eBPF vs Kernel Spinlock Benchmark"
echo "Quick Start Guide"
echo "================================"
echo ""

# Build kernel module
echo "[1/4] Building kernel module..."
cd "${MODULE_DIR}"
make clean
make
echo "✓ Kernel module built: spinlock_bench.ko"
echo ""

# Build eBPF program
echo "[2/4] Building eBPF program..."
cd "${LINUX_ROOT}/tools/testing/selftests/bpf"
# Note: Full BPF build requires full selftests setup
echo "✓ eBPF program sources ready at: tools/testing/selftests/bpf/progs/spinlock_bench_bpf.c"
echo ""

# Show module information
echo "[3/4] Module Information:"
file "${MODULE_DIR}/spinlock_bench.ko"
echo ""

# Run Python test harness (demonstration)
echo "[4/4] Test Harness Ready:"
cd "${BENCHMARK_DIR}"
python3 -c "import benchmark; print('✓ Python harness ready')"
echo ""

echo "================================"
echo "Next Steps:"
echo "================================"
echo ""
echo "1. Load kernel module with parameters:"
echo "   sudo insmod ${MODULE_DIR}/spinlock_bench.ko \\"
echo "     data_struct=0 lock_strategy=0 num_threads=4 time_to_run_sec=5 num_test_keys=100"
echo ""
echo "2. View results:"
echo "   dmesg | tail -20"
echo ""
echo "3. Or run automated benchmark:"
echo "   cd ${BENCHMARK_DIR}"
echo "   python3 benchmark.py --help"
echo ""
echo "4. Run specific test scenario:"
echo "   python3 benchmark.py \\"
echo "     --data-struct list \\"
echo "     --lock-strategy per-element \\"
echo "     --threads 4 \\"
echo "     --time-to-run 5 \\"
echo "     --output-csv results.csv"
echo ""
echo "5. Run all configured tests:"
echo "   python3 benchmark.py --config config.yaml"
echo ""
echo "================================"
echo "Data Structures & Lock Strategies:"
echo "================================"
echo ""
echo "Linked List:"
echo "  - global       (single lock for whole list)"
echo "  - per-element  (lock per node)"
echo ""
echo "Hash Table:"
echo "  - global       (single lock)"
echo "  - per-bucket   (lock per bucket)"
echo ""
echo "Tree (AVL with hand-over-hand locking):"
echo "  - global       (single lock at root)"
echo "  - per-node     (nested locks during traversal)"
echo ""
echo "================================"
echo "Module Parameters:"
echo "================================"
echo ""
echo "  data_struct=0    # 0=list, 1=hash, 2=tree"
echo "  lock_strategy=0  # 0=global, 1=per-elem, 2=per-bucket, 3=per-node"
echo "  num_threads=4    # 1-256"
echo "  time_to_run_sec=5 # Duration"
echo "  num_test_keys=100 # Keys for update operations"
echo ""
