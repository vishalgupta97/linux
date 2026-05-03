#!/bin/bash
# Parameter sweep for the BPF spinlock undo-log benchmark.
# Runs all DS × variant combinations across a range of pool sizes.
# Output: CSV to stdout; progress messages to stderr.
#
# Usage: sudo ./run_bench_spinlock.sh [output_file]
#        sudo ./run_bench_spinlock.sh results.csv

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH="${SCRIPT_DIR}/../bench_spinlock"
KMOD="${SCRIPT_DIR}/../bench_kmod/bench_spinlock_kmod.ko"

OUTPUT="${1:-/dev/stdout}"

if [ ! -x "$BENCH" ]; then
	echo "ERROR: $BENCH not found. Build first:" >&2
	echo "  make -C tools/testing/selftests/bpf bench_spinlock" >&2
	exit 1
fi

if [ ! -f "$KMOD" ]; then
	echo "ERROR: $KMOD not found. Build first:" >&2
	echo "  make -C tools/testing/selftests/bpf bench_kmod/bench_spinlock_kmod.ko" >&2
	exit 1
fi

# Load module if not already loaded
if ! lsmod | grep -q bench_spinlock_kmod; then
	echo "Loading bench_spinlock_kmod.ko ..." >&2
	insmod "$KMOD"
	trap 'echo "Unloading module..."; rmmod bench_spinlock_kmod 2>/dev/null' EXIT
fi

# Header printed once by the first bench_spinlock invocation
HEADER_PRINTED=0
THREADS=$(nproc)

run_bench() {
	local pool="$1"
	local extra_args="${2:-}"

	"$BENCH" \
		--threads "$THREADS" \
		--pool "$pool" \
		--warmup-ms 5000 \
		--bench-ms 15000 \
		$extra_args
}

{
	# First run: print header + all DS/variants, low contention
	run_bench 256

	# High contention: pool size = 1 (all threads fight for 1 element)
	run_bench 1 --variant kmod
	run_bench 1 --variant undo_log
	run_bench 1 --variant arena

	# Sweep pool sizes (no header repeat — awk filters it out)
	for pool in 8 32 128; do
		run_bench "$pool"
	done
} | awk 'NR==1 || !/^variant/' > "$OUTPUT"

echo "Done. Results written to: $OUTPUT" >&2
