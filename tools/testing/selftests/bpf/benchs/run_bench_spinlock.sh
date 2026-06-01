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

run_bench() {
	local thread="$1"
	local pool="$2"
	local op="$3"
	local init_size="$4"
	local variant="$5"

	"$BENCH" \
		--threads "$thread" \
		--pool "$pool" \
		--op "$op" \
		--init-size "$init_size" \
		--warmup-ms 5000 \
		--bench-ms 30000
}

{
	for thread in 1 2 4 8 16 32 64 80 96 112 128; do
		for pool in 128; do #1 8 32 128; do
			for op in insert; do  #lookup update delete; do
				for variant in kmod kmod_bpf; do
				# For insert, no prefill; for read/write/delete ops
				# prefill to full pool capacity.
				if [ "$op" = "insert" ]; then
					init_size=0
				else
					init_size=$pool
				fi
				run_bench "$thread" "$pool" "$op" "$init_size" "$variant"
				done
			done
		done
	done
} | awk 'NR==1 || !/^variant/' > "$OUTPUT"

echo "Done. Results written to: $OUTPUT" >&2
