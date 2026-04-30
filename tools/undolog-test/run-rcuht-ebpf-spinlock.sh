source defaults.sh

#locks=(table_bpf_spinlock_baseline)
locks=(table_bpf_spinlock_undolog)
binaries=(baseline)

lock_type=spinlock

rw_writes=(100)
rw_total=100
buckets=(1024)
entries_ratios=(1 2 4 6 8)

time=${runtime}

parent_dir=${results_dir}

DIR=${parent_dir}/results-${lock_type}-${ncores}cores-${time}seconds

#make clean
make || exit

numlocks=${#locks[@]}

for binary in ${binaries[@]}
do
for bucket in ${buckets[@]}
do
for((i=0; i<$numlocks; i++))
do
	l=${locks[i]}
	for entry_ratio in ${entries_ratios[@]}
	do
		pushd ebpf
		sed -i "s/#define NUM_ENTRIES ./#define NUM_ENTRIES ${entry_ratio}/" rcuhashbash.bpf.c
		make
		popd
		entries=$((${bucket}*${entry_ratio}))
		for write in ${rw_writes[@]}
		do
			lock_name='modified_bpf_'$l'_'$binary
			out_dir=${DIR}/${bucket}buckets-${entries}entries/${run}/${lock_name}/${write}percent_writes
			echo ${out_dir}
			mkdir -p ${out_dir} || exit
			for c in ${cores[@]}
			do
				echo "lock: ${l} threads: ${c} rw_ratio: ${write} entries: ${entries} buckets: ${bucket}"
				sudo dmesg -C
				sudo insmod ${binary}.ko reader_type=$l writer_type=$l \
					ro=0 rw=$c \
					rw_writes=${write} rw_total=${rw_total} \
					buckets=${bucket} \
					entries=${entries}
				sudo ./ebpf/loader --buckets ${bucket} --entries-per-bucket ${entry_ratio} &
				loader_pid=$!
				sleep 2
				sudo ./send-ioctl 1
				sleep ${time}
				sudo ./send-ioctl 2
				kill -TERM ${loader_pid}
				wait ${loader_pid}
				sudo rmmod ${binary}.ko
				sleep 1
				sudo dmesg > ${out_dir}/core.${c}
				sleep 2
			done
		done
	done
done
done
done
