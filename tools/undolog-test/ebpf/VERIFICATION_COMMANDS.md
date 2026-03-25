# Arena Migration Verification Commands

Run these from:

cd /home/vishal/project/linux/tools/undolog-test/ebpf

## 1) Build artifacts

make clean && make

## 2) Confirm generated symbols (init program + arena map)

rg "init_entries_arena" rcuhashbash.skel.h
llvm-objdump -h rcuhashbash.bpf.o | rg "maps|.maps"

## 3) Loader smoke test

sudo ./loader --buckets 1024 --entries-per-bucket 8

## 4) Kernel log watch (run in a separate terminal)

cd /home/vishal/project/linux/tools/undolog-test/ebpf
sudo dmesg -w

## 5) Boundary / negative tests

sudo ./loader --buckets 1024 --entries-per-bucket 64
./loader --buckets 1024 --entries-per-bucket 65
./loader --buckets 0 --entries-per-bucket 8
./loader --buckets 1024

## 6) Check loaded programs/maps while loader is running

sudo bpftool prog show | rg "attach_cs_ht|init_entries_arena|rcuhashbash"
sudo bpftool map show | rg "arena|global_lock_map|rcuhashbash"

## 7) Benchmark flow validation

cd /home/vishal/project/linux/tools/undolog-test
./run-rcuht-ebpf-spinlock.sh

## 8) Optional: quick cleanup of generated files

cd /home/vishal/project/linux/tools/undolog-test/ebpf
make clean
