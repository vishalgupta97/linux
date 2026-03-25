CLANG   ?= clang
BPFTOOL ?= bpftool
CC      ?= gcc

BPF_CFLAGS := -g -O2 -target bpf -D__TARGET_ARCH_x86 \
              -I/usr/include/x86_64-linux-gnu \
			  -I. \
			  -I../../testing/selftests/bpf

LDFLAGS := -lbpf -lelf -lz

all: loader

# 1. Compile BPF object
rcuhashbash.bpf.o: rcuhashbash.bpf.c vmlinux.h
	$(CLANG) $(BPF_CFLAGS) -c $< -o $@

# 2. Generate skeleton
rcuhashbash.skel.h: rcuhashbash.bpf.o
	$(BPFTOOL) gen skeleton $< > $@

# 3. Compile loader
loader: loader.c rcuhashbash.skel.h
	$(CLANG) -g -O2 -o $@ loader.c $(LDFLAGS)

# Generate vmlinux.h for the running kernel
vmlinux.h:
	$(BPFTOOL) btf dump file /sys/kernel/btf/vmlinux format c > $@

clean:
	rm -f *.o *.skel.h loader vmlinux.h


