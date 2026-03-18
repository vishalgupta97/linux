# Makefile (for eBPF + loader)
CLANG   ?= clang
ARCH    := $(shell uname -m | sed 's/x86_64/x86/' | sed 's/aarch64/arm64/')
VMLINUX := vmlinux.h

all: fentry_mymodule.bpf.o loader

vmlinux.h:
	bpftool btf dump file /sys/kernel/btf/vmlinux format c > vmlinux.h

fentry_mymodule.bpf.o: fentry_mymodule.bpf.c $(VMLINUX)
	$(CLANG) -g -O2 -target bpf -D__TARGET_ARCH_$(ARCH) \
		-I. -c fentry_mymodule.bpf.c -o fentry_mymodule.bpf.o

loader: loader.c
	gcc -o loader loader.c -lbpf

clean:
	rm -f *.o loader vmlinux.h

