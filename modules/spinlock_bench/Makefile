# Spinlock Benchmark Module Makefile
# When building within the kernel tree, use the tree's kernel version
# Otherwise use the running kernel's headers

KDIR ?= $(PWD)/../..
ifeq ($(KDIR),$(PWD)/../..)
  # Check if we're in the kernel tree
  ifeq ($(shell test -f $(KDIR)/Makefile && echo yes),yes)
    # We're in the kernel tree
  else
    # Fall back to running kernel's build directory
    KDIR ?= /lib/modules/$(shell uname -r)/build
  endif
endif

obj-m := spinlock_bench.o
spinlock_bench-objs := spinlock_bench_qspinlock.o

.PHONY: all clean

all:
	@echo "Building spinlock_bench module..."
	@echo "Using KDIR: $(KDIR)"
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
