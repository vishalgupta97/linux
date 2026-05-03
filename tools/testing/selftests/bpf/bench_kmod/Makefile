BENCH_KMOD_DIR := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
KDIR ?= $(abspath $(BENCH_KMOD_DIR)/../../../../..)

ifeq ($(V),1)
Q =
else
Q = @
endif

obj-m := bench_spinlock_kmod.o

all:
	$(Q)$(MAKE) -C $(KDIR) M=$(BENCH_KMOD_DIR) modules

clean:
	$(Q)$(MAKE) -C $(KDIR) M=$(BENCH_KMOD_DIR) clean
