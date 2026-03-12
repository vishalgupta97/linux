PWD := $(shell pwd)
KDIR ?= $(PWD)/../..

ifneq ($(KERNELRELEASE),)

obj-m += baseline.o withundolog.o withundologatomic.o withundologprefetch.o withundologstore.o
common_objs := locks/qspinlock.o \
               locks/aqs.o \
               locks/cna.o \
               locks/komb.o \
               timing_stats.o \
               lib/combiner.o

baseline-y := rcuht_baseline.o
baseline-y += $(common_objs)

withundolog-y := rcuht_withundolog.o
withundolog-y += $(common_objs)

withundologatomic-y := rcuht_withundologatomic.o
withundologatomic-y += $(common_objs)

withundologprefetch-y := rcuht_withundologprefetch.o
withundologprefetch-y += $(common_objs)

withundologstore-y := rcuht_withundologstore.o
withundologstore-y += $(common_objs)


baseline.o withundolog.o withundologatomic.o withundologprefetch.o withundologstore.o: private objtool := true

ccflags-y := -g -DDEBUG -DKERNEL_SYNCSTRESS -I$(src)/include -I$(src)/spinlock

else

all: CPUSEQ
	$(MAKE) -C $(KDIR) M=$(PWD) src=$(PWD) modules

CPUSEQ:
	$(Q)python3 ./script/gen-cpuseq.py

clean:
	$(Q)rm -f include/cpuseq.h
	sed -i 's/NUM_PREFETCH_LINES ./NUM_PREFETCH_LINES 6/' include/lib/combiner.h 

endif
