PWD := $(shell pwd)
KDIR ?= $(PWD)/../..

ifneq ($(KERNELRELEASE),)

obj-m += baseline.o withundolog.o
baseline-y := rcuht_baseline.o \
                  locks/qspinlock.o \
                  locks/aqs.o \
                  locks/cna.o \
                  locks/komb.o \
                  timing_stats.o \
                  lib/combiner.o
withundolog-y := rcuht_withundolog.o \
                  locks/qspinlock.o \
                  locks/aqs.o \
                  locks/cna.o \
                  locks/komb.o \
                  timing_stats.o \
                  lib/combiner.o

baseline.o withundolog.o: private objtool := true

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
