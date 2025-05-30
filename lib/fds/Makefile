HOSTNAME := $(shell hostname | tail -c 2)

obj-y += fds.o model.o oracle.o proc.o util.o collect.o
CFLAGS_model.o := -DFDS_HOSTNAME='$(HOSTNAME)' -DFDS_SRV1='1' -DFDS_SRV8='8' -DFDS_SRV9='9'
