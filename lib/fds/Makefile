HOSTNAME := $(shell hostname | tail -c 2)

obj-y += fds.o model.o oracle.c proc.c util.c
CFLAGS_model.o := -DFDS_HOSTNAME='$(HOSTNAME)' -DFDS_SRV1='1' -DFDS_SRV8='8' -DFDS_SRV9='9'
