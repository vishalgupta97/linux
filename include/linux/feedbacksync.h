// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#ifndef _LINUX_FEEDBACKSYNC_H
#define _LINUX_FEEDBACKSYNC_H

#define NUM_BUCKETS 8192
#define FDS_MAX_CPUS 256
#define DEFAULT_FDS_LOCK FDS_QSPINLOCK

enum fds_lock_mechanisms {
	FDS_QSPINLOCK 	= (1 << 0),
	FDS_CNA 	= (1 << 1),
	FDS_SHFLLOCK    = (1 << 2),
	FDS_TCLOCK	= (1 << 3),
	FDS_TDLOCK	= (1 << 4),
	FDS_LOCKM_MAX	= (1 << 5),
	FDS_PERCPU	= (1 << 6),
	FDS_DISABLE	= (1 << 7),
};

struct fds_lock_key {
	const char *name;
	uint64_t bucket[FDS_MAX_CPUS * 8];
	enum fds_lock_mechanisms lockm;
};

#define __FDS_LOCK_KEY_INITIALIZER(keyname) \
{\
	.name = #keyname, \
	.lockm = FDS_QSPINLOCK \
}

void init_fds_lock_key(struct fds_lock_key *key, const char* _name, enum fds_lock_mechanisms _lockm);

void read_stat_lock_acquire(struct fds_lock_key *key);
void write_stat_lock_acquire(struct fds_lock_key *key);
void mutex_stat_lock_acquire(struct fds_lock_key *key);
void spin_stat_lock_acquire(struct fds_lock_key *key);
void print_fds_stats(void);

#endif
