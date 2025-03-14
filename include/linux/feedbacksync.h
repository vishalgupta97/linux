// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#ifndef _LINUX_FEEDBACKSYNC_H
#define _LINUX_FEEDBACKSYNC_H

#define NUM_BUCKETS 8192

enum fds_lock_mechanisms {
	FDS_QSPINLOCK 	= (1 << 0),
	FDS_TAS 	= (1 << 1),
	FDS_TCLOCK	= (1 << 2),
	FDS_TDLOCK	= (1 << 3),
	FDS_LOCKM_MAX	= (1 << 4),
	FDS_PERCPU	= (1 << 5),
	FDS_DISABLE	= (1 << 6),
};

struct fds_lock_key {
	struct fds_lock_key *ptr;
	const char *name;
	enum fds_lock_mechanisms lockm;
};

#define DEFAULT_FDS_LOCK FDS_QSPINLOCK

#define init_fds_lock_key(key, _name, _lockm) \
	{                                     \
		if (key->ptr == NULL) {       \
			key->ptr = key;       \
			key->name = _name;    \
			key->lockm = _lockm;  \
		}                             \
	}

void read_stat_lock_acquire(struct fds_lock_key *key);
void write_stat_lock_acquire(struct fds_lock_key *key);
void mutex_stat_lock_acquire(struct fds_lock_key *key);
void spin_stat_lock_acquire(struct fds_lock_key *key);
void print_fds_stats(void);

#endif
