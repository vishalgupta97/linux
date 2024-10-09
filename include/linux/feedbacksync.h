// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#ifndef _LINUX_FEEDBACKSYNC_H
#define _LINUX_FEEDBACKSYNC_H

#define NUM_BUCKETS 8192

struct fds_lock_key {
        struct fds_lock_key *ptr;
        const char *name;
};

void __init feedback_sync_init(void);
void read_stat_lock_acquire(struct fds_lock_key *key);
void write_stat_lock_acquire(struct fds_lock_key *key);
void print_fds_stats(void);

#endif