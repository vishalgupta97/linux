// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#ifndef __FDS_H__
#define __FDS_H__

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fds.h>
#include <linux/hashtable.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/combiner.h>
#include <linux/seq_file.h>
#include <linux/sched/clock.h>
#include <linux/math.h>
#include <linux/cpumask.h>

#define HASHTABLE_BITS 5
#define PRINT_COUNT_LIMIT 1000
#define IS_VALUE 1
#define IS_DIRECTION 2
#define QSPINLOCK_LIMIT 10000
#define QSPINLOCK_PER_SECOND 15000
#define MUTEX_PER_SECOND 15000
#define MAX_CONTENDING_LOCKS 10
#define NELEMS(x) (sizeof(x) / sizeof((x)[0]))
#define MAX_STATES 1024

struct lock_stat {
	union {
		struct {
			struct fds_lock_key *key;
			const char *name;
			uint64_t counter;
			uint64_t read_counter;
			struct hlist_node hnode;
			uint64_t min_counter;
			uint64_t max_counter;
			uint64_t total_cs_time;
			uint64_t total_cs_count;
			uint64_t min_avg_cs_time;
			uint64_t max_avg_cs_time;
			cpumask_t contending_cpus;
		};
		char alignment[256];
	};
};

enum fds_lock_type {
	FDS_SPINLOCK,
	FDS_MUTEX,
	FDS_READ_SEM,
	FDS_WRITE_SEM,
};

enum HASHTABLE_TYPE {
	READ_HASHTABLE,
	WRITE_HASHTABLE,
	SPIN_HASHTABLE,
	MUTEX_HASHTABLE,
};

struct contending_locks {
	enum fds_lock_type ltype;
	struct lock_stat *lock;
};

extern long fds_monitor_time;
extern bool fds_oracle_running;
extern bool fds_running;

extern struct contending_locks observed_locks[MAX_CONTENDING_LOCKS];
extern long num_contending_locks;

DECLARE_PER_CPU_ALIGNED(struct lock_stat, write_lock_stats[NUM_BUCKETS]);
DECLARE_PER_CPU_ALIGNED(struct lock_stat, spin_lock_stats[NUM_BUCKETS]);
DECLARE_PER_CPU_ALIGNED(struct lock_stat, mutex_lock_stats[NUM_BUCKETS]);

extern DECLARE_HASHTABLE(write_stats_ht, HASHTABLE_BITS);
extern DECLARE_HASHTABLE(spin_stats_ht, HASHTABLE_BITS);
extern DECLARE_HASHTABLE(mutex_stats_ht, HASHTABLE_BITS);

extern spinlock_t stat_ht_lock;

const char *get_str_lockm(enum fds_lock_mechanisms lockm);
const char *get_str_ltype(enum fds_lock_type ltype);
enum fds_lock_mechanisms
get_optimal_spinlock_random_forest_classifier(struct lock_stat *tmp);
enum fds_lock_mechanisms
get_optimal_rwsem_random_forest_classifier(struct lock_stat *tmp);
extern void reset_fds(void);
extern void collect_fds_stats(void);
extern void __reset_fds_stats(struct lock_stat *tmp);
extern void reset_fds_stats(void);

#endif //__FDS_H__