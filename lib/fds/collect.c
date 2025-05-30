// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

/*
	------------------------- Lock Stat Collector -------------------
*/

#include "fds.h"

DEFINE_PER_CPU_ALIGNED(struct lock_stat, write_lock_stats[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(struct lock_stat, spin_lock_stats[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(struct lock_stat, mutex_lock_stats[NUM_BUCKETS]);

__always_inline struct lock_stat *get_stat_ptr(uint64_t bucket,
					       enum HASHTABLE_TYPE ht_type)
{
	switch (ht_type) {
	case READ_HASHTABLE:
	case WRITE_HASHTABLE:
		return this_cpu_ptr(&write_lock_stats[bucket]);
	case SPIN_HASHTABLE:
		return this_cpu_ptr(&spin_lock_stats[bucket]);
	case MUTEX_HASHTABLE:
		return this_cpu_ptr(&mutex_lock_stats[bucket]);
	}
	return NULL;
}

static void __stat_lock_time(struct fds_lock_key *key,
			     enum HASHTABLE_TYPE ht_type, uint64_t time)
{
	if (!fds_running || key == NULL || time <= 0)
		return;

	struct lock_stat *stat_ptr = NULL;
	uint64_t bucket = key->bucket[smp_processor_id() * 8];

	if (bucket) {
		stat_ptr = get_stat_ptr(bucket, ht_type);
		if (stat_ptr->key == key) {
			stat_ptr->total_cs_time += time;
			stat_ptr->total_cs_count++;
			return;
		}
		printk(KERN_ALERT
		       "CHECK cpuid: %d bucket: %lld addr1: %px addr2: %px name: %s\n",
		       smp_processor_id(), bucket, stat_ptr->key, key,
		       key->name);
	}
	BUG_ON(true);
}

void read_stat_lock_time(struct fds_lock_key *key, uint64_t time)
{
	__stat_lock_time(key, READ_HASHTABLE, time);
}

void write_stat_lock_time(struct fds_lock_key *key, uint64_t time)
{
	__stat_lock_time(key, WRITE_HASHTABLE, time);
}

void mutex_stat_lock_time(struct fds_lock_key *key, uint64_t time)
{
	__stat_lock_time(key, MUTEX_HASHTABLE, time);
}

void spin_stat_lock_time(struct fds_lock_key *key, uint64_t time)
{
	__stat_lock_time(key, SPIN_HASHTABLE, time);
}

static void __stat_lock_acquire(struct fds_lock_key *key,
				enum HASHTABLE_TYPE ht_type)
{
	if (!fds_running || key == NULL)
		return;

	struct lock_stat *stat_ptr = NULL;
	uint64_t bucket = key->bucket[smp_processor_id() * 8];

	if (bucket) {
		stat_ptr = get_stat_ptr(bucket, ht_type);
		if (stat_ptr->key == key) {
			if (ht_type == READ_HASHTABLE)
				stat_ptr->read_counter++;
			else
				stat_ptr->counter++;
			goto out;
		}
		printk(KERN_ALERT
		       "CHECK cpuid: %d bucket: %lld addr1: %px addr2: %px name: %s\n",
		       smp_processor_id(), bucket, stat_ptr->key, key,
		       key->name);
		BUG_ON(true);
	}

	bucket = (((uint64_t)key) & (0x1fff));
	while (bucket < NUM_BUCKETS) {
		stat_ptr = get_stat_ptr(bucket, ht_type);
		if (stat_ptr->key == NULL) {
			stat_ptr->key = key;
			stat_ptr->counter = 1;
			if (ht_type == READ_HASHTABLE)
				stat_ptr->read_counter = 1;
			stat_ptr->name = kstrdup(key->name, GFP_KERNEL);
			key->bucket[smp_processor_id() * 8] = bucket;
			goto out;
		} else if (stat_ptr->key == key) {
			if (ht_type == READ_HASHTABLE)
				stat_ptr->read_counter++;
			else
				stat_ptr->counter++;
			goto out;
		}
		bucket++;
	}

out:
	return;
}

void read_stat_lock_acquire(struct fds_lock_key *key)
{
	__stat_lock_acquire(key, READ_HASHTABLE);
}

void write_stat_lock_acquire(struct fds_lock_key *key)
{
	__stat_lock_acquire(key, WRITE_HASHTABLE);
}

void mutex_stat_lock_acquire(struct fds_lock_key *key)
{
	__stat_lock_acquire(key, MUTEX_HASHTABLE);
}

void spin_stat_lock_acquire(struct fds_lock_key *key)
{
	__stat_lock_acquire(key, SPIN_HASHTABLE);
}