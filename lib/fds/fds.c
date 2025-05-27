// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#include "fds.h"

static bool fds_running = false;

long fds_monitor_time = 5000; // In milli seconds

/*
	------------------------- Lock Stat Collector -------------------
*/

DEFINE_PER_CPU_ALIGNED(struct lock_stat, read_lock_stats[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(struct lock_stat, write_lock_stats[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(struct lock_stat, spin_lock_stats[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(struct lock_stat, mutex_lock_stats[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(bool, bucket_usage[NUM_BUCKETS]);
DEFINE_PER_CPU_ALIGNED(uint64_t, collisions);

enum HASHTABLE_TYPE {
	READ_HASHTABLE,
	WRITE_HASHTABLE,
	SPIN_HASHTABLE,
	MUTEX_HASHTABLE,
};

__always_inline void init_fds_lock_key(struct fds_lock_key *key,
				       const char *_name,
				       enum fds_lock_mechanisms _lockm)
{
	int i;
	if (key->name == NULL) {
		key->name = _name;
		key->lockm = _lockm;
		for (i = 0; i < (FDS_MAX_CPUS * 8); i++)
			key->bucket[i] = 0;
	}
}

__always_inline struct lock_stat *get_stat_ptr(uint64_t bucket,
					       enum HASHTABLE_TYPE ht_type)
{
	switch (ht_type) {
	case READ_HASHTABLE:
		//return this_cpu_ptr(&read_lock_stats[bucket]);
	case WRITE_HASHTABLE:
		return this_cpu_ptr(&write_lock_stats[bucket]);
	case SPIN_HASHTABLE:
		return this_cpu_ptr(&spin_lock_stats[bucket]);
	case MUTEX_HASHTABLE:
		return this_cpu_ptr(&mutex_lock_stats[bucket]);
	}
	return NULL;
}

void __stat_lock_time(struct fds_lock_key *key, enum HASHTABLE_TYPE ht_type,
		      uint64_t time)
{
	if (!fds_running || key == NULL ||
	    time <= 0) // || key->lockm == FDS_DISABLE)
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
		       "CHECK cpuid: %d bucket: %ld addr1: %px addr2: %px name: %s\n",
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

void __stat_lock_acquire(struct fds_lock_key *key, enum HASHTABLE_TYPE ht_type)
{
	if (!fds_running || key == NULL) // || key->lockm == FDS_DISABLE)
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
		       "CHECK cpuid: %d bucket: %ld addr1: %px addr2: %px name: %s\n",
		       smp_processor_id(), bucket, stat_ptr->key, key,
		       key->name);
		BUG_ON(true);
	}

	bucket = (((uint64_t)key) & (0x1fff));
	while (bucket < NUM_BUCKETS) {
		stat_ptr = get_stat_ptr(bucket, ht_type);
		if (stat_ptr->key == NULL) {
			if (stat_ptr->counter > 0)
				this_cpu_inc(collisions);
			stat_ptr->key = key;
			stat_ptr->counter = 1;
			if (ht_type == READ_HASHTABLE)
				stat_ptr->read_counter = 1;
			stat_ptr->name = kstrdup(key->name, GFP_KERNEL);
			key->bucket[smp_processor_id() * 8] = bucket;
			//printk(KERN_ALERT "ALLOCATED cpuid: %d bucket: %ld addr1: %px addr2: %px name: %s\n",
			//   smp_processor_id(), bucket, stat_ptr->key, key, key->name);
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
	//*this_cpu_ptr(&bucket_usage[bucket]) = true;
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

static DEFINE_SPINLOCK(stat_ht_lock);

DEFINE_HASHTABLE(write_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(spin_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(mutex_stats_ht, HASHTABLE_BITS);

inline void set_min_max_counter(struct lock_stat *stat, struct lock_stat *tmp)
{
	if (tmp->max_counter == 0) {
		tmp->max_counter = stat->counter;
		tmp->min_counter = stat->counter;
		return;
	}
	if (stat->counter > tmp->max_counter)
		tmp->max_counter = stat->counter;
	if (stat->counter < tmp->min_counter)
		tmp->min_counter = stat->counter;
}

inline void set_min_max_cs_time(struct lock_stat *stat, struct lock_stat *tmp)
{
	if (stat->total_cs_count == 0)
		return;
	uint64_t avg_cs_time = (stat->total_cs_time / stat->total_cs_count);
	if (tmp->max_avg_cs_time == 0) {
		tmp->max_avg_cs_time = avg_cs_time;
		tmp->min_avg_cs_time = avg_cs_time;
	}
	if (avg_cs_time > tmp->max_avg_cs_time)
		tmp->max_avg_cs_time = avg_cs_time;
	if (avg_cs_time < tmp->min_avg_cs_time)
		tmp->min_avg_cs_time = avg_cs_time;
}

inline bool __collect_fds_stats(struct lock_stat *stat, struct lock_stat *tmp,
				int cpu)
{
	if (stat == tmp) {
		if (stat->counter > 0) {
			cpumask_set_cpu(cpu, &tmp->contending_cpus);
			set_min_max_counter(stat, tmp);
			set_min_max_cs_time(stat, tmp);
		}
		return true;
	}
	if (stat->key != NULL && tmp->key != NULL) {
		if (stat->key == tmp->key) {
			if (stat->counter > 0) {
				tmp->counter += stat->counter;
				tmp->read_counter += stat->read_counter;
				tmp->total_cs_time += stat->total_cs_time;
				tmp->total_cs_count += stat->total_cs_count;
				cpumask_set_cpu(cpu, &tmp->contending_cpus);
				set_min_max_counter(stat, tmp);
				set_min_max_cs_time(stat, tmp);
			}
			stat->counter = 0;
			stat->read_counter = 0;
			stat->total_cs_time = 0;
			stat->total_cs_count = 0;
			stat->min_counter = 0;
			stat->max_counter = 0;
			stat->min_avg_cs_time = 0;
			stat->max_avg_cs_time = 0;
			return true;
		}
	}
	return false;
}

// void __collect_fds_stats(struct hlist_head (*p)[32], struct lock_stat *stat) {
// 	int bkt;
// 	struct lock_stat *tmp;

// 	if (stat->counter > 0) {
// 		hash_for_each(*p, bkt, tmp, hnode) {
// 			if (stat == tmp)
// 				return;

// 			if (stat->key != NULL && tmp->key != NULL) {
// 				if (stat->key == tmp->key) {
// 					tmp->counter += stat->counter;
// 					stat->counter = 0;
// 					return;
// 				}
// 			}
// 		}
// 		hash_add(*p, &stat->hnode, stat->key);
// 	}
// }

void collect_fds_stats(void)
{
	int i, j, bkt;
	struct lock_stat *stat, *tmp;

	// for (j = 0; j < NUM_BUCKETS; j++) {
	// 	for_each_online_cpu(i) {
	// 		stat = per_cpu_ptr(&write_lock_stats[j], i);
	// 		__collect_fds_stats(&write_stats_ht, stat);
	// 		stat = per_cpu_ptr(&spin_lock_stats[j], i);
	// 		__collect_fds_stats(&spin_stats_ht, stat);
	// 		stat = per_cpu_ptr(&mutex_lock_stats[j], i);
	// 		__collect_fds_stats(&mutex_stats_ht, stat);
	// 	}
	// }

	// __collect_fds_stats(&write_stats_ht, &write_lock_stats);
	// __collect_fds_stats(&spin_stats_ht, &spin_lock_stats);
	// __collect_fds_stats(&mutex_stats_ht, &mutex_lock_stats);

	spin_lock(&stat_ht_lock);

	for (j = 0; j < NUM_BUCKETS; j++) {
		for_each_online_cpu(i) {
			stat = per_cpu_ptr(&write_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(write_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp,
								    i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i,
							&stat->contending_cpus);
					set_min_max_counter(stat, stat);
					set_min_max_cs_time(stat, stat);
					hash_add(write_stats_ht, &stat->hnode,
						 stat->key);
				}
			}

			stat = per_cpu_ptr(&spin_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp,
								    i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i,
							&stat->contending_cpus);
					set_min_max_counter(stat, stat);
					set_min_max_cs_time(stat, stat);
					hash_add(spin_stats_ht, &stat->hnode,
						 stat->key);
				}
			}

			stat = per_cpu_ptr(&mutex_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp,
								    i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i,
							&stat->contending_cpus);
					set_min_max_counter(stat, stat);
					set_min_max_cs_time(stat, stat);
					hash_add(mutex_stats_ht, &stat->hnode,
						 stat->key);
				}
			}
		}
	}

	spin_unlock(&stat_ht_lock);
}

// void __reset_fds_stats(struct hlist_head (*p)[32]) {
// 	int bkt;
// 	struct lock_stat *tmp;

// 	hash_for_each(*p, bkt, tmp, hnode) {
// 		tmp->counter = 0;
// 	}
// }

void __reset_fds_stats(struct lock_stat *tmp)
{
	tmp->counter = 0;
	tmp->read_counter = 0;
	cpumask_clear(&tmp->contending_cpus);
	tmp->min_counter = 0;
	tmp->max_counter = 0;
	tmp->total_cs_count = 0;
	tmp->total_cs_time = 0;
	tmp->min_avg_cs_time = 0;
	tmp->max_avg_cs_time = 0;
}

void reset_fds_stats(void)
{
	// __reset_fds_stats(&write_stats_ht);
	// __reset_fds_stats(&spin_stats_ht);
	// __reset_fds_stats(&mutex_stats_ht);

	int bkt;
	struct lock_stat *tmp;

	spin_lock(&stat_ht_lock);

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		__reset_fds_stats(tmp);
	}
	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		__reset_fds_stats(tmp);
	}
	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		__reset_fds_stats(tmp);
	}

	spin_unlock(&stat_ht_lock);
}

inline void __print_fds_stats(struct lock_stat *tmp, const char *type,
			      uint64_t *count)
{
	if (tmp->counter) {
		if (tmp->counter > PRINT_COUNT_LIMIT ||
		    tmp->read_counter > PRINT_COUNT_LIMIT) {
			printk(KERN_ALERT
			       "%s Name: %s, Counter: %ld Max Counter: %ld Min Counter: %ld Read Counter: %ld Contending_CPUs: %d\n",
			       type, tmp->name, tmp->counter, tmp->max_counter,
			       tmp->min_counter, tmp->read_counter,
			       cpumask_weight(&tmp->contending_cpus));
			printk(KERN_ALERT
			       "%s Name: %s, total_cs_time: %ld total_cs_count: %ld Average CS: %ld Max CS: %ld Min CS: %ld\n",
			       type, tmp->name, tmp->total_cs_time,
			       tmp->total_cs_count,
			       tmp->total_cs_count > 0 ? (tmp->total_cs_time /
							  tmp->total_cs_count) :
							 0,
			       tmp->max_avg_cs_time, tmp->min_avg_cs_time);
		}
		*count = *count + 1;
	}
}

void print_fds_stats(void)
{
	printk(KERN_ALERT "======== Feedback sync stats ========\n");
	int bkt;
	struct lock_stat *tmp;
	uint64_t rcount = 0, wcount = 0, scount = 0, mcount = 0;

	// collect_fds_stats();

	printk(KERN_ALERT "Traversal done\n");

	spin_lock(&stat_ht_lock);

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		__print_fds_stats(tmp, "WRITE", &wcount);
	}

	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		__print_fds_stats(tmp, "SPIN", &scount);
	}

	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		__print_fds_stats(tmp, "MUTEX", &mcount);
	}

	spin_unlock(&stat_ht_lock);

	printk(KERN_ALERT
	       "Read locks: %ld Write locks: %ld Spin locks: %ld Mutex locks: %ld\n",
	       rcount, wcount, scount, mcount);
}

/*
	------------------------- Lock Switcher -------------------
*/

uint64_t value = 0;
uint64_t direction = 0;

static struct task_struct *fdsthreads;

static ssize_t fds_write(const char __user *buffer, size_t count, int type)
{
	char buf[64];

	if (count > 64)
		return -EINVAL;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	if (type == IS_VALUE)
		kstrtoll(buf, 0, &value);
	else if (type == IS_DIRECTION)
		kstrtoll(buf, 0, &direction);
	else
		return -EOPNOTSUPP;

	printk(KERN_ALERT "value: %lld, direction: %lld\n", value, direction);

	if (IS_VALUE) {
	}

	return count;
}

static void reset_fds(void)
{
	int bkt;
	struct lock_stat *tmp;

	printk(KERN_ALERT "Resetting FDS\n");

	collect_fds_stats(); // To reset all the per-CPU counters.

	spin_lock(&stat_ht_lock);

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		tmp->key->lockm = DEFAULT_FDS_LOCK;
		__reset_fds_stats(tmp);
	}
	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		tmp->key->lockm = DEFAULT_FDS_LOCK;
		__reset_fds_stats(tmp);
	}
	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		tmp->key->lockm = DEFAULT_FDS_LOCK;
		__reset_fds_stats(tmp);
	}

	spin_unlock(&stat_ht_lock);
}

struct contending_locks {
	enum fds_lock_type ltype;
	struct lock_stat *lock;
};

static long iterations = -1;
static long num_contending_locks = 0;

static struct contending_locks observed_locks[MAX_CONTENDING_LOCKS];

inline void __monitor_fds_stats(struct lock_stat *tmp, const char *type,
				enum fds_lock_type ltype)
{
	int i, j;
	long feature_vector[8];
	int max_value, max_index;
	enum fds_lock_mechanisms before = tmp->key->lockm;
	if (tmp->counter > QSPINLOCK_LIMIT ||
	    tmp->read_counter > QSPINLOCK_LIMIT || before != FDS_QSPINLOCK) {
		if (before == FDS_DISABLE)
			return;

		switch (ltype) {
		case FDS_SPINLOCK:
			tmp->key->lockm =
				get_optimal_spinlock_random_forest_classifier(
					tmp);
			break;
		case FDS_WRITE_SEM:
			tmp->key->lockm =
				get_optimal_rwsem_random_forest_classifier(tmp);
			break;
		case FDS_MUTEX:
			tmp->key->lockm =
				get_optimal_spinlock_random_forest_classifier(
					tmp);
			break;
		}
		printk(KERN_ALERT
		       "Flipping %s write Name: %s, Counter: %ld before: %s new: %s\n",
		       type, tmp->name, tmp->counter, get_str_lockm(before),
		       get_str_lockm(tmp->key->lockm));
	} else {
		tmp->key->lockm = FDS_QSPINLOCK;
	}
}

void monitor_fds_stats(void)
{
	int bkt;
	struct lock_stat *tmp;

	spin_lock(&stat_ht_lock);

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "WRITE SEM", FDS_WRITE_SEM);
	}

	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "SPINLOCK", FDS_SPINLOCK);
	}

	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "MUTEX", FDS_MUTEX);
	}

	spin_unlock(&stat_ht_lock);
}

int fds_monitor(void *args)
{
	printk(KERN_ALERT "Starting fds monitor\n");
	while (!kthread_should_stop()) {
		msleep(fds_monitor_time);

		if (fds_oracle_running)
			goto monitor_end;

		preempt_disable();
		print_komb_stats();
		collect_fds_stats();
		print_fds_stats();
		monitor_fds_stats();
		reset_fds_stats();
		preempt_enable();
monitor_end:
		if (need_resched())
			cond_resched();
	}

	return 0;
}

static int __init feedback_sync_init(void)
{
	fds_running = true;

	proc_mkdir("fds", NULL);
	proc_create("fds/value", 0222, NULL, &value_proc_ops);
	proc_create("fds/direction", 0222, NULL, &direction_proc_ops);
	proc_create("fds/reset", 0222, NULL, &reset_fds_proc_ops);
	proc_create("fds/getstat", 0444, NULL, &get_fds_proc_ops);
	proc_create("fds/oracle", 0222, NULL, &oracle_proc_ops);
	proc_create("fds/fds_monitor_time", 0222, NULL,
		    &fds_monitor_time_proc_ops);

	komb_rwsem_init();

	fdsthreads = kthread_create(fds_monitor, NULL, "fds_monitor");
	kthread_bind(fdsthreads, 0);
	if (fdsthreads) {
		wake_up_process(fdsthreads);
		return 0;
	} else {
		printk(KERN_ALERT "failed to create fds threads\n");
		return -1;
	}
}

module_init(feedback_sync_init)
