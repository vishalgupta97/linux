// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#include "fds.h"

bool fds_running = false;
long fds_monitor_time = 5000; // In milli seconds

DEFINE_SPINLOCK(stat_ht_lock);

DEFINE_HASHTABLE(write_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(spin_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(mutex_stats_ht, HASHTABLE_BITS);

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

__always_inline struct lock_stat *
get_cpu_stat_ptr(enum HASHTABLE_TYPE ht_type, uint64_t cpu, uint64_t bucket)
{
	switch (ht_type) {
	case READ_HASHTABLE:
	case WRITE_HASHTABLE:
		return per_cpu_ptr(&write_lock_stats[bucket], cpu);
	case SPIN_HASHTABLE:
		return per_cpu_ptr(&spin_lock_stats[bucket], cpu);
	case MUTEX_HASHTABLE:
		return per_cpu_ptr(&mutex_lock_stats[bucket], cpu);
	}
	return NULL;
}

inline bool ___collect_fds_stats(struct lock_stat *stat, struct lock_stat *tmp,
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

__always_inline bool __collect_fds_stats(enum HASHTABLE_TYPE ht_type,
					 struct lock_stat *stat, int cpu)
{
	struct lock_stat *tmp;
	int bkt;
	bool found = false;
	switch (ht_type) {
	case READ_HASHTABLE:
	case WRITE_HASHTABLE:
		hash_for_each(write_stats_ht, bkt, tmp, hnode) {
			found = ___collect_fds_stats(stat, tmp, cpu);
			if (found)
				return found;
		}
		break;
	case SPIN_HASHTABLE:
		hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
			found = ___collect_fds_stats(stat, tmp, cpu);
			if (found)
				return found;
		}
		break;
	case MUTEX_HASHTABLE:
		hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
			found = ___collect_fds_stats(stat, tmp, cpu);
			if (found)
				return found;
		}
		break;
	}
	return found;
}

__always_inline void add_to_ht(enum HASHTABLE_TYPE ht_type,
			       struct lock_stat *stat)
{
	switch (ht_type) {
	case READ_HASHTABLE:
	case WRITE_HASHTABLE:
		hash_add(write_stats_ht, &stat->hnode, (uint64_t)stat->key);
		break;
	case SPIN_HASHTABLE:
		hash_add(spin_stats_ht, &stat->hnode, (uint64_t)stat->key);
		break;
	case MUTEX_HASHTABLE:
		hash_add(mutex_stats_ht, &stat->hnode, (uint64_t)stat->key);
		break;
	}
}

__always_inline void ht_collect_fds_stats(enum HASHTABLE_TYPE ht_type,
					  uint64_t cpu, uint64_t bucket)
{
	struct lock_stat *stat;
	stat = get_cpu_stat_ptr(ht_type, cpu, bucket);
	if (stat->counter > 0) {
		bool found = __collect_fds_stats(ht_type, stat, cpu);
		if (!found) {
			cpumask_set_cpu(cpu, &stat->contending_cpus);
			set_min_max_counter(stat, stat);
			set_min_max_cs_time(stat, stat);
			add_to_ht(ht_type, stat);
		}
	}
}

void collect_fds_stats(void)
{
	int i, j;

	spin_lock(&stat_ht_lock);

	for (j = 0; j < NUM_BUCKETS; j++) {
		for_each_online_cpu(i) {
			ht_collect_fds_stats(WRITE_HASHTABLE, i, j);
			ht_collect_fds_stats(SPIN_HASHTABLE, i, j);
			ht_collect_fds_stats(MUTEX_HASHTABLE, i, j);
		}
	}

	spin_unlock(&stat_ht_lock);
}

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
			       "%s Name: %s, Counter: %lld Max Counter: %lld Min Counter: %lld Read Counter: %lld Contending_CPUs: %d\n",
			       type, tmp->name, tmp->counter, tmp->max_counter,
			       tmp->min_counter, tmp->read_counter,
			       cpumask_weight(&tmp->contending_cpus));
			printk(KERN_ALERT
			       "%s Name: %s, total_cs_time: %lld total_cs_count: %lld Average CS: %lld Max CS: %lld Min CS: %lld\n",
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
	       "Read locks: %lld Write locks: %lld Spin locks: %lld Mutex locks: %lld\n",
	       rcount, wcount, scount, mcount);
}

/*
	------------------------- Lock Switcher -------------------
*/

inline void __monitor_fds_stats(struct lock_stat *tmp, const char *type,
				enum fds_lock_type ltype)
{
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
		default:
			break;
		}
		printk(KERN_ALERT
		       "Flipping %s write Name: %s, Counter: %lld before: %s new: %s\n",
		       type, tmp->name, tmp->counter, get_str_lockm(before),
		       get_str_lockm(tmp->key->lockm));
	} else {
		tmp->key->lockm = FDS_QSPINLOCK;
	}
}

static void monitor_fds_stats(void)
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

static int fds_monitor(void *args)
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

static struct task_struct *fdsthreads;

extern struct proc_ops value_proc_ops;
extern struct proc_ops direction_proc_ops;
extern struct proc_ops reset_fds_proc_ops;
extern struct proc_ops get_fds_proc_ops;
extern struct proc_ops oracle_proc_ops;
extern struct proc_ops fds_monitor_time_proc_ops;

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
