// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/feedbacksync.h>
#include <linux/hashtable.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/combiner.h>

static bool fds_running = false;

/*
	------------------------- Lock Stat Collector -------------------
*/

struct lock_stat {
	struct fds_lock_key *key;
	const char *name;
	u64 counter;
	struct hlist_node hnode;
};

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

void __stat_lock_acquire(struct fds_lock_key *key, enum HASHTABLE_TYPE ht_type)
{
	if (!fds_running || key == NULL || key->ptr == NULL)
		return;

	uint64_t bucket = (((uint64_t)key->ptr) & (0x1fff));
	while (bucket < NUM_BUCKETS) {
		struct lock_stat *stat_ptr = NULL;
		if (ht_type == READ_HASHTABLE)
			stat_ptr = this_cpu_ptr(&read_lock_stats[bucket]);
		else if (ht_type == WRITE_HASHTABLE)
			stat_ptr = this_cpu_ptr(&write_lock_stats[bucket]);
		else if (ht_type == SPIN_HASHTABLE)
			stat_ptr = this_cpu_ptr(&spin_lock_stats[bucket]);
		else if (ht_type == MUTEX_HASHTABLE)
			stat_ptr = this_cpu_ptr(&mutex_lock_stats[bucket]);
		if (stat_ptr->key == NULL) {
			if (stat_ptr->counter > 0)
				this_cpu_inc(collisions);
			stat_ptr->key = key->ptr;
			stat_ptr->counter = 1;
			stat_ptr->name = kstrdup(key->name, GFP_KERNEL);
			goto out;
		} else if (stat_ptr->key == key->ptr) {
			stat_ptr->counter++;
			goto out;
		}
		bucket++;
	}

out:
	*this_cpu_ptr(&bucket_usage[bucket]) = true;
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

#define HASHTABLE_BITS 5

DEFINE_HASHTABLE(read_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(write_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(spin_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(mutex_stats_ht, HASHTABLE_BITS);

inline bool __collect_fds_stats(struct lock_stat *stat, struct lock_stat *tmp)
{
	if (stat == tmp) {
		return true;
	}
	if (stat->key != NULL && tmp->key != NULL) {
		if (stat->key == tmp->key) {
			tmp->counter += stat->counter;
			stat->counter = 0;
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
	// 		stat = per_cpu_ptr(&read_lock_stats[j], i);
	// 		__collect_fds_stats(&read_stats_ht, stat);
	// 		stat = per_cpu_ptr(&write_lock_stats[j], i);
	// 		__collect_fds_stats(&write_stats_ht, stat);
	// 		stat = per_cpu_ptr(&spin_lock_stats[j], i);
	// 		__collect_fds_stats(&spin_stats_ht, stat);
	// 		stat = per_cpu_ptr(&mutex_lock_stats[j], i);
	// 		__collect_fds_stats(&mutex_stats_ht, stat);
	// 	}
	// }

	// __collect_fds_stats(&read_stats_ht, &read_lock_stats);
	// __collect_fds_stats(&write_stats_ht, &write_lock_stats);
	// __collect_fds_stats(&spin_stats_ht, &spin_lock_stats);
	// __collect_fds_stats(&mutex_stats_ht, &mutex_lock_stats);

	for (j = 0; j < NUM_BUCKETS; j++) {
		for_each_online_cpu(i) {
			stat = per_cpu_ptr(&read_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(read_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp);
					if (found)
						break;
				}
				if (!found)
					hash_add(read_stats_ht, &stat->hnode,
						 stat->key);
			}

			stat = per_cpu_ptr(&write_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(write_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp);
					if (found)
						break;
				}
				if (!found)
					hash_add(write_stats_ht, &stat->hnode,
						 stat->key);
			}

			stat = per_cpu_ptr(&spin_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp);
					if (found)
						break;
				}
				if (!found)
					hash_add(spin_stats_ht, &stat->hnode,
						 stat->key);
			}

			stat = per_cpu_ptr(&mutex_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp);
					if (found)
						break;
				}
				if (!found)
					hash_add(mutex_stats_ht, &stat->hnode,
						 stat->key);
			}
		}
	}
}

// void __reset_fds_stats(struct hlist_head (*p)[32]) {
// 	int bkt;
// 	struct lock_stat *tmp;

// 	hash_for_each(*p, bkt, tmp, hnode) {
// 		tmp->counter = 0;
// 	}
// }

void reset_fds_stats(void)
{
	// __reset_fds_stats(&read_stats_ht);
	// __reset_fds_stats(&write_stats_ht);
	// __reset_fds_stats(&spin_stats_ht);
	// __reset_fds_stats(&mutex_stats_ht);

	int bkt;
	struct lock_stat *tmp;

	hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		tmp->counter = 0;
	}
	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		tmp->counter = 0;
	}
	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		tmp->counter = 0;
	}
	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		tmp->counter = 0;
	}
}

#define PRINT_COUNT_LIMIT 100000

void print_fds_stats(void)
{
	printk(KERN_ALERT "======== Feedback sync stats ========\n");
	int bkt;
	struct lock_stat *tmp;
	uint64_t rcount = 0, wcount = 0, scount = 0, mcount = 0;

	collect_fds_stats();

	printk(KERN_ALERT "Traversal done\n");
	hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		if (tmp->counter) {
			if (tmp->counter > PRINT_COUNT_LIMIT)
				printk(KERN_ALERT
				       "Read Name: %s, Counter: %ld\n",
				       tmp->name, tmp->counter);
			rcount++;
		}
	}

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		if (tmp->counter) {
			if (tmp->counter > PRINT_COUNT_LIMIT)
				printk(KERN_ALERT
				       "Write Name: %s, Counter: %ld\n",
				       tmp->name, tmp->counter);
			wcount++;
		}
	}

	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		if (tmp->counter) {
			if (tmp->counter > PRINT_COUNT_LIMIT)
				printk(KERN_ALERT
				       "Spin Name: %s, Counter: %ld\n",
				       tmp->name, tmp->counter);
			scount++;
		}
	}

	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		if (tmp->counter) {
			if (tmp->counter > PRINT_COUNT_LIMIT)
				printk(KERN_ALERT
				       "Mutex Name: %s, Counter: %ld\n",
				       tmp->name, tmp->counter);
			mcount++;
		}
	}

	printk(KERN_ALERT
	       "Read locks: %ld Write locks: %ld Spin locks: %ld Mutex locks: %ld\n",
	       rcount, wcount, scount, mcount);

	// for_each_possible_cpu(i) {
	// 	counter = 0;
	// 	for(j = 0; j < NUM_BUCKETS; j++)
	// 		if(*per_cpu_ptr(&bucket_usage[j],i))
	// 			counter++;
	// 	printk(KERN_ALERT "CPU: %d, collisions: %ld bucket_usage: %ld\n", i, *per_cpu_ptr(&collisions, i), counter);
	// }
}

/*
	------------------------- Lock Switcher -------------------
*/

#define IS_VALUE 1
#define IS_DIRECTION 2

#define QSPINLOCK_LIMIT 1000000
#define MONITOR_TIME 20000 // In milliseconds

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

	if (IS_VALUE)
		kstrtoll(buf, 0, &value);
	else if (IS_DIRECTION)
		kstrtoll(buf, 0, &direction);
	else
		return -EOPNOTSUPP;

	printk(KERN_ALERT "value: %lld, direction: %lld\n", value, direction);

	if (IS_VALUE) {
	}

	return count;
}

static ssize_t fds_value_write(struct file *file, const char __user *buffer,
			       size_t count, loff_t *pos)
{
	return fds_write(buffer, count, IS_VALUE);
}

static ssize_t fds_direction_write(struct file *file, const char __user *buffer,
				   size_t count, loff_t *pos)
{
	return fds_write(buffer, count, IS_DIRECTION);
}

static const struct proc_ops value_proc_ops = { .proc_write = fds_value_write };

static const struct proc_ops direction_proc_ops = {
	.proc_write = fds_direction_write,
};

inline void __monitor_fds_stats(struct lock_stat *tmp, const char *type,
				bool is_spinlock)
{
	if (tmp->counter > QSPINLOCK_LIMIT) {
		printk(KERN_ALERT
		       "Flipping %s write Name: %s, Counter: %ld initial: %d\n",
		       type, tmp->name, tmp->counter, tmp->key->ptr->lockm);
		if (is_spinlock) {
			tmp->key->ptr->lockm = ((tmp->key->ptr->lockm + 1) %
						FDS_TDLOCK); //FDS_LOCKM_MAX
		} else if (tmp->key->ptr->lockm == FDS_QSPINLOCK) {
			tmp->key->ptr->lockm = FDS_TCLOCK;
		}
	}
}

void monitor_fds_stats(void)
{
	int bkt;
	struct lock_stat *tmp;

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "RWSEM", false);
	}

	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "SPINLOCK", true);
	}

	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "MUTEX", false);
	}
}

int fds_monitor(void *args)
{
	printk(KERN_ALERT "Starting fds monitor\n");
	while (!kthread_should_stop()) {
		msleep(MONITOR_TIME);
		preempt_disable();
		print_komb_stats();
		collect_fds_stats();
		print_fds_stats();
		monitor_fds_stats();
		reset_fds_stats();
		preempt_enable();
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

	fdsthreads = kthread_create(fds_monitor, NULL, "fds_monitor");
	kthread_bind(fdsthreads, 95);
	if (fdsthreads) {
		wake_up_process(fdsthreads);
		return 0;
	} else {
		printk(KERN_ALERT "failed to create fds threads\n");
		return -1;
	}
}

module_init(feedback_sync_init)