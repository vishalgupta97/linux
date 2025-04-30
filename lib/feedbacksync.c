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
#include <linux/seq_file.h>
#include <linux/sched/clock.h>
#include <linux/math.h>
#include <linux/cpumask.h>

//#include <fds/decision_tree.h>
//#include <fds/rwsem_decision_tree.h>

#include <fds/srv1_spinlock_random_forest.h>
#include <fds/srv1_rwsem_random_forest.h>
#include <fds/srv8_spinlock_random_forest.h>
#include <fds/srv8_rwsem_random_forest.h>
#include <fds/srv9_spinlock_random_forest.h>
#include <fds/srv9_rwsem_random_forest.h>

static bool fds_running = false;
static bool fds_oracle_running = false;

/*
	------------------------- Lock Stat Collector -------------------
*/

struct lock_stat {
	union {
		struct {
			struct fds_lock_key *key;
			const char *name;
			uint64_t counter;
                        uint64_t read_counter;
			struct hlist_node hnode;
			cpumask_t contending_cpus;
		};
		char alignment[128];
	};
} __cacheline_aligned_in_smp;

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

__always_inline void init_fds_lock_key(struct fds_lock_key *key, const char* _name, enum fds_lock_mechanisms _lockm)
{
        int i;        
        if (key->name == NULL) {      
                key->name = _name;    
                key->lockm = _lockm;
                for(i = 0; i < (FDS_MAX_CPUS * 8); i++)
                        key->bucket[i] = 0;  
        }                             
}

__always_inline struct lock_stat * get_stat_ptr(uint64_t bucket, enum HASHTABLE_TYPE ht_type) {
	switch(ht_type) {
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

void __stat_lock_acquire(struct fds_lock_key *key, enum HASHTABLE_TYPE ht_type)
{
	if (!fds_running || key == NULL) // || key->lockm == FDS_DISABLE)
		return;

	struct lock_stat *stat_ptr = NULL;
	uint64_t bucket = key->bucket[smp_processor_id() * 8];

	if(bucket) {
		stat_ptr = get_stat_ptr(bucket, ht_type);
		if(stat_ptr->key == key) {
                        if(ht_type == READ_HASHTABLE)
                                stat_ptr->read_counter++;
                        else
			        stat_ptr->counter++;
			goto out;	
		}
		printk(KERN_ALERT "CHECK cpuid: %d bucket: %ld addr1: %px addr2: %px name: %s\n", 
                        smp_processor_id(), bucket, stat_ptr->key, key, key->name);
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
                        if(ht_type == READ_HASHTABLE)
                                stat_ptr->read_counter = 1;
			stat_ptr->name = kstrdup(key->name, GFP_KERNEL);
			key->bucket[smp_processor_id() * 8] = bucket;
			//printk(KERN_ALERT "ALLOCATED cpuid: %d bucket: %ld addr1: %px addr2: %px name: %s\n", 
                        //   smp_processor_id(), bucket, stat_ptr->key, key, key->name);
			goto out;
		} else if (stat_ptr->key == key) {
                        if(ht_type == READ_HASHTABLE)
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
        //return;
	__stat_lock_acquire(key, READ_HASHTABLE);
}

void write_stat_lock_acquire(struct fds_lock_key *key)
{
        //return;
	__stat_lock_acquire(key, WRITE_HASHTABLE);
}

void mutex_stat_lock_acquire(struct fds_lock_key *key)
{
        //return;
	__stat_lock_acquire(key, MUTEX_HASHTABLE);
}

void spin_stat_lock_acquire(struct fds_lock_key *key)
{
	//return;
	__stat_lock_acquire(key, SPIN_HASHTABLE);
}

#define HASHTABLE_BITS 5

static DEFINE_SPINLOCK(stat_ht_lock);

//DEFINE_HASHTABLE(read_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(write_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(spin_stats_ht, HASHTABLE_BITS);
DEFINE_HASHTABLE(mutex_stats_ht, HASHTABLE_BITS);

inline bool __collect_fds_stats(struct lock_stat *stat, struct lock_stat *tmp, int cpu)
{
	if (stat == tmp) {
		if(stat->counter > 0)
			cpumask_set_cpu(cpu, &tmp->contending_cpus);

		return true;
	}
	if (stat->key != NULL && tmp->key != NULL) {
		if (stat->key == tmp->key) {
			if(stat->counter > 0) {
				tmp->counter += stat->counter;
				tmp->read_counter += stat->read_counter;
				cpumask_set_cpu(cpu, &tmp->contending_cpus);
			}
			stat->counter = 0;
			stat->read_counter = 0; 

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

	spin_lock(&stat_ht_lock);

	for (j = 0; j < NUM_BUCKETS; j++) {
		for_each_online_cpu(i) {
			/*stat = per_cpu_ptr(&read_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(read_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp, i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i, &stat->contending_cpus);
					hash_add(read_stats_ht, &stat->hnode,
						 stat->key);
				}
			}*/

			stat = per_cpu_ptr(&write_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(write_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp, i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i, &stat->contending_cpus);
					hash_add(write_stats_ht, &stat->hnode,
						 stat->key);
				}
			}

			stat = per_cpu_ptr(&spin_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp, i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i, &stat->contending_cpus);
					hash_add(spin_stats_ht, &stat->hnode,
						 stat->key);
				}
			}

			stat = per_cpu_ptr(&mutex_lock_stats[j], i);
			if (stat->counter > 0) {
				bool found = false;
				hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
					found = __collect_fds_stats(stat, tmp, i);
					if (found)
						break;
				}
				if (!found) {
					cpumask_set_cpu(i, &stat->contending_cpus);
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

void __reset_fds_stats(struct lock_stat *tmp) {
	tmp->counter = 0;
	tmp->read_counter = 0;
	cpumask_clear(&tmp->contending_cpus);
}

void reset_fds_stats(void)
{
	// __reset_fds_stats(&read_stats_ht);
	// __reset_fds_stats(&write_stats_ht);
	// __reset_fds_stats(&spin_stats_ht);
	// __reset_fds_stats(&mutex_stats_ht);

	int bkt;
	struct lock_stat *tmp;

	spin_lock(&stat_ht_lock);

	/*hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		__reset_fds_stats(tmp);
	}*/
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

#define PRINT_COUNT_LIMIT 1000

inline void __print_fds_stats(struct lock_stat *tmp, const char *type,
			      uint64_t *count)
{
	if (tmp->counter) {
		if (tmp->counter > PRINT_COUNT_LIMIT || tmp->read_counter > PRINT_COUNT_LIMIT)
			printk(KERN_ALERT "%s Name: %s, Counter: %ld Read Counter: %ld Contending_CPUs: %d\n", type,
			       tmp->name, tmp->counter, tmp->read_counter, cpumask_weight(&tmp->contending_cpus));
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

	/*hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		__print_fds_stats(tmp, "READ", &rcount);
	}*/

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

#define QSPINLOCK_LIMIT 10000
#define MONITOR_TIME 5000 // In milliseconds

#define QSPINLOCK_PER_SECOND 200000
#define MUTEX_PER_SECOND 150000

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

	/*hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		tmp->key->lockm = DEFAULT_FDS_LOCK;
		__reset_fds_stats(tmp);
	}*/
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

static ssize_t fds_value_write(struct file *file, const char __user *buffer,
			       size_t count, loff_t *pos)
{
	return fds_write(buffer, count, IS_VALUE);
}

static const struct proc_ops value_proc_ops = { .proc_write = fds_value_write };

static ssize_t fds_direction_write(struct file *file, const char __user *buffer,
				   size_t count, loff_t *pos)
{
	return fds_write(buffer, count, IS_DIRECTION);
}

static const struct proc_ops direction_proc_ops = {
	.proc_write = fds_direction_write,
};

static ssize_t reset_fds_write(struct file *file, const char __user *buffer,
			       size_t count, loff_t *pos)
{
	reset_fds();
	return count;
}

static const struct proc_ops reset_fds_proc_ops = { .proc_write =
							    reset_fds_write };

#define MAX_CONTENDING_LOCKS 10

enum fds_lock_type {
	FDS_SPINLOCK,
	FDS_MUTEX,
	FDS_READ_SEM,
	FDS_WRITE_SEM,
};

static inline const char *get_str_ltype(enum fds_lock_type ltype)
{
	switch (ltype) {
	case FDS_SPINLOCK:
		return "SPINLOCK";
	case FDS_MUTEX:
		return "MUTEX";
	case FDS_READ_SEM:
		return "SEMAPHORE-READ";
	case FDS_WRITE_SEM:
		return "SEMAPHORE-WRITE";
	default:
		return "UNDEFINED";
	}
}

static inline const char *get_str_lockm(enum fds_lock_mechanisms lockm)
{
	switch (lockm) {
	case FDS_QSPINLOCK:
		return "QSPINLOCK";
	case FDS_TAS:
		return "TAS";
	case FDS_TCLOCK:
		return "TCLOCK";
	case FDS_TDLOCK:
		return "TDLOCK";
	case FDS_PERCPU:
		return "PERCPU";
	default:
		return "UNDEFINED";
	}
}

struct contending_locks {
	enum fds_lock_type ltype;
	struct lock_stat *lock;
};

static long iterations = -1;
static long num_contending_locks = 0;

static struct contending_locks observed_locks[MAX_CONTENDING_LOCKS];

static enum fds_lock_mechanisms fds_spinlock_implementations[] = { FDS_TAS, FDS_QSPINLOCK, FDS_TCLOCK, FDS_TDLOCK};
static enum fds_lock_mechanisms fds_mutex_implementations[] = { FDS_QSPINLOCK, FDS_TCLOCK, FDS_TDLOCK };
static enum fds_lock_mechanisms fds_read_sem_implementations[] = { FDS_QSPINLOCK, FDS_PERCPU};
static enum fds_lock_mechanisms fds_write_sem_implementations[] = {FDS_QSPINLOCK, FDS_TCLOCK, FDS_TDLOCK};

#define NELEMS(x) (sizeof(x) / sizeof((x)[0]))

#define MAX_STATES 1024

static long oracle_states[1024];
static long num_states = 0;

static uint64_t oracle_start_time = 0;

static void seq_line(struct seq_file *m, char c, int offset, int length)
{
	int i;

	for (i = 0; i < offset; i++)
		seq_puts(m, " ");
	for (i = 0; i < length; i++)
		seq_printf(m, "%c", c);
	seq_puts(m, "\n");
}

static void seq_header(struct seq_file *m)
{
	seq_puts(m, "fds_stats\n");

	seq_line(m, '-', 0, 40 + 1 + 3 * (14 + 1));
	seq_printf(m, "%40s %14s %14s %14s\n", "lock-name", "slowpaths",
		   "lock-type", "implementation");
	seq_line(m, '-', 0, 40 + 1 + 3 * (14 + 1));
	seq_printf(m, "\n");
}

static void seq_stats(struct seq_file *m, long *v)
{
	int i = (*v - 1);
	BUG_ON(i < 0);
	if (i > MAX_CONTENDING_LOCKS || observed_locks[i].lock == NULL)
		return;
	seq_printf(m, "%40s: %14ld %14s %14s\n", observed_locks[i].lock->name,
		   observed_locks[i].lock->counter,
		   get_str_ltype(observed_locks[i].ltype),
		   get_str_lockm(observed_locks[i].lock->key->lockm));
}

static void *fds_stat_start(struct seq_file *m, loff_t *pos)
{
	if (*pos == 0)
		return SEQ_START_TOKEN;
	else if (*pos > num_contending_locks)
		return NULL;
	else
		return pos;
}

static void *fds_stat_next(struct seq_file *m, void *v, loff_t *pos)
{
	(*pos)++;
	return fds_stat_start(m, pos);
}

static void fds_stat_stop(struct seq_file *m, void *v)
{
}

static int fds_stat_show(struct seq_file *m, void *v)
{
	if (v == SEQ_START_TOKEN)
		seq_header(m);
	else
		seq_stats(m, v);

	return 0;
}

static const struct seq_operations fds_getstat_ops = {
	.start = fds_stat_start,
	.next = fds_stat_next,
	.stop = fds_stat_stop,
	.show = fds_stat_show,
};

static int get_fds_open(struct inode *inode, struct file *file)
{
	int res;

	res = seq_open(file, &fds_getstat_ops);

	return res;
}

static int get_fds_release(struct inode *inode, struct file *file)
{
	return seq_release(inode, file);
}

static const struct proc_ops get_fds_proc_ops = {
	.proc_open = get_fds_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = get_fds_release,
};

void recurse(long depth, long curr_state)
{
	if (depth == num_contending_locks) {
		oracle_states[num_states] = curr_state;
		printk(KERN_ALERT "index: %ld, state: %ld\n", num_states,
		       curr_state);
		num_states++;
		if (num_states > MAX_STATES)
			BUG_ON(true);

	} else {
		int num_implementations = 0;
		switch (observed_locks[depth].ltype) {
		case FDS_SPINLOCK:
			num_implementations =
				NELEMS(fds_spinlock_implementations);
			break;
		case FDS_MUTEX:
			num_implementations = NELEMS(fds_mutex_implementations);
			break;
		case FDS_READ_SEM:
			num_implementations =
				NELEMS(fds_read_sem_implementations);
			break;
		case FDS_WRITE_SEM:
			num_implementations =
				NELEMS(fds_write_sem_implementations);
			break;
		}
		for (int i = 0; i < num_implementations; i++)
			recurse(depth + 1, curr_state * 10 + i);
	}
}

void find_all_states(void)
{
	recurse(0, 0);
	for (int i = 0; i < num_states; i++)
		printk(KERN_ALERT "state i: %d => %ld\n", i, oracle_states[i]);
}

static void fds_oracle_restart(void)
{
	iterations = -1;
	num_contending_locks = 0;
	num_states = 0;
	fds_oracle_running = true;
	reset_fds();
	oracle_start_time = local_clock();
	printk(KERN_ALERT "Oracle starting at: %ld\n", oracle_start_time);
}

inline void __find_contending_locks(struct lock_stat *tmp, const char *type,
				    enum fds_lock_type ltype, uint64_t elapsed_time)
{
        bool is_lock_contending = false;

        switch(ltype) {
		case FDS_WRITE_SEM:
			is_lock_contending = ((tmp->counter / elapsed_time) > QSPINLOCK_PER_SECOND || (tmp->read_counter / elapsed_time) > QSPINLOCK_PER_SECOND);
                        break;
                case FDS_MUTEX:
                        is_lock_contending = (tmp->counter / elapsed_time) > MUTEX_PER_SECOND;
                        break;
                default:
                        is_lock_contending = (tmp->counter / elapsed_time) > QSPINLOCK_PER_SECOND;
        }

	if (is_lock_contending) {
		printk(KERN_ALERT
		       "Contending lock type: %s Name: %s, Counter: %ld lock_type: %s\n",
		       type, tmp->name, tmp->counter,
		       get_str_lockm(tmp->key->lockm));
		observed_locks[num_contending_locks].ltype = ltype;
		observed_locks[num_contending_locks].lock = tmp;
		num_contending_locks++;
		if (num_contending_locks > MAX_CONTENDING_LOCKS)
			BUG_ON(true);
	}
}

void find_contending_locks(void)
{
	int bkt;
	struct lock_stat *tmp;

	uint64_t time_now = local_clock();
	uint64_t elapsed_time = (time_now - oracle_start_time) / 1000000000;

	printk(KERN_ALERT "Oracle elapsed time: %ld\n", elapsed_time);

	spin_lock(&stat_ht_lock);

	/*hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		__find_contending_locks(tmp, "READ SEM", FDS_READ_SEM,
					elapsed_time);
	}*/

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		__find_contending_locks(tmp, "WRITE SEM", FDS_WRITE_SEM,
					elapsed_time);
	}

	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		__find_contending_locks(tmp, "SPINLOCK", FDS_SPINLOCK,
					elapsed_time);
	}

	hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		__find_contending_locks(tmp, "MUTEX", FDS_MUTEX, elapsed_time);
	}

	spin_unlock(&stat_ht_lock);
}

static ssize_t fds_oracle_set_next_state(size_t count)
{
	if (iterations == -1) {
		collect_fds_stats();
		printk(KERN_ALERT "ORACLE set next state\n");
		print_fds_stats();
		find_contending_locks();
		find_all_states();
		iterations = 0;
	} else if (iterations == num_states) {
		printk(KERN_ALERT "iterations: %ld, num_states: %ld\n",
		       iterations, num_states);
		return -ENODATA;
	}

	for (int i = 0; i < num_contending_locks; i++) {
		enum fds_lock_mechanisms new_lockm;
		int index = (oracle_states[iterations] /
			     int_pow(10, (num_contending_locks - 1 - i))) %
			    10;
		switch (observed_locks[i].ltype) {
		case FDS_SPINLOCK:
			new_lockm = fds_spinlock_implementations[index];
			break;
		case FDS_MUTEX:
			new_lockm = fds_mutex_implementations[index];
			break;
		case FDS_READ_SEM:
			new_lockm = fds_read_sem_implementations[index];
			break;
		case FDS_WRITE_SEM:
			new_lockm = fds_write_sem_implementations[index];
			break;
		}
		observed_locks[i].lock->key->lockm = new_lockm;
		printk(KERN_ALERT
		       "Switching lock type: %s Name: %s, Counter: %ld Implementation: %s\n",
		       get_str_ltype(observed_locks[i].ltype),
		       observed_locks[i].lock->name,
		       observed_locks[i].lock->counter,
		       get_str_lockm(new_lockm));
	}

	iterations++;

	return count;
}

static ssize_t fds_oracle_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *pos)
{
	char buf[64];

	if (count > 64)
		return -EINVAL;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	uint64_t val = 0xdeadbeef;
	kstrtoll(buf, 0, &val);

	switch (val) {
	case 0:
		fds_oracle_running = false;
		break;
	case 1:
		fds_oracle_restart();
		break;
	case 2:
		reset_fds_stats();
		oracle_start_time = local_clock();
		printk(KERN_ALERT "Find contending locks starting at: %ld\n",
		       oracle_start_time);
		break;
	case 3:
		return fds_oracle_set_next_state(count);
	default:
		return -EOPNOTSUPP;
	}

	return count;
}

static const struct proc_ops oracle_proc_ops = {
	.proc_write = fds_oracle_write,
};

//inline enum fds_lock_mechanisms get_optimal_rwsem_decision_tree_regression(struct lock_stat *tmp) 
//{
//	int max_value, max_index, j;
//	long feature_vector[8];
//	feature_vector[0] = 2;
//	feature_vector[1] = (tmp->counter * 100) / (tmp->read_counter + tmp->counter);
//	feature_vector[2] = cpumask_weight(&tmp->contending_cpus);
//	feature_vector[3] = (tmp->counter + tmp->read_counter) / (MONITOR_TIME / 1000);
//	max_value = 0;
//	max_index = 0;
//	j = 0;
//	for(j = 0; j < 4; j++) {
//		if(j > 0)
//			feature_vector[4 + (j - 1)] = 0;
//		feature_vector[4 + j] = 1;
//		int value = rwsem_decision_tree(feature_vector);
//		if(value > max_value) {
//			max_value = value;
//			max_index = 4 + j;
//		}
//	}
//	printk(KERN_ALERT "RWRATIO: %ld CPUCNT:%ld RPS:%ld max_value: %d max_index: %d\n",
//			feature_vector[1], feature_vector[2], feature_vector[3],
//			max_value, max_index);
//	enum fds_lock_mechanisms next_lock_type = FDS_QSPINLOCK;
//	switch(max_index) {
//		case 4:
//		case 5:
//			next_lock_type = FDS_TCLOCK;
//			break;
//		case 6:
//			next_lock_type = FDS_PERCPU;
//			break;
//		case 7:
//			next_lock_type = FDS_QSPINLOCK;
//			break;
//	}
//	return next_lock_type;
//}

inline enum fds_lock_mechanisms get_optimal_spinlock_random_forest_classifier(struct lock_stat *tmp) 
{
	int feature_vector[2];
	feature_vector[0] = cpumask_weight(&tmp->contending_cpus);
	feature_vector[1] = (tmp->counter) / (MONITOR_TIME / 1000);
	int optimal_index = predict_srv1_spinlock_random_forest(feature_vector) ;
	enum fds_lock_mechanisms next_lock_type = FDS_QSPINLOCK;
	switch(optimal_index) {
		case 0: next_lock_type = FDS_QSPINLOCK; break; //AQS
		case 1: next_lock_type = FDS_QSPINLOCK; break;
		case 2: next_lock_type = FDS_TDLOCK; break; //TCLOCK
		case 3: next_lock_type = FDS_TDLOCK; break; //TDLOCK
	}

	printk(KERN_ALERT "SPINLOCK CPUCNT:%ld RPS:%ld next_lock: %s\n",
			feature_vector[0], feature_vector[1],
			get_str_lockm(next_lock_type));
	return next_lock_type;
}


inline enum fds_lock_mechanisms get_optimal_rwsem_random_forest_classifier(struct lock_stat *tmp) 
{
	int feature_vector[3];
	feature_vector[0] = (tmp->counter * 100) / (tmp->read_counter + tmp->counter);
	feature_vector[1] = cpumask_weight(&tmp->contending_cpus);
	feature_vector[2] = (tmp->counter + tmp->read_counter) / (MONITOR_TIME / 1000);
	int optimal_index = predict_srv1_rwsem_random_forest(feature_vector) ;
	enum fds_lock_mechanisms next_lock_type = FDS_QSPINLOCK;
	switch(optimal_index) {
		case 0: next_lock_type = FDS_PERCPU; break;
		case 1: next_lock_type = FDS_QSPINLOCK; break;
		case 2: next_lock_type = FDS_TCLOCK; break;
		case 3: next_lock_type = FDS_TDLOCK; break; //TDLOCK
	}

	printk(KERN_ALERT "RWSEM RWRATIO: %ld CPUCNT:%ld RPS:%ld next_lock: %s\n",
			feature_vector[0], feature_vector[1], feature_vector[2],
			get_str_lockm(next_lock_type));
	return next_lock_type;
}


inline void __monitor_fds_stats(struct lock_stat *tmp, const char *type,
				enum fds_lock_type ltype)
{
	int i, j;
	long feature_vector[8];
	int max_value, max_index;
	if (tmp->counter > QSPINLOCK_LIMIT || tmp->read_counter > QSPINLOCK_LIMIT) {
		enum fds_lock_mechanisms before = tmp->key->lockm;
		if(before == FDS_DISABLE)
			return;

		switch (ltype) {
		case FDS_SPINLOCK:
//			for(i = 0; i < NELEMS(fds_spinlock_implementations); i++)
//				if(tmp->key->lockm == fds_spinlock_implementations[i])
//					break;
//			tmp->key->lockm = fds_spinlock_implementations[(i+1) % NELEMS(fds_spinlock_implementations)];
			
//			feature_vector[0] = 4;
//			feature_vector[1] = cpumask_weight(&tmp->contending_cpus);
//			feature_vector[2] = tmp->counter / (MONITOR_TIME / 1000);
//			max_value = 0;
//			max_index = 0;
//			j = 0;
//			for(j = 0; j < 5; j++) {
//				if(j > 0)
//					feature_vector[3 + (j - 1)] = 0;
//				feature_vector[3 + j] = 1;
//				int value = decision_tree(feature_vector);
//				if(value > max_value) {
//					max_value = value;
//					max_index = 3 + j;
//				}
//			}
//			printk(KERN_ALERT "feature_vector: %ld %ld %ld max_value: %d max_index: %d\n",
//					feature_vector[0], feature_vector[1], feature_vector[2],
//					max_value, max_index);
//			enum fds_lock_mechanisms next_lock_type = FDS_QSPINLOCK;
//			switch(max_index) {
//				case 3:
//				case 4:
//				case 7:
//					next_lock_type = FDS_QSPINLOCK;
//					break;
//				case 5:
//					next_lock_type = FDS_TCLOCK;
//					break;
//				case 6:
//					next_lock_type = FDS_TCLOCK; //Change to TDLOCK
//					break;
//			}
//			tmp->key->lockm = next_lock_type;
//			break;
			tmp->key->lockm = get_optimal_spinlock_random_forest_classifier(tmp);
			break;
		case FDS_WRITE_SEM:
			tmp->key->lockm = get_optimal_rwsem_random_forest_classifier(tmp);
			break;

		/*case FDS_READ_SEM:
			for(i = 0; i < NELEMS(fds_read_sem_implementations); i++)
				if(tmp->key->lockm == fds_read_sem_implementations[i])
					break;
			tmp->key->lockm = fds_read_sem_implementations[(i+1) % NELEMS(fds_read_sem_implementations)];
			break;
		case FDS_WRITE_SEM:
			for(i = 0; i < NELEMS(fds_write_sem_implementations); i++)
				if(tmp->key->lockm == fds_write_sem_implementations[i])
					break;
			tmp->key->lockm = fds_write_sem_implementations[(i+1) % NELEMS(fds_write_sem_implementations)];
			break;
		case FDS_MUTEX:
			for(i = 0; i < NELEMS(fds_mutex_implementations); i++)
				if(tmp->key->lockm == fds_mutex_implementations[i])
					break;
			tmp->key->lockm = fds_mutex_implementations[(i+1) % NELEMS(fds_mutex_implementations)];
			break;*/
		}
		printk(KERN_ALERT
		       "Flipping %s write Name: %s, Counter: %ld before: %s new: %s\n",
		       type, tmp->name, tmp->counter,
		       get_str_lockm(before),
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

	/*hash_for_each(read_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "READ SEM", FDS_READ_SEM);
	}*/

	hash_for_each(write_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "WRITE SEM", FDS_WRITE_SEM);
	}

	hash_for_each(spin_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "SPINLOCK", FDS_SPINLOCK);
	}

	/*hash_for_each(mutex_stats_ht, bkt, tmp, hnode) {
		__monitor_fds_stats(tmp, "MUTEX", FDS_MUTEX);
	}*/

	spin_unlock(&stat_ht_lock);
}

int fds_monitor(void *args)
{
	printk(KERN_ALERT "Starting fds monitor\n");
	while (!kthread_should_stop()) {
		msleep(MONITOR_TIME);
		
		//ssleep(30);

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
