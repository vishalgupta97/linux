// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

static bool fds_oracle_running = false;

static enum fds_lock_mechanisms fds_spinlock_implementations[] = {
	FDS_QSPINLOCK, FDS_CNA, FDS_TCLOCK
}; //, FDS_TDLOCK};
static enum fds_lock_mechanisms fds_mutex_implementations[] = {
	FDS_QSPINLOCK, FDS_TCLOCK
}; //, FDS_TDLOCK };
static enum fds_lock_mechanisms fds_read_sem_implementations[] = {
	FDS_QSPINLOCK, FDS_PERCPU
};
static enum fds_lock_mechanisms fds_write_sem_implementations[] = {
	FDS_QSPINLOCK, FDS_TCLOCK
}; //, FDS_TDLOCK};

static long oracle_states[MAX_STATES];
static long num_states = 0;

static uint64_t oracle_start_time = 0;

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
				    enum fds_lock_type ltype,
				    uint64_t elapsed_time)
{
	bool is_lock_contending = false;

	switch (ltype) {
	case FDS_WRITE_SEM:
		is_lock_contending =
			((tmp->counter / elapsed_time) > QSPINLOCK_PER_SECOND ||
			 (tmp->read_counter / elapsed_time) >
				 QSPINLOCK_PER_SECOND);
		break;
	case FDS_MUTEX:
		is_lock_contending = (tmp->counter / elapsed_time) >
				     MUTEX_PER_SECOND;
		break;
	default:
		is_lock_contending = (tmp->counter / elapsed_time) >
				     QSPINLOCK_PER_SECOND;
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
