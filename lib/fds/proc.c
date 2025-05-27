// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

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

static ssize_t reset_fds_write(struct file *file, const char __user *buffer,
			       size_t count, loff_t *pos)
{
	reset_fds();
	return count;
}

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

static ssize_t fds_monitor_time_write(struct file *file,
				      const char __user *buffer, size_t count,
				      loff_t *pos)
{
	char buf[64];

	if (count > 64)
		return -EINVAL;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	uint64_t val = 0;
	kstrtoll(buf, 0, &val);

	if (val % 1000 == 0)
		fds_monitor_time = val;
	else
		return -EINVAL;

	return count;
}

static const struct seq_operations fds_getstat_ops = {
	.start = fds_stat_start,
	.next = fds_stat_next,
	.stop = fds_stat_stop,
	.show = fds_stat_show,
};

static const struct proc_ops get_fds_proc_ops = {
	.proc_open = get_fds_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = get_fds_release,
};

static const struct proc_ops reset_fds_proc_ops = { .proc_write =
							    reset_fds_write };

static const struct proc_ops value_proc_ops = { .proc_write = fds_value_write };

static const struct proc_ops direction_proc_ops = {
	.proc_write = fds_direction_write,
};

static const struct proc_ops fds_monitor_time_proc_ops = {
	.proc_write = fds_monitor_time_write,
};
