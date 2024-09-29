// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/feedbacksync.h>

#define IS_VALUE 1
#define IS_DIRECTION 2

uint64_t value = 0;
uint64_t direction = 0;

bool do_fds_fallback = false;
bool do_tas_fallback = false;

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
		if (value == 0) {
			do_fds_fallback = false;
			do_tas_fallback = false;
		} else if (value == 1) {
			do_fds_fallback = true;
			do_tas_fallback = false;
		} else if (value == 2) {
			do_tas_fallback = true;
			do_fds_fallback = true;
		}
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

void __init feedback_sync_init(void)
{
	proc_mkdir("fds", NULL);
	proc_create("fds/value", 0222, NULL, &value_proc_ops);
	proc_create("fds/direction", 0222, NULL, &direction_proc_ops);
}