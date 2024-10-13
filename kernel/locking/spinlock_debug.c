/*
 * Copyright 2005, Red Hat, Inc., Ingo Molnar
 * Released under the General Public License (GPL).
 *
 * This file contains the spinlock/rwlock implementations for
 * DEBUG_SPINLOCK.
 */

#include <linux/spinlock.h>
#include <linux/nmi.h>
#include <linux/interrupt.h>
#include <linux/debug_locks.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/pid.h>

void __raw_spin_lock_init(raw_spinlock_t *lock, const char *name,
			  struct fds_lock_key *key)
{
	lock->raw_lock = (arch_spinlock_t)__ARCH_SPIN_LOCK_UNLOCKED;
	lock->key.name = name;
	lock->key.ptr = key;
	init_fds_lock_key(key, name);
}

EXPORT_SYMBOL(__raw_spin_lock_init);
