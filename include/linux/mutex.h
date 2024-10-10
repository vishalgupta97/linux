/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#ifndef __LINUX_MUTEX_H
#define __LINUX_MUTEX_H

#include <asm/current.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>
#include <linux/lockdep.h>
#include <linux/atomic.h>
#include <asm/processor.h>
#include <linux/osq_lock.h>
#include <linux/debug_locks.h>
#include <linux/cleanup.h>
#include <linux/mutex_types.h>

struct device;
static inline void mutex_destroy(struct mutex *lock)
{
}

#define mutex_init(mutex)                               \
	do {                                            \
		static struct fds_lock_key __key;       \
                                                        \
		___mutex_init((mutex), #mutex, &__key); \
	} while (0)

#define __MUTEX_INITIALIZER(lockname)                                         \
	{                                                                     \
		.tail = NULL, .state = ATOMIC_INIT(0), .combiner_task = NULL, \
		.key = {                                                      \
			.name = #lockname,                                    \
			.ptr = NULL,                                          \
		},                                                            \
	}

#define DEFINE_MUTEX(mutexname) \
	struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

extern void ___mutex_init(struct mutex *lock, const char *name,
			  struct fds_lock_key *key);
extern void __mutex_init(struct mutex *lock, const char *name,
			 struct lock_class_key *key);

extern bool mutex_is_locked(struct mutex *lock);

#define devm_mutex_init(dev, mutex)             \
	({                                      \
		typeof(mutex) mutex_ = (mutex); \
                                                \
		mutex_init(mutex_);             \
		__devm_mutex_init(dev, mutex_); \
	})

extern void mutex_lock(struct mutex *lock);
extern int __must_check mutex_lock_interruptible(struct mutex *lock);
extern int __must_check mutex_lock_killable(struct mutex *lock);
extern void mutex_lock_io(struct mutex *lock);

#define mutex_lock_nested(lock, subclass) mutex_lock(lock)
#define mutex_lock_interruptible_nested(lock, subclass) \
	mutex_lock_interruptible(lock)
#define mutex_lock_killable_nested(lock, subclass) mutex_lock_killable(lock)
#define mutex_lock_nest_lock(lock, nest_lock) mutex_lock(lock)
#define mutex_lock_io_nested(lock, subclass) mutex_lock_io(lock)

extern int mutex_trylock(struct mutex *lock);
extern void mutex_unlock(struct mutex *lock);

extern int atomic_dec_and_mutex_lock(atomic_t *cnt, struct mutex *lock);

DEFINE_GUARD(mutex, struct mutex *, mutex_lock(_T), mutex_unlock(_T))
DEFINE_GUARD_COND(mutex, _try, mutex_trylock(_T))
DEFINE_GUARD_COND(mutex, _intr, mutex_lock_interruptible(_T) == 0)

#endif /* __LINUX_MUTEX_H */
