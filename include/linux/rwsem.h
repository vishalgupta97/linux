/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#ifndef _LINUX_RWSEM_H
#define _LINUX_RWSEM_H

#include <linux/linkage.h>
#include <linux/aqm.h>

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/feedbacksync.h>

#define BRAVO 1

/*
 * Writer states & reader shift and bias.
 */
#define _KOMB_RWSEM_W_LOCKED 0xff /* A writer holds the lock */
#define _KOMB_RWSEM_W_COMBINER 0x70 /* A combiner holds the lock */
#define _KOMB_RWSEM_W_OOO 0x7f /* A combiner holds the lock */
#define _KOMB_RWSEM_W_DOWNGRADE 0x77 /* A writer combiner requested downgrade */
#define _KOMB_RWSEM_W_WMASK 0x1ff /* Writer mask		   */
#define _KOMB_RWSEM_W_WAITING 0x100 /* Writer waiting */
#define _KOMB_RWSEM_R_SHIFT 9 /* Reader count shift	   */
#define _KOMB_RWSEM_R_BIAS (1U << _KOMB_RWSEM_R_SHIFT)

#define _Q_COMPLETED_OFFSET (_Q_LOCKED_OFFSET + _Q_LOCKED_BITS)
#define _Q_COMPLETED_BITS 8
#define _Q_COMPLETED_MASK _Q_SET_MASK(COMPLETED)

#define NUM_SLOT (1024)
#define TABLE_SIZE ((NUM_SLOT * 8))
#define V(i) ((i * 8))

#define CHECK_FOR_BIAS 16
#define MULTIPLIER 9

struct rw_semaphore {
	union {
		union {
			atomic_long_t cnts;
			struct {
				u8 wlocked;
				u8 rcount[7];
			};
		};
		char __padding[128];
	};
	union {
		struct {
			struct aqm_mutex reader_wait_lock;
#ifdef BRAVO
			int rbias;
			u64 inhibit_until;
#endif
		};
		char __padding3[128];
	};
	union {
		struct {
			struct mutex_node *writer_tail;
			struct fds_lock_key key;
		};
		char __padding4[128];
	};
};

static inline int rwsem_is_locked(struct rw_semaphore *sem)
{
	return atomic_long_read(&sem->cnts) != 0;
}

#define RWSEM_UNLOCKED_VALUE 0L

#define __RWSEM_OPT_INIT(lockname)

#define __RWSEM_INITIALIZER(lockname)                               \
	{                                                           \
		.cnts = ATOMIC_LONG_INIT(0),                        \
		.reader_wait_lock.val = ATOMIC_INIT(0),             \
		.reader_wait_lock.tail = NULL, .writer_tail = NULL, \
		.key = {                                            \
			.name = #lockname,                          \
			.ptr = NULL,                                \
		},                                                  \
	}

#define DECLARE_RWSEM(name) struct rw_semaphore name = __RWSEM_INITIALIZER(name)

extern void __init_rwsem(struct rw_semaphore *sem, const char *name,
			 struct fds_lock_key *key);

#define init_rwsem(sem)                            \
	do {                                       \
		static struct fds_lock_key __key;  \
                                                   \
		__init_rwsem((sem), #sem, &__key); \
	} while (0)

extern void komb_rwsem_init(void);

static inline int rwsem_is_contended(struct rw_semaphore *sem)
{
	return (sem->writer_tail != NULL);
}

extern void down_read(struct rw_semaphore *sem);
extern int __must_check down_read_interruptible(struct rw_semaphore *sem);
extern int __must_check down_read_killable(struct rw_semaphore *sem);
extern int down_read_trylock(struct rw_semaphore *sem);
extern void down_write(struct rw_semaphore *sem);
extern int __must_check down_write_killable(struct rw_semaphore *sem);
extern int down_write_trylock(struct rw_semaphore *sem);

extern void up_read(struct rw_semaphore *sem);
extern void up_write(struct rw_semaphore *sem);
extern void downgrade_write(struct rw_semaphore *sem);

#define down_read_nested(sem, subclass) down_read(sem)
#define down_read_killable_nested(sem, subclass) down_read_killable(sem)
#define down_write_nest_lock(sem, nest_lock) down_write(sem)
#define down_write_nested(sem, subclass) down_write(sem)
#define down_write_killable_nested(sem, subclass) down_write_killable(sem)
#define down_read_non_owner(sem) down_read(sem)
#define up_read_non_owner(sem) up_read(sem)

//TODO: Implement these
#define rwsem_assert_held(sem)
#define rwsem_assert_held_write(sem)
#define rwsem_assert_held_write_nolockdep(sem)

DEFINE_GUARD(rwsem_read, struct rw_semaphore *, down_read(_T), up_read(_T))
DEFINE_GUARD_COND(rwsem_read, _try, down_read_trylock(_T))
DEFINE_GUARD_COND(rwsem_read, _intr, down_read_interruptible(_T) == 0)

DEFINE_GUARD(rwsem_write, struct rw_semaphore *, down_write(_T), up_write(_T))
DEFINE_GUARD_COND(rwsem_write, _try, down_write_trylock(_T))

#endif /* _LINUX_RWSEM_H */
