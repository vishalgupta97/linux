/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#ifndef _LINUX_KOMB_RWSEM_H
#define _LINUX_KOMB_RWSEM_H

#include <linux/rwsem.h>
#include <linux/linkage.h>
#include <linux/aqm.h>

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/komb_mutex.h>

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

struct komb_rwsem {
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
		struct aqm_mutex reader_wait_lock;
		char __padding3[128];
	};
	union {
		struct komb_mutex_node *writer_tail;
		char __padding4[128];
	};
};

static inline int komb_rwsem_is_locked(struct komb_rwsem *sem)
{
	return atomic_long_read(&sem->cnts) != 0;
}

#define __KOMB_RWSEM_INITIALIZER(lockname)                          \
	{                                                           \
		.cnts = ATOMIC_LONG_INIT(0),                        \
		.reader_wait_lock.val = ATOMIC_INIT(0),             \
		.reader_wait_lock.tail = NULL, .writer_tail = NULL, \
	}

extern void komb_init_rwsem(struct komb_rwsem *sem);

static inline int komb_rwsem_is_contended(struct komb_rwsem *sem)
{
	return (sem->writer_tail != NULL);
}

extern void komb_down_read(struct komb_rwsem *sem);
extern int __must_check komb_down_read_interruptible(struct komb_rwsem *sem);
extern int __must_check komb_down_read_killable(struct komb_rwsem *sem);
extern int komb_down_read_trylock(struct komb_rwsem *sem);
extern void komb_down_write(struct komb_rwsem *sem);
extern int __must_check komb_down_write_killable(struct komb_rwsem *sem);
extern int komb_down_write_trylock(struct komb_rwsem *sem);

extern void komb_up_read(struct komb_rwsem *sem);
extern void komb_up_write(struct komb_rwsem *sem);
extern void komb_downgrade_write(struct komb_rwsem *sem);

#define komb_down_read_nested(sem, subclass) komb_down_read(sem)
#define komb_down_read_killable_nested(sem, subclass) \
	komb_down_read_killable(sem)
#define komb_down_write_nest_lock(sem, nest_lock) komb_down_write(sem)
#define komb_down_write_nested(sem, subclass) komb_down_write(sem)
#define komb_down_write_killable_nested(sem, subclass) \
	komb_down_write_killable(sem)
#define komb_down_read_non_owner(sem) komb_down_read(sem)
#define komb_up_read_non_owner(sem) komb_up_read(sem)

//TODO: Implement these
#define komb_rwsem_assert_held(sem)
#define komb_rwsem_assert_held_write(sem)
#define komb_rwsem_assert_held_write_nolockdep(sem)

#endif /* _LINUX_KOMB_RWSEM_H */
