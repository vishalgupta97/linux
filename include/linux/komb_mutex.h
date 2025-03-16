/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#ifndef __LINUX_KOMB_MUTEX_H
#define __LINUX_KOMB_MUTEX_H

#include <asm/current.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>
#include <linux/lockdep.h>
#include <linux/atomic.h>
#include <asm/processor.h>
#include <linux/osq_lock.h>
#include <linux/debug_locks.h>
#include <linux/cleanup.h>
#include <linux/topology.h>

struct komb_mutex_node {
	union {
		struct {
			struct komb_mutex_node *next;

			int socket_id; //Socket ID
			int cpuid;

			uint64_t rsp;
			struct task_struct *task_struct_ptr;
			void *lock;
		};
		char alignment1[128];
	};

	union {
		union {
			atomic_long_t val;
			atomic_long_t cnts;
			struct {
				u8 completed;
				u8 locked;
				u8 __unused[6];
			};
			struct {
				u16 locked_completed;
				u8 __unused1[6];
			};
			struct {
				u16 wlocked;
				u8 rcount[6];
			};
		};
		char alignment2[128];
	};
};

struct komb_mutex {
	union {
		struct {
			struct komb_mutex_node *tail;
			union {
				atomic_t state;
				u8 locked;
			};
		};
		char alignment[64];
	};
};

extern void komb_mutex_init(struct komb_mutex *lock);
extern void komb_mutex_lock(struct komb_mutex *lock);
extern int komb_mutex_trylock(struct komb_mutex *lock);
extern void komb_mutex_unlock(struct komb_mutex *lock);

#endif /* __LINUX_KOMB_MUTEX_H */
