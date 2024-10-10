/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MUTEX_TYPES_H
#define __LINUX_MUTEX_TYPES_H

#include <linux/atomic.h>
#include <linux/lockdep_types.h>
#include <linux/osq_lock.h>
#include <linux/spinlock_types.h>
#include <linux/types.h>
#include <linux/feedbacksync.h>

#ifndef CONFIG_PREEMPT_RT

/*
 * Simple, straightforward mutexes with strict semantics:
 *
 * - only one task can hold the mutex at a time
 * - only the owner can unlock the mutex
 * - multiple unlocks are not permitted
 * - recursive locking is not permitted
 * - a mutex object must be initialized via the API
 * - a mutex object must not be initialized via memset or copying
 * - task may not exit with mutex held
 * - memory areas where held locks reside must not be freed
 * - held mutexes must not be reinitialized
 * - mutexes may not be used in hardware or software interrupt
 *   contexts such as tasklets and timers
 *
 * These semantics are fully enforced when DEBUG_MUTEXES is
 * enabled. Furthermore, besides enforcing the above rules, the mutex
 * debugging code also implements a number of additional features
 * that make lock debugging easier and faster:
 *
 * - uses symbolic names of mutexes, whenever they are printed in debug output
 * - point-of-acquire tracking, symbolic lookup of function names
 * - list of all locks held in the system, printout of them
 * - owner tracking
 * - detects self-recursing locks and prints out all relevant info
 * - detects multi-task circular deadlocks and prints out all affected
 *   locks and tasks (and only those tasks)
 */
struct mutex_node {
	struct mutex_node *next;
	struct mutex_node *tail;

	int socket_id; //Socket ID
	int cpuid;

	uint64_t rsp;
	struct task_struct *task_struct_ptr;
	void *lock;
	struct ww_acquire_ctx *ww_ctx;
	char dummy1[80];

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

	char dummy[16];
};

struct mutex {
	struct mutex_node *tail;
	union {
		atomic_t state;
		u8 locked;
	};
	struct task_struct *combiner_task;
	struct fds_lock_key key;
};

#else /* !CONFIG_PREEMPT_RT */
/*
 * Preempt-RT variant based on rtmutexes.
 */
#include <linux/rtmutex.h>

struct mutex {
	struct rt_mutex_base rtmutex;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif
};

#endif /* CONFIG_PREEMPT_RT */

#endif /* __LINUX_MUTEX_TYPES_H */
