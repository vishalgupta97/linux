/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mutexes: blocking mutual exclusion locks
 *
 * started by Ingo Molnar:
 *
 *  Copyright (C) 2004, 2005, 2006 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 */

#include <linux/komb_mutex.h>
#include <linux/sched/signal.h>
#include <linux/sched/rt.h>
#include <linux/sched/wake_q.h>
#include <linux/sched/debug.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/debug_locks.h>
#include <linux/osq_lock.h>

#include <linux/smp.h>
#include <linux/bug.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/prefetch.h>
#include <linux/atomic.h>
#include <asm/byteorder.h>
#include <linux/vmalloc.h>
#include <linux/sched/stat.h>
#include <linux/sched/task.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/combiner.h>

#if DSM_DEBUG
#define print_debug(fmt, ...)                                            \
	({                                                               \
		printk(KERN_ALERT "[%d] [%d] komb (%s) lock(%px): " fmt, \
		       smp_processor_id(), current->pid, __func__, lock, \
		       ##__VA_ARGS__);                                   \
	})
#else
#define print_debug(fmt, ...)
#endif

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#define smp_cond_load_relaxed_sleep(curr_node, ptr, cond_expr)                 \
	({                                                                     \
		typeof(ptr) __PTR = (ptr);                                     \
		__unqual_scalar_typeof(*ptr) VAL;                              \
		for (;;) {                                                     \
			VAL = READ_ONCE(*__PTR);                               \
			if (cond_expr)                                         \
				break;                                         \
			cpu_relax();                                           \
			if (need_resched()) {                                  \
				if (single_task_running())                     \
					schedule_out_curr_task();              \
				else {                                         \
					if (READ_ONCE(curr_node->completed) == \
					    KOMB_WAITER_UNPROCESSED)           \
						park_waiter(curr_node);        \
					else                                   \
						schedule_out_curr_task();      \
				}                                              \
			}                                                      \
		}                                                              \
		(typeof(*ptr))VAL;                                             \
	})

#ifndef smp_cond_load_relaxed_sched
#define smp_cond_load_relaxed_sched(ptr, cond_expr)       \
	({                                                \
		typeof(ptr) __PTR = (ptr);                \
		__unqual_scalar_typeof(*ptr) VAL;         \
		for (;;) {                                \
			VAL = READ_ONCE(*__PTR);          \
			if (cond_expr)                    \
				break;                    \
			cpu_relax();                      \
			if (need_resched()) {             \
				schedule_out_curr_task(); \
			}                                 \
		}                                         \
		(typeof(*ptr))VAL;                        \
	})
#endif

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr) \
	({                                                     \
		typeof(ptr) __PTR = (ptr);                     \
		__unqual_scalar_typeof(*ptr) VAL;              \
		for (;;) {                                     \
			VAL = READ_ONCE(*__PTR);               \
			if (cond_expr)                         \
				break;                         \
			cpu_relax();                           \
			park_komb_mutex_thread();              \
		}                                              \
		(typeof(*ptr))VAL;                             \
	})

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                 \
	({                                                          \
		__unqual_scalar_typeof(*ptr) _val;                  \
		_val = smp_cond_load_relaxed_sched(ptr, cond_expr); \
		smp_acquire__after_ctrl_dep();                      \
		(typeof(*ptr))_val;                                 \
	})
#endif

#define UINT64_MAX 0xffffffffffffffffL

#define _Q_LOCKED_COMBINER_VAL 3
#define _Q_UNLOCKED_OOO_VAL 7 //Unlocked a lock out-of-order

#define KOMB_WAITER_UNPROCESSED 0
#define KOMB_WAITER_PARKED 1
#define KOMB_WAITER_PROCESSING 2
#define KOMB_WAITER_PROCESSED 4
