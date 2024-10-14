// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/feedbacksync.h>
#include <asm-generic/qspinlock.h>

#define DEBUG_KOMB 0
#define DSM_DEBUG 0

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#if DSM_DEBUG
#define print_debug(fmt, ...)                                                \
	({                                                                   \
		printk(KERN_EMERG "[%d] {%p} (%s) " fmt, smp_processor_id(), \
		       lock, __func__, ##__VA_ARGS__);                       \
	})
#else
#define print_debug(fmt, ...)
#endif

#define smp_cond_load_relaxed_sched(ptr, cond_expr) \
	({                                          \
		typeof(ptr) __PTR = (ptr);          \
		__unqual_scalar_typeof(*ptr) VAL;   \
		for (;;) {                          \
			VAL = READ_ONCE(*__PTR);    \
			if (cond_expr)              \
				break;              \
			cpu_relax();                \
		}                                   \
		(typeof(*ptr))VAL;                  \
	})

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr) \
	({                                                     \
		typeof(ptr) __PTR = (ptr);                     \
		__unqual_scalar_typeof(*ptr) VAL;              \
		for (;;) {                                     \
			VAL = READ_ONCE(*__PTR);               \
			if (cond_expr)                         \
				break;                         \
			cpu_relax();                           \
			if (need_resched()) {                  \
				preempt_enable();              \
				cond_resched();                \
				preempt_disable();             \
			}                                      \
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

#define atomic_cond_read_acquire_sched(v, c) \
	smp_cond_load_acquire_sched(&(v)->counter, (c))

#define SIZE_OF_SHADOW_STACK 8192L
#define IRQ_NUMA_NODE 255
#define UINT64_MAX 0xffffffffffffffffL

#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)
#define _Q_LOCKED_COMBINER_VAL 7
#define _Q_UNLOCKED_OOO_VAL 15 //Unlocked a lock out-of-order
#define _Q_LOCKED_IRQ_VAL 31 // Lock Stealing by IRQ

DECLARE_PER_CPU_SHARED_ALIGNED(struct komb_node, komb_nodes[MAX_NODES]);
DECLARE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

u32 encode_tail(int cpu, int idx);
struct komb_node *decode_tail(u32 tail);
u32 get_cpu_from_tail(u32 tail);
u32 get_cpu_from_tail(u32 tail);
void clear_locked_set_completed(struct komb_node *lock);
u32 xchg_tail(struct qspinlock *lock, u32 tail);
u32 cmpxchg_tail(struct qspinlock *lock, u32 tail, u32 new_tail);
void add_to_local_queue(struct komb_node *node);
struct komb_node *get_next_node(struct komb_node *my_node);
void execute_cs(struct qspinlock *lock, struct komb_node *curr_node);
void *get_shadow_stack_ptr(void);
struct komb_node *get_komb_node(void);
u32 komb_fetch_set_pending_acquire(struct qspinlock *lock);
void clear_pending(struct qspinlock *lock);
void set_locked(struct qspinlock *lock);
void clear_pending_set_locked(struct qspinlock *lock);
