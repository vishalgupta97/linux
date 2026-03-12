#ifndef __KOMB_DELEGATION_H__
#define __KOMB_DELEGATION_H__

#include "komb.h"
#include "topology.h"

#define DEFINE_KOMB_DELEGATION_SPINLOCK(x)                                                         \
	struct qspinlock(x) = (struct qspinlock)__ORIG_QSPIN_LOCK_UNLOCKED

/*
 * TODO (Correctness optimization): 
 * Add for BIG ENDIAN
 */
struct komb_delegation_node {
	union {
		struct {
			struct komb_delegation_node *next;
			int tail;
			int count;
			int socket_id;
			int cpuid;
			int pos;
			void *rsp;
			struct qspinlock *lock;
			int irqs_disabled;
			struct task_struct *task_struct_ptr;
		};
		char alignment1[128];
	};

	union {
		struct {
			u8 completed;
			u8 locked;
		};
		struct {
			u16 locked_completed;
		};
		char alignment[128];
	};
};

/*
 * komb_init and komb_free should be called only when the system boots up and
 * shut down. They are used to setup and free per-core variables.
 */
void komb_delegation_init(int num_delegation_threads);
void komb_delegation_free(void);

/*
 * Public API
 */
extern void komb_delegation_spin_lock_init(struct qspinlock *lock);
extern void komb_delegation_spin_lock(struct qspinlock *lock);
extern void komb_delegation_spin_lock_nested(struct qspinlock *lock, int level);
extern void komb_delegation_spin_unlock(struct qspinlock *lock);
extern void komb_delegation_spin_unlock_nested(struct qspinlock *lock, int level);

#ifdef KOMB_STATS
void komb_delegation_print_stats(void);
#endif

#endif
