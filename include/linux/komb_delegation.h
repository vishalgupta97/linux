#ifndef __KOMB_DELEGATION_H__
#define __KOMB_DELEGATION_H__

#define ENABLE_IRQS_CHECK 1
#include <asm/qspinlock.h>

#define MAX_NODES 4
#define DEFINE_KOMBSPINLOCK(x) \
	arch_spinlock_t(x) = (arch_spinlock_t)__ORIG_QSPIN_LOCK_UNLOCKED

#define KOMB_STATS 1

/*
 * TODO (Correctness optimization): 
 * Add for BIG ENDIAN
 */
struct kd_node {
	union {
		struct {
			struct kd_node *next;
			struct kd_node *prev;
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
void kd_init(void);
void kd_free(void);

/*
 * Public API
 */
extern void kd_spin_lock_init(struct qspinlock *lock);
extern void kd_spin_lock(struct qspinlock *lock);
extern bool kd_spin_trylock(struct qspinlock *lock);
extern void kd_spin_unlock(struct qspinlock *lock);
extern int kd_spin_is_locked(struct qspinlock *lock);
extern int kd_spin_value_unlocked(struct qspinlock lock);
extern int kd_spin_is_contended(struct qspinlock *lock);

#endif
