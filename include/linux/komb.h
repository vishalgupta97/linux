#ifndef __KOMB_H__
#define __KOMB_H__

#include <asm/qspinlock.h>

#define MAX_NODES 4
#define DEFINE_KOMBSPINLOCK(x) \
	arch_spinlock_t(x) = (arch_spinlock_t)__ORIG_QSPIN_LOCK_UNLOCKED

/*
 * TODO (Correctness optimization): 
 * Add for BIG ENDIAN
 */
struct komb_node {
	union {
		struct {
			struct komb_node *next;
			struct komb_node *prev;
			int tail;
			int count;
			int socket_id;
			int cpuid;
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

struct shadow_stack {
	union {
		struct {
			uint64_t lock_addr[8];
			void *ptr;
			uint32_t curr_cs_cpu;
			uint32_t prev_cs_cpu;
			uint64_t counter_val;
			struct komb_node *next_node_ptr;
			uint64_t local_shadow_stack_ptr;
			struct komb_node *local_queue_head;
			struct komb_node *local_queue_tail;
			int irqs_disabled;
			bool is_local_queue_tail_last;
		};
		char dummy[128];
	};
};

#define _Q_COMPLETED_OFFSET (_Q_LOCKED_OFFSET + _Q_LOCKED_BITS)
#define _Q_COMPLETED_BITS 8
#define _Q_COMPLETED_MASK _Q_SET_MASK(COMPLETED)

/*
 * komb_init and komb_free should be called only when the system boots up and
 * shut down. They are used to setup and free per-core variables.
 */
void komb_init(void);
void komb_free(void);

/*
 * Public API
 */
extern void komb_spin_lock_init(struct qspinlock *lock);
extern int komb_spin_is_locked(struct qspinlock *lock);
extern int komb_spin_value_unlocked(struct qspinlock lock);
extern int komb_spin_is_contended(struct qspinlock *lock);
extern int komb_spin_trylock(struct qspinlock *lock);
extern void komb_spin_lock(struct qspinlock *lock);
extern void komb_spin_lock_fds(struct qspinlock *lock, enum fds_lock_mechanisms lockm);
extern void komb_spin_unlock(struct qspinlock *lock);

extern void kd_spin_lock(struct qspinlock *lock);
extern void kd_spin_unlock(struct qspinlock *lock);
#endif
