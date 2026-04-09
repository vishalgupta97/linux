#ifndef __BPF_KOMB_H__
#define __BPF_KOMB_H__

#include <asm/qspinlock.h>

struct komb_node {
	union {
		struct {
			struct komb_node *next;
			int tail;
			int count;
			int socket_id;
			int cpuid;
			void *rsp;
			struct qspinlock *lock;
			int irqs_disabled;
			struct task_struct *task_struct_ptr;
		};
		char alignment[128];
	};

	union {
		struct {
			u8 completed;
			u8 locked;
		};
		struct {
			u16 locked_completed;
		};
		char alignment1[128];
	};
};

#define _Q_COMPLETED_OFFSET (_Q_LOCKED_OFFSET + _Q_LOCKED_BITS)
#define _Q_COMPLETED_BITS 8
#define _Q_COMPLETED_MASK _Q_SET_MASK(COMPLETED)
#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)
#define _Q_LOCKED_COMBINER_VAL 3

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
#endif

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

void komb_init(void);
int komb_spin_trylock(struct qspinlock *lock);
void komb_spin_lock(struct qspinlock *lock);
void komb_spin_unlock(struct qspinlock *lock);

#endif
