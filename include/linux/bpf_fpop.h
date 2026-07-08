#ifndef __BPF_FPOP_H__
#define __BPF_FPOP_H__

#include <asm/qspinlock.h>

typedef u64 (*bpf_callback_t)(u64, u64, u64, u64, u64);

struct fpop_node {
	union {
		struct {
			struct fpop_node *next;
			bpf_callback_t callback;
			u64 v1;
			u64 v2;
			u64 v3;
			int tail;
			int socket_id;
			int cpuid;
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

#define _FPOP_PRCSD 0x0002
#define _FPOP_PRCSING 0x0001
#define _FPOP_UNPRCSD 0x0000

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

void fpop_execute(struct qspinlock *lock, bpf_callback_t callback,
		  u64 v1, u64 v2, u64 v3);

#endif // __BPF_FPOP_H__
