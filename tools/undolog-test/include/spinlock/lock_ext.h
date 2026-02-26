#ifndef __LOCK_EXT_H__
#define __LOCK_EXT_H__

#ifdef KERNEL_SYNCSTRESS
#include "qspinlock_i.h"
#include "lib/combiner.h"
#else
#include <asm/qspinlock.h>
#endif

#define DEFINE_LOCK_EXT_SPINLOCK(x)                                                 \
	struct lock_ext_spinlock(x) = (struct lock_ext_spinlock)__ORIG_QSPIN_LOCK_UNLOCKED

#define LOCK_TAS 0
#define LOCK_QSPINLOCK 1
#define LOCK_KOMB 2
#define LOCK_DELEGATION 3

/*
 * TODO (Correctness optimization): 
 * Add for BIG ENDIAN
 */
struct lock_ext_spinlock {
	union {
		atomic_t val;

		/*
		 * bit 0-7: Locked
		 * bit 8-15: lock_type
		 * bit 16-17: tail index
		 * bit 18-31: tail cpu
		 */
		struct {
			u8 locked;
			u8 type;
		};
		struct {
			u16 locked_type;
			u16 tail;
		};
	};
};

/*
 * Bitfields in the atomic value:
 *
 *  0- 7: locked byte
 *  8-15: type
 * 16-17: tail index
 * 18-31: tail cpu (+1)
 */
#define	_LE_SET_MASK(type)	(((1U << _LE_ ## type ## _BITS) - 1)\
                                 << _LE_ ## type ## _OFFSET)

/* This is directly used by the tail bytes (2 bytes) */
#define _LE_LOCKED_OFFSET 0
#define _LE_LOCKED_BITS 8
#define _LE_LOCKED_MASK _LE_SET_MASK(LOCKED)

#define _LE_LOCK_TYPE_OFFSET (_LE_LOCKED_OFFSET + _LE_LOCKED_BITS)
#define _LE_LOCK_TYPE_BITS 8
#define _LE_LOCK_TYPE_MASK _LE_SET_MASK(LOCK_TYPE)

#define _LE_TAIL_IDX_OFFSET (_LE_LOCK_TYPE_OFFSET + _LE_LOCK_TYPE_BITS)
#define _LE_TAIL_IDX_BITS 2
#define _LE_TAIL_IDX_MASK _LE_SET_MASK(TAIL_IDX)

#define _LE_TAIL_CPU_OFFSET (_LE_TAIL_IDX_OFFSET + _LE_TAIL_IDX_BITS)
#define _LE_TAIL_CPU_BITS (32 - _LE_TAIL_CPU_OFFSET)
#define _LE_TAIL_CPU_MASK _LE_SET_MASK(TAIL_CPU)

#define _LE_TAIL_OFFSET _LE_TAIL_IDX_OFFSET
#define _LE_TAIL_MASK (_LE_TAIL_IDX_MASK | _LE_TAIL_CPU_MASK)

#define _LE_LOCKED_VAL (1U << _LE_LOCKED_OFFSET)
#define _LE_LOCKED_COMBINER_VAL 3U
#define _LE_LOCKED_DELEGATION_VAL 7U
#define _LE_UNLOCKED_OOO_VAL 15U //Unlocked a lock out-of-order

/*
 * TODO (Correctness optimization): 
 * Add for BIG ENDIAN
 */
struct lock_ext_node {
	union{
		struct {
			struct lock_ext_node *next;
			int tail;
			int count;
			int socket_id;
			int cpuid;
			void* rsp;
			struct lock_ext_spinlock *lock;
			u8 lock_type;
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

#define _LE_COMPLETED_OFFSET (_LE_LOCKED_OFFSET + _LE_LOCKED_BITS)
#define _LE_COMPLETED_BITS 8
#define _LE_COMPLETED_MASK _LE_SET_MASK(COMPLETED)

/*
 * lock_ext_init and lock_ext_free should be called only when the system boots up and
 * shut down. They are used to setup and free per-core variables.
 */
void lock_ext_init(struct lock_ext_spinlock *lock);
void lock_ext_free(void);

/*
 * Public API
 */
extern void lock_ext_spin_lock_init(struct lock_ext_spinlock *lock);
extern int lock_ext_spin_is_locked(struct lock_ext_spinlock *lock);
extern int lock_ext_spin_value_unlocked(struct lock_ext_spinlock lock);
extern int lock_ext_spin_is_contended(struct lock_ext_spinlock *lock);
extern int lock_ext_spin_trylock(struct lock_ext_spinlock *lock);
extern void lock_ext_spin_lock(struct lock_ext_spinlock *lock);
extern void lock_ext_spin_unlock(struct lock_ext_spinlock *lock);

struct task_struct *lock_ext_get_current(spinlock_t *lock);
void lock_ext_set_current_state(spinlock_t *lock, unsigned int state);

#ifdef KOMB_STATS
void lock_ext_print_stats(void);
#endif

#endif
