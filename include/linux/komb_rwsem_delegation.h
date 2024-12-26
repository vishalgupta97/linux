#ifndef __KOMB_RWSEM_DELEGATION_H__
#define __KOMB_RWSEM_DELEGATION_H__

#ifdef KERNEL_SYNCSTRESS
#include "komb_rwsem.h"
#include "topology.h"
#else
#include <linux/aqm.h>
#include <linux/komb_mutex_delegation.h>

#define WRITE_BOUNDED_OPPORTUNISTIC_SPIN 0

#define _KOMB_RWSEM_W_LOCKED 0xff /* A writer holds the lock */
#define _KOMB_RWSEM_W_COMBINER 0x70 /* A combiner holds the lock */
#define _KOMB_RWSEM_W_OOO 0x7f /* A combiner holds the lock */
#define _KOMB_RWSEM_W_WMASK 0x1ff /* Writer mask		   */
#define _KOMB_RWSEM_W_WAITING 0x100 /* Writer waiting */
#define _KOMB_RWSEM_R_SHIFT 9 /* Reader count shift	   */
#define _KOMB_RWSEM_R_BIAS (1U << _KOMB_RWSEM_R_SHIFT)
#define _KOMB_RWSEM_W_DOWNGRADE 0x77 /* A writer combiner requested downgrade */

#define _Q_COMPLETED_OFFSET (_Q_LOCKED_OFFSET + _Q_LOCKED_BITS)
#define _Q_COMPLETED_BITS 8
#define _Q_COMPLETED_MASK _Q_SET_MASK(COMPLETED)
#endif

struct kombd_rwsem {
	union {
		atomic_long_t cnts;
		struct {
			u8 wlocked;
			u8 rcount[7];
		};
	};
	char dummy1[128];
	char dummy2[128];
	struct aqm_mutex reader_wait_lock;
	char dummy3[128];
	struct kombd_mutex_node *writer_tail;
	char dummy4[128];
};

#define __KOMB_RWSEM_DELEGATION_INITIALIZER(lockname)             \
	{                                                         \
		.cnts = ATOMIC_LONG_INIT(0), .writer_tail = NULL, \
		.reader_wait_lock.val = ATOMIC_INIT(0),           \
		.reader_wait_lock.tail = NULL                     \
	}

#define DECLARE_KOMB_RWSEM_DELEGATION(krwsemname) \
	struct kombd_rwsem krwsemname =           \
		__KOMB_RWSEM_DELEGATION_INITIALIZER(krwsemname)

/*
 * komb_init and komb_free should be called only when the system boots up and
 * shut down. They are used to setup and free per-core variables.
 */
void kombd_rwsem_init(int num_delegation_threads);
void kombd_rwsem_free(void);

/*
 * Public API
 */
void kombd_init_rwsem(struct kombd_rwsem *sem);
void kombd_rwsem_down_read(struct kombd_rwsem *sem);
bool kombd_rwsem_down_read_trylock(struct kombd_rwsem *sem);
void kombd_rwsem_down_write(struct kombd_rwsem *sem);
void kombd_rwsem_down_write_nested(struct kombd_rwsem *sem, int subclass);
void kombd_rwsem_downgrade_write(struct kombd_rwsem *sem);
void kombd_rwsem_up_read(struct kombd_rwsem *sem);
void kombd_rwsem_up_read_non_owner(struct kombd_rwsem *sem);
void kombd_rwsem_up_write(struct kombd_rwsem *sem);
bool kombd_rwsem_is_contended(struct kombd_rwsem *sem);

#ifdef KOMB_STATS
void kombd_rwsem_print_stats(void);
#endif

#endif
