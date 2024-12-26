#ifndef __KOMB_MUTEX_DELEGATION_H__
#define __KOMB_MUTEX_DELEGATION_H__

#ifdef KERNEL_SYNCSTRESS
#include "komb_mutex.h"
#include "topology.h"
#else
#include <linux/combiner.h>
#define _Q_COMPLETED_OFFSET (_Q_LOCKED_OFFSET + _Q_LOCKED_BITS)
#define _Q_COMPLETED_BITS 8
#define _Q_COMPLETED_MASK _Q_SET_MASK(COMPLETED)
#endif

/*
 * TODO (Correctness optimization): 
 * Add for BIG ENDIAN
 */
struct kombd_mutex_node {
	union {
		struct {
			struct kombd_mutex_node *next;
			struct kombd_mutex_node *tail;
			int socket_id;
			int cpuid;
			
            void *rsp;
			void *lock;
			struct task_struct *task_struct_ptr;
		};
		char alignment1[128];
	};

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
};

struct kombd_mutex {
	/* These two will be sufficient to design simple komb lock */
	struct kombd_mutex_node *tail;
	union {
		atomic_t state;
		struct {
			u8 locked;
		};
	};
	struct task_struct *combiner_task; //TODO: Merge locked bit with owner.
};


#define __KOMBMUTEX_DELEGATION_INITIALIZER(lockname)             {                         \
		.tail = NULL, .state = ATOMIC_INIT(0), .combiner_task = NULL }  \

#define DEFINE_KOMBMUTEX_DELEGATION(mname)                                                \
	struct kombd_mutex mname = __KOMBMUTEX_DELEGATION_INITIALIZER(mname)


/*
 * komb_init and komb_free should be called only when the system boots up and
 * shut down. They are used to setup and free per-core variables.
 */
void kombd_mutex_init(void);
void kombd_mutex_free(void);

/*
 * Public API
 */
extern void kombd_mutex_lock_init(struct kombd_mutex *lock);
extern void kombd_mutex_lock(struct kombd_mutex *lock);
extern void kombd_mutex_unlock(struct kombd_mutex *lock);

#ifdef KOMB_STATS
void kombd_mutex_print_stats(void);
#endif

#endif
