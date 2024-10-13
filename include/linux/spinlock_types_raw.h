#ifndef __LINUX_SPINLOCK_TYPES_RAW_H
#define __LINUX_SPINLOCK_TYPES_RAW_H

#include <linux/types.h>

#if defined(CONFIG_SMP)
#include <asm/spinlock_types.h>
#else
#include <linux/spinlock_types_up.h>
#endif

#include <linux/lockdep_types.h>
#include <linux/feedbacksync.h>

typedef struct raw_spinlock {
	arch_spinlock_t raw_lock;
	struct fds_lock_key key;
} raw_spinlock_t;

#define SPINLOCK_MAGIC 0xdead4ead

#define SPINLOCK_OWNER_INIT ((void *)-1L)

#define RAW_SPIN_DEP_MAP_INIT(lockname) \
	.key = {                        \
		.name = #lockname,      \
		.ptr = NULL,            \
	}
#define SPIN_DEP_MAP_INIT(lockname) \
	.key = {                    \
		.name = #lockname,  \
		.ptr = NULL,        \
	}

#define LOCAL_SPIN_DEP_MAP_INIT(lockname) \
	.key = {                          \
		.name = #lockname,        \
		.ptr = NULL,              \
	}

#ifdef CONFIG_DEBUG_SPINLOCK
#define SPIN_DEBUG_INIT(lockname) \
	.magic = SPINLOCK_MAGIC, .owner_cpu = -1, .owner = SPINLOCK_OWNER_INIT,
#else
#define SPIN_DEBUG_INIT(lockname)
#endif

#define __RAW_SPIN_LOCK_INITIALIZER(lockname)          \
	{                                              \
		.raw_lock = __ARCH_SPIN_LOCK_UNLOCKED, \
		RAW_SPIN_DEP_MAP_INIT(lockname)        \
	}

#define __RAW_SPIN_LOCK_UNLOCKED(lockname) \
	(raw_spinlock_t) __RAW_SPIN_LOCK_INITIALIZER(lockname)

#define DEFINE_RAW_SPINLOCK(x) raw_spinlock_t x = __RAW_SPIN_LOCK_UNLOCKED(x)

#endif /* __LINUX_SPINLOCK_TYPES_RAW_H */
