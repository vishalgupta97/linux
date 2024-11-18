#ifndef __MUTEX_DELEGATION_H__
#define __MUTEX_DELEGATION_H__

#include <linux/mutex.h>

#define KOMB_STATS 1

/*
 * Public API
 */
extern void md_spin_lock(struct mutex *lock);

#endif
