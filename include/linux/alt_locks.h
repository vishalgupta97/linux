#ifndef __ALT_LOCKS_H__
#define __ALT_LOCKS_H__

#include <linux/komb_mutex.h>

#define alt_mutex komb_mutex
#define alt_mutex_init komb_mutex_init
#define alt_mutex_lock komb_mutex_lock
#define alt_mutex_unlock komb_mutex_unlock
#define alt_mutex_trylock komb_mutex_trylock

#endif
