//#ifndef __ALT_LOCKS_H__
//#define __ALT_LOCKS_H__

#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/percpu-rwsem.h>

#define alt_spinlock_t			spinlock_t
#define alt_spin_lock_init		spin_lock_init
#define alt_spin_lock			spin_lock
#define alt_spin_unlock			spin_unlock
#define __ALT_SPIN_LOCK_UNLOCKED	__SPIN_LOCK_UNLOCKED

#define alt_mutex 			mutex
#define alt_mutex_init 			mutex_init
#define alt_mutex_lock 			mutex_lock
#define alt_mutex_unlock 		mutex_unlock
#define alt_mutex_trylock 		mutex_trylock

#define alt_rw_semaphore 		percpu_rw_semaphore
#define __ALT_RWSEM_INITIALIZER 	DEFINE_PER_CPU
#define alt_rwsem_assert_held 		percpu_rwsem_assert_held
#define alt_rwsem_assert_held_write 	percpu_rwsem_assert_held_write
#define alt_init_rwsem 			percpu_init_rwsem
#define alt_down_write			percpu_down_write
#define alt_down_write_nested		percpu_down_write_nested
#define alt_down_write_killable		percpu_down_write_killable
#define alt_up_write			percpu_up_write	
#define alt_downgrade_write		percpu_downgrade_write
#define alt_down_write			percpu_down_write
#define alt_down_read			percpu_down_read
#define alt_down_read_killable		percpu_down_read_killable
#define alt_down_read_trylock		percpu_down_read_trylock
#define alt_up_read			percpu_up_read
#define alt_up_read_non_owner		percpu_up_read
#define alt_rwsem_is_contended		percpu_rwsem_is_contended
#define alt_rwsem_is_locked		percpu_rwsem_is_locked
#define alt_down_read_nested		percpu_down_read_nested
#define alt_down_write_trylock		percpu_down_write_trylock
//#endif
