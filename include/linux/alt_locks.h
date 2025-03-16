#ifndef __ALT_LOCKS_H__
#define __ALT_LOCKS_H__

#include <linux/komb_mutex.h>
#include <linux/komb_rwsem.h>

#define alt_mutex 			komb_mutex
#define alt_mutex_init 			komb_mutex_init
#define alt_mutex_lock 			komb_mutex_lock
#define alt_mutex_unlock 		komb_mutex_unlock
#define alt_mutex_trylock 		komb_mutex_trylock

#define alt_rw_semaphore 		komb_rwsem
#define __ALT_RWSEM_INITIALIZER 	__KOMB_RWSEM_INITIALIZER
#define alt_rwsem_assert_held 		komb_rwsem_assert_held
#define alt_rwsem_assert_held_write 	komb_rwsem_assert_held_write
#define alt_init_rwsem 			komb_init_rwsem
#define alt_down_write			komb_down_write
#define alt_down_write_nested		komb_down_write_nested
#define alt_down_write_killable		komb_down_write_killable
#define alt_up_write			komb_up_write	
#define alt_downgrade_write		komb_downgrade_write
#define alt_down_write			komb_down_write
#define alt_down_read			komb_down_read
#define alt_down_read_killable		komb_down_read_killable
#define alt_down_read_trylock		komb_down_read_trylock
#define alt_up_read			komb_up_read
#define alt_up_read_non_owner		komb_up_read
#define alt_rwsem_is_contended		komb_rwsem_is_contended

#endif
