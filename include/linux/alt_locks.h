#ifndef __ALT_LOCKS_H__
#define __ALT_LOCKS_H__

#include <linux/spinlock.h>
#include <linux/mutex.h>

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

struct percpu_rw_semaphore;
extern void percpu_down_read(struct percpu_rw_semaphore *sem);
extern bool percpu_down_read_trylock(struct percpu_rw_semaphore *sem);
extern void percpu_up_read(struct percpu_rw_semaphore *sem);
extern bool percpu_is_read_locked(struct percpu_rw_semaphore *);
extern bool percpu_rwsem_is_locked(struct percpu_rw_semaphore *);
extern void percpu_down_write(struct percpu_rw_semaphore *);
extern bool percpu_down_write_trylock(struct percpu_rw_semaphore *);
extern void percpu_up_write(struct percpu_rw_semaphore *);
extern bool percpu_is_write_locked(struct percpu_rw_semaphore *sem);
extern int percpu_down_write_killable(struct percpu_rw_semaphore *);
extern int percpu_down_read_killable(struct percpu_rw_semaphore *);
extern int percpu_rwsem_is_contended(struct percpu_rw_semaphore *);
extern int __percpu_init_rwsem(struct percpu_rw_semaphore *,const char *);

extern void percpu_free_rwsem(struct percpu_rw_semaphore *);

#define percpu_init_rwsem(sem)					\
({								\
	__percpu_init_rwsem(sem, #sem);		\
})

extern void percpu_downgrade_write(struct percpu_rw_semaphore *sem);

# define percpu_down_read_nested(sem, subclass)			percpu_down_read(sem)
# define percpu_down_write_nested(sem, subclass)		percpu_down_write(sem)
# define percpu_down_read_non_owner(sem)			percpu_down_read(sem)

#define alt_rw_semaphore 		percpu_rw_semaphore
#define __ALT_RWSEM_INITIALIZER 	DEFINE_PER_CPU
#define alt_rwsem_assert_held(l)	do { (void)(l); } while (0)
#define alt_rwsem_assert_held_write(l)	do { (void)(l); } while (0)
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
#endif
