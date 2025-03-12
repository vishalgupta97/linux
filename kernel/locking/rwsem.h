
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <linux/sched/stat.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

#if DSM_DEBUG
#define print_debug(fmt, ...)                                              \
	({                                                                 \
		printk(KERN_EMERG "[%d] komb (%s) lock(%px): " fmt,        \
		       smp_processor_id(), __func__, lock, ##__VA_ARGS__); \
	})
#define print_debug_without_lock(fmt, ...)                                    \
	({                                                                    \
		printk(KERN_EMERG "[%d] komb (%s): " fmt, smp_processor_id(), \
		       __func__, ##__VA_ARGS__);                              \
	})
#else
#define print_debug(fmt, ...)
#define print_debug_without_lock(fmt, ...)
#endif

#if DEBUG_KOMB
#define KOMB_BUG_ON(cond_expr) BUG_ON(cond_expr)
#else
#define KOMB_BUG_ON(cond_expr)
#endif

#define READ_STATE 0
#define WRITE_STATE 1

#define KOMB_WAITER_UNPROCESSED 0
#define KOMB_WAITER_PARKED 1
#define KOMB_WAITER_PROCESSING 2
#define KOMB_WAITER_PROCESSED 4
#define KOMB_WAITER_WAKER 8

#define KOMB_WAKER_COUNT_SHIFT 16
#define KOMB_WAKER_COUNT (1U << KOMB_WAKER_COUNT_SHIFT)

extern void schedule_out_curr_task(void);

#define smp_cond_load_relaxed_sleep(curr_node, ptr, cond_expr)                 \
	({                                                                     \
		typeof(ptr) __PTR = (ptr);                                     \
		__unqual_scalar_typeof(*ptr) VAL;                              \
		for (;;) {                                                     \
			VAL = READ_ONCE(*__PTR);                               \
			if (cond_expr)                                         \
				break;                                         \
			cpu_relax();                                           \
			if (need_resched()) {                                  \
				if (single_task_running())                     \
					schedule_out_curr_task();              \
				else {                                         \
					if (READ_ONCE(curr_node->completed) == \
					    KOMB_WAITER_UNPROCESSED)           \
						park_waiter(curr_node);        \
					else                                   \
						schedule_out_curr_task();      \
				}                                              \
			}                                                      \
		}                                                              \
		(typeof(*ptr))VAL;                                             \
	})

#define smp_cond_load_relaxed_sched(ptr, cond_expr)       \
	({                                                \
		typeof(ptr) __PTR = (ptr);                \
		__unqual_scalar_typeof(*ptr) VAL;         \
		for (;;) {                                \
			VAL = READ_ONCE(*__PTR);          \
			if (cond_expr)                    \
				break;                    \
			cpu_relax();                      \
			if (need_resched()) {             \
				schedule_out_curr_task(); \
			}                                 \
		}                                         \
		(typeof(*ptr))VAL;                        \
	})

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr) \
	({                                                     \
		typeof(ptr) __PTR = (ptr);                     \
		__unqual_scalar_typeof(*ptr) VAL;              \
		for (;;) {                                     \
			VAL = READ_ONCE(*__PTR);               \
			if (cond_expr)                         \
				break;                         \
			cpu_relax();                           \
			park_komb_rwsem_thread();              \
		}                                              \
		(typeof(*ptr))VAL;                             \
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

#define atomic_long_cond_read_acquire_sched(v, c) \
	smp_cond_load_acquire_sched(&(v)->counter, (c))

#define _Q_UNLOCKED_OOO_VAL 7 //Unlocked a lock out-of-order

//#define KOMB_WAITER_UNPROCESSED 0
//#define KOMB_WAITER_PARKED 1
//#define KOMB_WAITER_PROCESSING 2
//#define KOMB_WAITER_PROCESSED 4

#define UINT64_MAX 0xffffffffffffffffL

#define PERCPU_TABLE_SIZE 1024

extern void komb_rwsemd_down_write(struct rw_semaphore *lock);
extern void wait_for_visible_readers(struct rw_semaphore *lock);
