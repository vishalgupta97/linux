#ifndef __RCU_HT_H_
#define __RCU_HT_H_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/topology.h>
#include <linux/compiler.h>
#include <linux/random.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/percpu.h>
#include <linux/atomic.h>
#include <linux/ktime.h>
#include <asm/byteorder.h>
#include <linux/sort.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/wait.h>

#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/mutex.h>
#include <linux/rwlock.h>
#include <linux/percpu-rwsem.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>

struct rcu_random_state {
		unsigned long rrs_state;
		long rrs_count;
};

extern unsigned long rcu_random(struct rcu_random_state *rrsp);

#define DECLARE_TABLE_LOCK(l, linit, wl, wul, rl, rul)                                             \
                                                                                                   \
		static __cacheline_aligned_in_smp linit(l);                                                \
                                                                                                   \
		static void l##_write_lock_buckets(struct rcuhashbash_bucket *b1,                          \
										   struct rcuhashbash_bucket *b2)                          \
		{                                                                                          \
				preempt_disable();                                                                 \
				wl(&l);                                                                            \
		}                                                                                          \
                                                                                                   \
		static void l##_write_unlock_buckets(struct rcuhashbash_bucket *b1,                        \
											 struct rcuhashbash_bucket *b2)                        \
		{                                                                                          \
				wul(&l);                                                                           \
				preempt_enable();                                                                  \
		}                                                                                          \
                                                                                                   \
		static void l##_read_lock_bucket(struct rcuhashbash_bucket *bucket)                        \
		{                                                                                          \
				preempt_disable();                                                                 \
				rl(&l);                                                                            \
		}                                                                                          \
                                                                                                   \
		static void l##_read_unlock_bucket(struct rcuhashbash_bucket *bucket)                      \
		{                                                                                          \
				rul(&l);                                                                           \
				preempt_enable();                                                                  \
		}

#define DECLARE_PARTITIONED_TABLE_LOCK(l, linit, wl, wul, rl, rul)                                 \
                                                                                                   \
		static __cacheline_aligned_in_smp linit(l);                                                \
		static __cacheline_aligned_in_smp linit(l##_2);                                            \
                                                                                                   \
		static void l##_write_lock_buckets(struct rcuhashbash_bucket *b1,                          \
										   struct rcuhashbash_bucket *b2)                          \
		{                                                                                          \
				preempt_disable();                                                                 \
				if (((uint64_t)b1 < (buckets / 2)) == ((uint64_t)b2 < (buckets / 2))) {            \
						if ((uint64_t)b1 < (buckets / 2))                                          \
								wl(&l);                                                            \
						else                                                                       \
								wl(&l##_2);                                                        \
				} else {                                                                           \
						wl(&l);                                                                    \
						wl##_nested(&l##_2, 1);                                                    \
				}                                                                                  \
		}                                                                                          \
                                                                                                   \
		static void l##_write_unlock_buckets(struct rcuhashbash_bucket *b1,                        \
											 struct rcuhashbash_bucket *b2)                        \
		{                                                                                          \
				if (((uint64_t)b1 < (buckets / 2)) == ((uint64_t)b2 < (buckets / 2))) {            \
						if ((uint64_t)b1 < (buckets / 2))                                          \
								wul(&l);                                                           \
						else                                                                       \
								wul(&l##_2);                                                       \
				} else {                                                                           \
						wul(&l##_2);                                                               \
						wul(&l);                                                                   \
				}                                                                                  \
				preempt_enable();                                                                  \
		}                                                                                          \
                                                                                                   \
		static void l##_read_lock_bucket(struct rcuhashbash_bucket *bucket)                        \
		{                                                                                          \
				preempt_disable();                                                                 \
				if ((uint64_t)bucket < (buckets / 2))                                              \
						rl(&l);                                                                    \
				else                                                                               \
						rl(&l##_2);                                                                \
		}                                                                                          \
                                                                                                   \
		static void l##_read_unlock_bucket(struct rcuhashbash_bucket *bucket)                      \
		{                                                                                          \
				if ((uint64_t)bucket < (buckets / 2))                                              \
						rul(&l);                                                                   \
				else                                                                               \
						rul(&l##_2);                                                               \
				preempt_enable();                                                                  \
		}

#define DECLARE_TABLE_LOCK_W_NODE(l, linit, wl, wul, rl, rul, _t_)                                 \
                                                                                                   \
		static __cacheline_aligned_in_smp linit(l);                                                \
                                                                                                   \
		static void l##_write_lock_buckets(struct rcuhashbash_bucket *b1,                          \
										   struct rcuhashbash_bucket *b2)                          \
		{                                                                                          \
				struct aqm_node node ____cacheline_aligned;                                        \
				wl(&l, &node, _t_);                                                                \
		}                                                                                          \
                                                                                                   \
		static void l##_write_unlock_buckets(struct rcuhashbash_bucket *b1,                        \
											 struct rcuhashbash_bucket *b2)                        \
		{                                                                                          \
				wul(&l);                                                                           \
		}                                                                                          \
                                                                                                   \
		static void l##_read_lock_bucket(struct rcuhashbash_bucket *bucket)                        \
		{                                                                                          \
				struct aqm_node node ____cacheline_aligned;                                        \
				rl(&l, &node, _t_);                                                                \
		}                                                                                          \
                                                                                                   \
		static void l##_read_unlock_bucket(struct rcuhashbash_bucket *bucket)                      \
		{                                                                                          \
				rul(&l);                                                                           \
		}

#define DECLARE_TABLE_LOCK_FFWD(l)                                                                 \
                                                                                                   \
		static void l##_write_lock_buckets(struct rcuhashbash_bucket *b1,                          \
										   struct rcuhashbash_bucket *b2)                          \
		{                                                                                          \
				BUG_ON(true);                                                                      \
		}                                                                                          \
                                                                                                   \
		static void l##_write_unlock_buckets(struct rcuhashbash_bucket *b1,                        \
											 struct rcuhashbash_bucket *b2)                        \
		{                                                                                          \
				BUG_ON(true);                                                                      \
		}                                                                                          \
                                                                                                   \
		static void l##_read_lock_bucket(struct rcuhashbash_bucket *bucket)                        \
		{                                                                                          \
				BUG_ON(true);                                                                      \
		}                                                                                          \
                                                                                                   \
		static void l##_read_unlock_bucket(struct rcuhashbash_bucket *bucket)                      \
		{                                                                                          \
				BUG_ON(true);                                                                      \
		}

#define DECLARE_TABLE_LOCK_W_NODE(l, linit, wl, wul, rl, rul, _t_)                                 \
                                                                                                   \
		static __cacheline_aligned_in_smp linit(l);                                                \
                                                                                                   \
		static void l##_write_lock_buckets(struct rcuhashbash_bucket *b1,                          \
										   struct rcuhashbash_bucket *b2)                          \
		{                                                                                          \
				struct aqm_node node ____cacheline_aligned;                                        \
				wl(&l, &node, _t_);                                                                \
		}                                                                                          \
                                                                                                   \
		static void l##_write_unlock_buckets(struct rcuhashbash_bucket *b1,                        \
											 struct rcuhashbash_bucket *b2)                        \
		{                                                                                          \
				wul(&l);                                                                           \
		}                                                                                          \
                                                                                                   \
		static void l##_read_lock_bucket(struct rcuhashbash_bucket *bucket)                        \
		{                                                                                          \
				struct aqm_node node ____cacheline_aligned;                                        \
				rl(&l, &node, _t_);                                                                \
		}                                                                                          \
                                                                                                   \
		static void l##_read_unlock_bucket(struct rcuhashbash_bucket *bucket)                      \
		{                                                                                          \
				rul(&l);                                                                           \
		}

#endif /* __RCU_HT_H_ */
