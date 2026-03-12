/* faa: test module for stressing spinlocks using FAA
 * Written by Vishal Gupta
 * Adapted from rcuht
 */

#define XSTR(x) STR(x)
#define STR(x) #x

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

#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/mutex.h>
#include <linux/rwlock.h>
#include <linux/percpu-rwsem.h>
#include <linux/sched/clock.h>
#include <stdatomic.h>
#include <linux/delay.h>

#include "cpuseq.h"
#include "stresser.h"
#include "spinlock/qspinlock.h"
#include "spinlock/aqs.h"
#include "spinlock/cna.h"
#include "spinlock/komb.h"
#include "spinlock/komb_delegation.h"
//#include "spinlock/ffwd.h"
#include "spinlock/ffwd_orig.h"

#include "timing_stats.h"

#include "faa.h"

static char *reader_type = "table_spinlock"; /* Reader implementation to benchmark */
static char *writer_type = "table_spinlock"; /* Writer implementation to benchmark */
static int rw = 47; /* Number of mixed reader/writer threads; defaults to online CPUs */

long komb_batch_size = 1048576; //262144;

module_param(reader_type, charp, 0444);
MODULE_PARM_DESC(reader_type, "Hash table reader implementation");
module_param(writer_type, charp, 0444);
MODULE_PARM_DESC(writer_type, "Hash table writer implementation");
module_param(rw, int, 0444);
MODULE_PARM_DESC(rw, "Number of mixed reader/writer threads");

module_param(komb_batch_size, long, 0444);
MODULE_PARM_DESC(writer_range, "batch_size for komb");

struct stats {
	union {
		struct {
			volatile u64 counter;
			u64 time;
		};
		char cache_alignment[128];
	};
} ____cacheline_aligned;

struct rcuhashbash_ops {
	int (*write)(struct stats *stats);
	void (*write_lock_buckets)(void);
	void (*write_unlock_buckets)(void);
	bool limit_writers;
	int max_writers;
	const char *reader_type;
	const char *writer_type;
};

static struct rcuhashbash_ops *ops;

DECLARE_TABLE_LOCK(table_spinlock, DEFINE_SPINLOCK, spin_lock, spin_unlock);

DECLARE_TABLE_LOCK(table_komb, DEFINE_KOMBSPINLOCK, komb_spin_lock,
		   komb_spin_unlock);

DECLARE_TABLE_LOCK(table_komb_delegation, DEFINE_KOMB_DELEGATION_SPINLOCK, komb_delegation_spin_lock,
		   komb_delegation_spin_unlock);

//DECLARE_TABLE_LOCK(table_ffwd, DEFINE_FFWD, ffwd_lock,
//		                   ffwd_unlock);

DECLARE_TABLE_LOCK_FFWD(table_ffwd_orig);

DECLARE_TABLE_LOCK(table_aqs, DEFINE_AQSLOCK, __aqs_acquire, __aqs_release);

DECLARE_TABLE_LOCK(table_cna, DEFINE_CNALOCK, cna_spin_lock, cna_spin_unlock);


static struct task_struct **tasks;

static struct stats *thread_stats;

#if LOCK_MEASURE_TIME
static uint64_t prev_lock_holder;
static DEFINE_PER_CPU_ALIGNED(uint64_t, write_critical_section_loop);
#endif

static void noinline ffwd_write_critical_section(u32 src, u32 dst, void *stats_ptr) {
	((struct stats*)stats_ptr)->counter++;
}

static int rcuhashbash_write_ffwd(struct stats *stats)
{
	ffwd_send_request(ffwd_write_critical_section, 0, 0, stats);

	return 0;
}


static int rcuhashbash_write_lock(struct stats *stats)
{
	//LOCK_DEFINE_TIMING_VAR(write_critical_section);
	//LOCK_DEFINE_TIMING_VAR(write_path);

	LOCK_START_TIMING_DISABLE(writer_path_t, write_path);

	if (ops->write_lock_buckets)
		ops->write_lock_buckets();
	LOCK_START_TIMING_DISABLE(write_critical_section_t, write_critical_section);
	//LOCK_START_TIMING(write_critical_section_t, write_critical_section);
	 
	stats->counter++;

#if LOCK_MEASURE_TIME
//	*this_cpu_ptr(&write_critical_section_loop) = prev_lock_holder;
//	LOCK_END_TIMING_PER_CPU(write_critical_section_loop);
//	LOCK_START_TIMING_PER_CPU(write_critical_section_loop);
//	prev_lock_holder = *this_cpu_ptr(&write_critical_section_loop);
#endif

	LOCK_END_TIMING_DISABLE(write_critical_section_t, write_critical_section);
	//LOCK_END_TIMING(write_critical_section_t, write_critical_section);

	if (ops->write_unlock_buckets)
		ops->write_unlock_buckets();

	LOCK_END_TIMING_DISABLE(write_path_t, write_path);

	return 0;
}

static int rcuhashbash_rw_thread(void *arg)
{
	int err;
	struct stats *stats_ret = arg;
	//struct stats stats = {};
	struct timespec64 start_t, end_t;
	
#if LOCK_MEASURE_TIME
	*this_cpu_ptr(&do_timing) = true;
	// u64 counter = 0;
#endif

	/* set_user_nice(current, 19); */
	ktime_get_raw_ts64(&start_t);
	do {
		if (need_resched())
			cond_resched();

			err = ops->write(stats_ret);

	} while (!kthread_should_stop() && !err);
	ktime_get_raw_ts64(&end_t);

	end_t = timespec64_sub(end_t, start_t);

	cond_resched();

	stats_ret->time = timespec64_to_ns(&end_t);
	//*stats_ret = stats;

	__set_current_state(TASK_RUNNING);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);

	return err;
}

static struct rcuhashbash_ops all_ops[] = {
	{
		.reader_type = "table_spinlock",
		.writer_type = "table_spinlock",
		.write = rcuhashbash_write_lock,
		.write_lock_buckets = table_spinlock_write_lock_buckets,
		.write_unlock_buckets = table_spinlock_write_unlock_buckets,
	},
//        {
//                .reader_type = "table_ffwd",
//                .writer_type = "table_ffwd",
//                .write = rcuhashbash_write_lock,
//                .write_lock_buckets = table_ffwd_write_lock_buckets,
//                .write_unlock_buckets = table_ffwd_write_unlock_buckets,
//        },
        {
                .reader_type = "table_ffwd_orig",
                .writer_type = "table_ffwd_orig",
                .write = rcuhashbash_write_ffwd,
                .write_lock_buckets = table_ffwd_orig_write_lock_buckets,
                .write_unlock_buckets = table_ffwd_orig_write_unlock_buckets,
        },
	{
		.reader_type = "table_komb",
		.writer_type = "table_komb",
		.write = rcuhashbash_write_lock,
		.write_lock_buckets = table_komb_write_lock_buckets,
		.write_unlock_buckets = table_komb_write_unlock_buckets,
	},
	{
		.reader_type = "table_komb_delegation",
		.writer_type = "table_komb_delegation",
		.write = rcuhashbash_write_lock,
		.write_lock_buckets = table_komb_delegation_write_lock_buckets,
		.write_unlock_buckets = table_komb_delegation_write_unlock_buckets,
	},
	{
		.reader_type = "table_aqs",
		.writer_type = "table_aqs",
		.write = rcuhashbash_write_lock,
		.write_lock_buckets = table_aqs_write_lock_buckets,
		.write_unlock_buckets = table_aqs_write_unlock_buckets,
	},
	{
		.reader_type = "table_cna",
		.writer_type = "table_cna",
		.write = rcuhashbash_write_lock,
		.write_lock_buckets = table_cna_write_lock_buckets,
		.write_unlock_buckets = table_cna_write_unlock_buckets,
	},
};

static struct rcuhashbash_ops *ops;

static void rcuhashbash_print_stats(void)
{
	u64 min_time = thread_stats->time;
	u64 max_time = thread_stats->time;
	u64 counter = thread_stats->counter;

	if (!thread_stats) {
		printk(KERN_ALERT "rcuhashbash stats unavailable\n");
		return;
	}

	printk(KERN_ALERT
	       "rcuhashbash summary: rw=%d reader_type=%s writer_type=%s\n" KERN_ALERT
	       "rcuhashbash summary: total: %llu (avg: %llu min: %llu max: %llu)\n",
	       rw, reader_type, writer_type,
				 counter, min_time, min_time, max_time); 
}

static void rcuhashbash_exit(void)
{
	unsigned long i;
#if LOCK_MEASURE_TIME
	unsigned long i__, cpu;
#endif
	int ret;

	printk(KERN_ALERT "rcuhashbash exiting threads\n");

	//rcuht_helper_exit();

	if (tasks) {
		for (i = 0; i < rw; i++) {
			if (tasks[i]) {
				ret = kthread_stop(tasks[i]);
				if (ret)
					printk(KERN_ALERT
					       "rcuhashbash task returned error %d\n",
					       ret);
			}
		}
		vfree(tasks);
	}

	/* Wait for all RCU callbacks to complete. */
	rcu_barrier();

#if LOCK_MEASURE_TIME
	printk(KERN_ALERT "Preparing for Timing Stats\n");
	locktime_print_timing_stats();
	printk(KERN_ALERT "Free Timing Stats\n");
	for (i__ = 0; i__ < TIMING_NUM; i__ ++){
		for_each_possible_cpu (cpu){
			vfree(per_cpu(BucketTimingstats_percpu_locktime[i__], cpu));
		}
		vfree(BucketTimingsamples_bucket[i__]);
	}
#endif

#if KOMB_STATS
	komb_print_stats();
#endif
#if FFWD_STATS
	ffwd_print_stats();
#endif

	if (thread_stats)
		rcuhashbash_print_stats();

	if(strcmp(writer_type, "table_ffwd_orig") == 0)
		ffwd_delegate_exit();

	if(strcmp(writer_type, "table_komb_delegation") == 0 || strcmp(writer_type, "komb_delegation") == 0)
		komb_delegation_free();

	vfree(thread_stats);
	komb_free();
	//komb_rwfree();

	printk(KERN_ALERT "rcuhashbash done\n");
}

static __init int rcuhashbash_init(void)
{
	bool delegation_style;
	int ret;
	u32 i, new_cpuid;
#if LOCK_MEASURE_TIME
        u32 i__, cpu;
#endif

	for (i = 0; i < ARRAY_SIZE(all_ops); i++)
		if (strcmp(reader_type, all_ops[i].reader_type) == 0 &&
		    strcmp(writer_type, all_ops[i].writer_type) == 0) {
			ops = &all_ops[i];
		}
	if (!ops) {
		printk(KERN_ALERT
		       "rcuhashbash: No implementation with %s reader and %s writer\n",
		       reader_type, writer_type);
		return -EINVAL;
	}

	if (rw < 0)
		rw = num_online_cpus();
	if (ops->limit_writers && rw > ops->max_writers) {
		printk(KERN_ALERT
		       "rcuhashbash: %s writer implementation supports at most %d writers\n",
		       writer_type, ops->max_writers);
		return -EINVAL;
	}

	if (rw > 0 && !ops->write) {
		printk(KERN_ALERT
		       "rcuhashbash: Internal error: rw > 0 but write function NULL\n");
		return -EINVAL;
	}

	komb_init(); //Add call to initialize per-cpu variables for Komb.
	if(strcmp(writer_type, "table_ffwd_orig") == 0)
		ffwd_delegate_init();
	
	if(strcmp(writer_type, "table_komb_delegation") == 0 || strcmp(writer_type, "komb_delegation") == 0)
		komb_delegation_init(online_sockets);

	//thread_stats = kcalloc(1, sizeof(struct stats), GFP_KERNEL);
	thread_stats = vzalloc(sizeof(struct stats));
	if (!thread_stats)
		goto enomem;

	//tasks = kcalloc(rw, sizeof(tasks[0]), GFP_KERNEL);
	tasks = vzalloc(sizeof(struct task_struct*) * rw);
	if (!tasks)
		goto enomem;

#if LOCK_MEASURE_TIME
	for (i__ = 0; i__ < TIMING_NUM; i__ ++){
		for_each_possible_cpu(cpu){
			per_cpu(BucketTimingstats_percpu_locktime[i__], cpu) = vzalloc(sizeof(unsigned long) * N_BUCKETS);
			*per_cpu_ptr(&write_critical_section_loop, cpu) = UINT64_MAX;
		}
		BucketTimingsamples_bucket[i__] = vzalloc(sizeof(unsigned long) * TIME_UPPER_BOUND);
	}
	prev_lock_holder = 0;
#endif
	
	delegation_style = strcmp(writer_type, "table_komb_delegation") == 0 || strcmp(writer_type, "table_ffwd_orig") == 0 
						|| strcmp(writer_type, "komb_delegation") == 0;
	if(delegation_style)
		BUG_ON((rw) > (num_online_cpus()-online_sockets));
	else 
		BUG_ON((rw) > num_online_cpus());

	printk(KERN_ALERT "rcuhashbash starting threads\n");
	
	new_cpuid = 0;
	for (i = 0; i < rw; i++) {
		struct task_struct *task = kthread_create(rcuhashbash_rw_thread,
					      thread_stats,
					      "rcuhashbash_rw");
		if (IS_ERR(task)) {
			ret = PTR_ERR(task);
			goto error;
		}

		tasks[i] = task;

		if(delegation_style && ((i % (num_cores_per_socket - 1)) == 0))
			new_cpuid += 1;
		kthread_bind(tasks[i], cpuseq[new_cpuid]);
		new_cpuid += 1;
		wake_up_process(tasks[i]);
	}

	return 0;

enomem:
	komb_free();
	//komb_rwfree();
	ret = -ENOMEM;
error:
	rcuhashbash_exit();
	return ret;
}

module_init(rcuhashbash_init);
module_exit(rcuhashbash_exit);

MODULE_AUTHOR("Vishal Gupta");
MODULE_DESCRIPTION("FAA stress testing for spinlocks.");
MODULE_LICENSE("GPL");
