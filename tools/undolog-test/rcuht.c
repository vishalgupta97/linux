/* undolog-test: test undo logging support.
 */

#define XSTR(x) STR(x)
#define STR(x) #x

#include "rcuht.h"

#include "cpuseq.h"
#include "stresser.h"
#include "spinlock/qspinlock.h"
#include "spinlock/aqs.h"
#include "spinlock/cna.h"
#include "spinlock/komb.h"

#include "linux/bpf_qspinlock.h"

#define RCU_RANDOM_MULT 39916801 /* prime */
#define RCU_RANDOM_ADD 479001701 /* prime */
#define RCU_RANDOM_REFRESH 10000

#define RCUHT_STAT_GAP 1000

#define DEFINE_RCU_RANDOM(name) struct rcu_random_state name = { 0, 0 }

static char *reader_type = "rcu"; /* Reader implementation to benchmark */
static char *writer_type = "spinlock"; /* Writer implementation to benchmark */
static int ro = 0; /* Number of reader-only threads; defaults to 0 */
static int rw = -1; /* Number of mixed reader/writer threads; defaults to online CPUs */
static unsigned long rw_writes = 1; /* Number of writes out of each total */
static unsigned long rw_total = 100; /* Total rw operations to divide into readers and writers */
static unsigned long buckets = 1024; /* Number of hash table buckets */
static unsigned long entries = 4096; /* Number of entries initially added */
static unsigned long reader_range = 0; /* Upper bound of reader range */
static unsigned long writer_range = 0; /* Upper bound of writer range */

long komb_batch_size = 1048576; //262144;

module_param(reader_type, charp, 0444);
MODULE_PARM_DESC(reader_type, "Hash table reader implementation");
module_param(writer_type, charp, 0444);
MODULE_PARM_DESC(writer_type, "Hash table writer implementation");
module_param(ro, int, 0444);
MODULE_PARM_DESC(ro, "Number of reader-only threads");
module_param(rw, int, 0444);
MODULE_PARM_DESC(rw, "Number of mixed reader/writer threads");
module_param(rw_writes, ulong, 0444);
MODULE_PARM_DESC(rw_writes, "Number of writes out of each total");
module_param(rw_total, ulong, 0444);
MODULE_PARM_DESC(rw_total, "Total rw operations to divide into readers and writers");
module_param(buckets, ulong, 0444);
MODULE_PARM_DESC(buckets, "Number of hash buckets");
module_param(entries, ulong, 0444);
MODULE_PARM_DESC(entries, "Number of hash table entries");
module_param(reader_range, ulong, 0444);
MODULE_PARM_DESC(reader_range, "Upper bound of reader operating range (default 2*entries)");
module_param(writer_range, ulong, 0444);
MODULE_PARM_DESC(writer_range, "Upper bound of writer operating range (default 2*entries)");

module_param(komb_batch_size, long, 0444);
MODULE_PARM_DESC(writer_range, "batch_size for komb");

struct rcuhashbash_bucket {
		struct hlist_head head;
		union {
				spinlock_t spinlock;
				rwlock_t rwlock;
				struct mutex mutex;
				struct rw_semaphore rwsem;
				struct percpu_rw_semaphore percpu_rwsem;
				struct orig_qspinlock komb;
		};
};

struct stats {
		union {
				struct {
						u64 read_hits;
						u64 read_misses;
						u64 write_moves;
						u64 write_dests_in_use;
						u64 write_misses;
						u64 time;
						u64 prev_total;
						u64 prev_time;
						struct rcu_random_state *rand;
				};
				char cache_alignment[128];
		};
} ____cacheline_aligned;

struct rcuhashbash_ops {
		void (*init_bucket)(struct rcuhashbash_bucket *);
		int (*read)(u32 value, struct stats *stats);
		void (*read_lock_bucket)(struct rcuhashbash_bucket *);
		void (*read_unlock_bucket)(struct rcuhashbash_bucket *);
		int (*write)(u32 src_value, u32 dst_value, struct stats *stats);
		void (*write_lock_buckets)(struct rcuhashbash_bucket *, struct rcuhashbash_bucket *);
		void (*write_unlock_buckets)(struct rcuhashbash_bucket *, struct rcuhashbash_bucket *);
		bool limit_writers;
		int max_writers;
		const char *reader_type;
		const char *writer_type;
};

struct rcuhashbash_entry {
        union {
            struct {
		        struct hlist_node node;
		        struct rcu_head rcu_head;
		        u64 value;
            };
            char cache_alignment[64];
        };
} ____cacheline_aligned;

static struct rcuhashbash_ops *ops;

DECLARE_TABLE_LOCK(table_spinlock, DEFINE_SPINLOCK, spin_lock, spin_unlock, spin_lock, spin_unlock);

DECLARE_TABLE_LOCK(table_bpf_qspinlock, DEFINE_KOMBSPINLOCK, __internal__bpf_spin_lock, __internal__bpf_spin_unlock, __internal__bpf_spin_lock, __internal__bpf_spin_unlock);

DECLARE_TABLE_LOCK(table_komb, DEFINE_KOMBSPINLOCK, komb_spin_lock, komb_spin_unlock,
				   komb_spin_lock, komb_spin_unlock);

DECLARE_TABLE_LOCK(table_rwlock, DEFINE_RWLOCK, write_lock, write_unlock, read_lock, read_unlock);

DECLARE_TABLE_LOCK(table_mutex, DEFINE_MUTEX, mutex_lock, mutex_unlock, mutex_lock, mutex_unlock);

DECLARE_TABLE_LOCK(table_rwsem, DECLARE_RWSEM, down_write, up_write, down_read, up_read);

DECLARE_TABLE_LOCK(table_aqs, DEFINE_AQSLOCK, __aqs_acquire, __aqs_release, __aqs_acquire,
				   __aqs_release);

DECLARE_TABLE_LOCK(table_cna, DEFINE_CNALOCK, cna_spin_lock, cna_spin_unlock, cna_spin_lock,
				   cna_spin_unlock);

static struct rcuhashbash_bucket *hash_table;

static struct kmem_cache *entry_cache;

static struct task_struct **tasks;

static struct stats *thread_stats;

/*
 * Crude but fast random-number generator.  Uses a linear congruential
 * generator, with occasional help from cpu_clock().
 */
unsigned long rcu_random(struct rcu_random_state *rrsp)
{
		if (--rrsp->rrs_count < 0) {
				rrsp->rrs_state += (unsigned long)cpu_clock(raw_smp_processor_id());
				rrsp->rrs_count = RCU_RANDOM_REFRESH;
		}
		rrsp->rrs_state = rrsp->rrs_state * RCU_RANDOM_MULT + RCU_RANDOM_ADD;
		return swahw32(rrsp->rrs_state);
}

noinline void init_ht(void)
{
    printk(KERN_ALERT "init_ht called\n");
}
EXPORT_SYMBOL(init_ht);

noinline int attach_cs_ht(u32 src_value, u32 dst_value, struct stats *stats)
{
	stats->write_moves++;
	return 0;
}
EXPORT_SYMBOL(attach_cs_ht);

static int rcuhashbash_read_lock(u32 value, struct stats *stats)
{
		struct rcuhashbash_entry *entry;
		bool node_present = false;
		u32 bucket;

		bucket = value % buckets;

		ops->read_lock_bucket(&hash_table[bucket]);

		hlist_for_each_entry (entry, &hash_table[value % buckets].head, node) {
				if (entry->value == value) {
						node_present = true;
						break;
				}
		}

		ops->read_unlock_bucket(&hash_table[bucket]);

		if (node_present)
				stats->read_hits++;
		else
				stats->read_misses++;

		return 0;
}

#if USE_UNDO_LOG
DEFINE_PER_CPU(u64*, undo_log);
#endif

#if USE_UNDO_LOG
#if USE_UNDO_LOG_STORE
static inline void bring_modified_and_store(struct rcuhashbash_entry *entry, int i)
{
    typeof((*this_cpu_ptr(&undo_log))[i]) *dst;
    typeof(entry->value) val;

    /* Resolve per-CPU destination outside the asm block */
    dst = &(*this_cpu_ptr(&undo_log))[i];

    asm volatile(
        /* Instruction 1:
         * Add 0 directly to entry->value in MEMORY.
         * This is a read-modify-write on entry->value's cache line,
         * forcing it into MESI Modified state on this CPU's cache. */
        "addq $0, %[src]\n\t"

        /* Instruction 2:
         * Load the (now Modified) entry->value from memory into a register. */
        "movq %[src], %[tmp]\n\t"

        /* Instruction 3:
         * Store the value to the per-CPU target location. */
        "movq %[tmp], %[dst]\n\t"

        : [src] "+m" (entry->value),   /* read-write memory: entry->value (modified in-place) */
          [dst] "=m" (*dst),           /* write-only memory: per-CPU target location */
          [tmp] "=&r" (val)            /* early-clobber register: scratch for the transfer */
        :                              /* no pure inputs; [src] already a "+m" read-write */
        : "cc"                         /* addq modifies FLAGS */
    );
}
#endif
#endif

static int rcuhashbash_write_lock(u32 src_value, u32 dst_value, struct stats *stats)
{
		u32 src_bucket;
		u32 dst_bucket;
		struct rcuhashbash_entry *entry = NULL;
		int i = 0;

		src_bucket = src_value % buckets;
		dst_bucket = dst_value % buckets;

		ops->write_lock_buckets(&hash_table[src_bucket], &hash_table[dst_bucket]);

		hlist_for_each_entry (entry, &hash_table[src_bucket].head, node) {
#if USE_UNDO_LOG
#if USE_UNDO_LOG_STORE
                bring_modified_and_store(entry, i);
#elif USE_UNDO_LOG_PREFETCH
                prefetchw(&(entry->value));
				(*this_cpu_ptr(&undo_log))[i] = entry->value;
#elif USE_UNDO_LOG_ATOMIC
				(*this_cpu_ptr(&undo_log))[i] = __sync_fetch_and_add(&(entry->value), 0);
#else
				(*this_cpu_ptr(&undo_log))[i] = entry->value;
#endif
#endif
				entry->value = dst_value + i;
				i++;
		}

		ops->write_unlock_buckets(&hash_table[src_bucket], &hash_table[dst_bucket]);
		stats->write_moves++;

		return 0;
}

static int rcuhashbash_ro_thread(void *arg)
{
		int err;
		struct stats *stats_ret = arg;
		struct stats stats = {};
		DEFINE_RCU_RANDOM(rand);

		set_user_nice(current, 19);

		do {
				cond_resched();
				err = ops->read(rcu_random(&rand) % reader_range, &stats);
		} while (!kthread_should_stop() && !err);

		*stats_ret = stats;

		__set_current_state(TASK_RUNNING);
		while (!kthread_should_stop())
				schedule_timeout_interruptible(1);
		return err;
}

static int rcuhashbash_rw_thread(void *arg)
{
		int err;
		struct stats *stats_ret = arg;
		struct timespec64 start_t, end_t;
		DEFINE_RCU_RANDOM(rand);

		void *ptr = vzalloc(SIZE_OF_SHADOW_STACK);

		stats_ret->rand = &rand;
		BUG_ON(ptr == NULL);

		ktime_get_raw_ts64(&start_t);
		do {
				if (need_resched())
						cond_resched();

				if ((rcu_random(&rand) % rw_total) < rw_writes)
						err = ops->write(rcu_random(&rand) % writer_range,
										 rcu_random(&rand) % writer_range, stats_ret);
				else
						err = ops->read(rcu_random(&rand) % reader_range, stats_ret);

		} while (!kthread_should_stop() && !err);
		ktime_get_raw_ts64(&end_t);

		end_t = timespec64_sub(end_t, start_t);

		cond_resched();

		stats_ret->time = timespec64_to_ns(&end_t);

		__set_current_state(TASK_RUNNING);
		while (!kthread_should_stop())
				schedule_timeout_interruptible(1);
		return err;
}

static void spinlock_init_bucket(struct rcuhashbash_bucket *bucket)
{
		spin_lock_init(&bucket->spinlock);
}

static void rwlock_init_bucket(struct rcuhashbash_bucket *bucket)
{
		rwlock_init(&bucket->rwlock);
}

static void mutex_init_bucket(struct rcuhashbash_bucket *bucket)
{
		mutex_init(&bucket->mutex);
}

static void rwsem_init_bucket(struct rcuhashbash_bucket *bucket)
{
		init_rwsem(&bucket->rwsem);
}

static void percpu_rwsem_init_bucket(struct rcuhashbash_bucket *bucket)
{
		percpu_init_rwsem(&bucket->percpu_rwsem);
}

static void komb_spin_lock_init_bucket(struct rcuhashbash_bucket *bucket)
{
		komb_spin_lock_init(&bucket->komb);
}

static void spinlock_read_lock_bucket(struct rcuhashbash_bucket *bucket)
{
		spin_lock(&bucket->spinlock);
}

static void rwlock_read_lock_bucket(struct rcuhashbash_bucket *bucket)
{
		read_lock(&bucket->rwlock);
}

static void mutex_read_lock_bucket(struct rcuhashbash_bucket *bucket)
{
		mutex_lock(&bucket->mutex);
}

static void rwsem_read_lock_bucket(struct rcuhashbash_bucket *bucket)
{
		down_read(&bucket->rwsem);
}

static void percpu_rwsem_read_lock_bucket(struct rcuhashbash_bucket *bucket)
{
		percpu_down_read(&bucket->percpu_rwsem);
}

static void komb_spin_lock_read_lock_bucket(struct rcuhashbash_bucket *bucket)
{
		komb_spin_lock(&bucket->komb);
}

static void spinlock_read_unlock_bucket(struct rcuhashbash_bucket *bucket)
{
		spin_unlock(&bucket->spinlock);
}

static void rwlock_read_unlock_bucket(struct rcuhashbash_bucket *bucket)
{
		read_unlock(&bucket->rwlock);
}

static void mutex_read_unlock_bucket(struct rcuhashbash_bucket *bucket)
{
		mutex_unlock(&bucket->mutex);
}

static void rwsem_read_unlock_bucket(struct rcuhashbash_bucket *bucket)
{
		up_read(&bucket->rwsem);
}

static void percpu_rwsem_read_unlock_bucket(struct rcuhashbash_bucket *bucket)
{
		percpu_up_read(&bucket->percpu_rwsem);
}

static void komb_spin_lock_read_unlock_bucket(struct rcuhashbash_bucket *bucket)
{
		komb_spin_unlock(&bucket->komb);
}

static void spinlock_write_lock_buckets(struct rcuhashbash_bucket *b1,
										struct rcuhashbash_bucket *b2)
{
		if (b1 == b2)
				spin_lock(&b1->spinlock);
		else if (b1 < b2) {
				spin_lock(&b1->spinlock);
				spin_lock_nested(&b2->spinlock, SINGLE_DEPTH_NESTING);
		} else {
				spin_lock(&b2->spinlock);
				spin_lock_nested(&b1->spinlock, SINGLE_DEPTH_NESTING);
		}
}

static void rwlock_write_lock_buckets(struct rcuhashbash_bucket *b1, struct rcuhashbash_bucket *b2)
{
		if (b1 == b2)
				write_lock(&b1->rwlock);
		else if (b1 < b2) {
				write_lock(&b1->rwlock);
				write_lock(&b2->rwlock);
		} else {
				write_lock(&b2->rwlock);
				write_lock(&b1->rwlock);
		}
}

static void mutex_write_lock_buckets(struct rcuhashbash_bucket *b1, struct rcuhashbash_bucket *b2)
{
		if (b1 == b2)
				mutex_lock(&b1->mutex);
		else if (b1 < b2) {
				mutex_lock(&b1->mutex);
				mutex_lock_nested(&b2->mutex, SINGLE_DEPTH_NESTING);
		} else {
				mutex_lock(&b2->mutex);
				mutex_lock_nested(&b1->mutex, SINGLE_DEPTH_NESTING);
		}
}

static void rwsem_write_lock_buckets(struct rcuhashbash_bucket *b1, struct rcuhashbash_bucket *b2)
{
		if (b1 == b2)
				down_write(&b1->rwsem);
		else if (b1 < b2) {
				down_write(&b1->rwsem);
				down_write(&b2->rwsem);
		} else {
				down_write(&b2->rwsem);
				down_write(&b1->rwsem);
		}
}

static void percpu_rwsem_write_lock_buckets(struct rcuhashbash_bucket *b1,
											struct rcuhashbash_bucket *b2)
{
		if (b1 == b2)
				percpu_down_write(&b1->percpu_rwsem);
		else if (b1 < b2) {
				percpu_down_write(&b1->percpu_rwsem);
				percpu_down_write(&b2->percpu_rwsem);
		} else {
				percpu_down_write(&b2->percpu_rwsem);
				percpu_down_write(&b1->percpu_rwsem);
		}
}

static void komb_spin_lock_write_lock_buckets(struct rcuhashbash_bucket *b1,
											  struct rcuhashbash_bucket *b2)
{
		if (b1 == b2)
				komb_spin_lock(&b1->komb);
		else if (b1 < b2) {
				komb_spin_lock(&b1->komb);
				komb_spin_lock(&b2->komb);
		} else {
				komb_spin_lock(&b2->komb);
				komb_spin_lock(&b1->komb);
		}
}

static void spinlock_write_unlock_buckets(struct rcuhashbash_bucket *b1,
										  struct rcuhashbash_bucket *b2)
{
		spin_unlock(&b1->spinlock);
		if (b1 != b2)
				spin_unlock(&b2->spinlock);
}

static void rwlock_write_unlock_buckets(struct rcuhashbash_bucket *b1,
										struct rcuhashbash_bucket *b2)
{
		write_unlock(&b1->rwlock);
		if (b1 != b2)
				write_unlock(&b2->rwlock);
}

static void mutex_write_unlock_buckets(struct rcuhashbash_bucket *b1, struct rcuhashbash_bucket *b2)
{
		mutex_unlock(&b1->mutex);
		if (b1 != b2)
				mutex_unlock(&b2->mutex);
}

static void rwsem_write_unlock_buckets(struct rcuhashbash_bucket *b1, struct rcuhashbash_bucket *b2)
{
		up_write(&b1->rwsem);
		if (b1 != b2)
				up_write(&b2->rwsem);
}

static void percpu_rwsem_write_unlock_buckets(struct rcuhashbash_bucket *b1,
											  struct rcuhashbash_bucket *b2)
{
		percpu_up_write(&b1->percpu_rwsem);
		if (b1 != b2)
				percpu_up_write(&b2->percpu_rwsem);
}

static void komb_spin_lock_write_unlock_buckets(struct rcuhashbash_bucket *b1,
												struct rcuhashbash_bucket *b2)
{
		komb_spin_unlock(&b1->komb);
		if (b1 != b2)
				komb_spin_unlock(&b2->komb);
}

static struct rcuhashbash_ops all_ops[] = {
		{
				.reader_type = "komb",
				.writer_type = "komb",
				.init_bucket = komb_spin_lock_init_bucket,
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = komb_spin_lock_read_lock_bucket,
				.read_unlock_bucket = komb_spin_lock_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = komb_spin_lock_write_lock_buckets,
				.write_unlock_buckets = komb_spin_lock_write_unlock_buckets,
		},
		{
				.reader_type = "spinlock",
				.writer_type = "spinlock",
				.init_bucket = spinlock_init_bucket,
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = spinlock_read_lock_bucket,
				.read_unlock_bucket = spinlock_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = spinlock_write_lock_buckets,
				.write_unlock_buckets = spinlock_write_unlock_buckets,
		},
		{
				.reader_type = "rwlock",
				.writer_type = "rwlock",
				.init_bucket = rwlock_init_bucket,
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = rwlock_read_lock_bucket,
				.read_unlock_bucket = rwlock_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = rwlock_write_lock_buckets,
				.write_unlock_buckets = rwlock_write_unlock_buckets,
		},
		{
				.reader_type = "mutex",
				.writer_type = "mutex",
				.init_bucket = mutex_init_bucket,
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = mutex_read_lock_bucket,
				.read_unlock_bucket = mutex_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = mutex_write_lock_buckets,
				.write_unlock_buckets = mutex_write_unlock_buckets,
		},
		{
				.reader_type = "rwsem",
				.writer_type = "rwsem",
				.init_bucket = rwsem_init_bucket,
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = rwsem_read_lock_bucket,
				.read_unlock_bucket = rwsem_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = rwsem_write_lock_buckets,
				.write_unlock_buckets = rwsem_write_unlock_buckets,
		},
		{
				.reader_type = "percpu_rwsem",
				.writer_type = "percpu_rwsem",
				.init_bucket = percpu_rwsem_init_bucket,
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = percpu_rwsem_read_lock_bucket,
				.read_unlock_bucket = percpu_rwsem_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = percpu_rwsem_write_lock_buckets,
				.write_unlock_buckets = percpu_rwsem_write_unlock_buckets,
		},
		{
				.reader_type = "table_spinlock",
				.writer_type = "table_spinlock",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_spinlock_read_lock_bucket,
				.read_unlock_bucket = table_spinlock_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_spinlock_write_lock_buckets,
				.write_unlock_buckets = table_spinlock_write_unlock_buckets,
		},
		{
				.reader_type = "table_bpf_spinlock_undolog",
				.writer_type = "table_bpf_spinlock_undolog",
				.read = NULL,
				.read_lock_bucket = NULL,
				.read_unlock_bucket = NULL,
				.write = attach_cs_ht,
				.write_lock_buckets = NULL,
				.write_unlock_buckets = NULL,
		},
		{
				.reader_type = "table_komb",
				.writer_type = "table_komb",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_komb_read_lock_bucket,
				.read_unlock_bucket = table_komb_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_komb_write_lock_buckets,
				.write_unlock_buckets = table_komb_write_unlock_buckets,
		},
		{
				.reader_type = "table_bpf_qspinlock",
				.writer_type = "table_bpf_qspinlock",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_bpf_qspinlock_read_lock_bucket,
				.read_unlock_bucket = table_bpf_qspinlock_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_bpf_qspinlock_write_lock_buckets,
				.write_unlock_buckets = table_bpf_qspinlock_write_unlock_buckets,
		},
		{
				.reader_type = "table_aqs",
				.writer_type = "table_aqs",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_aqs_read_lock_bucket,
				.read_unlock_bucket = table_aqs_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_aqs_write_lock_buckets,
				.write_unlock_buckets = table_aqs_write_unlock_buckets,
		},
		{
				.reader_type = "table_cna",
				.writer_type = "table_cna",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_cna_read_lock_bucket,
				.read_unlock_bucket = table_cna_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_cna_write_lock_buckets,
				.write_unlock_buckets = table_cna_write_unlock_buckets,
		},
		{
				.reader_type = "table_rwlock",
				.writer_type = "table_rwlock",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_rwlock_read_lock_bucket,
				.read_unlock_bucket = table_rwlock_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_rwlock_write_lock_buckets,
				.write_unlock_buckets = table_rwlock_write_unlock_buckets,
		},
		{
				.reader_type = "table_mutex",
				.writer_type = "table_mutex",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_mutex_read_lock_bucket,
				.read_unlock_bucket = table_mutex_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_mutex_write_lock_buckets,
				.write_unlock_buckets = table_mutex_write_unlock_buckets,
		},
		{
				.reader_type = "table_rwsem",
				.writer_type = "table_rwsem",
				.read = rcuhashbash_read_lock,
				.read_lock_bucket = table_rwsem_read_lock_bucket,
				.read_unlock_bucket = table_rwsem_read_unlock_bucket,
				.write = rcuhashbash_write_lock,
				.write_lock_buckets = table_rwsem_write_lock_buckets,
				.write_unlock_buckets = table_rwsem_write_unlock_buckets,
		},
};

static struct rcuhashbash_ops *ops;

static int cmp_tput(const void *ja, const void *jb)
{
		const struct stats *a, *b;
		u64 ta, tb;
		a = ja;
		b = jb;

		ta = a->read_hits + a->read_misses + a->read_misses + a->write_moves +
			 a->write_dests_in_use + a->write_misses;
		tb = b->read_hits + b->read_misses + b->read_misses + b->write_moves +
			 b->write_dests_in_use + b->write_misses;
		return (ta == tb) ? 0 : (ta > tb) ? 1 : -1;
}

static void rcuhashbash_print_stats(void)
{
		int i;
		struct stats s = {};
		u64 min_time = thread_stats[0].time;
		u64 max_time = thread_stats[0].time;
		u64 up_sum = 0, down_sum = 0;

		if (!thread_stats) {
				printk(KERN_ALERT "rcuhashbash stats unavailable\n");
				return;
		}

		for (i = 0; i < ro + rw; i++) {
				s.time += thread_stats[i].time;
				if (thread_stats[i].time > max_time)
						max_time = thread_stats[i].time;
				if (thread_stats[i].time < min_time)
						min_time = thread_stats[i].time;
				s.read_hits += thread_stats[i].read_hits;
				s.read_misses += thread_stats[i].read_misses;
				s.write_moves += thread_stats[i].write_moves;
				s.write_dests_in_use += thread_stats[i].write_dests_in_use;
				s.write_misses += thread_stats[i].write_misses;
		}

		if (ro + rw > 1) {
				sort(thread_stats, ro + rw, sizeof(thread_stats[0]), cmp_tput, NULL);
				for (i = 0; i < ro + rw; ++i) {
						if (i < (ro + rw) / 2)
								up_sum += thread_stats[i].read_hits + thread_stats[i].read_misses +
										  thread_stats[i].write_moves +
										  thread_stats[i].write_misses +
										  thread_stats[i].write_dests_in_use;
						else
								down_sum +=
										thread_stats[i].read_hits + thread_stats[i].read_misses +
										thread_stats[i].write_moves + thread_stats[i].write_misses +
										thread_stats[i].write_dests_in_use;
				}
		}

		printk(KERN_ALERT
			   "rcuhashbash summary: (up: %llu down: %llu)\n", up_sum, down_sum);
		printk(KERN_ALERT
			   "rcuhashbash summary: ro=%d rw=%d reader_type=%s writer_type=%s\n"
			   "rcuhashbash summary: writer proportion %lu/%lu\n"
			   "rcuhashbash summary: buckets=%lu entries=%lu reader_range=%lu writer_range=%lu\n"
			   "rcuhashbash summary: writes: %llu moves, %llu dests in use, %llu misses (%llu)\n"
			   "rcuhashbash summary: reads: %llu hits, %llu misses (%llu)\n"
			   "rcuhashbash summary: total: %llu (avg: %llu min: %llu max: %llu)\n",
			   ro, rw, reader_type, writer_type, rw_writes, rw_total, buckets, entries,
			   reader_range, writer_range, s.write_moves, s.write_dests_in_use, s.write_misses,
			   s.write_moves + s.write_dests_in_use + s.write_misses, s.read_hits, s.read_misses,
			   s.read_hits + s.read_misses,
			   s.write_moves + s.write_dests_in_use + s.write_misses + s.read_hits + s.read_misses,
			   s.time / (ro + rw), min_time, max_time);
}

// ----------- IOCTL DEF BEGIN -------------

#define DEVICE_NAME "bpf_rcuht_sync"
#define CLASS_NAME "bpf_rcuht_sync_class"
#define IOCTL_MAGIC 'B'

#define BPF_READY _IO(IOCTL_MAGIC, 1)
#define BPF_EXIT _IO(IOCTL_MAGIC, 2)

static int major_num;
static struct class* bpf_class = NULL;
static struct device* bpf_dev = NULL;
static struct cdev bpf_cdev;

static bool bpf_ready = false;        // Flag for other module code
static bool finish_benchmark = false;
static DECLARE_WAIT_QUEUE_HEAD(bpf_wq);  // Waitqueue for kthread + others

static struct task_struct *waiter_kthread;

// Other module threads can wait like this:
// wait_event(bpf_wq, bpf_ready);

static long bpf_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    switch (cmd) {
    case BPF_READY:
        pr_info("BPF_IOCTL: BPF program attached, signaling ready!\n");
        bpf_ready = true;
        wake_up(&bpf_wq);  // Wake kthread + anyone else
        return 0;
    case BPF_EXIT:
         pr_info("BPF_IOCTL: Remove BPF program\n");
         finish_benchmark = true;
         wake_up(&bpf_wq);
         return 0; 
    default:
        return -ENOTTY;
    }
}

static struct file_operations fops = {
    .unlocked_ioctl = bpf_ioctl,
};

// ----------- IOCTL DEF END -------------

static void rcuhashbash_end_benchmark(void)
{
		unsigned long i;
		int ret;

		printk(KERN_ALERT "rcuhashbash exiting threads\n");

		if (tasks) {
				for (i = 0; i < ro + rw; i++)
						if (tasks[i]) {
								ret = kthread_stop(tasks[i]);
								if (ret)
										printk(KERN_ALERT "rcuhashbash task returned error %d\n",
											   ret);
						}
				kfree(tasks);
		}

		/* Wait for all RCU callbacks to complete. */
		rcu_barrier();

		if (hash_table) {
				for (i = 0; i < buckets; i++) {
						struct hlist_head *head = &hash_table[i].head;
						while (!hlist_empty(head)) {
								struct rcuhashbash_entry *entry;
								entry = hlist_entry(head->first, struct rcuhashbash_entry, node);
								hlist_del(head->first);
								kmem_cache_free(entry_cache, entry);
						}
				}
				vfree(hash_table);
		}

		if (entry_cache)
				kmem_cache_destroy(entry_cache);

#if KOMB_STATS
		komb_print_stats();
#endif

		if (thread_stats)
				rcuhashbash_print_stats();

		kfree(thread_stats);
		komb_free();
#if USE_UNDO_LOG
        for_each_possible_cpu(i) {
            vfree(*per_cpu_ptr(&undo_log, i));
        }
#endif
}

static void rcuhashbash_exit(void)
{
        // ----------- IOCTL DEF BEGIN -------------
        pr_info("Removing IOCTL chardev\n");
        if (waiter_kthread) {
            kthread_stop(waiter_kthread);
        }
        device_destroy(bpf_class, MKDEV(major_num, 0));
        class_destroy(bpf_class);
        unregister_chrdev(major_num, DEVICE_NAME);
        // ----------- IOCTL DEF END -------------
		printk(KERN_ALERT "rcuhashbash done\n");
}

static void rcuhashbash_start_benchmark(void)
{
    u32 i, new_cpuid; 

    printk(KERN_ALERT "rcuhashbash starting threads\n");

    new_cpuid = 0;
    for (i = 0; i < ro + rw; i++) {
            struct task_struct *task;

            if (i < ro)
                    task = kthread_create(rcuhashbash_ro_thread, &thread_stats[i],
                                          "rcuhashbash_ro");
            else
                    task = kthread_create(rcuhashbash_rw_thread, &thread_stats[i],
                                          "rcuhashbash_rw");
            if (IS_ERR(task)) {
                    printk(KERN_ALERT "error creating thread\n");
            }

            tasks[i] = task;

            kthread_bind(tasks[i], cpuseq[new_cpuid]);
            new_cpuid += 1;
            wake_up_process(tasks[i]);
    }
}

// Kthread that blocks until ioctl arrives
static int waiter_thread(void *data) {
    pr_info("BPF waiter kthread: waiting for BPF attach signal...\n");
    
    // Block indefinitely until woken
    wait_event(bpf_wq, bpf_ready);
    init_ht();
    rcuhashbash_start_benchmark();
   
    wait_event(bpf_wq, finish_benchmark);
    rcuhashbash_end_benchmark(); 
 
    pr_info("BPF waiter kthread exiting: BPF ready! Flag set.\n");
    waiter_kthread = NULL;
    return 0;
}

static __init int rcuhashbash_init(void)
{
		int ret;
		u32 i;

		for (i = 0; i < ARRAY_SIZE(all_ops); i++)
				if (strcmp(reader_type, all_ops[i].reader_type) == 0 &&
					strcmp(writer_type, all_ops[i].writer_type) == 0) {
						ops = &all_ops[i];
				}
		if (!ops) {
				printk(KERN_ALERT "rcuhashbash: No implementation with %s reader and %s writer\n",
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

		/*if (!ops->read) {
				printk(KERN_ALERT "rcuhashbash: Internal error: read function NULL\n");
				return -EINVAL;
		}*/
		if (rw > 0 && !ops->write) {
				printk(KERN_ALERT "rcuhashbash: Internal error: rw > 0 but write function NULL\n");
				return -EINVAL;
		}

		if (reader_range == 0)
				reader_range = 2 * entries;
		if (writer_range == 0)
				writer_range = 2 * entries;

		entry_cache = KMEM_CACHE(rcuhashbash_entry, 0);
		if (!entry_cache)
				goto enomem;

		hash_table = vzalloc(buckets * sizeof(hash_table[0]));
		if (!hash_table)
				goto enomem;

		if (ops->init_bucket)
				for (i = 0; i < buckets; i++)
						ops->init_bucket(&hash_table[i]);

		komb_init(); //Add call to initialize per-cpu variables for Komb.

		for (i = 0; i < entries; i++) {
				struct rcuhashbash_entry *entry;
				entry = kmem_cache_zalloc(entry_cache, GFP_KERNEL);
				if (!entry)
						goto enomem;
				entry->value = i;
				hlist_add_head(&entry->node, &hash_table[entry->value % buckets].head);
		}

#if USE_UNDO_LOG
        for_each_possible_cpu(i) {
            *per_cpu_ptr(&undo_log, i) = vzalloc(sizeof(u32) * (entries / buckets)); 
        }
#endif

		thread_stats = kcalloc(rw + ro, sizeof(thread_stats[0]), GFP_KERNEL);
		if (!thread_stats)
				goto enomem;

		tasks = kcalloc(rw + ro, sizeof(tasks[0]), GFP_KERNEL);
		if (!tasks)
				goto enomem;

		BUG_ON((ro + rw) > num_online_cpus());

        // ----------- IOCTL DEF BEGIN -------------
        
        // Register char device
        major_num = register_chrdev(0, DEVICE_NAME, &fops);
        if (major_num < 0)
            goto enochrdev;
        
        // Sysfs class/device
        bpf_class = class_create(CLASS_NAME);
        if (IS_ERR(bpf_class))
            goto enoclass;
        
        bpf_dev = device_create(bpf_class, NULL, MKDEV(major_num, 0), NULL, DEVICE_NAME);
        if (IS_ERR(bpf_dev))
            goto enodevice;
        
        bpf_ready = false;
        
        // Spawn background kthread
        waiter_kthread = kthread_run(waiter_thread, NULL, "bpf_waiter");
        if (IS_ERR(waiter_kthread))
            goto ewaiter_kthread;
        
        pr_info("BPF_SYNC: Module loaded, device /dev/%s created. Send BPF_READY ioctl to unblock.\n", DEVICE_NAME);
        // ----------- IOCTL DEF END -------------

		return 0;

ewaiter_kthread:
        pr_err("BPF_SYNC: Failed to create kthread\n");
        device_destroy(bpf_class, MKDEV(major_num, 0));
enodevice:
        pr_err("BPF_SYNC: Failed to create class\n");
        class_destroy(bpf_class);
enoclass:
        pr_err("BPF_SYNC: Failed to create device\n");
        unregister_chrdev(major_num, DEVICE_NAME);
enochrdev:
        pr_err("BPF_SYNC: Failed to register device\n");
enomem:
		komb_free();
		ret = -ENOMEM;
		return ret;
}

module_init(rcuhashbash_init);
module_exit(rcuhashbash_exit);

MODULE_AUTHOR("Vishal Gupta");
MODULE_DESCRIPTION("Simple stress testing for undo logging.");
MODULE_LICENSE("GPL");
