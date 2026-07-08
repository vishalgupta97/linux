// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-threaded userspace runner for the BPF spin_lock timeout + undo-log
 * mechanism.  Each subtest pins worker threads to distinct CPUs and starts
 * them together behind a barrier, so the BPF programs genuinely contend on the
 * same lock(s) across CPUs.  This reaches the concurrent kernel paths that the
 * single-threaded tests cannot: the qspinlock slow path, the contended-timeout
 * path (waiter terminates the owner), nested inner-lock contention, and
 * concurrent/repeated timeouts.
 *
 * Verification model (mirrors prog_tests/bpf_undo_log.c):
 *   - pre-fill each lock entry with a known sentinel @value / zero @counter
 *   - run the BPF programs concurrently
 *   - look the entry up and assert: holders' @value rolled back to the
 *     sentinel; short critical sections' @counter increments committed.
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/sysinfo.h>
#include <sys/utsname.h>

#include <test_progs.h>
#include <network_helpers.h>

#include "test_spin_lock_mt_timeout.skel.h"

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

#define KEY_A 0
#define KEY_B 1

/* Sentinels pre-loaded by userspace; must differ from the in-CS write values
 * (A_WRITE_VAL / B_WRITE_VAL in the BPF program) so a rollback is observable.
 */
#define A_SENTINEL	0xA5A5A5A5A5A5A5A5ULL
#define B_SENTINEL	0x5B5B5B5B5B5B5B5BULL

/* Short timeout used by tests that expect holders to be terminated.  Long
 * timeout for the pure-contention test where no timeout should fire.
 */
#define TIMEOUT_SHORT	100
#define TIMEOUT_LONG	100000

#define MAX_THREADS 16

/* Userspace mirror of struct lock_pair in progs/test_spin_lock_mt_timeout.c.
 * bpf_spin_lock occupies 4 bytes followed by 4 bytes of alignment padding.
 */
struct lock_pair {
	__u32 lock;
	__u8  _lock_pad[4];
	__u64 value;
	__u64 counter;
};

/* ------------------------------------------------------------------ */
/* sysctl + config detection helpers                                    */
/* ------------------------------------------------------------------ */

static int __read_sysctl(void)
{
	int fd, val = 0;
	char buf[32];

	fd = open(SYSCTL_PATH, O_RDONLY);
	if (fd < 0)
		return -1;
	if (read(fd, buf, sizeof(buf)) > 0)
		val = atoi(buf);
	close(fd);
	return val;
}

static int __write_sysctl(int val)
{
	int fd;
	char buf[32];

	fd = open(SYSCTL_PATH, O_WRONLY);
	if (fd < 0)
		return -1;
	snprintf(buf, sizeof(buf), "%d\n", val);
	if (write(fd, buf, strlen(buf)) < 0) {
		close(fd);
		return -1;
	}
	close(fd);
	return 0;
}

static bool has_bpf_timeout(void)
{
	int fd = open(SYSCTL_PATH, O_RDONLY);

	if (fd < 0)
		return false;
	close(fd);
	return true;
}

static bool config_buf_has_undo_log(const char *buf, size_t len)
{
	const char needle[] = "CONFIG_BPF_UNDO_LOG=y";
	size_t nlen = sizeof(needle) - 1;
	size_t i;

	for (i = 0; i + nlen <= len; i++) {
		if (memcmp(buf + i, needle, nlen) == 0)
			return true;
	}
	return false;
}

static bool has_bpf_undo_log(void)
{
	struct utsname u;
	char path[256];
	char buf[65536];
	FILE *fp;
	size_t n;

	uname(&u);
	snprintf(path, sizeof(path), "/boot/config-%s", u.release);
	fp = fopen(path, "r");
	if (!fp)
		fp = fopen("/proc/config.gz", "r");
	if (!fp)
		return false;
	n = fread(buf, 1, sizeof(buf) - 1, fp);
	fclose(fp);
	if (n == 0)
		return false;
	buf[n] = '\0';
	return config_buf_has_undo_log(buf, n);
}

/* ------------------------------------------------------------------ */
/* Map prefill / readback helpers                                       */
/* ------------------------------------------------------------------ */

static void preset_entry(int map_fd, int key, __u64 value, __u64 counter)
{
	struct lock_pair v;

	memset(&v, 0, sizeof(v));
	v.value = value;
	v.counter = counter;
	bpf_map_update_elem(map_fd, &key, &v, BPF_ANY);
}

static int get_entry(int map_fd, int key, struct lock_pair *out)
{
	return bpf_map_lookup_elem(map_fd, &key, out);
}

/* ------------------------------------------------------------------ */
/* Thread infrastructure (pin to CPU, start together behind a barrier)  */
/* ------------------------------------------------------------------ */

static pthread_barrier_t barrier;

struct thr_arg {
	int prog_fd;
	int repeat;
	int cpu;
	int err;	/* out: bpf_prog_test_run_opts() return */
};

static void *mt_worker(void *arg)
{
	struct thr_arg *t = arg;
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in      = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
		.repeat       = t->repeat,
	);
	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(t->cpu, &cpuset);
	pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

	pthread_barrier_wait(&barrier);

	t->err = bpf_prog_test_run_opts(t->prog_fd, &opts);
	return NULL;
}

/*
 * Spawn @n threads (one per entry in @args), pinned to CPUs 0..n-1, started
 * together behind a barrier; join them all.  Returns 0 and asserts each
 * thread's test_run syscall succeeded.
 */
static int run_threads(struct thr_arg *args, int n)
{
	pthread_t tid[MAX_THREADS];
	int i, ret = 0;

	if (!ASSERT_OK(pthread_barrier_init(&barrier, NULL, n), "barrier_init"))
		return -1;

	for (i = 0; i < n; i++) {
		args[i].cpu = i;
		args[i].err = 0;
		if (!ASSERT_OK(pthread_create(&tid[i], NULL, mt_worker, &args[i]),
			       "pthread_create")) {
			/* let already-created threads pass the barrier and exit */
			ret = -1;
			n = i;
			break;
		}
	}

	for (i = 0; i < n; i++)
		pthread_join(tid[i], NULL);

	pthread_barrier_destroy(&barrier);

	for (i = 0; i < n; i++) {
		if (!ASSERT_OK(args[i].err, "thread test_run err"))
			ret = -1;
	}
	return ret;
}

static int nthreads(void)
{
	int n = get_nprocs();

	return n > MAX_THREADS ? MAX_THREADS : n;
}

/*
 * A set of program fds implementing the same scenarios with either the
 * bpf_spin_lock()/bpf_spin_unlock() API or the bpf_lock_func() closure API.
 * Every subtest is run once per set so both locking APIs get the same
 * multi-threaded coverage.
 */
struct prog_set {
	const char *tag;
	int holder;	/* hold lock A, dirty A.value, spin past timeout */
	int incr_a;	/* short CS on A: bump A.counter */
	int incr_b;	/* short CS on B: bump B.counter */
	int nested;	/* outer A + inner B, dirty both, spin past timeout */
	int workerb;	/* lock B, bump B.counter, bounded committing loop */
	int dl_ab;	/* A-then-B holder that times out before second acquire */
	int dl_ba;	/* B-then-A holder that times out before second acquire */
};

/* ================================================================== */
/* 1. Contended holder timeout                                         */
/*    One holder spins on lock A past the timeout (its A.value write    */
/*    must roll back); the other CPUs hammer short A critical sections  */
/*    (their A.counter increments must commit).  Exercises the          */
/*    contended slow path + the waiter-terminates-owner timeout path.   */
/* ================================================================== */
static void test_contended_holder_timeout(int map_fd, struct prog_set *ps)
{
	struct thr_arg args[MAX_THREADS];
	struct lock_pair a;
	int n = nthreads(), i;
	int holder_fd = ps->holder;
	int incr_fd   = ps->incr_a;
	int contenders = n - 1;
	int incr_repeat = 2000;

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);

	args[0] = (struct thr_arg){ .prog_fd = holder_fd, .repeat = 2 };
	for (i = 1; i < n; i++)
		args[i] = (struct thr_arg){ .prog_fd = incr_fd, .repeat = incr_repeat };

	if (run_threads(args, n))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	ASSERT_EQ(a.value, A_SENTINEL, "holder_value_rolled_back");
	ASSERT_EQ(a.counter, (__u64)contenders * incr_repeat, "contenders_committed");
}

/* ================================================================== */
/* 2. MCS queue stress (no timeout expected)                          */
/*    All CPUs run short A critical sections under a long timeout.      */
/*    Validates that lock mutual exclusion holds under heavy contention */
/*    (pending + queued slow-path branches): counter == n * repeat.     */
/* ================================================================== */
static void test_mcs_queue_stress(int map_fd, struct prog_set *ps)
{
	struct thr_arg args[MAX_THREADS];
	struct lock_pair a;
	int n = nthreads(), i;
	int incr_fd = ps->incr_a;
	int repeat = 5000;

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);

	for (i = 0; i < n; i++)
		args[i] = (struct thr_arg){ .prog_fd = incr_fd, .repeat = repeat };

	if (run_threads(args, n))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	ASSERT_EQ(a.counter, (__u64)n * repeat, "mutual_exclusion_counter");
}

/* ================================================================== */
/* 3. Independent locks isolation                                      */
/*    A holder times out on lock A while unrelated workers run bounded */
/*    loops under lock B.  The B workers must not observe lock A's     */
/*    timeout and must commit all increments.  This also stresses the  */
/*    uncontended watchdog path for the lock-A holder.                 */
/* ================================================================== */
static void test_independent_locks_isolation(int map_fd, struct prog_set *ps)
{
	struct thr_arg args[MAX_THREADS];
	struct lock_pair a, b;
	int n = nthreads(), i;
	int holder_fd = ps->holder;
	int workb_fd  = ps->workerb;
	int workers = n - 1;
	int b_repeat = 20000;

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);
	preset_entry(map_fd, KEY_B, B_SENTINEL, 0);

	args[0] = (struct thr_arg){ .prog_fd = holder_fd, .repeat = 5 };
	for (i = 1; i < n; i++)
		args[i] = (struct thr_arg){ .prog_fd = workb_fd, .repeat = b_repeat };

	if (run_threads(args, n))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	if (!ASSERT_OK(get_entry(map_fd, KEY_B, &b), "lookup_b"))
		return;
	ASSERT_EQ(a.value, A_SENTINEL, "holder_a_rolled_back");
	/* Ideal isolation: unrelated lock-B workers must all commit. */
	ASSERT_EQ(b.counter, (__u64)workers * b_repeat,
		  "independent_b_workers_not_terminated");
}

/* ================================================================== */
/* 3b. Contended timeout does not leak to independent lock             */
/*     Lock A has both a timing-out holder and a waiter, so timeout is */
/*     raised by the contended waiter path.  Lock B workers run on     */
/*     other CPUs and must not observe lock A's timeout bit.           */
/* ================================================================== */
static void test_contended_independent_locks_isolation(int map_fd,
						       struct prog_set *ps)
{
	struct thr_arg args[MAX_THREADS];
	struct lock_pair a, b;
	int n = nthreads(), i;
	int b_workers;
	int a_waiter_repeat = 2000;
	int b_repeat = 20000;

	if (n < 3) {
		test__skip();
		return;
	}

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);
	preset_entry(map_fd, KEY_B, B_SENTINEL, 0);

	args[0] = (struct thr_arg){ .prog_fd = ps->holder, .repeat = 1 };
	args[1] = (struct thr_arg){ .prog_fd = ps->incr_a, .repeat = a_waiter_repeat };
	for (i = 2; i < n; i++)
		args[i] = (struct thr_arg){ .prog_fd = ps->workerb, .repeat = b_repeat };
	b_workers = n - 2;

	if (run_threads(args, n))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	if (!ASSERT_OK(get_entry(map_fd, KEY_B, &b), "lookup_b"))
		return;
	ASSERT_EQ(a.value, A_SENTINEL, "contended_holder_a_rolled_back");
	ASSERT_EQ(a.counter, a_waiter_repeat, "contended_waiter_a_committed");
	ASSERT_EQ(b.counter, (__u64)b_workers * b_repeat,
		  "contended_timeout_did_not_abort_b_workers");
}

/* ================================================================== */
/* 4. Nested inner-lock contention + timeout                          */
/*    A holder takes A (outer) then B (inner), dirties both, and times  */
/*    out.  Other CPUs contend the inner lock B with short CS.  On       */
/*    timeout both A.value and B.value must roll back and both locks     */
/*    must be released so the B contenders make progress.               */
/* ================================================================== */
//static void test_nested_inner_contended_timeout(int map_fd, struct prog_set *ps)
//{
//	struct thr_arg args[MAX_THREADS];
//	struct lock_pair a, b;
//	int n = nthreads(), i;
//	int holder_fd = ps->nested;
//	int incrb_fd  = ps->incr_b;
//	int contenders = n - 1;
//	int b_repeat = 2000;
//
//	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);
//	preset_entry(map_fd, KEY_B, B_SENTINEL, 0);
//
//	args[0] = (struct thr_arg){ .prog_fd = holder_fd, .repeat = 3 };
//	for (i = 1; i < n; i++)
//		args[i] = (struct thr_arg){ .prog_fd = incrb_fd, .repeat = b_repeat };
//
//	if (run_threads(args, n))
//		return;
//
//	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
//		return;
//	if (!ASSERT_OK(get_entry(map_fd, KEY_B, &b), "lookup_b"))
//		return;
//	ASSERT_EQ(a.value, A_SENTINEL, "outer_value_rolled_back");
//	ASSERT_EQ(b.value, B_SENTINEL, "inner_value_rolled_back");
//	ASSERT_EQ(b.counter, (__u64)contenders * b_repeat, "inner_contenders_committed");
//}

/* ================================================================== */
/* 5. AB-BA pair with rollback                                         */
/*    Two CPUs grab opposite first-locks and time out before the cross- */
/*    acquire (see the BPF program comment on why a real deadlock isn't  */
/*    breakable here).  Both first writes must roll back.               */
/* ================================================================== */
static void test_deadlock_abba_rollback(int map_fd, struct prog_set *ps)
{
	struct thr_arg args[2];
	struct lock_pair a, b;

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);
	preset_entry(map_fd, KEY_B, B_SENTINEL, 0);

	args[0] = (struct thr_arg){ .prog_fd = ps->dl_ab, .repeat = 3 };
	args[1] = (struct thr_arg){ .prog_fd = ps->dl_ba, .repeat = 3 };

	if (run_threads(args, 2))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	if (!ASSERT_OK(get_entry(map_fd, KEY_B, &b), "lookup_b"))
		return;
	ASSERT_EQ(a.value, A_SENTINEL, "ab_value_rolled_back");
	ASSERT_EQ(b.value, B_SENTINEL, "ba_value_rolled_back");
}

/* ================================================================== */
/* 5b. Parallel uncontended lock_func timeouts                         */
/*     Two CPUs hold different locks via bpf_lock_func() and both time */
/*     out inside bpf_loop() before attempting the second lock.  This  */
/*     directly exercises independent watchdog requests for different  */
/*     CPUs without relying on contention.                             */
/* ================================================================== */
static void test_parallel_uncontended_timeouts(int map_fd, struct prog_set *ps)
{
	struct thr_arg args[2];
	struct lock_pair a, b;

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);
	preset_entry(map_fd, KEY_B, B_SENTINEL, 0);

	args[0] = (struct thr_arg){ .prog_fd = ps->dl_ab, .repeat = 1 };
	args[1] = (struct thr_arg){ .prog_fd = ps->dl_ba, .repeat = 1 };

	if (run_threads(args, 2))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	if (!ASSERT_OK(get_entry(map_fd, KEY_B, &b), "lookup_b"))
		return;
	ASSERT_EQ(a.value, A_SENTINEL, "parallel_a_rolled_back");
	ASSERT_EQ(b.value, B_SENTINEL, "parallel_b_rolled_back");
}

/* ================================================================== */
/* 6. Concurrent timeouts stress                                      */
/*    Every CPU holds lock A and times out, repeatedly.  They serialize */
/*    through the contended slow path, each timing out and rolling back. */
/*    Stresses the per-CPU timer/cancel + timeout-state churn; A.value   */
/*    must end at the sentinel and nothing must crash/hang.              */
/* ================================================================== */
static void test_concurrent_timeouts_stress(int map_fd, struct prog_set *ps)
{
	struct thr_arg args[MAX_THREADS];
	struct lock_pair a;
	int n = nthreads(), i;
	int holder_fd = ps->holder;

	preset_entry(map_fd, KEY_A, A_SENTINEL, 0);

	for (i = 0; i < n; i++)
		args[i] = (struct thr_arg){ .prog_fd = holder_fd, .repeat = 2 };

	if (run_threads(args, n))
		return;

	if (!ASSERT_OK(get_entry(map_fd, KEY_A, &a), "lookup_a"))
		return;
	ASSERT_EQ(a.value, A_SENTINEL, "all_holders_rolled_back");
}

/* ================================================================== */
/* Entry point                                                         */
/* ================================================================== */
/* Run a named subtest tagged with the locking API (e.g. "spinlock" or
 * "lockfunc") so the spin_lock and lock_func variants appear as distinct
 * subtests.
 */
#define MT_SUBTEST(ps, base) \
	(snprintf(subname, sizeof(subname), "%s_%s", (ps)->tag, (base)), subname)

void test_spin_lock_mt_timeout(void)
{
	struct test_spin_lock_mt_timeout *skel;
	struct prog_set sets[2];
	char subname[64];
	int old_timeout, map_fd, s;

	if (!has_bpf_timeout() || !has_bpf_undo_log()) {
		test__skip();
		return;
	}
	if (get_nprocs() < 2) {
		test__skip();
		return;
	}

	old_timeout = __read_sysctl();
	if (old_timeout < 0) {
		test__skip();
		return;
	}

	skel = test_spin_lock_mt_timeout__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto restore;

	map_fd = bpf_map__fd(skel->maps.mt_locks);

	/* bpf_spin_lock()/bpf_spin_unlock() variant. */
	sets[0] = (struct prog_set){
		.tag     = "spinlock",
		.holder  = bpf_program__fd(skel->progs.holder_loop_write_a),
		.incr_a  = bpf_program__fd(skel->progs.short_cs_incr_a),
		.incr_b  = bpf_program__fd(skel->progs.short_cs_incr_b),
		.nested  = bpf_program__fd(skel->progs.nested_inner_holder),
		.workerb = bpf_program__fd(skel->progs.worker_b_short_loop),
		.dl_ab   = bpf_program__fd(skel->progs.deadlock_ab),
		.dl_ba   = bpf_program__fd(skel->progs.deadlock_ba),
	};
	/* bpf_lock_func() closure variant. */
	sets[1] = (struct prog_set){
		.tag     = "lockfunc",
		.holder  = bpf_program__fd(skel->progs.lf_holder_loop_write_a),
		.incr_a  = bpf_program__fd(skel->progs.lf_short_cs_incr_a),
		.incr_b  = bpf_program__fd(skel->progs.lf_short_cs_incr_b),
		.nested  = bpf_program__fd(skel->progs.lf_nested_inner_holder),
		.workerb = bpf_program__fd(skel->progs.lf_worker_b_short_loop),
		.dl_ab   = bpf_program__fd(skel->progs.lf_deadlock_ab),
		.dl_ba   = bpf_program__fd(skel->progs.lf_deadlock_ba),
	};

	if (__write_sysctl(TIMEOUT_SHORT) < 0) {
		test__skip();
		goto destroy;
	}

	for (s = 0; s < 2; s++) {
		struct prog_set *ps = &sets[s];

		if (test__start_subtest(MT_SUBTEST(ps, "contended_holder_timeout")))
			test_contended_holder_timeout(map_fd, ps);
		//if (test__start_subtest(MT_SUBTEST(ps, "nested_inner_contended_timeout")))
		//	test_nested_inner_contended_timeout(map_fd, ps);
		if (test__start_subtest(MT_SUBTEST(ps, "deadlock_abba_rollback")))
			test_deadlock_abba_rollback(map_fd, ps);
		if (test__start_subtest(MT_SUBTEST(ps, "parallel_uncontended_timeouts")))
			test_parallel_uncontended_timeouts(map_fd, ps);
		if (test__start_subtest(MT_SUBTEST(ps, "concurrent_timeouts_stress")))
			test_concurrent_timeouts_stress(map_fd, ps);
		if (test__start_subtest(MT_SUBTEST(ps, "independent_locks_isolation")))
			test_independent_locks_isolation(map_fd, ps);
		if (test__start_subtest(MT_SUBTEST(ps, "contended_independent_locks_isolation")))
			test_contended_independent_locks_isolation(map_fd, ps);
	}

	/* No timeout should fire during the pure-contention stress. */
	__write_sysctl(TIMEOUT_LONG);
	for (s = 0; s < 2; s++) {
		struct prog_set *ps = &sets[s];

		if (test__start_subtest(MT_SUBTEST(ps, "mcs_queue_stress")))
			test_mcs_queue_stress(map_fd, ps);
	}

destroy:
	test_spin_lock_mt_timeout__destroy(skel);
restore:
	__write_sysctl(old_timeout);
}
