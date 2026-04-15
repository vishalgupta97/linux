// SPDX-License-Identifier: GPL-2.0
#include <pthread.h>
#include <test_progs.h>
#include <network_helpers.h>

#include "execute_fn_with_lock.skel.h"
#include "execute_fn_with_lock_fail.skel.h"

struct shared_val {
	struct bpf_spin_lock lock;
	int cnt;
};

static char log_buf[1024 * 1024];

static struct {
	const char *prog_name;
	const char *err_msg;
} fail_tests[] = {
	{ "execute_fn_with_lock_null_deref", "invalid mem access 'scalar'" },
	{ "execute_fn_with_lock_bad_ret", "At callback return the register R0 has" },
};

struct thread_arg {
	int prog_fd;
	int repeat;
};

static void *run_thread(void *arg)
{
	struct thread_arg *thread_arg = arg;
	LIBBPF_OPTS(bpf_test_run_opts, topts,
		.data_in = &pkt_v4,
		.data_size_in = sizeof(pkt_v4),
		.repeat = thread_arg->repeat,
	);
	int err;

	err = bpf_prog_test_run_opts(thread_arg->prog_fd, &topts);
	ASSERT_OK(err, "bpf_prog_test_run_opts");
	ASSERT_OK(topts.retval, "test_run_retval");

	return arg;
}

static void run_fail_prog(const char *prog_name, const char *err_msg)
{
	LIBBPF_OPTS(bpf_object_open_opts, opts,
		.kernel_log_buf = log_buf,
		.kernel_log_size = sizeof(log_buf),
		.kernel_log_level = 1);
	struct execute_fn_with_lock_fail *skel;
	struct bpf_program *prog;
	int ret;

	skel = execute_fn_with_lock_fail__open_opts(&opts);
	if (!ASSERT_OK_PTR(skel, "execute_fn_with_lock_fail__open_opts"))
		return;

	prog = bpf_object__find_program_by_name(skel->obj, prog_name);
	if (!ASSERT_OK_PTR(prog, "bpf_object__find_program_by_name"))
		goto cleanup;

	bpf_program__set_autoload(prog, true);

	ret = execute_fn_with_lock_fail__load(skel);
	if (!ASSERT_ERR(ret, "execute_fn_with_lock_fail__load"))
		goto cleanup;

	if (!ASSERT_HAS_SUBSTR(log_buf, err_msg, "expected_verifier_msg"))
		fprintf(stderr, "Verifier log:\n%s\n", log_buf);

cleanup:
	execute_fn_with_lock_fail__destroy(skel);
}

void test_execute_fn_with_lock(void)
{
	const int thread_cnt = 4;
	const int repeat = 2000;
	struct execute_fn_with_lock *skel;
	pthread_t threads[thread_cnt];
	struct thread_arg arg;
	struct shared_val value;
	int map_fd;
	int key = 0;
	int i;
	void *ret;

	skel = execute_fn_with_lock__open_and_load();
	if (!ASSERT_OK_PTR(skel, "execute_fn_with_lock__open_and_load"))
		return;

	arg.prog_fd = bpf_program__fd(skel->progs.execute_fn_with_lock_test);
	arg.repeat = repeat;

	for (i = 0; i < thread_cnt; i++) {
		if (!ASSERT_OK(pthread_create(&threads[i], NULL, run_thread, &arg),
			       "pthread_create"))
			goto cleanup;
	}

	for (i = 0; i < thread_cnt; i++) {
		if (!ASSERT_OK(pthread_join(threads[i], &ret), "pthread_join"))
			goto cleanup;
		if (!ASSERT_EQ(ret, &arg, "thread_return"))
			goto cleanup;
	}

	if (!ASSERT_EQ(skel->bss->failures, 0, "helper_failures"))
		goto cleanup;
	if (!ASSERT_EQ(skel->bss->null_cb_seen, thread_cnt * repeat, "null_cb_seen"))
		goto cleanup;

	map_fd = bpf_map__fd(skel->maps.shared_map);
	if (!ASSERT_OK(bpf_map_lookup_elem(map_fd, &key, &value), "bpf_map_lookup_elem"))
		goto cleanup;

	ASSERT_EQ(value.cnt, thread_cnt * repeat, "locked_increment_count");

cleanup:
	execute_fn_with_lock__destroy(skel);

	for (i = 0; i < ARRAY_SIZE(fail_tests); i++) {
		if (!test__start_subtest(fail_tests[i].prog_name))
			continue;
		run_fail_prog(fail_tests[i].prog_name, fail_tests[i].err_msg);
	}
}
