// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta, Kumar Kartikeya Dwivedi, Sixiao Xu

#include "spinlock/ffwd.h"
#include "spinlock/komb.h"
#include "timing_stats.h"

#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/atomic.h>

#if LOCK_MEASURE_TIME
static DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_combiner_loop);
//static DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_combiner_loop_unlockfn);
#endif

#if DSM_DEBUG
#define print_debug(fmt, ...)                                                                      \
	({ printk(KERN_EMERG "[%d] komb (%s): " fmt, smp_processor_id(), __func__, ##__VA_ARGS__); })
#else
#define print_debug(fmt, ...)
#endif

#if KERNEL_SYNCSTRESS
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
	({                                                                                               \
		typeof(ptr) __PTR = (ptr);                                                                     \
		__unqual_scalar_typeof(*ptr) VAL;                                                              \
		for (;;) {                                                                                     \
			VAL = READ_ONCE(*__PTR);                                                                     \
			if (cond_expr)                                                                               \
				break;                                                                                     \
			cpu_relax();                                                                                 \
			if (need_resched()) {                                                                        \
				preempt_enable();                                                                          \
				schedule();                                                                                \
				preempt_disable();                                                                         \
			}                                                                                            \
		}                                                                                              \
		(typeof(*ptr)) VAL;                                                                            \
	})
#else
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
	({                                                                                               \
		typeof(ptr) __PTR = (ptr);                                                                     \
		__unqual_scalar_typeof(*ptr) VAL;                                                              \
		for (;;) {                                                                                     \
			VAL = READ_ONCE(*__PTR);                                                                     \
			if (cond_expr)                                                                               \
				break;                                                                                     \
			cpu_relax();                                                                                 \
		}                                                                                              \
		(typeof(*ptr)) VAL;                                                                            \
	})
#endif

static struct task_struct *hthread;

DEFINE_PER_CPU_SHARED_ALIGNED(struct ffwd_node, ffwd_nodes[MAX_NODES]);

DEFINE_PER_CPU_SHARED_ALIGNED(struct delegation_request, delegation_requests);
//DEFINE_PER_CPU_SHARED_ALIGNED(struct delegation_server, delegation_servers);

static struct delegation_server server;

#if FFWD_STATS
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_waiters_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_waiter_cacheline_count_total);
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_waiter_cacheline_count);
#endif

__attribute__((noipa)) noinline notrace static void *get_shadow_stack_ptr(void)
{
	struct delegation_request *ptr = this_cpu_ptr(&delegation_requests);
	return &(ptr->shadow_stack_ptr);
}

__attribute__((noipa)) noinline notrace static struct delegation_request *
get_delegation_request(void)
{
	return this_cpu_ptr(&delegation_requests);
}

#if ENABLE_RESPONSE_BATCHING
static noinline bool get_ith_bit(u64 number, int pos)
{
	return (number << (63L - pos)) >> 63L;
}

static noinline u64 flip_ith_bit(u64 number, int pos)
{
	return number ^ (1L << pos);
}
#endif

/**************************************************************
 * 
 * Server thread functions
 * 
 * **************************************************************/

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void ffwd_execute(struct delegation_request *request)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct delegation_server *server_ptr =
		&server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);

	print_debug("delegate [%d]", request->cpu_id);

	// Server -> client thread
	incoming_rsp_ptr = &(request->main_stack_ptr);
	outgoing_rsp_ptr = &(server_ptr->server_stack_ptr);

#if DEBUG_DELEGATION
	BUG_ON(smp_processor_id() != DELEGATION_CPU);
	BUG_ON(*(char *)incoming_rsp_ptr == 0);
	//BUG_ON(*(char *)outgoing_rsp_ptr == 0);
#endif

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
/*****
 * Execute requests on a socket, update response together
 * ******/
void ffwd_execute_socket(uint32_t socket_id)
{
#if ENABLE_JUMP == 0
#if ENABLE_RESPONSE_BATCHING
	u64 updated_response;
#endif // ENABLE_RESPONSE_BATCHING
	uint32_t socket_min_cpu, socket_max_cpu, i;
	struct delegation_server *server_ptr =
		&server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);

#if ENABLE_RESPONSE_BATCHING
	updated_response = server_ptr->responses[socket_id].toggle;
#endif

	// Iterate through each core on the socket
	socket_min_cpu = socket_id * CORES_PER_SOCKET;
	socket_max_cpu = (socket_id + 1) * CORES_PER_SOCKET;
	for (i = socket_min_cpu; i < socket_max_cpu; i++) {
		struct delegation_request *request = per_cpu_ptr(&delegation_requests, i);
		// Check if has request
		// i.e. toggle bit differs in request and reponse
#if ENABLE_RESPONSE_BATCHING
		if (READ_ONCE(request->toggle) != get_ith_bit(server_ptr->responses[socket_id].toggle, request->cpu_id_on_socket)) {
#else
		if (READ_ONCE(request->toggle) != READ_ONCE(server_ptr->responses[i].toggle)) {
#endif

#if ENABLE_PREFETCH
			if ((i + 1) < socket_max_cpu) {
				int j;
				struct delegation_request *next_request = per_cpu_ptr(&delegation_requests, i + 1);
				prefetchw(next_request->main_stack_ptr);
				for (j = 1; j < NUM_PREFETCH_LINES; j++)
					prefetchw(next_request->main_stack_ptr + (64 * j));
				prefetchw(&server_ptr->responses[i + 1].toggle);
			}
#endif
#if LOCK_MEASURE_TIME
			LOCK_START_TIMING_PER_CPU_DISABLE(ffwd_combiner_loop);
#endif
			server_ptr->cur_client_cpu_id = request->cpu_id;
			ffwd_execute(request);

			// Update toggle bit
#if ENABLE_RESPONSE_BATCHING
			updated_response = flip_ith_bit(updated_response, request->cpu_id_on_socket);
#else
			WRITE_ONCE(server_ptr->responses[i].toggle, !(READ_ONCE(server_ptr->responses[i].toggle)));
#endif

#if LOCK_MEASURE_TIME
			LOCK_END_TIMING_PER_CPU_DISABLE(ffwd_combiner_loop);
#endif
		}
	}
	// Write response
#if ENABLE_RESPONSE_BATCHING
	WRITE_ONCE(server_ptr->responses[socket_id].toggle, updated_response);
#endif

#else
	uint32_t socket_min_cpu, socket_max_cpu, i;
	bool found_one;
	struct delegation_server *server_ptr =
		&server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);

#if ENABLE_RESPONSE_BATCHING
	BUG_ON(true); //Response batching not implemented
#endif

	//server_ptr->cur_updated_response = server_ptr->responses[socket_id].toggle;
	server_ptr->cur_socket_id = socket_id;
	// Find first core with request on current socket
	socket_min_cpu = socket_id * CORES_PER_SOCKET;
	socket_max_cpu = (socket_id + 1) * CORES_PER_SOCKET;
	found_one = false;
	for (i = socket_min_cpu; i < socket_max_cpu; i++) {
		struct delegation_request *request = per_cpu_ptr(&delegation_requests, i);
		// Check if has request
		// i.e. toggle bit differs in request and reponse
		if (READ_ONCE(request->toggle) != READ_ONCE(server_ptr->responses[i].toggle)) {
#if LOCK_MEASURE_TIME
			*this_cpu_ptr(&ffwd_combiner_loop) = UINT64_MAX;
			BUG_ON(smp_processor_id() != DELEGATION_CPU);
#endif
			server_ptr->prev_client_cpu_id = -1;
			server_ptr->next_client_cpu_id = -1;
			server_ptr->cur_client_cpu_id = request->cpu_id;
			server_ptr->waiters_combined = 0;
			found_one = true;
			ffwd_execute(request);
			// jump to next core will be handled in delegate_finish()
			break;
		}
	}
	if (found_one) {
		i = server_ptr->prev_client_cpu_id;
#if DEBUG_DELEGATION
		BUG_ON(i == -1);
#endif
		WRITE_ONCE(server_ptr->responses[i].toggle, !(READ_ONCE(server_ptr->responses[i].toggle)));
		server_ptr->prev_client_cpu_id = -1;
#if FFWD_STATS
		this_cpu_add(ffwd_waiters_combined, server_ptr->waiters_combined);
		this_cpu_inc(ffwd_combiner_count);
#endif
	}

	// Write response
	//WRITE_ONCE(server_ptr->responses[socket_id].toggle,
	//	   server_ptr->cur_updated_response);
#endif
}

int ffwd_thread(void *args)
{
	uint32_t i;

	print_debug("ffwd delegate thread start");

	while (!kthread_should_stop()) {
		// Iterate through each socket
		for (i = 0; i < MAX_CORES / CORES_PER_SOCKET; i++) {
			ffwd_execute_socket(i);
			if (need_resched()) {
				cond_resched();
			}
		}
	}

	cond_resched();

	__set_current_state(TASK_RUNNING);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);

	return 0;
}

/**************************************************************
 * 
 * Client thread functions
 * 
 * **************************************************************/

/******
 * 3. Set up request, wait for response
 * *******/
__attribute__((noipa)) noinline notrace static void __ffwd_delegate_slowpath(struct qspinlock *lock)
{
	struct delegation_request *request;
	struct delegation_server *server_ptr;
	uint32_t socket_id;
#if ENABLE_RESPONSE_BATCHING
	uint32_t cpu_id_on_socket;
#endif
	bool toggle;

	server_ptr = &server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);

	request = this_cpu_ptr(&delegation_requests);
	socket_id = request->socket_id;
#if ENABLE_RESPONSE_BATCHING
	cpu_id_on_socket = request->cpu_id_on_socket;
#endif
	// Read toggle bit in response
	toggle = !request->toggle;

	// LOCK_START_TIMING_PER_CPU(delegation_loop);
	// Set request toggle bit
	WRITE_ONCE(request->toggle, toggle);

	// Wait for the server to execute request
	// i.e. toggle bit != toggle bit on server
	print_debug("wait on toggle bit [%d]", toggle);
#if ENABLE_RESPONSE_BATCHING
	while(toggle != get_ith_bit(
			READ_ONCE(server_ptr->responses[socket_id].toggle), cpu_id_on_socket)) {
#else
	while (READ_ONCE(request->toggle) != READ_ONCE(server_ptr->responses[request->cpu_id].toggle)) {
#endif

#if DEBUG_DELEGATION
		BUG_ON(READ_ONCE(request->toggle) != toggle);
#endif
		cpu_relax();
		if (need_resched()) {
			preempt_enable();
			schedule();
			preempt_disable();
		}
	}
	// LOCK_END_TIMING_PER_CPU(delegation_loop);
	print_debug("wake up");
}

/*******
 * 2. Client main -> shadow thread
 * ******/
__attribute__((noipa)) noinline notrace void ffwd_delegate_slowpath(struct qspinlock *lock)
{
	asm volatile("pushq %%rbp\n"
							 "pushq %%rbx\n"
							 "pushq %%r12\n"
							 "pushq %%r13\n"
							 "pushq %%r14\n"
							 "pushq %%r15\n"
							 :
							 :
							 : "memory");
	asm volatile("callq %P0\n"
							 "movq %%rsp, %c1(%%rax)\n"
							 :
							 : "i"(get_delegation_request),
								 "i"(offsetof(struct delegation_request, main_stack_ptr))
							 : "memory");
	asm volatile("callq %P0\n"
							 "movq (%%rax), %%rsp\n"
							 :
							 : "i"(get_shadow_stack_ptr)
							 : "memory");

	__ffwd_delegate_slowpath(lock);

	asm volatile("callq %P0\n"
							 "movq %%rsp, (%%rax)\n"
							 :
							 : "i"(get_shadow_stack_ptr)
							 : "memory");
	asm volatile("callq %P0\n"
							 "movq %c1(%%rax), %%rsp\n"
							 :
							 : "i"(get_delegation_request),
								 "i"(offsetof(struct delegation_request, main_stack_ptr))
							 : "memory");
	asm volatile("popq %%r15\n"
							 "popq %%r14\n"
							 "popq %%r13\n"
							 "popq %%r12\n"
							 "popq %%rbx\n"
							 "popq %%rbp\n"
							 "retq\n"
							 :
							 :
							 : "memory");
}
#pragma GCC pop_options

/*********
 * Called by client thread, to replace unlock()
 * Executed by server thread
 * ********/
__attribute__((noipa)) noinline notrace void ffwd_delegate_finish(struct qspinlock *lock)
{
	struct delegation_request *request;
	struct delegation_server *server_ptr;
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;

#if ENABLE_JUMP
	struct delegation_request *next_request;
	uint32_t next_cli;
#endif

#if LOCK_MEASURE_TIME
	LOCK_END_TIMING_PER_CPU(ffwd_combiner_loop);
	LOCK_START_TIMING_PER_CPU(ffwd_combiner_loop);
#endif

	LOCK_START_TIMING_PER_CPU_DISABLE(ffwd_combiner_loop_unlockfn);

	server_ptr = &server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);

	//print_debug("finish [%d]", server_ptr->cur_client_cpu_id);
	request = per_cpu_ptr(&delegation_requests, server_ptr->cur_client_cpu_id);

#if DEBUG_DELEGATION
	BUG_ON(smp_processor_id() != DELEGATION_CPU);
#endif

#if ENABLE_JUMP == 0
	// Client -> server thread
	incoming_rsp_ptr = &(server_ptr->server_stack_ptr);
	outgoing_rsp_ptr = &(request->main_stack_ptr);

#if DEBUG_DELEGATION
	BUG_ON(*(char *)incoming_rsp_ptr == 0);
#endif

	LOCK_END_TIMING_PER_CPU_DISABLE(ffwd_combiner_loop_unlockfn);

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	return;
#else
	// Update temporary toggle bit
	//server_ptr->cur_updated_response =
	//	flip_ith_bit(server_ptr->cur_updated_response,
	//		     server_ptr->cur_client_cpu_id);
	// Find next core with request on same socket

	next_cli = server_ptr->next_client_cpu_id;
	server_ptr->prev_client_cpu_id = server_ptr->cur_client_cpu_id;
	//prefetchw(&(server_ptr->responses[server_ptr->prev_client_cpu_id].toggle));

	if (next_cli != -1 && !need_resched()) {
		next_request = per_cpu_ptr(&delegation_requests, next_cli);

		print_debug("jump [%d]->[%d]", request->cpu_id, next_request->cpu_id);

		server_ptr->cur_client_cpu_id = next_request->cpu_id;
		server_ptr->waiters_combined++;
		// Client -> next client with request on same socket
		incoming_rsp_ptr = &(next_request->main_stack_ptr);
	} else {
		print_debug("back [%d]->[s]", server_ptr->cur_client_cpu_id);
		server_ptr->cur_client_cpu_id = -1;
		// Jump back to server
		incoming_rsp_ptr = &(server_ptr->server_stack_ptr);
	}

	// LOCK_END_TIMING_PER_CPU(delegation_loop);

	// LOCK_START_TIMING_PER_CPU(delegation_loop);

	outgoing_rsp_ptr = &(request->main_stack_ptr);

	LOCK_END_TIMING_PER_CPU_DISABLE(ffwd_combiner_loop_unlockfn);

#if DEBUG_DELEGATION
	BUG_ON(*(char *)incoming_rsp_ptr == 0);
#endif
	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	return;
#endif
}

/**************************************************************
 * 
 * Init & exit
 * 
 * ************************************************************/

void ffwd_delegate_init(struct qspinlock *lock)
{
	uint32_t i;
	void *stack_ptr;
	struct delegation_request *ptr;
	struct delegation_server *server_ptr;
	print_debug("ffwd delegate thread init\n");

	// Init delegation request for each cpu
	for_each_possible_cpu (i) {
		stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		BUG_ON(stack_ptr == NULL);
		ptr = per_cpu_ptr(&delegation_requests, i);
		ptr->base_ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		ptr->shadow_stack_ptr = NULL;
		ptr->shadow_stack_ptr = stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->cpu_id = i;
		ptr->socket_id = i / CORES_PER_SOCKET;
		ptr->cpu_id_on_socket = i - ptr->socket_id * CORES_PER_SOCKET;
		ptr->toggle = false;

#if LOCK_MEASURE_TIME
		*per_cpu_ptr(&ffwd_combiner_loop, i) = UINT64_MAX;
#endif
	}

	// Init server info
	/*server_ptr = (struct delegation_server *)vzalloc(
		sizeof(struct delegation_server));*/
	server_ptr = &server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);
	server_ptr->lock = lock;
	server_ptr->server_stack_ptr = NULL;
	server_ptr->prev_client_cpu_id = -1;
	server_ptr->cur_client_cpu_id = -1;
	server_ptr->next_client_cpu_id = -1;
	for (i = 0; i < MAX_CORES; i++) {
		//server_ptr->responses[i] = (struct response *)vzalloc(sizeof(struct response));
		server_ptr->responses[i].toggle = 0;
	}

#if LOCK_MEASURE_TIME
	*per_cpu_ptr(&do_timing, DELEGATION_CPU) = true;
#endif

	// Init server thread
	hthread = kthread_create(ffwd_thread, NULL, "ffwd_thread");
	// Bind delegation thread to core 19 (current VM: 20 cores)
	kthread_bind(hthread, DELEGATION_CPU);
	if (hthread) {
		wake_up_process(hthread);
	} else {
		printk(KERN_ERR "failed to create ffwd delegate thread\n");
	}
}

void ffwd_delegate_exit(void)
{
	//msleep(2000);  // Wait for remaining task to finish
	int ret = kthread_stop(hthread);
	uint32_t i;

	print_debug("ffwd delegate thread exit\n");

	if (ret) {
		printk(KERN_ALERT "ffwd delegate thread returned error %d\n", ret);
	}

	for_each_possible_cpu (i) {
		vfree(per_cpu_ptr(&delegation_requests, i)->base_ptr - SIZE_OF_SHADOW_STACK);
	}
}

__attribute__((noipa)) noinline notrace void ffwd_lock(struct qspinlock *lock)
{
#if ENABLE_JUMP
	uint32_t cur_cli, prev_cli, next_cli, i;
	struct delegation_server *server_ptr;
	bool found_next;
	struct delegation_request *next_request, *tmp_request;
	uint32_t socket_id, socket_max_cpu;
	void *rsp_ptr;
	int cacheline_count;
#endif
	ffwd_delegate_slowpath(lock);

#if DEBUG_DELEGATION
	BUG_ON(smp_processor_id() != DELEGATION_CPU); // Executed on delegation CPU
#endif

#if ENABLE_JUMP
	server_ptr = &server; //per_cpu_ptr(&delegation_servers, DELEGATION_CPU);
	cur_cli = server_ptr->cur_client_cpu_id;

#if DEBUG_DELEGATION
	BUG_ON(cur_cli == DELEGATION_CPU);
	BUG_ON(cur_cli == -1);
#endif

	prev_cli = server_ptr->prev_client_cpu_id;
	if (prev_cli != -1) {
		WRITE_ONCE(server_ptr->responses[prev_cli].toggle,
							 !(READ_ONCE(server_ptr->responses[prev_cli].toggle)));
		server_ptr->prev_client_cpu_id = -1;
	}

	found_next = false;
	next_request = NULL;
	socket_id = server_ptr->cur_socket_id;
	socket_max_cpu = (socket_id + 1) * CORES_PER_SOCKET;
	next_cli = server_ptr->cur_client_cpu_id + 1;
	cacheline_count = 0;

	for (; next_cli < socket_max_cpu; next_cli++) {
		cacheline_count++;
		next_request = per_cpu_ptr(&delegation_requests, next_cli);
		// Check if has request
		// i.e. toggle bit differs in request and reponse
		if (READ_ONCE(next_request->toggle) != READ_ONCE(server_ptr->responses[next_cli].toggle)) {
			found_next = true;
			break;
		}
	}


	if (!found_next) {
		//Do another run from start.
		next_cli = socket_id * CORES_PER_SOCKET;
		for (; next_cli < socket_max_cpu; next_cli++) {
			cacheline_count++;
			if (next_cli == server_ptr->cur_client_cpu_id)
				continue;
			next_request = per_cpu_ptr(&delegation_requests, next_cli);
			// Check if has request
			// i.e. toggle bit differs in request and reponse
			if (READ_ONCE(next_request->toggle) != READ_ONCE(server_ptr->responses[next_cli].toggle)) {
				found_next = true;
				break;
			}
		}
	}


#if FFWD_STATS
	this_cpu_add(ffwd_waiter_cacheline_count_total, cacheline_count);
	this_cpu_inc(ffwd_waiter_cacheline_count);
#endif

	if (found_next) {
		server_ptr->next_client_cpu_id = next_cli;
#if ENABLE_PREFETCH
		tmp_request = per_cpu_ptr(&delegation_requests, ((next_cli + 1) % MAX_CORES));
		prefetchw(tmp_request);
		prefetch((char *)tmp_request + 64);
		rsp_ptr = next_request->main_stack_ptr;
		prefetchw(rsp_ptr);
		for (i = 1; i < NUM_PREFETCH_LINES; i++)
			prefetchw(rsp_ptr + (64 * i));

#endif
	} else {
		server_ptr->next_client_cpu_id = -1;
	}
#endif
}

//__attribute__((noipa)) noinline notrace
void __always_inline ffwd_unlock(struct qspinlock *lock)
{
#if DEBUG_DELEGATION
	BUG_ON(smp_processor_id() != DELEGATION_CPU); // Executed on delegation CPU
#endif
	ffwd_delegate_finish(lock);
}

void ffwd_helper_exit(void)
{
	int ret = kthread_stop(hthread);
	if (ret)
		printk(KERN_ALERT "ffwd helper thread returned error %d\n", ret);
}

#if FFWD_STATS
void ffwd_print_stats(void)
{
	uint32_t i;
	uint64_t total_counters[4] = { 0 };
	printk(KERN_ALERT "======== FFWD spinlock stats ========\n");
	for_each_online_cpu (i) {
		total_counters[0] += per_cpu(ffwd_combiner_count, i);
		total_counters[1] += per_cpu(ffwd_waiters_combined, i);
		total_counters[2] += per_cpu(ffwd_waiter_cacheline_count_total, i);
		total_counters[3] += per_cpu(ffwd_waiter_cacheline_count, i);
	}

	printk(KERN_ALERT "Combiner_count: %lld\n", total_counters[0]);
	printk(KERN_ALERT "waiter_combined: %lld\n", total_counters[1]);
	printk(KERN_ALERT "Waiter_cacheline_count_total: %lld\n", total_counters[2]);
	printk(KERN_ALERT "Waiter_cacheline_count: %lld\n", total_counters[3]);
}
#endif
