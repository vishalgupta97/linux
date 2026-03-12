// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta, Kumar Kartikeya Dwivedi, Sixiao Xiu

#include "spinlock/ffwd_orig.h"
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

#if PRINT_DEBUG_DELEGATION
#define print_debug(fmt, ...)                                                                      \
		({                                                                                         \
				printk(KERN_EMERG "[%d] komb (%s): " fmt, smp_processor_id(), __func__,            \
					   ##__VA_ARGS__);                                                             \
		})
#else
#define print_debug(fmt, ...)
#endif

#if DEBUG_DELEGATION
#define FFWD_BUG_ON(cond) BUG_ON(cond)
#else
#define FFWD_BUG_ON(cond)
#endif

#if KERNEL_SYNCSTRESS
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
		({                                                                                         \
				typeof(ptr) __PTR = (ptr);                                                         \
				__unqual_scalar_typeof(*ptr) VAL;                                                  \
				for (;;) {                                                                         \
						VAL = READ_ONCE(*__PTR);                                                   \
						if (cond_expr)                                                             \
								break;                                                             \
						cpu_relax();                                                               \
						if (need_resched()) {                                                      \
								preempt_enable();                                                  \
								schedule();                                                        \
								preempt_disable();                                                 \
						}                                                                          \
				}                                                                                  \
				(typeof(*ptr)) VAL;                                                                \
		})
#else
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                                                \
		({                                                                                         \
				typeof(ptr) __PTR = (ptr);                                                         \
				__unqual_scalar_typeof(*ptr) VAL;                                                  \
				for (;;) {                                                                         \
						VAL = READ_ONCE(*__PTR);                                                   \
						if (cond_expr)                                                             \
								break;                                                             \
						cpu_relax();                                                               \
				}                                                                                  \
				(typeof(*ptr)) VAL;                                                                \
		})
#endif

static struct task_struct *hthread;

DEFINE_PER_CPU_SHARED_ALIGNED(struct ffwd_orig_delegation_request, ffwd_orig_delegation_requests);
//DEFINE_PER_CPU_SHARED_ALIGNED(struct ffwd_orig_delegation_server, ffwd_orig_delegation_servers);

static struct ffwd_orig_delegation_server server;

#if LOCK_MEASURE_TIME
//static DEFINE_PER_CPU_ALIGNED(uint64_t, combiner_loop);
#endif


#if FFWD_STATS
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_combiner_count);
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_waiters_combined);
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_waiter_cacheline_count_total);
DEFINE_PER_CPU_ALIGNED(uint64_t, ffwd_waiter_cacheline_count);
#endif

static noinline bool get_ith_bit(u64 number, int pos)
{
	return (number << (63L - pos)) >> 63L;
}

static noinline u64 flip_ith_bit(u64 number, int pos)
{
	return number ^ (1L << pos);
}

/**************************************************************
 * 
 * Server thread functions
 * 
 * **************************************************************/

/*****
 * Execute requests on a socket, update ffwd_orig_response together
 * ******/
void ffwd_orig_execute_socket(uint32_t socket_id)
{
		u64 updated_ffwd_orig_response;
		uint32_t socket_offset, i;
		struct ffwd_orig_delegation_request *request;
		struct ffwd_orig_delegation_server *server_ptr =
				&server; //per_cpu_ptr(&ffwd_orig_delegation_servers, DELEGATION_CPU);

		updated_ffwd_orig_response = server_ptr->ffwd_orig_responses[socket_id].toggle;
		//print_debug("Initial server ffwd_orig_response: %lld socket_id: %d\n", updated_ffwd_orig_response, socket_id);
		// Iterate through each core on the socket
		socket_offset = socket_id * CORES_PER_SOCKET;
		for (i = 0; i < CORES_PER_SOCKET; i++) {
#if LOCK_MEASURE_TIME
				LOCK_START_TIMING_PER_CPU_DISABLE(combiner_loop);
#endif
				request = per_cpu_ptr(&ffwd_orig_delegation_requests, i + socket_offset);
				// Check if has request
				// i.e. toggle bit differs in request and reponse
				if (READ_ONCE(request->toggle) != get_ith_bit(server_ptr->ffwd_orig_responses[socket_id].toggle, i)) {
						print_debug("Executing CS coreid: %d  socketid: %d\n", i, socket_id);
						request->req_func_ptr(request->arg1, request->arg2, request->arg3);

						// Update toggle bit
						updated_ffwd_orig_response = flip_ith_bit(updated_ffwd_orig_response, i);
#if LOCK_MEASURE_TIME
						LOCK_END_TIMING_PER_CPU_DISABLE(combiner_loop);
#endif
				}
		}
		//print_debug("New server ffwd_orig_response: %lld socket_id: %d\n", updated_ffwd_orig_response, socket_id);
		// Write ffwd_orig_response
		WRITE_ONCE(server_ptr->ffwd_orig_responses[socket_id].toggle,
			   updated_ffwd_orig_response);
}

int ffwd_orig_thread(void *args)
{
		uint32_t i;

		print_debug("ffwd delegate thread start");

		while (!kthread_should_stop()) {
				// Iterate through each socket
				for (i = 0; i < MAX_CORES / CORES_PER_SOCKET; i++) {
						ffwd_orig_execute_socket(i);
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
 * Init & exit
 * 
 * ************************************************************/

void ffwd_orig_delegate_init(void)
{
		uint32_t i;
		struct ffwd_orig_delegation_request *ptr;
		struct ffwd_orig_delegation_server *server_ptr;
		print_debug("ffwd delegate thread init\n");

		// Init delegation request for each cpu
		for_each_possible_cpu (i) {
				ptr = per_cpu_ptr(&ffwd_orig_delegation_requests, i);
				ptr->cpu_id = i;
				ptr->socket_id = i / CORES_PER_SOCKET;
				ptr->cpu_id_on_socket = i - ptr->socket_id * CORES_PER_SOCKET;
				ptr->toggle = false;

		}

		// Init server info
		/*server_ptr = (struct ffwd_orig_delegation_server *)vzalloc(
		sizeof(struct ffwd_orig_delegation_server));*/
		server_ptr = &server; //per_cpu_ptr(&ffwd_orig_delegation_servers, DELEGATION_CPU);
		for (i = 0; i < (MAX_CORES / CORES_PER_SOCKET); i++) {
				//server_ptr->ffwd_orig_responses[i] = (struct ffwd_orig_response *)vzalloc(sizeof(struct ffwd_orig_response));
				server_ptr->ffwd_orig_responses[i].toggle = false;
		}

#if LOCK_MEASURE_TIME
	*per_cpu_ptr(&do_timing, DELEGATION_CPU) = true;
#endif

		// Init server thread
		hthread = kthread_create(ffwd_orig_thread, NULL, "ffwd_orig_thread");
		// Bind delegation thread to core 19 (current VM: 20 cores)
		kthread_bind(hthread, DELEGATION_CPU);
		if (hthread) {
				wake_up_process(hthread);
		} else {
				printk(KERN_ERR "failed to create ffwd delegate thread\n");
		}
}

void ffwd_orig_delegate_exit(void)
{
		//msleep(2000);  // Wait for remaining task to finish
		int ret = kthread_stop(hthread);

		print_debug("ffwd delegate thread exit\n");

		if (ret) {
				printk(KERN_ALERT "ffwd delegate thread returned error %d\n", ret);
		}
}

__always_inline void ffwd_send_request(void (*ffwd_write_cs)(u32, u32, void*), u32 src_value, u32 dst_value, void* stats)
{
		struct ffwd_orig_delegation_request *request;
		struct ffwd_orig_delegation_server *server_ptr;
		uint32_t socket_id, cpu_id_on_socket;
		bool toggle;

		server_ptr = &server; //per_cpu_ptr(&ffwd_orig_delegation_servers, DELEGATION_CPU);

		request = this_cpu_ptr(&ffwd_orig_delegation_requests);
		socket_id = request->socket_id;
		cpu_id_on_socket = request->cpu_id_on_socket;
		FFWD_BUG_ON(ffwd_write_cs == NULL);
		request->req_func_ptr = ffwd_write_cs;
		request->arg1 = src_value;
		request->arg2 = dst_value;
		request->arg3 = stats;

		// Read toggle bit in ffwd_orig_response
		toggle = !request->toggle;

		// Set request toggle bit
		WRITE_ONCE(request->toggle, toggle);

		print_debug("Sending request to server\n");

		// Wait for the server to execute request
		// i.e. toggle bit != toggle bit on server
		print_debug("wait on toggle bit [%d]", toggle);
		while(toggle != get_ith_bit(
			READ_ONCE(server_ptr->ffwd_orig_responses[socket_id].toggle), cpu_id_on_socket)) {
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
		print_debug("woken up");
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
