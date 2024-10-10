// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <linux/combiner.h>
#include <linux/feedbacksync.h>
long komb_batch_size = 262144;

#include <linux/syscalls.h>
/*
 * incoming_rsp_ptr -> rdi
 * outgoing_rsp_ptr -> rsi
 *
 * Assume IRQ is disabled. Will enable when returning.
 */
#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace void
komb_context_switch(void *incoming_rsp_ptr, void *outgoing_rsp_ptr)
{
	asm volatile("pushq %%rbp\n"
		     "pushq %%rbx\n"
		     "pushq %%r12\n"
		     "pushq %%r13\n"
		     "pushq %%r14\n"
		     "pushq %%r15\n"
		     "movq %%rsp, (%%rsi)\n"
		     "movq (%%rdi), %%rsp\n"
		     "popq %%r15\n"
		     "popq %%r14\n"
		     "popq %%r13\n"
		     "popq %%r12\n"
		     "popq %%rbx\n"
		     "popq %%rbp\n"
		     :
		     :
		     : "memory");
}
#pragma GCC pop_options

#ifdef KOMB_STATS

DEFINE_PER_CPU_ALIGNED(u64, lock_not_in_task);

void print_komb_stats(void)
{
	printk(KERN_ALERT "======== KOMB spinlock stats ========\n");
	int i;
	uint64_t total_counters[17] = { 0 };
	for_each_online_cpu(i) {
		total_counters[0] += per_cpu(combiner_count, i);
		total_counters[1] += per_cpu(waiter_combined, i);
		total_counters[2] += per_cpu(ooo_unlocks, i);
		total_counters[3] += per_cpu(ooo_combiner_count, i);
		total_counters[4] += per_cpu(ooo_waiter_combined, i);
		total_counters[5] += per_cpu(lock_not_in_task, i);
		total_counters[6] += per_cpu(mutex_combiner_count, i);
		total_counters[7] += per_cpu(mutex_waiter_combined, i);
		total_counters[8] += per_cpu(mutex_ooo_unlocks, i);
		total_counters[9] += per_cpu(mutex_qspinlock, i);
		total_counters[10] += per_cpu(mutex_tclock, i);
		total_counters[11] += per_cpu(rwsem_combiner_count, i);
		total_counters[12] += per_cpu(rwsem_waiter_combined, i);
		total_counters[13] += per_cpu(rwsem_ooo_unlocks, i);
		total_counters[14] += per_cpu(rwsem_reads, i);
		total_counters[15] += per_cpu(rwsem_writes, i);
		total_counters[16] += per_cpu(rwsem_downgrade, i);
	}

	printk(KERN_ALERT "Combiner_count: %ld\n", total_counters[0]);
	printk(KERN_ALERT "waiter_combined: %ld\n", total_counters[1]);
	printk(KERN_ALERT "ooo_unlocks: %ld\n", total_counters[2]);
	printk(KERN_ALERT "ooo_combiner_count: %ld\n", total_counters[3]);
	printk(KERN_ALERT "ooo_waiter_combined: %ld\n", total_counters[4]);
	printk(KERN_ALERT "lock_not_in_task: %ld\n", total_counters[5]);
	printk(KERN_ALERT "mutex_Combiner_count: %ld\n", total_counters[6]);
	printk(KERN_ALERT "mutex_waiter_combined: %ld\n", total_counters[7]);
	printk(KERN_ALERT "mutex_ooo_unlocks: %ld\n", total_counters[8]);
	printk(KERN_ALERT "mutex_qspinlock: %ld\n", total_counters[9]);
	printk(KERN_ALERT "mutex_tclock: %ld\n", total_counters[10]);
	printk(KERN_ALERT "rwsem_Combiner_count: %ld\n", total_counters[11]);
	printk(KERN_ALERT "rwsem_waiter_combined: %ld\n", total_counters[12]);
	printk(KERN_ALERT "rwsem_ooo_unlocks: %ld\n", total_counters[13]);
	printk(KERN_ALERT "rwsem_reads: %ld\n", total_counters[14]);
	printk(KERN_ALERT "rwsem_writes: %ld\n", total_counters[15]);
	printk(KERN_ALERT "rwsem_downgrade: %ld\n", total_counters[16]);
}

SYSCALL_DEFINE0(komb_stats)
{
	print_komb_stats();
	print_fds_stats();
	return 0;
}

SYSCALL_DEFINE0(komb_clear_stats)
{
	printk(KERN_ALERT "======== KOMB stats cleared ========\n");
	int i;
	for_each_online_cpu(i) {
		*per_cpu_ptr(&combiner_count, i) = 0;
		*per_cpu_ptr(&waiter_combined, i) = 0;
		*per_cpu_ptr(&ooo_unlocks, i) = 0;
		*per_cpu_ptr(&ooo_combiner_count, i) = 0;
		*per_cpu_ptr(&ooo_waiter_combined, i) = 0;
		*per_cpu_ptr(&lock_not_in_task, i) = 0;
		*per_cpu_ptr(&mutex_combiner_count, i) = 0;
		*per_cpu_ptr(&mutex_waiter_combined, i) = 0;
		*per_cpu_ptr(&mutex_ooo_unlocks, i) = 0;
		*per_cpu_ptr(&mutex_qspinlock, i) = 0;
		*per_cpu_ptr(&mutex_tclock, i) = 0;
		*per_cpu_ptr(&rwsem_combiner_count, i) = 0;
		*per_cpu_ptr(&rwsem_waiter_combined, i) = 0;
		*per_cpu_ptr(&rwsem_ooo_unlocks, i) = 0;
		*per_cpu_ptr(&rwsem_reads, i) = 0;
		*per_cpu_ptr(&rwsem_writes, i) = 0;
		*per_cpu_ptr(&rwsem_downgrade, i) = 0;
	}
	return 0;
}

#else
SYSCALL_DEFINE0(komb_stats)
{
	return 0;
}

SYSCALL_DEFINE0(komb_clear_stats)
{
	return 0;
}
#endif
