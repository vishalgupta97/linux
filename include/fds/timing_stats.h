#ifndef __TIMING_STATS_H_
#define __TIMING_STATS_H_

#include <asm/msr.h>
#include <linux/cpu.h>
#include <linux/fds.h>

static inline unsigned long locktime_timing_start(void)
{
	unsigned long rax, rdx;
	__asm__ __volatile__("rdtscp\n" : "=a"(rax), "=d"(rdx) : : "%ecx");
	return (rdx << 32) + rax;
}

static inline unsigned long locktime_timing_end(void)
{
	unsigned long rax, rdx;
	__asm__ __volatile__("rdtscp\n" : "=a"(rax), "=d"(rdx) : : "%ecx");
	return (rdx << 32) + rax;
}

#if FDS_MEASURE_TIME

#define LOCK_START_TIMING_PER_CPU_DISABLE(name)
#define LOCK_END_TIMING_PER_CPU_DISABLE(name)

#define LOCK_START_TIMING_PER_CPU(name)                         \
	do {                                                    \
		barrier();                                      \
		*this_cpu_ptr(&name) = locktime_timing_start(); \
		barrier();                                      \
	} while (0)

#define LOCK_END_TIMING_PER_CPU(name)                                   \
	unsigned long name##_diff;                                      \
	barrier();                                                      \
	name##_diff = (locktime_timing_end() - (*this_cpu_ptr(&name))); \
	barrier();

#define LOCK_GET_TIMING_DIFF(name) (name##_diff)

#else // FDS_MEASURE_TIME

#define LOCK_START_TIMING_PER_CPU_DISABLE(name)
#define LOCK_END_TIMING_PER_CPU_DISABLE(name)
#define LOCK_START_TIMING_PER_CPU(name)
#define LOCK_END_TIMING_PER_CPU(name)
#endif

#endif /* __TIMING_STATS_H_ */
