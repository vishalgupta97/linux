/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_BPF_LOCK_TIMER_H
#define _LINUX_BPF_LOCK_TIMER_H

#include <linux/hrtimer.h>
#include <linux/types.h>

/**
 * struct bpf_lock_timer_ops - timer backend operations
 * @start: Start a timeout. @ctx is backend-specific state, @timeout_ns is
 *         the timeout duration in nanoseconds.
 * @cancel: Cancel a previously started timeout.
 */
struct bpf_lock_timer_ops {
	void (*start)(void *ctx, u64 timeout_ns);
	void (*cancel)(void *ctx);
};

/**
 * struct bpf_lock_timer - timer instance
 * @ops:  Pointer to the active backend operations.
 * @ctx:  Backend-specific context (e.g., pointer to an hrtimer).
 */
struct bpf_lock_timer {
	const struct bpf_lock_timer_ops *ops;
	void *ctx;
    int bpf_cpuid;
};

static inline void bpf_lock_timer_start(struct bpf_lock_timer *t, u64 timeout_ns)
{
	if (t && t->ops && t->ops->start)
		t->ops->start(t->ctx, timeout_ns);
}

static inline void bpf_lock_timer_cancel(struct bpf_lock_timer *t)
{
	if (t && t->ops && t->ops->cancel)
		t->ops->cancel(t->ctx);
}

#endif /* _LINUX_BPF_LOCK_TIMER_H */
