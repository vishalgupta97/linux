#ifndef __FAA_H_
#define __FAA_H_

#define DECLARE_TABLE_LOCK(l, linit, wl, wul)                         \
                                                                               \
	static __cacheline_aligned_in_smp linit(l);                            \
                                                                               \
	static void l##_write_lock_buckets(void)      \
	{                                                                      \
		preempt_disable();                                             \
		wl(&l);                                                        \
	}                                                                      \
                                                                               \
	static void l##_write_unlock_buckets(void)    \
	{                                                                      \
		wul(&l);                                                       \
		preempt_enable();                                              \
	}                                                                      \


#define DECLARE_TABLE_LOCK_FFWD(l)                         \
                                                                               \
	static void l##_write_lock_buckets(void)      \
	{BUG_ON(true);                                                                      \
	}                                                                      \
                                                                               \
	static void l##_write_unlock_buckets(void)    \
	{BUG_ON(true);                                                                      \
	}                                                                      \
                                                                               \



#endif /* __FAA_H_ */
