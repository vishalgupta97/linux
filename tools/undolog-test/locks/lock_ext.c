// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

/*
 * TODO: (Performance optimiztion)
 * Fix the index part when same is acquired multiple times.
 * Currently nested locking becomes TTAS lock.
 * irqs_disabled() will hurt performance when running in VM.
 */
#if KERNEL_SYNCSTRESS
#include "spinlock/lock_ext.h"
#include "timing_stats.h"
#else
#include <asm-generic/qspinlock.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#endif
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

#if DSM_DEBUG
#define print_debug(fmt, ...)                                                  \
	({                                                                     \
		printk(KERN_ALERT "[%d] lock_ext (%s): " fmt,                  \
		       smp_processor_id(), __func__, ##__VA_ARGS__);           \
	})
#else
#define print_debug(fmt, ...)
#endif

#if KERNEL_SYNCSTRESS
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                            \
	({                                                                     \
		typeof(ptr) __PTR = (ptr);                                     \
		__unqual_scalar_typeof(*ptr) VAL;                              \
		for (;;) {                                                     \
			VAL = READ_ONCE(*__PTR);                               \
			if (cond_expr)                                         \
				break;                                         \
			cpu_relax();                                           \
			if (need_resched()) {                                  \
				cond_resched();                                \
			}                                                      \
		}                                                              \
		(typeof(*ptr)) VAL;                                            \
	})

#define smp_cond_load_relaxed_sched_delegation(ptr, cond_expr)                 \
	({                                                                     \
		typeof(ptr) __PTR = (ptr);                                     \
		__unqual_scalar_typeof(*ptr) VAL;                              \
		for (;;) {                                                     \
			VAL = READ_ONCE(*__PTR);                               \
			if (cond_expr || kthread_should_stop())                \
				break;                                         \
			cpu_relax();                                           \
			if (need_resched()) {                                  \
				cond_resched();                                \
			}                                                      \
		}                                                              \
		(typeof(*ptr)) VAL;                                            \
	})

#else
#define smp_cond_load_relaxed_sched(ptr, cond_expr)                            \
	({                                                                     \
		typeof(ptr) __PTR = (ptr);                                     \
		__unqual_scalar_typeof(*ptr) VAL;                              \
		for (;;) {                                                     \
			VAL = READ_ONCE(*__PTR);                               \
			if (cond_expr)                                         \
				break;                                         \
			cpu_relax();                                           \
		}                                                              \
		(typeof(*ptr)) VAL;                                            \
	})

#endif

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                            \
	({                                                                     \
		__unqual_scalar_typeof(*ptr) _val;                             \
		_val = smp_cond_load_relaxed_sched(ptr, cond_expr);            \
		smp_acquire__after_ctrl_dep();                                 \
		(typeof(*ptr)) _val;                                           \
	})
#endif

#define atomic_cond_read_acquire_sched(v, c)                                   \
	smp_cond_load_acquire_sched(&(v)->counter, (c))

struct shadow_stack {
	/* 
	 * lock_addr represents the lock addresses acquired by the current CPU.
	 * when acquiring multiple locks on the same CPU (level locking). Each
	 * lock address is stored in the lock_addr array. This helps when locks
	 * are released out of order [Example : Acquire(A), Acquire(B),
	 * Release(A), Release(B)]
	 *
	 * TODO (Space Optimization): Fix this to be at the base of the shadow 
	 * stack.
	 */
	void *lock_addr[9]; // Changed for komb delegation

	/*
	 * ptr points to the base of the shadow stack.
	 */
	void *ptr;
	/*
	 *
	 * Used by the combiner to identify for which CPU is the critical section 
 	 * currently executing.
 	 */
	uint32_t curr_cs_cpu;
	uint32_t prev_cs_cpu;
	uint64_t counter_val;
	uint8_t lock_type;
	struct lock_ext_node *next_node_ptr;
	void *local_shadow_stack_ptr;
	/*
	 * Create a queue which is used by the combiner to store nodes which belong to 
	 * different socket.
	 */

	struct lock_ext_node *local_queue_head;
	struct lock_ext_node *local_queue_tail;

	char dummy[128];
};

static struct task_struct *hthread;

/*
 * Used by all threads to add itself to the queue on the slowpath.
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct lock_ext_node,
				     lock_ext_nodes[MAX_NODES]);

/*
 * 8KB shadow stack used by all the threads. For waiter threads, switch to 
 * shadow stack, because of IRQs. For combiner thread, switch to shadow stack 
 * to handle nesting
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

static inline void initialize_node(struct lock_ext_node *node, u32 tail,
				   struct lock_ext_spinlock *lock)
{
	node->locked = true;
	node->completed = false;
	node->next = NULL;
	node->tail = tail;
	node->socket_id = numa_node_id();
	node->cpuid = smp_processor_id();
	node->lock = lock;
	node->lock_type = lock->type;
	node->task_struct_ptr = current;
}

/*
 * We must be able to distinguish between no-tail and the tail at 0:0,
 * therefore increment the cpu number by one.
 */
static inline __pure u32 encode_tail(int cpu, int idx)
{
	u32 tail;

	tail = (cpu + 1) << _LE_TAIL_CPU_OFFSET;
	tail |= idx << _LE_TAIL_IDX_OFFSET; /* assume < 4 */

	return tail;
}

static inline __pure struct lock_ext_node *decode_tail(u32 tail)
{
	int cpu = (tail >> _LE_TAIL_CPU_OFFSET) - 1;
	int idx = (tail & _LE_TAIL_IDX_MASK) >> _LE_TAIL_IDX_OFFSET;

#if DEBUG_KOMB
	BUG_ON(idx != 0); // TODO: Fix this for the kernel.
#endif

	return per_cpu_ptr(&lock_ext_nodes[idx], cpu);
}

static inline __pure u32 get_cpu_from_tail(u32 tail)
{
	return ((tail >> _LE_TAIL_CPU_OFFSET) - 1);
}

static __always_inline void
clear_locked_set_completed(struct lock_ext_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

static __always_inline u32 xchg_tail(struct lock_ext_spinlock *lock, u32 tail)
{
	return ((u32)xchg(&lock->tail, tail >> _LE_TAIL_OFFSET))
	       << _LE_TAIL_OFFSET;
}

static __always_inline u32 cmpxchg_tail(struct lock_ext_spinlock *lock,
					u32 tail, u32 new_tail)
{
	return ((u32)cmpxchg(&lock->tail, tail >> _LE_TAIL_OFFSET,
			     new_tail >> _LE_TAIL_OFFSET))
	       << _LE_TAIL_OFFSET;
}

/**
 * set_locked - Set the lock bit and own the lock
 * @lock: Pointer to queued spinlock structure
 *
 * *,*,0 -> *,0,1
 */
static __always_inline void set_locked(struct lock_ext_spinlock *lock)
{
	WRITE_ONCE(lock->locked, _LE_LOCKED_VAL);
}

static __always_inline void check_and_set_value(struct lock_ext_spinlock *lock,
						u8 lock_value)
{
	u32 val, new_val;

	while (true) {
		smp_cond_load_relaxed_sched(&lock->locked, !(VAL));

		val = atomic_read(&lock->val);

		if (val & _LE_LOCKED_MASK)
			continue;

		new_val = val >> _LE_LOCKED_BITS;
		new_val <<= _LE_LOCKED_BITS;
		new_val |= lock_value;

		if (atomic_cmpxchg_acquire(&lock->val, val, new_val) == val)
			return;
	}
}

static inline bool check_if_uncontended(struct lock_ext_spinlock *lock,
					u32 tail)
{
	u32 val, new_val;

	val = atomic_read(&lock->val);

	if (val & _LE_LOCKED_MASK)
		return false;

	new_val = val & _LE_LOCK_TYPE_MASK;
	/*new_val = new_val >> _LE_LOCKED_BITS;
	new_val <<= _LE_LOCKED_BITS;*/
	new_val |= _LE_LOCKED_VAL;

	return (((val & _LE_TAIL_MASK) == tail) &&
		(atomic_cmpxchg_acquire(&lock->val, val, new_val) == val));
}

__always_inline static void add_to_local_queue(struct lock_ext_node *node)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (ptr->local_queue_head == NULL) {
		ptr->local_queue_head = node;
		ptr->local_queue_tail = node;
	} else {
		ptr->local_queue_tail->next = node;
		ptr->local_queue_tail = node;
	}
}

__always_inline static bool suitable_node(struct lock_ext_node *node, struct shadow_stack *ptr)
{
	return (node != NULL && node->lock_type == ptr->lock_type);
}

__always_inline static struct lock_ext_node *
get_next_node(struct lock_ext_node *my_node, struct shadow_stack *ptr)
{
	struct lock_ext_node *curr_node, *next_node;
	int i;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (!suitable_node(next_node, ptr) ||
		    (next_node->lock_type == LOCK_KOMB &&
		     !(suitable_node(next_node->next, ptr))))
			goto next_node_null;

		if (ptr->lock_type == LOCK_DELEGATION || next_node->socket_id == numa_node_id()) {
			void *rsp_ptr = (void *)(per_cpu_ptr(&lock_ext_nodes[0],
							     next_node->cpuid)
							 ->rsp);
			prefetchw(rsp_ptr);
			for (i = 1; i < NUM_PREFETCH_LINES; i++)
				prefetchw(rsp_ptr + (64 * i));

			prefetch(next_node->next);
			return next_node;
		}

		add_to_local_queue(next_node);
		curr_node = next_node;
		next_node = curr_node->next;
	}

next_node_null:
	return next_node;
}

#pragma GCC push_options
#pragma GCC optimize("O3")

__attribute__((noipa)) noinline notrace static void
execute_cs(struct lock_ext_spinlock *lock, struct lock_ext_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct lock_ext_node *next_node = NULL;

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->curr_cs_cpu = curr_node->cpuid;

#if DEBUG_KOMB
	BUG_ON(curr_node->cpuid == smp_processor_id());
	if ((ptr->ptr - (ptr->local_shadow_stack_ptr)) > SIZE_OF_SHADOW_STACK) {
		printk(KERN_ALERT "%d %px %px\n", smp_processor_id(), ptr->ptr,
		       ptr->local_shadow_stack_ptr);
		BUG_ON(true);
	}
#endif

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &(ptr->local_shadow_stack_ptr);

	/*
	 * Make the actual switch, the pushed return address is after this
	 * function call, which we will resume execution at using the switch
	 * in unlock.
	 */
#if DEBUG_KOMB
	BUG_ON(*(u64 *)incoming_rsp_ptr == 0);
	BUG_ON(*(u64 *)outgoing_rsp_ptr == 0);
	BUG_ON(*(u64 *)incoming_rsp_ptr == 0xdeadbeef);
	BUG_ON(*(u64 *)outgoing_rsp_ptr == 0xdeadbeef);
#endif

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

#if DEBUG_KOMB
	if ((ptr->ptr - (ptr->local_shadow_stack_ptr)) > SIZE_OF_SHADOW_STACK) {
		printk(KERN_ALERT "%d %px %px\n", smp_processor_id(), ptr->ptr,
		       ptr->local_shadow_stack_ptr);
		BUG_ON(true);
	}
	BUG_ON(ptr->ptr - (ptr->local_shadow_stack_ptr) > SIZE_OF_SHADOW_STACK);
#endif

	if (lock->locked == _LE_UNLOCKED_OOO_VAL) {
		BUG_ON(true); //TODO: Fix this for delegation
		print_debug("Combiner got control back OOO unlock\n");

#if KOMB_STATS
		this_cpu_add(ooo_waiter_combined, ptr->counter_val);
		this_cpu_inc(ooo_combiner_count);
#endif

		if (ptr->curr_cs_cpu != -1) {
			print_debug("OOO waking up %d\n", ptr->curr_cs_cpu);
			curr_node = per_cpu_ptr(&lock_ext_nodes[0],
						ptr->curr_cs_cpu);
			curr_node->rsp = this_cpu_ptr(&lock_ext_nodes[0])->rsp;
			clear_locked_set_completed(curr_node);
#if DEBUG_KOMB
			BUG_ON(ptr->prev_cs_cpu != -1);
#endif
		}
		ptr->prev_cs_cpu = -1;
		ptr->curr_cs_cpu = -1;
		lock->locked = _LE_LOCKED_COMBINER_VAL;

		next_node = ptr->next_node_ptr;

		if (next_node != NULL && next_node->next != NULL) {
			execute_cs(lock, ptr->next_node_ptr);
		}
	}
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
run_combiner(struct lock_ext_spinlock *lock, struct lock_ext_node *curr_node)
{
	struct shadow_stack *ptr;
	struct lock_ext_node *next_node = curr_node->next;

	ptr = this_cpu_ptr(&local_shadow_stack);
	ptr->lock_type = LOCK_KOMB; // TODO: Fix this for nested locks

#if DEBUG_KOMB
	BUG_ON(curr_node == NULL);
#endif
	if (!suitable_node(curr_node, ptr) || !suitable_node(next_node, ptr)) {
		set_locked(lock);
		/*
		 * Make this node spin on the locked variable and then it will 
		 * become the combiner.
		 */
		curr_node->locked = false;
		smp_mb();
		return;
	}


	ptr->counter_val = 0;

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	execute_cs(lock, curr_node);

	print_debug(
		"Combiner got the control back: %d counter: %lld last_waiter: %d\n",
		smp_processor_id(), ptr->counter_val, ptr->curr_cs_cpu);

#if KOMB_STATS
	this_cpu_add(waiter_combined, ptr->counter_val);
	this_cpu_inc(combiner_count);
#endif

	if (ptr->prev_cs_cpu != -1) {
		clear_locked_set_completed(
			per_cpu_ptr(&lock_ext_nodes[0], ptr->prev_cs_cpu));
		ptr->prev_cs_cpu = -1;
	}

	next_node = ptr->next_node_ptr;

	if (ptr->local_queue_head != NULL) {
		ptr->local_queue_tail->next = next_node;
		next_node = ptr->local_queue_head;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
	}

	print_debug("After combiner %d, next node: %d\n", smp_processor_id(),
		    next_node->cpuid);

#if DEBUG_KOMB
	BUG_ON(next_node == NULL);
#endif

	set_locked(lock);

	ptr->curr_cs_cpu = -1;

	/* 
	 * Make this node spin on the locked variable and then it will become 
	 * the combiner.
	 */
	next_node->locked = false;
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static int
__lock_ext_spin_lock_longjmp(struct lock_ext_spinlock *lock, int tail,
			     register struct lock_ext_node *curr_node)
{
	register struct lock_ext_node *prev_node = NULL, *next_node = NULL;
	struct lock_ext_spinlock *parent_lock;
	int old_tail, i, j;

	uint32_t prev_cs_cpu;
	bool prev_locked_val;
	void *prev_rsp;
	uint64_t prev_counter_val;
	struct lock_ext_node *prev_next_node_ptr = NULL;
	struct lock_ext_node *prev_local_queue_head;
	struct lock_ext_node *prev_local_queue_tail;
	struct shadow_stack *ptr;

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _LE_TAIL_MASK) {
		prev_node = decode_tail(old_tail);

		prev_node->next = curr_node;

		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

		ptr = this_cpu_ptr(&local_shadow_stack);

		if (curr_node->completed) {
			for (j = 7; j >= 0; j--)
				if (ptr->lock_addr[j] != NULL)
					break;

			curr_node->count--;

			if (j >= 0) {
				parent_lock = ptr->lock_addr[j];
#if DEBUG_KOMB
				BUG_ON(parent_lock == lock);
#endif
				if (parent_lock->locked ==
				    _LE_UNLOCKED_OOO_VAL) {
					BUG_ON(true); //TODO: Fix for OOO unlock
					print_debug("Waiter unlocked OOO\n");
					return 1;
				}
			}

			for (i = 0; i < NUM_PREFETCH_LINES; i++)
				prefetchw(((char *)curr_node->rsp) + (64 * i));

			return 0;
		}
	}

	smp_cond_load_relaxed_sched(&lock->locked, !(VAL));

	if (check_if_uncontended(lock, tail)) {
		print_debug("Uncontended lock released. No one in queue: %d\n",
			    atomic_read(&lock->val));
		goto release;
	}

	check_and_set_value(lock, _LE_LOCKED_COMBINER_VAL);

	/*
	 * contended path; wait for next if not observed yet, release.
	 */
	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	next_node = curr_node->next;
#if DEBUG_KOMB
	BUG_ON(next_node == NULL);
#endif

	ptr = this_cpu_ptr(&local_shadow_stack);

	prev_cs_cpu = ptr->curr_cs_cpu;
	ptr->curr_cs_cpu = -1;

#if DEBUG_KOMB
	BUG_ON(ptr->prev_cs_cpu != -1);
#endif
	/*
	 * curr_node at the head of the queue. 
	 * Release the node and run_combiner.
	 */
	curr_node->count--;
	prev_locked_val = lock->locked;
#if DEBUG_KOMB
	//BUG_ON(prev_locked_val >= _LE_LOCKED_COMBINER_VAL);// TODO: Fix this condition
#endif
	lock->locked = _LE_LOCKED_COMBINER_VAL;
	prev_rsp = curr_node->rsp;
	curr_node->rsp = NULL;

	prev_counter_val = ptr->counter_val;
	ptr->counter_val = 0;

	prev_next_node_ptr = ptr->next_node_ptr;
	ptr->next_node_ptr = NULL;

	prev_local_queue_head = ptr->local_queue_head;
	prev_local_queue_tail = ptr->local_queue_tail;

	ptr->local_queue_head = NULL;
	ptr->local_queue_tail = NULL;

	j = 7;
	for (j = 7; j >= 0; j--)
		if (ptr->lock_addr[j] != NULL)
			break;
	j += 1;
#if DEBUG_KOMB
	BUG_ON(j >= 8 || j < 0);
#endif
	ptr->lock_addr[j] = lock;

	run_combiner(lock, next_node);

#if DEBUG_KOMB
	BUG_ON(this_cpu_ptr(&local_shadow_stack)->lock_addr[j] != lock);
#endif

	ptr->lock_addr[j] = NULL;

	ptr->next_node_ptr = prev_next_node_ptr;
	ptr->counter_val = prev_counter_val;

	ptr->local_queue_head = prev_local_queue_head;
	ptr->local_queue_tail = prev_local_queue_tail;

	curr_node->rsp = prev_rsp;

	if (lock->locked == _LE_UNLOCKED_OOO_VAL) {
		BUG_ON(true); //TODO: Fix for OOO unlock
		if (prev_cs_cpu != -1) {
			print_debug("Waking up %d\n", prev_cs_cpu);
			clear_locked_set_completed(
				per_cpu_ptr(&lock_ext_nodes[0], prev_cs_cpu));
		}
		ptr->curr_cs_cpu = -1;
	} else {
		ptr->curr_cs_cpu = prev_cs_cpu;
	}
	lock->locked = prev_locked_val;

	return 0;
release:
	/* 
	 * release the node
	 */
	curr_node->count--;
	return 0;
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace static int
__lock_ext_spin_lock_slowpath(struct lock_ext_spinlock *lock)
{
	struct lock_ext_node *curr_node;
	int tail, idx;

	curr_node = this_cpu_ptr(&lock_ext_nodes[0]);
	idx = curr_node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	initialize_node(curr_node, tail, lock);

	return __lock_ext_spin_lock_longjmp(lock, tail, curr_node);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static struct lock_ext_node *
run_delegation(struct lock_ext_spinlock *lock, struct lock_ext_node *curr_node)
{
	struct shadow_stack *ptr;
	int j;
#if DEBUG_KOMB
	BUG_ON(curr_node == NULL);
#endif
	ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->counter_val = 0;
	ptr->prev_cs_cpu = -1;
	ptr->curr_cs_cpu = -1;
	ptr->next_node_ptr = NULL;
	ptr->local_queue_head = NULL;
	ptr->local_queue_tail = NULL;
	ptr->curr_cs_cpu = curr_node->cpuid;
	j = 7;
	for (j = 7; j >= 0; j--)
		if (ptr->lock_addr[j] != NULL)
			break;
	j += 1;
#if DEBUG_KOMB
	BUG_ON(j >= 8 || j < 0);
#endif
	ptr->lock_addr[j] = lock;

	print_debug("Combiner %d giving control to %d\n", smp_processor_id(),
		    curr_node->cpuid);

	ptr->lock_type = LOCK_DELEGATION;

	execute_cs(curr_node->lock, curr_node);

	ptr->lock_addr[j] = NULL;
	//TODO: Revert state for nested locks

	print_debug(
		"Combiner got the control back: %d counter: %lld last_waiter: %d\n",
		smp_processor_id(), ptr->counter_val, ptr->curr_cs_cpu);

#if KOMB_STATS
	this_cpu_add(waiter_combined, ptr->counter_val);
	this_cpu_inc(combiner_count);
#endif

#if DEBUG_KOMB
	BUG_ON(ptr->prev_cs_cpu == -1 || ptr->prev_cs_cpu == KOMB_CPU);
#endif

	ptr->curr_cs_cpu = -1;
	return per_cpu_ptr(&lock_ext_nodes[0], ptr->prev_cs_cpu);
}
#pragma GCC pop_options

int komb_delegation_thread(void *args)
{
	int my_tail, old_tail;
	struct lock_ext_spinlock *lock;
	struct shadow_stack *ptr;
	struct lock_ext_node *my_node, *prev_node;

	ptr = this_cpu_ptr(&local_shadow_stack);
	my_tail = encode_tail(KOMB_CPU, 0);
	lock = (struct lock_ext_spinlock *)
		       ptr->lock_addr[8]; //TODO: Fix it for multiple locks
	my_node = this_cpu_ptr(&lock_ext_nodes[0]);

	while (!kthread_should_stop()) {
		initialize_node(my_node, my_tail, lock);
		my_node->rsp = (void *)0xdeadbeef;
		my_node->lock_type = -1;

		smp_cond_load_relaxed_sched_delegation(
			&lock->type, (VAL == LOCK_DELEGATION));

		if (lock->type != LOCK_DELEGATION) {
#if DEBUG_KOMB
			BUG_ON(my_node->next != NULL);
#endif
			if (kthread_should_stop())
				break;
			else
				continue;
		}

		old_tail = xchg_tail(lock, my_tail);

		if (old_tail & _LE_TAIL_MASK) {
			prev_node = decode_tail(old_tail);

			prev_node->next = my_node;

			smp_cond_load_relaxed_sched(&my_node->locked, !(VAL));

#if DEBUG_KOMB
			BUG_ON(my_node->completed == true);
#endif
		}

		check_and_set_value(lock, _LE_LOCKED_DELEGATION_VAL);

		while (true) {
			smp_cond_load_relaxed_sched_delegation(&my_node->next,
							       (VAL));
			if (kthread_should_stop()) {
#if DEBUG_KOMB
				BUG_ON(my_node->next != NULL);
#endif
				break;
			}
#if DEBUG_KOMB
			BUG_ON(my_node->next == NULL);
			BUG_ON(my_node->next == my_node);
#endif
			if (my_node->next->lock_type != LOCK_DELEGATION) {
				my_node->next->locked =
					false; // Pass lock to next node
				lock->locked = 0; // Unlock the lock
				break;
			}

			print_debug("Running combiner with node from: %d\n",
				    my_node->next->cpuid);
			prev_node = run_delegation(lock, my_node->next);
			my_node->next = NULL;
			print_debug("Got control back prev node: %d\n",
				    prev_node->cpuid);

#if DEBUG_KOMB
			BUG_ON(prev_node == NULL);
#endif

			my_node->next = prev_node->next;
			if (my_node->next == NULL) {
				if (cmpxchg_tail(lock, prev_node->tail,
						 my_tail) != prev_node->tail) {
					print_debug(
						"Someone else joined the queue: %d\n",
						prev_node->cpuid);
					smp_cond_load_relaxed_sched(
						&prev_node->next, (VAL));
					my_node->next = prev_node->next;
				}
			}
			clear_locked_set_completed(prev_node);
			if (need_resched())
				cond_resched();
		}
	}

	cond_resched();

	__set_current_state(TASK_RUNNING);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);

	return 0;
}

/*
 * Public API
 */

void lock_ext_init(struct lock_ext_spinlock *lock)
{
	int i, j;

	lock->type = LOCK_KOMB;

	for_each_possible_cpu (i) {
		void *stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		struct shadow_stack *ptr = per_cpu_ptr(&local_shadow_stack, i);

#if DEBUG_KOMB
		BUG_ON(stack_ptr == NULL);
#endif

		ptr->ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		for (j = 0; j < 8; j++)
			ptr->lock_addr[j] = 0;
		ptr->lock_addr[8] = lock; //TODO: Added for delegation. Fix it
		ptr->local_shadow_stack_ptr =
			stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = -1;
		ptr->counter_val = 0;
		ptr->next_node_ptr = NULL;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
	}

	hthread = kthread_create(komb_delegation_thread, NULL,
				 "komb_delegation_thread");
	kthread_bind(hthread, KOMB_CPU);
	if (hthread)
		wake_up_process(hthread);
	else
		printk(KERN_ALERT "failed to create komb delegation thread\n");
}

void lock_ext_free(void)
{
	int ret = kthread_stop(hthread);
	uint32_t i;

	print_debug("lock ext delegate thread exit\n");

	if (ret)
		printk(KERN_ALERT "komb delegate thread returned error %d\n",
		       ret);

	for_each_possible_cpu (i) {
		vfree(per_cpu_ptr(&local_shadow_stack, i)->ptr -
		      SIZE_OF_SHADOW_STACK);
	}
}

void lock_ext_spin_lock_init(struct lock_ext_spinlock *lock)
{
	atomic_set(&lock->val, 0);
}

__attribute__((noipa)) noinline notrace static void *get_shadow_stack_ptr(void)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	return &(ptr->local_shadow_stack_ptr);
}

__attribute__((noipa)) noinline notrace static struct lock_ext_node *
get_lock_ext_node(void)
{
	return this_cpu_ptr(&lock_ext_nodes[0]);
}

/* 
 * When the following function is called, the compiler in caller frame does
 * this sequence:
 *
 * push caller-saved-regs			(rdi, rsi, rdx, rcx, ...)
 * ...<prepare arguments>...
 * .- push eip+1				(eip + 1 == pop insn)
 * callq |
 * `- jmp lock_ext_spin_lock
 * pop caller-saved-regs
 *
 * This means that the following function must in no way modify the the rsp
 * before it is written to the task_frame, otherwise the top of the stack
 * (rsp) would no longer point to the return address. To do this, emit a
 * call in assembly and pass the top of the stack as an argument so that
 * we can use the stack while preserving the semantics of the combiner.
 *
 * NOTE: One advantage of offloading the caller-saved register spill to the
 * compiler is that it only saves the caller-saved registers in use in the
 * caller frame. If we were to inline this whole logic in assembly, we wouldn't
 * know the registers in use and would have to save all possible caller-saved
 * registers.
 */

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace void
lock_ext_spin_lock_slowpath(struct lock_ext_spinlock *lock)
{
	register int ret_val;
	/* 
	 * We must save the callee-saved registers now, since we will resume
	 * execution at the return address currently at top of stack. If we
	 * defer saving to when context switch happens later, the C function
	 * may have saved callee-saved registers and will restore them before
	 * returning.
	 *
	 * Due to unstructured control flow, do the saving now.
	 *
	 * NOTE: When we touch the stack inside this function, so a 16-byte
	 * stack's alignment will change to 8-byte alignment when callq pushes
	 * our return address, but Linux only assumes 8-byte stack alignment,
	 * so it isn't a problem for us.
	 */
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
		     : "i"(get_lock_ext_node),
		       "i"(offsetof(struct lock_ext_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");

	ret_val = __lock_ext_spin_lock_slowpath(lock);

	if (ret_val) {
		asm volatile("callq %P0\n"
			     "movq (%%rax), %%rsp\n"
			     "popq %%r15\n"
			     "popq %%r14\n"
			     "popq %%r13\n"
			     "popq %%r12\n"
			     "popq %%rbx\n"
			     "popq %%rbp\n"
			     "retq\n"
			     :
			     : "i"(get_shadow_stack_ptr)
			     : "memory");
	} else {
		asm volatile("callq %P0\n"
			     "movq %%rsp, (%%rax)\n"
			     :
			     : "i"(get_shadow_stack_ptr)
			     : "memory");
		asm volatile("callq %P0\n"
			     "movq %c1(%%rax), %%rsp\n"
			     :
			     : "i"(get_lock_ext_node),
			       "i"(offsetof(struct lock_ext_node, rsp))
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
}
#pragma GCC pop_options

__attribute__((noipa)) noinline notrace void
lock_ext_spin_lock(struct lock_ext_spinlock *lock)
{
	u32 val;
	struct lock_ext_node *curr_node = NULL;
	struct shadow_stack *ptr;
	uint32_t curr_cpuid, prev_cpuid;
	struct lock_ext_node *prev_node, *next_node;

	u32 tail, idx, old_tail;
	void *prev_rsp;

	/*
	 * Fastpath
	 */

	val = atomic_cmpxchg_acquire(
		&lock->val, 0,
		_LE_LOCKED_VAL); //TODO: Fix this, will not work with LOCK_KOMB
	if (val == 0)
		return;

	/*
	 * Slowpath
	 */
	curr_node = this_cpu_ptr(&lock_ext_nodes[0]);
#if DEBUG_KOMB
	BUG_ON(curr_node == NULL);
#endif
	if (lock->type == LOCK_QSPINLOCK) {
		idx = curr_node->count++;

		if (unlikely(idx >= MAX_NODES)) {
			while (!lock_ext_spin_trylock(lock))
				cpu_relax();
			goto irq_release;
		}

		curr_node += idx;
		tail = encode_tail(smp_processor_id(), idx);

		barrier();

		prev_rsp = curr_node->rsp;
		initialize_node(curr_node, tail, lock);
		curr_node->socket_id = IRQ_NUMA_NODE;
		curr_node->rsp = (void *)0xdeadbeef;

		old_tail = xchg_tail(lock, tail);

		if (old_tail & _LE_TAIL_MASK) {
			prev_node = decode_tail(old_tail);
			WRITE_ONCE(prev_node->next, curr_node);

			print_debug("IRQ going to waiting for lock\n");
			smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));
		}

		curr_node->rsp = prev_rsp;

		print_debug("IRQ spinning on the locked field\n");

		smp_cond_load_relaxed_sched(&lock->locked, !(VAL));

		if (check_if_uncontended(lock, tail))
			goto irq_release;

		check_and_set_value(lock, _LE_LOCKED_VAL);

		print_debug("IRQ got the lock\n");

		smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
		next_node = curr_node->next;

#if DEBUG_KOMB
		BUG_ON(next_node == NULL);
#endif

		WRITE_ONCE(next_node->locked, false);

		print_debug("IRQ passing lock next node: %d\n",
			    next_node->cpuid);

	irq_release:
		curr_node = this_cpu_ptr(&lock_ext_nodes[0]);
		curr_node->count--;
		return;
	} else {
		lock_ext_spin_lock_slowpath(lock);

		ptr = this_cpu_ptr(&local_shadow_stack);
		curr_cpuid = ptr->curr_cs_cpu;
		if (curr_cpuid != -1) {
			curr_node = per_cpu_ptr(&lock_ext_nodes[0], curr_cpuid);
			if (curr_node->lock == lock) {
#if DEBUG_KOMB
				BUG_ON(!(lock->locked ==
						 _LE_LOCKED_COMBINER_VAL ||
					 lock->locked ==
						 _LE_LOCKED_DELEGATION_VAL));
#endif
				next_node = get_next_node(curr_node, ptr);
				ptr->next_node_ptr = next_node;
			}

			prev_cpuid = ptr->prev_cs_cpu;
			if (prev_cpuid != -1) {
				prev_node = per_cpu_ptr(&lock_ext_nodes[0],
							prev_cpuid);

#if DEBUG_KOMB
				BUG_ON(prev_node->lock != lock);
#endif
				//print_debug("Waking up prev waiter: %d\n",
				//	    prev_cpuid);
				clear_locked_set_completed(prev_node);
				ptr->prev_cs_cpu = -1;
			}
		}
	}
}
EXPORT_SYMBOL_GPL(lock_ext_spin_lock);

struct task_struct *lock_ext_get_current(spinlock_t *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	int j, my_idx;

	j = 0;
	my_idx = -1;

	for (j = 0; j < 8; j++) {
		if (ptr->lock_addr[j] == lock) {
#if DEBUG_KOMB
			BUG_ON(ptr->curr_cs_cpu < 0);
#endif
			return per_cpu_ptr(&lock_ext_nodes[0], ptr->curr_cs_cpu)
				->task_struct_ptr;
		}
	}

	return current;
}

void lock_ext_set_current_state(spinlock_t *lock, unsigned int state)
{
	WRITE_ONCE(lock_ext_get_current(lock)->__state, state);
	smp_mb();
}

/* 
 * No IPA is even more relevant here, because even if assembly is removed,
 * compiler shouldn't assume which caller-saved registers are in use inside
 * the function. When we jump from the spin loop to the return address after
 * the unlock function, we have caller-saved registers that are completely
 * inconsistent, because we only restored the callee-saved registers.
 */
__attribute__((noipa)) noinline notrace void
lock_ext_spin_unlock(struct lock_ext_spinlock *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
	int from_cpuid = ptr->curr_cs_cpu;
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct lock_ext_node *curr_node;
	struct lock_ext_node *next_node;
	uint64_t counter;
	int j, max_idx, my_idx;
	void *temp_lock_addr;

	j = 0;
	max_idx = -1;
	my_idx = -1;

	for (j = 0; j < 8; j++) {
		temp_lock_addr = ptr->lock_addr[j];
		if (temp_lock_addr != NULL)
			max_idx = j;
		if (temp_lock_addr == lock)
			my_idx = j;
		if (temp_lock_addr == NULL)
			break;
	}

	if (my_idx == -1) {
		if (lock->locked == _LE_LOCKED_VAL) {
			//print_debug("Lock released on fast path\n");
			lock->locked = false;
		} else if (lock->locked == _LE_LOCKED_COMBINER_VAL ||
			   lock->locked == _LE_LOCKED_DELEGATION_VAL) {
			print_debug("OOO unlock: %d\n", lock->locked);
			BUG_ON(true); //TODO: Fix this for OOO locks
#if KOMB_STATS
			this_cpu_inc(ooo_unlocks);
#endif
			lock->locked = _LE_UNLOCKED_OOO_VAL;
			print_debug("OOO unlock\n");
		} else
			BUG_ON(true);
		return;
	}

#if DEBUG_KOMB
	BUG_ON(!(lock->locked == _LE_LOCKED_COMBINER_VAL ||
		 lock->locked == _LE_LOCKED_DELEGATION_VAL));

	BUG_ON(from_cpuid == -1);

	if (max_idx < 0) {
		BUG_ON(true);
	}

#endif
	if (my_idx < max_idx) {
#if KOMB_STATS
		this_cpu_inc(ooo_unlocks);
#endif

		lock->locked = _LE_UNLOCKED_OOO_VAL;
		return;
	}

	curr_node = per_cpu_ptr(&lock_ext_nodes[0], from_cpuid);

	next_node = ptr->next_node_ptr;

	counter = ptr->counter_val;

	if (!suitable_node(next_node, ptr) ||
	    (next_node->lock_type == LOCK_KOMB &&
	     !suitable_node(next_node->next, ptr)) ||
	    counter >= komb_batch_size || need_resched()) {
		incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = curr_node->cpuid;

	} else {
		ptr->curr_cs_cpu = next_node->cpuid;
		ptr->prev_cs_cpu = curr_node->cpuid;
		incoming_rsp_ptr = &(next_node->rsp);
		ptr->counter_val = counter + 1;
		//prefetchw(((char*)curr_node) + 195);
		print_debug("Jumping to the next waiter: %d\n",
			    next_node->cpuid);
	}

	/*
	 * Komb node still active here, because cpu (from_cpuid) still spinning.
	 */
	outgoing_rsp_ptr = &(curr_node->rsp);

#if DEBUG_KOMB
	BUG_ON(*(u64 *)incoming_rsp_ptr == 0);
	BUG_ON(*(u64 *)outgoing_rsp_ptr == 0);
	BUG_ON(*(u64 *)incoming_rsp_ptr == 0xdeadbeef);
	BUG_ON(*(u64 *)outgoing_rsp_ptr == 0xdeadbeef);
#endif

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	ptr = this_cpu_ptr(&local_shadow_stack);
	return;
}
EXPORT_SYMBOL_GPL(lock_ext_spin_unlock);

__always_inline int lock_ext_spin_trylock(struct lock_ext_spinlock *lock)
{
	u32 val;

	// Qspinlock Fastpath

	val = atomic_cmpxchg_acquire(&lock->val, 0, _LE_LOCKED_VAL);
	if (val == 0)
		return true;

	return false;
}

void lock_ext_spin_lock_nested(struct lock_ext_spinlock *lock, int level)
{
	lock_ext_spin_lock(lock);
}
EXPORT_SYMBOL_GPL(lock_ext_spin_lock_nested);

void lock_ext_assert_spin_locked(struct lock_ext_spinlock *lock)
{
	BUG_ON(!lock_ext_spin_is_locked(lock));
}

__always_inline int lock_ext_spin_is_locked(struct lock_ext_spinlock *lock)
{
	return atomic_read(&lock->val);
}
EXPORT_SYMBOL(lock_ext_spin_is_locked);

__always_inline int lock_ext_spin_is_contended(struct lock_ext_spinlock *lock)
{
	return atomic_read(&lock->val) & ~_LE_LOCKED_MASK;
}

__always_inline int lock_ext_spin_value_unlocked(struct lock_ext_spinlock lock)
{
	return !atomic_read(&lock.val);
}

#if KOMB_STATS
void lock_ext_print_stats(void)
{
	int i;
	uint64_t total_counters[5] = { 0 };
	printk(KERN_ALERT "======== KOMB spinlock stats ========\n");
	for_each_online_cpu (i) {
		total_counters[0] += per_cpu(combiner_count, i);
		total_counters[1] += per_cpu(waiter_combined, i);
		total_counters[2] += per_cpu(ooo_unlocks, i);
		total_counters[3] += per_cpu(ooo_combiner_count, i);
		total_counters[4] += per_cpu(ooo_waiter_combined, i);
	}

	printk(KERN_ALERT "Combiner_count: %lld\n", total_counters[0]);
	printk(KERN_ALERT "waiter_combined: %lld\n", total_counters[1]);
	printk(KERN_ALERT "ooo_unlocks: %lld\n", total_counters[2]);
	printk(KERN_ALERT "ooo_combiner_count: %lld\n", total_counters[3]);
	printk(KERN_ALERT "ooo_waiter_combined: %lld\n", total_counters[4]);
}
#endif
