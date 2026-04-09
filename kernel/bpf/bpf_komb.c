// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <asm-generic/qspinlock.h>
#include <linux/sched.h>
#include <linux/combiner.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>

#include <linux/percpu-defs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

#define smp_cond_load_relaxed_sched(ptr, cond_expr) \
	({                                          \
		typeof(ptr) __PTR = (ptr);          \
		__unqual_scalar_typeof(*ptr) VAL;   \
		for (;;) {                          \
			VAL = READ_ONCE(*__PTR);    \
			if (cond_expr)              \
				break;              \
			cpu_relax();                \
		}                                   \
		(typeof(*ptr))VAL;                  \
	})
#endif

#ifndef smp_cond_load_acquire_sched
#define smp_cond_load_acquire_sched(ptr, cond_expr)                 \
	({                                                          \
		__unqual_scalar_typeof(*ptr) _val;                  \
		_val = smp_cond_load_relaxed_sched(ptr, cond_expr); \
		smp_acquire__after_ctrl_dep();                      \
		(typeof(*ptr))_val;                                 \
	})
#endif

#define atomic_cond_read_acquire_sched(v, c) \
	smp_cond_load_acquire_sched(&(v)->counter, (c))

struct shadow_stack {
	void *lock_addr;

	/*
	 * ptr points to the base of the shadow stack.
	 */
	void *ptr;
	/*
 * Used by the combiner to identify for which CPU is the critical section 
 * currently executing.
 */
	uint32_t curr_cs_cpu;
	uint32_t prev_cs_cpu;
	uint64_t counter_val;
	struct komb_node *next_node_ptr;
	void *local_shadow_stack_ptr;
	/*
 * Create a queue which is used by the combiner to store nodes which belong to 
 * different socket.
 */

	struct komb_node *local_queue_head;
	struct komb_node *local_queue_tail;

	char dummy[128];
};

/*
 * Used by all threads to add itself to the queue on the slowpath.
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct komb_node, komb_nodes[MAX_NODES]);

/*
 * 8KB shadow stack used by all the threads. For waiter threads, switch to 
 * shadow stack, because of IRQs. For combiner thread, switch to shadow stack 
 * to handle nesting
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct shadow_stack, local_shadow_stack);

#define _Q_LOCKED_PENDING_MASK (_Q_LOCKED_MASK | _Q_PENDING_MASK)
#define _Q_LOCKED_COMBINER_VAL 3
#define _Q_LOCKED_IRQ_VAL 15 // Lock Stealing by IRQ
/*
 * We must be able to distinguish between no-tail and the tail at 0:0,
 * therefore increment the cpu number by one.
 */
static inline __pure u32 encode_tail(int cpu, int idx)
{
	u32 tail;

	tail = (cpu + 1) << _Q_TAIL_CPU_OFFSET;
	tail |= idx << _Q_TAIL_IDX_OFFSET; /* assume < 4 */

	return tail;
}

static inline __pure struct komb_node *decode_tail(u32 tail)
{
	int cpu = (tail >> _Q_TAIL_CPU_OFFSET) - 1;
	int idx = (tail & _Q_TAIL_IDX_MASK) >> _Q_TAIL_IDX_OFFSET;

	return per_cpu_ptr(&komb_nodes[idx], cpu);
}

inline __pure u32 get_cpu_from_tail(u32 tail)
{
	return ((tail >> _Q_TAIL_CPU_OFFSET) - 1);
}

__always_inline void clear_locked_set_completed(struct komb_node *lock)
{
	WRITE_ONCE(lock->locked_completed, 1);
}

__always_inline void clear_pending_set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked_pending, _Q_LOCKED_VAL);
}

static __always_inline u32 xchg_tail(struct qspinlock *lock, u32 tail)
{
	return ((u32)xchg(&lock->tail, tail >> _Q_TAIL_OFFSET))
	       << _Q_TAIL_OFFSET;
}

__always_inline u32 cmpxchg_tail(struct qspinlock *lock, u32 tail, u32 new_tail)
{
	return ((u32)cmpxchg(&lock->tail, tail >> _Q_TAIL_OFFSET,
			     new_tail >> _Q_TAIL_OFFSET))
	       << _Q_TAIL_OFFSET;
}

/**
 * clear_pending - clear the pending bit.
 * @lock: Pointer to queued spinlock structure
 *
 * *,1,* -> *,0,*
 */
static __always_inline void clear_pending(struct qspinlock *lock)
{
	atomic_andnot(_Q_PENDING_VAL, &lock->val);
}

/**
 * set_locked - Set the lock bit and own the lock
 * @lock: Pointer to queued spinlock structure
 *
 * *,*,0 -> *,0,1
 */
static __always_inline void set_locked(struct qspinlock *lock)
{
	WRITE_ONCE(lock->locked, _Q_LOCKED_VAL);
}

static __always_inline void check_and_set_combiner(struct qspinlock *lock)
{
	u32 val, new_val;

	if (lock->locked > 0) {
		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_COMBINER_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) ==
			    val)
				return;
		}
	}
	WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
}

/**
 * queued_fetch_set_pending_acquire - fetch the whole lock value and set pending
 * @lock : Pointer to queued spinlock structure
 * Return: The previous lock value
 *
 * *,*,* -> *,1,*
 */
static __always_inline u32 komb_fetch_set_pending_acquire(struct qspinlock *lock)
{
	return atomic_fetch_or_acquire(_Q_PENDING_VAL, &lock->val);
}

__always_inline static void add_to_local_queue(struct komb_node *node)
{
	//printk(KERN_ALERT "%d Move node to local queue: %d\n",
	//     smp_processor_id(), node->cpuid);

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (ptr->local_queue_head == NULL) {
		ptr->local_queue_head = node;
		ptr->local_queue_tail = node;
	} else {
		ptr->local_queue_tail->next = node;
		ptr->local_queue_tail = node;
	}
}

__always_inline static struct komb_node *
get_next_node(struct komb_node *my_node)
{
	int i;
	struct komb_node *curr_node, *next_node;

	curr_node = my_node;
	next_node = curr_node->next;

	while (true) {
		if (next_node == NULL || next_node->next == NULL)
			goto next_node_null;

		if (next_node->socket_id == numa_node_id()) {
			void *rsp_ptr = (void *)(per_cpu_ptr(&komb_nodes[0],
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
execute_cs(struct qspinlock *lock, struct komb_node *curr_node)
{
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct komb_node *next_node = NULL;

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->curr_cs_cpu = curr_node->cpuid;

	incoming_rsp_ptr = &(curr_node->rsp);
	outgoing_rsp_ptr = &(ptr->local_shadow_stack_ptr);

	/*
	 * Make the actual switch, the pushed return address is after this
	 * function call, which we will resume execution at using the switch
	 * in unlock.
	 */
	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static void
run_combiner(struct qspinlock *lock, struct komb_node *curr_node)
{
	struct shadow_stack *ptr;
	struct komb_node *next_node = curr_node->next;

	if (next_node == NULL) {
		set_locked(lock);
		/*
		 * Make this node spin on the locked variable and then it will 
		 * become the combiner.
		 */
		curr_node->locked = false;
		smp_mb();
		return;
	}

	ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->counter_val = 0;

	execute_cs(lock, curr_node);

	if (ptr->prev_cs_cpu != -1) {
		clear_locked_set_completed(
			per_cpu_ptr(&komb_nodes[0], ptr->prev_cs_cpu));
		ptr->prev_cs_cpu = -1;
	}

	next_node = ptr->next_node_ptr;

	if (ptr->local_queue_head != NULL) {
		ptr->local_queue_tail->next = next_node;
		next_node = ptr->local_queue_head;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
	}

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
__komb_spin_lock_longjmp(struct qspinlock *lock, int tail,
			 register struct komb_node *curr_node)
{
	register struct komb_node *prev_node = NULL, *next_node = NULL;
	struct qspinlock *parent_lock;
	int old_tail, val, j;
	int i;

	/*
	 * TODO: Make sure these variables are stored on the combiner stack
	 * so that they can be restored later.
	 * These variables are needed when there is a waiter or a combiner
	 * running within a combiner (For same or different lock).
	 */
	struct shadow_stack *ptr;

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);

		prev_node->next = curr_node;

		smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

		/*while (READ_ONCE(curr_node->locked)) {
			cpu_relax();
			count++;
			if (count == INT_MAX)
				BUG_ON(true);
		}*/

		ptr = this_cpu_ptr(&local_shadow_stack);

		if (curr_node->completed) {
			curr_node->count--;

			for (i = 0; i < NUM_PREFETCH_LINES; i++)
				prefetchw(((char *)curr_node->rsp) + (64 * i));

			return 0;
		}
	}

	/*
	 * we're at the head of the waitqueue, wait for the owner & pending to
	 * go away.
	 *
	 * *,x,y -> *,0,0
	 *
	 * this wait loop must use a load-acquire such that we match the
	 * store-release that clears the locked bit and create lock
	 * sequentiality; this is because the set_locked() function below
	 * does not imply a full barrier.
	 *
	 */

	val = atomic_cond_read_acquire(&lock->val,
				       !(VAL & _Q_LOCKED_PENDING_MASK));

	/*
	 * claim the lock:
	 *
	 * n,0,0 -> 0,0,1 : lock, uncontended
	 * *,*,0 -> *,*,1 : lock, contended
	 *
	 * If the queue head is the only one in the queue (lock value == tail)
	 * and nobody is pending, clear the tail code and grab the lock.
	 * Otherwise, we only need to grab the lock.
	 */

	if (((val & _Q_TAIL_MASK) == tail) &&
	    atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL))
		goto release; /* No contention */

	/* Either somebody is queued behind us or _Q_PENDING_VAL is set */
	check_and_set_combiner(lock);

	/*
	 * contended path; wait for next if not observed yet, release.
	 */
	smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
	next_node = curr_node->next;

	ptr = this_cpu_ptr(&local_shadow_stack);

	ptr->curr_cs_cpu = -1;

	/*
	 * curr_node at the head of the queue. 
	 * Release the node and run_combiner.
	 */
	curr_node->count--;
	lock->locked = _Q_LOCKED_COMBINER_VAL;

	curr_node->rsp = NULL;
	ptr->counter_val = 0;
	ptr->next_node_ptr = NULL;
	ptr->local_queue_head = NULL;
	ptr->local_queue_tail = NULL;

	ptr->lock_addr = lock;

	run_combiner(lock, next_node);

	ptr->lock_addr = NULL;

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
__komb_spin_lock_slowpath(struct qspinlock *lock)
{
	struct komb_node *curr_node;
	int tail, idx;

	curr_node = this_cpu_ptr(&komb_nodes[0]);
	idx = curr_node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	/*
	 * Initialize curr_node
	 */
	curr_node->locked = true;
	curr_node->completed = false;
	curr_node->next = NULL;
	curr_node->tail = tail;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;

	return __komb_spin_lock_longjmp(lock, tail, curr_node);
}

/*
 * Public API
 */

void komb_init(void)
{
	int i, j;
	for_each_possible_cpu(i) {
		void *stack_ptr = vzalloc(SIZE_OF_SHADOW_STACK);
		struct shadow_stack *ptr = per_cpu_ptr(&local_shadow_stack, i);

		ptr->ptr = stack_ptr + SIZE_OF_SHADOW_STACK;
		ptr->lock_addr = NULL;
		ptr->local_shadow_stack_ptr =
			stack_ptr + SIZE_OF_SHADOW_STACK - 8;
		ptr->curr_cs_cpu = -1;
		ptr->prev_cs_cpu = -1;
		ptr->counter_val = 0;
		ptr->next_node_ptr = NULL;
		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;

	}
}

void komb_free(void)
{
	int i;
	for_each_possible_cpu(i) {
		vfree(per_cpu_ptr(&local_shadow_stack, i)->ptr -
		      SIZE_OF_SHADOW_STACK);
	}
}

void komb_spin_lock_init(struct qspinlock *lock)
{
	atomic_set(&lock->val, 0);
}

__attribute__((noipa)) noinline notrace static void *get_shadow_stack_ptr(void)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	return &(ptr->local_shadow_stack_ptr);
}

__attribute__((noipa)) noinline notrace static struct komb_node *
get_komb_node(void)
{
	return this_cpu_ptr(&komb_nodes[0]);
}

/* 
 * When the following function is called, the compiler in caller frame does
 * this sequence:
 *
 * push caller-saved-regs			(rdi, rsi, rdx, rcx, ...)
 * ...<prepare arguments>...
 * .- push eip+1				(eip + 1 == pop insn)
 * callq |
 * `- jmp komb_spin_lock
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
komb_spin_lock_slowpath(struct qspinlock *lock)
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
		     : "i"(get_komb_node), "i"(offsetof(struct komb_node, rsp))
		     : "memory");
	asm volatile("callq %P0\n"
		     "movq (%%rax), %%rsp\n"
		     :
		     : "i"(get_shadow_stack_ptr)
		     : "memory");

	__komb_spin_lock_slowpath(lock);

		asm volatile("callq %P0\n"
			     "movq %%rsp, (%%rax)\n"
			     :
			     : "i"(get_shadow_stack_ptr)
			     : "memory");
		asm volatile("callq %P0\n"
			     "movq %c1(%%rax), %%rsp\n"
			     :
			     : "i"(get_komb_node),
			       "i"(offsetof(struct komb_node, rsp))
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

__attribute__((noipa)) noinline notrace void
komb_spin_lock(struct qspinlock *lock)
{
	u32 val, cnt;
	struct komb_node *curr_node = NULL;
	struct shadow_stack *ptr;
	uint32_t curr_cpuid, prev_cpuid;
	struct komb_node *prev_node, *next_node;

	/*
	 * Fastpath
	 */

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		return;

	/*
	 * Mid-path
	 *
	 * Wait for in-progress pending->locked hand-overs with a bounded
	 * number of spins so that we guarantee forward progress.
	 *
	 * 0,1,0 -> 0,0,1
	 */
	if (val == _Q_PENDING_VAL) {
		cnt = _Q_PENDING_LOOPS;
		val = atomic_cond_read_relaxed(
			&lock->val, (VAL != _Q_PENDING_VAL) || !cnt--);
	}

	/*
	 * If we observe any contention; queue.
	 */
	if (val & ~_Q_LOCKED_MASK ||
	    (val & _Q_LOCKED_MASK) == _Q_LOCKED_IRQ_VAL)
		goto queue;

	/*
     	* trylock || pending
     	*
     	* 0,0,* -> 0,1,* -> 0,0,1 pending, trylock
     	*/
	val = komb_fetch_set_pending_acquire(lock);

	/*
     	* If we observe contention, there is a concurrent locker.
     	*
     	* Undo and queue; our setting of PENDING might have made the
     	* n,0,0 -> 0,0,0 transition fail and it will now be waiting
     	* on @next to become !NULL.
     	*/
	if (unlikely(val & ~_Q_LOCKED_MASK)) {
		/* Undo PENDING if we set it. */
		if (!(val & _Q_PENDING_MASK))
			clear_pending(lock);

		goto queue;
	}

	/*
     	* We're pending, wait for the owner to go away.
     	*
     	* 0,1,1 -> 0,1,0
     	*
     	* this wait loop must be a load-acquire such that we match the
     	* store-release that clears the locked bit and create lock
     	* sequentiality; this is because not all
     	* clear_pending_set_locked() implementations imply full
     	* barriers.
     	*/
	if (val & _Q_LOCKED_MASK)
		atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_MASK));

	/*
     	* take ownership and clear the pending bit.
     	*
     	* 0,1,0 -> 0,0,1
     	*/
	clear_pending_set_locked(lock);
	return;

	/*
	 * End of pending bit optimistic spinning and beginning of MCS
	 * queuing.
	 */
queue:

	curr_node = this_cpu_ptr(&komb_nodes[0]);
		komb_spin_lock_slowpath(lock);

		ptr = this_cpu_ptr(&local_shadow_stack);
		curr_cpuid = ptr->curr_cs_cpu;
		if (curr_cpuid != -1) {
			curr_node = per_cpu_ptr(&komb_nodes[0], curr_cpuid);
			if (curr_node->lock == lock) {
				next_node = get_next_node(curr_node);
				ptr->next_node_ptr = next_node;
			}

			prev_cpuid = ptr->prev_cs_cpu;
			if (prev_cpuid != -1) {
				prev_node =
					per_cpu_ptr(&komb_nodes[0], prev_cpuid);

				print_debug("Waking up prev waiter: %d\n",
					    prev_cpuid);
				clear_locked_set_completed(prev_node);
				ptr->prev_cs_cpu = -1;

			}
		}
}
EXPORT_SYMBOL_GPL(komb_spin_lock);

/* 
 * No IPA is even more relevant here, because even if assembly is removed,
 * compiler shouldn't assume which caller-saved registers are in use inside
 * the function. When we jump from the spin loop to the return address after
 * the unlock function, we have caller-saved registers that are completely
 * inconsistent, because we only restored the callee-saved registers.
 */
__attribute__((noipa)) noinline notrace void
komb_spin_unlock(struct qspinlock *lock)
{
	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
	int from_cpuid = ptr->curr_cs_cpu;
	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
	struct komb_node *curr_node;
	struct komb_node *next_node;
	uint64_t counter;
	if (ptr->lock_addr == NULL) {
		lock->locked = false;
		return;
	}

	curr_node = per_cpu_ptr(&komb_nodes[0], from_cpuid);

	next_node = ptr->next_node_ptr;

	counter = ptr->counter_val;

	if (next_node == NULL || next_node->next == NULL ||
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

	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);
	return;
}
EXPORT_SYMBOL_GPL(komb_spin_unlock);
