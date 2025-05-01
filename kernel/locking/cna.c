/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/topology.h>
#include <linux/sched/clock.h>
#include <linux/moduleparam.h>
#include <linux/sched/rt.h>
#include <linux/random.h>

#include "komb.h"

/*
 * Implement a NUMA-aware version of MCS (aka CNA, or compact NUMA-aware lock).
 *
 * In CNA, spinning threads are organized in two queues, a primary queue for
 * threads running on the same NUMA node as the current lock holder, and a
 * secondary queue for threads running on other nodes. Schematically, it
 * looks like this:
 *
 *    komb_node
 *   +----------+     +--------+         +--------+
 *   |mcs:next  | --> |mcs:next| --> ... |mcs:next| --> NULL  [Primary queue]
 *   |mcs:cna_queue| -.  +--------+         +--------+
 *   +----------+  |
 *                 `----------------------.
 *                                        v
 *                 +--------+         +--------+
 *                 |mcs:next| --> ... |mcs:next|            [Secondary queue]
 *                 +--------+         +--------+
 *                     ^                    |
 *                     `--------------------'
 *
 * N.B. cna_queue := 1 if secondary queue is absent. Otherwise, it contains the
 * encoded pointer to the tail of the secondary queue, which is organized as a
 * circular list.
 *
 * After acquiring the MCS lock and before acquiring the spinlock, the MCS lock
 * holder checks whether the next waiter in the primary queue (if exists) is
 * running on the same NUMA node. If it is not, that waiter is detached from the
 * main queue and moved into the tail of the secondary queue. This way, we
 * gradually filter the primary queue, leaving only waiters running on the same
 * preferred NUMA node.
 *
 * For more details, see https://arxiv.org/abs/1810.05600.
 *
 * Authors: Alex Kogan <alex.kogan@oracle.com>
 *          Dave Dice <dave.dice@oracle.com>
 */

#define FLUSH_SECONDARY_QUEUE	1
#define CNA_PRIORITY_NODE      0xffff

//struct komb_node {
//	struct komb_node	mcs;
//	u16			numa_node;
//	u16			socket_id;
//	u32			cna_encoded_tail;	/* self */
//        u64                     start_time;
//};

static ulong numa_spinlock_threshold_ns = 10000000;   /* 1ms, by default */
module_param(numa_spinlock_threshold_ns, ulong, 0644);

static inline bool intra_node_threshold_reached(struct komb_node *node)
{
	u64 current_time = local_clock();
	u64 threshold = node->start_time + numa_spinlock_threshold_ns;

	return current_time > threshold;
}

/*
 * Controls the probability for enabling the ordering of the main queue
 * when the secondary queue is empty. The chosen value reduces the amount
 * of unnecessary shuffling of threads between the two waiting queues
 * when the contention is low, while responding fast enough and enabling
 * the shuffling when the contention is high.
 */
#define SHUFFLE_REDUCTION_PROB_ARG  (7)

/* Per-CPU pseudo-random number seed */
static DEFINE_PER_CPU(u32, seed);

/*
 * Return false with probability 1 / 2^@num_bits.
 * Intuitively, the larger @num_bits the less likely false is to be returned.
 * @num_bits must be a number between 0 and 31.
 */
static bool probably(unsigned int num_bits)
{
	u32 s;

	s = this_cpu_read(seed);
	s = next_pseudo_random32(s);
	this_cpu_write(seed, s);

	return s & ((1 << num_bits) - 1);
}

static inline bool check_cna_node(struct komb_node *node)
{
	return (node->lockm == FDS_CNA);
}


//static __always_inline void cna_init_node(struct komb_node *node, u32 tail)
//{
//        bool priority = !in_task() || irqs_disabled() || rt_task(current);
//
//        node->socket_id = numa_node_id();
//        node->cna_encoded_tail = tail;
//        node->cna_numa_node = priority ? CNA_PRIORITY_NODE : node->socket_id;
//        node->start_time = 0;
//}

/*
 * cna_splice_head -- splice the entire secondary queue onto the head of the
 * primary queue.
 *
 * Returns the new primary head node or NULL on failure.
 */
static struct komb_node *
cna_splice_head(struct qspinlock *lock, u32 val,
		struct komb_node *node, struct komb_node *next)
{
	struct komb_node *head_2nd, *tail_2nd;
	u32 new;

	tail_2nd = decode_tail(node->cna_queue);
	head_2nd = tail_2nd->next;

	KOMB_BUG_ON(!check_cna_node(tail_2nd));

	if (next) {
		/*
		 * If the primary queue is not empty, the primary tail doesn't
		 * need to change and we can simply link the secondary tail to
		 * the old primary head.
		 */
		tail_2nd->next = next;
	} else {
		/*
		 * When the primary queue is empty, the secondary tail becomes
		 * the primary tail.
		 */

		/*
		 * Speculatively break the secondary queue's circular link such
		 * that when the secondary tail becomes the primary tail it all
		 * works out.
		 */
		tail_2nd->next = NULL;

		/*
		 * tail_2nd->next = NULL;	old = xchg_tail(lock, tail);
		 *				prev = decode_tail(old);
		 * try_cmpxchg_release(...);	WRITE_ONCE(prev->next, node);
		 *
		 * If the following cmpxchg() succeeds, our stores will not
		 * collide.
		 */
		new = tail_2nd->cna_encoded_tail |
			_Q_LOCKED_VAL;
		if (!atomic_try_cmpxchg_release(&lock->val, &val, new)) {
			/* Restore the secondary queue's circular link. */
			tail_2nd->next = head_2nd;
			return NULL;
		}
	}

	/* The primary queue head now is what was the secondary queue head. */
	return head_2nd;
}

static inline bool cna_try_clear_tail(struct qspinlock *lock, u32 val,
				      struct komb_node *node)
{
	/*
	 * We're here because the primary queue is empty; check the secondary
	 * queue for remote waiters.
	 */
	if (node->cna_queue > 1) {
		struct komb_node *next;

		/*
		 * When there are waiters on the secondary queue, try to move
		 * them back onto the primary queue and let them rip.
		 */
		next = cna_splice_head(lock, val, node, NULL);
		if (next) {
			WRITE_ONCE(next->locked, false);
			return true;
		}

		return false;
	}

	/* Both queues are empty. Do what MCS does. */
	return atomic_try_cmpxchg_relaxed(&lock->val, &val, _Q_LOCKED_VAL);
}

/*
 * cna_splice_next -- splice the next node from the primary queue onto
 * the secondary queue.
 */
static void cna_splice_next(struct komb_node *node,
			    struct komb_node *next,
			    struct komb_node *nnext)
{
	KOMB_BUG_ON(!check_cna_node(next));
	/* remove 'next' from the main queue */
	node->next = nnext;

	/* stick `next` on the secondary queue tail */
	if (node->cna_queue <= 1) { /* if secondary queue is empty */
		/* create secondary queue */
		next->next = next;
                node->start_time = local_clock();
	} else {
		/* add to the tail of the secondary queue */
		struct komb_node *tail_2nd = decode_tail(node->cna_queue);
		struct komb_node *head_2nd = tail_2nd->next;

		tail_2nd->next = next;
		next->next = head_2nd;
	}

	node->cna_queue = next->cna_encoded_tail;
}

/*
 * cna_order_queue - check whether the next waiter in the main queue is on
 * the same NUMA node as the lock holder; if not, and it has a waiter behind
 * it in the main queue, move the former onto the secondary queue.
 * Returns 1 if the next waiter runs on the same NUMA node; 0 otherwise.
 */
static int cna_order_queue(struct komb_node *node)
{
	struct komb_node *next = READ_ONCE(node->next);
	int numa_node, next_numa_node;

	if (!next || !check_cna_node(next))
		return 0;

	numa_node = node->cna_numa_node;
	next_numa_node = next->cna_numa_node;

	if (next_numa_node != numa_node && next_numa_node != CNA_PRIORITY_NODE) {
		struct komb_node *nnext = READ_ONCE(next->next);

		if (nnext)
			cna_splice_next(node, next, nnext);

		return 0;
	}
	return 1;
}

#define LOCK_IS_BUSY(lock) (atomic_read(&(lock)->val) & _Q_LOCKED_PENDING_MASK)

/* Abuse the pv_wait_head_or_lock() hook to get some work done */
static __always_inline u32 cna_wait_head_or_lock(struct qspinlock *lock,
						 struct komb_node *node)
{
	if (node->cna_queue <= 1 && probably(SHUFFLE_REDUCTION_PROB_ARG)) {
		/*
		 * When the secondary queue is empty, skip the calls to
		 * cna_order_queue() below with high probability. This optimization
		 * reduces the overhead of unnecessary shuffling of threads
		 * between waiting queues when the lock is only lightly contended.
		 */
		return 0;
	}

	if (!node->start_time || !intra_node_threshold_reached(node)) {
		/*
		 * We are at the head of the wait queue, no need to use
		 * the fake NUMA node ID.
		 */
		if (node->cna_numa_node == CNA_PRIORITY_NODE)
			node->cna_numa_node = node->socket_id;


		/*
		 * Try and put the time otherwise spent spin waiting on
		 * _Q_LOCKED_PENDING_MASK to use by sorting our lists.
		 */
		while (LOCK_IS_BUSY(lock) && !cna_order_queue(node))
			cpu_relax();
	} else {
		node->start_time = FLUSH_SECONDARY_QUEUE;
	}

	return 0; /* we lied; we didn't wait, go do so now */
}

static inline void cna_lock_handoff(struct komb_node *node,
				 struct komb_node *next)
{
	u32 val = 1;

	if (node->start_time != FLUSH_SECONDARY_QUEUE) {
		if (node->cna_queue > 1) {
			val = node->cna_queue;	/* preseve secondary queue */

			/*
			 * We have a local waiter, either real or fake one;
			 * reload @next in case it was changed by cna_order_queue().
			 */
			next = node->next;
 
			/*
			 * Pass over NUMA node id of primary queue, to maintain the
			 * preference even if the next waiter is on a different node.
			 */
			next->cna_numa_node = node->cna_numa_node;
			next->start_time = node->start_time;
		}
	} else {
 		/*
		 * We decided to flush the secondary queue;
		 * this can only happen if that queue is not empty.
 		 */
		WARN_ON(node->cna_queue <= 1);
 		/*
		 * Splice the secondary queue onto the primary queue and pass the lock
		 * to the longest waiting remote waiter.
 		 */
		next = cna_splice_head(NULL, 0, node, next);
 	}

	WRITE_ONCE(next->locked, false);
}

/**
 * queued_spin_lock_slowpath - acquire the queued spinlock
 * @lock: Pointer to queued spinlock structure
 * @val: Current value of the queued spinlock 32-bit word
 *
 * (queue tail, pending bit, lock value)
 *
 *              fast     :    slow                                  :    unlock
 *                       :                                          :
 * uncontended  (0,0,0) -:--> (0,0,1) ------------------------------:--> (*,*,0)
 *                       :       | ^--------.------.             /  :
 *                       :       v           \      \            |  :
 * pending               :    (0,1,1) +--> (0,1,0)   \           |  :
 *                       :       | ^--'              |           |  :
 *                       :       v                   |           |  :
 * uncontended           :    (n,x,y) +--> (n,0,0) --'           |  :
 *   queue               :       | ^--'                          |  :
 *                       :       v                               |  :
 * contended             :    (*,x,y) +--> (*,0,0) ---> (*,0,1) -'  :
 *   queue               :         ^--'                             :
 */
void cna_spin_lock_slowpath(struct qspinlock *lock, struct fds_lock_key *key)
{
	struct komb_node *prev, *next, *node;
	u32 old, tail;
	int idx;

//	/*
//	 * Wait for in-progress pending->locked hand-overs with a bounded
//	 * number of spins so that we guarantee forward progress.
//	 *
//	 * 0,1,0 -> 0,0,1
//	 */
//	if (val == _Q_PENDING_VAL) {
//		int cnt = _Q_PENDING_LOOPS;
//		val = atomic_cond_read_relaxed(&lock->val,
//					       (VAL != _Q_PENDING_VAL) || !cnt--);
//	}
//
//	/*
//	 * If we observe any contention; queue.
//	 */
//	if (val & ~_Q_LOCKED_MASK)
//		goto queue;
//
//	/*
//	 * trylock || pending
//	 *
//	 * 0,0,* -> 0,1,* -> 0,0,1 pending, trylock
//	 */
//	val = queued_fetch_set_pending_acquire(lock);
//
//	/*
//	 * If we observe contention, there is a concurrent locker.
//	 *
//	 * Undo and queue; our setting of PENDING might have made the
//	 * n,0,0 -> 0,0,0 transition fail and it will now be waiting
//	 * on @next to become !NULL.
//	 */
//	if (unlikely(val & ~_Q_LOCKED_MASK)) {
//
//		/* Undo PENDING if we set it. */
//		if (!(val & _Q_PENDING_MASK))
//			clear_pending(lock);
//
//		goto queue;
//	}
//
//	/*
//	 * We're pending, wait for the owner to go away.
//	 *
//	 * 0,1,1 -> *,1,0
//	 *
//	 * this wait loop must be a load-acquire such that we match the
//	 * store-release that clears the locked bit and create lock
//	 * sequentiality; this is because not all
//	 * clear_pending_set_locked() implementations imply full
//	 * barriers.
//	 */
//	if (val & _Q_LOCKED_MASK)
//		smp_cond_load_acquire(&lock->locked, !VAL);
//
//	/*
//	 * take ownership and clear the pending bit.
//	 *
//	 * 0,1,0 -> 0,0,1
//	 */
//	clear_pending_set_locked(lock);
//	lockevent_inc(lock_pending);
//	return;
//
//	/*
//	 * End of pending bit optimistic spinning and beginning of MCS
//	 * queuing.
//	 */
//queue:
//	lockevent_inc(lock_slowpath);
//pv_queue:
	node = this_cpu_ptr(&komb_nodes[0]);
	
	idx = node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	/*
	 * 4 nodes are allocated based on the assumption that there will
	 * not be nested NMIs taking spinlocks. That may not be true in
	 * some architectures even though the chance of needing more than
	 * 4 nodes will still be extremely unlikely. When that happens,
	 * we fall back to spinning on the lock directly without using
	 * any MCS node. This is not the most elegant solution, but is
	 * simple enough.
	 */
	if (unlikely(idx >= MAX_NODES)) {
		while (!queued_spin_trylock(lock))
			cpu_relax();
		goto release;
	}

	/*
	 * Ensure that we increment the head node->count before initialising
	 * the actual node. If the compiler is kind enough to reorder these
	 * stores, then an IRQ could overwrite our assignments.
	 */
	barrier();

        bool priority = !in_task() || irqs_disabled() || rt_task(current);

        node->socket_id = numa_node_id();
        node->cna_encoded_tail = tail;
	KOMB_BUG_ON(node->cna_encoded_tail == 1);
        node->cna_numa_node = priority ? CNA_PRIORITY_NODE : node->socket_id;
        node->start_time = 0;
	node->cna_queue = 0;

	node->locked = true;
	node->next = NULL;
	node->tail = tail;
	node->cpuid = smp_processor_id();
	node->irqs_disabled = false;
	node->lock = lock;
	node->task_struct_ptr = current;
	node->diff_preempt_count = 0;
	node->lockm = FDS_CNA;
	node->rsp = 0xdeadbeef;

	/*
	 * We touched a (possibly) cold cacheline in the per-cpu queue node;
	 * attempt the trylock once more in the hope someone let go while we
	 * weren't watching.
	 */
	if (queued_spin_trylock(lock))
		goto release;

	/*
	 * Ensure that the initialisation of @node is complete before we
	 * publish the updated tail via xchg_tail() and potentially link
	 * @node into the waitqueue via WRITE_ONCE(prev->next, node) below.
	 */
	smp_wmb();

	/*
	 * Publish the updated tail.
	 * We have already touched the queueing cacheline; don't bother with
	 * pending stuff.
	 *
	 * p,*,* -> n,*,*
	 */
	old = xchg_tail(lock, tail);
	next = NULL;

	spin_stat_lock_acquire(key);

	/*
	 * if there was a previous node; link it and wait until reaching the
	 * head of the waitqueue.
	 */
	if (old & _Q_TAIL_MASK) {
		prev = decode_tail(old);

		/* Link @node into the waitqueue. */
		WRITE_ONCE(prev->next, node);

		smp_cond_load_relaxed_sched(&node->locked, !(VAL));

		/*
		 * While waiting for the MCS lock, the next pointer may have
		 * been set by another lock waiter. We optimistically load
		 * the next pointer & prefetch the cacheline for writing
		 * to reduce latency in the upcoming MCS unlock operation.
		 */
		next = READ_ONCE(node->next);
		if (next)
			prefetchw(next);
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
	 * The PV pv_wait_head_or_lock function, if active, will acquire
	 * the lock and return a non-zero value. So we have to skip the
	 * atomic_cond_read_acquire() call. As the next PV queue head hasn't
	 * been designated yet, there is no way for the locked value to become
	 * _Q_SLOW_VAL. So both the set_locked() and the
	 * atomic_cmpxchg_relaxed() calls will be safe.
	 *
	 * If PV isn't active, 0 will be returned instead.
	 *
	 */
	
	u32 val;

	if ((val = cna_wait_head_or_lock(lock, node)))
		goto locked;

	val = atomic_cond_read_acquire(&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

locked:
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

	/*
	 * In the PV case we might already have _Q_LOCKED_VAL set, because
	 * of lock stealing; therefore we must also allow:
	 *
	 * n,0,1 -> 0,0,1
	 *
	 * Note: at this point: (val & _Q_PENDING_MASK) == 0, because of the
	 *       above wait condition, therefore any concurrent setting of
	 *       PENDING will make the uncontended transition fail.
	 */
	if ((val & _Q_TAIL_MASK) == tail) {
		if (cna_try_clear_tail(lock, val, node))
			goto release; /* No contention */
	}

	/*
	 * Either somebody is queued behind us or _Q_PENDING_VAL got set
	 * which will then detect the remaining tail and queue behind us
	 * ensuring we'll see a @next.
	 */
	set_locked(lock); // TODO: Check if this needs to be atomic?

	/*
	 * contended path; wait for next if not observed yet, release.
	 */
	if (!next)
		next = smp_cond_load_relaxed(&node->next, (VAL));

	cna_lock_handoff(node, next);

release:
	/*
	 * release the node
	 */
	node = this_cpu_ptr(&komb_nodes[0]);
	node->count--;
}


