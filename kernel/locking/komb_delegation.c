// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Vishal Gupta, Kumar Kartikeya Dwivedi

#include <linux/file.h>
#include <linux/module.h>
#include "komb.h"

static struct task_struct **dthreads;
static int num_delegation_threads = 1;
static int num_cores_per_socket = 1;

static DEFINE_PER_CPU_ALIGNED(struct komb_node, *lock_rq_tail);

extern enum system_states system_state;

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__((noipa)) noinline notrace static struct komb_node *
run_combiner(struct komb_node *curr_node)
{
	struct shadow_stack *ptr;
	struct qspinlock *lock;

	KOMB_BUG_ON(curr_node == NULL);
	KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) != 0);
	ptr = this_cpu_ptr(&local_shadow_stack);
	lock = curr_node->lock;

	ptr->counter_val = 0;
	ptr->prev_cs_cpu = -1;
	ptr->next_node_ptr = NULL;
	ptr->curr_cs_cpu = curr_node->cpuid;

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

	KOMB_BUG_ON(ptr->prev_cs_cpu == -1 || ptr->prev_cs_cpu == 0);

	ptr->curr_cs_cpu = -1;
	return per_cpu_ptr(&komb_nodes[0], ptr->prev_cs_cpu);
}
#pragma GCC pop_options

static inline __pure u32 select_delegation_cpu(struct qspinlock *lock)
{
	return (num_cores_per_socket * numa_node_id());
	// 				+ ((u64)lock % (num_delegation_threads / num_online_nodes())));
}

__attribute__((noipa)) noinline notrace static int
__kd_spin_lock_slowpath(struct qspinlock *lock)
{
	struct komb_node *curr_node;
	register struct komb_node *prev_node;
	struct komb_node **rq_tail;
	u32 tail, old_tail, i, val, new_val, idx;

	curr_node = this_cpu_ptr(&komb_nodes[0]);
	idx = curr_node->count++;
	tail = encode_tail(smp_processor_id(), idx);

	curr_node->locked = true;
	curr_node->completed = false;
	curr_node->next = NULL;
	curr_node->tail = tail;
	curr_node->socket_id = numa_node_id();
	curr_node->cpuid = smp_processor_id();
	curr_node->irqs_disabled = false;
	curr_node->lock = lock;
	curr_node->task_struct_ptr = current;

	print_debug("my_tail: %x\n", tail);

	old_tail = xchg_tail(lock, tail);

	if (old_tail & _Q_TAIL_MASK) {
		prev_node = decode_tail(old_tail);
		WRITE_ONCE(prev_node->next, curr_node);
		smp_mb();
	} else {
head_of_queue:
		print_debug("Head of queue\n");
		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			KOMB_BUG_ON(lock->locked != 0);

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) ==
			    val)
				break;
		}

		print_debug("Got the lock\n");

		rq_tail =
			per_cpu_ptr(&lock_rq_tail, select_delegation_cpu(lock));
		if (READ_ONCE(*rq_tail) == 0xdeadbeef ||
		    !task_is_running(dthreads[numa_node_id()]) ||
		    cmpxchg(rq_tail, NULL, curr_node) != NULL) {
			// Fallback to qspinlock
			// this_cpu_inc(qspinlock_fallback);
			print_debug("Delegation %d running something else\n",
				    select_delegation_cpu(lock));
			curr_node->locked = false;
			curr_node->completed = true;

			val = atomic_read(&lock->val);

			if (((val & _Q_TAIL_MASK) == tail) &&
			    atomic_try_cmpxchg_relaxed(&lock->val, &val,
						       _Q_LOCKED_VAL)) {
				print_debug(
					"IRQ only one in the queue unlocked\n");
				goto continue_with_cs_execution;
			} else {
				print_debug("Someone else joined the queue\n");
			}

			smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
			KOMB_BUG_ON(curr_node->next == NULL);
			print_debug("Next node now head of queue: %d\n",
				    curr_node->next->cpuid);
			WRITE_ONCE(curr_node->next->locked, false);

		} else {
			KOMB_BUG_ON(!task_is_running(dthreads[numa_node_id()]));
			print_debug("Added to the delegation thread: %d\n",
				    select_delegation_cpu(lock));
		}
	}

	smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));

	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);

	if (!curr_node->completed) {
		print_debug("Head of queue but not completed\n");
		curr_node->locked = true;
		curr_node->completed = false;
		goto head_of_queue;
	}

continue_with_cs_execution:
	if (curr_node->irqs_disabled) {
		ptr->irqs_disabled = curr_node->irqs_disabled;
	}
	curr_node->count--;
	KOMB_BUG_ON((char *)curr_node->rsp == 0);
	for (i = 0; i < NUM_PREFETCH_LINES; i++)
		prefetchw(((char *)curr_node->rsp) + (64 * i));

	return 0;
}

int komb_thread(void *args)
{
	struct qspinlock *lock;
	struct shadow_stack *ptr;
	struct komb_node *prev_node, *next_node;
	struct komb_node **rq_tail;
	int j, prev_preempt_count;

	ptr = this_cpu_ptr(&local_shadow_stack);
	rq_tail = this_cpu_ptr(&lock_rq_tail);
	lock = NULL;

	while (true) {
		if (READ_ONCE(*rq_tail) != 0xdeadbeef)
			break;
		cpu_relax();
		cond_resched();
	}

	preempt_disable();

	while (true) {
		print_debug("komb thread waiting for lock\n");
		smp_cond_load_relaxed_sched_delegation(rq_tail, (VAL));

		next_node = *rq_tail;
		KOMB_BUG_ON(next_node == NULL);
		KOMB_BUG_ON(next_node->cpuid == smp_processor_id());
		lock = next_node->lock;
		KOMB_BUG_ON(lock->locked ==
			    0); //Lock should already be acquired.
		WRITE_ONCE(lock->locked, _Q_LOCKED_COMBINER_VAL);
		print_debug("Running combiner with node from: %d\n",
			    next_node->cpuid);

		j = 0;
		for (j = 0; j < 8; j++)
			if (ptr->lock_addr[j] == NULL)
				break;

		KOMB_BUG_ON(j !=
			    0); //TODO: Update this condition nested delegation

		ptr->lock_addr[j] = lock;

		ptr->local_queue_head = NULL;
		ptr->local_queue_tail = NULL;
		ptr->is_local_queue_tail_last = false;

		KOMB_BUG_ON(!in_task());
		KOMB_BUG_ON(irqs_disabled());

		prev_preempt_count = preempt_count();

		prev_node = run_combiner(next_node);

		KOMB_BUG_ON(prev_preempt_count != preempt_count());

		KOMB_BUG_ON(ptr->lock_addr[j] != lock);
		ptr->lock_addr[j] = NULL;

		print_debug("Got control back prev node: %d\n",
			    prev_node->cpuid);
		KOMB_BUG_ON(prev_node == NULL);
		next_node = NULL;
		WRITE_ONCE(*rq_tail, NULL); //Combining done

		//lock = prev_node->lock; //TODO: Check why this is needed.
		if (READ_ONCE(prev_node->next) == NULL) {
			if (ptr->local_queue_head != NULL) {
				if (!ptr->is_local_queue_tail_last) {
					ptr->local_queue_tail->next = NULL;
					if (cmpxchg_tail(lock, prev_node->tail,
							 ptr->local_queue_tail
								 ->tail) !=
					    prev_node->tail) {
						print_debug(
							"local_queue_tail exists Someone else joined the queue: %d\n",
							prev_node->cpuid);
						smp_cond_load_relaxed_sched(
							&prev_node->next,
							(VAL));
						print_debug(
							"local_queue_tail exists Got the node: %d\n",
							READ_ONCE(
								prev_node->next
									->cpuid));

						ptr->local_queue_tail->next =
							prev_node->next;
					}
				}
				next_node = ptr->local_queue_head;
				print_debug(
					"prev_node next null, local_queue_head: %d local_queue_tail: %d\n",
					ptr->local_queue_head->cpuid,
					ptr->local_queue_tail->cpuid);
			} else {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				if (cmpxchg_tail(lock, prev_node->tail, 0) !=
				    prev_node->tail) {
					print_debug(
						"local_queue_tail exists Someone else joined the queue: %d\n",
						prev_node->cpuid);
					smp_cond_load_relaxed_sched(
						&prev_node->next, (VAL));
					print_debug(
						"local_queue_tail exists Got the node: %d\n",
						READ_ONCE(
							prev_node->next->cpuid));
					next_node = prev_node->next;
				}
			}
		} else {
			if (ptr->local_queue_head != NULL) {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				ptr->local_queue_tail->next = prev_node->next;
				next_node = ptr->local_queue_head;
				print_debug(
					"prev_node next %d, local_queue_head: %d local_queue_tail: %d\n",
					prev_node->next->cpuid,
					ptr->local_queue_head->cpuid,
					ptr->local_queue_tail->cpuid);
			} else {
				KOMB_BUG_ON(ptr->is_local_queue_tail_last);
				next_node = prev_node->next;
				print_debug(
					"prev_node next %d, local_queue_head NULL\n",
					prev_node->next->cpuid);
			}
		}

		if (READ_ONCE(next_node) != NULL) {
			next_node->locked = false;
		}

		clear_locked_set_completed(prev_node);

		KOMB_BUG_ON(lock->locked != _Q_LOCKED_COMBINER_VAL);
		print_debug("Releasing the lock from combiner\n");
		WRITE_ONCE(lock->locked, 0);
	}

	BUG_ON(true);
	preempt_enable();

	return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static __attribute__((noipa)) noinline notrace void
kd_spin_lock_slowpath(struct qspinlock *lock)
{
	register int ret_val;
	KOMB_BUG_ON(irqs_disabled());
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

	ret_val = __kd_spin_lock_slowpath(lock);

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
}
#pragma GCC pop_options

inline bool kd_spin_trylock(struct qspinlock *lock)
{
	return (atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL) == 0);
}

__attribute__((noipa)) noinline notrace void
kd_spin_lock(struct qspinlock *lock)
{
	struct komb_node *curr_node = NULL;
	struct shadow_stack *ptr;
	uint32_t curr_cpuid, prev_cpuid;
	struct komb_node *prev_node, *next_node;
	u32 val, cnt;

	val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
	if (val == 0)
		return;

	KOMB_BUG_ON(!in_atomic());

queue:
	if (unlikely(system_state != SYSTEM_RUNNING)) {
		while (!kd_spin_trylock(lock))
			cpu_relax();
		return;
	}

	curr_node = this_cpu_ptr(&komb_nodes[0]);
	KOMB_BUG_ON(curr_node == NULL);

	if (curr_node->count > 0 || !in_task() || irqs_disabled() ||
	    current->migration_disabled ||
	    ((smp_processor_id() % num_cores_per_socket) == 0) ||
	    !task_is_running(dthreads[numa_node_id()])) {
		struct komb_node *prev_node, *next_node;
		u32 tail, idx;

		idx = curr_node->count++;
		if (unlikely(idx >= MAX_NODES)) {
			while (!kd_spin_trylock(lock))
				cpu_relax();
			goto irq_release;
		}

		curr_node += idx;
		tail = encode_tail(smp_processor_id(), idx);

		barrier();

		curr_node->locked = true;
		curr_node->completed = false;
		curr_node->next = NULL;
		curr_node->tail = tail;
		curr_node->socket_id = IRQ_NUMA_NODE;
		curr_node->cpuid = smp_processor_id();
		curr_node->irqs_disabled = false;
		curr_node->lock = lock;
		curr_node->task_struct_ptr = current;

		uint64_t prev_rsp = curr_node->rsp;
		curr_node->rsp = 0xdeadbeef;

		smp_wmb();

		u32 old_tail = xchg_tail(lock, tail);

		if (old_tail & _Q_TAIL_MASK) {
			prev_node = decode_tail(old_tail);
			WRITE_ONCE(prev_node->next, curr_node);

			print_debug("IRQ going to waiting for lock\n");
			smp_cond_load_relaxed_sched(&curr_node->locked, !(VAL));
		}

		curr_node->rsp = prev_rsp;

		u32 val, new_val;

		print_debug("IRQ spinning on the locked field\n");

		val = atomic_cond_read_acquire(&lock->val,
					       !(VAL & _Q_LOCKED_PENDING_MASK));

		if (((val & _Q_TAIL_MASK) == tail) &&
		    atomic_try_cmpxchg_relaxed(&lock->val, &val,
					       _Q_LOCKED_IRQ_VAL)) {
			print_debug("IRQ only one in the queue unlocked\n");
			goto irq_release;
		}

		while (true) {
			val = atomic_cond_read_relaxed(
				&lock->val, !(VAL & _Q_LOCKED_PENDING_MASK));

			KOMB_BUG_ON(lock->locked != 0);

			new_val = val >> _Q_LOCKED_BITS;
			new_val <<= _Q_LOCKED_BITS;
			new_val |= _Q_LOCKED_IRQ_VAL;

			if (atomic_cmpxchg_acquire(&lock->val, val, new_val) ==
			    val)
				break;
		}

		print_debug("IRQ got the lock\n");

		smp_cond_load_relaxed_sched(&curr_node->next, (VAL));
		next_node = curr_node->next;

		KOMB_BUG_ON(next_node == NULL);

		WRITE_ONCE(next_node->locked, false);

		print_debug("IRQ passing lock next node: %d\n",
			    next_node->cpuid);

irq_release:
		curr_node = this_cpu_ptr(&komb_nodes[0]);
		curr_node->count--;
		return;
	} else {
		KOMB_BUG_ON((smp_processor_id() % num_cores_per_socket) == 0);

		print_debug("Going on slowpath\n");

		kd_spin_lock_slowpath(lock);

		ptr = this_cpu_ptr(&local_shadow_stack);
		curr_cpuid = ptr->curr_cs_cpu;
		if (curr_cpuid != -1) {
			curr_node = per_cpu_ptr(&komb_nodes[0], curr_cpuid);
			next_node = get_next_node(curr_node);
			ptr->next_node_ptr = next_node;

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
}

// __attribute__((noipa)) noinline notrace void
// kd_spin_unlock(struct qspinlock *lock)
// {
// 	struct shadow_stack *ptr = this_cpu_ptr(&local_shadow_stack);
// 	int from_cpuid = ptr->curr_cs_cpu;
// 	void *incoming_rsp_ptr, *outgoing_rsp_ptr;
// 	struct komb_node *curr_node;
// 	struct komb_node *next_node;
// 	uint64_t counter;
// 	int j, max_idx, my_idx;
// 	void *temp_lock_addr;

// 	j = 0;
// 	max_idx = -1;
// 	my_idx = -1;

// 	for (j = 0; j < 8; j++) {
// 		temp_lock_addr = ptr->lock_addr[j];
// 		if (temp_lock_addr != NULL)
// 			max_idx = j;
// 		if (temp_lock_addr == lock)
// 			my_idx = j;
// 		if (temp_lock_addr == NULL)
// 			break;
// 	}

// 	if (my_idx == -1) {
// 		if (lock->locked == _Q_LOCKED_VAL ||
// 		    lock->locked == _Q_LOCKED_IRQ_VAL) {
// 			lock->locked = false;
// 		} else {
// 			KOMB_BUG_ON(true);
// 		}
// 		return;
// 	}

// 	KOMB_BUG_ON(my_idx < max_idx);

// #if DEBUG_KOMB
// 	//Delegation thread should be on CPU 0 on each socket
// 	if ((smp_processor_id() % num_cores_per_socket) != 0) {
// 		printk(KERN_ALERT "curr_cpu: %d\n", smp_processor_id());
// 		BUG_ON(true);
// 	}
// 	BUG_ON(from_cpuid == -1);
// #endif

// 	curr_node = per_cpu_ptr(&komb_nodes[0], from_cpuid);

// 	next_node = ptr->next_node_ptr;

// 	counter = ptr->counter_val;

// 	if (next_node == NULL || need_resched()) {
// 		incoming_rsp_ptr = &(ptr->local_shadow_stack_ptr);
// 		ptr->curr_cs_cpu = -1;
// 		ptr->prev_cs_cpu = curr_node->cpuid;

// 	} else {
// 		ptr->is_local_queue_tail_last = false;
// 		ptr->curr_cs_cpu = next_node->cpuid;
// 		ptr->prev_cs_cpu = curr_node->cpuid;
// 		incoming_rsp_ptr = &(next_node->rsp);
// 		ptr->counter_val = counter + 1;
// 		print_debug("Jumping to the next waiter: %d\n",
// 			    next_node->cpuid);
// 	}

// 	outgoing_rsp_ptr = &(curr_node->rsp);

// 	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == 0);
// 	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == 0);
// 	KOMB_BUG_ON(*(uint64_t *)incoming_rsp_ptr == 0xdeadbeef);
// 	KOMB_BUG_ON(*(uint64_t *)outgoing_rsp_ptr == 0xdeadbeef);

// 	curr_node->irqs_disabled = irqs_disabled();

// 	komb_context_switch(incoming_rsp_ptr, outgoing_rsp_ptr);

// 	ptr = this_cpu_ptr(&local_shadow_stack);
// 	if (ptr->irqs_disabled) {
// 		ptr->irqs_disabled = false;
// 		local_irq_disable();
// 	}
// 	return;
// }

static int __init kd_init(void)
{
	int i, j;
	struct komb_node *komb_node;

	for_each_possible_cpu(i) {
		*per_cpu_ptr(&lock_rq_tail, i) = 0;
	}

	komb_node = per_cpu_ptr(&komb_nodes[0], 0);
	komb_node->next = NULL;
	komb_node->rsp = (void *)0xdeadbeef;
	komb_node->locked_completed = 0;
	komb_node->socket_id = -1;

	num_delegation_threads = num_online_nodes();
	num_cores_per_socket = num_online_cpus() / num_online_nodes();

	printk(KERN_ALERT "======== KOMB starting delegation ========\n");

	dthreads =
		vzalloc(num_delegation_threads * sizeof(struct task_struct *));
	for (i = 0; i < num_delegation_threads; i++) {
		dthreads[i] = kthread_create(komb_thread, NULL, "komb_thread");
		kthread_bind(dthreads[i], i * num_cores_per_socket);
		if (dthreads[i])
			wake_up_process(dthreads[i]);
		else
			printk(KERN_ALERT
			       "failed to create komb delegation threads\n");
	}

	printk(KERN_ALERT "starting delegation threads\n");

	return 0;
}

module_init(kd_init)