#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time64.h>
#include <linux/ktime.h>
#include <linux/cpumask.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include "operations.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Linux Kernel Spinlock Benchmark");
MODULE_DESCRIPTION("Performance benchmark for qspinlock vs eBPF spinlocks");

/* Module parameters */
static int num_threads = 4;
module_param(num_threads, int, 0644);
MODULE_PARM_DESC(num_threads, "Number of threads (1-256)");

static int time_to_run_sec = 5;
module_param(time_to_run_sec, int, 0644);
MODULE_PARM_DESC(time_to_run_sec, "Time to run benchmark in seconds");

static int data_struct = 0;  /* 0: list, 1: hash, 2: tree */
module_param(data_struct, int, 0644);
MODULE_PARM_DESC(data_struct, "Data structure: 0=list, 1=hash, 2=tree");

static int lock_strategy = 0;  /* 0: global, 1: per-element, 2: per-bucket, 3: per-node */
module_param(lock_strategy, int, 0644);
MODULE_PARM_DESC(lock_strategy, "Lock strategy: 0=global, 1=per-element, 2=per-bucket, 3=per-node");

static int num_test_keys = 100;
module_param(num_test_keys, int, 0644);
MODULE_PARM_DESC(num_test_keys, "Number of keys for update operations");

/* Global spinlock for protecting shared data structures */
static spinlock_t global_data_lock;

/* Per-CPU operation counters */
static DEFINE_PER_CPU(__u64, op_count) = 0;

/* Benchmark context */
struct bench_thread_ctx {
	struct task_struct *thread;
	__u64 time_to_run_ns;
	enum data_struct_type data_struct_type;
	enum lock_strategy lock_strat;
	__u64 *test_keys;
	__u32 num_test_keys;
	void *data_structure;
	spinlock_t *global_lock;
	int cpu_id;
};

/* ===================== Linked List Implementation ===================== */

static struct list_node *list_insert_key(struct list_node *head, __u64 key, __u64 value, enum lock_strategy strat)
{
	struct list_node *current, *new_node, *prev;
	
	if (!head) {
		head = kmalloc(sizeof(*head), GFP_KERNEL);
		if (!head)
			return NULL;
		head->kv.key = key;
		head->kv.value = value;
		head->next = NULL;
		spin_lock_init(&head->lock);
		return head;
	}
	
	/* Find insertion point */
	if (strat == LOCK_PER_ELEMENT) {
		spin_lock(&head->lock);
	}
	
	current = head->next;
	prev = head;
	
	while (current && current->kv.key < key) {
		if (strat == LOCK_PER_ELEMENT) {
			spin_unlock(&prev->lock);
			spin_lock(&current->lock);
		}
		prev = current;
		current = current->next;
	}
	
	new_node = kmalloc(sizeof(*new_node), GFP_KERNEL);
	if (!new_node) {
		if (strat == LOCK_PER_ELEMENT)
			spin_unlock(&prev->lock);
		return head;
	}
	
	new_node->kv.key = key;
	new_node->kv.value = value;
	new_node->next = current;
	spin_lock_init(&new_node->lock);
	prev->next = new_node;
	
	if (strat == LOCK_PER_ELEMENT)
		spin_unlock(&prev->lock);
	
	return head;
}

static void list_update_key(struct list_node *head, __u64 key, __u64 value, enum lock_strategy strat)
{
	struct list_node *current, *prev;
	
	if (!head)
		return;
	
	if (strat == LOCK_GLOBAL) {
		spin_unlock(&head->lock);
		spin_lock(&head->lock);
		current = head->next;
		while (current) {
			if (current->kv.key == key) {
				current->kv.value = value;
				return;
			}
			current = current->next;
		}
		spin_unlock(&head->lock);
	} else if (strat == LOCK_PER_ELEMENT) {
		spin_lock(&head->lock);
		current = head->next;
		prev = head;
		
		while (current) {
			spin_lock(&current->lock);
			spin_unlock(&prev->lock);
			
			if (current->kv.key == key) {
				current->kv.value = value;
				spin_unlock(&current->lock);
				return;
			}
			prev = current;
			current = current->next;
		}
		spin_unlock(&prev->lock);
	}
}

static void list_free(struct list_node *head)
{
	struct list_node *current, *next;
	
	if (!head)
		return;
	
	current = head->next;
	while (current) {
		next = current->next;
		kfree(current);
		current = next;
	}
	kfree(head);
}

/* ===================== Hash Table Implementation ===================== */

static struct hash_bucket *hash_table_init(void)
{
	struct hash_bucket *buckets = kzalloc(MAX_HASH_BUCKETS * sizeof(*buckets), GFP_KERNEL);
	int i;
	
	if (!buckets)
		return NULL;
	
	for (i = 0; i < MAX_HASH_BUCKETS; i++) {
		buckets[i].head = NULL;
		spin_lock_init(&buckets[i].lock);
	}
	
	return buckets;
}

static struct hash_bucket *hash_insert_key(struct hash_bucket *buckets, __u64 key, __u64 value)
{
	struct list_node *new_node, *current;
	__u32 idx;
	
	if (!buckets)
		return NULL;
	
	idx = (key * 2654435761ULL) % MAX_HASH_BUCKETS;
	spin_lock(&buckets[idx].lock);
	
	new_node = kmalloc(sizeof(*new_node), GFP_KERNEL);
	if (!new_node) {
		spin_unlock(&buckets[idx].lock);
		return buckets;
	}
	
	new_node->kv.key = key;
	new_node->kv.value = value;
	new_node->next = buckets[idx].head;
	spin_lock_init(&new_node->lock);
	
	buckets[idx].head = new_node;
	spin_unlock(&buckets[idx].lock);
	
	return buckets;
}

static void hash_update_key(struct hash_bucket *buckets, __u64 key, __u64 value, enum lock_strategy strat)
{
	struct list_node *current;
	__u32 idx;
	
	if (!buckets)
		return;
	
	idx = (key * 2654435761ULL) % MAX_HASH_BUCKETS;
	spin_lock(&buckets[idx].lock);
	
	current = buckets[idx].head;
	while (current) {
		if (current->kv.key == key) {
			current->kv.value = value;
			spin_unlock(&buckets[idx].lock);
			return;
		}
		current = current->next;
	}
	
	spin_unlock(&buckets[idx].lock);
}

static void hash_table_free(struct hash_bucket *buckets)
{
	struct list_node *current, *next;
	int i;
	
	if (!buckets)
		return;
	
	for (i = 0; i < MAX_HASH_BUCKETS; i++) {
		current = buckets[i].head;
		while (current) {
			next = current->next;
			kfree(current);
			current = next;
		}
	}
	
	kfree(buckets);
}

/* ===================== Tree Implementation ===================== */

static int tree_get_height(struct tree_node *node)
{
	return node ? node->height : 0;
}

static void tree_update_height_node(struct tree_node *node)
{
	if (node) {
		int left_h = tree_get_height(node->left);
		int right_h = tree_get_height(node->right);
		node->height = (left_h > right_h ? left_h : right_h) + 1;
	}
}

static struct tree_node *tree_rotate_right_local(struct tree_node *y)
{
	struct tree_node *x = y->left;
	struct tree_node *temp = x->right;
	
	x->right = y;
	y->left = temp;
	
	tree_update_height_node(y);
	tree_update_height_node(x);
	
	return x;
}

static struct tree_node *tree_rotate_left_local(struct tree_node *x)
{
	struct tree_node *y = x->right;
	struct tree_node *temp = y->left;
	
	y->left = x;
	x->right = temp;
	
	tree_update_height_node(x);
	tree_update_height_node(y);
	
	return y;
}

static struct tree_node *tree_insert_recursive_local(struct tree_node *node, __u64 key, __u64 value)
{
	int balance;
	
	if (!node) {
		node = kmalloc(sizeof(*node), GFP_KERNEL);
		if (!node)
			return NULL;
		node->kv.key = key;
		node->kv.value = value;
		node->left = NULL;
		node->right = NULL;
		node->height = 1;
		spin_lock_init(&node->lock);
		return node;
	}
	
	if (key < node->kv.key) {
		node->left = tree_insert_recursive_local(node->left, key, value);
	} else if (key > node->kv.key) {
		node->right = tree_insert_recursive_local(node->right, key, value);
	} else {
		node->kv.value = value;
		return node;
	}
	
	tree_update_height_node(node);
	balance = tree_get_height(node->left) - tree_get_height(node->right);
	
	/* Left-left case */
	if (balance > 1 && key < node->left->kv.key)
		return tree_rotate_right_local(node);
	
	/* Right-right case */
	if (balance < -1 && key > node->right->kv.key)
		return tree_rotate_left_local(node);
	
	/* Left-right case */
	if (balance > 1 && key > node->left->kv.key) {
		node->left = tree_rotate_left_local(node->left);
		return tree_rotate_right_local(node);
	}
	
	/* Right-left case */
	if (balance < -1 && key < node->right->kv.key) {
		node->right = tree_rotate_right_local(node->right);
		return tree_rotate_left_local(node);
	}
	
	return node;
}

static void tree_update_handover_lock(struct tree_node *root, __u64 key, __u64 value)
{
	struct tree_node *current, *parent;
	
	if (!root)
		return;
	
	parent = NULL;
	current = root;
	
	while (current) {
		spin_lock(&current->lock);
		
		if (current->kv.key == key) {
			current->kv.value = value;
			spin_unlock(&current->lock);
			if (parent)
				spin_unlock(&parent->lock);
			return;
		}
		
		if (key < current->kv.key) {
			if (!current->left) {
				spin_unlock(&current->lock);
				if (parent)
					spin_unlock(&parent->lock);
				return;
			}
			if (parent)
				spin_unlock(&parent->lock);
			parent = current;
			current = current->left;
		} else {
			if (!current->right) {
				spin_unlock(&current->lock);
				if (parent)
					spin_unlock(&parent->lock);
				return;
			}
			if (parent)
				spin_unlock(&parent->lock);
			parent = current;
			current = current->right;
		}
	}
	
	if (parent)
		spin_unlock(&parent->lock);
}

static void tree_free(struct tree_node *root)
{
	if (!root)
		return;
	
	tree_free(root->left);
	tree_free(root->right);
	kfree(root);
}

/* ===================== Benchmark Thread Function ===================== */

static int bench_thread_fn(void *data)
{
	struct bench_thread_ctx *ctx = (struct bench_thread_ctx *)data;
	ktime_t start_time, current_time;
	__u64 i, ops_count;
	__u64 *counter;
	
	set_cpus_allowed_ptr(current, cpumask_of(ctx->cpu_id));
	
	counter = this_cpu_ptr(&op_count);
	*counter = 0;
	
	start_time = ktime_get();
	
	/* Bulk insert phase */
	for (i = 0; i < ctx->num_test_keys; i++) {
		if (ctx->data_struct_type == DS_LINKED_LIST) {
			ctx->data_structure = list_insert_key(ctx->data_structure, ctx->test_keys[i], 
							      ctx->test_keys[i] + 1000, ctx->lock_strat);
		} else if (ctx->data_struct_type == DS_HASH_TABLE) {
			hash_insert_key((struct hash_bucket *)ctx->data_structure, ctx->test_keys[i], 
				       ctx->test_keys[i] + 1000);
		} else if (ctx->data_struct_type == DS_TREE) {
			ctx->data_structure = tree_insert_recursive_local((struct tree_node *)ctx->data_structure,
									 ctx->test_keys[i], ctx->test_keys[i] + 1000);
		}
	}
	
	/* Timed update phase */
	ops_count = 0;
	while (1) {
		__u64 key = ctx->test_keys[ops_count % ctx->num_test_keys];
		
		if (ctx->data_struct_type == DS_LINKED_LIST) {
			list_update_key((struct list_node *)ctx->data_structure, key, ops_count, ctx->lock_strat);
		} else if (ctx->data_struct_type == DS_HASH_TABLE) {
			hash_update_key((struct hash_bucket *)ctx->data_structure, key, ops_count, ctx->lock_strat);
		} else if (ctx->data_struct_type == DS_TREE) {
			tree_update_handover_lock((struct tree_node *)ctx->data_structure, key, ops_count);
		}
		
		(*counter)++;
		ops_count++;
		
		current_time = ktime_get();
		if (ktime_sub(current_time, start_time) >= ctx->time_to_run_ns)
			break;
		
		if (kthread_should_stop())
			break;
	}
	
	/* Bulk delete phase */
	if (ctx->data_struct_type == DS_LINKED_LIST) {
		list_free((struct list_node *)ctx->data_structure);
	} else if (ctx->data_struct_type == DS_HASH_TABLE) {
		hash_table_free((struct hash_bucket *)ctx->data_structure);
	} else if (ctx->data_struct_type == DS_TREE) {
		tree_free((struct tree_node *)ctx->data_structure);
	}
	
	return 0;
}

/* ===================== Module Init/Exit ===================== */

static __u64 test_keys[MAX_KEYS];

static int __init spinlock_bench_init(void)
{
	struct bench_thread_ctx *ctx_array;
	struct task_struct **threads;
	__u64 total_ops;
	ktime_t bench_start, bench_end;
	s64 elapsed_ns;
	double throughput;
	int i, ret;
	
	if (num_threads < 1 || num_threads > MAX_THREADS) {
		pr_err("Invalid num_threads: %d\n", num_threads);
		return -EINVAL;
	}
	
	if (data_struct < 0 || data_struct >= DS_MAX) {
		pr_err("Invalid data_struct: %d\n", data_struct);
		return -EINVAL;
	}
	
	if (lock_strategy < 0 || lock_strategy >= LOCK_MAX) {
		pr_err("Invalid lock_strategy: %d\n", lock_strategy);
		return -EINVAL;
	}
	
	if (num_test_keys < 1 || num_test_keys > MAX_KEYS) {
		pr_err("Invalid num_test_keys: %d\n", num_test_keys);
		return -EINVAL;
	}
	
	/* Initialize test keys */
	for (i = 0; i < num_test_keys; i++)
		test_keys[i] = i + 1000;
	
	/* Allocate thread contexts and structures */
	ctx_array = kzalloc(sizeof(*ctx_array) * num_threads, GFP_KERNEL);
	if (!ctx_array)
		return -ENOMEM;
	
	threads = kzalloc(sizeof(*threads) * num_threads, GFP_KERNEL);
	if (!threads) {
		kfree(ctx_array);
		return -ENOMEM;
	}
	
	spin_lock_init(&global_data_lock);
	
	/* Initialize per-CPU counters */
	for_each_possible_cpu(i) {
		per_cpu(op_count, i) = 0;
	}
	
	pr_info("Starting benchmark: data_struct=%d, lock_strategy=%d, num_threads=%d, time_to_run=%d sec\n",
		data_struct, lock_strategy, num_threads, time_to_run_sec);
	
	bench_start = ktime_get();
	
	/* Create and start threads */
	for (i = 0; i < num_threads; i++) {
		ctx_array[i].data_struct_type = (enum data_struct_type)data_struct;
		ctx_array[i].lock_strat = (enum lock_strategy)lock_strategy;
		ctx_array[i].test_keys = test_keys;
		ctx_array[i].num_test_keys = num_test_keys;
		ctx_array[i].time_to_run_ns = (s64)time_to_run_sec * NSEC_PER_SEC;
		ctx_array[i].global_lock = &global_data_lock;
		ctx_array[i].cpu_id = i % num_online_cpus();
		
		/* Each thread gets its own data structure copy to avoid contention on allocation */
		ctx_array[i].data_structure = NULL;
		
		threads[i] = kthread_run(bench_thread_fn, &ctx_array[i], "bench_thread_%d", i);
		if (IS_ERR(threads[i])) {
			ret = PTR_ERR(threads[i]);
			pr_err("Failed to create thread %d: %d\n", i, ret);
			/* Stop remaining threads */
			for (int j = 0; j < i; j++)
				kthread_stop(threads[j]);
			kfree(ctx_array);
			kfree(threads);
			return ret;
		}
	}
	
	/* Wait for all threads to complete */
	for (i = 0; i < num_threads; i++) {
		kthread_stop(threads[i]);
	}
	
	bench_end = ktime_get();
	elapsed_ns = ktime_sub(bench_end, bench_start);
	
	/* Calculate total operations and throughput */
	total_ops = 0;
	for_each_possible_cpu(i) {
		total_ops += per_cpu(op_count, i);
	}
	
	throughput = (double)total_ops * NSEC_PER_SEC / elapsed_ns;
	
	pr_info("Benchmark completed:\n");
	pr_info("  Total operations: %llu\n", total_ops);
	pr_info("  Elapsed time: %lld ns (%.3f sec)\n", elapsed_ns, (double)elapsed_ns / NSEC_PER_SEC);
	pr_info("  Throughput: %.2f ops/sec\n", throughput);
	pr_info("  data_struct=%d, lock_strategy=%d, num_threads=%d\n", data_struct, lock_strategy, num_threads);
	
	kfree(ctx_array);
	kfree(threads);
	
	return 0;
}

static void __exit spinlock_bench_exit(void)
{
	pr_info("Spinlock benchmark module unloaded\n");
}

module_init(spinlock_bench_init);
module_exit(spinlock_bench_exit);
