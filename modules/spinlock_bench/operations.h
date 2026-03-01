#ifndef __SPINLOCK_BENCH_OPERATIONS_H__
#define __SPINLOCK_BENCH_OPERATIONS_H__

#include <linux/types.h>

/* Configuration constants */
#define MAX_THREADS 256
#define MAX_KEYS 10000
#define MAX_HASH_BUCKETS 256

/* Data structure types */
enum data_struct_type {
	DS_LINKED_LIST = 0,
	DS_HASH_TABLE = 1,
	DS_TREE = 2,
	DS_MAX
};

/* Lock strategy types */
enum lock_strategy {
	LOCK_GLOBAL = 0,
	LOCK_PER_ELEMENT = 1,  /* For linked list */
	LOCK_PER_BUCKET = 2,   /* For hash table */
	LOCK_PER_NODE = 3,     /* For tree - hand-over-hand */
	LOCK_MAX
};

/* Key-value pair structure */
struct kv_pair {
	__u64 key;
	__u64 value;
};

/* Linked list node */
struct list_node {
	struct kv_pair kv;
	struct list_node *next;
	spinlock_t lock;  /* Used only for LOCK_PER_ELEMENT strategy */
};

/* Hash table bucket */
struct hash_bucket {
	struct list_node *head;
	spinlock_t lock;  /* Used only for LOCK_PER_BUCKET strategy */
};

/* Binary tree node (for balanced tree) */
struct tree_node {
	struct kv_pair kv;
	struct tree_node *left;
	struct tree_node *right;
	spinlock_t lock;  /* Used for LOCK_PER_NODE strategy (hand-over-hand) */
	int height;       /* For AVL balancing */
};

/* Benchmark context structure */
struct bench_context {
	enum data_struct_type data_struct;
	enum lock_strategy lock_strategy;
	__u32 num_threads;
	__u64 time_to_run_ns;  /* Run time in nanoseconds */
	
	/* Global lock (used for LOCK_GLOBAL strategy) */
	spinlock_t global_lock;
	
	/* Per-CPU operation counters */
	__u64 __percpu *op_count;
	
	/* Data structure pointers */
	void *data_structure;
	
	/* Bulk insert/delete test parameters */
	__u32 num_test_keys;
	__u64 test_keys[MAX_KEYS];
};

/* Operation definitions */

/* Linked List Operations */
static inline struct list_node *list_alloc_node(__u64 key, __u64 value)
{
	struct list_node *node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return NULL;
	node->kv.key = key;
	node->kv.value = value;
	node->next = NULL;
	spin_lock_init(&node->lock);
	return node;
}

static inline void list_free_node(struct list_node *node)
{
	kfree(node);
}

/* Hash Table Operations */
static inline struct hash_bucket *hash_init(void)
{
	struct hash_bucket *buckets = kzalloc(MAX_HASH_BUCKETS * sizeof(*buckets), GFP_KERNEL);
	int i;
	if (!buckets)
		return NULL;
	for (i = 0; i < MAX_HASH_BUCKETS; i++) {
		spin_lock_init(&buckets[i].lock);
		buckets[i].head = NULL;
	}
	return buckets;
}

static inline __u32 hash_fn(__u64 key)
{
	return (key * 2654435761ULL) % MAX_HASH_BUCKETS;
}

/* Tree Operations */
static inline struct tree_node *tree_alloc_node(__u64 key, __u64 value)
{
	struct tree_node *node = kmalloc(sizeof(*node), GFP_KERNEL);
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

static inline void tree_free_node(struct tree_node *node)
{
	kfree(node);
}

static inline int tree_height(struct tree_node *node)
{
	return node ? node->height : 0;
}

static inline int tree_balance_factor(struct tree_node *node)
{
	return node ? tree_height(node->left) - tree_height(node->right) : 0;
}

static inline void tree_update_height(struct tree_node *node)
{
	if (node) {
		int left_h = tree_height(node->left);
		int right_h = tree_height(node->right);
		node->height = (left_h > right_h ? left_h : right_h) + 1;
	}
}

/* Right rotation for AVL balancing */
static inline struct tree_node *tree_rotate_right(struct tree_node *y)
{
	struct tree_node *x = y->left;
	struct tree_node *temp = x->right;
	
	x->right = y;
	y->left = temp;
	
	tree_update_height(y);
	tree_update_height(x);
	
	return x;
}

/* Left rotation for AVL balancing */
static inline struct tree_node *tree_rotate_left(struct tree_node *x)
{
	struct tree_node *y = x->right;
	struct tree_node *temp = y->left;
	
	y->left = x;
	x->right = temp;
	
	tree_update_height(x);
	tree_update_height(y);
	
	return y;
}

/* Bulk insert a key-value pair into tree with AVL balancing */
static struct tree_node *tree_insert_recursive(struct tree_node *node, __u64 key, __u64 value)
{
	if (!node)
		return tree_alloc_node(key, value);
	
	if (key < node->kv.key) {
		node->left = tree_insert_recursive(node->left, key, value);
	} else if (key > node->kv.key) {
		node->right = tree_insert_recursive(node->right, key, value);
	} else {
		/* Key already exists, update value */
		node->kv.value = value;
		return node;
	}
	
	tree_update_height(node);
	
	int balance = tree_balance_factor(node);
	
	/* Left-left case */
	if (balance > 1 && key < node->left->kv.key)
		return tree_rotate_right(node);
	
	/* Right-right case */
	if (balance < -1 && key > node->right->kv.key)
		return tree_rotate_left(node);
	
	/* Left-right case */
	if (balance > 1 && key > node->left->kv.key) {
		node->left = tree_rotate_left(node->left);
		return tree_rotate_right(node);
	}
	
	/* Right-left case */
	if (balance < -1 && key < node->right->kv.key) {
		node->right = tree_rotate_right(node->right);
		return tree_rotate_left(node);
	}
	
	return node;
}

#endif /* __SPINLOCK_BENCH_OPERATIONS_H__ */
