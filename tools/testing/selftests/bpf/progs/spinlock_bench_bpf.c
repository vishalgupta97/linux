#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

/* Arena memory for data structure allocation - 64MB */
#define BENCH_ARENA_SIZE (64 * 1024 * 1024)

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__ulong(max_entries, BENCH_ARENA_SIZE / 4096);
} bench_arena SEC(".maps");

/* Configuration and state storage */
struct bench_config {
	__u32 data_struct;        /* enum data_struct_type */
	__u32 lock_strategy;      /* enum lock_strategy */
	__u32 num_threads;
	__u64 time_to_run_ns;
	__u32 num_test_keys;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, struct bench_config);
} config_map SEC(".maps");

/* Per-CPU operation counter */
struct {
	__uint(type, BPF_MAP_TYPE_PERCPU_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, __u64);
} op_counter SEC(".maps");

/* Test keys for bulk insert/update/delete */
struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 10000);
	__type(key, __u32);
	__type(value, __u64);
} test_keys SEC(".maps");

/* Shared data structure pointer (arena offset) */
struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, __u64);  /* Arena pointer offset */
} data_root SEC(".maps");

/* ==================== Data Structures (Arena-based) ==================== */

/* Linked list node - stored in arena */
struct list_node {
	__u64 key;
	__u64 value;
	__u64 next;            /* Offset in arena */
	bpf_spin_lock_t lock;
};

/* Hash table bucket - array of buckets in arena */
struct hash_bucket {
	__u64 head;            /* Offset to first list_node in arena */
	bpf_spin_lock_t lock;
};

/* Tree node - stored in arena */
struct tree_node {
	__u64 key;
	__u64 value;
	__u64 left;            /* Offset in arena */
	__u64 right;           /* Offset in arena */
	bpf_spin_lock_t lock;
	int height;
};

/* ==================== Arena Helper Macros ==================== */

#define ARENA_OFFSET_TO_PTR(offset) ((void *)(__u64)offset)
#define PTR_TO_ARENA_OFFSET(ptr) ((__u64)ptr)

/* ==================== Linked List Operations ==================== */

__u64 list_insert(struct list_node *head, __u64 key, __u64 value) {
	struct list_node *current, *prev, *new_node;
	__u64 current_offset, new_offset;
	
	if (!head)
		return 0;
	
	bpf_spin_lock(&head->lock);
	
	current_offset = head->next;
	current = ARENA_OFFSET_TO_PTR(current_offset);
	prev = head;
	
	/* Find insertion point */
	#pragma unroll(100)
	for (int i = 0; i < 100; i++) {
		if (!current)
			break;
		if (current->key >= key)
			break;
		prev = current;
		current_offset = current->next;
		current = ARENA_OFFSET_TO_PTR(current_offset);
	}
	
	/* Allocate new node in arena */
	new_node = bpf_arena_alloc_pages(&bench_arena, 0, sizeof(*new_node), BPF_ANY);
	if (!new_node) {
		bpf_spin_unlock(&head->lock);
		return 0;
	}
	
	new_node->key = key;
	new_node->value = value;
	new_node->next = current_offset;
	
	new_offset = PTR_TO_ARENA_OFFSET(new_node);
	if (prev == head) {
		prev->next = new_offset;
	} else {
		bpf_spin_lock(&prev->lock);
		prev->next = new_offset;
		bpf_spin_unlock(&prev->lock);
	}
	
	bpf_spin_unlock(&head->lock);
	return new_offset;
}

void list_update(struct list_node *head, __u64 key, __u64 value, __u32 strategy) {
	struct list_node *current;
	__u64 current_offset;
	__u64 *counter;
	__u32 zero = 0;
	
	if (!head)
		return;
	
	counter = bpf_map_lookup_elem(&op_counter, &zero);
	if (!counter)
		return;
	
	if (strategy == 0) {  /* LOCK_GLOBAL */
		bpf_spin_lock(&head->lock);
		current = head->next;
		
		for (int i = 0; i < 100; i++) {
			current = ARENA_OFFSET_TO_PTR(head->next);
			if (!current)
				break;
			if (current->key == key) {
				current->value = value;
				(*counter)++;
				break;
			}
			current_offset = current->next;
			if (!current_offset)
				break;
		}
		bpf_spin_unlock(&head->lock);
	} else if (strategy == 1) {  /* LOCK_PER_ELEMENT */
		current = head->next;
		prev = head;
		
		for (int i = 0; i < 100; i++) {
			current = ARENA_OFFSET_TO_PTR(head->next);
			if (!current)
				break;
			
			if (current->key == key) {
				bpf_spin_lock(&current->lock);
				current->value = value;
				(*counter)++;
				bpf_spin_unlock(&current->lock);
				break;
			}
			current_offset = current->next;
			if (!current_offset)
				break;
		}
	}
}

/* ==================== Hash Table Operations ==================== */

__u64 hash_insert(struct hash_bucket *buckets, __u64 key, __u64 value) {
	struct hash_bucket *bucket;
	struct list_node *new_node;
	__u32 hash_idx;
	__u64 new_offset;
	
	if (!buckets)
		return 0;
	
	hash_idx = (key * 2654435761ULL) & 0xFF;  /* MAX_HASH_BUCKETS = 256 */
	bucket = &buckets[hash_idx];
	
	bpf_spin_lock(&bucket->lock);
	
	new_node = bpf_arena_alloc_pages(&bench_arena, 0, sizeof(*new_node), BPF_ANY);
	if (!new_node) {
		bpf_spin_unlock(&bucket->lock);
		return 0;
	}
	
	new_node->key = key;
	new_node->value = value;
	new_offset = PTR_TO_ARENA_OFFSET(new_node);
	new_node->next = bucket->head;
	bucket->head = new_offset;
	
	bpf_spin_unlock(&bucket->lock);
	return new_offset;
}

void hash_update(struct hash_bucket *buckets, __u64 key, __u64 value, __u32 strategy) {
	struct hash_bucket *bucket;
	struct list_node *current;
	__u32 hash_idx;
	__u64 *counter;
	__u32 zero = 0;
	
	if (!buckets)
		return;
	
	counter = bpf_map_lookup_elem(&op_counter, &zero);
	if (!counter)
		return;
	
	hash_idx = (key * 2654435761ULL) & 0xFF;
	bucket = &buckets[hash_idx];
	
	if (strategy == 0) {  /* LOCK_GLOBAL (not used for hash, but for completeness) */
		bpf_spin_lock(&bucket->lock);
		current = ARENA_OFFSET_TO_PTR(bucket->head);
		
		for (int i = 0; i < 100; i++) {
			if (!current)
				break;
			if (current->key == key) {
				current->value = value;
				(*counter)++;
				break;
			}
			current = ARENA_OFFSET_TO_PTR(current->next);
		}
		bpf_spin_unlock(&bucket->lock);
	} else if (strategy == 2) {  /* LOCK_PER_BUCKET */
		bpf_spin_lock(&bucket->lock);
		current = ARENA_OFFSET_TO_PTR(bucket->head);
		
		for (int i = 0; i < 100; i++) {
			if (!current)
				break;
			if (current->key == key) {
				current->value = value;
				(*counter)++;
				break;
			}
			current = ARENA_OFFSET_TO_PTR(current->next);
		}
		bpf_spin_unlock(&bucket->lock);
	}
}

/* ==================== Tree Operations ==================== */

struct tree_node *tree_search(struct tree_node *root, __u64 key) {
	struct tree_node *current = root;
	
	#pragma unroll(50)
	for (int i = 0; i < 50; i++) {
		if (!current)
			return NULL;
		
		if (current->key == key)
			return current;
		
		if (key < current->key) {
			current = ARENA_OFFSET_TO_PTR(current->left);
		} else {
			current = ARENA_OFFSET_TO_PTR(current->right);
		}
	}
	
	return NULL;
}

void tree_update_handover(struct tree_node *root, __u64 key, __u64 value) {
	struct tree_node *current, *parent;
	__u64 *counter;
	__u32 zero = 0;
	
	if (!root)
		return;
	
	counter = bpf_map_lookup_elem(&op_counter, &zero);
	if (!counter)
		return;
	
	parent = NULL;
	current = root;
	
	/* Hand-over-hand locking during traversal */
	#pragma unroll(50)
	for (int i = 0; i < 50; i++) {
		if (!current)
			break;
		
		bpf_spin_lock(&current->lock);
		
		if (current->key == key) {
			current->value = value;
			(*counter)++;
			bpf_spin_unlock(&current->lock);
			if (parent)
				bpf_spin_unlock(&parent->lock);
			break;
		}
		
		if (key < current->key) {
			struct tree_node *next = ARENA_OFFSET_TO_PTR(current->left);
			if (!next) {
				bpf_spin_unlock(&current->lock);
				if (parent)
					bpf_spin_unlock(&parent->lock);
				break;
			}
			
			if (parent)
				bpf_spin_unlock(&parent->lock);
			parent = current;
			current = next;
		} else {
			struct tree_node *next = ARENA_OFFSET_TO_PTR(current->right);
			if (!next) {
				bpf_spin_unlock(&current->lock);
				if (parent)
					bpf_spin_unlock(&parent->lock);
				break;
			}
			
			if (parent)
				bpf_spin_unlock(&parent->lock);
			parent = current;
			current = next;
		}
	}
}

/* ==================== Main Benchmark Program ==================== */

SEC("syscall")
int bench_run(void *ctx) {
	struct bench_config *cfg;
	struct list_node *list_head;
	struct hash_bucket *hash_buckets;
	struct tree_node *tree_root;
	__u64 *test_key, *data_ptr;
	__u32 zero = 0;
	__u64 *counter;
	__u32 i;
	
	cfg = bpf_map_lookup_elem(&config_map, &zero);
	if (!cfg)
		return 0;
	
	counter = bpf_map_lookup_elem(&op_counter, &zero);
	if (!counter)
		return 0;
	
	data_ptr = bpf_map_lookup_elem(&data_root, &zero);
	if (!data_ptr)
		return 0;
	
	/* Allocate initial data structure */
	if (*data_ptr == 0) {
		if (cfg->data_struct == 0) {  /* Linked List */
			list_head = bpf_arena_alloc_pages(&bench_arena, 0, sizeof(*list_head), BPF_ANY);
			if (list_head) {
				list_head->key = 0;
				list_head->value = 0;
				list_head->next = 0;
				*data_ptr = PTR_TO_ARENA_OFFSET(list_head);
			}
		} else if (cfg->data_struct == 1) {  /* Hash Table */
			hash_buckets = bpf_arena_alloc_pages(&bench_arena, 0, 256 * sizeof(*hash_buckets), BPF_ANY);
			if (hash_buckets) {
				*data_ptr = PTR_TO_ARENA_OFFSET(hash_buckets);
			}
		} else if (cfg->data_struct == 2) {  /* Tree */
			tree_root = bpf_arena_alloc_pages(&bench_arena, 0, sizeof(*tree_root), BPF_ANY);
			if (tree_root) {
				tree_root->key = 0xFFFFFFFFFFFFFFFFULL;  /* Sentinel */
				tree_root->value = 0;
				tree_root->left = 0;
				tree_root->right = 0;
				*data_ptr = PTR_TO_ARENA_OFFSET(tree_root);
			}
		}
	}
	
	/* Bulk insert phase */
	#pragma unroll(100)
	for (i = 0; i < cfg->num_test_keys && i < 100; i++) {
		test_key = bpf_map_lookup_elem(&test_keys, &i);
		if (!test_key)
			break;
		
		if (cfg->data_struct == 0) {  /* Linked List */
			list_head = ARENA_OFFSET_TO_PTR(*data_ptr);
			if (list_head)
				list_insert(list_head, *test_key, *test_key + 1000);
		} else if (cfg->data_struct == 1) {  /* Hash Table */
			hash_buckets = ARENA_OFFSET_TO_PTR(*data_ptr);
			if (hash_buckets)
				hash_insert(hash_buckets, *test_key, *test_key + 1000);
		}
	}
	
	/* Timed update loop - repeatedly perform updates */
	#pragma unroll(1000)
	for (i = 0; i < 1000; i++) {
		__u32 key_idx = i % cfg->num_test_keys;
		test_key = bpf_map_lookup_elem(&test_keys, &key_idx);
		if (!test_key)
			break;
		
		if (cfg->data_struct == 0) {  /* Linked List */
			list_head = ARENA_OFFSET_TO_PTR(*data_ptr);
			if (list_head)
				list_update(list_head, *test_key, i, cfg->lock_strategy);
		} else if (cfg->data_struct == 1) {  /* Hash Table */
			hash_buckets = ARENA_OFFSET_TO_PTR(*data_ptr);
			if (hash_buckets)
				hash_update(hash_buckets, *test_key, i, cfg->lock_strategy);
		} else if (cfg->data_struct == 2) {  /* Tree */
			tree_root = ARENA_OFFSET_TO_PTR(*data_ptr);
			if (tree_root)
				tree_update_handover(tree_root, *test_key, i);
		}
	}
	
	return 0;
}

char LICENSE[] SEC("license") = "GPL";
