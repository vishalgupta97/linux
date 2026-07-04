#ifndef _LINUX_CACHE_EXT_H
#define _LINUX_CACHE_EXT_H 1

/*
 * BPF-Exposed data structures for cache_ext.
 */

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
/*
 * Note: deliberately do NOT include <linux/hashtable.h> here. This header is
 * pulled in transitively by <linux/memcontrol.h> (which is included almost
 * everywhere), and hashtable.h defines function-like macros (hash_add, ...)
 * that collide with same-named local helpers in unrelated files (e.g.
 * kernel/trace/ftrace.c). DECLARE_HASHTABLE is expanded manually below so the
 * struct layout is unchanged; .c files that operate on these tables include
 * <linux/hashtable.h> directly.
 */
// #include <linux/bpf.h>

typedef u64 (*bpf_callback_t)(u64, u64, u64, u64, u64);

bool cache_ext_is_callback_calling_kfunc_iterate(u32 btf_id);
bool cache_ext_is_callback_calling_kfunc_sample(u32 btf_id);

/*
 * cache_ext struct_ops per-cgroup attach: the custom bpf_link carries the
 * target cgroup. Implemented in kernel/bpf/bpf_struct_ops.c.
 */
struct bpf_link;
struct cgroup *bpf_cache_ext_link_to_cgroup(struct bpf_link *link);

/******************************************************************************
 * Linked List ****************************************************************
 *****************************************************************************/

/*
 * Indexed Linked List.
 *
 * This is a linked list that is indexed by the folio it contains. This is
 * because we need to be able to quickly:
 * - Find the folio corresponding to a given node.
 * - Find the node corresponding to a given folio.
 *
 * Instead of maintaining a hash-table per list, we can piggyback on the valid
 * folios hashtable we already maintain. It will also keep a pointer to the node
 * in the valid_folio struct.
 */
struct cache_ext_list {
	struct list_head head;
	// This is for the ds registry.
	struct hlist_node h_node;
};

struct cache_ext_list_node {
	struct folio *folio;

	struct list_head node;
};

/*
 * BPF API
 */

struct sampling_options {
	__u32 sample_size;
	__u32 select_size;
};

int bpf_cache_ext_list_add(u64 list, struct folio *folio);
int bpf_cache_ext_list_add_tail(u64 list, struct folio *folio);
int bpf_cache_ext_list_move(u64 list, struct folio *folio, bool tail);
int bpf_cache_ext_list_del(struct folio *folio);
int bpf_cache_ext_list_iterate(struct mem_cgroup *memcg, u64 list,
			       int(iter_fn)(int idx,
					    struct cache_ext_list_node *node),
			       struct cache_ext_eviction_ctx *ctx);
int bpf_cache_ext_list_sample(struct mem_cgroup *memcg, u64 list,
			      s64(score_fn)(struct cache_ext_list_node *a),
				  struct sampling_options *opts,
				  struct cache_ext_eviction_ctx *ctx);
u64 bpf_cache_ext_ds_registry_new_list(struct mem_cgroup *memcg);
u64 bpf_cache_ext_registry_lock_addr(struct mem_cgroup *memcg);

/*
 * Used by the valid_folios_set code
 */
struct cache_ext_list_node *cache_ext_list_node_alloc(struct folio *folio);
void cache_ext_list_node_free(struct cache_ext_list_node *node);

/*
 * cache_ext data structure registry.
 */

#define CACHE_EXT_REGISTRY_MAX_ENTRIES 5

// NOTE: For now, tie the registry lifetime to the struct_ops lifetime.
// Release all the data structures when the struct_ops is released.
// Do not permit any structure to be released while the struct_ops is
// still in use.
struct cache_ext_ds_registry {
	/* DECLARE_HASHTABLE(ds_hash, 5) expanded to avoid including hashtable.h */
	struct hlist_head ds_hash[1 << 5];
	/*
	 * Single lock guarding both this registry's ds_hash and the per-folio
	 * cache_ext_list linkage. It is shared with pure-BPF policies: the kernel
	 * takes it with spin_lock()/spin_unlock(), while a policy takes the SAME
	 * lock word via bpf_spin_lock()/bpf_spin_unlock() over a writable-cast
	 * (struct bpf_spin_lock *) pointer (see cache_ext_ds.bpf.h). bpf_spin_lock()
	 * uses the standard qspinlock word format, so the two paths interoperate. A
	 * kernel waiter blocked behind a hung BPF lock holder is unblocked by the
	 * BPF spin-lock timeout/termination machinery, which forcibly releases the
	 * holder.
	 */
	spinlock_t lock;
	int nr_entries;
};

void cache_ext_ds_registry_init(struct cache_ext_ds_registry *registry);
void cache_ext_ds_registry_read_lock(struct folio *folio);
void cache_ext_ds_registry_read_unlock(struct folio *folio);
void cache_ext_ds_registry_write_lock(struct folio *folio);
void cache_ext_ds_registry_write_unlock(struct folio *folio);
void cache_ext_ds_registry_del_all(struct mem_cgroup *memcg);
struct cache_ext_list *cache_ext_ds_registry_new_list(struct mem_cgroup *memcg);
struct cache_ext_list *
cache_ext_ds_registry_get(struct cache_ext_ds_registry *registry, u64 list_ptr);
struct cache_ext_ds_registry *
cache_ext_ds_registry_from_folio(struct folio *folio);
struct cache_ext_ds_registry *
cache_ext_ds_registry_from_memcg(struct mem_cgroup *memcg);
#endif // _LINUX_CACHE_EXT_H
