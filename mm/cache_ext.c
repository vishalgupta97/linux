#include <linux/bpf_verifier.h>
#include <linux/bpf.h>
#include <linux/btf.h>
#include <linux/kernel.h>
#include <linux/memcontrol.h>
#include <linux/mm_types.h>
#include <linux/types.h>

static const struct btf_type *cache_ext_eviction_ctx_type;
static const struct btf_type *cache_ext_admission_ctx_type;

static int bpf_cache_ext_init(struct btf *btf)
{
	u32 eviction_type_id, admission_type_id;

	eviction_type_id = btf_find_by_name_kind(btf, "cache_ext_eviction_ctx",
					BTF_KIND_STRUCT);
	if (eviction_type_id < 0) {
		pr_err("cache_ext: failed to find struct cache_ext_eviction_ctx\n");
		return -EINVAL;
	}

	admission_type_id = btf_find_by_name_kind(btf, "cache_ext_admission_ctx",
					BTF_KIND_STRUCT);
	if (admission_type_id < 0) {
		pr_err("cache_ext: failed to find struct cache_ext_admission_ctx\n");
		return -EINVAL;
	}
	
	cache_ext_eviction_ctx_type = btf_type_by_id(btf, eviction_type_id);
	cache_ext_admission_ctx_type = btf_type_by_id(btf, admission_type_id);
	return 0;
}

// What helpers are available?
static const struct bpf_func_proto *
bpf_cache_ext_get_func_proto(enum bpf_func_id func_id,
				  const struct bpf_prog *prog)
{
	switch (func_id) {
	case BPF_FUNC_get_current_pid_tgid:
		return &bpf_get_current_pid_tgid_proto;
	default:
		return bpf_base_func_proto(func_id, prog);
	}
}

// This is the verifier's check for context access. Context is always R1.
static bool bpf_cache_ext_is_valid_access(int off, int size,
					       enum bpf_access_type type,
					       const struct bpf_prog *prog,
					       struct bpf_insn_access_aux *info)
{
	// 	return false;
	// if (type != BPF_READ)
	// 	return false;
	if (off % size != 0)
		return false;

	return btf_ctx_access(off, size, type, prog, info);
}

static int bpf_cache_ext_btf_struct_access(struct bpf_verifier_log *log,
						const struct bpf_reg_state *reg,
						int off, int size)
{
	// For each of the allowed types, allow read access.
	const struct btf_type *t;

	t = btf_type_by_id(reg->btf, reg->btf_id);
	if (t == cache_ext_eviction_ctx_type) {
		if (off + size > sizeof(struct cache_ext_eviction_ctx)) {
			bpf_log(log,
				"out of bounds access at off %d with size %d\n",
				off, size);
			return -EACCES;
		}
		return SCALAR_VALUE;
	} else if (t == cache_ext_admission_ctx_type) {
		if (off + size > sizeof(struct cache_ext_admission_ctx)) {
			bpf_log(log,
				"out of bounds access at off %d with size %d\n",
				off, size);
			return -EACCES;
		}
		return SCALAR_VALUE;
	}

	return -EACCES;
}

static int bpf_cache_ext_reg(void *kdata, struct bpf_link *link)
{
	struct cache_ext_ops *ops = kdata;
	struct cgroup *cgrp = bpf_cache_ext_link_to_cgroup(link);
	struct mem_cgroup *memcg;
	int ret = 0;

	if (!cgrp) {
		pr_err("cache_ext: cgroup is NULL\n");
		return -EINVAL;
	}

	// Get the memory cgroup
	memcg = mem_cgroup_from_css(cgrp->subsys[memory_cgrp_id]);
	if (memcg == NULL) {
		pr_crit("cache_ext: failed to get memcg for registration!\n");
		return -EINVAL;
	}
	pr_info("cache_ext: Calling init\n");
	if (ops->init) {
		ret = ops->init(memcg);
		if (ret) {
			pr_err("cache_ext: init failed with error code: %d\n",
			       ret);
			cache_ext_ds_registry_del_all(memcg);
			return ret;
		}
	}

	pr_info("cache_ext: Registering struct ops\n");
	return 0;
}

static void bpf_cache_ext_unreg(void *kdata, struct bpf_link *link)
{
	struct cgroup *cgrp = bpf_cache_ext_link_to_cgroup(link);
	struct mem_cgroup *memcg;

	pr_info("cache_ext: Unregistering struct ops\n");

	// Delete the registry and all data structures from the memory cgroup.
	if (!cgrp) {
		pr_err("cache_ext: cgroup is NULL\n");
		return;
	}
	
	memcg = mem_cgroup_from_css(cgrp->subsys[memory_cgrp_id]);
	if (memcg == NULL) {
		pr_crit("cache_ext: failed to get memcg for release!\n");
		return;
	}
	pr_info("cache_ext: unreg: Memcg pointer: %p\n", memcg);
	cache_ext_ds_registry_del_all(memcg);
}

static int bpf_cache_ext_init_member(const struct btf_type *t,
					  const struct btf_member *member,
					  void *kdata, const void *udata)
{
	// Initialize attributes of struct_ops. Not for the function pointers.
	// For example, sched_ext uses it to initialize timeout, flags, etc.
	// Not needed yet.
	return 0;
}

static int bpf_cache_ext_check_member(const struct btf_type *t,
					   const struct btf_member *member,
					   const struct bpf_prog *prog)
{
	// Check the attached member functions or attributes.
	// TODO: Check sleepability here!
	return 0;
}

static int bpf_cache_ext_validate(void *kdata)
{
	return 0;
}

static const struct bpf_verifier_ops bpf_cache_ext_verifier_ops = {
	.get_func_proto = bpf_cache_ext_get_func_proto,
	.is_valid_access = bpf_cache_ext_is_valid_access,
	.btf_struct_access = bpf_cache_ext_btf_struct_access,
};

/*
 * CFI stubs. v7.0 requires struct_ops to provide a vtable of stub functions
 * (used as indirect-call CFI targets for the BPF programs).
 */
static s32 __cache_ext_init(struct mem_cgroup *memcg)
{
	return 0;
}
static void __cache_ext_evict_folios(struct cache_ext_eviction_ctx *ctx,
				     struct mem_cgroup *memcg)
{
}
static void __cache_ext_folio_added(struct folio *folio,
				    mem_cgroup_per_node_bpf_writable *pn)
{
}
static void __cache_ext_folio_accessed(struct folio *folio)
{
}
static void __cache_ext_folio_evicted(struct folio *folio)
{
}
static bool __cache_ext_admit_folio(struct cache_ext_admission_ctx *ctx)
{
	return false;
}

static struct cache_ext_ops __bpf_cache_ext_ops = {
	.init = __cache_ext_init,
	.evict_folios = __cache_ext_evict_folios,
	.folio_added = __cache_ext_folio_added,
	.folio_accessed = __cache_ext_folio_accessed,
	.folio_evicted = __cache_ext_folio_evicted,
	.admit_folio = __cache_ext_admit_folio,
};

// Callbacks to define the struct_ops map
struct bpf_struct_ops bpf_cache_ext_ops = {
	.verifier_ops = &bpf_cache_ext_verifier_ops,
	.init = bpf_cache_ext_init,
	.reg = bpf_cache_ext_reg,
	.unreg = bpf_cache_ext_unreg,
	.init_member = bpf_cache_ext_init_member,
	.check_member = bpf_cache_ext_check_member,
	.validate = bpf_cache_ext_validate,
	.cfi_stubs = &__bpf_cache_ext_ops,
	.owner = THIS_MODULE,
	.name = "cache_ext_ops",
};

static int __init bpf_cache_ext_struct_ops_init(void)
{
	return register_bpf_struct_ops(&bpf_cache_ext_ops, cache_ext_ops);
}
late_initcall(bpf_cache_ext_struct_ops_init);
