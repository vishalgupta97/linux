// fentry_mymodule.bpf.c
// SPDX-License-Identifier: GPL-2.0
#define BPF_NO_KFUNC_PROTOTYPES
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_experimental.h"
#include "bpf_arena_common.h"

#define MAX_BUCKETS            1024
#define MAX_ENTRIES            65536
#define MAX_ENTRIES_PER_BUCKET 64
#define NUM_ENTRIES 4          
#define ENTRIES_ARENA_BYTES    (MAX_ENTRIES * sizeof(struct entry_val))
#define ENTRIES_ARENA_PAGES    ((ENTRIES_ARENA_BYTES + PAGE_SIZE - 1) / PAGE_SIZE)

/* ─── Map key/value types ─────────────────────────────────────── */

struct entry_val {
    union {
    	u64 value;
	char __padding[64];
    };
};

/* Wrapper that carries the bpf_spin_lock.
 * A bpf_spin_lock MUST be embedded in a map value; it cannot be
 * a bare global.  We use a 1-element ARRAY as a "global" lock. */
struct global_lock_val {
    struct bpf_spin_lock lock;
};

/* ─── Maps ────────────────────────────────────────────────────── */

/* The single global lock */
struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, 1);
    __type(key, u32);
    __type(value, struct global_lock_val);
} global_lock_map SEC(".maps");

/* Arena backing for entry storage */
struct {
    __uint(type, BPF_MAP_TYPE_ARENA);
    __uint(map_flags, BPF_F_MMAPABLE);
    __uint(max_entries, ENTRIES_ARENA_PAGES);
#ifdef __TARGET_ARCH_arm64
    __ulong(map_extra, 0x1ull << 32);
#else
    __ulong(map_extra, 0x1ull << 44);
#endif
} arena SEC(".maps");

struct entry_val __arena *entries_base;
__u32 entries_initialized;
__u32 init_total_entries;

static __always_inline struct entry_val __arena *get_entry_ptr(u32 entry_key)
{
    struct entry_val __arena *base = entries_base;

    void __arena *ab = arena_base(&arena);

    if (!ab || !entries_initialized || !base || entry_key >= MAX_ENTRIES) {
	bpf_printk("return null entry_ptr key: %d\n", entry_key);
        return NULL;
    }

    cast_kern(base);
    return &base[entry_key];
}

SEC("syscall")
int init_entries_arena(void *ctx)
{
    struct entry_val __arena *base;
    u32 i;
    u32 total_entries;

    if (entries_initialized)
        return 0;

    base = bpf_arena_alloc_pages(&arena, NULL, ENTRIES_ARENA_PAGES,
                                 NUMA_NO_NODE, 0);
    if (!base)
        return 0;

    entries_base = base;
    cast_kern(base);

    total_entries = init_total_entries;
    if (!total_entries || total_entries > MAX_ENTRIES)
        total_entries = MAX_ENTRIES;

    for (i = 0; i < total_entries; i++)
        base[i].value = i;

    entries_initialized = 1;
    return 0;
}

/* ─── fentry program ──────────────────────────────────────────── */

SEC("fentry/attach_cs_ht")
int BPF_PROG(trace_attach_cs_ht, u32 src_value, u32 dst_value, void *stats)
{
    u32 src_bucket = src_value % 1024;

    u32 lock_key = 0;
    struct global_lock_val *glv = bpf_map_lookup_elem(&global_lock_map, &lock_key);
    if (!glv)
        return 0;

    bpf_spin_lock(&glv->lock);
	
    for(int i = 0; i < NUM_ENTRIES; i++)
	{
	    u32 entry_key = (src_bucket * NUM_ENTRIES) + i;

        struct entry_val __arena *eval = get_entry_ptr(entry_key);
	    if (!eval) {
		bpf_printk("no entry\n");
		goto end;
            }
        cast_kern(eval);
	    eval->value = dst_value + i;
	}
end:
    bpf_spin_unlock(&glv->lock);

    return 0;
}

SEC("fentry/init_ht")
int BPF_PROG(trace_init_ht)
{
    __u64 pid_tgid = bpf_get_current_pid_tgid();
    __u32 pid  = (__u32)(pid_tgid & 0xFFFFFFFF);
    __u32 tgid = (__u32)(pid_tgid >> 32);
    __u32 cpu  = bpf_get_smp_processor_id();

    bpf_printk("fentry init_ht: pid=%u tgid=%u cpu=%u\n",
               pid, tgid, cpu);
    return 0;
}

char LICENSE[] SEC("license") = "GPL";
