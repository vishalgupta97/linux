// fentry_mymodule.bpf.c
// SPDX-License-Identifier: GPL-2.0
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

#define MAX_BUCKETS            1024
#define MAX_ENTRIES            65536
#define MAX_ENTRIES_PER_BUCKET 64

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

/* Entry storage: keyed by {bucket, index} */
struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, MAX_ENTRIES);
    __type(key, u32);
    __type(value, struct entry_val);
} entries SEC(".maps");

/* Loader writes num_buckets here at index 0 */
/* Loader writes entries_per_bucket here at index 1 */
struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, 2);
    __type(key, u32);
    __type(value, u32);
} num_bucket_config SEC(".maps");

static int write_entry_cb(u32 i, void *ctx)
{
    u32 *entry_key = ctx;

    struct entry_val *eval = bpf_map_lookup_elem(&entries, entry_key);
    if (!eval)
        return 0; /* skip missing entries, keep going */

    eval->value = *entry_key + i;
    return 0;
}

/* ─── fentry program ──────────────────────────────────────────── */

SEC("fentry/attach_cs_ht")
int BPF_PROG(trace_attach_cs_ht, u32 src_value, u32 dst_value, void *stats)
{
    /* Read num_buckets from config */
    u32 cfg_key = 0;
    u32 *num_buckets = bpf_map_lookup_elem(&num_bucket_config, &cfg_key);
    cfg_key = 1;
    u32 *entries_per_bucket = bpf_map_lookup_elem(&num_bucket_config, &cfg_key);
    if (!num_buckets || *num_buckets == 0)
        return 0;

    if(!entries_per_bucket || *entries_per_bucket == 0 || *entries_per_bucket > MAX_ENTRIES_PER_BUCKET)
	return 0;

    u32 src_bucket = src_value % *num_buckets;

    u32 lock_key = 0;
    struct global_lock_val *glv = bpf_map_lookup_elem(&global_lock_map, &lock_key);
    if (!glv)
        return 0;

    u32 entry_key = src_bucket * (*entries_per_bucket);

    bpf_spin_lock(&glv->lock);
	
    //bpf_loop(*entries_per_bucket, write_entry_cb, &entry_key, 0);
    bpf_loop(4, write_entry_cb, &entry_key, 0);


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
