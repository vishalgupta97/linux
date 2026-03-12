# BPF vs Kernel qspinlock Data Structure Microbenchmark — Implementation Plan

Date: 2026-03-11

---

## Context

This workspace contains a modified Linux kernel tree (`bpf-next` based) with custom BPF spinlock enhancements:

1. **Custom BPF qspinlock** (`kernel/bpf/bpf_qspinlock.c`): Replaces the legacy `arch_spin_lock` in `bpf_spin_lock` with a custom qspinlock slow path. The waiter at the MCS queue head (or a global kthread for uncontended case) manages the hrtimer timeout. See `bpf-spinlock-timeout-plan.md`.

2. **Per-CPU undo log** (`kernel/bpf/helpers.c`): Every memory write inside a `bpf_spin_lock` / `bpf_spin_unlock` critical section is recorded. On timeout, the log replays in reverse to restore atomicity-on-abort. Injected by the verifier via `do_misc_fixups()`. See `bpf-spinlock-undo-log.md`.

3. **BPF arena infrastructure**: The kernel supports `BPF_MAP_TYPE_ARENA` maps for user-managed memory in BPF programs. Existing headers provide an arena allocator (`bpf_arena_alloc.h`), intrusive linked list (`bpf_arena_list.h`), and chaining hash table (`bpf_arena_htab.h`). There is also a full BPF-side qspinlock implementation (`progs/bpf_arena_spin_lock.h`) but this benchmark uses the kernel-helper `bpf_spin_lock` instead.

4. **Benchmark framework**: `tools/testing/selftests/bpf/bench.c` provides a standard benchmark runner with producer/consumer threads, CPU affinity, warmup/duration timers, and per-second progress reporting. Benchmarks register via `struct bench` in `benchs/bench_*.c`.

5. **Kernel module test pattern**: `test_kmods/bpf_test_rqspinlock.c` demonstrates kthread creation with `kthread_create()` + `kthread_bind()`, per-CPU histograms, and module parameter configuration.

---

## Assumptions

1. **`bpf_spin_lock` works with arena data + BPF map locks**: Data structure nodes live in arena memory. The `bpf_spin_lock` is stored in `BPF_MAP_TYPE_ARRAY` map values (not embedded in arena nodes). Both the lock entry and arena node are separately cacheline-aligned (64 bytes). This is because `bpf_spin_lock` requires map-value storage — it cannot protect raw arena pointers directly.

2. **`SEC("syscall")` programs can use `bpf_spin_lock`**: Verified — `BPF_PROG_TYPE_SYSCALL` is neither a socket filter nor a tracing prog type, so the verifier allows spinlock usage. `bpf_get_prandom_u32()`, `bpf_ktime_get_ns()`, and arena kfuncs are all available.

3. **One arena map per program**: The verifier enforces this. A single arena map suffices for all data structure nodes plus the allocator's internal state.

4. **Nested `bpf_spin_lock` on distinct map values**: The verifier allows up to 32 simultaneously held `bpf_spin_lock`s on _different_ map values. This enables hand-over-hand locking where two locks from the `node_locks` array are held simultaneously during traversal.

5. **No nesting for per-bucket hash table**: Per user decision — a single bucket lock is acquired/released per operation, no nested locking.

6. **No existing arena BST implementation**: Must write `bpf_arena_bst.h` from scratch, following patterns from `bpf_arena_list.h` and `bpf_arena_htab.h`.

7. **Kernel module mirrors BPF layout for fair comparison**: Lock array separate from data nodes, same cacheline alignment, same algorithmic complexity. Kernel uses `spinlock_t` (which is `qspinlock` on x86) with `spin_lock_nested()` for hand-over-hand patterns.

8. **Batched BPF invocations**: Each `bpf_prog_test_run_opts()` call runs a batch of operations (default 64) to amortize entry/exit overhead. The kernel module's kthread runs continuously in a tight loop.

9. **Key size is fixed at 8 bytes (`__u64`)**: Value size is configurable up to 1024 bytes.

10. **Zipfian distribution**: Precomputed 64K-entry sampling table using inverse CDF of Zipf(s=1.0, N=num_keys). BPF side: populated by userspace into a `BPF_MAP_TYPE_ARRAY`. Kernel module: populated in `init_module`.

---

## File Map

### New Files

| File | Purpose |
|------|---------|
| `tools/testing/selftests/bpf/bpf_arena_bst.h` | Arena-based binary search tree header |
| `tools/testing/selftests/bpf/progs/bench_ds_lock_common.h` | Shared types/constants between BPF and userspace |
| `tools/testing/selftests/bpf/progs/bench_ds_lock.c` | BPF programs (setup + worker) |
| `tools/testing/selftests/bpf/benchs/bench_ds_lock.c` | Userspace benchmark driver |
| `tools/testing/selftests/bpf/test_kmods/bpf_test_ds_lock.c` | Kernel module benchmark |
| `tools/testing/selftests/bpf/benchs/run_bench_ds_lock.sh` | Parameter sweep script with CSV generation |

### Modified Files

| File | Change |
|------|--------|
| `tools/testing/selftests/bpf/Makefile` | Add skeleton dep + link rule for `bench_ds_lock` |
| `tools/testing/selftests/bpf/bench.c` | Register `bench_ds_lock` in `benchs[]` array |
| `tools/testing/selftests/bpf/test_kmods/Makefile` | Add `bpf_test_ds_lock.ko` to `MODULES` |

---

## Step-by-Step Implementation

### Step 1: Create `tools/testing/selftests/bpf/bpf_arena_bst.h` — Arena BST

Model after `bpf_arena_list.h` and `bpf_arena_htab.h`.

**Structures:**

```c
struct bst_node {
    __u64 key;
    __u32 lock_idx;          /* index into node_locks BPF map */
    __u32 __pad;
    struct bst_node __arena *left;
    struct bst_node __arena *right;
    char value[];            /* flexible trailing array, sized at alloc time */
} __attribute__((aligned(64)));

struct bst_root {
    struct bst_node __arena *node;
};
```

**Functions (all `__weak`, `__arg_arena`):**

- `bst_insert(struct bst_root __arena *root, __u64 key, void *value, __u32 val_size, __u32 lock_idx)`:
  - Allocate `bst_node` via `bpf_alloc(sizeof(struct bst_node) + val_size)` (round up to cacheline)
  - Iterative insertion: walk tree comparing keys, attach as left/right child
  - Use `cond_break` to bound the loop for the verifier
  - Assign `lock_idx` to the new node

- `bst_lookup(struct bst_root __arena *root, __u64 key)`:
  - Iterative BST search, returns `struct bst_node __arena *` or NULL
  - Use `cast_kern()` on each pointer dereference, `cond_break` loop bound

- `bst_update_value(struct bst_node __arena *node, void *new_value, __u32 val_size)`:
  - Bounded `memcpy` into `node->value` (caller holds lock)

Use `WRITE_ONCE` for pointer stores. Include `bpf_arena_common.h` and `bpf_arena_alloc.h`.

---

### Step 2: Create `tools/testing/selftests/bpf/progs/bench_ds_lock_common.h` — Shared Types

```c
#ifndef BENCH_DS_LOCK_COMMON_H
#define BENCH_DS_LOCK_COMMON_H

#define MAX_VALUE_SIZE     1024
#define MAX_KEYS           65536
#define HIST_BUCKETS       24      /* covers 64ns to ~500ms in log2 buckets */
#define ZIPF_TABLE_SIZE    65536
#define DEFAULT_BATCH_SIZE 64

enum ds_type {
    DS_LINKED_LIST  = 0,
    DS_HASH_TABLE   = 1,
    DS_BINARY_TREE  = 2,
};

enum lock_type {
    LOCK_GLOBAL      = 0,
    LOCK_PER_ELEMENT = 1,   /* linked list: one lock per list element */
    LOCK_PER_BUCKET  = 2,   /* hash table: one lock per bucket */
    LOCK_PER_NODE    = 3,   /* binary tree: one lock per tree node */
};

enum dist_type {
    DIST_UNIFORM  = 0,
    DIST_ZIPFIAN  = 1,
};

/* Valid combinations:
 *   linked_list:  global, per_element
 *   hash_table:   global, per_bucket
 *   binary_tree:  global, per_node
 */

#endif
```

---

### Step 3: Create `tools/testing/selftests/bpf/progs/bench_ds_lock.c` — BPF Programs

**Includes:**

```c
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include "bpf_arena_common.h"
#include "bpf_arena_alloc.h"
#include "bpf_arena_list.h"
#include "bpf_arena_htab.h"
#include "bpf_arena_bst.h"          /* new */
#include "bench_ds_lock_common.h"
```

**Maps:**

```c
struct {
    __uint(type, BPF_MAP_TYPE_ARENA);
    __uint(map_flags, BPF_F_MMAPABLE);
    __uint(max_entries, 1000);       /* pages — resized by userspace if needed */
    __ulong(map_extra, 0x1ull << 44);
} arena SEC(".maps");

struct lock_elem {
    struct bpf_spin_lock lock;
    char __pad[60];                  /* pad to 64 bytes = cacheline */
};

struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, 1);
    __type(key, __u32);
    __type(value, struct lock_elem);
} global_lock SEC(".maps");

struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, MAX_KEYS);   /* resized by userspace */
    __type(key, __u32);
    __type(value, struct lock_elem);
} node_locks SEC(".maps");

struct {
    __uint(type, BPF_MAP_TYPE_PERCPU_ARRAY);
    __uint(max_entries, 1);
    __type(key, __u32);
    __type(value, __u64);
} hits_map SEC(".maps");

struct {
    __uint(type, BPF_MAP_TYPE_PERCPU_ARRAY);
    __uint(max_entries, HIST_BUCKETS);
    __type(key, __u32);
    __type(value, __u64);
} latency_hist SEC(".maps");

struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, ZIPF_TABLE_SIZE);
    __type(key, __u32);
    __type(value, __u64);
} zipf_keys SEC(".maps");
```

**Configuration globals:**

```c
const volatile int cfg_ds_type = DS_LINKED_LIST;
const volatile int cfg_lock_type = LOCK_GLOBAL;
const volatile int cfg_num_keys = 1000;
const volatile int cfg_value_size = 64;
const volatile int cfg_distribution = DIST_UNIFORM;
const volatile int cfg_batch_size = DEFAULT_BATCH_SIZE;
```

**Arena data structure wrappers — Linked List:**

Extend the arena list with key-value storage:

```c
struct list_kv_node {
    __u64 key;
    __u32 lock_idx;
    __u32 __pad;
    struct arena_list_node list_node;
    char value[];
} __attribute__((aligned(64)));
```

Store the list head in a `.bss` global:

```c
struct arena_list_head __arena *list_head_ptr;
```

**Arena data structure wrappers — Hash Table:**

Reuse `struct htab` from `bpf_arena_htab.h`. Extend `hashtab_elem` with `lock_idx`:

Since modifying the upstream htab header is undesirable, create a local wrapper:

```c
struct htab_kv_elem {
    __u64 hash;
    __u64 key;
    __u32 lock_idx;       /* for per-element, unused for per-bucket */
    __u32 __pad;
    struct arena_list_node hash_node;
    char value[];
} __attribute__((aligned(64)));
```

Store htab pointer in `.bss`:

```c
struct htab __arena *htab_ptr;
```

NOTE: Since the existing `bpf_arena_htab.h` has a specific `hashtab_elem` layout, implement custom hash table lookup/insert functions in this file that work with `htab_kv_elem` instead. Follow the same bucket-chaining pattern.

**Arena data structure wrappers — BST:**

Use `struct bst_node` and `struct bst_root` from `bpf_arena_bst.h`.

Store root in `.bss`:

```c
struct bst_root __arena *bst_root_ptr;
```

**SEC("syscall") `bench_ds_lock_setup`:**

```
1. Switch on cfg_ds_type:
   - DS_LINKED_LIST:
     a. Allocate list_head in arena
     b. For i = 0..cfg_num_keys-1:
        - Allocate list_kv_node (sizeof base + cfg_value_size, rounded to 64B)
        - Set key = i, lock_idx = i (for per-element) or 0 (for global)
        - Zero-fill value
        - list_add_head(&node->list_node, list_head)
     c. Store list_head_ptr in bss

   - DS_HASH_TABLE:
     a. Allocate htab in arena, call custom htab_init()
     b. For i = 0..cfg_num_keys-1:
        - Allocate htab_kv_elem, set key = i, hash = i
        - lock_idx = bucket_index (for per-bucket) or 0 (for global)
        - Insert into bucket chain
     c. Store htab_ptr in bss

   - DS_BINARY_TREE:
     a. Allocate bst_root in arena
     b. For i = 0..cfg_num_keys-1:
        - bst_insert(root, key=i, zeroed_value, cfg_value_size, lock_idx=i or 0)
     c. Store bst_root_ptr in bss

2. Return 0 on success.
```

**SEC("syscall") `bench_ds_lock_work`:**

```
For b = 0..cfg_batch_size-1:
  1. Generate key:
     - DIST_UNIFORM:  key = bpf_get_prandom_u32() % cfg_num_keys
     - DIST_ZIPFIAN:  idx = bpf_get_prandom_u32() % ZIPF_TABLE_SIZE
                      lookup zipf_keys map at idx → key

  2. t_start = bpf_ktime_get_ns()

  3. Acquire lock(s) based on cfg_lock_type:

     - LOCK_GLOBAL:
       zero = 0
       lock_elem = bpf_map_lookup_elem(&global_lock, &zero)
       if (!lock_elem) continue
       bpf_spin_lock(&lock_elem->lock)

     - LOCK_PER_ELEMENT / LOCK_PER_BUCKET / LOCK_PER_NODE:
       (See traversal+locking patterns below per data structure)

  4. Lookup key in data structure, update value field with
     bounded bpf_probe_read_kernel or __builtin_memcpy of cfg_value_size bytes.

  5. Release lock(s) via bpf_spin_unlock()

  6. t_end = bpf_ktime_get_ns()
     delta = t_end - t_start
     bucket = log2(delta) - 6    (clamped to [0, HIST_BUCKETS-1])
     hist_val = bpf_map_lookup_elem(&latency_hist, &bucket)
     if (hist_val) (*hist_val)++

  7. hits_val = bpf_map_lookup_elem(&hits_map, &zero)
     if (hits_val) (*hits_val)++
```

**Per-element hand-over-hand (linked list):**

```
Traverse list from head:
  prev_lock_idx = -1
  For each node via list_for_each_entry:
    lock_idx = node->lock_idx
    lock_elem = bpf_map_lookup_elem(&node_locks, &lock_idx)
    bpf_spin_lock(&lock_elem->lock)         /* lock current */
    if (prev_lock_idx >= 0):
      prev_elem = bpf_map_lookup_elem(&node_locks, &prev_lock_idx)
      bpf_spin_unlock(&prev_elem->lock)     /* unlock previous */
    if (node->key == target_key):
      /* update value, unlock current, break */
    prev_lock_idx = lock_idx
    cond_break  /* verifier loop bound */
  If prev_lock_idx >= 0 and not found: unlock last
```

**Per-bucket (hash table, no nesting):**

```
bucket_idx = key % n_buckets    (or identity hash)
lock_elem = bpf_map_lookup_elem(&node_locks, &bucket_idx)
bpf_spin_lock(&lock_elem->lock)
/* traverse bucket chain, find key, update value */
bpf_spin_unlock(&lock_elem->lock)
```

**Per-node hand-over-hand (BST):**

```
Traverse tree from root:
  prev_lock_idx = -1
  cur = root->node
  While cur != NULL:
    cast_kern(cur)
    lock_idx = cur->lock_idx
    lock_elem = bpf_map_lookup_elem(&node_locks, &lock_idx)
    bpf_spin_lock(&lock_elem->lock)         /* lock current */
    if (prev_lock_idx >= 0):
      prev_elem = bpf_map_lookup_elem(&node_locks, &prev_lock_idx)
      bpf_spin_unlock(&prev_elem->lock)     /* unlock previous */
    if (cur->key == target_key):
      /* update value, unlock current, break */
    prev_lock_idx = lock_idx
    if (target_key < cur->key) cur = cur->left
    else cur = cur->right
    cond_break  /* verifier loop bound */
  If prev_lock_idx >= 0 and not found: unlock last
```

---

### Step 4: Create `tools/testing/selftests/bpf/benchs/bench_ds_lock.c` — Userspace Driver

**Custom CLI args (via `struct argp`):**

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--ds-type` | string | `linked_list` | Data structure: `linked_list`, `hash_table`, `binary_tree` |
| `--lock-type` | string | `global` | Lock strategy: `global`, `per_element`, `per_bucket`, `per_node` |
| `--num-keys` | int | `1000` | Number of keys to populate |
| `--value-size` | int | `64` | Value size in bytes (max 1024) |
| `--distribution` | string | `uniform` | Key distribution: `uniform`, `zipfian` |
| `--batch-size` | int | `64` | Operations per BPF invocation |

**`validate()`:**

- Check valid ds_type + lock_type combos (reject e.g. per_bucket + linked_list)
- Check value_size <= MAX_VALUE_SIZE
- Check num_keys <= MAX_KEYS
- Reject consumer_cnt != 0 (no consumer threads)

**`setup()`:**

```
1. skel = bench_ds_lock__open()
2. Set const volatile globals:
   skel->rodata->cfg_ds_type = ...
   skel->rodata->cfg_lock_type = ...
   skel->rodata->cfg_num_keys = ...
   skel->rodata->cfg_value_size = ...
   skel->rodata->cfg_distribution = ...
   skel->rodata->cfg_batch_size = ...

3. Resize maps:
   - node_locks: max_entries =
       LOCK_PER_ELEMENT / LOCK_PER_NODE: cfg_num_keys
       LOCK_PER_BUCKET: 2 * PAGE_SIZE / sizeof(htab_bucket)  [match arena htab]
       LOCK_GLOBAL: 1
   - arena: max_entries = enough pages (e.g. cfg_num_keys * node_size / PAGE_SIZE + 100)

4. bench_ds_lock__load()

5. Fault-in arena page 0:
   area = bpf_map__initial_value(skel->maps.arena, &arena_sz)
   *(volatile int *)area = 0x55aa

6. Run setup program:
   LIBBPF_OPTS(bpf_test_run_opts, opts)
   bpf_prog_test_run_opts(bpf_program__fd(skel->progs.bench_ds_lock_setup), &opts)
   Check opts.retval == 0

7. If DIST_ZIPFIAN:
   Precompute Zipfian table (inverse CDF, s=1.0, N=cfg_num_keys):
     For i = 0..ZIPF_TABLE_SIZE-1:
       u = (i + 0.5) / ZIPF_TABLE_SIZE
       key = inverse_zipf_cdf(u, s=1.0, N=cfg_num_keys)
       bpf_map_update_elem(zipf_keys_fd, &i, &key, BPF_ANY)

8. Store worker program fd for producer threads.
```

**`producer_thread(void *ctx)`:**

```
LIBBPF_OPTS(bpf_test_run_opts, opts)
while (!env.bench_done)
    bpf_prog_test_run_opts(worker_prog_fd, &opts)
return NULL
```

**`measure(struct bench_res *res)`:**

```
__u64 total_hits = 0
For each possible CPU (nr_cpus):
    Read hits_map[0] percpu values via bpf_map_lookup_elem
    Sum into total_hits
    Write 0 back (or use BPF_MAP_TYPE_PERCPU_ARRAY lookup+delete pattern)
res->hits = total_hits
res->duration_ns = ... (framework handles this)
```

Alternatively, use `atomic_swap` on a `.bss` counter if percpu map reading per-interval is too expensive. But percpu array avoids BPF-side atomics.

**`report_final(struct bench_res res[], int res_cnt)`:**

```
1. Print throughput: average hits/s across non-warmup intervals

2. Aggregate latency histogram:
   For each bucket 0..HIST_BUCKETS-1:
     Sum all percpu values from latency_hist map
   Compute total count from histogram
   Walk buckets to find p50, p99, p999 by CDF

3. Print:
   "Throughput: X.XXX Mops/s"
   "Latency p50: XXX ns  p99: XXX ns  p999: XXX ns"
```

**Registration:**

```c
const struct bench bench_ds_lock = {
    .name = "ds-lock",
    .argp = &ds_lock_argp,
    .validate = ds_lock_validate,
    .setup = ds_lock_setup,
    .producer_thread = ds_lock_producer,
    .measure = ds_lock_measure,
    .report_progress = ops_report_progress,
    .report_final = ds_lock_report_final,
};
```

---

### Step 5: Create `tools/testing/selftests/bpf/test_kmods/bpf_test_ds_lock.c` — Kernel Module

**Includes:**

```c
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/rbtree.h>       /* or open-code BST */
#include <linux/hashtable.h>
#include <linux/percpu.h>
#include <linux/ktime.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/sched.h>
```

**Module parameters:**

```c
static int ds_type = 0;          /* 0=list, 1=htab, 2=bst */
static int lock_type = 0;        /* 0=global, 1=per_element, 2=per_bucket, 3=per_node */
static int num_keys = 1000;
static int value_size = 64;
static int num_threads = 1;
static int runtime_sec = 10;
static int distribution = 0;     /* 0=uniform, 1=zipfian */
static int batch_size = 64;

module_param(ds_type, int, 0444);
module_param(lock_type, int, 0444);
module_param(num_keys, int, 0444);
module_param(value_size, int, 0444);
module_param(num_threads, int, 0444);
module_param(runtime_sec, int, 0444);
module_param(distribution, int, 0444);
module_param(batch_size, int, 0444);
```

**Data structures (all `__cacheline_aligned`):**

```c
struct lock_entry {
    spinlock_t lock;
} __cacheline_aligned;

struct kmod_list_node {
    struct list_head list;
    u64 key;
    u32 lock_idx;
    char value[] __aligned(8);
} __cacheline_aligned;

struct kmod_htab_elem {
    struct hlist_node hnode;
    u64 key;
    u32 lock_idx;
    char value[] __aligned(8);
} __cacheline_aligned;

struct kmod_htab_bucket {
    struct hlist_head chain;
} __cacheline_aligned;

struct kmod_bst_node {
    struct kmod_bst_node *left;
    struct kmod_bst_node *right;
    u64 key;
    u32 lock_idx;
    char value[] __aligned(8);
} __cacheline_aligned;
```

**Global state:**

```c
static struct lock_entry *lock_array;
static int num_locks;

/* Linked list */
static LIST_HEAD(kmod_list_head);

/* Hash table */
static struct kmod_htab_bucket *kmod_buckets;
static int kmod_n_buckets;

/* BST */
static struct kmod_bst_node *kmod_bst_root;

/* Zipfian table */
static u64 *zipf_table;

/* Per-CPU stats */
struct kmod_percpu_stats {
    u64 hits;
    u64 latency_hist[24];   /* HIST_BUCKETS */
} ____cacheline_aligned;

static struct kmod_percpu_stats __percpu *percpu_stats;

/* kthreads */
static struct task_struct **threads;
static ktime_t bench_end_time;
```

**`init_module` flow:**

```
1. Validate parameters (valid ds+lock combos, value_size <= 1024, etc.)

2. Allocate lock_array:
   - LOCK_GLOBAL: num_locks = 1
   - LOCK_PER_ELEMENT / LOCK_PER_NODE: num_locks = num_keys
   - LOCK_PER_BUCKET: num_locks = n_buckets (e.g., roundup_pow_of_two(num_keys))
   lock_array = kmalloc_array(num_locks, sizeof(struct lock_entry), GFP_KERNEL)
   For each: spin_lock_init(&lock_array[i].lock)

3. Build data structure with sequential keys 0..num_keys-1:
   - DS_LINKED_LIST: kmalloc each kmod_list_node (base + value_size),
                     list_add_tail() to kmod_list_head
   - DS_HASH_TABLE: kmalloc bucket array, kmalloc each elem,
                     hlist_add_head() to appropriate bucket
   - DS_BINARY_TREE: iterative BST insert (kmalloc each node)

4. If DIST_ZIPFIAN:
   zipf_table = kmalloc_array(ZIPF_TABLE_SIZE, sizeof(u64), GFP_KERNEL)
   Precompute using integer Zipf inverse CDF (s=1.0, N=num_keys)

5. percpu_stats = alloc_percpu(struct kmod_percpu_stats)

6. bench_end_time = ktime_add_ns(ktime_get(), runtime_sec * NSEC_PER_SEC)

7. threads = kmalloc_array(num_threads, sizeof(struct task_struct *), GFP_KERNEL)
   For i = 0..num_threads-1:
     threads[i] = kthread_create(worker_fn, (void *)(long)i, "ds_lock_%d", i)
     kthread_bind(threads[i], i % num_online_cpus())   /* pin to CPU */
     wake_up_process(threads[i])
```

**Worker kthread function:**

```c
static int worker_fn(void *data)
{
    struct kmod_percpu_stats *stats = this_cpu_ptr(percpu_stats);

    while (!kthread_should_stop() && ktime_before(ktime_get(), bench_end_time)) {
        for (int b = 0; b < batch_size; b++) {
            u64 key;

            /* Generate key */
            if (distribution == DIST_ZIPFIAN)
                key = zipf_table[get_random_u32() % ZIPF_TABLE_SIZE];
            else
                key = get_random_u32() % num_keys;

            u64 t_start = ktime_get_ns();

            /* Acquire lock(s), lookup, update, release — same patterns as BPF */
            switch (ds_type) {
            case 0: list_lookup_update(key, stats); break;
            case 1: htab_lookup_update(key, stats); break;
            case 2: bst_lookup_update(key, stats); break;
            }

            u64 delta = ktime_get_ns() - t_start;
            int bucket = ilog2(delta) - 6;
            if (bucket < 0) bucket = 0;
            if (bucket >= 24) bucket = 23;
            stats->latency_hist[bucket]++;
            stats->hits++;
        }
        cond_resched();
    }
    return 0;
}
```

**Hand-over-hand locking (kernel module):**

For per-element (list) and per-node (BST), use `spin_lock_nested()` with lockdep subclass 1 for the second lock to suppress false-positive lockdep warnings:

```c
spin_lock(&lock_array[cur_idx].lock);           /* subclass 0 (default) */
spin_lock_nested(&lock_array[next_idx].lock, 1); /* subclass 1 */
spin_unlock(&lock_array[cur_idx].lock);
/* ... advance ... */
```

**`cleanup_module` flow:**

```
1. For each thread: kthread_stop(threads[i])

2. Aggregate per-CPU stats:
   u64 total_hits = 0
   u64 total_hist[24] = {0}
   for_each_online_cpu(cpu):
     stats = per_cpu_ptr(percpu_stats, cpu)
     total_hits += stats->hits
     for each bucket: total_hist[b] += stats->latency_hist[b]

3. Compute p50/p99/p999 from total_hist CDF

4. Print parseable result:
   pr_info("DS_LOCK_RESULT: throughput=%llu duration=%d p50=%llu p99=%llu p999=%llu\n",
           total_hits, runtime_sec, p50_ns, p99_ns, p999_ns)

5. Free data structures, lock_array, zipf_table, percpu_stats, threads
```

---

### Step 6: Create `tools/testing/selftests/bpf/benchs/run_bench_ds_lock.sh` — Runner Script

```bash
#!/bin/bash
source $(dirname $0)/run_common.sh

RESULT_DIR="${1:-results}"

DS_TYPES=(linked_list hash_table binary_tree)
LOCK_MAP_LL=(global per_element)
LOCK_MAP_HT=(global per_bucket)
LOCK_MAP_BT=(global per_node)
NUM_KEYS=(100 1000 10000)
VALUE_SIZES=(8 64 256 1024)
THREADS=(1 2 4 8 16)
DISTRIBUTIONS=(uniform zipfian)
RUNTIME=10
WARMUP=3

# Map string names to integer codes for kernel module
declare -A DS_INT=( [linked_list]=0 [hash_table]=1 [binary_tree]=2 )
declare -A LOCK_INT=( [global]=0 [per_element]=1 [per_bucket]=2 [per_node]=3 )
declare -A DIST_INT=( [uniform]=0 [zipfian]=1 )

get_locks_for_ds() {
    case $1 in
        linked_list) echo "${LOCK_MAP_LL[@]}" ;;
        hash_table)  echo "${LOCK_MAP_HT[@]}" ;;
        binary_tree) echo "${LOCK_MAP_BT[@]}" ;;
    esac
}

CSV_FILE="$RESULT_DIR/results.csv"
echo "lock_impl,ds_type,lock_strategy,num_keys,value_size,threads,distribution,throughput_ops,p50_ns,p99_ns,p999_ns" > "$CSV_FILE"

for ds in "${DS_TYPES[@]}"; do
  for lock in $(get_locks_for_ds $ds); do
    for keys in "${NUM_KEYS[@]}"; do
      for vsize in "${VALUE_SIZES[@]}"; do
        for threads in "${THREADS[@]}"; do
          for dist in "${DISTRIBUTIONS[@]}"; do

            DIR="$RESULT_DIR/$ds/$lock/$keys/$vsize/$threads/$dist"
            mkdir -p "$DIR"

            # --- BPF benchmark ---
            echo "Running BPF: ds=$ds lock=$lock keys=$keys vsize=$vsize threads=$threads dist=$dist"
            sudo ./bench -w$WARMUP -d$RUNTIME -a -p$threads \
                --ds-type $ds --lock-type $lock --num-keys $keys \
                --value-size $vsize --distribution $dist \
                ds-lock > "$DIR/bpf.txt" 2>&1

            bpf_throughput=$(grep "Summary" "$DIR/bpf.txt" | sed -E 's/.*throughput=([0-9.]+).*/\1/')
            bpf_p50=$(grep "Summary" "$DIR/bpf.txt" | sed -E 's/.*p50=([0-9]+).*/\1/')
            bpf_p99=$(grep "Summary" "$DIR/bpf.txt" | sed -E 's/.*p99=([0-9]+).*/\1/')
            bpf_p999=$(grep "Summary" "$DIR/bpf.txt" | sed -E 's/.*p999=([0-9]+).*/\1/')
            echo "bpf,$ds,$lock,$keys,$vsize,$threads,$dist,$bpf_throughput,$bpf_p50,$bpf_p99,$bpf_p999" >> "$CSV_FILE"

            # --- Kernel module benchmark ---
            echo "Running kmod: ds=$ds lock=$lock keys=$keys vsize=$vsize threads=$threads dist=$dist"
            sudo insmod test_kmods/bpf_test_ds_lock.ko \
                ds_type=${DS_INT[$ds]} lock_type=${LOCK_INT[$lock]} \
                num_keys=$keys value_size=$vsize \
                num_threads=$threads runtime_sec=$RUNTIME \
                distribution=${DIST_INT[$dist]}

            sleep $((RUNTIME + 5))

            dmesg | grep "DS_LOCK_RESULT" | tail -1 > "$DIR/kmod.txt"
            sudo rmmod bpf_test_ds_lock

            kmod_throughput=$(sed -E 's/.*throughput=([0-9]+).*/\1/' "$DIR/kmod.txt")
            kmod_p50=$(sed -E 's/.*p50=([0-9]+).*/\1/' "$DIR/kmod.txt")
            kmod_p99=$(sed -E 's/.*p99=([0-9]+).*/\1/' "$DIR/kmod.txt")
            kmod_p999=$(sed -E 's/.*p999=([0-9]+).*/\1/' "$DIR/kmod.txt")
            echo "kmod,$ds,$lock,$keys,$vsize,$threads,$dist,$kmod_throughput,$kmod_p50,$kmod_p99,$kmod_p999" >> "$CSV_FILE"

          done
        done
      done
    done
  done
done

echo "Results written to $CSV_FILE"
```

---

### Step 7: Update Build Files

**`tools/testing/selftests/bpf/Makefile`:** Add near the existing skeleton dependency rules (~line 858):

```makefile
$(OUTPUT)/bench_ds_lock.o: $(OUTPUT)/bench_ds_lock.skel.h
```

Add `$(OUTPUT)/bench_ds_lock.o` to the `$(OUTPUT)/bench:` link rule (the `\`-continued list around line 840).

**`tools/testing/selftests/bpf/bench.c`:**

Add extern declaration near existing `extern const struct bench` declarations (~line 504):

```c
extern const struct bench bench_ds_lock;
```

Add `&bench_ds_lock` to the `benchs[]` array (~line 570+).

**`tools/testing/selftests/bpf/test_kmods/Makefile`:**

Add `bpf_test_ds_lock.ko` to the `MODULES` variable.

---

## Valid Parameter Combinations

| Data Structure | Valid Lock Strategies | Nested Locking |
|---|---|---|
| `linked_list` | `global`, `per_element` | `per_element`: hand-over-hand (max 2 locks held) |
| `hash_table` | `global`, `per_bucket` | None — single bucket lock per operation |
| `binary_tree` | `global`, `per_node` | `per_node`: hand-over-hand down tree (max 2 locks held) |

---

## Verification

1. **Build BPF benchmark**: `make -C tools/testing/selftests/bpf bench` — must compile cleanly
2. **Build kernel module**: `make -C tools/testing/selftests/bpf/test_kmods` — must produce `bpf_test_ds_lock.ko`
3. **Smoke test BPF** (single config):
   ```
   sudo ./bench -w1 -d3 -a -p1 --ds-type linked_list --lock-type global --num-keys 100 --value-size 64 ds-lock
   ```
   Expect non-zero throughput and latency percentiles.
4. **Smoke test kernel module**:
   ```
   sudo insmod bpf_test_ds_lock.ko ds_type=0 lock_type=0 num_keys=100 value_size=64 num_threads=1 runtime_sec=3
   sleep 8
   dmesg | grep DS_LOCK_RESULT
   sudo rmmod bpf_test_ds_lock
   ```
   Expect `DS_LOCK_RESULT:` line with non-zero throughput.
5. **Correctness check**: Run with 1 thread, 10 keys — after setup, walk arena data structure from userspace (via `.bss` pointer) to verify all keys present.
6. **Full sweep**: `bash benchs/run_bench_ds_lock.sh` — verify `results/results.csv` has 2 rows (bpf + kmod) per valid parameter combination (3 DS × 2 locks × 3 keys × 4 vsizes × 5 threads × 2 dists = 720 combos → 1440 rows).

---

## Decisions

- **Lock storage**: `bpf_spin_lock` lives in `BPF_MAP_TYPE_ARRAY` values, separate from arena data nodes. Kernel module mirrors this with a `lock_entry` array. Both are cacheline-aligned.
- **No per-bucket nesting**: Hash table per-bucket strategy acquires a single bucket lock per operation — no nested locking.
- **Hand-over-hand for list/tree**: Per-element (list) and per-node (BST) use 2-lock hand-over-hand traversal. Kernel module uses `spin_lock_nested()` with lockdep subclass.
- **Single BPF program, config via `const volatile`**: One `.c` file handles all 3 data structures via runtime config globals. No per-DS compilation needed.
- **Batched BPF invocations**: Default batch size of 64 operations per `bpf_prog_test_run_opts()` call amortizes syscall overhead.
- **Latency histogram**: 24 log2-scale buckets covering 64ns to ~500ms. Percentiles computed from aggregated per-CPU histograms.
- **Zipfian via precomputed table**: 64K-entry sampling table, Zipf(s=1.0, N=num_keys), avoids floating-point in BPF/kernel.
