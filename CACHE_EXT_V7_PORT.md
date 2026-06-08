# cache_ext forward-port to Linux v7.0

This document summarizes the forward-port of **cache_ext** (BPF-customizable page-cache
eviction policy) from its original Linux **v6.6.8** base onto this **v7.0** tree, which
already carries BPF spinlock timeout/termination + undo-logging. It also records the
debugging steps taken to get the existing kfunc-based policies (FIFO, sampling, s3fifo)
loading and attaching on v7.0.

## Why forward-port (instead of backporting)

The end goal is to implement the cache_ext data-structure API in BPF (not kernel kfuncs),
which requires the kernel to safely terminate a misbehaving BPF program holding a lock and
roll back its writes. That machinery (termination + undo logging, built on BPF qspinlock +
`bpf_throw`) already exists on this v7.0 tree. Backporting it into v6.6.8 would mean porting
a fragile verifier/JIT feature stack into an older BPF subsystem — the highest-risk path.
cache_ext itself is ~1,900 lines, mostly in self-contained files, so forward-porting it onto
v7.0 is the smaller, safer move and lands the project on the kernel where the end-goal infra
is first-class.

## Scope

Branch: `cache-ext-v7` (base `e2510ca212 wip: fpop`). All 23 functional files changed by the
v6.6.8 cache_ext tree (audited against base `e6dc9f5e8a73 "Linux v6.6.8"`) are present here,
adapted to v7.0 APIs. ~28 files, ~+2,180 lines.

---

## Kernel changes by area

### 1. Core data structures (self-contained, near drop-in)
- **`mm/cache_ext.c`** — struct_ops glue + verifier ops (`get_func_proto`,
  `is_valid_access`, `btf_struct_access`), the `init`/`reg`/`unreg` callbacks, CFI stubs, and
  the `register_bpf_struct_ops()` registration.
- **`mm/cache_ext_ds.c`** — the indexed linked-list + per-memcg DS registry, and the kfuncs
  `bpf_cache_ext_list_add/_add_tail/_del/_move/_iterate/_iterate_extended/_sample` and
  `bpf_cache_ext_ds_registry_new_list`.
- **`include/linux/cache_ext.h`** — DS structs + kfunc/registry prototypes.
- **`include/linux/mm_types.h`** — `cache_ext_ops` vtable, `cache_ext_eviction_ctx`,
  `cache_ext_admission_ctx`.
- **`include/linux/memcontrol.h`** — `valid_folios_set` (folio-validity hashtable) + per-node
  `valid_folios_set`/`cache_ext_ds_registry` embedded in `mem_cgroup_per_node`, and
  `cache_ext_valid` on `mem_cgroup`.
- **`include/linux/bpf-cgroup-defs.h`** — per-cgroup `cache_ext_enabled` / `cache_ext_ops` /
  `cache_ext_sem`.

### 2. struct_ops per-cgroup attach (rewritten for v7.0)
v6.6.8 changed the *global* struct_ops `.reg(kdata)` signature to thread a cgroup. v7.0 keeps
the native `.reg(kdata, struct bpf_link *)`, so instead:
- **`kernel/bpf/bpf_struct_ops.c`** — a custom `bpf_cache_ext_ops_link` (a `bpf_link`
  subclass carrying the target cgroup), its link-ops (dealloc/show_fdinfo/fill_link_info/
  update_map), `bpf_cache_ext_ops_link_create()` (gets the cgroup from `target_fd`, calls
  `st_ops_desc->st_ops->reg(kdata, &link->link)`, enables cache_ext on the cgroup), and a
  `bpf_cache_ext_link_to_cgroup()` accessor.
- **`mm/cache_ext.c`** — `reg`/`unreg` take the `bpf_link` and recover the cgroup via the
  accessor. Registered via `register_bpf_struct_ops(&bpf_cache_ext_ops, cache_ext_ops)`
  (v7.0 BTF_ID-based; the old `bpf_struct_ops_types.h` was removed upstream). `.cfi_stubs`
  added (mandatory in v7.0).
- **`include/uapi/linux/bpf.h`** — new `BPF_CACHE_EXT_OPS` attach type,
  `BPF_LINK_TYPE_CACHE_EXT_OPS = 15`, and `bpf_link_info.cache_ext_ops`.
- **`include/linux/bpf_types.h`** — `BPF_LINK_TYPE(BPF_LINK_TYPE_CACHE_EXT_OPS, cache_ext_ops)`.
- **`include/linux/bpf.h`** — `bpf_cache_ext_ops_link_create()` prototype.
- **`kernel/bpf/syscall.c`** — dispatch `BPF_CACHE_EXT_OPS` in `link_create()`.

### 3. mm hooks (re-placed at v7.0 sites)
- **`mm/swap.c`** — `folio_accessed` hook in `folio_mark_accessed`.
- **`mm/filemap.c`** — `folio_added` (in `__filemap_add_folio`) + `valid_folios_add`;
  `folio_evicted` (in `__filemap_remove_folio` and the batch remove) + `valid_folios_del`;
  the **admit_folio** admission hook in `filemap_get_pages`, `__cache_ext_dio` /
  `filemap_read_folio_cache_ext` out-of-cache read path, and the `cache_ext_flag` threading
  in `filemap_read`. (Adapted: removed `folio_clear_error`/`PG_error`; `filemap_alloc_folio`
  is now 3-arg; v7.0 dropbehind `put_folios` loop.)
- **`mm/vmscan.c`** — `cache_ext_isolate_folio`, `__cache_ext_isolate_and_reclaim`,
  `cache_ext_isolate_and_reclaim`, wired into `shrink_lruvec` (with corrected `nr_reclaimed`
  accounting for v7.0's single `sc->nr_reclaimed += nr_reclaimed` at function end).
- **`mm/memcontrol.c`** — the `valid_folios_set` implementation (init/add/del/exists/clear/
  lookup), `get_cache_ext_ops`, `cache_ext_cgroup_enabled`, `add_cache_ext_structures`
  (per-node alloc) wired into `mem_cgroup_css_online` (matches cgroups named `cache_ext*`),
  and the per-node free path.
- **`kernel/bpf/cgroup.c`** — initialize `cache_ext_sem`/`enabled`/`ops` in
  `cgroup_bpf_inherit`.

### 4. Verifier support
- **`kernel/bpf/verifier.c`** — callback-state setters for the iterate (ret range 0..2) and
  sample (s64 score) kfuncs; cache_ext kfuncs added to `is_sync_callback_calling_kfunc` (so
  `mark_calls_callback` fires); dispatch in `check_kfunc_call`. The sample callback returns an
  arbitrary s64, so it is marked `callback_ret_unrestricted` to skip the s32
  `callback_ret_range` enforcement in `prepare_func_exit`.
- **`include/linux/bpf_verifier.h`** — `callback_ret_unrestricted` flag on `bpf_func_state`
  (placed before `stack` so `copy_func_state`'s memcpy preserves it).
- **`include/linux/btf_ids.h`** — `BTF_TRACING_TYPE_CACHE_EXT_LIST_NODE` (so the verifier can
  hand callbacks a typed `struct cache_ext_list_node *`).
- **`kernel/trace/bpf_trace.c`** — `BTF_ID(func, vfs_open)` added to `btf_allowlist_d_path`
  (the dir_watcher's `fexit/vfs_open` calls `bpf_d_path`).

### 5. Misc / build
- **`include/linux/hashtable.h`** — `hash_bucket_idx()` macro (used by the bucket-locked
  folio set). Note: cache_ext.h/memcontrol.h **expand `DECLARE_HASHTABLE` manually** and do
  *not* include `<linux/hashtable.h>`, because pulling its function-like macros (`hash_add`,
  …) through the widely-included `memcontrol.h` collides with same-named local helpers in
  unrelated files (e.g. v7.0 `kernel/trace/ftrace.c`'s `hash_add()`). The `.c` files include
  `<linux/hashtable.h>` directly.
- **`include/trace/events/filemap.h` + `mm/readahead.c`** —
  `mm_filemap_add_to_page_cache_prefetch` tracepoint at 4 readahead sites.
- **`mm/Makefile`** — build `cache_ext.o` + `cache_ext_ds.o`.

### 6. Userspace libbpf (patched)
- **`tools/lib/bpf/libbpf.{c,h,map}` + `tools/include/uapi/linux/bpf.h`** —
  `bpf_map__attach_cache_ext_ops(map, cgroup_fd)` (issues `BPF_LINK_CREATE` with
  `BPF_CACHE_EXT_OPS` + `target_fd = cgroup_fd`), the `BPF_CACHE_EXT_OPS` /
  `BPF_LINK_TYPE_CACHE_EXT_OPS` enums + name-table entries, and — importantly — the skeleton
  auto-attach loop (`bpf_object__attach_skeleton`) **skips struct_ops maps of type
  `cache_ext_ops`**, because v7.0 libbpf auto-attaches struct_ops maps generically (v6.6.8 did
  not) and the generic path has no cgroup. Built + installed to `/usr/local`.

### Intentionally NOT ported (N/A on v7.0)
- `net/bpf/bpf_dummy_struct_ops.c`, `net/ipv4/bpf_tcp_ca.c`, and the `bpf.h` `.reg/.unreg`
  signature change — v6.6.8 changed the global struct_ops reg signature, forcing those edits;
  v7.0 keeps the native `.reg(kdata, bpf_link*)`, so they need no change.
- `kernel/bpf/bpf_struct_ops_types.h` — removed upstream; replaced by
  `register_bpf_struct_ops()`.

---

## Debugging journey (what broke, and the fix)

The forward-port compiled object-by-object, but bringing up a live policy surfaced a chain of
"v7.0 moved the goalposts since v6.6.8" issues. In order:

1. **Build break — `hash_add` macro collision.** Putting `<linux/hashtable.h>` in `cache_ext.h`
   leaked the `hash_add` macro (via `memcontrol.h`) into v7.0's `kernel/trace/ftrace.c`, which
   has a local `hash_add()` function. **Fix:** expand `DECLARE_HASHTABLE` manually in the
   headers; include `<linux/hashtable.h>` only in `.c` files.

2. **struct_ops API drift.** `bpf_base_func_proto` is 2-arg in v7.0; `.reg/.unreg` take a
   `bpf_link`; `.cfi_stubs` is mandatory; registration is BTF_ID-based. **Fix:** rewrote the
   descriptor + custom link (above).

3. **Policy BPF compile drift.** `page->flags` is now `memdesc_flags_t { unsigned long f; }`
   and `PG_hugetlb` became the page-type `PGTY_hugetlb`. Also the installed libbpf
   `bpf_helpers.h` redeclared `bpf_stream_vprintk` (clashing with vmlinux.h). **Fix:**
   `policies/cache_ext_lib.bpf.h` uses `page->flags.f` and `page_type >> 24 == PGTY_hugetlb`;
   `policies/Makefile` seds the duplicate decl out of the generated `vmlinux.h` and adds
   `-I/usr/local/include`. Installed `clang-14` (project-pinned).

4. **`bpf_d_path` rejected in the dir_watcher.** The watch-dir mechanism uses `fexit/vfs_open`
   + `bpf_d_path`, which v7.0 rejects ("helper not allowed in probe") unless the attach target
   is allowlisted. **Fix:** add `vfs_open` to `btf_allowlist_d_path` (kernel rebuild).

5. **Generic struct_ops auto-attach → "cgroup is NULL".** v7.0 libbpf's skeleton attach
   auto-attaches struct_ops maps generically; for cache_ext that path has no cgroup and `reg`
   fails. **Fix:** skip `cache_ext_ops` maps in the libbpf skeleton-attach loop (userspace
   rebuild, no kernel reboot).

6. **`verifier bug: ... !calls_callback`.** The iterate callback was recognized by the
   *general* `is_callback_calling_kfunc` but not `is_sync_callback_calling_kfunc`, so the CFG
   pass never set the `calls_callback` flag. **Fix:** add the cache_ext kfuncs to
   `is_sync_callback_calling_kfunc`.

7. **Sample policy: "At callback return … should have been in [-2147483648, 2147483647]".**
   The sampling score callback returns s64 (an `INT64_MAX` sentinel), but v7.0's
   `callback_ret_range` is s32. **Fix:** the `callback_ret_unrestricted` flag (above). The
   accompanying "Global function … doesn't return scalar" log line is **non-fatal** (v7.0
   marks the main struct_ops program's BTF "unreliable" and continues).

After these, **FIFO, sampling, and s3fifo all load, verify, attach to a cgroup, and detach
cleanly** (dmesg: `Calling init` → `Registering struct ops` with a valid cgroup, then
`Unregistering struct ops` on exit).

---

## Build / test

```sh
# Kernel (long; use screen). Install + reboot is a manual step.
cd /home/vishal/ebpf/linux
make -j"$(nproc)" vmlinux            # or a full bzImage build
sudo make modules_install && sudo make install && sudo reboot

# Patched libbpf (userspace only — no reboot)
cd tools/lib/bpf && make -j
sudo make install_lib install_headers prefix=/usr/local libdir=/usr/local/lib64 && sudo ldconfig

# Policies (on the booted cache_ext kernel)
cd /home/vishal/ebpf/cache_ext/policies && make -j     # needs clang-14 + /usr/local/sbin/bpftool

# Smoke test (cgroup named cache_ext* triggers add_cache_ext_structures on css_online)
sudo mkdir -p /sys/fs/cgroup/cache_ext_test
mkdir -p /tmp/ce_watch && echo hi > /tmp/ce_watch/f
sudo ./cache_ext_fifo.out --watch_dir /tmp/ce_watch --cgroup_path /sys/fs/cgroup/cache_ext_test
# s3fifo also needs: --cgroup_size <bytes>
# Expect dmesg: "cache_ext: Calling init" / "Registering struct ops" (no "cgroup is NULL").
```

## Status & next steps
- **Done:** full kernel + libbpf forward-port; FIFO/sampling/s3fifo attach + detach on v7.0.
- **Next:** eviction-under-memory-pressure validation via the `bench/` drivers; then expose
  folios through the verifier; then reimplement the list/sample/registry kfuncs as pure BPF
  (now safe to build on, since termination + undo logging are native to this kernel).
