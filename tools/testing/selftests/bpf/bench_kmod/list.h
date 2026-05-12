
noinline void bench_kmod_list_init(u32 pool_size)
{
	u32 i;

	if (!kmod_list_pool)
		return;
	for (i = 0; i < pool_size; i++) {
		spin_lock_init(&kmod_list_pool[i].lock);
		kmod_list_pool[i].next_idx = (u32)~0;
		kmod_list_pool[i].data = 0;
	}
	kmod_list_head_idx = (u32)~0;
}

noinline void bench_kmod_list_insert(u32 new_idx, u32 head_lock_idx)
{
	if (new_idx >= KMOD_MAX_POOL)
		return;

	spin_lock(&kmod_list_pool[head_lock_idx].lock);
	spin_lock(&kmod_list_pool[new_idx].lock);

	kmod_list_pool[new_idx].next_idx = kmod_list_head_idx;
	kmod_list_pool[new_idx].data     = ktime_get_mono_fast_ns();
	kmod_list_head_idx               = new_idx;

	spin_unlock(&kmod_list_pool[new_idx].lock);
	spin_unlock(&kmod_list_pool[head_lock_idx].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

/* Traverse the list to position (idx % list_length) and return its data. */
noinline u64 bench_kmod_list_lookup(u32 idx)
{
	u32 cur, i;
	u64 ret = 0;

	if (!kmod_list_pool)
		return 0;

	spin_lock(&kmod_list_pool[0].lock);
	cur = kmod_list_head_idx;
	for (i = 0; i < idx && cur != (u32)~0; i++)
		cur = kmod_list_pool[cur].next_idx;
	if (cur != (u32)~0)
		ret = kmod_list_pool[cur].data;
	spin_unlock(&kmod_list_pool[0].lock);

	this_cpu_inc(bench_cpu_stats.ops);
	return ret;
}

/* Traverse to position (idx % list_length) and overwrite data. */
noinline void bench_kmod_list_update(u32 idx, u64 val)
{
	u32 cur, i;

	if (!kmod_list_pool)
		return;

	spin_lock(&kmod_list_pool[0].lock);
	cur = kmod_list_head_idx;
	for (i = 0; i < idx && cur != (u32)~0; i++)
		cur = kmod_list_pool[cur].next_idx;
	if (cur != (u32)~0)
		kmod_list_pool[cur].data = val;
	spin_unlock(&kmod_list_pool[0].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

/* Remove the head node (O(1)); no-op when list is empty. */
noinline void bench_kmod_list_delete(void)
{
	u32 head;

	if (!kmod_list_pool)
		return;

	spin_lock(&kmod_list_pool[0].lock);
	head = kmod_list_head_idx;
	if (head != (u32)~0)
		kmod_list_head_idx = kmod_list_pool[head].next_idx;
	spin_unlock(&kmod_list_pool[0].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}


noinline void bench_undo_list_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_undo_list_init);

noinline void bench_undo_list_insert(u32 new_idx, u32 head_lock_idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_list_insert);


noinline u64 bench_undo_list_lookup(u32 idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_undo_list_lookup);

noinline void bench_undo_list_update(u32 idx, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_list_update);

noinline void bench_undo_list_delete(void)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_list_delete);

noinline void bench_arena_list_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_arena_list_init);

noinline void bench_arena_list_insert(u32 new_idx, u32 head_lock_idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_list_insert);

noinline u64 bench_arena_list_lookup(u32 idx)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_arena_list_lookup);

noinline void bench_arena_list_update(u32 idx, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_list_update);

noinline void bench_arena_list_delete(void)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_list_delete);
