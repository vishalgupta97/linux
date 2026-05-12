
noinline void bench_kmod_ring_init(u32 num_slots)
{
	u32 i;

	if (!kmod_ring_pool)
		return;
	kmod_ring_size = num_slots;
	atomic_set(&kmod_ring_head, 0);
	for (i = 0; i < num_slots; i++) {
		spin_lock_init(&kmod_ring_pool[i].lock);
		kmod_ring_pool[i].valid = 0;
		kmod_ring_pool[i].data  = 0;
	}
}

noinline void bench_kmod_ring_enqueue(u64 val)
{
	u32 slot;

	if (!kmod_ring_pool || !kmod_ring_size)
		return;
	slot = (u32)(atomic_fetch_add(1, &kmod_ring_head) % kmod_ring_size);

	spin_lock(&kmod_ring_pool[slot].lock);
	kmod_ring_pool[slot].data  = val;
	kmod_ring_pool[slot].valid = 1;
	spin_unlock(&kmod_ring_pool[slot].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

noinline u64 bench_kmod_ring_lookup(u32 slot)
{
	u64 ret = 0;

	if (!kmod_ring_pool || !kmod_ring_size)
		return 0;
	slot %= kmod_ring_size;

	spin_lock(&kmod_ring_pool[slot].lock);
	if (kmod_ring_pool[slot].valid)
		ret = kmod_ring_pool[slot].data;
	spin_unlock(&kmod_ring_pool[slot].lock);

	this_cpu_inc(bench_cpu_stats.ops);
	return ret;
}

noinline void bench_kmod_ring_update(u32 slot, u64 val)
{
	if (!kmod_ring_pool || !kmod_ring_size)
		return;
	slot %= kmod_ring_size;

	spin_lock(&kmod_ring_pool[slot].lock);
	kmod_ring_pool[slot].data = val;
	spin_unlock(&kmod_ring_pool[slot].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

/* Mark slot invalid (dequeue). */
noinline void bench_kmod_ring_dequeue(u32 slot)
{
	if (!kmod_ring_pool || !kmod_ring_size)
		return;
	slot %= kmod_ring_size;

	spin_lock(&kmod_ring_pool[slot].lock);
	kmod_ring_pool[slot].valid = 0;
	spin_unlock(&kmod_ring_pool[slot].lock);

	this_cpu_inc(bench_cpu_stats.ops);
}


noinline void bench_undo_ring_init(u32 num_slots) { }
EXPORT_SYMBOL_GPL(bench_undo_ring_init);

noinline void bench_undo_ring_enqueue(u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_ring_enqueue);


noinline u64 bench_undo_ring_lookup(u32 slot)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_undo_ring_lookup);

noinline void bench_undo_ring_update(u32 slot, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_ring_update);

noinline void bench_undo_ring_dequeue(u32 slot)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_ring_dequeue);


noinline void bench_arena_ring_init(u32 num_slots) { }
EXPORT_SYMBOL_GPL(bench_arena_ring_init);

noinline void bench_arena_ring_enqueue(u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_ring_enqueue);

noinline u64 bench_arena_ring_lookup(u32 slot)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_arena_ring_lookup);

noinline void bench_arena_ring_update(u32 slot, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_ring_update);

noinline void bench_arena_ring_dequeue(u32 slot)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_ring_dequeue);
