
noinline void bench_kmod_trie_init(u32 pool_size)
{
	if (!kmod_trie_pool)
		return;
	memset(kmod_trie_pool, 0, pool_size * sizeof(*kmod_trie_pool));
	kmod_trie_root = 0;
	atomic_set(&kmod_trie_alloc, 1);
	spin_lock_init(&kmod_trie_lock);
}

/* Binary trie insert on 64-bit key, one bit per level from MSB */
noinline void bench_kmod_trie_insert(u64 key, u64 val)
{
	u32 cur, parent, bit, new_node;
	int dir;

	spin_lock(&kmod_trie_lock);
	cur = kmod_trie_root;
	parent = 0;
	dir = 0;

	for (bit = 63; ; bit--) {
		int b = (key >> bit) & 1;

		if (!cur) {
			new_node = (u32)atomic_fetch_add(1, &kmod_trie_alloc);
			if (new_node >= KMOD_MAX_POOL)
				goto out;
			kmod_trie_pool[new_node].child[0] = 0;
			kmod_trie_pool[new_node].child[1] = 0;
			kmod_trie_pool[new_node].key_bit   = bit;
			kmod_trie_pool[new_node].val       = val;
			if (parent)
				kmod_trie_pool[parent].child[dir] = new_node;
			else
				kmod_trie_root = new_node;
			break;
		}
		parent = cur;
		dir = b;
		cur = kmod_trie_pool[cur].child[b];
		if (!bit)
			break;
	}
out:
	spin_unlock(&kmod_trie_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}

/* BST traversal: follow bit-by-bit from MSB; return val at the leaf. */
noinline u64 bench_kmod_trie_lookup(u64 key)
{
	u32 cur;
	u64 ret = 0;
	int bit;

	if (!kmod_trie_pool)
		return 0;

	spin_lock(&kmod_trie_lock);
	cur = kmod_trie_root;
	for (bit = 63; bit >= 0 && cur; bit--) {
		int b = (key >> bit) & 1;

		if (!kmod_trie_pool[cur].child[0] && !kmod_trie_pool[cur].child[1]) {
			ret = kmod_trie_pool[cur].val;
			break;
		}
		cur = kmod_trie_pool[cur].child[b];
	}
	spin_unlock(&kmod_trie_lock);

	this_cpu_inc(bench_cpu_stats.ops);
	return ret;
}

/* Same traversal but write val at the leaf. */
noinline void bench_kmod_trie_update(u64 key, u64 val)
{
	u32 cur;
	int bit;

	if (!kmod_trie_pool)
		return;

	spin_lock(&kmod_trie_lock);
	cur = kmod_trie_root;
	for (bit = 63; bit >= 0 && cur; bit--) {
		if (!kmod_trie_pool[cur].child[0] && !kmod_trie_pool[cur].child[1]) {
			kmod_trie_pool[cur].val = val;
			break;
		}
		cur = kmod_trie_pool[cur].child[(key >> bit) & 1];
	}
	spin_unlock(&kmod_trie_lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

/* Find the leaf for key and unlink it from its parent. */
noinline void bench_kmod_trie_delete(u64 key)
{
	u32 cur, parent;
	int bit, dir = 0;

	if (!kmod_trie_pool)
		return;

	spin_lock(&kmod_trie_lock);
	cur = kmod_trie_root;
	parent = 0;
	for (bit = 63; bit >= 0 && cur; bit--) {
		int b = (key >> bit) & 1;

		if (!kmod_trie_pool[cur].child[0] && !kmod_trie_pool[cur].child[1]) {
			kmod_trie_pool[cur].key_bit = 0;
			kmod_trie_pool[cur].val     = 0;
			if (parent)
				kmod_trie_pool[parent].child[dir] = 0;
			else
				kmod_trie_root = 0;
			break;
		}
		parent = cur;
		dir    = b;
		cur    = kmod_trie_pool[cur].child[b];
	}
	spin_unlock(&kmod_trie_lock);

	this_cpu_inc(bench_cpu_stats.ops);
}


noinline void bench_undo_trie_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_undo_trie_init);

noinline void bench_undo_trie_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_trie_insert);


noinline u64 bench_undo_trie_lookup(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_undo_trie_lookup);

noinline void bench_undo_trie_update(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_trie_update);

noinline void bench_undo_trie_delete(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_trie_delete);


noinline void bench_arena_trie_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_arena_trie_init);

noinline void bench_arena_trie_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_trie_insert);


noinline u64 bench_arena_trie_lookup(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_arena_trie_lookup);

noinline void bench_arena_trie_update(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_trie_update);

noinline void bench_arena_trie_delete(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_trie_delete);
