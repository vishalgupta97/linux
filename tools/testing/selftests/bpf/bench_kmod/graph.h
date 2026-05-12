
noinline void bench_kmod_graph_init(u32 num_nodes, u32 num_edges)
{
	u32 i;

	if (!kmod_graph_nodes || !kmod_graph_edges)
		return;
	kmod_graph_num_nodes = num_nodes;
	for (i = 0; i < num_nodes; i++) {
		kmod_graph_nodes[i].first_edge = (u32)~0;
		kmod_graph_nodes[i].data = 0;
	}
	atomic_set(&kmod_graph_edge_alloc, 0);
	spin_lock_init(&kmod_graph_lock);
}

noinline void bench_kmod_graph_add_edge(u32 src, u32 dst, u64 weight)
{
	u32 e;

	if (!kmod_graph_nodes || !kmod_graph_edges)
		return;

	spin_lock(&kmod_graph_lock);
	e = (u32)atomic_fetch_add(1, &kmod_graph_edge_alloc);
	if (e >= KMOD_MAX_POOL || src >= kmod_graph_num_nodes)
		goto out;

	kmod_graph_edges[e].src      = src;
	kmod_graph_edges[e].dst      = dst;
	kmod_graph_edges[e].weight   = weight;
	kmod_graph_edges[e].next_out = kmod_graph_nodes[src].first_edge;
	kmod_graph_nodes[src].first_edge = e;
out:
	spin_unlock(&kmod_graph_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}

noinline u64 bench_kmod_graph_lookup(u32 src, u32 dst)
{
	u32 e;
	u64 ret = 0;

	if (!kmod_graph_nodes || !kmod_graph_edges || src >= kmod_graph_num_nodes)
		return 0;

	spin_lock(&kmod_graph_lock);
	e = kmod_graph_nodes[src].first_edge;
	while (e != (u32)~0 && e < KMOD_MAX_POOL) {
		if (kmod_graph_edges[e].dst == dst) {
			ret = kmod_graph_edges[e].weight;
			break;
		}
		e = kmod_graph_edges[e].next_out;
	}
	spin_unlock(&kmod_graph_lock);

	this_cpu_inc(bench_cpu_stats.ops);
	return ret;
}

noinline void bench_kmod_graph_update(u32 src, u32 dst, u64 weight)
{
	u32 e;

	if (!kmod_graph_nodes || !kmod_graph_edges || src >= kmod_graph_num_nodes)
		return;

	spin_lock(&kmod_graph_lock);
	e = kmod_graph_nodes[src].first_edge;
	while (e != (u32)~0 && e < KMOD_MAX_POOL) {
		if (kmod_graph_edges[e].dst == dst) {
			kmod_graph_edges[e].weight = weight;
			break;
		}
		e = kmod_graph_edges[e].next_out;
	}
	spin_unlock(&kmod_graph_lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

/* Unlink edge (src, dst) from src's adjacency list. */
noinline void bench_kmod_graph_delete(u32 src, u32 dst)
{
	u32 e, prev;

	if (!kmod_graph_nodes || !kmod_graph_edges || src >= kmod_graph_num_nodes)
		return;

	spin_lock(&kmod_graph_lock);
	e    = kmod_graph_nodes[src].first_edge;
	prev = (u32)~0;
	while (e != (u32)~0 && e < KMOD_MAX_POOL) {
		if (kmod_graph_edges[e].dst == dst) {
			if (prev == (u32)~0)
				kmod_graph_nodes[src].first_edge = kmod_graph_edges[e].next_out;
			else
				kmod_graph_edges[prev].next_out  = kmod_graph_edges[e].next_out;
			break;
		}
		prev = e;
		e    = kmod_graph_edges[e].next_out;
	}
	spin_unlock(&kmod_graph_lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

noinline void bench_undo_graph_init(u32 num_nodes, u32 num_edges) { }
EXPORT_SYMBOL_GPL(bench_undo_graph_init);

noinline void bench_undo_graph_add_edge(u32 src, u32 dst, u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_graph_add_edge);

noinline u64 bench_undo_graph_lookup(u32 src, u32 dst)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_undo_graph_lookup);

noinline void bench_undo_graph_update(u32 src, u32 dst, u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_graph_update);

noinline void bench_undo_graph_delete(u32 src, u32 dst)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_graph_delete);

noinline void bench_arena_graph_init(u32 num_nodes, u32 num_edges) { }
EXPORT_SYMBOL_GPL(bench_arena_graph_init);

noinline void bench_arena_graph_add_edge(u32 src, u32 dst, u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_graph_add_edge);

noinline u64 bench_arena_graph_lookup(u32 src, u32 dst)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_arena_graph_lookup);

noinline void bench_arena_graph_update(u32 src, u32 dst, u64 weight)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_graph_update);

noinline void bench_arena_graph_delete(u32 src, u32 dst)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_graph_delete);
