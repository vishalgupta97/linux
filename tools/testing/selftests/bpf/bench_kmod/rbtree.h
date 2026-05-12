
noinline void bench_kmod_rbtree_init(u32 pool_size)
{
	if (!kmod_rb_pool)
		return;
	memset(kmod_rb_pool, 0, pool_size * sizeof(*kmod_rb_pool));
	/* node 0 = nil sentinel (black) */
	kmod_rb_pool[0].color = KMOD_RB_BLACK;
	kmod_rb_pool[0].left = kmod_rb_pool[0].right = kmod_rb_pool[0].parent = 0;
	kmod_rb_root = 0;
	atomic_set(&kmod_rb_alloc, 1);
	spin_lock_init(&kmod_rb_lock);
}

static void kmod_rb_rotate_left(u32 x)
{
	u32 y = kmod_rb_pool[x].right;

	kmod_rb_pool[x].right = kmod_rb_pool[y].left;
	if (kmod_rb_pool[y].left)
		kmod_rb_pool[kmod_rb_pool[y].left].parent = x;
	kmod_rb_pool[y].parent = kmod_rb_pool[x].parent;
	if (!kmod_rb_pool[x].parent)
		kmod_rb_root = y;
	else if (x == kmod_rb_pool[kmod_rb_pool[x].parent].left)
		kmod_rb_pool[kmod_rb_pool[x].parent].left = y;
	else
		kmod_rb_pool[kmod_rb_pool[x].parent].right = y;
	kmod_rb_pool[y].left = x;
	kmod_rb_pool[x].parent = y;
}

static void kmod_rb_rotate_right(u32 x)
{
	u32 y = kmod_rb_pool[x].left;

	kmod_rb_pool[x].left = kmod_rb_pool[y].right;
	if (kmod_rb_pool[y].right)
		kmod_rb_pool[kmod_rb_pool[y].right].parent = x;
	kmod_rb_pool[y].parent = kmod_rb_pool[x].parent;
	if (!kmod_rb_pool[x].parent)
		kmod_rb_root = y;
	else if (x == kmod_rb_pool[kmod_rb_pool[x].parent].right)
		kmod_rb_pool[kmod_rb_pool[x].parent].right = y;
	else
		kmod_rb_pool[kmod_rb_pool[x].parent].left = y;
	kmod_rb_pool[y].right = x;
	kmod_rb_pool[x].parent = y;
}

static void kmod_rb_insert_fixup(u32 z)
{
	u32 y;
	int depth = 0;

	while (kmod_rb_pool[kmod_rb_pool[z].parent].color == KMOD_RB_RED &&
	       depth++ < 64) {
		u32 p = kmod_rb_pool[z].parent;
		u32 g = kmod_rb_pool[p].parent;

		if (p == kmod_rb_pool[g].left) {
			y = kmod_rb_pool[g].right;
			if (kmod_rb_pool[y].color == KMOD_RB_RED) {
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[y].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				z = g;
			} else {
				if (z == kmod_rb_pool[p].right) {
					z = p;
					kmod_rb_rotate_left(z);
					p = kmod_rb_pool[z].parent;
					g = kmod_rb_pool[p].parent;
				}
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				kmod_rb_rotate_right(g);
			}
		} else {
			y = kmod_rb_pool[g].left;
			if (kmod_rb_pool[y].color == KMOD_RB_RED) {
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[y].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				z = g;
			} else {
				if (z == kmod_rb_pool[p].left) {
					z = p;
					kmod_rb_rotate_right(z);
					p = kmod_rb_pool[z].parent;
					g = kmod_rb_pool[p].parent;
				}
				kmod_rb_pool[p].color = KMOD_RB_BLACK;
				kmod_rb_pool[g].color = KMOD_RB_RED;
				kmod_rb_rotate_left(g);
			}
		}
	}
	kmod_rb_pool[kmod_rb_root].color = KMOD_RB_BLACK;
}

noinline void bench_kmod_rbtree_insert(u64 key, u64 val)
{
	u32 z, p, x;

	spin_lock(&kmod_rb_lock);
	z = (u32)atomic_fetch_add(1, &kmod_rb_alloc);
	if (z >= KMOD_MAX_POOL)
		goto out;

	kmod_rb_pool[z].key    = key;
	kmod_rb_pool[z].val    = val;
	kmod_rb_pool[z].color  = KMOD_RB_RED;
	kmod_rb_pool[z].left   = 0;
	kmod_rb_pool[z].right  = 0;
	kmod_rb_pool[z].parent = 0;

	p = 0;
	x = kmod_rb_root;
	while (x) {
		p = x;
		if (key < kmod_rb_pool[x].key)
			x = kmod_rb_pool[x].left;
		else
			x = kmod_rb_pool[x].right;
	}
	kmod_rb_pool[z].parent = p;
	if (!p) {
		kmod_rb_root = z;
	} else if (key < kmod_rb_pool[p].key) {
		kmod_rb_pool[p].left = z;
	} else {
		kmod_rb_pool[p].right = z;
	}
	kmod_rb_insert_fixup(z);
out:
	spin_unlock(&kmod_rb_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}

noinline u64 bench_kmod_rbtree_lookup(u64 key)
{
	u32 cur;
	u64 ret = 0;

	if (!kmod_rb_pool)
		return 0;

	spin_lock(&kmod_rb_lock);
	cur = kmod_rb_root;
	while (cur) {
		if (key == kmod_rb_pool[cur].key) {
			ret = kmod_rb_pool[cur].val;
			break;
		}
		cur = (key < kmod_rb_pool[cur].key)
			? kmod_rb_pool[cur].left
			: kmod_rb_pool[cur].right;
	}
	spin_unlock(&kmod_rb_lock);

	this_cpu_inc(bench_cpu_stats.ops);
	return ret;
}

noinline void bench_kmod_rbtree_update(u64 key, u64 val)
{
	u32 cur;

	if (!kmod_rb_pool)
		return;

	spin_lock(&kmod_rb_lock);
	cur = kmod_rb_root;
	while (cur) {
		if (key == kmod_rb_pool[cur].key) {
			kmod_rb_pool[cur].val = val;
			break;
		}
		cur = (key < kmod_rb_pool[cur].key)
			? kmod_rb_pool[cur].left
			: kmod_rb_pool[cur].right;
	}
	spin_unlock(&kmod_rb_lock);

	this_cpu_inc(bench_cpu_stats.ops);
}

static void kmod_rb_transplant(u32 u, u32 v)
{
	if (!kmod_rb_pool[u].parent) {
		kmod_rb_root = v;
	} else if (u == kmod_rb_pool[kmod_rb_pool[u].parent].left) {
		kmod_rb_pool[kmod_rb_pool[u].parent].left = v;
	} else {
		kmod_rb_pool[kmod_rb_pool[u].parent].right = v;
	}
	if (v)
		kmod_rb_pool[v].parent = kmod_rb_pool[u].parent;
}

static void kmod_rb_delete_fixup(u32 x, u32 x_parent)
{
	u32 w;
	int depth = 0;

	while (x != kmod_rb_root &&
	       (!x || kmod_rb_pool[x].color == KMOD_RB_BLACK) &&
	       depth++ < 64) {
		if (x == kmod_rb_pool[x_parent].left) {
			w = kmod_rb_pool[x_parent].right;
			if (w && kmod_rb_pool[w].color == KMOD_RB_RED) {
				kmod_rb_pool[w].color = KMOD_RB_BLACK;
				kmod_rb_pool[x_parent].color = KMOD_RB_RED;
				kmod_rb_rotate_left(x_parent);
				w = kmod_rb_pool[x_parent].right;
			}
			if ((!kmod_rb_pool[w].left  ||
			     kmod_rb_pool[kmod_rb_pool[w].left].color  == KMOD_RB_BLACK) &&
			    (!kmod_rb_pool[w].right ||
			     kmod_rb_pool[kmod_rb_pool[w].right].color == KMOD_RB_BLACK)) {
				if (w)
					kmod_rb_pool[w].color = KMOD_RB_RED;
				x = x_parent;
				x_parent = kmod_rb_pool[x].parent;
			} else {
				if (!kmod_rb_pool[w].right ||
				    kmod_rb_pool[kmod_rb_pool[w].right].color == KMOD_RB_BLACK) {
					if (kmod_rb_pool[w].left)
						kmod_rb_pool[kmod_rb_pool[w].left].color = KMOD_RB_BLACK;
					if (w)
						kmod_rb_pool[w].color = KMOD_RB_RED;
					kmod_rb_rotate_right(w);
					w = kmod_rb_pool[x_parent].right;
				}
				kmod_rb_pool[w].color = kmod_rb_pool[x_parent].color;
				kmod_rb_pool[x_parent].color = KMOD_RB_BLACK;
				if (kmod_rb_pool[w].right)
					kmod_rb_pool[kmod_rb_pool[w].right].color = KMOD_RB_BLACK;
				kmod_rb_rotate_left(x_parent);
				x = kmod_rb_root;
			}
		} else {
			w = kmod_rb_pool[x_parent].left;
			if (w && kmod_rb_pool[w].color == KMOD_RB_RED) {
				kmod_rb_pool[w].color = KMOD_RB_BLACK;
				kmod_rb_pool[x_parent].color = KMOD_RB_RED;
				kmod_rb_rotate_right(x_parent);
				w = kmod_rb_pool[x_parent].left;
			}
			if ((!kmod_rb_pool[w].right ||
			     kmod_rb_pool[kmod_rb_pool[w].right].color == KMOD_RB_BLACK) &&
			    (!kmod_rb_pool[w].left  ||
			     kmod_rb_pool[kmod_rb_pool[w].left].color  == KMOD_RB_BLACK)) {
				if (w)
					kmod_rb_pool[w].color = KMOD_RB_RED;
				x = x_parent;
				x_parent = kmod_rb_pool[x].parent;
			} else {
				if (!kmod_rb_pool[w].left ||
				    kmod_rb_pool[kmod_rb_pool[w].left].color == KMOD_RB_BLACK) {
					if (kmod_rb_pool[w].right)
						kmod_rb_pool[kmod_rb_pool[w].right].color = KMOD_RB_BLACK;
					if (w)
						kmod_rb_pool[w].color = KMOD_RB_RED;
					kmod_rb_rotate_left(w);
					w = kmod_rb_pool[x_parent].left;
				}
				kmod_rb_pool[w].color = kmod_rb_pool[x_parent].color;
				kmod_rb_pool[x_parent].color = KMOD_RB_BLACK;
				if (kmod_rb_pool[w].left)
					kmod_rb_pool[kmod_rb_pool[w].left].color = KMOD_RB_BLACK;
				kmod_rb_rotate_right(x_parent);
				x = kmod_rb_root;
			}
		}
	}
	if (x)
		kmod_rb_pool[x].color = KMOD_RB_BLACK;
}

/* Cormen RB delete: BST search, transplant, delete-fixup. */
noinline void bench_kmod_rbtree_delete(u64 key)
{
	u32 z, y, x, x_parent;
	u32 y_orig_color;
	int depth = 0;

	if (!kmod_rb_pool)
		return;

	spin_lock(&kmod_rb_lock);

	z = kmod_rb_root;
	while (z && depth++ < KMOD_MAX_POOL) {
		if (key == kmod_rb_pool[z].key)
			break;
		z = (key < kmod_rb_pool[z].key)
			? kmod_rb_pool[z].left : kmod_rb_pool[z].right;
	}
	if (!z || kmod_rb_pool[z].key != key)
		goto out;

	y             = z;
	y_orig_color  = kmod_rb_pool[y].color;
	x_parent      = 0;

	if (!kmod_rb_pool[z].left) {
		x        = kmod_rb_pool[z].right;
		x_parent = kmod_rb_pool[z].parent;
		kmod_rb_transplant(z, x);
	} else if (!kmod_rb_pool[z].right) {
		x        = kmod_rb_pool[z].left;
		x_parent = kmod_rb_pool[z].parent;
		kmod_rb_transplant(z, x);
	} else {
		/* Minimum of right subtree */
		y = kmod_rb_pool[z].right;
		depth = 0;
		while (kmod_rb_pool[y].left && depth++ < KMOD_MAX_POOL)
			y = kmod_rb_pool[y].left;
		y_orig_color = kmod_rb_pool[y].color;
		x = kmod_rb_pool[y].right;
		if (kmod_rb_pool[y].parent == z) {
			x_parent = y;
		} else {
			x_parent = kmod_rb_pool[y].parent;
			kmod_rb_transplant(y, x);
			kmod_rb_pool[y].right = kmod_rb_pool[z].right;
			if (kmod_rb_pool[y].right)
				kmod_rb_pool[kmod_rb_pool[y].right].parent = y;
		}
		kmod_rb_transplant(z, y);
		kmod_rb_pool[y].left = kmod_rb_pool[z].left;
		if (kmod_rb_pool[y].left)
			kmod_rb_pool[kmod_rb_pool[y].left].parent = y;
		kmod_rb_pool[y].color = kmod_rb_pool[z].color;
	}

	if (y_orig_color == KMOD_RB_BLACK)
		kmod_rb_delete_fixup(x, x_parent);

	/* Return deleted node to a zeroed state so the slot is reusable. */
	kmod_rb_pool[z].key    = 0;
	kmod_rb_pool[z].val    = 0;
	kmod_rb_pool[z].left   = 0;
	kmod_rb_pool[z].right  = 0;
	kmod_rb_pool[z].parent = 0;
	kmod_rb_pool[z].color  = KMOD_RB_BLACK;
out:
	spin_unlock(&kmod_rb_lock);
	this_cpu_inc(bench_cpu_stats.ops);
}


noinline void bench_undo_rbtree_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_undo_rbtree_init);

noinline void bench_undo_rbtree_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_rbtree_insert);


noinline u64 bench_undo_rbtree_lookup(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_undo_rbtree_lookup);

noinline void bench_undo_rbtree_update(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_rbtree_update);

noinline void bench_undo_rbtree_delete(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_undo_rbtree_delete);


noinline void bench_arena_rbtree_init(u32 pool_size) { }
EXPORT_SYMBOL_GPL(bench_arena_rbtree_init);

noinline void bench_arena_rbtree_insert(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_rbtree_insert);


noinline u64 bench_arena_rbtree_lookup(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
	return 0;
}
EXPORT_SYMBOL_GPL(bench_arena_rbtree_lookup);

noinline void bench_arena_rbtree_update(u64 key, u64 val)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_rbtree_update);

noinline void bench_arena_rbtree_delete(u64 key)
{
	this_cpu_inc(bench_cpu_stats.ops);
}
EXPORT_SYMBOL_GPL(bench_arena_rbtree_delete);
