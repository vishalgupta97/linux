#!/usr/bin/env python3
import sys
import pathlib
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.cm as cm

# ── User-defined config superset (from bench_kmod/bench_spinlock_kmod.c) ─────
DATA_STRUCTURES = ["list", "graph", "ring"] #, "trie"] #"rbtree"
OPERATIONS      = ["insert"] #, "lookup", "update", "delete"]
VARIANTS        = ["kmod_bpf", "lock_func"] #"undo_log", "kmod", "kmod_bpf"] #"arena"
THREADS         = [1, 2, 4, 8, 12, 16, 20, 28, 56, 84, 112, 128, 168, 224]
# ─────────────────────────────────────────────────────────────────────────────

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <server-config-exectime.csv> [...]", file=sys.stderr)
    sys.exit(1)

frames = []
for path in sys.argv[1:]:
    p = pathlib.Path(path)
    parts = p.stem.split("-")
    server = parts[0]
    config = "-".join(parts[1:-1])
    exectime = parts[-1]
    frame = pd.read_csv(path)
    frame["config"] = config
    frame["server"] = server
    frame["exectime"] = exectime
    frames.append(frame)

df = pd.concat(frames, ignore_index=True)

for (server, exectime, pool_size), group in df.groupby(["server", "exectime", "pool_size"]):
    configs = sorted(group["config"].unique())
    combos = [(v, c) for v in VARIANTS for c in configs]

    color_map = {combo: cm.tab10(i / max(len(combos) - 1, 1)) for i, combo in enumerate(combos)}

    n_rows = len(OPERATIONS)
    n_cols = len(DATA_STRUCTURES)

    fig, axes = plt.subplots(
        n_rows, n_cols,
        figsize=(4 * n_cols, 3.5 * n_rows),
        sharex=False, sharey=False,
        squeeze=False,
    )

    threads_in_data = [t for t in THREADS if t <= group["threads"].max()]
    thread_to_idx = {t: i for i, t in enumerate(threads_in_data)}

    for row_idx, op in enumerate(OPERATIONS):
        for col_idx, ds in enumerate(DATA_STRUCTURES):
            ax = axes[row_idx][col_idx]

            for variant, config in combos:
                subset = group[
                    (group["op"] == op) &
                    (group["ds"] == ds) &
                    (group["variant"] == variant) &
                    (group["config"] == config) &
                    (group["threads"].isin(THREADS))
                ].sort_values("threads")

                if subset.empty:
                    continue

                ax.plot(
                    subset["threads"].map(thread_to_idx),
                    subset["ops_per_sec"],
                    marker="o",
                    markersize=4,
                    linewidth=1.5,
                    color=color_map[(variant, config)],
                    label=f"{variant}_{config}",
                )

            if row_idx == 0:
                ax.set_title(ds, fontsize=11, fontweight="bold")
            if col_idx == 0:
                ax.set_ylabel(f"{op}\nops/s", fontsize=9)
            if row_idx == n_rows - 1:
                ax.set_xlabel("threads", fontsize=9)

            ax.set_xticks(range(len(threads_in_data)))
            ax.set_xticklabels(threads_in_data)
            ax.xaxis.set_tick_params(rotation=45)
            ax.yaxis.set_major_formatter(ticker.FuncFormatter(
                lambda x, _: f"{x/1e6:.1f}M" if x >= 1e6 else f"{x:.0f}"
            ))
            ax.tick_params(axis="both", labelsize=8)
            ax.grid(True, linestyle="--", alpha=0.4)

    handles = [
        plt.Line2D([0], [0], color=color_map[(v, c)], marker="o", markersize=5, label=f"{v}_{c}")
        for v, c in combos
    ]
    fig.legend(handles=handles, loc="upper center", ncol=len(combos),
               fontsize=9, bbox_to_anchor=(0.5, 1.01), frameon=True)

    fig.suptitle(f"{server}  exec={exectime}  pool={pool_size}", fontsize=13, y=1.04)
    fig.tight_layout()

    outname = f"{server}-{exectime}-{pool_size}.png"
    fig.savefig(outname, dpi=150, bbox_inches="tight")
    print(f"Saved {outname}")
    plt.close(fig)
