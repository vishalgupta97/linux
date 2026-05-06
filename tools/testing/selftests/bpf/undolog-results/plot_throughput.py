#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

CSV = "notimeout-srv10.csv"
OUT = "throughput-srv10.png"

df = pd.read_csv(CSV)

data_structures = sorted(df["ds"].unique())
pool_sizes = sorted(df["pool_size"].unique())

n_rows = len(pool_sizes)   # vertical axis: pool size
n_cols = len(data_structures)  # horizontal axis: data structure

fig, axes = plt.subplots(
    n_rows, n_cols,
    figsize=(4 * n_cols, 3.5 * n_rows),
    sharex=False, sharey=False,
    squeeze=False,
)

colors = {"undo_log": "#1f77b4", "kmod": "#ff7f0e"}
labels = {"undo_log": "undo_log", "kmod": "kmod"}

for row_idx, pool_size in enumerate(pool_sizes):
    for col_idx, ds in enumerate(data_structures):
        ax = axes[row_idx][col_idx]

        for variant in ("undo_log", "kmod"):
            subset = df[
                (df["variant"] == variant) &
                (df["ds"] == ds) &
                (df["pool_size"] == pool_size)
            ].sort_values("threads")

            if subset.empty:
                continue

            ax.plot(
                subset["threads"],
                subset["ops_per_sec"] / 1e6,
                marker="o",
                markersize=4,
                linewidth=1.5,
                color=colors[variant],
                label=labels[variant],
            )

        # titles and labels
        if row_idx == 0:
            ax.set_title(ds, fontsize=11, fontweight="bold")
        if col_idx == 0:
            ax.set_ylabel(f"pool={pool_size}\nMops/s", fontsize=9)
        if row_idx == n_rows - 1:
            ax.set_xlabel("threads", fontsize=9)

        ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True, nbins=6))
        ax.tick_params(axis="both", labelsize=8)
        ax.grid(True, linestyle="--", alpha=0.4)

# single shared legend at the top
handles = [
    plt.Line2D([0], [0], color=colors["undo_log"], marker="o", markersize=5, label="undo_log"),
    plt.Line2D([0], [0], color=colors["kmod"],    marker="o", markersize=5, label="kmod"),
]
fig.legend(handles=handles, loc="upper center", ncol=2, fontsize=10,
           bbox_to_anchor=(0.5, 1.01), frameon=True)

fig.suptitle("Throughput: undo_log vs kmod (no timeout)", fontsize=13, y=1.04)
fig.tight_layout()

fig.savefig(OUT, dpi=150, bbox_inches="tight")
print(f"Saved {OUT}")
