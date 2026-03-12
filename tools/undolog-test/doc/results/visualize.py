#!/usr/bin/env python3
"""
Visualize rcuhashbash benchmark results as grouped bar charts.

Directory layout expected:
  <BASE_DIR>/<server>/results-spinlock-<MAX_CORES>cores-<DURATION>seconds/
      <BUCKETS>buckets-<entries>entries/
          modified_table_<lock>_<undolog>/
              <WRITE_PCT>/
                  core.<cores>
"""

import os
import re
import warnings

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# CONFIG — modify these parameters to match your experiment layout
# ---------------------------------------------------------------------------

# Path to the "results/" directory (relative to this script's location)
BASE_DIR = os.path.join(os.path.dirname(__file__), "")

# Servers to include
SERVERS = ["srv10vm"]

# Sub-directory template inside each server folder
# {max_cores} and {duration} are filled from MAX_CORES / DURATION below
RESULT_SUBDIR_TEMPLATE = "results-spinlock-{max_cores}cores-{duration}seconds"
#MAX_CORES = 112
MAX_CORES = 96
DURATION = 10

# Hash-table parameters
BUCKETS = 1024
ENTRIES_LIST = [1024, 2048, 4096, 6144, 8192]

# Lock implementations
LOCKS = ["spinlock", "cna", "aqs"]

# Undo-log variants (suffix after lock name in the directory)
UNDOLOG_TYPES = ["baseline", "withundolog", "withundologatomic", "withundologprefetch", "withundologstore"]

# Core counts to include
# CORE_COUNTS = [4, 8, 28, 56, 112]
CORE_COUNTS = [8, 24, 48, 72, 96]

# Write-percentage subdirectory
WRITE_PCT = "100percent_writes"

# Output directory for generated figures (relative to this script)
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "plots")

# Figure appearance
FIGURE_WIDTH_PER_SUBPLOT = 4.0   # inches per entries column
FIGURE_HEIGHT = 5.0               # inches
DPI = 150

# Whether all subplots in a figure share the same Y-axis range
SHARED_Y_AXIS = False

# ---------------------------------------------------------------------------
# Regex for extracting total throughput
# ---------------------------------------------------------------------------
_TOTAL_RE = re.compile(r"rcuhashbash summary: total:\s+(\d+)")


def parse_throughput(filepath: str):
    """Return the integer total throughput from a result file, or None."""
    try:
        with open(filepath, "r") as fh:
            for line in fh:
                m = _TOTAL_RE.search(line)
                if m:
                    return int(m.group(1))
    except OSError:
        return None
    return None


def load_data() -> pd.DataFrame:
    """
    Walk all parameter combinations, parse throughput from each result file.
    Missing or unparseable files produce throughput=0 and emit a warning.
    """
    warnings.simplefilter("always")

    result_subdir = RESULT_SUBDIR_TEMPLATE.format(
        max_cores=MAX_CORES, duration=DURATION
    )

    rows = []
    for server in SERVERS:
        for entries in ENTRIES_LIST:
            for lock in LOCKS:
                for undolog in UNDOLOG_TYPES:
                    for cores in CORE_COUNTS:
                        filepath = os.path.join(
                            BASE_DIR,
                            server,
                            result_subdir,
                            f"{BUCKETS}buckets-{entries}entries",
                            f"modified_table_{lock}_{undolog}",
                            WRITE_PCT,
                            f"core.{cores}",
                        )
                        throughput = parse_throughput(filepath)
                        if throughput is None:
                            warnings.warn(
                                f"Missing or unparseable file (throughput=0): {filepath}"
                            )
                            throughput = 0
                        rows.append(
                            {
                                "server": server,
                                "entries": entries,
                                "lock": lock,
                                "undolog": undolog,
                                "cores": cores,
                                "throughput": throughput,
                            }
                        )

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

# Consistent colors and labels for the four undolog variants
_UNDOLOG_COLORS = {
    "baseline":            "#4C72B0",
    "withundolog":         "#DD8452",
    "withundologatomic":   "#55A868",
    "withundologprefetch": "#C44E52",
    "withundologstore": "#4AA2A0",
}
_UNDOLOG_LABELS = {
    "baseline":            "Baseline",
    "withundolog":         "Undo Log",
    "withundologatomic":   "Undo Log (Atomic)",
    "withundologprefetch": "Undo Log (Prefetch)",
    "withundologstore": "Undo Log (Store)",
}


def plot_lock_figure(df: pd.DataFrame, server: str, lock: str) -> plt.Figure:
    """
    Create a figure with one subplot per entries count for a given lock type.
    Each subplot is a grouped bar chart: X = core count, bars = undolog types.
    """
    lock_df = df[(df["lock"] == lock) & (df["server"] == server)]

    n_entries = len(ENTRIES_LIST)
    fig, axes = plt.subplots(
        1, n_entries,
        figsize=(FIGURE_WIDTH_PER_SUBPLOT * n_entries, FIGURE_HEIGHT),
        sharey=SHARED_Y_AXIS,
    )
    if n_entries == 1:
        axes = [axes]

    n_undolog = len(UNDOLOG_TYPES)
    x = np.arange(len(CORE_COUNTS))
    bar_width = 0.8 / n_undolog   # total group width = 0.8

    for ax, entries in zip(axes, ENTRIES_LIST):
        sub = lock_df[lock_df["entries"] == entries]

        for i, undolog in enumerate(UNDOLOG_TYPES):
            vals = []
            for cores in CORE_COUNTS:
                row = sub[(sub["undolog"] == undolog) & (sub["cores"] == cores)]
                vals.append(int(row["throughput"].iloc[0]) if len(row) else 0)

            offset = (i - n_undolog / 2 + 0.5) * bar_width
            ax.bar(
                x + offset,
                vals,
                width=bar_width,
                color=_UNDOLOG_COLORS[undolog],
                label=_UNDOLOG_LABELS[undolog],
                edgecolor="white",
                linewidth=0.5,
            )

        ax.set_title(f"{entries} entries", fontsize=10, pad=6)
        ax.set_xticks(x)
        ax.set_xticklabels([str(c) for c in CORE_COUNTS])
        ax.set_xlabel("Core count", fontsize=9)
        ax.yaxis.set_major_formatter(
            mticker.FuncFormatter(lambda v, _: f"{v/1e6:.1f}M" if v >= 1e6 else str(int(v)))
        )
        ax.tick_params(axis="both", labelsize=8)
        ax.grid(axis="y", linestyle="--", linewidth=0.5, alpha=0.7)
        ax.set_axisbelow(True)

    # Y label only on first subplot
    axes[0].set_ylabel("Throughput (ops/10s)", fontsize=9)

    # Single legend at the top of the figure
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles, labels,
        loc="upper center",
        ncol=n_undolog,
        fontsize=8,
        frameon=True,
        bbox_to_anchor=(0.5, 1.02),
    )

    fig.suptitle(
        f"Lock: {lock.upper()}  —  {BUCKETS} buckets, {DURATION}s, 100% writes",
        fontsize=11,
        y=1.07,
    )
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Loading data...")
    df = load_data()

    total = len(df)
    missing = (df["throughput"] == 0).sum()
    print(f"Loaded {total} data points ({missing} missing/zero).")

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for server in SERVERS:
        for lock in LOCKS:
            fig = plot_lock_figure(df, server, lock)
            out_path = os.path.join(OUTPUT_DIR, f"{server}_{lock}_throughput.png")
            fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved: {out_path}")

    print("Done.")


if __name__ == "__main__":
    main()
