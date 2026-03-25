#!/usr/bin/env python3
"""
Visualize rcuhashbash benchmark results as line charts.

Directory layout expected:
  <BASE_DIR>/<server>/results-spinlock-<MAX_CORES>cores-<DURATION>seconds/
      <BUCKETS>buckets-<entries>entries/
          modified_<lock>_<undolog>/
              <WRITE_PCT>/
                  core.<cores>
"""

import os
import re
import warnings

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import pandas as pd

# ---------------------------------------------------------------------------
# CONFIG — modify these parameters to match your experiment layout
# ---------------------------------------------------------------------------

# Path to the "results/" directory (relative to this script's location)
BASE_DIR = os.path.join(os.path.dirname(__file__), "")

# Servers to include
SERVERS = ["srv1vm", "srv10vm"]

# Per-server experiment layout
SERVER_CONFIGS = {
    "srv1vm": {
        "max_cores": 112,
        "core_counts": [1, 2, 4, 8, 14, 28, 42, 56, 70, 84, 98, 112],
    },
    "srv10vm": {
        "max_cores": 96,
        "core_counts": [1, 2, 4, 8, 12, 24, 36, 48, 60, 72, 84, 96],
    },
}

# Sub-directory template inside each server folder
# {max_cores} and {duration} are filled from SERVER_CONFIGS / DURATION below
RESULT_SUBDIR_TEMPLATE = "results-spinlock-{max_cores}cores-{duration}seconds"
DURATION = 30

# Hash-table parameters
BUCKETS = 1024
ENTRIES_LIST = [1024, 2048, 4096, 6144, 8192]

# Lock implementations
LOCKS = ["table_spinlock", "bpf_table_bpf_spinlock_baseline", "bpf_table_bpf_spinlock_undolog"]

# Undo-log variants (suffix after lock name in the directory)
UNDOLOG_TYPES = ["baseline"] #, "withundolog", "withundologatomic", "withundologprefetch", "withundologstore"]

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
# Regex for extracting throughput summary fields
# ---------------------------------------------------------------------------
_SUMMARY_RE = re.compile(r"rcuhashbash summary: total:\s+(\d+)\s+\(avg:\s+(\d+)")


def parse_throughput(filepath: str):
    """Return throughput in ops/sec computed from total ops and avg ns, or None."""
    try:
        with open(filepath, "r") as fh:
            for line in fh:
                m = _SUMMARY_RE.search(line)
                if m:
                    total_ops = int(m.group(1))
                    avg_ns = int(m.group(2))
                    if avg_ns <= 0:
                        return None
                    return total_ops * 1e9 / avg_ns
    except OSError:
        return None
    return None


def load_data() -> pd.DataFrame:
    """
    Walk all parameter combinations, parse throughput from each result file.
    Missing or unparseable files produce throughput=0 and emit a warning.
    """
    warnings.simplefilter("always")

    rows = []
    for server in SERVERS:
        if server not in SERVER_CONFIGS:
            raise ValueError(f"Missing server config for '{server}' in SERVER_CONFIGS")

        server_cfg = SERVER_CONFIGS[server]
        result_subdir = RESULT_SUBDIR_TEMPLATE.format(
            max_cores=server_cfg["max_cores"],
            duration=DURATION,
        )

        for entries in ENTRIES_LIST:
            for lock in LOCKS:
                for undolog in UNDOLOG_TYPES:
                    for cores in server_cfg["core_counts"]:
                        filepath = os.path.join(
                            BASE_DIR,
                            server,
                            result_subdir,
                            f"{BUCKETS}buckets-{entries}entries",
                            f"modified_{lock}_{undolog}",
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

def lock_to_label(lock: str) -> str:
    """Create a readable legend label from a lock identifier."""
    token_overrides = {
        "aqs": "AQS",
        "bpf": "BPF",
        "cna": "CNA",
        "qspinlock": "Qspinlock",
    }
    tokens = lock.split("_")
    pretty_tokens = [token_overrides.get(tok, tok.capitalize()) for tok in tokens]
    return " ".join(pretty_tokens)


def build_lock_styles(locks):
    """Generate lock colors and labels from the configured lock list."""
    cmap = plt.get_cmap("tab20" if len(locks) > 10 else "tab10")
    colors = {lock: cmap(i % cmap.N) for i, lock in enumerate(locks)}
    labels = {lock: lock_to_label(lock) for lock in locks}
    return colors, labels


def ops_formatter(value, _):
    """Render y-axis values with compact engineering suffixes."""
    abs_value = abs(value)
    if abs_value >= 1e9:
        return f"{value / 1e9:.1f}G"
    if abs_value >= 1e6:
        return f"{value / 1e6:.1f}M"
    if abs_value >= 1e3:
        return f"{value / 1e3:.1f}K"
    return f"{value:.0f}"



def plot_lock_figure(df: pd.DataFrame, server: str, undolog: str) -> plt.Figure:
    """
    Create a figure with one subplot per entries count for a given undolog type.
    Each subplot is a line chart: X = core count, one line per lock.
    """
    undolog_df = df[(df["undolog"] == undolog) & (df["server"] == server)]
    server_cfg = SERVER_CONFIGS[server]
    core_counts = server_cfg["core_counts"]
    lock_colors, lock_labels = build_lock_styles(LOCKS)

    n_entries = len(ENTRIES_LIST)
    fig, axes = plt.subplots(
        1, n_entries,
        figsize=(FIGURE_WIDTH_PER_SUBPLOT * n_entries, FIGURE_HEIGHT),
        sharey=SHARED_Y_AXIS,
    )
    if n_entries == 1:
        axes = [axes]

    n_locks = len(LOCKS)

    for ax, entries in zip(axes, ENTRIES_LIST):
        sub = undolog_df[undolog_df["entries"] == entries]

        for lock in LOCKS:
            vals = []
            for cores in core_counts:
                row = sub[(sub["lock"] == lock) & (sub["cores"] == cores)]
                vals.append(float(row["throughput"].iloc[0]) if len(row) else 0)

            ax.plot(
                core_counts,
                vals,
                color=lock_colors[lock],
                label=lock_labels[lock],
                marker="o",
                markersize=4,
                linewidth=1.8,
            )

        ax.set_title(f"{entries} entries", fontsize=10, pad=6)
        ax.set_xticks(core_counts)
        ax.set_xticklabels([str(c) for c in core_counts])
        ax.set_xlabel("Core count", fontsize=9)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(ops_formatter))
        ax.tick_params(axis="both", labelsize=8)
        ax.grid(axis="y", linestyle="--", linewidth=0.5, alpha=0.7)
        ax.set_axisbelow(True)

    # Y label only on first subplot
    axes[0].set_ylabel("Throughput (ops/sec)", fontsize=9)

    # Single legend at the top of the figure
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles, labels,
        loc="upper center",
        ncol=max(1, len(labels)),
        fontsize=8,
        frameon=True,
        bbox_to_anchor=(0.5, 1.01),
    )

    fig.suptitle(
        f"Undolog: {undolog.upper()}  —  {BUCKETS} buckets, {DURATION}s, 100% writes",
        fontsize=11,
        y=1.08,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.9])
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
        for undolog in UNDOLOG_TYPES:
            fig = plot_lock_figure(df, server, undolog)
            out_path = os.path.join(OUTPUT_DIR, f"{server}_{undolog}_throughput.png")
            fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved: {out_path}")

    print("Done.")


if __name__ == "__main__":
    main()
