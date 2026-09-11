#!/usr/bin/env python3
"""Render the obstacle-scale comparison used as Fig. 7."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


HERE = Path(__file__).resolve().parent
DATA = HERE / "test_data/obstacle_scale_sweep.csv"
OUT = HERE / "pics/experiments/scale_sweep"
BLOCKS = (5, 10, 20, 40, 60, 100)
METHODS = {
    "yopo_simple": {"label": "YOPO-Simple", "color": "#356DAA", "marker": "s"},
    "scalenav": {"label": "TopoGuide", "color": "#2D8C74", "marker": "o"},
}
INK = "#253642"
MUTED = "#65747E"
AMBER = "#946B2F"
FAIL = "#C44E52"
MISMATCHED = {60, 100}


def load_data(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "block_m", "method", "trial", "outcome", "path_m", "duration_s",
        "average_speed_mps", "final_error_m", "mission_distance_m", "timeout_s",
    }
    if not rows or not required.issubset(rows[0]):
        missing = sorted(required.difference(rows[0] if rows else set()))
        raise ValueError(f"Missing fields in {path}: {missing}")
    for row in rows:
        row["block_m"] = int(row["block_m"])
        row["trial"] = int(row["trial"])
        for field in (
            "path_m", "duration_s", "average_speed_mps", "final_error_m",
            "mission_distance_m", "timeout_s",
        ):
            row[field] = float(row[field])
        if row["method"] not in METHODS or row["block_m"] not in BLOCKS:
            raise ValueError(f"Unexpected experiment row: {row}")
        if row["outcome"] not in {"success", "timeout"}:
            raise ValueError(f"Unexpected outcome: {row}")
    for method in METHODS:
        for block in BLOCKS:
            group = select(rows, method, block)
            expected = 4 if method == "gcn" and block == 20 else 2
            if len(group) != expected:
                raise ValueError(f"Expected {expected} rows for {method}, {block} m")
            if len({row["mission_distance_m"] for row in group}) != 1:
                raise ValueError(f"Inconsistent mission distance: {method}, {block} m")
    return rows


def select(rows: list[dict], method: str, block: int) -> list[dict]:
    return sorted(
        [row for row in rows if row["method"] == method and row["block_m"] == block],
        key=lambda row: row["trial"],
    )


def success_values(rows: list[dict], method: str, block: int, field: str) -> np.ndarray:
    group = [row for row in select(rows, method, block) if row["outcome"] == "success"]
    if field == "path_efficiency_percent":
        return np.asarray(
            [100.0 * row["mission_distance_m"] / row["path_m"] for row in group],
            dtype=float,
        )
    return np.asarray([row[field] for row in group], dtype=float)


def style_axis(axis, ylabel: str, show_xlabels: bool) -> None:
    axis.set_xlim(-0.48, len(BLOCKS) - 0.52)
    axis.set_xticks(range(len(BLOCKS)), [str(block) for block in BLOCKS])
    axis.set_ylabel(ylabel, color=MUTED, labelpad=4)
    axis.tick_params(axis="both", length=0, pad=3, colors=MUTED,
                     labelbottom=show_xlabels)
    axis.set_axisbelow(True)
    axis.grid(axis="y", color="#DCE3E8", linewidth=0.55)
    axis.grid(axis="x", color="#EEF1F4", linewidth=0.45, linestyle=":")
    for spine in axis.spines.values():
        spine.set_visible(False)
    for index, block in enumerate(BLOCKS):
        if block in MISMATCHED:
            axis.axvspan(index - 0.5, index + 0.5, color="#FBF2E3", zorder=0)


def draw_metric(axis, rows: list[dict], field: str, ylabel: str,
                show_xlabels: bool) -> None:
    style_axis(axis, ylabel, show_xlabels)
    x_values = np.arange(len(BLOCKS), dtype=float)
    for method, style in METHODS.items():
        means = []
        deviations = []
        for block in BLOCKS:
            values = success_values(rows, method, block, field)
            means.append(float(np.mean(values)) if len(values) else np.nan)
            deviations.append(float(np.std(values, ddof=1)) if len(values) > 1 else 0.0)
            if len(values):
                offsets = np.linspace(-0.055, 0.055, len(values))
                axis.scatter(
                    BLOCKS.index(block) + offsets, values, s=8,
                    color=style["color"], alpha=0.38, linewidths=0, zorder=4,
                )
        axis.plot(x_values, means, color=style["color"], linewidth=1.35,
                  alpha=0.96, zorder=2)
        for index, mean, deviation in zip(range(len(BLOCKS)), means, deviations):
            if np.isfinite(mean):
                axis.errorbar(
                    index, mean, yerr=deviation, fmt=style["marker"],
                    color=style["color"], markersize=3.8,
                    markeredgecolor="white", markeredgewidth=0.55,
                    elinewidth=0.8, capsize=2.0, zorder=5,
                )


def draw_success(axis, rows: list[dict], show_xlabels: bool) -> None:
    style_axis(axis, "Success rate (%)", show_xlabels)
    axis.set_ylim(-5, 110)
    axis.set_yticks([0, 25, 50, 75, 100])
    width = 0.20
    for method_index, (method, style) in enumerate(METHODS.items()):
        offset = (method_index - 1) * 0.23
        for index, block in enumerate(BLOCKS):
            group = select(rows, method, block)
            successes = sum(row["outcome"] == "success" for row in group)
            rate = 100.0 * successes / len(group)
            axis.bar(
                index + offset, rate, width=width, color=style["color"],
                alpha=0.86, edgecolor="white", linewidth=0.45, zorder=3,
            )
            label_y = rate - 7 if rate >= 96 else rate + 3
            axis.text(
                index + offset, label_y, f"{successes}/{len(group)}",
                ha="center", va="top" if rate >= 96 else "bottom",
                fontsize=5.8, color="white" if rate >= 96 else style["color"],
                fontweight="semibold", zorder=5,
            )


def mark_timeouts(axis, rows: list[dict]) -> None:
    timeout = 90.0
    axis.axhline(timeout, color="#9AA5AE", linewidth=0.8, linestyle=(0, (3, 3)),
                 zorder=1)
    axis.text(-0.43, timeout + 1.2, "90 s limit", fontsize=5.9, color=MUTED,
              va="bottom")
    for method, style in METHODS.items():
        for index, block in enumerate(BLOCKS):
            failures = [
                row for row in select(rows, method, block)
                if row["outcome"] == "timeout"
            ]
            if failures:
                axis.scatter(
                    [index], [timeout], marker="X", s=30, color=FAIL,
                    edgecolors="white", linewidths=0.7, zorder=6,
                )


def make_figure(rows: list[dict], output: Path) -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 7.0,
        "axes.labelsize": 7.0,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "text.color": INK,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    figure = plt.figure(figsize=(3.6, 4.9))
    grid = figure.add_gridspec(
        3, 1, left=0.155, right=0.97, top=0.94, bottom=0.085,
        hspace=0.22,
    )
    axes = [figure.add_subplot(grid[row, 0]) for row in range(3)]

    handles = [
        Line2D([], [], color=style["color"], marker=style["marker"],
               linewidth=1.5, markersize=4.2, label=style["label"])
        for style in METHODS.values()
    ]
    axes[0].legend(
        handles=handles, loc="upper left",
        ncol=1, frameon=False, fontsize=7.2, handlelength=1.8,
    )

    specs = [
        ("path_m", "Path length (m)", "(a) Completed path"),
        ("duration_s", "Duration (s)", "(b) Completion time"),
        ("average_speed_mps", "Mean speed (m/s)", "(c) Mean speed"),
    ]
    for axis, (field, ylabel, title) in zip(axes, specs):
        draw_metric(axis, rows, field, ylabel, show_xlabels=(axis is axes[-1]))
        axis.set_title(title, loc="left", fontsize=8.2, fontweight="semibold", pad=4)
    mark_timeouts(axes[1], rows)

    axes[0].set_ylim(130, 285)
    axes[0].set_yticks([140, 180, 220, 260])
    axes[1].set_ylim(20, 102)
    axes[1].set_yticks([20, 40, 60, 80, 100])
    axes[2].set_ylim(2.5, 5.8)
    axes[2].set_yticks([3, 4, 5])
    axes[2].set_xlabel("Block length (m)", color=MUTED, labelpad=4)

    yopo_path = np.mean(success_values(rows, "yopo_simple", 40, "path_m"))
    scale_path = np.mean(success_values(rows, "scalenav", 40, "path_m"))
    yopo_time = np.mean(success_values(rows, "yopo_simple", 40, "duration_s"))
    scale_time = np.mean(success_values(rows, "scalenav", 40, "duration_s"))
    axes[0].text(0.98, 0.92, f"40 m: {100 * (scale_path / yopo_path - 1):+.1f}%",
                 transform=axes[0].transAxes, ha="right",
                 color=METHODS["scalenav"]["color"], fontsize=6.0)
    axes[1].text(0.98, 0.92, f"40 m: {100 * (scale_time / yopo_time - 1):+.1f}%",
                 transform=axes[1].transAxes, ha="right",
                 color=METHODS["scalenav"]["color"], fontsize=6.0)

    output.parent.mkdir(parents=True, exist_ok=True)
    for suffix in (".png", ".pdf"):
        figure.savefig(output.with_suffix(suffix), dpi=400, bbox_inches="tight", pad_inches=0.03)
        print(f"Wrote {output.with_suffix(suffix)}")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DATA)
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()
    make_figure(load_data(args.data), args.output)


if __name__ == "__main__":
    main()
