#!/usr/bin/env python3
"""Plot active local-search resource scaling against persistent graph size."""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from analyze_graph_memory import build_rows, parse_json_line


COLORS = {
    "map": "#2878A6",
    "graph": "#D97732",
    "total": "#23856D",
    "local_window": "#2878A6",
    "expanded": "#D97732",
    "astar": "#7A4EAB",
}

LOCAL_WINDOW_PATTERN = re.compile(
    r"global_nodes=(\d+).*?local_graph_nodes=(\d+)"
)


def read_sessions(summary: Path | None, explicit: list[Path]) -> list[Path]:
    sessions = list(explicit)
    if summary:
        with summary.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row.get("outcome") in {"success", "collision", "timeout"}:
                    sessions.append(Path(row["session_dir"]))
    unique = []
    seen = set()
    for session in sessions:
        resolved = session.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(resolved)
    return unique


def read_local_window_records(summary: Path | None):
    records = []
    if summary is None:
        return records
    with summary.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            raw_path = row.get("console_log") or ""
            path = Path(raw_path)
            if not path.is_file():
                continue
            with path.open(encoding="utf-8", errors="replace") as log:
                for line in log:
                    match = LOCAL_WINDOW_PATTERN.search(line)
                    if match:
                        records.append((int(match.group(1)), int(match.group(2))))
    return records


def timing_records(session: Path):
    planner = []
    stage_latencies = defaultdict(list)
    with (session / "index.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            record = parse_json_line(line)
            if record.get("kind") != "timing":
                continue
            data = record.get("data") or {}
            stamp = int(record.get("stamp_ns", 0))
            if data.get("module") == "planner" and "nodes" in data:
                planner.append((stamp, int(data["nodes"]), data))
                total_ms = float(data.get("total_ms", math.nan))
                if math.isfinite(total_ms):
                    stage_latencies["planner"].append(total_ms)
            elif data.get("module") in {"cloud", "background"}:
                total_ms = float(data.get("total_ms", math.nan))
                if math.isfinite(total_ms):
                    stage_latencies[data["module"]].append(total_ms)
    planner.sort(key=lambda item: item[0])
    return planner, stage_latencies


def collect(sessions: list[Path], point_bytes: int, local_windows):
    values = defaultdict(list)
    stage_latencies = defaultdict(list)
    values["local_window"].extend(local_windows)
    use_json_local_windows = not local_windows
    for session in sessions:
        for row in build_rows(session, point_bytes):
            nodes = row["nodes"]
            map_mb = row["map_payload_bytes"] / 1e6
            graph_mb = row["graph_snapshot_bytes"] / 1e6
            values["map"].append((nodes, map_mb))
            values["graph"].append((nodes, graph_mb))
            values["total"].append((nodes, map_mb + graph_mb))

        planner, session_latencies = timing_records(session)
        for stage, samples in session_latencies.items():
            stage_latencies[stage].extend(samples)
        for _, nodes, data in planner:
            if use_json_local_windows and "local_graph_nodes" in data:
                values["local_window"].append(
                    (nodes, int(data["local_graph_nodes"]))
                )
            # Route-hold ticks do not run a fresh A* search.  Mixing them with
            # active searches would understate the local-search workload.
            if data.get("searched") is not True:
                continue
            expanded = int(data.get("astar_expanded_nodes", 0))
            astar_ms = float(data.get("astar_ms", math.nan))
            edge_evaluations = int(data.get("astar_edge_evaluations", 0))
            values["expanded"].append((nodes, expanded))
            values["edge_evaluations"].append((nodes, edge_evaluations))
            if math.isfinite(astar_ms):
                values["astar"].append((expanded, astar_ms))
                stage_latencies["astar"].append(astar_ms)
    return values, stage_latencies


def binned(values, width: int, minimum_count: int):
    groups = defaultdict(list)
    for x_value, value in values:
        lower = (int(x_value) // width) * width
        groups[lower].append(float(value))
    result = []
    for lower, samples in sorted(groups.items()):
        if len(samples) < minimum_count:
            continue
        array = np.asarray(samples)
        result.append({
            "x": lower + width / 2,
            "x_min": lower,
            "x_max": lower + width,
            "count": len(samples),
            "p05": float(np.percentile(array, 5)),
            "p50": float(np.percentile(array, 50)),
            "p95": float(np.percentile(array, 95)),
        })
    return result


def draw_grouped(axis, categories, first, second, labels, panel, ylabel,
                 decimals=0):
    positions = np.arange(len(categories))
    width = 0.34
    first_bars = axis.bar(
        positions - width / 2, first, width, color="#2878A6", label=labels[0],
        edgecolor="white", linewidth=0.5,
    )
    second_bars = axis.bar(
        positions + width / 2, second, width, color="#D97732", label=labels[1],
        edgecolor="white", linewidth=0.5,
    )
    value_format = f"{{:.{decimals}f}}"
    for bars, values in ((first_bars, first), (second_bars, second)):
        axis.bar_label(
            bars, labels=[value_format.format(value) for value in values],
            padding=2, fontsize=6.5,
        )
    axis.text(
        -0.13, 1.08, panel, transform=axis.transAxes, fontsize=10,
        fontweight="bold", va="top",
    )
    axis.set_xticks(positions, categories)
    axis.set_ylabel(ylabel)
    axis.set_ylim(0, max(max(first), max(second)) * 1.20)
    axis.grid(axis="y", color="#D9DEE2", linewidth=0.6)
    axis.set_axisbelow(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(width=0.7, length=3)
    axis.legend(frameon=True, facecolor="white", edgecolor="#B8BEC3",
                framealpha=1.0, loc="upper left")


def write_binned_csv(path: Path, all_records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "metric", "x_min", "x_max", "x_center",
                "samples", "p05", "p50", "p95",
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        for metric, records in all_records.items():
            for row in records:
                writer.writerow({
                    "metric": metric,
                    "x_min": row["x_min"],
                    "x_max": row["x_max"],
                    "x_center": row["x"],
                    "samples": row["count"],
                    "p05": row["p05"],
                    "p50": row["p50"],
                    "p95": row["p95"],
                })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--session", action="append", type=Path, default=[])
    parser.add_argument("--output", type=Path, required=True,
                        help="output path without extension")
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--bin-width", type=int, default=50)
    parser.add_argument("--workload-bin-width", type=int, default=20)
    parser.add_argument("--minimum-bin-count", type=int, default=10)
    parser.add_argument("--point-bytes", type=int, default=16)
    args = parser.parse_args()
    if (args.bin_width <= 0 or args.workload_bin_width <= 0
            or args.minimum_bin_count <= 0 or args.point_bytes <= 0):
        parser.error("bin widths, minimum count, and point bytes must be positive")
    sessions = read_sessions(args.summary, args.session)
    if not sessions:
        parser.error("provide --summary or at least one --session")

    local_windows = read_local_window_records(args.summary)
    values, stage_latencies = collect(sessions, args.point_bytes, local_windows)
    if not values["local_window"]:
        parser.error("no local_graph_nodes records found in JSON or console logs")
    records = {}
    for metric, samples in values.items():
        width = args.workload_bin_width if metric == "astar" else args.bin_width
        records[metric] = binned(samples, width, args.minimum_bin_count)
    write_binned_csv(args.output_csv, records)

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 7.5,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 6.8,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    figure, axes = plt.subplots(
        1, 2, figsize=(7.15, 2.45), gridspec_kw={"width_ratios": [1.0, 1.65]}
    )

    local_samples = [value for _, value in values["local_window"]]
    expanded_samples = [value for _, value in values["expanded"]]
    draw_grouped(
        axes[0], ["Local\nwindow", "A*\nexpanded"],
        [np.percentile(local_samples, 50), np.percentile(expanded_samples, 50)],
        [np.percentile(local_samples, 95), np.percentile(expanded_samples, 95)],
        ["P50", "P95"], "(a)", "Node count",
    )

    latency_stages = ["cloud", "background", "planner", "astar"]
    latency_categories = ["Cloud", "Graph\nupdate", "Planner\ntick", "Active\nA*"]
    latency_mean = [np.mean(stage_latencies[stage]) for stage in latency_stages]
    latency_p95 = [np.percentile(stage_latencies[stage], 95)
                   for stage in latency_stages]
    draw_grouped(
        axes[1], latency_categories, latency_mean, latency_p95,
        ["Mean", "P95"], "(b)", "Time (ms)", decimals=1,
    )
    figure.subplots_adjust(left=0.065, right=0.995, top=0.93, bottom=0.22,
                           wspace=0.34)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0.02)
    figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight", pad_inches=0.02)
    figure.savefig(args.output.with_suffix(".png"), dpi=300,
                   bbox_inches="tight", pad_inches=0.02)
    plt.close(figure)

    maxima = {metric: max((row["p95"] for row in rows), default=math.nan)
              for metric, rows in records.items()}
    active_searches = len(values["expanded"])
    edge_values = np.asarray([value for _, value in values["edge_evaluations"]])
    print(f"sessions: {len(sessions)}")
    print(
        f"local-window samples: {len(local_samples)}; nodes P50/P95: "
        f"{np.percentile(local_samples, 50):.0f}/"
        f"{np.percentile(local_samples, 95):.0f}"
    )
    print(
        f"active searches: {active_searches}; expanded nodes P50/P95: "
        f"{np.percentile([value for _, value in values['expanded']], 50):.0f}/"
        f"{np.percentile([value for _, value in values['expanded']], 95):.0f}; "
        "edge evaluations P50/P95: "
        f"{np.percentile(edge_values, 50):.0f}/{np.percentile(edge_values, 95):.0f}"
    )
    print(f"maximum workload-bin P95 A* latency: {maxima['astar']:.2f} ms")
    print(f"maximum graph-snapshot payload P95: {maxima['graph']:.3f} MB")
    print(f"maximum combined retained payload P95: {maxima['total']:.3f} MB")


if __name__ == "__main__":
    main()
