#!/usr/bin/env python3
"""Plot the shortest selected TopoGuide flight against the longest FAR flight on Map5."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
PLOT_IMPL = HERE / "plot_speed_trajectories.py"
OUT = HERE / "pics" / "candidates" / "map5_best_topoguide_vs_far_worst"
GOAL = np.array([0.0, 200.0, 140.6])

OUR_SUMMARY = HERE / "test_data/closed_loop/gcn_map5_z140p6_semantic_20260911_183259/summary.csv"
FAR_SUMMARY = HERE / "test_data/closed_loop/far_map5_z140p6_no_semantic_20260911_172447/summary.csv"


def load_impl(path=PLOT_IMPL):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    if spec is None or spec.loader is None:
        raise ImportError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def select_row(path: Path, choose_max: bool) -> dict[str, str]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    successful = [row for row in rows if row["outcome"] == "success"]
    return sorted(successful, key=lambda row: float(row["path_m"]), reverse=choose_max)[0]


def cloud_background(rows, flights):
    analysis = load_impl(REPO_ROOT / "scalenav_ws/docs/test_reports/analyze_round_trip_density.py")
    occupied = set()
    for row, flight in zip(rows, flights):
        session = Path(row["session_dir"])
        odom, clouds, _ = analysis.load_index(session)
        with (session / "index.jsonl").open() as stream:
            for line in stream:
                if '"kind":"pointcloud"' in line:
                    record = json.loads(line)
                    if record["data"]["frame_id"] != "base_link":
                        raise ValueError("Point-cloud transform expects base_link")
        cells, frames, points = analysis.load_occupied_cells(
            session, odom, clouds, int(flight[4][0]), int(flight[4][-1]),
            cell_size=0.5, min_z=GOAL[2] - 2.0, max_z=GOAL[2] + 2.0,
        )
        occupied.update(cells)
        print(f"Cloud: {session.name}: {frames} sampled frames, {points} accepted points")
    if not occupied:
        raise ValueError("No observed point cloud in the flight-height slab")
    return (np.asarray(sorted(occupied)) + 0.5) * 0.5


def plot_speed(axis, flight, name, color, linestyle):
    elapsed = (flight[4] - flight[4][0]) / 1e9
    if np.any(np.diff(elapsed) <= 0):
        raise ValueError("Odometry timestamps must increase")
    axis.plot(elapsed, flight[1], color=color, alpha=0.18, linewidth=0.6)
    edges = np.append(np.arange(0, elapsed[-1], 1.0), elapsed[-1])
    indices = np.minimum(np.searchsorted(edges, elapsed, side="right") - 1, len(edges) - 2)
    counts = np.bincount(indices, minlength=len(edges) - 1)
    totals = np.bincount(indices, weights=flight[1], minlength=len(edges) - 1)
    means = np.full(len(counts), np.nan)
    np.divide(totals, counts, out=means, where=counts > 0)
    axis.plot(edges[1:], means, color=color, linestyle=linestyle, linewidth=1.9,
              label=name)
    axis.scatter([elapsed[-1]], [flight[1][-1]], color=color, marker="*", s=60, zorder=5)


def main() -> None:
    impl = load_impl()
    ours = select_row(OUR_SUMMARY, choose_max=False)
    far = select_row(FAR_SUMMARY, choose_max=True)
    for summary in (OUR_SUMMARY, FAR_SUMMARY):
        config = json.loads(summary.with_name("config.json").read_text())
        if config["start"] != [0.0, 0.0, 140.6] or config["goal"] != GOAL.tolist():
            raise ValueError("Comparison requires matching start, goal, and height")
    ours_flight = impl.load_flight(Path(ours["session_dir"]) / "index.jsonl", GOAL, 0.5, 0.3, return_timestamps=True)
    far_flight = impl.load_flight(Path(far["session_dir"]) / "index.jsonl", GOAL, 0.5, 0.3, return_timestamps=True)
    cloud = cloud_background((ours, far), (ours_flight, far_flight))

    plt.rcParams.update({"font.size": 9, "font.family": "DejaVu Sans"})
    fig, (ax, speed_axis) = plt.subplots(2, 1, figsize=(9.0, 7.8),
                                       gridspec_kw={"height_ratios": [1.65, 1]})
    ax.scatter(cloud[:, 1], cloud[:, 0], s=2, c="#929ca6", alpha=0.5,
               linewidths=0, rasterized=True, label="Observed cloud (both flights)", zorder=1)
    ours_xy = np.column_stack((ours_flight[0][:, 1], ours_flight[0][:, 0]))
    far_xy = np.column_stack((far_flight[0][:, 1], far_flight[0][:, 0]))
    ax.plot(ours_xy[:, 0], ours_xy[:, 1], color="#d62728", linewidth=2.6,
            label=f"GCN (ours), best ({float(ours['path_m']):.1f} m)", zorder=3)
    ax.plot(far_xy[:, 0], far_xy[:, 1], color="#7a5195", linewidth=2.2,
            linestyle="--", label=f"FAR worst ({float(far['path_m']):.1f} m)", zorder=2)
    ax.scatter([ours_xy[0, 0]], [ours_xy[0, 1]], s=45, c="#202124",
               marker="o", label="Start", zorder=5)
    ax.scatter([ours_xy[-1, 0]], [ours_xy[-1, 1]], s=90, c="#d62728",
               marker="*", edgecolors="white", linewidths=0.7, label="Goal",
               zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Mission progress $y$ (m)")
    ax.set_ylabel("Lateral position $x$ (m)")
    ax.set_title("Map5 path comparison at $z=140.6$ m")
    trajectory = np.vstack((ours_xy, far_xy))
    ax.set_xlim(-12, 212)
    ax.set_ylim(trajectory[:, 1].min() - 20, trajectory[:, 1].max() + 20)
    ax.grid(True, color="#d9dde1", linewidth=0.6, alpha=0.8)
    ax.legend(frameon=False, loc="best")
    plot_speed(speed_axis, ours_flight, "GCN (ours), semantic on", "#d62728", "-")
    plot_speed(speed_axis, far_flight, "FAR, semantic off", "#7a5195", "--")
    speed_axis.set(xlabel="Elapsed flight time (s)", ylabel="Speed (m/s)",
                   title="Odometry speed: raw samples (faint) and 1-s means")
    speed_axis.set_xlim(left=0)
    speed_axis.set_ylim(bottom=0)
    speed_axis.grid(True, color="#d9dde1", linewidth=0.6, alpha=0.8)
    speed_axis.legend(frameon=False, loc="lower center", ncol=2)
    fig.text(0.5, 0.025,
             "Backup illustration: shortest GCN vs longest FAR by path length, selected from 2 trials each.\n"
             "Not batch averages. Cloud: observed surfaces at z = 138.6–142.6 m, 0.5-m cells; not a complete map.",
             ha="center", fontsize=8, color="#4b5563")
    fig.tight_layout(rect=(0, 0.075, 1, 1))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT.with_suffix(".pdf"), bbox_inches="tight")
    OUT.with_suffix(".json").write_text(json.dumps({
        "selection": "shortest successful GCN and longest successful FAR in the specified batches, by path_m",
        "ours_summary": str(OUR_SUMMARY.relative_to(REPO_ROOT)), "ours": ours,
        "far_summary": str(FAR_SUMMARY.relative_to(REPO_ROOT)), "far": far,
        "cloud": "Union of both selected flight intervals; base_link transformed with nearest odometry; z=138.6..142.6; 0.5m cells; frames at least 0.5s apart; every second stored point",
        "speed": "Odometry velocity magnitude; raw samples and 1-second means, final partial bin retained; no time normalization or padding",
    }, indent=2) + "\n")
    plt.close(fig)
    print(f"GCN (ours) trial {ours['trial']} path={ours['path_m']} m")
    print(f"FAR trial {far['trial']} path={far['path_m']} m")
    print(f"wrote {OUT}.png and {OUT}.pdf")


if __name__ == "__main__":
    main()
