#!/usr/bin/env python3
"""Archive and plot the September 12 z=60.6 m ScaleNav/FAR backup flights."""

import argparse
import csv
import hashlib
import json
import re
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_speed_trajectories import load_flight


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DATA = HERE / "test_data/backup_z60p6_20260912"
OUTPUT = HERE / "pics/experiments/backup_z60p6_20260912"
BATCHES = (
    ("scalenav_z60p6_semantic_20260912_183241", "run_20260912_183241_179330"),
    ("far_z60p6_no_semantic_20260912_183409", "run_20260912_183409_184093"),
)
COLORS = ("#cf3e36", "#ed913e", "#3567ae")


def records(session):
    with (session / "index.jsonl").open() as stream:
        for line in stream:
            yield json.loads(line.replace(":-inf", ":-Infinity")
                             .replace(":inf", ":Infinity").replace(":nan", ":NaN"))


def observed_cloud(session, events, start_ns, end_ns):
    odom = [event for event in events if event["kind"] == "odom"]
    stamps = np.array([event["stamp_ns"] for event in odom], dtype=np.int64)
    samples = []
    next_stamp = start_ns
    frame_count = 0
    maximum_sync_ms = 0.0
    for event in events:
        stamp = event["stamp_ns"]
        if event["kind"] != "pointcloud" or not next_stamp <= stamp <= end_ns:
            continue
        next_stamp = stamp + 200_000_000
        if event["data"]["frame_id"] != "base_link":
            raise ValueError("Expected body-frame point cloud")
        index = int(np.searchsorted(stamps, stamp))
        candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(odom)]
        nearest = min(candidates, key=lambda candidate: abs(int(stamps[candidate]) - stamp))
        sync_ms = abs(int(stamps[nearest]) - stamp) / 1e6
        if sync_ms > 50.0:
            raise ValueError(f"Unsynchronized cloud: {sync_ms:.1f} ms")
        maximum_sync_ms = max(maximum_sync_ms, sync_ms)
        frame_count += 1
        if event["data"]["stored_points"] == 0:
            continue
        with (session / event["file"]).open() as stream:
            for line in stream:
                if line.startswith("DATA"):
                    if line.strip() != "DATA ascii":
                        raise ValueError("Expected ASCII PCD")
                    break
            points = np.loadtxt(stream, usecols=(0, 1, 2), ndmin=2)
        pose = odom[nearest]["data"]
        quaternion = np.array(pose["orientation"], dtype=float)
        quaternion /= np.linalg.norm(quaternion)
        vector = quaternion[:3]
        cross = 2.0 * np.cross(vector, points)
        world = points + quaternion[3] * cross + np.cross(vector, cross) + pose["position"]
        world = world[np.isfinite(world).all(axis=1) & (np.abs(world[:, 2] - 60.6) <= 2.0)]
        samples.append(world)
    cloud = np.vstack(samples) if samples else np.empty((0, 3))
    if len(cloud):
        _, indices = np.unique(np.floor(cloud / 0.25).astype(np.int64), axis=0, return_index=True)
        cloud = cloud[indices]
    return cloud, {"sampled_frames": frame_count, "retained_points": len(cloud),
                   "maximum_pose_sync_ms": maximum_sync_ms}


def extract():
    DATA.mkdir(parents=True, exist_ok=True)
    metadata = {"purpose": "Backup only; excluded from main tables and aggregate_metrics.csv",
                "scene": "Scene identifier not recorded; do not infer Map2 or Map5 from mission coordinates",
                "cloud": "Union of observed body-frame obstacle clouds transformed by nearest odometry; "
                         "0.2 s frame sampling, z=58.6..62.6 m, 0.25 m voxel deduplication; not ground truth",
                "flights": []}
    arrays = {}
    for archive_name, run_name in BATCHES:
        source = ROOT / "scalenav_ws/src/aut_test/results" / run_name
        archive = HERE / "test_data/closed_loop" / archive_name
        archive.mkdir(parents=True, exist_ok=True)
        files = [source / name for name in ("config.json", "metrics.json", "summary.csv")]
        files += sorted(source.glob("trial_*.json")) + sorted(source.glob("trial_*_stack.log"))
        hashes = {}
        for path in files:
            shutil.copy2(path, archive / path.name)
            hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        config = json.loads((archive / "config.json").read_text())
        assert config["start"] == [0.0, 0.0, 60.6] and config["goal"] == [0.0, 140.0, 60.6]
        metric = json.loads((archive / "metrics.json").read_text())
        (archive / "README.md").write_text(
            f"# {archive_name}\n\nBackup experiment; excluded from the main paper tables.\n\n"
            f"Source: `scalenav_ws/src/aut_test/results/{run_name}`.\n\n"
            f"Stack: `{config['stack']}`; semantics: `{config['semantic']}`; "
            f"completed flights: {metric['valid_flights']}; successes: {metric['outcomes'].get('success', 0)}.\n\n"
            "Mission: `(0,0,60.6)` to `(0,140,60.6)` m; timeout 90 s; "
            "goal tolerances 0.5 m and 0.3 m/s. The requested ten-flight batch was stopped early. "
            "Trial outcomes are preserved as recorded, including ScaleNav trial 2 marked success.\n\n"
            "Configuration, metrics, summary, trial records and console logs are byte-identical copies. "
            "Scene identifier is not recorded. Runtime settings are preserved in the console logs. "
            "ScaleNav logs show local sliding graph and map radii of 40 m, semantics enabled, "
            "and no GCN selector. FAR has semantics disabled.\n\n"
            "Comparison, extracted plotting data and checksums: `../../backup_z60p6_20260912/`.\n")
        with (archive / "summary.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            assert row["outcome"] == "success"
            session = Path(row["session_dir"])
            events = list(records(session))
            flight = load_flight(session / "index.jsonl", np.array(config["goal"]),
                                 0.5, 0.3, return_timestamps=True)
            positions, speeds, _, _, stamps = flight
            key = f"{config['stack']}_{row['trial']}"
            elapsed = (stamps - stamps[0]) / 1e9
            cloud, cloud_info = observed_cloud(session, events, int(stamps[0]), int(stamps[-1]))
            local = [event for event in events if event["kind"] == "local_goal"
                     and stamps[0] <= event["stamp_ns"] <= stamps[-1]]
            local_positions = np.array([event["data"]["position"] for event in local])
            arrays.update({f"{key}_position": positions, f"{key}_speed": speeds,
                           f"{key}_elapsed": elapsed, f"{key}_stamp_ns": stamps,
                           f"{key}_cloud": cloud, f"{key}_local_goal": local_positions})
            with (DATA / f"{key}_trajectory.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("stamp_ns", "elapsed_s", "x_m", "y_m", "z_m", "speed_mps"))
                writer.writerows((int(stamp), float(time), *position, float(speed))
                                 for stamp, time, position, speed in zip(stamps, elapsed, positions, speeds))
            lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
            midpoint_y = (positions[1:, 1] + positions[:-1, 1]) / 2
            segment_lengths = {f"y_{lower}_{upper}": float(lengths[(midpoint_y >= lower) &
                               (midpoint_y < upper)].sum()) for lower, upper in ((0, 40), (40, 80), (80, 120), (120, 145))}
            updates = [event for event in events if event["kind"] == "timing" and
                       event["data"].get("module") == "planner" and stamps[0] <= event["stamp_ns"] <= stamps[-1]]
            direct = next((event for event in updates if event["data"].get("mission_goal_direct")), None)
            nonempty = next((event for event in events if event["kind"] == "pointcloud"
                            and event["data"]["stored_points"] and event["stamp_ns"] >= stamps[0]), None)
            console = (archive / f"trial_{int(row['trial']):04d}_stack.log").read_text()
            shortcut_matches = re.findall(r"\[ScaleNav bubble shortcut\] removed=(\d+) path_nodes=(\d+)->(\d+)", console)
            info = {"key": key, "label": f"{'ScaleNav' if config['stack'] == 'scalenav' else 'FAR'} trial {row['trial']}",
                    "summary": row, "archive": str(archive.relative_to(ROOT)), "source_run": run_name,
                    "source_sha256": hashes, "semantic": config["semantic"], "cloud": cloud_info,
                    "plot_duration_s": float(elapsed[-1]), "plot_path_m": float(lengths.sum()),
                    "path_by_y_band_m": segment_lengths, "max_abs_x_m": float(np.abs(positions[:, 0]).max()),
                    "first_local_goal": local_positions[0].tolist(),
                    "first_local_goal_delay_s": float((local[0]["stamp_ns"] - stamps[0]) / 1e9),
                    "first_obstacle_return_s": float((nonempty["stamp_ns"] - stamps[0]) / 1e9) if nonempty else None,
                    "direct_goal_first_diagnostic_s": float((direct["stamp_ns"] - stamps[0]) / 1e9) if direct else None,
                    "shortcut_log_count": len(shortcut_matches),
                    "max_nodes_removed_in_logged_shortcut": max((int(match[0]) for match in shortcut_matches), default=0),
                    "frontier_commands": re.findall(r"\[ScaleNav frontier command\].*", console)}
            metadata["flights"].append(info)
    np.savez_compressed(DATA / "plot_data.npz", **arrays)
    (DATA / "analysis.json").write_text(json.dumps(metadata, indent=2) + "\n")


def plot():
    metadata = json.loads((DATA / "analysis.json").read_text())
    with np.load(DATA / "plot_data.npz") as arrays:
        cloud = np.vstack([arrays[f"{flight['key']}_cloud"] for flight in metadata["flights"]])
        plt.rcParams.update({"font.size": 10, "font.family": "DejaVu Sans"})
        figure = plt.figure(figsize=(12, 9), layout="constrained")
        grid = figure.add_gridspec(2, 2, height_ratios=(1.5, 1))
        overview = figure.add_subplot(grid[0, :])
        early = figure.add_subplot(grid[1, 0])
        speed_axis = figure.add_subplot(grid[1, 1])
        for axis in (overview, early):
            axis.scatter(cloud[:, 1], cloud[:, 0], s=2.5, color="#667785", alpha=0.3,
                         linewidths=0, rasterized=True, label="Observed obstacle cloud (all 3 flights)")
            axis.scatter([0], [0], color="#1f2937", marker="o", s=45, zorder=6)
            axis.set(xlabel="World y / mission direction (m)", ylabel="World x / lateral position (m)")
            axis.set_aspect("equal", adjustable="box")
            axis.grid(alpha=0.2)
        for index, flight in enumerate(metadata["flights"]):
            key = flight["key"]
            position, elapsed, speed = (arrays[f"{key}_{field}"] for field in ("position", "elapsed", "speed"))
            line_style = "--" if key.startswith("far") else (":" if index == 1 else "-")
            label = f"{flight['label']}: {float(flight['summary']['path_m']):.2f} m, {float(flight['summary']['duration_s']):.2f} s"
            for axis in (overview, early):
                axis.plot(position[:, 1], position[:, 0], color=COLORS[index], linestyle=line_style,
                          linewidth=2.2, label=label, zorder=4)
            means = [speed[(elapsed >= second) & (elapsed < second + 1)].mean()
                     for second in range(int(elapsed[-1]) + 1)]
            speed_axis.plot(np.arange(len(means)) + 0.5, means, color=COLORS[index],
                            linestyle=line_style, linewidth=2, label=flight["label"])
        overview.scatter([140], [0], color="#1f2937", marker="*", s=140, zorder=6)
        overview.annotate("Goal", (140, 0), xytext=(5, 8), textcoords="offset points")
        overview.set_xlim(-8, 150)
        trajectories = np.vstack([arrays[f"{flight['key']}_position"] for flight in metadata["flights"]])
        overview.set_ylim(min(-22, trajectories[:, 0].min() - 12), max(22, trajectories[:, 0].max() + 12))
        overview.set_title("All recorded trajectories with observed point cloud | z = 60.6 m", fontweight="bold")
        overview.legend(loc="upper left", fontsize=9, framealpha=0.9)
        early.set(xlim=(-3, 42), ylim=(-10, 24), title="Early-flight detail")
        speed_axis.set(xlabel="Elapsed mission time (s)", ylabel="Speed (m/s)",
                       title="Odometry speed, 1-second means", xlim=(0, 36), ylim=(0, 7))
        speed_axis.grid(alpha=0.2)
        speed_axis.legend(frameon=False)
        figure.supxlabel("Backup only: ScaleNav semantics ON (n=2), FAR semantics OFF (n=1). No GCN run.\n"
                            "Cloud: measured surfaces at z=58.6–62.6 m; empty regions are not certified free space.",
                            fontsize=9)
        OUTPUT.mkdir(parents=True, exist_ok=True)
        for extension in ("png", "pdf"):
            figure.savefig(OUTPUT / f"trajectory_pointcloud_comparison.{extension}", dpi=240)
        plt.close(figure)
    print(OUTPUT / "trajectory_pointcloud_comparison.png")
    for flight in metadata["flights"]:
        print(flight["label"], "path bands", flight["path_by_y_band_m"], "cloud", flight["cloud"],
              "first goal delay", flight["first_local_goal_delay_s"], "shortcuts", flight["shortcut_log_count"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extract", action="store_true", help="Archive raw batches and extract plotting data from local sessions")
    arguments = parser.parse_args()
    if arguments.extract:
        extract()
    plot()
