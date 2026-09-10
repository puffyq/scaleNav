#!/usr/bin/env python3
"""Compose the PEARL resolution contrast with a 3-D graph-structure view."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects
from matplotlib import cm, colors
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from PIL import Image
from scipy import ndimage


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
HELPER_PATH = HERE / "plot_obstacle_heatmap_trajectory.py"
TOP_IMAGE = HERE / "pics/candidates/pearl_powerline_resolution_4grid_red_blue.png"
SUMMARY = REPO_ROOT / "scalenav_ws/src/aut_test/results/run_20260906_225711_315755/summary.csv"
TRUTH_NPZ = HERE / "pics/map4_mesh_truth_3d_20260906.npz"
SESSION = REPO_ROOT / "log_scalenav/session_20260906_225714_272"
GRAPH_FILE = "graph/graph_96.json"
OUT = HERE / "pics/candidates/pearl_powerline_graph3d_candidate_v26"


def load_helper():
    spec = importlib.util.spec_from_file_location("map4_plot_helper", HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(HELPER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_summary(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def shortest_success_session(summary_path: Path) -> Path:
    rows = load_summary(summary_path)
    successes = [row for row in rows if row.get("outcome") == "success"]
    if not successes:
        raise RuntimeError("summary has no successful trials")
    row = min(successes, key=lambda item: float(item["path_m"]))
    return Path(row["session_dir"]) / "index.jsonl"


def marker(graph: dict, name: str) -> dict:
    return next((m for m in graph.get("markers", []) if m.get("ns") == name), {})


def points(graph: dict, name: str) -> np.ndarray:
    values = marker(graph, name).get("points", [])
    return np.asarray(values, dtype=float).reshape(-1, 3) if values else np.empty((0, 3))


def pose(graph: dict, name: str) -> np.ndarray:
    value = marker(graph, name).get("pose", {}).get("position")
    return np.asarray(value, dtype=float) if value is not None else np.zeros(3, dtype=float)


def box_for_points(*arrays: np.ndarray, pad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    valid = [arr for arr in arrays if len(arr)]
    if not valid:
        raise RuntimeError("no points to bound")
    stacked = np.vstack(valid)
    return stacked.min(axis=0) - pad, stacked.max(axis=0) + pad


def points_in_box(points: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.all((points >= lo) & (points <= hi), axis=1)


def load_graph(graph_path: Path) -> dict:
    return json.loads(graph_path.read_text(encoding="utf-8"))


def load_map4_truth() -> tuple[np.ndarray, float]:
    data = np.load(TRUTH_NPZ)
    occupied = np.asarray(data["occupied"]) > 0
    bounds = np.asarray(data["bounds"], dtype=float)
    resolution = float(data["resolution"])
    cells = np.argwhere(occupied)
    points = bounds[:, 0] + (cells.astype(float) + 0.5) * resolution
    return points, resolution


def trajectory_line(ax, xs, ys, zs=None, color="#d62828", lw=2.3):
    if zs is None:
        line, = ax.plot(xs, ys, color=color, linewidth=lw)
    else:
        line, = ax.plot(xs, ys, zs, color=color, linewidth=lw)
    line.set_path_effects([
        patheffects.Stroke(linewidth=lw + 1.8, foreground="white", alpha=0.96),
        patheffects.Normal(),
    ])
    return line


def style_3d(ax):
    ax.xaxis.pane.set_facecolor((1, 1, 1, 0))
    ax.yaxis.pane.set_facecolor((1, 1, 1, 0))
    ax.zaxis.pane.set_facecolor((1, 1, 1, 0))
    ax.xaxis.pane.set_edgecolor((1, 1, 1, 0))
    ax.yaxis.pane.set_edgecolor((1, 1, 1, 0))
    ax.zaxis.pane.set_edgecolor((1, 1, 1, 0))
    ax.xaxis._axinfo["grid"]["linewidth"] = 0.0
    ax.yaxis._axinfo["grid"]["linewidth"] = 0.0
    ax.zaxis._axinfo["grid"]["linewidth"] = 0.0
    ax.xaxis.line.set_color((1, 1, 1, 0))
    ax.yaxis.line.set_color((1, 1, 1, 0))
    ax.zaxis.line.set_color((1, 1, 1, 0))
    ax.tick_params(pad=1)


def draw_height_voxels(ax, cloud: np.ndarray, lo: np.ndarray,
                       hi: np.ndarray, voxel_size: float = 1.0) -> None:
    """Render the local skeleton cloud as physical-size height-colored voxels."""
    voxel_size = np.array([voxel_size, voxel_size, voxel_size], dtype=float)
    origin = np.floor(lo / voxel_size) * voxel_size
    extent = np.ceil(hi / voxel_size) * voxel_size
    shape = np.maximum(np.ceil((extent - origin) / voxel_size).astype(int), 1)
    edges = [
        origin[i] + np.arange(shape[i] + 1, dtype=float) * voxel_size[i]
        for i in range(3)
    ]
    indices = np.floor((cloud - origin) / voxel_size).astype(int)
    valid = np.all((indices >= 0) & (indices < shape), axis=1)
    indices = indices[valid]
    filled = np.zeros(tuple(shape), dtype=bool)
    if len(indices):
        filled[indices[:, 0], indices[:, 1], indices[:, 2]] = True

    norm = colors.Normalize(vmin=float(lo[2]), vmax=float(hi[2]))
    cmap = plt.colormaps.get_cmap("viridis")
    facecolors = np.zeros(filled.shape + (4,), dtype=float)
    for iz in range(filled.shape[2]):
        facecolors[:, :, iz, :] = cmap(norm(edges[2][iz] + 0.5 * voxel_size[2]))
    grid = np.meshgrid(edges[0], edges[1], edges[2], indexing="ij")
    ax.voxels(
        *grid, filled,
        facecolors=facecolors, edgecolor="none", linewidth=0.0,
        alpha=0.20,
    )


def draw_tall_thin_structures(ax, cloud: np.ndarray, lo: np.ndarray,
                              hi: np.ndarray, voxel_size: float = 0.5) -> None:
    origin = np.floor(lo / voxel_size) * voxel_size
    shape = np.maximum(
        np.ceil((hi - origin) / voxel_size).astype(int), 1,
    )
    indices = np.floor((cloud - origin) / voxel_size).astype(int)
    valid = np.all((indices >= 0) & (indices < shape), axis=1)
    occupied = np.zeros(tuple(shape), dtype=bool)
    occupied[tuple(indices[valid].T)] = True
    labels, count = ndimage.label(occupied, structure=ndimage.generate_binary_structure(3, 1))
    for label in range(1, count + 1):
        cells = np.argwhere(labels == label)
        if len(cells) < 12:
            continue
        span = cells.max(axis=0) - cells.min(axis=0) + 1
        if max(span[0], span[1]) > 6 or span[2] < 12 or len(cells) > 140:
            continue
        points = origin + (cells.astype(float) + 0.5) * voxel_size
        points = points[:, [1, 0, 2]]
        ax.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            s=7.0, color="#263238", alpha=0.92, linewidths=0,
            depthshade=False, zorder=7,
        )


def obstacle_height_profile(trajectory: np.ndarray, truth: np.ndarray,
                            y_grid: np.ndarray) -> np.ndarray:
    """Estimate the latest obstacle ceiling along the flown corridor.

    At each mission-progress position, use the highest occupied Map4 truth
    voxel in a short longitudinal/lateral tube around the aircraft.  This is
    a truth-derived obstacle-height trace, not an executed-flight clearance
    measurement.
    """
    x_ref = np.interp(y_grid, trajectory[:, 1], trajectory[:, 0])
    height = np.full_like(y_grid, np.nan, dtype=float)
    for i, (y, x) in enumerate(zip(y_grid, x_ref)):
        local = truth[
            (np.abs(truth[:, 1] - y) <= 1.5)
            & (np.abs(truth[:, 0] - x) <= 5.0)
            & (truth[:, 2] >= 0.0)
        ]
        if len(local):
            height[i] = float(np.percentile(local[:, 2], 95.0))
    valid = np.isfinite(height)
    if valid.sum() >= 2:
        height[~valid] = np.interp(y_grid[~valid], y_grid[valid], height[valid])
    return height


def draw_graph_snapshot(ax, graph: dict, trajectory: np.ndarray,
                        truth: np.ndarray) -> tuple[float, float]:
    nodes = points(graph, "scalenav_skeleton_nodes")
    edges = points(graph, "scalenav_skeleton_edges").reshape(-1, 2, 3)
    witness = points(graph, "scalenav_polynomial_witness_path")
    astar = points(graph, "scalenav_astar_topology_path")
    vehicle = pose(graph, "scalenav_vehicle_pose")
    local_goal = pose(graph, "scalenav_local_goal")
    frontier_goal = pose(graph, "scalenav_frontier_goal")
    global_goal = pose(graph, "scalenav_global_goal")

    # Preserve the original full Map4 voxel composition.  The mission axis y
    # is horizontal in the projection; only the camera roll adds the small
    # three-dimensional turn requested for the side view.
    lo = np.array([-40.0, -5.0, 0.0], dtype=float)
    hi = np.array([40.0, 145.0, 9.0], dtype=float)
    truth = truth[points_in_box(truth, lo, hi)]
    traj = trajectory[points_in_box(trajectory, lo, hi)]
    witness = witness[points_in_box(witness, lo, hi)]
    astar = astar[points_in_box(astar, lo, hi)]

    plot_lo = lo[[1, 0, 2]]
    plot_hi = hi[[1, 0, 2]]
    draw_height_voxels(ax, truth[:, [1, 0, 2]], plot_lo, plot_hi,
                       voxel_size=1.0)
    if len(astar):
        ax.plot(astar[:, 1], astar[:, 0], astar[:, 2], color="#087F8C",
                linewidth=1.4, zorder=6)
    if len(witness):
        ax.plot(witness[:, 1], witness[:, 0], witness[:, 2],
                color="#E8890C", linewidth=1.5, linestyle=(0, (4, 2)),
                zorder=7)
    trajectory_line(ax, traj[:, 1], traj[:, 0], traj[:, 2],
                    color="#D62828", lw=2.8)

    ax.scatter(trajectory[0, 1], trajectory[0, 0], trajectory[0, 2],
               s=32, color="#24343D", edgecolors="white", linewidths=0.6,
               depthshade=False, zorder=8)
    ax.scatter(global_goal[1], global_goal[0], global_goal[2],
               s=58, marker="*", color="#D62828", edgecolors="white",
               linewidths=0.6, depthshade=False, zorder=8)
    ax.scatter(vehicle[1], vehicle[0], vehicle[2],
               s=28, marker="D", color="#C46E13", edgecolors="white",
               linewidths=0.6, depthshade=False, zorder=8)

    ax.set_xlim(float(lo[1]), float(hi[1]))
    ax.set_ylim(float(lo[0]), float(hi[0]))
    ax.set_zlim(float(lo[2]), float(hi[2]))
    ax.set_xlabel("")
    ax.set_ylabel("")
    ax.set_zlabel("")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_box_aspect((12.0, 2.4, 1.25), zoom=0.9)
    ax.set_proj_type("ortho")
    ax.view_init(elev=28, azim=-90, roll=0)
    style_3d(ax)
    ax.legend(
        handles=[
            Line2D([], [], color="#D62828", lw=2.0, label="flown trajectory"),
            Line2D([], [], color="#087F8C", lw=1.5, label="A* route"),
            Line2D([], [], color="#E8890C", lw=1.5, linestyle=(0, (4, 2)),
                   label="polynomial fit"),
        ],
        loc="upper left", bbox_to_anchor=(0.02, 0.98), fontsize=5.2,
        frameon=False, ncol=2, handlelength=2.0, columnspacing=0.8,
    )
    return float(trajectory[:, 1].min()), float(trajectory[:, 1].max())


def draw_altitude_profile(ax, trajectory: np.ndarray, witness: np.ndarray,
                          y_window: tuple[float, float]) -> None:
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_facecolor("none")
    ax.axvspan(y_window[0], y_window[1], color="#6F90AE", alpha=0.09, linewidth=0)
    ax.plot(trajectory[:, 1], trajectory[:, 2], color="#D62828", linewidth=2.3)
    if len(witness):
        ax.plot(witness[:, 1], witness[:, 2], color="#E8890C", linewidth=1.8,
                linestyle=(0, (4, 2)))
    ax.set_xlim(0, 145)
    z_values = [trajectory[:, 2]]
    if len(witness):
        z_values.append(witness[:, 2])
    zmin = min(float(values.min()) for values in z_values)
    zmax = max(float(values.max()) for values in z_values)
    ax.set_ylim(max(0.0, zmin - 0.3), zmax + 0.3)
    ax.set_xlabel("Mission progress y (m)")
    ax.set_ylabel("Altitude z (m)")
    ax.set_title("Flight altitude profile", fontweight="semibold", pad=4)
    ax.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.85)
    imin = int(np.argmin(trajectory[:, 2]))
    imax = int(np.argmax(trajectory[:, 2]))
    for idx, label, offset in ((imin, "low", (6, -0.35)), (imax, "high", (-18, 0.25))):
        y = float(trajectory[idx, 1])
        z = float(trajectory[idx, 2])
        ax.scatter([y], [z], s=20, color="#D62828", zorder=4)
        ax.annotate(
            label, xy=(y, z), xytext=(y + offset[0], z + offset[1]),
            fontsize=6.3, color="#444444",
            arrowprops=dict(arrowstyle="-", color="#888888", lw=0.8),
        )
    ax.legend(
        handles=[
            Line2D([], [], color="#D62828", lw=2.0, label="flown trajectory"),
            Line2D([], [], color="#E8890C", lw=1.8,
                   linestyle=(0, (4, 2)), label="witness path"),
        ],
        loc="upper right", fontsize=6.3, frameon=False,
    )


def draw_side_voxels(ax, cloud: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                     voxel_size: float = 0.5) -> None:
    """Project occupied Map4 cells onto the y-z plane as a side elevation."""
    origin = np.array([lo[1], lo[2]], dtype=float)
    shape = np.maximum(
        np.ceil((np.array([hi[1], hi[2]]) - origin) / voxel_size).astype(int),
        1,
    )
    edges_y = origin[0] + np.arange(shape[0] + 1, dtype=float) * voxel_size
    edges_z = origin[1] + np.arange(shape[1] + 1, dtype=float) * voxel_size
    filled = np.zeros(tuple(shape), dtype=bool)
    if len(cloud):
        yi = np.floor((cloud[:, 1] - origin[0]) / voxel_size).astype(int)
        zi = np.floor((cloud[:, 2] - origin[1]) / voxel_size).astype(int)
        valid = (yi >= 0) & (yi < shape[0]) & (zi >= 0) & (zi < shape[1])
        if valid.any():
            filled[yi[valid], zi[valid]] = True
    cmap = plt.colormaps.get_cmap("viridis")
    facecolors = np.zeros(filled.shape + (4,), dtype=float)
    denom = max(int(shape[1]), 1)
    for iz in range(int(shape[1])):
        facecolors[:, iz, :] = cmap((iz + 0.5) / denom)
    facecolors[..., 3] = np.where(filled, 0.78, 0.0)
    ax.pcolormesh(
        edges_y, edges_z, np.transpose(facecolors, (1, 0, 2)),
        shading="flat", zorder=1, rasterized=True,
    )


def draw_side_view(ax, graph: dict, trajectory: np.ndarray,
                   truth: np.ndarray) -> None:
    """Side (y-z) view: mission progress fills the panel left to right."""
    witness = points(graph, "scalenav_polynomial_witness_path")
    astar = points(graph, "scalenav_astar_topology_path")
    vehicle = pose(graph, "scalenav_vehicle_pose")
    local_goal = pose(graph, "scalenav_local_goal")
    frontier_goal = pose(graph, "scalenav_frontier_goal")
    global_goal = pose(graph, "scalenav_global_goal")

    lo = np.array([-40.0, -5.0, 0.0], dtype=float)
    hi = np.array([40.0, 145.0, 9.0], dtype=float)
    truth = truth[points_in_box(truth, lo, hi)]
    if len(witness):
        witness = witness[points_in_box(witness, lo, hi)]
    if len(astar):
        astar = astar[points_in_box(astar, lo, hi)]
    traj = trajectory[points_in_box(trajectory, lo, hi)]

    draw_side_voxels(ax, truth, lo, hi, voxel_size=0.5)

    trajectory_line(ax, traj[:, 1], traj[:, 2], color="#D62828", lw=3.0)

    ax.scatter(vehicle[1], vehicle[2], s=34, color="#24343D",
               edgecolors="white", linewidths=0.6, zorder=8)
    ax.scatter(local_goal[1], local_goal[2], s=42, marker="D",
               color="#A64E88", edgecolors="white", linewidths=0.6, zorder=8)
    ax.scatter(frontier_goal[1], frontier_goal[2], s=74, marker="*",
               color="#C46E13", edgecolors="white", linewidths=0.6, zorder=8)
    ax.scatter(global_goal[1], global_goal[2], s=58, marker="X",
               color="#8A8F95", edgecolors="white", linewidths=0.6, zorder=8)

    ax.set_xlim(float(lo[1]), float(hi[1]))
    ax.set_ylim(float(lo[2]), float(hi[2]))
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.set_xlabel("Mission progress y (m)")
    ax.set_ylabel("Altitude z (m)")
    ax.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.85)
    ax.legend(
        handles=[
            Line2D([], [], color="#D62828", lw=2.0, label="flown trajectory"),
            Line2D([], [], color="#3E5F8A", lw=0, marker="s",
                   markersize=4.0, label="Map4 truth voxels"),
        ],
        loc="upper left", fontsize=6.5,
        frameon=True, framealpha=0.9, edgecolor="none",
        ncol=2, handlelength=2.0, columnspacing=0.8,
    )


def main() -> None:
    helper = load_helper()
    index_path = shortest_success_session(SUMMARY)
    start_ns, end_ns = helper.mission_interval(index_path)
    trajectory, _ = helper.load_trajectory(index_path, start_ns, end_ns)

    graph = load_graph(SESSION / GRAPH_FILE)
    truth, _ = load_map4_truth()

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 7.8,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })

    # Render the 3-D band on its own wide canvas, then composite it with the
    # resolution panel at exactly the same pixel width; mplot3d sizes content
    # to a square viewport inside its axes, so in-figure placement cannot be
    # controlled reliably.
    band_fig = plt.figure(figsize=(11.0, 2.6))
    graph_ax = band_fig.add_axes([0.0, 0.0, 1.0, 1.0], projection="3d")
    draw_graph_snapshot(graph_ax, graph, trajectory, truth)
    band_path = OUT.with_name(OUT.name + "_band_tmp.png")
    band_fig.savefig(band_path, dpi=300, bbox_inches="tight", pad_inches=0.0)
    plt.close(band_fig)

    top = Image.open(TOP_IMAGE).convert("RGB")
    band = Image.open(band_path).convert("RGB")
    # Crop white margins left by the mplot3d viewport, keeping the legend.
    mask = (np.asarray(band) < 245).any(axis=2)
    cols = np.where(mask.any(axis=0))[0]
    rows = np.where(mask.any(axis=1))[0]
    pad = 12
    band = band.crop((max(int(cols[0]) - pad, 0), max(int(rows[0]) - pad, 0),
                      min(int(cols[-1]) + pad, band.width),
                      min(int(rows[-1]) + pad, band.height)))
    width = top.width
    band_height = round(band.height * width / band.width)
    band = band.resize((width, band_height), Image.Resampling.LANCZOS)
    combo = Image.new("RGB", (width, top.height + band_height), "white")
    combo.paste(top, (0, 0))
    combo.paste(band, (0, top.height))
    combo.save(OUT.with_suffix(".png"), dpi=(300, 300))
    band_path.unlink()
    print(f"wrote {OUT}.png")


if __name__ == "__main__":
    main()