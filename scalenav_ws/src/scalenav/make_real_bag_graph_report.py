#!/usr/bin/env python3
"""Render one coherent replay-time graph/semantic/frontier planning snapshot."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patheffects as path_effects
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection


def load_graph_log_snapshot(path: Path | None) -> dict:
    """Compatibility fallback for collections made before planning_snapshot existed."""
    candidates: list[dict] = []
    if path is None or not path.is_file():
        return {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get("event") == "graph_snapshot":
            candidates.append(record)
    if not candidates:
        return {}
    record = max(candidates, key=lambda item: (bool(item.get("found")), item.get("stamp_ns", 0)))
    nodes = record.get("nodes", [])
    edges = []
    for node in nodes:
        for neighbor in node.get("neighbors", []):
            if node.get("id", -1) < neighbor < len(nodes):
                edges.append([node["center"], nodes[neighbor]["center"]])
    semantic_points = [
        {"position": node["center"], "color": [0.88, 0.24, 0.18, 1.0]}
        for node in nodes if node.get("is_virtual_semantic", False)
    ]
    return {
        "stamp_ns": record.get("stamp_ns", 0),
        "nodes": [node["center"] for node in nodes],
        "edges": edges,
        "semantic_edges": [],
        "selected_path": [nodes[index]["center"] for index in record.get("path", [])
                          if 0 <= index < len(nodes)],
        "semantic_points": semantic_points,
        "frontier_goal": record.get("frontier_goal") if record.get("found") else None,
        "local_goal": record.get("local_goal") if record.get("local_goal_valid") else None,
        "mission_goal": record.get("mission_goal"),
        "vehicle": record.get("position"),
    }


def finite_points(values) -> np.ndarray:
    points = np.asarray(values or [], dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        return np.empty((0, 3), dtype=np.float64)
    return points[np.isfinite(points).all(axis=1)]


def quaternion_matrix(quaternion) -> np.ndarray:
    """Return the world-from-body rotation for an xyzw quaternion."""
    values = np.asarray(quaternion, dtype=np.float64).reshape(-1)
    if values.size != 4 or not np.isfinite(values).all():
        return np.eye(3, dtype=np.float64)
    x, y, z, w = values
    norm = float(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float64)


def render_snapshot(snapshot: dict, destination: Path, title: str,
                    gcn_column: int | None = None) -> None:
    nodes = finite_points(snapshot.get("nodes"))
    edges = [finite_points(edge) for edge in snapshot.get("edges", [])]
    edges = [edge for edge in edges if len(edge) == 2]
    selected_path = finite_points(snapshot.get("selected_path"))
    semantics = snapshot.get("current_semantic_points") or snapshot.get("semantic_points", [])
    semantic_points = finite_points([item.get("position") for item in semantics])
    semantic_colors = np.asarray(
        [item.get("color", [0.88, 0.24, 0.18, 1.0]) for item in semantics], dtype=np.float64
    )
    if len(semantic_colors) != len(semantic_points):
        semantic_colors = np.tile([0.88, 0.24, 0.18, 1.0], (len(semantic_points), 1))
    semantic_colors = np.clip(semantic_colors, 0.0, 1.0)
    vehicle = finite_points([snapshot.get("vehicle")])
    frontier = finite_points([snapshot.get("frontier_goal")])
    mission_goal = finite_points([snapshot.get("mission_goal")])

    # The paper view is intentionally limited to the perception evidence and
    # the GCN decision projected back onto the RGB/PEARL image. The graph is
    # used by the selector but is not rendered in this figure.
    media_dir = destination.parent
    rgb_path = media_dir / "rgb_capture.jpg"
    heatmap_path = media_dir / "pearl_heatmap.png"
    overlay_path = media_dir / "pearl_overlay.jpg"
    if rgb_path.is_file() and heatmap_path.is_file() and overlay_path.is_file():
        plt.rcParams.update({
            "font.family": "serif", "font.serif": ["DejaVu Serif"],
            "font.size": 8.0, "pdf.fonttype": 42, "ps.fonttype": 42,
            "svg.fonttype": "none",
        })
        fig, axis = plt.subplots(1, 1, figsize=(7.2, 4.25), dpi=300)
        image = plt.imread(overlay_path)
        axis.imshow(image)
        axis.set_title("GCN direction projected onto RGB + PEARL heatmap",
                       loc="left", fontsize=10.0, fontweight="bold", pad=6)
        axis.set_axis_off()
        height, width = image.shape[:2]
        center_x, center_y = width * 0.5, height * 0.98
        selected = int(gcn_column) if gcn_column is not None and 0 <= int(gcn_column) < 5 else -1
        # Project the recorded ENU mission goal into the current FLU camera
        # plane using the vehicle marker quaternion captured in the same frame.
        target_angle_deg = None
        orientation = snapshot.get("vehicle_orientation")
        if len(vehicle) and len(mission_goal) and orientation is not None:
            body_goal = quaternion_matrix(orientation).T @ (mission_goal[0] - vehicle[0])
            if np.linalg.norm(body_goal[:2]) > 1e-6:
                target_angle_deg = float(np.degrees(np.arctan2(body_goal[1], body_goal[0])))
        # The five body-relative GCN columns are spaced by 20 degrees. Draw
        # all five 35 m rays: gray means rejected, red means selected.
        for column in range(5):
            offset_deg = (column - 2) * 20.0
            # Body-FLU positive yaw points left, while image u grows right.
            endpoint_x = center_x - np.tan(np.deg2rad(offset_deg)) / np.tan(np.deg2rad(45.0)) * width * 0.42
            endpoint_y = height * 0.47
            start = np.array([center_x, center_y], dtype=np.float64)
            end = np.array([endpoint_x, endpoint_y], dtype=np.float64)
            ray = end - start
            ray /= max(float(np.linalg.norm(ray)), 1e-6)
            normal = np.array([-ray[1], ray[0]], dtype=np.float64)
            tip_length = max(16.0, height * 0.038)
            tip_base = end - ray * tip_length
            chosen = column == selected
            color = "#ff3b30" if chosen else "#7b858b"
            alpha = 0.98 if chosen else 0.62
            near_width = height * (0.055 if chosen else 0.026)
            far_width = height * (0.018 if chosen else 0.009)
            outer = np.vstack([
                start + normal * near_width * 0.62,
                start - normal * near_width * 0.62,
                tip_base - normal * far_width * 0.72,
                tip_base + normal * far_width * 0.72,
            ])
            axis.add_patch(Polygon(outer, closed=True, facecolor="white",
                                   edgecolor="white", alpha=0.78 if chosen else 0.55,
                                   linewidth=0.7, zorder=8))
            inner = np.vstack([
                start + normal * near_width * 0.40,
                start - normal * near_width * 0.40,
                tip_base - normal * far_width * 0.40,
                tip_base + normal * far_width * 0.40,
            ])
            axis.add_patch(Polygon(inner, closed=True, facecolor=color,
                                   edgecolor=color, alpha=alpha, linewidth=0.4, zorder=9))
            tip_width = max(far_width * 1.8, 9.0 if chosen else 5.0)
            tip = np.vstack([end, tip_base + normal * tip_width,
                             tip_base - normal * tip_width])
            axis.add_patch(Polygon(tip, closed=True, facecolor=color,
                                   edgecolor="white" if chosen else color,
                                   alpha=alpha, linewidth=1.4 if chosen else 0.4, zorder=10))
        if target_angle_deg is not None:
            # Keep a far-behind goal visible at the edge of the perspective
            # image while retaining its left/right sign.
            target_angle_deg = float(np.clip(target_angle_deg, -70.0, 70.0))
            endpoint_x = center_x - np.tan(np.deg2rad(target_angle_deg)) / np.tan(np.deg2rad(45.0)) * width * 0.42
            endpoint_x = float(np.clip(endpoint_x, width * 0.035, width * 0.965))
            target_start = np.array([center_x, center_y], dtype=np.float64)
            target_end = np.array([endpoint_x, height * 0.40], dtype=np.float64)
            axis.plot([target_start[0], target_end[0]], [target_start[1], target_end[1]],
                      color="white", linewidth=height * 0.010, alpha=0.82,
                      solid_capstyle="round", zorder=10.5)
            axis.plot([target_start[0], target_end[0]], [target_start[1], target_end[1]],
                      color="#21a366", linewidth=height * 0.006, linestyle=(0, (7, 5)),
                      solid_capstyle="round", zorder=11)
            target_ray = target_end - target_start
            target_ray /= max(float(np.linalg.norm(target_ray)), 1e-6)
            target_normal = np.array([-target_ray[1], target_ray[0]], dtype=np.float64)
            target_base = target_end - target_ray * max(15.0, height * 0.034)
            target_tip = np.vstack([
                target_end,
                target_base + target_normal * max(7.0, height * 0.018),
                target_base - target_normal * max(7.0, height * 0.018),
            ])
            axis.add_patch(Polygon(target_tip, closed=True, facecolor="#21a366",
                                   edgecolor="white", linewidth=1.0, zorder=12))
        axis.text(0.03, 0.94,
                  f"red: GCN selected (column {selected})   gray: other directions\n"
                  "green: mission goal direction   35 m",
                  transform=axis.transAxes, color="#ff3b30" if selected >= 0 else "#59656a",
                  fontsize=8.0, fontweight="bold",
                  bbox=dict(facecolor="white", alpha=0.80, edgecolor="none", pad=2.5))
        fig.subplots_adjust(left=0.005, right=0.995, top=0.88, bottom=0.02)
        for output in (destination, destination.with_suffix(".pdf"), destination.with_suffix(".svg")):
            fig.savefig(output, dpi=300, bbox_inches="tight", pad_inches=0.025,
                        facecolor="white")
        plt.close(fig)
        return

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["DejaVu Serif"],
        "font.size": 8.0,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
    })

    # All three panels use one planning snapshot and exactly the same XY limits.
    # This makes the figure read as a pipeline rather than three unrelated maps.
    plot_groups = [array for array in
                   (nodes, semantic_points, selected_path, vehicle, frontier) if len(array)]
    if plot_groups:
        plotted = np.vstack(plot_groups)
        low = plotted[:, :2].min(axis=0)
        high = plotted[:, :2].max(axis=0)
        span = np.maximum(high - low, [10.0, 10.0])
        xlim = (low[0] - 0.08 * span[0], high[0] + 0.12 * span[0])
        ylim = (low[1] - 0.12 * span[1], high[1] + 0.12 * span[1])
    else:
        xlim, ylim = (-5.0, 5.0), (-5.0, 5.0)

    media_dir = destination.parent
    media_files = [media_dir / "rgb_capture.jpg", media_dir / "pearl_heatmap.png",
                   media_dir / "pearl_overlay.jpg"]
    has_media = all(path.is_file() for path in media_files)
    if has_media:
        fig = plt.figure(figsize=(10.4, 6.9), dpi=300)
        grid = fig.add_gridspec(2, 3, height_ratios=(0.92, 1.08), hspace=0.16, wspace=0.08)
        media_axes = [fig.add_subplot(grid[0, index]) for index in range(3)]
        axes = [fig.add_subplot(grid[1, 0])]
        axes.extend(fig.add_subplot(grid[1, index], sharex=axes[0], sharey=axes[0])
                    for index in range(1, 3))
        media_titles = ["(a) Real RGB received by PEARL",
                        "(b) PEARL target heatmap (blue low → yellow high)",
                        "(c) RGB + PEARL projection"]
        for axis, path, media_title in zip(media_axes, media_files, media_titles):
            image = plt.imread(path)
            axis.imshow(image)
            axis.set_title(media_title, loc="left", fontsize=9.2, fontweight="bold", pad=5)
            axis.set_axis_off()
        panel_titles = ["(d) Geometric graph from depth",
                        "(e) Current semantic candidates",
                        "(f) Frontier selected by A*"]
    else:
        fig, axes = plt.subplots(1, 3, figsize=(10.4, 3.75), dpi=300,
                                 sharex=True, sharey=True)
        media_axes = []
        panel_titles = ["(a) Geometric graph from depth",
                        "(b) Current semantic candidates",
                        "(c) Frontier selected by A*"]
    fig.patch.set_facecolor("white")
    for axis in axes:
        axis.set_facecolor("white")
        axis.set_xlim(*xlim)
        axis.set_ylim(*ylim)
        axis.set_aspect("equal", adjustable="box")
        axis.set_axis_off()

    def draw_graph(axis, alpha: float = 1.0) -> None:
        if edges:
            axis.add_collection(LineCollection(
                [edge[:, :2] for edge in edges], colors="#82959C",
                linewidths=0.30, alpha=0.28 * alpha, zorder=1,
            ))
        if len(nodes):
            axis.scatter(nodes[:, 0], nodes[:, 1], s=4.0, color="#526B74",
                         alpha=0.72 * alpha, linewidths=0.0, zorder=2)

    def draw_vehicle(axis, annotate: bool = False) -> None:
        if not len(vehicle):
            return
        axis.scatter(vehicle[:, 0], vehicle[:, 1], marker="^", s=76,
                     color="#263840", edgecolors="white", linewidths=0.8, zorder=9)
        if annotate:
            axis.annotate("vehicle", vehicle[0, :2], xytext=(-7, 11),
                          textcoords="offset points", ha="right", va="bottom",
                          color="#263840", fontweight="bold", fontsize=8.0,
                          arrowprops=dict(arrowstyle="-", color="#263840", lw=0.55))

    # (a) Geometric input: the graph is produced by depth/odometry.
    draw_graph(axes[0])
    draw_vehicle(axes[0], annotate=True)
    axes[0].set_title(panel_titles[0], loc="left",
                      fontsize=9.2, fontweight="bold", pad=5)
    axes[0].text(0.02, 0.03, "nodes + collision-checked edges",
                 transform=axes[0].transAxes, color="#526B74", fontsize=7.5)

    # (b) Semantic input: only the current candidate set is shown.
    draw_graph(axes[1], alpha=0.42)
    if len(semantic_points):
        axes[1].scatter(semantic_points[:, 0], semantic_points[:, 1], marker="o", s=42,
                        color="#D34E45", edgecolors="white", linewidths=0.7,
                        alpha=0.96, zorder=7)
        for index, point in enumerate(semantic_points, start=1):
            axes[1].annotate(str(index), point[:2], xytext=(0, 0),
                             textcoords="offset points", ha="center", va="center",
                             color="white", fontsize=6.2, fontweight="bold", zorder=8)
    axes[1].set_title(panel_titles[1], loc="left",
                      fontsize=9.2, fontweight="bold", pad=5)
    axes[1].text(0.02, 0.03, f"candidate set $C_t$ ({len(semantic_points)} points)",
                 transform=axes[1].transAxes, color="#A63832", fontsize=7.5)

    # (c) Decision: A* selects one candidate/frontier and returns one graph path.
    draw_graph(axes[2], alpha=0.30)
    if len(semantic_points):
        axes[2].scatter(semantic_points[:, 0], semantic_points[:, 1], marker="o", s=34,
                        color="#DFA8A4", edgecolors="white", linewidths=0.55,
                        alpha=0.75, zorder=5)
    if len(selected_path) >= 2:
        line, = axes[2].plot(selected_path[:, 0], selected_path[:, 1], color="#007C83",
                             linewidth=2.15, solid_capstyle="round", zorder=6)
        line.set_path_effects([path_effects.Stroke(linewidth=3.8, foreground="white"),
                               path_effects.Normal()])
    draw_vehicle(axes[2], annotate=False)
    if len(frontier):
        axes[2].scatter(frontier[:, 0], frontier[:, 1], marker="*", s=180,
                        color="#E28A17", edgecolors="white", linewidths=1.0, zorder=10)
        axes[2].scatter(frontier[:, 0], frontier[:, 1], marker="*", s=180,
                        facecolors="none", edgecolors="#9C5708", linewidths=0.7, zorder=11)
        axes[2].annotate("selected frontier", frontier[0, :2], xytext=(8, 1),
                         textcoords="offset points", ha="left", va="center",
                         color="#9C5708", fontweight="bold", fontsize=8.0,
                         arrowprops=dict(arrowstyle="-", color="#9C5708", lw=0.6))
    axes[2].set_title(panel_titles[2], loc="left",
                      fontsize=9.2, fontweight="bold", pad=5)
    axes[2].text(0.02, 0.03, "one frontier + selected graph path",
                 transform=axes[2].transAxes, color="#007C83", fontsize=7.5)

    # A small visual cue reinforces the direction of the pipeline.
    for left_axis, right_axis in zip(axes[:-1], axes[1:]):
        left_box, right_box = left_axis.get_position(), right_axis.get_position()
        fig.text((left_box.x1 + right_box.x0) / 2.0, 0.52, "→", ha="center", va="center",
                 fontsize=16, color="#AAB5B9", fontweight="bold")
    fig.text(0.5, 0.015,
             "depth / odometry  →  semantic projection  →  A* frontier selection",
             ha="center", va="bottom", fontsize=8.2, color="#42545B")
    fig.subplots_adjust(left=0.012, right=0.995, top=0.91, bottom=0.09, wspace=0.08)
    for output in (destination, destination.with_suffix(".pdf"), destination.with_suffix(".svg")):
        fig.savefig(output, dpi=300, bbox_inches="tight", pad_inches=0.025,
                    facecolor="white")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("collection", type=Path)
    parser.add_argument("--graph-log", type=Path)
    parser.add_argument("--odom-csv", type=Path)
    parser.add_argument("--runtime-log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--title", default="0903 graph, semantics and frontier snapshot")
    parser.add_argument("--prompt", default="unknown")
    parser.add_argument("--depth-note", default="recorded inverse depth")
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--graph-safe-distance", type=float, default=0.61)
    args = parser.parse_args()

    data = json.loads(args.collection.read_text(encoding="utf-8"))
    snapshot = data.get("planning_snapshot") or load_graph_log_snapshot(args.graph_log)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    png = args.output_dir / "planning_snapshot.png"
    report = args.output_dir / "index.html"
    media = data.get("media", {})
    render_snapshot(snapshot, png, args.title, media.get("gcn_column"))

    semantics = snapshot.get("current_semantic_points") or snapshot.get("semantic_points", [])
    frontier = snapshot.get("frontier_goal")
    nodes = snapshot.get("nodes", [])
    edges = snapshot.get("edges", [])
    complete = bool(semantics) and frontier is not None
    status = "单张图：真实 RGB 与 PEARL 热力图叠加；红色为 GCN 选中方向，绿色虚线为 ENU 任务目标方向。"
    status_class = "ok" if media.get("gcn_column") is not None else "warning"
    frontier_text = "无有效目标" if frontier is None else ", ".join(f"{value:.2f}" for value in frontier)
    stamp_ns = int(snapshot.get("stamp_ns", 0))
    facts = {
        "PEARL prompt": args.prompt,
        "GCN column": data.get("media", {}).get("gcn_column", "not recorded"),
        "Vehicle altitude [m]": (f"{snapshot['vehicle'][2]:.2f}"
                                  if snapshot.get("vehicle") else "unknown"),
    }
    fact_html = "".join(
        f"<div><b>{html.escape(str(value))}</b><span>{html.escape(key)}</span></div>"
        for key, value in facts.items()
    )
    preview_files = (
        ("rgb_raw.jpg", "Raw Double Sphere RGB"),
        ("rgb_remap.jpg", "Calibrated perspective RGB"),
        ("rgb_capture.jpg", "RGB received by PEARL"),
        ("pearl_heatmap.png", "PEARL target heatmap"),
        ("pearl_overlay.jpg", "RGB + PEARL projection"),
        ("depth_raw.png", "Raw inverse-depth q"),
        ("depth_perspective.png", "Calibrated perspective depth"),
    )
    available = [(name, label) for name, label in preview_files if (args.output_dir / name).is_file()]
    previews = ""
    report.write_text(
        "<!doctype html><html lang='zh-CN'><meta charset='utf-8'><meta name='viewport' content='width=device-width'>"
        "<title>0903 planning snapshot</title><style>body{margin:0;background:#eef1f2;color:#172126;font:15px system-ui}"
        "main{max-width:1220px;margin:auto;padding:24px}h1{font-size:24px;margin:0 0 8px}h2{font-size:18px;margin-top:28px}"
        ".note{color:#59676d;margin:0 0 16px}.facts{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:15px 0}"
        ".facts div{background:#fff;border:1px solid #d5dcdf;padding:12px;border-radius:6px}.facts b,.facts span{display:block}"
        ".facts b{font-size:17px}.facts span{color:#647178;font-size:12px;margin-top:4px}.status{padding:12px;margin:12px 0;border-radius:6px}"
        ".ok{background:#e7f5ef;border:1px solid #72ad93}.warning{background:#fff4d6;border:1px solid #d6a93c}"
        ".main-image{display:block;width:100%;height:auto;background:#fff;border:1px solid #d5dcdf}.previews{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}"
        ".previews figure{margin:0}.previews img{display:block;width:100%;height:auto;border:1px solid #d5dcdf}.previews figcaption{font-size:12px;color:#59676d;margin-top:5px}"
        "@media(max-width:800px){.facts{grid-template-columns:1fr}.previews{grid-template-columns:repeat(2,1fr)}}"
        "</style><main><h1>" + html.escape(args.title) + "</h1>"
        "<p class='note'>真实 RGB 作为 PEARL 输入（prompt=" + html.escape(args.prompt) + "）；红色为 GCN 方向，绿色虚线为任务目标方向。</p>"
        "<section class='facts'>" + fact_html + "</section><section class='status " + status_class + "'>" + html.escape(status) + "</section>"
        "<img class='main-image' src='planning_snapshot.png' alt='planning graph semantic points and frontier goal'>"
        + previews + "</main></html>",
        encoding="utf-8",
    )
    print(report)


if __name__ == "__main__":
    main()
