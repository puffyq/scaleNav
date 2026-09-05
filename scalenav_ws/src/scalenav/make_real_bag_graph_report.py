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
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D


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


def render_snapshot(snapshot: dict, destination: Path, title: str) -> None:
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

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["DejaVu Serif"],
        "font.size": 8.0,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
    })
    fig, axis = plt.subplots(figsize=(7.16, 4.05), dpi=300)
    axis.set_facecolor("white")

    if edges:
        axis.add_collection(LineCollection(
            [edge[:, :2] for edge in edges], colors="#91A3AA",
            linewidths=0.32, alpha=0.34, zorder=1,
        ))
    if len(nodes):
        axis.scatter(nodes[:, 0], nodes[:, 1], s=4.4, color="#536F79",
                     alpha=0.80, linewidths=0.0, zorder=2)
    if len(selected_path) >= 2:
        line, = axis.plot(selected_path[:, 0], selected_path[:, 1], color="#007C83",
                          linewidth=2.15, solid_capstyle="round", zorder=6)
        line.set_path_effects([
            path_effects.Stroke(linewidth=3.8, foreground="white"),
            path_effects.Normal(),
        ])

    selected_semantic = np.zeros(len(semantic_points), dtype=bool)
    if len(frontier) and len(semantic_points):
        selected_semantic = np.linalg.norm(semantic_points - frontier[0], axis=1) < 0.15
    other_semantics = semantic_points[~selected_semantic]
    if len(other_semantics):
        axis.scatter(other_semantics[:, 0], other_semantics[:, 1], marker="o", s=36,
                     color="#D34E45", edgecolors="white", linewidths=0.65,
                     alpha=0.92, zorder=7)
    if len(vehicle):
        axis.scatter(vehicle[:, 0], vehicle[:, 1], marker="^", s=78,
                     color="#263840", edgecolors="white", linewidths=0.8, zorder=9)
        axis.annotate(r"vehicle $\mathbf{p}_t$", vehicle[0, :2], xytext=(-8, 12),
                      textcoords="offset points", ha="right", va="bottom",
                      color="#263840", fontweight="bold", fontsize=8.2,
                      arrowprops=dict(arrowstyle="-", color="#263840", lw=0.55))
    if len(frontier):
        axis.scatter(frontier[:, 0], frontier[:, 1], marker="*", s=175,
                     color="#E28A17", edgecolors="white", linewidths=1.1, zorder=10)
        axis.scatter(frontier[:, 0], frontier[:, 1], marker="*", s=175,
                     facecolors="none", edgecolors="#9C5708", linewidths=0.65, zorder=11)
        axis.annotate(r"frontier goal $\mathbf{g}^{\mathrm{front}}_t$",
                      frontier[0, :2], xytext=(8, 0), textcoords="offset points",
                      ha="left", va="center", color="#A55C09", fontweight="bold",
                      fontsize=8.4,
                      arrowprops=dict(arrowstyle="-", color="#A55C09", lw=0.6))

    plot_groups = [array for array in (nodes, semantic_points, vehicle, frontier)
                   if len(array)]
    if plot_groups:
        plotted = np.vstack(plot_groups)
        low = plotted[:, :2].min(axis=0)
        high = plotted[:, :2].max(axis=0)
        span = np.maximum(high - low, [10.0, 10.0])
        axis.set_xlim(low[0] - 0.055 * span[0], high[0] + 0.10 * span[0])
        axis.set_ylim(low[1] - 0.10 * span[1], high[1] + 0.10 * span[1])
        scale_m = 5.0
        bar_x = low[0] + 0.02 * span[0]
        bar_y = low[1] - 0.055 * span[1]
        axis.plot([bar_x, bar_x + scale_m], [bar_y, bar_y], color="#263840",
                  lw=1.2, solid_capstyle="butt", zorder=12)
        axis.plot([bar_x, bar_x], [bar_y - 0.25, bar_y + 0.25], color="#263840", lw=0.7)
        axis.plot([bar_x + scale_m, bar_x + scale_m], [bar_y - 0.25, bar_y + 0.25],
                  color="#263840", lw=0.7)
        axis.text(bar_x + scale_m / 2.0, bar_y + 0.65, "5 m", ha="center",
                  va="bottom", color="#263840", fontsize=7.3)
    axis.set_aspect("equal", adjustable="box")
    axis.set_axis_off()
    legend = [
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#536F79",
               label="persistent graph nodes", markersize=4.2),
        Line2D([0], [0], color="#91A3AA", linewidth=0.75,
               label="collision-checked edges"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#D34E45",
               markeredgecolor="white", label="current semantic candidates", markersize=5.5),
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#E28A17",
               markeredgecolor="#9C5708", label="selected frontier", markersize=9),
        Line2D([0], [0], color="#007C83", linewidth=2.0,
               label=r"selected A$^*$ graph path"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=5, frameon=False,
               bbox_to_anchor=(0.5, 0.008), fontsize=7.0,
               handlelength=1.8, columnspacing=1.0)
    fig.subplots_adjust(left=0.012, right=0.995, top=0.995, bottom=0.13)
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
    parser.add_argument("--depth-note", default="recorded inverse depth")
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--graph-safe-distance", type=float, default=0.61)
    args = parser.parse_args()

    data = json.loads(args.collection.read_text(encoding="utf-8"))
    snapshot = data.get("planning_snapshot") or load_graph_log_snapshot(args.graph_log)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    png = args.output_dir / "planning_snapshot.png"
    report = args.output_dir / "index.html"
    render_snapshot(snapshot, png, args.title)

    semantics = snapshot.get("current_semantic_points") or snapshot.get("semantic_points", [])
    frontier = snapshot.get("frontier_goal")
    nodes = snapshot.get("nodes", [])
    edges = snapshot.get("edges", [])
    complete = bool(semantics) and frontier is not None
    if complete:
        status = "该图来自同一个 /scalenav/graph 消息，包含 graph、语义点和有效 frontier goal。"
        status_class = "ok"
    else:
        missing = []
        if not semantics:
            missing.append("语义点")
        if frontier is None:
            missing.append("有效 frontier goal")
        status = "该次回放没有同时产生" + "、".join(missing) + "；图中未补画或伪造缺失数据。"
        status_class = "warning"
    frontier_text = "无有效目标" if frontier is None else ", ".join(f"{value:.2f}" for value in frontier)
    stamp_ns = int(snapshot.get("stamp_ns", 0))
    facts = {
        "Graph nodes": len(nodes),
        "Graph edges": len(edges),
        "Semantic points": len(semantics),
        "Frontier goal [m]": frontier_text,
        "ROS stamp [s]": f"{stamp_ns / 1e9:.3f}" if stamp_ns else "unknown",
        "Graph safe distance [m]": f"{args.graph_safe_distance:.2f}",
    }
    fact_html = "".join(
        f"<div><b>{html.escape(str(value))}</b><span>{html.escape(key)}</span></div>"
        for key, value in facts.items()
    )
    preview_files = (
        ("rgb_raw.jpg", "Raw Double Sphere RGB"),
        ("rgb_remap.jpg", "Calibrated perspective RGB"),
        ("depth_raw.png", "Raw inverse-depth q"),
        ("depth_perspective.png", "Calibrated perspective depth"),
    )
    available = [(name, label) for name, label in preview_files if (args.output_dir / name).is_file()]
    previews = "" if not available else "<h2>输入校验</h2><section class='previews'>" + "".join(
        f"<figure><img src='{name}' alt='{html.escape(label)}'><figcaption>{html.escape(label)}</figcaption></figure>"
        for name, label in available
    ) + "</section>"
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
        "<p class='note'>单时刻规划快照。主图不显示累计地面点云或完整里程计轨迹。</p>"
        "<section class='facts'>" + fact_html + "</section><section class='status " + status_class + "'>" + html.escape(status) + "</section>"
        "<img class='main-image' src='planning_snapshot.png' alt='planning graph semantic points and frontier goal'>"
        + previews + "</main></html>",
        encoding="utf-8",
    )
    print(report)


if __name__ == "__main__":
    main()
