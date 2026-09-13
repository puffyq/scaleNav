#!/usr/bin/env python3
"""Fig. 2: TopoGuide pipeline -- single-row, six panels, one synchronized log.

(a) input (RGB+query / depth+odom), (b) semantic valuation, (c) topology
graph, (d) FrontierGCN ranking, (e) A* acceptance, (f) execution.  All
panels render logged data from the same t* as the teaser
(session_20260906_223557_314).  Colored arrows show who consumes what:
red = semantic stream, teal = geometry stream, purple = GCN proposal,
orange = moving local goal.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch


HERE = Path(__file__).resolve().parent
HELPER = HERE / "make_teaser_candidate_v4.py"
OUT = HERE / "system_architecture_pipeline"

spec = importlib.util.spec_from_file_location("teaser_v4", HELPER)
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)

SEM_C, TOPO_C, GCN_C, EXEC_C = "#B44D5A", "#2D8C74", "#7856D8", "#D3820F"
INK = "#26343C"


def load_all():
    events = T.load_events()
    rgb_event = T.event_for_file(events, "rgb", T.RGB_FILE)
    stamp = int(rgb_event["stamp_ns"])
    gcn_event = T.nearest(events, "gcn_frontier_column", stamp)
    odom_event = T.nearest(events, "odom", stamp)
    rgb = np.asarray(Image.open(T.SESSION / T.RGB_FILE).convert("RGB"))
    semantic = np.clip(T.read_scalar(T.SEMANTIC_FILE), 0.0, 1.0)
    depth = T.read_scalar(T.DEPTH_FILE)
    graph = json.loads((T.SESSION / T.GRAPH_FILE).read_text(encoding="utf-8"))
    origin = np.asarray(odom_event["data"]["position"], dtype=float)
    yaw = T.yaw_from_quaternion(odom_event["data"]["orientation"])
    trail = []
    for e in events:
        if e.get("kind") == "odom":
            dt = (int(e["stamp_ns"]) - stamp) / 1e9
            if -2.0 <= dt <= 34.0:
                trail.append((dt, np.asarray(e["data"]["position"], dtype=float)))
    trail.sort()
    return (rgb, semantic, depth, graph, origin, yaw,
            int(gcn_event["data"]["column"]), trail)


def style_panel(axis, xlim=None, ylim=None, equal=True, image=False):
    if xlim:
        axis.set_xlim(*xlim)
    if ylim:
        axis.set_ylim(*ylim)
    if equal:
        axis.set_aspect("equal",
                        adjustable="datalim" if image else "box")
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_visible(False)


def chip(ax, text, x=0.03, y=0.05, fc=INK, va="bottom", size=4.2, ha="left"):
    ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va, color="white",
            fontsize=size, fontweight="bold", linespacing=1.12,
            bbox=dict(boxstyle="square,pad=0.13", facecolor=fc,
                      edgecolor="none", alpha=0.92))


def nodes_colored(graph, origin, yaw):
    m = next(m for m in graph["markers"]
             if m.get("ns") == "scalenav_skeleton_nodes")
    pts = T.body_view(np.asarray(m["points"], dtype=float), origin, yaw)
    cols = np.asarray(m["colors"], dtype=float)
    return pts, cols


def graph_layers(axis, graph, origin, yaw, with_route=False, selected=None,
                 with_goals=False, with_witness=False):
    bv = lambda xyz: T.body_view(xyz, origin, yaw)
    axis.fill_between([5.5, 31.0], [19.0, 19.0], [38.0, 38.0],
                      color="#D9DEDF", alpha=0.88, zorder=0)
    edges = bv(T.points(graph, "scalenav_skeleton_edges")).reshape(-1, 2, 2)
    axis.add_collection(LineCollection(edges, colors="#A9B7BC",
                                       linewidths=0.45, alpha=0.66, zorder=2))
    nodes, cols = nodes_colored(graph, origin, yaw)
    axis.scatter(nodes[:, 0], nodes[:, 1], s=5.4, c=cols, alpha=0.95,
                 edgecolors="white", linewidths=0.2, zorder=3)
    links = bv(T.points(graph, "scalenav_semantic_links"))
    if len(links):
        axis.add_collection(LineCollection(
            links.reshape(-1, 2, 2), colors="#E49A3A", linewidths=0.6,
            linestyles=(0, (1.2, 1.5)), alpha=0.5, zorder=2))
    sem_pts = bv(T.points(graph, "scalenav_semantic_points"))
    axis.scatter(sem_pts[::8, 0], sem_pts[::8, 1], marker="*", s=18,
                 color="#D6544D", alpha=0.5, zorder=4)
    current = bv(T.points(graph, "scalenav_current_semantic_points"))
    axis.scatter(current[:, 0], current[:, 1], marker="*", s=85,
                 color="#D6544D", edgecolors="white", linewidths=0.55,
                 alpha=0.95, zorder=5)
    if with_route:
        path = bv(T.points(graph, "scalenav_astar_topology_path"))
        if len(path) > 1:
            axis.plot(path[:, 0], path[:, 1], color="white", lw=3.2, zorder=5)
            axis.plot(path[:, 0], path[:, 1], color="#00878B", lw=2.0,
                      zorder=6)
    if with_witness:
        witness = bv(T.points(graph, "scalenav_polynomial_witness_path"))
        if len(witness) > 1:
            axis.plot(witness[:, 0], witness[:, 1], color="#E58A17", lw=1.5,
                      ls=(0, (4, 2)), zorder=7)
            for frac in (0.35, 0.7):
                i = int(frac * (len(witness) - 1))
                axis.text(witness[i, 0] + 0.4, witness[i, 1] + 0.4, "\u2713",
                          color="#2E7D46", fontsize=6.0, fontweight="bold",
                          fontfamily="DejaVu Sans", zorder=8)
    if selected is not None:
        for column, angle in enumerate(np.deg2rad([40, 20, 0, -20, -40])):
            endpoint = np.array([-16.0 * np.sin(angle), 16.0 * np.cos(angle)])
            active = column == selected
            axis.annotate("", xy=endpoint, xytext=(0, 0),
                          arrowprops=dict(arrowstyle="-|>", mutation_scale=8,
                                          color=GCN_C if active else "#B9A8E8",
                                          lw=2.6 if active else 1.0,
                                          alpha=1.0 if active else 0.85),
                          zorder=8)
        axis.text(16.0 * -np.sin(np.deg2rad(0)), 17.2, "C", ha="center",
                  fontsize=5.0, color=GCN_C, fontweight="bold", zorder=9)
    axis.scatter(0, 0, marker="^", s=48, color="#273941", edgecolors="white",
                 linewidths=0.7, zorder=10)
    if with_goals:
        frontier = bv(T.pose(graph, "scalenav_frontier_goal")[None, :])[0]
        axis.scatter(frontier[0], frontier[1], marker="p", s=68,
                     color="#D37714", edgecolors="white", linewidths=0.7,
                     zorder=10)
        local = bv(T.pose(graph, "scalenav_local_goal")[None, :])[0]
        axis.scatter(local[0], local[1], marker="D", s=38, color="#B44D8A",
                     edgecolors="white", linewidths=0.6, zorder=10)
        return frontier, local
    return None, None


def main() -> None:
    rgb, semantic, depth, graph, origin, yaw, selected, trail = load_all()

    plt.rcParams.update({
        "font.family": "serif", "font.size": 7,
        "pdf.fonttype": 42, "ps.fonttype": 42,
        "figure.facecolor": "#FFFFFF", "savefig.facecolor": "#FFFFFF",
    })
    fig = plt.figure(figsize=(7.16, 2.02))
    grid = fig.add_gridspec(1, 6, wspace=0.115, left=0.008, right=0.998,
                            top=0.80, bottom=0.13)

    XWIN, YWIN = (-14, 19), (-4, 34)

    # (a) input: RGB+query over depth+odom -----------------------------------
    ga = grid[0].subgridspec(2, 1, hspace=0.12)
    ax_rgb = fig.add_subplot(ga[0])
    ax_dep = fig.add_subplot(ga[1])
    ax_rgb.imshow(rgb, interpolation="nearest")
    shown_depth = np.ma.masked_where(depth >= T.DEPTH_CLIP_M, depth)
    depth_cmap = plt.get_cmap("viridis").copy()
    depth_cmap.set_bad("white")
    ax_dep.imshow(shown_depth, cmap=depth_cmap, vmin=0.0, vmax=T.DEPTH_CLIP_M,
                  interpolation="nearest")
    chip(ax_rgb, 'RGB + q', fc=SEM_C)
    style_panel(ax_rgb, image=True)
    style_panel(ax_dep, image=True)

    # (b) semantic valuation: heatmap top-aligned with the RGB above ---------
    ax_sem = fig.add_subplot(grid[1])
    heatmap = LinearSegmentedColormap.from_list(
        "pearl", ["#160B39", "#482173", "#8B2981", "#D94E63", "#F6D746"])
    ax_sem.imshow(semantic, cmap=heatmap, vmin=0.0, vmax=1.0,
                  interpolation="nearest")
    ax_sem.set_xlim(-0.5, 159.5)
    ax_sem.set_ylim(215.0, -0.5)   # white space below: tops align with RGB
    ax_sem.set_aspect("equal", adjustable="box")
    ax_sem.set_xticks([])
    ax_sem.set_yticks([])
    for spine in ax_sem.spines.values():
        spine.set_visible(False)

    # (c) topology graph ------------------------------------------------------
    ax_topo = fig.add_subplot(grid[2])
    graph_layers(ax_topo, graph, origin, yaw)
    handles = [
        Line2D([], [], marker="o", ls="none", markersize=4.0,
               markerfacecolor="#526D77", markeredgecolor="white",
               markeredgewidth=0.3, label="verified"),
        Line2D([], [], marker="o", ls="none", markersize=4.0,
               markerfacecolor="#D6544D", markeredgecolor="white",
               markeredgewidth=0.3, label="sem.-scored"),
        Line2D([], [], marker="*", ls="none", markersize=6.2,
               markerfacecolor="#D6544D", markeredgecolor="white",
               markeredgewidth=0.3,
               label="provisional"),
    ]
    ax_topo.legend(handles=handles, loc="lower left", fontsize=3.7,
                   frameon=False, borderpad=0.22,
                   labelspacing=0.2, handletextpad=0.4, borderaxespad=0.18)
    style_panel(ax_topo, XWIN, YWIN)

    # (d) FrontierGCN ranking --------------------------------------------------
    ax_gcn = fig.add_subplot(grid[3])
    graph_layers(ax_gcn, graph, origin, yaw, selected=selected)
    goal_body = T.body_view(np.array([[0.0, 140.0, 1.6]]), origin, yaw)[0]
    gdir = goal_body / np.linalg.norm(goal_body)
    ax_gcn.annotate("goal", xytext=(gdir[0] * 15.4, gdir[1] * 15.4),
                    xy=(gdir[0] * 13.0, gdir[1] * 13.0),
                    fontsize=4.8, color="#7A6410", fontweight="bold",
                    ha="left", va="center",
                    arrowprops=dict(arrowstyle="-|>", mutation_scale=8,
                                    color="#B69A2E", lw=1.2), zorder=9)
    style_panel(ax_gcn, XWIN, YWIN)

    # (e) A* acceptance ----------------------------------------------------------
    ax_astar = fig.add_subplot(grid[4])
    frontier, local = graph_layers(ax_astar, graph, origin, yaw,
                                   with_route=True, with_goals=True,
                                   with_witness=True)
    ax_astar.annotate("frontier goal", frontier, xytext=(6, -4),
                      textcoords="offset points", fontsize=4.6,
                      color="#D37714", fontweight="bold", ha="left",
                      va="center",
                      arrowprops=dict(arrowstyle="-", color="#D37714",
                                      lw=0.8))
    ax_astar.annotate("local goal", local, xytext=(1.2, 10.2),
                      textcoords="data", fontsize=4.6, color="#B44D8A",
                      fontweight="bold", ha="left", va="center",
                      arrowprops=dict(arrowstyle="->", color="#B44D8A",
                                      lw=0.8, shrinkA=2, shrinkB=3))
    style_panel(ax_astar, (-10, 14), (-3, 31))

    # (f) execution --------------------------------------------------------------
    ax_exec = fig.add_subplot(grid[5])
    ax_exec.fill_between([5.5, 31.0], [19.0, 19.0], [38.0, 38.0],
                         color="#D9DEDF", alpha=0.88, zorder=0)
    pts = np.array([p for _, p in trail])
    dt = np.array([t for t, _ in trail])
    body = T.body_view(pts, origin, yaw)
    future = dt >= 0.0
    ax_exec.plot(body[~future, 0], body[~future, 1], color="#9AA7AD", lw=0.9,
                 zorder=3)
    ax_exec.plot(body[future, 0], body[future, 1], color="#00878B", lw=1.7,
                 zorder=4)
    ax_exec.scatter(body[future, 0][-1], body[future, 1][-1], s=22,
                    color="#00878B", zorder=5, edgecolors="white",
                    linewidths=0.5)
    ax_exec.scatter(0, 0, marker="^", s=48, color="#273941",
                    edgecolors="white", linewidths=0.7, zorder=10)
    style_panel(ax_exec, XWIN, YWIN)

    # ---------------------------------------------------------------- furniture
    fig.canvas.draw()
    chips_def = [(ax_rgb, "(a) Input", "#526D77"),
                 (ax_sem, "(b) Semantic", SEM_C),
                 (ax_topo, "(c) Topology", TOPO_C),
                 (ax_gcn, "(d) FrontierGCN", GCN_C),
                 (ax_astar, "(e) A* accept", "#3973B7"),
                 (ax_exec, "(f) Execute", EXEC_C)]
    header_y = max(ax.get_position().y1
                   for ax, _, _ in chips_def) + 0.012
    for ax, text, color in chips_def:
        pos = ax.get_position()
        fig.text(pos.x0 + pos.width / 2, header_y, " " + text + " ",
                 fontsize=6.2, fontweight="bold", ha="center", va="bottom",
                 color="white",
                 bbox=dict(boxstyle="round,pad=0.24", facecolor=color,
                           edgecolor="none", alpha=0.95))

    p_rgb, p_dep = ax_rgb.get_position(), ax_dep.get_position()
    p_sem, p_topo = ax_sem.get_position(), ax_topo.get_position()
    p_gcn, p_astar, p_exec = (ax_gcn.get_position(), ax_astar.get_position(),
                              ax_exec.get_position())

    def flow(p_from, p_to, color="#5A6167", lw=1.6, ls="-", rad=0.0,
             scale=11):
        fig.add_artist(FancyArrowPatch(
            p_from, p_to, transform=fig.transFigure, arrowstyle="-|>",
            mutation_scale=scale, color=color, lw=lw, linestyle=ls,
            connectionstyle=f"arc3,rad={rad}", zorder=9))

    mid = lambda p: (p.y0 + p.y1) / 2
    # depth + odometry feeds the graph directly (horizontal link)
    flow((p_dep.x1 + 0.006, mid(p_dep)),
         (p_topo.x0 - 0.006, mid(p_dep)),
         color=TOPO_C, lw=1.8, rad=0.0)
    fig.text((p_dep.x1 + p_topo.x0) / 2 - 0.022, mid(p_dep) + 0.035,
             "depth + odom\nrange $\\leq$ 20 m", ha="center",
             va="bottom", fontsize=5.2, color=TOPO_C, fontweight="bold",
             linespacing=1.2)

    fig.savefig(OUT.with_suffix(".pdf"), dpi=400)
    fig.savefig(OUT.with_suffix(".png"), dpi=400)
    plt.close(fig)
    print(f"wrote {OUT}.pdf/.png")


if __name__ == "__main__":
    main()
