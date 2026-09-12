#!/usr/bin/env python3
"""WildOS-style top-down schematic built from a real ScaleNav flight log.

One synchronized planner state at an evidence time t*, cropped to the local
planning window around the vehicle: depth horizon vs. semantic reach, the
persistent free-space graph with witness edges, far-field semantic nodes and
their exposure on witnesses, the selected route, the flown trajectory, and
the goal hierarchy.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
sys.path.insert(0, str(SCRIPT_DIR))

from make_teaser import (  # noqa: E402
    interpolated_odom_position,
    load_events,
    load_graph,
    marker_points,
    marker_points_with_colors,
    marker_pose_position,
    nearest_event,
    quat_rotate,
    semantic_risk_anchors,
    segment_anchor_risk,
)

DEFAULT_SESSION = REPO_ROOT / "log_scalenav/session_20260826_223311_430"
OUTPUT = SCRIPT_DIR / "semantic_foresight_log"

INK = "#24343D"
OBSTACLE = "#9AA4AC"
DEPTH_FILL = "#DCEBF5"
DEPTH_EDGE = "#7FB2D9"
SEM_FILL = "#F9E3E0"
SEM_EDGE = "#E2A39B"
GRAPH = "#8E9AA4"
SELECTED = "#0E8074"
RISK = "#D43D2A"
MISSION = "#C2295B"
FRONTIER = "#E69A2E"
LOCAL = "#8E44AD"

PC_EVERY = 4
PC_STRIDE = 2
PC_MAX = 40000
SEM_REACH_M = 30.0

# Local crop window in world_enu: mission forward is +y, lateral is x.
WIN_BEHIND, WIN_AHEAD = 10.0, 26.0
WIN_LAT = 15.0

event_session: Path


def screen(points: np.ndarray) -> np.ndarray:
    """Rotate world_enu (x East, y North/forward) so +y is screen-right."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return np.column_stack([points[:, 1], -points[:, 0]])


def choose_evidence_time(events: dict[str, list[dict]]) -> int:
    """Earliest graph snapshot with confirmed high-risk semantic labels once
    the mission is underway, so the accumulated semantic memory is still
    sparse and localized."""
    odom = events["odom"]
    for event in events["graph"][::4]:
        y = interpolated_odom_position(odom, event["stamp_ns"])[1]
        if not 25.0 <= y <= 135.0:
            continue
        markers = load_graph(event_session, event)
        if len(semantic_risk_anchors(markers)) >= 3:
            return event["stamp_ns"]
    return events["graph"][len(events["graph"]) // 2]["stamp_ns"]


def line_segments(markers, namespace) -> np.ndarray:
    pts = marker_points(markers, namespace)
    return pts.reshape(-1, 2, 3) if len(pts) >= 2 else np.empty((0, 2, 3))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    global event_session
    event_session = args.session
    events = load_events(args.session)
    odom = events["odom"]

    t_ns = choose_evidence_time(events)
    graph = load_graph(args.session, nearest_event(events["graph"], t_ns))
    vehicle = interpolated_odom_position(odom, t_ns)
    mission_goal = np.asarray(events["goal"][0]["data"]["position"], dtype=float)
    vx, vy = vehicle[0], vehicle[1]

    def in_window(points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        return (
            (points[:, 1] >= vy - WIN_BEHIND)
            & (points[:, 1] <= vy + WIN_AHEAD)
            & (np.abs(points[:, 0] - vx) <= WIN_LAT)
        )

    fig, ax = plt.subplots(figsize=(8.6, 5.4), dpi=300)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # --- horizons ------------------------------------------------------
    v2 = screen(vehicle)[0]
    ax.add_patch(Circle(v2, SEM_REACH_M, facecolor=SEM_FILL,
                        edgecolor=SEM_EDGE, lw=1.2, zorder=1))
    depth_event = nearest_event(events["pointcloud"], t_ns)
    if depth_event.get("file"):
        near = np.loadtxt(args.session / depth_event["file"], skiprows=11)
        near = near.reshape(-1, 3) if near.ndim == 2 else np.empty((0, 3))
        depth_r = float(np.percentile(np.linalg.norm(near[:, :2], axis=1), 95)) \
            if len(near) else 18.0
    else:
        depth_r = 18.0
    depth_r = min(max(depth_r, 10.0), 25.0)
    ax.add_patch(Circle(v2, depth_r, facecolor=DEPTH_FILL,
                        edgecolor=DEPTH_EDGE, lw=1.4, zorder=2))

    # --- obstacles: accumulated point cloud up to t* --------------------
    cloud_events = [e for e in events["pointcloud"]
                    if e.get("file") and e["stamp_ns"] <= t_ns]
    clouds = []
    for event in cloud_events[::PC_EVERY]:
        pts = np.loadtxt(args.session / event["file"], skiprows=11)
        if pts.ndim != 2 or not len(pts):
            continue
        o = nearest_event(odom, event["stamp_ns"])
        rotated = quat_rotate(o["data"]["orientation"], pts[::PC_STRIDE, :3])
        clouds.append(rotated + np.asarray(o["data"]["position"], dtype=float))
    if clouds:
        cloud = np.concatenate(clouds)
        cloud = cloud[in_window(cloud)]
        if len(cloud) > PC_MAX:
            pick = np.random.default_rng(0).choice(len(cloud), PC_MAX,
                                                   replace=False)
            cloud = cloud[pick]
        c2 = screen(cloud)
        ax.scatter(c2[:, 0], c2[:, 1], s=1.2, color=OBSTACLE, alpha=0.65,
                   linewidths=0, zorder=3)

    # --- graph witnesses (cropped); nodes are witness endpoints -----------
    def crop_segments(segments: np.ndarray) -> np.ndarray:
        if not len(segments):
            return segments
        mid = segments.mean(axis=1)
        return segments[in_window(mid)]

    witness_segments = crop_segments(
        line_segments(graph, "epic_edge_witness_paths"))
    if len(witness_segments):
        w2 = screen(witness_segments.reshape(-1, 3)).reshape(-1, 2, 2)
        for seg in w2:
            ax.plot(seg[:, 0], seg[:, 1], color=GRAPH, lw=0.9, alpha=0.8,
                    zorder=4)

    # --- semantic nodes + exposure field on witnesses ---------------------
    anchors = semantic_risk_anchors(graph)
    if len(witness_segments) and len(anchors):
        risk = segment_anchor_risk(witness_segments, anchors)
        for seg3, r in zip(witness_segments, risk):
            if r <= 0.20:
                continue
            s2 = screen(seg3)
            ax.plot(s2[:, 0], s2[:, 1], color=RISK, lw=0.8 + 3.0 * r,
                    alpha=0.10 + 0.45 * r, zorder=5, solid_capstyle="round")
    anchors_in = anchors[in_window(anchors)] if len(anchors) else anchors
    for a in anchors_in:
        a2 = screen(a)[0]
        ax.add_patch(Circle(a2, 1.4, facecolor=RISK, edgecolor="none",
                            alpha=0.14, zorder=5))
        ax.add_patch(Circle(a2, 0.7, facecolor=RISK, edgecolor="none",
                            alpha=0.22, zorder=5))
    if len(anchors_in):
        a2 = screen(anchors_in)
        ax.scatter(a2[:, 0], a2[:, 1], s=42, facecolor=RISK,
                   edgecolor="white", lw=0.9, zorder=8)

    # --- graph nodes (witness endpoints) -----------------------------------
    if len(witness_segments):
        endpoints = witness_segments.reshape(-1, 3)
        n2 = screen(endpoints)
        ax.scatter(n2[:, 0], n2[:, 1], s=18, facecolor="white",
                   edgecolor=INK, lw=0.7, zorder=7)

    # --- selected route ------------------------------------------------------
    selected = marker_points(graph, "epic_selected_witness_path")
    if len(selected) >= 2:
        s2 = screen(selected)
        ax.plot(s2[:, 0], s2[:, 1], color=SELECTED, lw=3.0, zorder=7,
                solid_capstyle="round")

    # --- trajectory ----------------------------------------------------------
    traj = np.asarray([e["data"]["position"] for e in odom], dtype=float)
    past = np.asarray([e["stamp_ns"] for e in odom], dtype=np.int64) <= t_ns
    t2 = screen(traj)
    ax.plot(t2[past, 0], t2[past, 1], color=INK, lw=1.6, zorder=6)
    ax.plot(t2[~past, 0], t2[~past, 1], color=INK, lw=1.2, ls=(0, (4, 3)),
            alpha=0.45, zorder=6)

    # --- vehicle + goal hierarchy ---------------------------------------------
    ax.scatter(*v2, marker="^", s=170, facecolor=INK, edgecolor="white",
               lw=1.0, zorder=10)
    frontier = marker_pose_position(graph, "epic_frontier_goal")
    local = marker_pose_position(graph, "epic_local_goal")
    g2 = screen(mission_goal)[0]
    dist_goal = float(np.linalg.norm(mission_goal[:2] - vehicle[:2]))
    goal_visible = (
        vy - WIN_BEHIND <= mission_goal[1] <= vy + WIN_AHEAD
        and abs(mission_goal[0] - vx) <= WIN_LAT
    )
    if goal_visible:
        ax.scatter(*g2, marker="*", s=300, facecolor=MISSION,
                   edgecolor="white", lw=1.1, zorder=10)
        ax.annotate("mission goal", g2, xytext=(g2[0] + 1.5, g2[1] + 3.0),
                    fontsize=7, color=MISSION, fontweight="bold", zorder=10)
    else:
        direction = (g2 - v2) / max(np.linalg.norm(g2 - v2), 1e-6)
        tip = v2 + direction * (WIN_AHEAD - 6.0)
        ax.add_patch(FancyArrowPatch(v2 + direction * 3.0, tip,
                                     arrowstyle="-|>", mutation_scale=14,
                                     color=MISSION, lw=1.6, ls=(0, (5, 3)),
                                     zorder=9))
        ax.scatter(*tip, marker="*", s=260, facecolor=MISSION,
                   edgecolor="white", lw=1.1, zorder=10)
        ax.annotate(f"mission goal ({dist_goal:.0f} m)", tip,
                    xytext=(tip[0] - 1.0, tip[1] + 2.6), fontsize=7,
                    color=MISSION, fontweight="bold", ha="center", zorder=10)
    if frontier is not None:
        f2 = screen(frontier)[0]
        ax.scatter(*f2, marker="D", s=120, facecolor=FRONTIER,
                   edgecolor="white", lw=1.1, zorder=10)
        ax.annotate("frontier goal", f2, xytext=(f2[0] + 0.8, f2[1] + 3.0),
                    fontsize=7, color=FRONTIER, fontweight="bold", zorder=10,
                    arrowprops=dict(arrowstyle="-", color=FRONTIER, lw=0.8))
    if local is not None:
        l2 = screen(local)[0]
        ax.scatter(*l2, s=95, facecolor=LOCAL, edgecolor="white", lw=1.1,
                   zorder=10)
        ax.annotate("local goal", l2, xytext=(l2[0] - 1.0, l2[1] + 2.8),
                    fontsize=7, color=LOCAL, fontweight="bold", zorder=10,
                    arrowprops=dict(arrowstyle="-", color=LOCAL, lw=0.8))

    # --- horizon labels are covered by the legend; arcs speak for themselves --

    # --- legend ------------------------------------------------------------------
    handles = [
        Line2D([], [], marker="s", color="none", markerfacecolor=OBSTACLE,
               markersize=8, label="obstacle points"),
        Line2D([], [], marker="o", color="none", markerfacecolor=DEPTH_FILL,
               markeredgecolor=DEPTH_EDGE, markersize=10, label="depth horizon"),
        Line2D([], [], marker="o", color="none", markerfacecolor=SEM_FILL,
               markeredgecolor=SEM_EDGE, markersize=10, label="semantic reach"),
        Line2D([], [], marker="o", color=GRAPH, markerfacecolor="white",
               markeredgecolor=INK, markersize=6, lw=0.9,
               label="graph node + witness edge"),
        Line2D([], [], marker="o", color="none", markerfacecolor=RISK,
               markersize=8, label="far-field semantic node"),
        Line2D([], [], color=RISK, lw=3, alpha=0.6,
               label="semantic exposure on witness"),
        Line2D([], [], color=SELECTED, lw=2.6, label="selected route"),
        Line2D([], [], color=INK, lw=1.6, label=r"trajectory to $t^*$"),
        Line2D([], [], color=INK, lw=1.2, ls=(0, (4, 3)), alpha=0.5,
               label=r"trajectory after $t^*$"),
        Line2D([], [], marker="*", color="none", markerfacecolor=MISSION,
               markersize=12, label="mission goal"),
        Line2D([], [], marker="D", color="none", markerfacecolor=FRONTIER,
               markersize=8, label="frontier goal"),
        Line2D([], [], marker="o", color="none", markerfacecolor=LOCAL,
               markersize=8, label="local goal"),
    ]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.02),
              ncols=4, frameon=False, fontsize=6.3, handletextpad=0.5,
              columnspacing=1.1)

    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(vy - WIN_BEHIND - 2.0, vy + WIN_AHEAD + 2.0)
    ax.set_ylim(-(vx + WIN_LAT + 2.0), -(vx - WIN_LAT - 2.0))
    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.13)

    fig.savefig(f"{args.output}.pdf")
    fig.savefig(f"{args.output}.png")
    print(f"t*={t_ns}, depth_r={depth_r:.1f} m -> {args.output}.pdf/.png")


if __name__ == "__main__":
    main()
