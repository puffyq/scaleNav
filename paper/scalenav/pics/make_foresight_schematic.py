#!/usr/bin/env python3
"""WildOS-style top-down schematic for ScaleNav's semantic foresight.

Pure vector schematic (no log data): a fork corridor, the depth horizon vs.
the semantic reach, the persistent free-space graph with witness polylines,
a far-field semantic node whose Gaussian field bends the selected route to
the far branch before depth arrives, and the three-level goal hierarchy.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT = SCRIPT_DIR / "semantic_foresight_schematic"

INK = "#24343D"
OBSTACLE = "#8A949C"
DEPTH_FILL = "#DCEBF5"
DEPTH_EDGE = "#7FB2D9"
SEM_FILL = "#F9E3E0"
SEM_EDGE = "#E2A39B"
GRAPH_EDGE = "#9AA7B0"
NODE_FACE = "#FFFFFF"
SELECTED = "#0E8074"
REJECTED = "#D97706"
RISK = "#D43D2A"
MISSION = "#C2295B"
FRONTIER = "#E69A2E"
LOCAL = "#8E44AD"

DEPTH_R = 8.0
SEM_R = 14.5


def witness_curve(p0, p1, bend, n=80):
    """Quadratic-bezier-like witness polyline between two nodes."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    mid = 0.5 * (p0 + p1) + np.asarray(bend, dtype=float)
    t = np.linspace(0.0, 1.0, n)[:, None]
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * mid + t**2 * p1


def main() -> None:
    fig, ax = plt.subplots(figsize=(8.6, 5.2), dpi=300)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # --- horizons ------------------------------------------------------
    ax.add_patch(
        Circle((0, 0), SEM_R, facecolor=SEM_FILL, edgecolor=SEM_EDGE,
               lw=1.2, zorder=1))
    ax.add_patch(
        Circle((0, 0), DEPTH_R, facecolor=DEPTH_FILL, edgecolor=DEPTH_EDGE,
               lw=1.4, zorder=2))

    # --- obstacles (gray blobs forming a fork) -------------------------
    obstacles = [
        Ellipse((4.5, 6.2), 7.5, 3.4, angle=12),
        Ellipse((4.5, -6.2), 7.5, 3.4, angle=-12),
        Ellipse((13.5, 0.0), 3.2, 5.0, angle=0),   # fork divider wall
        Ellipse((-3.5, 5.6), 4.5, 2.8, angle=-25),
        Ellipse((-3.5, -5.6), 4.5, 2.8, angle=25),
        Ellipse((17.5, 6.8), 5.0, 3.0, angle=-8),
        Ellipse((17.5, -6.8), 5.0, 3.0, angle=8),
    ]
    for ob in obstacles:
        ob.set(facecolor=OBSTACLE, edgecolor="none", alpha=0.9, zorder=3)
        ax.add_patch(ob)

    # --- graph nodes ----------------------------------------------------
    n_veh = (-1.2, 0.0)
    n_fork = (8.6, 0.0)
    upper = [(11.6, 2.6), (14.8, 3.4), (17.8, 2.8)]
    lower = [(11.6, -2.6), (14.8, -3.4), (17.8, -2.8)]
    n_goal = (20.6, 0.0)

    edges = [
        (n_veh, n_fork, (0.0, 0.0)),
        (n_fork, upper[0], (0.4, 0.6)),
        (upper[0], upper[1], (0.0, 0.35)),
        (upper[1], upper[2], (0.0, -0.3)),
        (upper[2], n_goal, (0.4, -0.9)),
        (n_fork, lower[0], (0.4, -0.6)),
        (lower[0], lower[1], (0.0, -0.35)),
        (lower[1], lower[2], (0.0, 0.3)),
        (lower[2], n_goal, (0.4, 0.9)),
    ]
    witnesses = [witness_curve(a, b, bend) for a, b, bend in edges]
    for w in witnesses:
        ax.plot(w[:, 0], w[:, 1], color=GRAPH_EDGE, lw=1.1, zorder=4)

    # --- far-field semantic node + Gaussian field on upper witness -----
    sem_node = np.asarray([16.4, 4.6])
    risky = witness_curve(upper[1], upper[2], (0.0, -0.3))
    d = np.linalg.norm(risky - sem_node, axis=1)
    exposure = np.exp(-(d**2) / (2 * 1.6**2))
    for i in range(len(risky) - 1):
        ax.plot(
            risky[i : i + 2, 0], risky[i : i + 2, 1],
            color=RISK, lw=1.0 + 6.0 * exposure[i],
            alpha=0.15 + 0.75 * exposure[i], zorder=5,
            solid_capstyle="round",
        )
    halo = Circle(sem_node, 1.7, facecolor=RISK, edgecolor="none",
                  alpha=0.16, zorder=5)
    ax.add_patch(halo)
    ax.add_patch(Circle(sem_node, 1.0, facecolor=RISK, edgecolor="none",
                        alpha=0.22, zorder=5))
    ax.scatter(*sem_node, s=64, marker="o", facecolor=RISK,
               edgecolor="white", lw=1.2, zorder=8)
    ax.annotate("far-field\nsemantic node", sem_node, xytext=(13.2, 7.6),
                fontsize=6.5, color=RISK, ha="center", va="center",
                fontweight="bold", zorder=9,
                arrowprops=dict(arrowstyle="-", color=RISK, lw=0.8))

    # --- routes ----------------------------------------------------------
    rejected = np.vstack([
        witness_curve(n_fork, upper[0], (0.4, 0.6)),
        witness_curve(upper[0], upper[1], (0.0, 0.35)),
    ])
    ax.plot(rejected[:, 0], rejected[:, 1], color=REJECTED, lw=1.8,
            ls=(0, (5, 3)), zorder=6)
    selected = np.vstack([
        witness_curve(n_veh, n_fork, (0.0, 0.0)),
        witness_curve(n_fork, lower[0], (0.4, -0.6)),
        witness_curve(lower[0], lower[1], (0.0, -0.35)),
        witness_curve(lower[1], lower[2], (0.0, 0.3)),
        witness_curve(lower[2], n_goal, (0.4, 0.9)),
    ])
    ax.plot(selected[:, 0], selected[:, 1], color=SELECTED, lw=3.0,
            zorder=7, solid_capstyle="round")

    # --- graph nodes on top ----------------------------------------------
    all_nodes = [n_veh, n_fork, *upper, *lower]
    ax.scatter([p[0] for p in all_nodes], [p[1] for p in all_nodes],
               s=110, facecolor=NODE_FACE, edgecolor=INK, lw=1.3, zorder=8)
    ax.scatter([p[0] for p in all_nodes], [p[1] for p in all_nodes],
               s=26, facecolor=INK, edgecolor="none", zorder=8)

    # --- vehicle ----------------------------------------------------------
    ax.scatter(*n_veh, marker="^", s=170, facecolor=INK, edgecolor="white",
               lw=1.0, zorder=10)
    ax.annotate("vehicle", n_veh, xytext=(-3.4, -1.8), fontsize=7,
                color=INK, fontweight="bold", zorder=10,
                arrowprops=dict(arrowstyle="-", color=INK, lw=0.7))

    # --- goal hierarchy ----------------------------------------------------
    local_pt = selected[np.argmin(np.abs(selected[:, 0] - 4.4))]
    ax.scatter(*local_pt, s=95, marker="o", facecolor=LOCAL,
               edgecolor="white", lw=1.2, zorder=9)
    ax.annotate("local goal", local_pt, xytext=(2.6, -3.3), fontsize=7,
                color=LOCAL, fontweight="bold", zorder=9,
                arrowprops=dict(arrowstyle="-", color=LOCAL, lw=0.8))
    ax.scatter(*n_fork, s=150, marker="D", facecolor=FRONTIER,
               edgecolor="white", lw=1.2, zorder=9)
    ax.annotate("frontier goal", n_fork, xytext=(6.6, 2.6), fontsize=7,
                color=FRONTIER, fontweight="bold", zorder=9,
                arrowprops=dict(arrowstyle="-", color=FRONTIER, lw=0.8))
    ax.scatter(*n_goal, s=300, marker="*", facecolor=MISSION,
               edgecolor="white", lw=1.2, zorder=9)
    ax.annotate("mission goal", n_goal, xytext=(20.6, 2.0), fontsize=7.5,
                color=MISSION, fontweight="bold", ha="center", zorder=9)

    # --- horizon labels -----------------------------------------------------
    ax.text(-5.9, 5.9, "depth horizon", fontsize=6.5, color=DEPTH_EDGE,
            fontweight="bold", rotation=43, zorder=9)
    ax.text(-10.2, 9.4, "semantic reach", fontsize=6.5, color=SEM_EDGE,
            fontweight="bold", rotation=39, zorder=9)

    # --- legend --------------------------------------------------------------
    handles = [
        Line2D([], [], marker="s", color="none", markerfacecolor=OBSTACLE,
               markersize=9, label="obstacles"),
        Line2D([], [], marker="o", color="none", markerfacecolor=DEPTH_FILL,
               markeredgecolor=DEPTH_EDGE, markersize=10, label="depth horizon"),
        Line2D([], [], marker="o", color="none", markerfacecolor=SEM_FILL,
               markeredgecolor=SEM_EDGE, markersize=10, label="semantic reach"),
        Line2D([], [], marker="o", color=GRAPH_EDGE, markerfacecolor=NODE_FACE,
               markeredgecolor=INK, markersize=7, lw=1.1,
               label="graph node + witness edge"),
        Line2D([], [], marker="o", color="none", markerfacecolor=RISK,
               markersize=8, label="far-field semantic node"),
        Line2D([], [], color=RISK, lw=3, alpha=0.6,
               label="semantic exposure on witness"),
        Line2D([], [], color=SELECTED, lw=2.6, label="selected route"),
        Line2D([], [], color=REJECTED, lw=1.8, ls=(0, (5, 3)),
               label="rejected branch"),
        Line2D([], [], marker="*", color="none", markerfacecolor=MISSION,
               markersize=12, label="mission goal"),
        Line2D([], [], marker="D", color="none", markerfacecolor=FRONTIER,
               markersize=8, label="frontier goal"),
        Line2D([], [], marker="o", color="none", markerfacecolor=LOCAL,
               markersize=8, label="local goal"),
    ]
    ax.legend(handles=handles, loc="upper center",
              bbox_to_anchor=(0.5, -0.02), ncols=4, frameon=False,
              fontsize=6.4, handletextpad=0.5, columnspacing=1.2)

    ax.set_xlim(-16.5, 23.5)
    ax.set_ylim(-10.5, 10.5)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.13)

    fig.savefig(f"{OUTPUT}.pdf")
    fig.savefig(f"{OUTPUT}.png")
    print(f"wrote {OUTPUT}.pdf/.png")


if __name__ == "__main__":
    main()
