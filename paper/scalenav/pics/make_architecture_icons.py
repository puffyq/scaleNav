#!/usr/bin/env python3
"""Architecture schematic with icons: feature-forward companion to the
data-pipeline figure.  All drawing is vector (matplotlib patches)."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import (Circle, FancyArrowPatch, FancyBboxPatch,
                                Polygon, Rectangle)

HERE = Path(__file__).resolve().parent
OUT = HERE / "system_architecture_icons"

C = {"sense": "#526D77", "semantic": "#B44D5A", "topology": "#2D8C74",
     "gcn": "#7856D8", "astar": "#3973B7", "exec": "#D3820F",
     "ink": "#26343C", "muted": "#8A979E"}


def chip(ax, x, y, w, h, color, alpha=0.10, lw=1.4, ls="-"):
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.6,rounding_size=1.6",
                         facecolor=color, edgecolor=color, alpha=alpha,
                         linewidth=lw, linestyle=ls, zorder=1)
    ax.add_patch(box)
    rim = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.6,rounding_size=1.6",
                         facecolor="none", edgecolor=color, linewidth=lw,
                         linestyle=ls, alpha=0.85, zorder=1)
    ax.add_patch(rim)


def icon_drone(ax, x, y, s, color):
    for dx, dy in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        ax.add_patch(Circle((x + dx * s * 0.62, y + dy * s * 0.62), s * 0.30,
                            facecolor="none", edgecolor=color, lw=1.6, zorder=5))
    ax.plot([x - s * 0.62, x + s * 0.62], [y - s * 0.62, y + s * 0.62],
            color=color, lw=1.6, zorder=4)
    ax.plot([x - s * 0.62, x + s * 0.62], [y + s * 0.62, y - s * 0.62],
            color=color, lw=1.6, zorder=4)
    ax.add_patch(Circle((x, y), s * 0.26, facecolor=color, edgecolor="white",
                        lw=0.8, zorder=6))


def icon_camera(ax, x, y, s, color):
    ax.add_patch(FancyBboxPatch((x - s, y - s * 0.62), 2 * s, 1.24 * s,
                                boxstyle="round,pad=0,rounding_size=0.35",
                                facecolor="none", edgecolor=color, lw=1.6,
                                zorder=5))
    ax.add_patch(Circle((x, y), s * 0.34, facecolor="none", edgecolor=color,
                        lw=1.6, zorder=6))
    ax.add_patch(Circle((x, y), s * 0.12, facecolor=color, edgecolor="none",
                        zorder=6))


def icon_graph(ax, x, y, s, color):
    pts = np.array([[-0.8, -0.5], [0.0, -0.75], [0.8, -0.45], [-0.45, 0.35],
                    [0.45, 0.4], [0.05, 0.85]]) * s
    links = [(0, 1), (1, 2), (0, 3), (1, 4), (3, 4), (3, 5), (4, 5), (2, 4)]
    for i, j in links:
        ax.plot([pts[i, 0] + x, pts[j, 0] + x], [pts[i, 1] + y, pts[j, 1] + y],
                color=color, lw=1.3, alpha=0.75, zorder=4)
    ax.scatter(pts[:, 0] + x, pts[:, 1] + y, s=s * 26, color=color,
               edgecolors="white", linewidths=0.7, zorder=5)


def icon_heatmap(ax, x, y, s):
    grad = np.linspace(0, 1, 64).reshape(8, 8)
    grad = grad + grad.T * 0.6
    ax.imshow(grad, extent=(x - s, x + s, y - s * 0.72, y + s * 0.72),
              cmap="inferno", interpolation="bilinear", zorder=4)
    ax.add_patch(Rectangle((x - s, y - s * 0.72), 2 * s, 1.44 * s,
                           facecolor="none", edgecolor=C["semantic"], lw=1.6,
                           zorder=5))


def icon_gcn(ax, x, y, s, color):
    ring = np.array([[np.cos(a), np.sin(a)] for a in
                     np.deg2rad([90, 18, -54, -126, 162])]) * s * 0.85
    for p in ring:
        ax.plot([x, x + p[0]], [y, y + p[1]], color=color, lw=1.2, alpha=0.7,
                zorder=4)
    ax.scatter(ring[:, 0] + x, ring[:, 1] + y, s=s * 22, color=color,
               edgecolors="white", linewidths=0.6, zorder=5)
    ax.scatter([x], [y], s=s * 46, marker="*", color=color,
               edgecolors="white", linewidths=0.7, zorder=6)


def icon_shield(ax, x, y, s, color):
    pts = np.array([[-0.8, 0.75], [0.8, 0.75], [0.8, -0.05], [0.0, -0.9],
                    [-0.8, -0.05]]) * s
    ax.add_patch(Polygon(pts + [x, y], facecolor="none", edgecolor=color,
                         lw=1.8, zorder=5))
    ax.plot([x - s * 0.36, x - s * 0.08, x + s * 0.42],
            [y + s * 0.05, y - s * 0.28, y + s * 0.38], color=color, lw=2.0,
            zorder=6)


def icon_flag(ax, x, y, s, color):
    ax.plot([x, x], [y - s * 0.8, y + s * 0.8], color=color, lw=1.8, zorder=5)
    ax.add_patch(Polygon(np.array([[0, 0.8], [0.95, 0.45], [0, 0.1]]) * s
                         + [x, y], facecolor=color, edgecolor="none", zorder=5))


def icon_plug(ax, x, y, s, color):
    ax.add_patch(FancyBboxPatch((x - s * 0.55, y - s * 0.45), s * 1.1, s * 0.9,
                                boxstyle="round,pad=0,rounding_size=0.3",
                                facecolor=color, edgecolor="none", zorder=6))
    ax.plot([x - s * 0.95, x - s * 0.55], [y + s * 0.2, y + s * 0.2],
            color=color, lw=2.2, zorder=6)
    ax.plot([x - s * 0.95, x - s * 0.55], [y - s * 0.2, y - s * 0.2],
            color=color, lw=2.2, zorder=6)


def arrow(ax, p0, p1, color, lw=2.2, ls="-", label=None, label_dy=1.1,
          label_size=6.4, connectionstyle="arc3,rad=0.0"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=15,
                                 color=color, lw=lw, linestyle=ls,
                                 connectionstyle=connectionstyle, zorder=7))
    if label:
        mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2 + label_dy
        ax.text(mx, my, label, ha="center", va="bottom", fontsize=label_size,
                color=color, fontweight="bold", zorder=8)


def label(ax, x, y, text, color, size=6.6, weight="bold"):
    ax.text(x, y, text, ha="center", va="top", fontsize=size, color=color,
            fontweight=weight, zorder=8)


def main() -> None:
    plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42,
                         "ps.fonttype": 42})
    fig, ax = plt.subplots(figsize=(7.16, 2.7))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 38)
    ax.axis("off")

    # zones
    chip(ax, 1.5, 8.0, 17.5, 26.0, C["sense"], alpha=0.08)
    chip(ax, 23.5, 2.5, 52.0, 33.0, C["topology"], alpha=0.07)
    chip(ax, 79.5, 8.0, 19.0, 26.0, C["exec"], alpha=0.10, ls=(0, (4, 2)))
    ax.text(10.2, 36.6, "SENSE", ha="center", fontsize=7.2,
            color=C["sense"], fontweight="bold")
    ax.text(49.5, 37.0, "TOPOGUIDE ROUTE LAYER", ha="center", fontsize=7.2,
            color=C["topology"], fontweight="bold")
    ax.text(89.0, 36.6, "EXECUTE (unmodified)", ha="center", fontsize=7.0,
            color=C["exec"], fontweight="bold")

    # sense zone
    icon_drone(ax, 10.2, 28.0, 2.6, C["sense"])
    icon_camera(ax, 10.2, 20.5, 1.9, C["sense"])
    ax.text(10.2, 16.4, "RGB-D + odometry", ha="center", va="center",
            fontsize=6.0, color=C["ink"], zorder=8)
    ax.text(10.2, 12.4, 'query $q$: "blocks, wall"', ha="center", va="center",
            fontsize=6.0, color=C["semantic"], style="italic", zorder=8)

    # route layer modules
    icon_graph(ax, 34.0, 26.0, 2.7, C["topology"])
    label(ax, 34.0, 22.6, "verified topology", C["topology"], size=5.9)
    icon_heatmap(ax, 34.0, 11.4, 2.5)
    label(ax, 34.0, 8.2, "soft semantic cost", C["semantic"], size=5.9)
    icon_gcn(ax, 51.0, 26.0, 2.5, C["gcn"])
    label(ax, 51.0, 23.0, "FrontierGCN\nranks", C["gcn"], size=5.9)
    icon_shield(ax, 63.5, 18.0, 2.6, C["astar"])
    label(ax, 63.5, 14.2, "A* + collision\nchecks authorize", C["astar"],
          size=5.9)
    icon_flag(ax, 71.8, 18.6, 2.3, C["exec"])

    # route-internal arrows
    arrow(ax, (37.4, 26.0), (48.2, 26.0), C["topology"], lw=1.8)
    arrow(ax, (54.0, 24.6), (61.4, 19.6), C["gcn"], lw=1.8, ls=(0, (4, 2)),
          label="proposal", label_dy=2.0, label_size=5.6)
    arrow(ax, (36.9, 23.8), (61.1, 17.4), C["topology"], lw=1.8,
          connectionstyle="arc3,rad=0.10")
    arrow(ax, (36.9, 11.8), (61.1, 16.6), C["semantic"], lw=1.8,
          ls=(0, (4, 2)), label="soft cost", label_dy=0.7, label_size=5.6,
          connectionstyle="arc3,rad=-0.06")
    arrow(ax, (66.4, 18.2), (69.9, 18.4), C["astar"], lw=1.8)

    # sense -> route
    arrow(ax, (19.4, 21.5), (31.0, 25.0), C["sense"], lw=2.2)
    ax.text(24.5, 25.4, "depth + odom", ha="center", fontsize=5.6,
            color=C["sense"], fontweight="bold", zorder=8)
    arrow(ax, (19.4, 12.4), (31.0, 11.5), C["semantic"], lw=2.2,
          label="RGB + query", label_dy=0.6, label_size=5.6)

    # boundary: only the local goal crosses
    ax.plot([77.5, 77.5], [6.0, 32.0], color=C["ink"], lw=1.2,
            ls=(0, (2, 2)), alpha=0.7, zorder=3)
    icon_plug(ax, 77.5, 30.0, 1.5, C["ink"])
    ax.text(77.5, 4.2, "plug-and-play boundary", ha="center", fontsize=5.2,
            color=C["ink"], style="italic", zorder=8)
    arrow(ax, (73.6, 18.8), (81.6, 20.4), C["exec"], lw=3.0,
          connectionstyle="arc3,rad=-0.10")
    ax.text(76.6, 22.4, "moving local goal", ha="center", fontsize=6.2,
            color=C["exec"], fontweight="bold", zorder=8)

    # exec zone
    icon_drone(ax, 89.0, 25.5, 3.0, C["exec"])
    label(ax, 89.0, 21.4, "goal-conditioned\nlocal planner", C["exec"],
          size=6.2)
    t = np.linspace(0, 1, 60)
    ax.plot(85.0 + 8.5 * t, 13.8 + 1.1 * np.sin(t * 5.2) + 0.9 * t,
            color=C["exec"], lw=1.7, zorder=4)
    ax.text(89.0, 10.6, "YOPO / EGO / SUPER", ha="center", fontsize=6.0,
            color=C["ink"], zorder=8)

    fig.savefig(OUT.with_suffix(".pdf"), dpi=400, bbox_inches="tight",
                pad_inches=0.03)
    fig.savefig(OUT.with_suffix(".png"), dpi=400, bbox_inches="tight",
                pad_inches=0.03)
    plt.close(fig)
    print(f"wrote {OUT}.pdf/.png")


if __name__ == "__main__":
    main()
