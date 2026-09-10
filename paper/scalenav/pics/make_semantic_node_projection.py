#!/usr/bin/env python3
"""Sec. 3-B: logged graph readout and virtual semantic frontiers."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Ellipse, FancyArrowPatch, Polygon
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SESSION = REPO / "log_scalenav" / "session_20260906_223557_314"
# This synchronized frame shows the block faces on the left and a
# lower-response open route on the right.
RGB_FILE = "rgb/rgb_81.ppm"
SEMANTIC_FILE = "semantic/semantic_90.pgm"
DEPTH_FILE = "depth/depth_89.pgm"
GRAPH_FILE = "graph/graph_48.json"
POINTCLOUD_FILE = "pointcloud/pointcloud_161.pcd"

INK = "#24343D"
OBSTACLE = "#B7C0C4"
PEARL = LinearSegmentedColormap.from_list(
    "pearl", ["#160B39", "#482173", "#8B2981", "#D94E63", "#F6D746"])
CAM_T = np.array([0.5, 0.0, -0.1], dtype=float)
FOV_H = 90.0
FOV_V = 60.0
MIN_RADIUS_M = 1.0
MAX_RADIUS_M = 20.0
MIN_SCORE = 0.20
DEPTH_CLIP_M = 20.0
SCORE_VMIN, SCORE_VMAX = 0.12, 0.50


def read_events() -> list[dict]:
    events = []
    for line in (SESSION / "index.jsonl").open(encoding="utf-8"):
        try:
            events.append(json.loads(line, parse_constant=lambda value: float(value)))
        except json.JSONDecodeError:
            continue
    return events


def nearest(events: list[dict], kind: str, stamp: int) -> dict:
    return min((e for e in events if e.get("kind") == kind),
               key=lambda e: abs(int(e["stamp_ns"]) - stamp))


def marker(graph: dict, name: str) -> dict:
    return next(m for m in graph["markers"] if m.get("ns") == name)


def marker_points(graph: dict, name: str) -> np.ndarray:
    values = marker(graph, name).get("points", [])
    return np.asarray(values, dtype=float).reshape(-1, 3) if values else np.empty((0, 3))


def quat_rotate(q: list[float], vectors: np.ndarray) -> np.ndarray:
    xyz = np.asarray(q[:3], dtype=float)
    scalar = float(q[3])
    cross = 2.0 * np.cross(xyz, vectors)
    return vectors + scalar * cross + np.cross(xyz, cross)


def world_to_cam(points: np.ndarray, body: np.ndarray, orientation: list[float]) -> np.ndarray:
    body_flu = quat_rotate([-orientation[0], -orientation[1], -orientation[2], orientation[3]],
                           points - body[None, :])
    return body_flu - CAM_T[None, :]


def project_cam(cam: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    depth = cam[:, 0]
    ht = np.tan(np.deg2rad(FOV_H) * 0.5)
    vt = np.tan(np.deg2rad(FOV_V) * 0.5)
    u = 0.5 - 0.5 * cam[:, 1] / np.maximum(depth * ht, 1e-6)
    v = 0.5 - 0.5 * cam[:, 2] / np.maximum(depth * vt, 1e-6)
    return u, v, depth


def read_pcd(path: Path) -> np.ndarray:
    points = []
    data = False
    with path.open(encoding="ascii", errors="ignore") as stream:
        for line in stream:
            if line.lower().startswith("data"):
                data = True
                continue
            if not data:
                continue
            fields = line.split()
            if len(fields) < 3:
                continue
            try:
                points.append(tuple(map(float, fields[:3])))
            except ValueError:
                continue
    return np.asarray(points, dtype=float)


def heatmap_at(heat: np.ndarray, u: int, v: int) -> float:
    if u < 0 or v < 0 or u >= heat.shape[1] or v >= heat.shape[0]:
        return 0.0
    return float(heat[v, u])


def kernel_sigma(heat: np.ndarray, u: float, v: float,
                 depth: float) -> tuple[float, float]:
    """Match TopoSemanticProjection: support grows with the image component."""
    height, width = heat.shape
    ht = np.tan(np.deg2rad(FOV_H) * 0.5)
    vt = np.tan(np.deg2rad(FOV_V) * 0.5)
    cu = int(np.clip(u * width, 0, width - 1))
    cv = int(np.clip(v * height, 0, height - 1))
    max_probe = max(2, min(width, height) // 3)
    extent_u = extent_v = 0
    for du, dv in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        extent = 0
        for step in range(1, max_probe + 1):
            pu, pv = cu + du * step, cv + dv * step
            if heatmap_at(heat, pu, pv) < MIN_SCORE:
                break
            extent += 1
        if du:
            extent_u = max(extent_u, extent)
        if dv:
            extent_v = max(extent_v, extent)
    radius_m = max(
        MIN_RADIUS_M,
        depth * ht * (extent_u + 1) / width,
        depth * vt * (extent_v + 1) / height,
    )
    radius_m = float(np.clip(radius_m, MIN_RADIUS_M, MAX_RADIUS_M))
    sigma_u = float(np.clip((0.5 * width) / ht * radius_m / depth, 1.25, 32.0))
    sigma_v = float(np.clip((0.5 * height) / vt * radius_m / depth, 1.25, 32.0))
    return sigma_u, sigma_v


def gaussian_score(heat: np.ndarray, u: float, v: float,
                   depth: float) -> tuple[float, float, float]:
    height, width = heat.shape
    sigma_u, sigma_v = kernel_sigma(heat, u, v, depth)
    center_u = u * width - 0.5
    center_v = v * height - 0.5
    ru, rv = int(np.ceil(3.0 * sigma_u)), int(np.ceil(3.0 * sigma_v))
    u0, u1 = max(0, int(center_u) - ru), min(width - 1, int(np.ceil(center_u)) + ru)
    v0, v1 = max(0, int(center_v) - rv), min(height - 1, int(np.ceil(center_v)) + rv)
    uu = np.arange(u0, u1 + 1)
    vv = np.arange(v0, v1 + 1)
    du = (uu[None, :] - center_u) / sigma_u
    dv = (vv[:, None] - center_v) / sigma_v
    weight = np.exp(-0.5 * (du * du + dv * dv))
    patch = heat[v0:v1 + 1, u0:u1 + 1]
    signal = patch >= MIN_SCORE
    if float(np.sum(weight * signal)) > 1e-4:
        score = float(np.sum(weight * patch * signal) / np.sum(weight * signal))
    else:
        mass = float(np.sum(weight))
        score = 0.0 if mass <= 1e-6 else float(np.sum(weight * patch) / mass)
    return float(np.clip(score, 0.0, 1.0)), sigma_u, sigma_v


def heatmap_panel(rgb: np.ndarray, heat: np.ndarray) -> np.ndarray:
    lo, hi = np.percentile(heat, (8, 98))
    shown = np.clip((heat - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    return np.clip(0.22 * rgb + 0.78 * PEARL(shown)[..., :3], 0.0, 1.0)


def middle_row_candidates(
    heat: np.ndarray, body: np.ndarray, orientation: list[float],
    distance_m: float = 35.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    height, width = heat.shape
    row = 1
    image_uv = []
    scores = []
    points_world = []
    tangent_u = np.tan(np.deg2rad(FOV_H) * 0.5)
    tangent_v = np.tan(np.deg2rad(FOV_V) * 0.5)
    for column in range(5):
        u0 = column * width // 5
        u1 = (column + 1) * width // 5
        v0 = row * height // 3
        v1 = (row + 1) * height // 3
        center_u = min(width - 1, u0 + max(1, u1 - u0) // 2)
        center_v = min(height - 1, v0 + max(1, v1 - v0) // 2)
        normalized_u = (center_u + 0.5) / width
        normalized_v = (center_v + 0.5) / height
        ray = np.array([
            1.0,
            -(2.0 * normalized_u - 1.0) * tangent_u,
            -(2.0 * normalized_v - 1.0) * tangent_v,
        ])
        ray /= max(np.linalg.norm(ray), 1e-6)
        camera_point = CAM_T + distance_m * ray
        world_point = body + quat_rotate(orientation, camera_point[None, :])[0]
        world_point[2] = body[2]
        points_world.append(world_point)
        image_uv.append((center_u + 0.5, center_v + 0.5))
        scores.append(float(np.mean(heat[v0:v1, u0:u1])))
    return (
        np.asarray(points_world, dtype=float),
        np.asarray(image_uv, dtype=float),
        np.asarray(scores, dtype=float),
        np.arange(5, dtype=int),
    )


def main() -> None:
    events = read_events()
    rgb_event = next(e for e in events if e.get("kind") == "rgb" and e.get("file") == RGB_FILE)
    stamp = int(rgb_event["stamp_ns"])
    odom = nearest(events, "odom", stamp)
    body = np.asarray(odom["data"]["position"], dtype=float)
    orientation = odom["data"]["orientation"]
    graph = json.loads((SESSION / GRAPH_FILE).read_text(encoding="utf-8"))
    nodes = marker_points(graph, "scalenav_skeleton_nodes")
    edges = marker_points(graph, "scalenav_skeleton_edges").reshape(-1, 2, 3)
    semantic_links = marker_points(graph, "scalenav_semantic_links").reshape(-1, 2, 3)
    rgb = np.asarray(Image.open(SESSION / RGB_FILE).convert("RGB"), dtype=float) / 255.0
    heat = np.clip(np.asarray(Image.open(SESSION / SEMANTIC_FILE), dtype=float) / 1000.0, 0.0, 1.0)
    # The topology is a ground-plane map in front of the camera. Keep XY and
    # drop z to the ground, otherwise a level camera projects every flight-
    # layer node onto the horizon line.
    nodes_ground = nodes.copy()
    nodes_ground[:, 2] = 0.0
    edges_ground = edges.copy()
    edges_ground[:, :, 2] = 0.0
    cam = world_to_cam(nodes_ground, body, orientation)
    u, v, depth = project_cam(cam)
    in_fov = (depth > 0.4) & (u >= 0.0) & (u <= 1.0) & (v >= 0.0) & (v <= 1.0)
    scores = np.full(len(nodes), np.nan)
    sigma = np.full((len(nodes), 2), np.nan)
    for index in np.flatnonzero(in_fov):
        scores[index], sigma[index, 0], sigma[index, 1] = gaussian_score(
            heat, float(u[index]), float(v[index]), float(depth[index]))

    # Use the point cloud synchronized with the RGB/heatmap frame. Convert the
    # base_link cloud through the same pose as the graph and remove ground
    # points in world coordinates before displaying the camera-frame source.
    cloud = read_pcd(SESSION / POINTCLOUD_FILE)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    cloud_world = body[None, :] + quat_rotate(orientation, cloud)
    cloud = world_to_cam(cloud_world, body, orientation)
    cloud = cloud[(cloud_world[:, 2] >= 0.6) & (cloud_world[:, 2] <= 2.6)]
    cloud = cloud[(cloud[:, 0] > 0.8) & (cloud[:, 0] < 22.0) &
                  (np.abs(cloud[:, 1]) < 13.0)]

    plt.rcParams.update({
        "font.family": "serif", "font.size": 7, "pdf.fonttype": 42, "ps.fonttype": 42,
        "figure.facecolor": "white", "savefig.facecolor": "white",
    })
    fig = plt.figure(figsize=(3.60, 1.62))
    fig.subplots_adjust(left=0.010, right=0.905, top=0.985, bottom=0.025,
                        wspace=0.065)
    grid = GridSpec(1, 2, figure=fig, width_ratios=(1.30, 1.0),
                    wspace=0.065)
    ax_w = fig.add_subplot(grid[0, 0], projection="3d")
    ax_h = fig.add_subplot(grid[0, 1])
    height, width = heat.shape
    candidate_world, candidate_uv, candidate_scores, candidate_columns = (
        middle_row_candidates(heat, body, orientation)
    )
    ax_h.imshow(heatmap_panel(rgb, heat), origin="upper", interpolation="nearest")
    ax_h.set_xlim(0, width)
    ax_h.set_ylim(height, 0)

    vis = in_fov & np.isfinite(scores)
    vis_idx = np.flatnonzero(vis)
    ax_h.scatter(u[vis] * width, v[vis] * height, s=8, c=scores[vis], cmap=PEARL,
                 vmin=SCORE_VMIN, vmax=SCORE_VMAX, edgecolors="white",
                 linewidths=0.22, zorder=4)
    ax_h.scatter(candidate_uv[:, 0], candidate_uv[:, 1], s=42,
                 c=candidate_scores, cmap=PEARL, vmin=SCORE_VMIN,
                 vmax=SCORE_VMAX, marker="*", edgecolors=INK,
                 linewidths=0.45, zorder=8)
    for column, (pixel_u, pixel_v) in zip(candidate_columns, candidate_uv):
        ax_h.text(pixel_u + 3, pixel_v - 4, f"C{column}", fontsize=4.0,
                  color=INK, fontweight="bold", zorder=9,
                  bbox=dict(facecolor="white", edgecolor="none", pad=0.45,
                            alpha=0.72))

    example = {}
    if len(vis_idx):
        left = vis_idx[u[vis_idx] < 0.34]
        mid = vis_idx[(u[vis_idx] > 0.40) & (u[vis_idx] < 0.62)]
        if len(left):
            example["building"] = left[np.nanargmax(scores[left])]
    if len(mid):
        example["road"] = mid[np.nanargmin(np.where(scores[mid] > 0, scores[mid], 1.0))]
    correspondences = []
    if "building" in example:
        correspondences.append(("1", example["building"], "#F6D746"))
    if "road" in example and example["road"] != example.get("building"):
        correspondences.append(("2", example["road"], "#72D6C2"))
    depth_examples = []
    if len(vis_idx) >= 2:
        near_index = vis_idx[np.argmin(depth[vis_idx])]
        far_index = vis_idx[np.argmax(depth[vis_idx])]
        depth_examples = [("N", near_index, "#E28A17"),
                          ("F", far_index, "#4C78A8")]
    for key, color, lw in (("building", "white", 0.9), ("road", "#E7EEF2", 0.6)):
        if key not in example:
            continue
        index = example[key]
        su, sv = sigma[index]
        ax_h.add_patch(Ellipse(
            (u[index] * width, v[index] * height), 4 * su, 4 * sv,
            fill=False, edgecolor=color, lw=lw, ls=(0, (2.2, 1.1)), zorder=5))
        ax_h.scatter(u[index] * width, v[index] * height, s=20, facecolors="none",
                     edgecolors=color, linewidths=0.7, zorder=6)
    for label, index, color in correspondences:
        ax_h.text(u[index] * width + 7, v[index] * height - 5, label,
                  color=color, fontsize=5.1, fontweight="bold", zorder=7,
                  bbox=dict(facecolor=INK, edgecolor="none", pad=0.7, alpha=0.8))
    for label, index, color in depth_examples:
        ax_h.scatter(u[index] * width, v[index] * height, s=34,
                     facecolors="none", edgecolors=color, linewidths=0.8,
                     zorder=7)
        ax_h.text(u[index] * width + 5, v[index] * height + 6,
                  f"{label} {depth[index]:.1f} m", color=color, fontsize=4.2,
                  fontweight="bold", zorder=8,
                  bbox=dict(facecolor="white", edgecolor="none", pad=0.5,
                            alpha=0.78))

    ax_h.text(0.03, 0.96, "3D node $\\rightarrow$ pixel", transform=ax_h.transAxes,
              fontsize=4.0, color="white", fontweight="bold", va="top",
              bbox=dict(facecolor=INK, edgecolor="none", alpha=0.65, pad=1.0))
    ax_h.text(0.02, 0.08, r"(b)", transform=ax_h.transAxes,
              fontsize=6.2, color=INK, fontweight="bold", zorder=10,
              bbox=dict(facecolor="white", edgecolor="none", pad=0.45,
                        alpha=0.72))
    ax_h.set_xticks([])
    ax_h.set_yticks([])
    for spine in ax_h.spines.values():
        spine.set_color("#A5B0B4")
        spine.set_linewidth(0.45)

    # Panel (a) is the source representation: the same 3-D graph nodes and
    # synchronized obstacle points, shown in the camera FLU frame. Panel (b)
    # receives these nodes through the perspective projection above.
    if len(cloud):
        ax_w.scatter(-cloud[:, 1], cloud[:, 0], cloud[:, 2], s=0.42,
                     c="#B7C0C4", alpha=0.30, depthshade=False, linewidths=0)
    node_x = -cam[:, 1]
    node_y = cam[:, 0]
    node_z = cam[:, 2]
    unscored = ~np.isfinite(scores)
    if np.any(unscored):
        ax_w.scatter(node_x[unscored], node_y[unscored], node_z[unscored],
                     s=10.5, c="#8A9AA0", edgecolors="white", linewidths=0.2,
                     depthshade=False)
    if np.any(~unscored):
        ax_w.scatter(node_x[~unscored], node_y[~unscored], node_z[~unscored],
                     s=10.5, c=scores[~unscored], cmap=PEARL,
                     vmin=SCORE_VMIN, vmax=SCORE_VMAX,
                     edgecolors="white", linewidths=0.2, depthshade=False)
    for edge in edges_ground:
        edge_cam = world_to_cam(edge, body, orientation)
        ax_w.plot(-edge_cam[:, 1], edge_cam[:, 0], edge_cam[:, 2],
                  color="#89979D", lw=0.42, alpha=0.55)
    for link in semantic_links:
        link_cam = world_to_cam(link, body, orientation)
        ax_w.plot(-link_cam[:, 1], link_cam[:, 0], link_cam[:, 2],
                  color="#D68B2C", lw=0.85, alpha=0.72, ls=(0, (2, 1)))
    candidate_cam = world_to_cam(candidate_world, body, orientation)
    ax_w.scatter(-candidate_cam[:, 1], candidate_cam[:, 0], candidate_cam[:, 2],
                 s=72, c=candidate_scores, cmap=PEARL, vmin=SCORE_VMIN,
                 vmax=SCORE_VMAX, marker="*", edgecolors=INK,
                 linewidths=0.45, depthshade=False, zorder=8)
    for column, point in zip(candidate_columns, candidate_cam):
        ax_w.text(-point[1] + 0.5, point[0], point[2] + 0.35, f"C{column}",
                  color=INK, fontsize=4.2, fontweight="bold")
    ax_w.scatter([0], [0], [0], marker="^", s=22, c=INK, depthshade=False)
    for label, index, color in correspondences:
        ax_w.scatter([node_x[index]], [node_y[index]], [node_z[index]], s=38,
                     facecolors="none", edgecolors=color, linewidths=1.0,
                     depthshade=False)
        ax_w.text(node_x[index] + 0.45, node_y[index], node_z[index] + 0.25,
                  label, color=color, fontsize=5.2, fontweight="bold")
    for label, index, color in depth_examples:
        ax_w.scatter([node_x[index]], [node_y[index]], [node_z[index]], s=34,
                     facecolors="none", edgecolors=color, linewidths=0.9,
                     depthshade=False)
        ax_w.text(node_x[index] + 0.45, node_y[index], node_z[index] - 0.3,
                  f"{label} {depth[index]:.1f} m", color=color, fontsize=4.1,
                  fontweight="bold")
    ax_w.set_xlim(-22.0, 21.0)
    ax_w.set_ylim(-1.0, 36.0)
    ax_w.set_zlim(-3.5, 8.6)
    ax_w.set_box_aspect((43, 37, 15))
    ax_w.set_proj_type("ortho")
    ax_w.view_init(elev=24, azim=-90)
    ax_w.set_axis_off()
    ax_w.text2D(
        0.060, 0.820, r"(a)", transform=ax_w.transAxes,
        fontsize=6.2, color=INK, fontweight="bold", ha="left", va="top",
        bbox=dict(facecolor="white", edgecolor="none", pad=0.45, alpha=0.72),
    )

    cax = fig.add_axes([0.925, 0.18, 0.020, 0.62])
    fig.colorbar(ScalarMappable(norm=Normalize(SCORE_VMIN, SCORE_VMAX), cmap=PEARL), cax=cax)
    cax.tick_params(labelsize=3.6, length=1.4, pad=0.6)
    cax.set_ylabel(r"$s_i$", fontsize=4.4, color=INK, labelpad=0.6)
    for spine in cax.spines.values():
        spine.set_linewidth(0.4)

    fig.canvas.draw()
    box_h, box_w = ax_h.get_position(), ax_w.get_position()
    y = 0.5 * (box_h.y0 + box_h.y1)
    fig.add_artist(FancyArrowPatch(
        (box_w.x1 + 0.006, y), (box_h.x0 - 0.010, y),
        transform=fig.transFigure, arrowstyle="-|>", mutation_scale=6,
        color=INK, lw=0.7, clip_on=False,
    ))
    fig.text(0.5 * (box_w.x1 + box_h.x0), y + 0.020,
             "project $(u_i,v_i)$\n+ Gaussian support",
             ha="center", va="bottom", fontsize=3.4, color=INK,
             linespacing=0.9)

    out = HERE / "semantic_node_projection"
    fig.savefig(out.with_suffix(".pdf"), dpi=400, bbox_inches="tight", pad_inches=0.01)
    fig.savefig(out.with_suffix(".png"), dpi=400, bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)
    scored = scores[np.isfinite(scores)]
    sig = sigma[np.isfinite(sigma[:, 0])]
    print(f"wrote {out}.pdf and .png; vis={int(vis.sum())}/{len(nodes)}; "
          f"score {scored.min():.3f}–{scored.max():.3f}; "
          f"sigma_u {sig[:,0].min():.1f}–{sig[:,0].max():.1f}px")


if __name__ == "__main__":
    main()
