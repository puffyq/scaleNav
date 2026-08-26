#!/usr/bin/env python3
"""Build a YOPO-style 2-D-to-3-D semantic projection from one real frame.

The figure uses one synchronized Map2 state. The image plane, 5x3 maxima,
fixed-Z rays, graph nodes, collision-checked witnesses, selected witness, and
goal hierarchy all come from that state. The layout follows the visual grammar
of YOPO's image-grid/trajectory figure without turning the data into a diagram.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.lines import Line2D
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
sys.path.insert(0, str(SCRIPT_DIR))

from make_teaser import (  # noqa: E402
    BACKGROUND,
    DEPTH_CLIP_M,
    FRONTIER,
    LOCAL_GOAL,
    MISSION,
    RISK_HIGH,
    SELECTED,
    TOPOLOGY,
    UAV,
    load_events,
    load_graph,
    load_ply_xyz,
    marker_points,
    marker_points_with_colors,
    marker_pose_position,
    nearest_event,
    quat_rotate,
    read_scalar_image,
)


DEFAULT_SESSION = REPO_ROOT / "log_scalenav/session_20260826_192224_661"
DEFAULT_RGB = "rgb/rgb_229.ppm"
DEFAULT_OUTPUT = SCRIPT_DIR / "semantic_risk_field_paper"

CAMERA_TRANSLATION_FLU = np.asarray([0.5, 0.0, -0.1])
HORIZONTAL_FOV_DEG = 90.0
VERTICAL_FOV_DEG = 60.0
VIRTUAL_OPTICAL_Z_M = 30.0
PATCH_COLS = 5
PATCH_ROWS = 3
ACTIVE_SCORE = 0.35
RISK_RADIUS_M = 10.0

GRID_BLUE = "#48A4D2"
ANCHOR = "#E6A33A"
RISK_LOW = "#58CBB5"
RISK_MID = "#E1B542"
INK = "#24343D"
LABEL_BOX = dict(
    boxstyle="square,pad=0.15", facecolor=BACKGROUND,
    edgecolor="none", alpha=0.92,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--rgb", default=DEFAULT_RGB,
                        help="Exact RGB file entry in index.jsonl")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def exact_event(events: dict[str, list[dict]], kind: str, file_name: str) -> dict:
    for event in events[kind]:
        if event.get("file") == file_name:
            return event
    raise ValueError(f"{kind} event not found: {file_name}")


def max_pool_and_project(
    semantic: np.ndarray,
    position: np.ndarray,
    orientation: list[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Reproduce the deployed pooling, calibration, and fixed-Z projection."""
    height, width = semantic.shape
    pixels: list[tuple[int, int]] = []
    raw_scores: list[float] = []
    for patch_v in range(PATCH_ROWS):
        for patch_u in range(PATCH_COLS):
            v0 = patch_v * height // PATCH_ROWS
            v1 = (patch_v + 1) * height // PATCH_ROWS
            u0 = patch_u * width // PATCH_COLS
            u1 = (patch_u + 1) * width // PATCH_COLS
            patch = semantic[v0:v1, u0:u1]
            local_v, local_u = np.unravel_index(np.argmax(patch), patch.shape)
            pixel_u, pixel_v = u0 + int(local_u), v0 + int(local_v)
            pixels.append((pixel_u, pixel_v))
            raw_scores.append(float(patch[local_v, local_u]))

    raw = np.asarray(raw_scores)
    quantile_index = int(math.floor(0.25 * (len(raw) - 1)))
    baseline = float(np.sort(raw)[quantile_index])
    background = min(0.25, max(0.0, baseline))
    calibrated = np.clip(
        (raw - background) / max(1e-3, 1.0 - background), 0.0, 1.0
    )

    horizontal_tangent = math.tan(math.radians(HORIZONTAL_FOV_DEG / 2.0))
    vertical_tangent = math.tan(math.radians(VERTICAL_FOV_DEG / 2.0))
    points_world = []
    for pixel_u, pixel_v in pixels:
        normalized_u = (pixel_u + 0.5) / width
        normalized_v = (pixel_v + 0.5) / height
        optical_ray_flu = np.asarray([
            1.0,
            -(2.0 * normalized_u - 1.0) * horizontal_tangent,
            -(2.0 * normalized_v - 1.0) * vertical_tangent,
        ])
        body_point = CAMERA_TRANSLATION_FLU + VIRTUAL_OPTICAL_Z_M * optical_ray_flu
        points_world.append(position + quat_rotate(orientation, body_point[None, :])[0])
    return np.asarray(pixels), raw, calibrated, np.asarray(points_world)


def world_to_body(
    points: np.ndarray,
    position: np.ndarray,
    orientation: list[float],
) -> np.ndarray:
    """Express world points in the synchronized vehicle FLU frame."""
    quaternion_inverse = [
        -orientation[0], -orientation[1], -orientation[2], orientation[3]
    ]
    return quat_rotate(quaternion_inverse, np.asarray(points) - position)


class PerspectiveProjector:
    """Small deterministic camera used to compose the paper visualization."""

    def __init__(self) -> None:
        self.camera = np.asarray([-18.0, 35.0, 18.0])
        target = np.asarray([15.0, 0.0, 2.5])
        forward = target - self.camera
        self.forward = forward / np.linalg.norm(forward)
        right = np.cross(self.forward, np.asarray([0.0, 0.0, 1.0]))
        self.right = right / np.linalg.norm(right)
        self.up = np.cross(self.right, self.forward)
        self.focal = 1.55

    def __call__(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        relative = points - self.camera
        depth = relative @ self.forward
        safe_depth = np.maximum(depth, 1e-3)
        # Match YOPO's composition: vehicle at lower left, image plane behind/right.
        horizontal = -self.focal * (relative @ self.right) / safe_depth
        vertical = self.focal * (relative @ self.up) / safe_depth
        return horizontal, vertical, depth


def build_plane_texture(
    rgb: np.ndarray,
    semantic: np.ndarray,
    depth: np.ndarray,
) -> np.ndarray:
    """Overlay the semantic heatmap directly on the synchronized RGB frame."""
    rgb_float = np.asarray(rgb, dtype=float) / 255.0

    risk_map = LinearSegmentedColormap.from_list(
        "image_semantic_risk", [RISK_MID, "#E4773E", RISK_HIGH]
    )
    risk_rgb = risk_map(np.clip(semantic / 0.85, 0.0, 1.0))[..., :3]
    risk_alpha = np.clip((semantic - 0.12) / 0.62, 0.0, 0.68)[..., None]
    texture = rgb_float * (1.0 - risk_alpha) + risk_rgb * risk_alpha
    return np.clip(texture, 0.0, 1.0)


def plane_points_from_pixels(
    pixel_u: np.ndarray,
    pixel_v: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    normalized_u = np.asarray(pixel_u, dtype=float) / width
    normalized_v = np.asarray(pixel_v, dtype=float) / height
    horizontal_tangent = math.tan(math.radians(HORIZONTAL_FOV_DEG / 2.0))
    vertical_tangent = math.tan(math.radians(VERTICAL_FOV_DEG / 2.0))
    return np.column_stack((
        np.full(np.size(normalized_u), CAMERA_TRANSLATION_FLU[0] + VIRTUAL_OPTICAL_Z_M),
        CAMERA_TRANSLATION_FLU[1]
        - (2.0 * normalized_u.ravel() - 1.0) * horizontal_tangent * VIRTUAL_OPTICAL_Z_M,
        CAMERA_TRANSLATION_FLU[2]
        - (2.0 * normalized_v.ravel() - 1.0) * vertical_tangent * VIRTUAL_OPTICAL_Z_M,
    ))


def draw_image_plane(
    ax: plt.Axes,
    projector: PerspectiveProjector,
    texture: np.ndarray,
) -> None:
    """Rasterize the real image as a perspective quadrilateral with a 5x3 grid."""
    height, width = texture.shape[:2]
    render_width = 80
    render_height = max(1, round(height * render_width / width))
    texture_small = np.asarray(
        Image.fromarray(np.uint8(texture * 255)).resize(
            (render_width, render_height), Image.Resampling.BILINEAR
        ),
        dtype=float,
    ) / 255.0

    u_edges = np.linspace(0.0, width, render_width + 1)
    v_edges = np.linspace(0.0, height, render_height + 1)
    grid_u, grid_v = np.meshgrid(u_edges, v_edges)
    plane = plane_points_from_pixels(grid_u, grid_v, width, height)
    screen_u, screen_v, _ = projector(plane)
    screen_u = screen_u.reshape(render_height + 1, render_width + 1)
    screen_v = screen_v.reshape(render_height + 1, render_width + 1)

    polygons = []
    for row in range(render_height):
        for column in range(render_width):
            polygons.append([
                (screen_u[row, column], screen_v[row, column]),
                (screen_u[row, column + 1], screen_v[row, column + 1]),
                (screen_u[row + 1, column + 1], screen_v[row + 1, column + 1]),
                (screen_u[row + 1, column], screen_v[row + 1, column]),
            ])
    ax.add_collection(PolyCollection(
        polygons,
        facecolors=texture_small.reshape(-1, 3),
        edgecolors="none",
        alpha=0.84,
        rasterized=True,
        zorder=1,
    ))

    for column in range(PATCH_COLS + 1):
        u = np.full(80, column * width / PATCH_COLS)
        v = np.linspace(0.0, height, 80)
        x, y, _ = projector(plane_points_from_pixels(u, v, width, height))
        ax.plot(x, y, color=GRID_BLUE, lw=1.35, alpha=0.96, zorder=2)
    for row in range(PATCH_ROWS + 1):
        u = np.linspace(0.0, width, 120)
        v = np.full(120, row * height / PATCH_ROWS)
        x, y, _ = projector(plane_points_from_pixels(u, v, width, height))
        ax.plot(x, y, color=GRID_BLUE, lw=1.35, alpha=0.96, zorder=2)


def segment_point_distance(points: np.ndarray, segments: np.ndarray) -> np.ndarray:
    """Distance from each segment to each supplied point."""
    starts = segments[:, 0, None, :]
    vectors = segments[:, 1, None, :] - starts
    point_vectors = points[None, :, :] - starts
    denominator = np.maximum(np.sum(vectors * vectors, axis=2), 1e-9)
    factors = np.clip(np.sum(point_vectors * vectors, axis=2) / denominator, 0.0, 1.0)
    closest = starts + factors[..., None] * vectors
    return np.linalg.norm(points[None, :, :] - closest, axis=2)


def segment_semantic_exposure(
    segments: np.ndarray,
    semantic_points: np.ndarray,
    semantic_scores: np.ndarray,
) -> np.ndarray:
    if not len(segments) or not len(semantic_points):
        return np.zeros(len(segments))
    distances = segment_point_distance(semantic_points, segments)
    influence = semantic_scores[None, :] * np.exp(
        -(distances ** 2) / (2.0 * RISK_RADIUS_M ** 2)
    )
    return np.max(influence, axis=1)


def distance_to_polyline(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    segments = np.stack((polyline[:-1], polyline[1:]), axis=1)
    distances = segment_point_distance(points, segments)
    return np.min(distances, axis=0)


def projected_line_segments(
    projector: PerspectiveProjector,
    segments: np.ndarray,
) -> list[np.ndarray]:
    result = []
    for segment in segments:
        x, y, _ = projector(segment)
        result.append(np.column_stack((x, y)))
    return result


def annotate_goal(
    ax: plt.Axes,
    projector: PerspectiveProjector,
    point: np.ndarray,
    label: str,
    color: str,
    marker: str,
    offset: tuple[float, float],
) -> None:
    x, y, _ = projector(point[None, :])
    ax.scatter(x, y, marker=marker, s=78, color=color,
               edgecolor="white", linewidth=1.0, zorder=14)
    ax.annotate(
        label, xy=(x[0], y[0]), xytext=offset, textcoords="offset points",
        fontsize=7.8, color=color, fontweight="bold", bbox=LABEL_BOX, zorder=15,
    )


def draw_coordinate_axes(
    ax: plt.Axes,
    projector: PerspectiveProjector,
    origin: np.ndarray,
) -> None:
    axes = (
        (np.asarray([3.4, 0.0, 0.0]), r"$x$", INK),
        (np.asarray([0.0, 3.4, 0.0]), r"$y$", TOPOLOGY),
        (np.asarray([0.0, 0.0, 3.4]), r"$z$", INK),
    )
    origin_x, origin_y, _ = projector(origin[None, :])
    for vector, label, color in axes:
        end_x, end_y, _ = projector((origin + vector)[None, :])
        ax.annotate(
            "", xy=(end_x[0], end_y[0]), xytext=(origin_x[0], origin_y[0]),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=1.0,
                            shrinkA=0, shrinkB=0), zorder=15,
        )
        ax.text(end_x[0], end_y[0], label, fontsize=8.0, color=color,
                ha="center", va="bottom", zorder=16)


def draw_scene(
    figure: plt.Figure,
    rgb: np.ndarray,
    semantic: np.ndarray,
    depth: np.ndarray,
    truth_world: np.ndarray,
    graph: list[dict],
    position: np.ndarray,
    orientation: list[float],
    pixels: np.ndarray,
    calibrated: np.ndarray,
    projected_world: np.ndarray,
    clipped_at_pixels: np.ndarray,
) -> None:
    ax = figure.add_axes([0.015, 0.10, 0.97, 0.86])
    projector = PerspectiveProjector()
    draw_image_plane(ax, projector, build_plane_texture(rgb, semantic, depth))

    active = (calibrated >= ACTIVE_SCORE) & clipped_at_pixels
    active_world = projected_world[active]
    active_local = world_to_body(active_world, position, orientation)
    active_scores = calibrated[active]

    # The real map is only visual context, as in YOPO's original figure.
    truth_local = world_to_body(truth_world, position, orientation)
    truth_mask = (
        (truth_local[:, 0] >= -3.0) & (truth_local[:, 0] <= 31.5)
        & (np.abs(truth_local[:, 1]) <= 25.0)
        & (truth_local[:, 2] >= -4.0) & (truth_local[:, 2] <= 11.5)
    )
    truth_local = truth_local[truth_mask]
    if len(truth_local) > 20_000:
        rng = np.random.default_rng(7)
        truth_local = truth_local[rng.choice(len(truth_local), 20_000, replace=False)]
    truth_x, truth_y, truth_depth = projector(truth_local)
    point_order = np.argsort(truth_depth)[::-1]
    ax.scatter(
        truth_x[point_order], truth_y[point_order], s=0.22,
        color="#87979F", alpha=0.13, linewidths=0,
        rasterized=True, zorder=3,
    )

    selected_world = marker_points(graph, "epic_selected_witness_path")
    selected_local = world_to_body(selected_world, position, orientation)
    witness_world = marker_points(graph, "epic_edge_witness_paths")
    witness_segments_world = witness_world[: len(witness_world) // 2 * 2].reshape(-1, 2, 3)
    witness_centers_local = world_to_body(
        witness_segments_world.mean(axis=1), position, orientation
    )
    near_selected = distance_to_polyline(
        witness_segments_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2).min(axis=1) <= 6.2
    visible = (
        (witness_centers_local[:, 0] >= -2.5)
        & (witness_centers_local[:, 0] <= 30.5)
        & (np.abs(witness_centers_local[:, 1]) <= 18.0)
        & (witness_centers_local[:, 2] >= -3.0)
        & (witness_centers_local[:, 2] <= 6.0)
    )
    witness_segments_world = witness_segments_world[near_selected & visible]
    witness_segments_local = world_to_body(
        witness_segments_world.reshape(-1, 3), position, orientation
    ).reshape(-1, 2, 3)
    exposure = segment_semantic_exposure(
        witness_segments_world, active_world, active_scores
    )
    exposure_map = LinearSegmentedColormap.from_list(
        "witness_semantic_exposure", [RISK_LOW, RISK_MID, RISK_HIGH]
    )
    exposure_norm = Normalize(0.0, 0.75)
    ax.add_collection(LineCollection(
        projected_line_segments(projector, witness_segments_local),
        colors=exposure_map(exposure_norm(exposure)),
        linewidths=1.75,
        alpha=0.90,
        zorder=7,
    ))

    edge_world = marker_points(graph, "epic_skeleton_edges")
    edge_segments_world = edge_world[: len(edge_world) // 2 * 2].reshape(-1, 2, 3)
    edge_centers_local = world_to_body(edge_segments_world.mean(axis=1), position, orientation)
    edge_near = distance_to_polyline(
        edge_segments_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2).min(axis=1) <= 5.2
    edge_visible = (
        (edge_centers_local[:, 0] >= -2.5) & (edge_centers_local[:, 0] <= 30.5)
        & (np.abs(edge_centers_local[:, 1]) <= 20.0)
        & (edge_centers_local[:, 2] >= -3.5) & (edge_centers_local[:, 2] <= 7.0)
    )
    edge_segments_local = world_to_body(
        edge_segments_world[edge_near & edge_visible].reshape(-1, 3),
        position, orientation,
    ).reshape(-1, 2, 3)
    ax.add_collection(LineCollection(
        projected_line_segments(projector, edge_segments_local),
        colors=TOPOLOGY, linewidths=0.55, alpha=0.34, zorder=5,
    ))

    nodes_world, node_colors = marker_points_with_colors(graph, "epic_skeleton_nodes")
    nodes_local = world_to_body(nodes_world, position, orientation)
    node_distance = distance_to_polyline(nodes_world, selected_world)
    node_mask = (
        (node_distance <= 5.4)
        & (nodes_local[:, 0] >= -2.5) & (nodes_local[:, 0] <= 30.5)
        & (np.abs(nodes_local[:, 1]) <= 20.0)
        & (nodes_local[:, 2] >= -3.5) & (nodes_local[:, 2] <= 7.0)
    )
    node_x, node_y, _ = projector(nodes_local[node_mask])
    visible_node_colors = (
        node_colors[node_mask] if node_colors is not None
        else np.repeat(np.asarray([[0.40, 0.48, 0.52, 1.0]]), np.count_nonzero(node_mask), axis=0)
    )
    ax.scatter(
        node_x, node_y, c=visible_node_colors, s=18.0,
        edgecolors="white", linewidths=0.42, alpha=1.0,
        rasterized=True, zorder=9,
    )

    # Orange anchors are the implementation's true fixed-Z semantic rays.
    camera_origin = CAMERA_TRANSLATION_FLU
    camera_x, camera_y, _ = projector(camera_origin[None, :])
    projected_x, projected_y, _ = projector(active_local)
    for endpoint_x, endpoint_y, score in zip(projected_x, projected_y, active_scores):
        ax.plot(
            [camera_x[0], endpoint_x], [camera_y[0], endpoint_y],
            color=ANCHOR, lw=1.1 + 1.35 * float(score), alpha=0.82,
            solid_capstyle="round", zorder=6,
        )
    ax.scatter(
        projected_x, projected_y, c=active_scores,
        cmap=LinearSegmentedColormap.from_list(
            "semantic_maxima", [RISK_MID, "#E4773E", RISK_HIGH]
        ),
        norm=Normalize(0.35, 0.85), s=48, marker="o",
        edgecolors="white", linewidths=0.9, zorder=12,
    )

    selected_x, selected_y, _ = projector(selected_local)
    ax.plot(selected_x, selected_y, color="white", lw=6.4,
            solid_capstyle="round", zorder=10)
    ax.plot(selected_x, selected_y, color=SELECTED, lw=4.1,
            solid_capstyle="round", zorder=11)

    ax.scatter(camera_x, camera_y, marker="^", s=95, color=UAV,
               edgecolor="white", linewidth=1.1, zorder=15)
    ax.annotate(
        r"vehicle at $t^*$", xy=(camera_x[0], camera_y[0]),
        xytext=(-6, -17), textcoords="offset points", ha="right",
        fontsize=7.7, color=UAV, fontweight="bold", bbox=LABEL_BOX, zorder=16,
    )
    draw_coordinate_axes(ax, projector, camera_origin)

    frontier_world = marker_pose_position(graph, "epic_frontier_goal", "epic_route_terminal")
    local_goal_world = marker_pose_position(graph, "epic_local_goal", "epic_yopo_next_goal")
    mission_world = marker_pose_position(graph, "epic_global_goal")
    if local_goal_world is not None:
        annotate_goal(
            ax, projector, world_to_body(local_goal_world[None, :], position, orientation)[0],
            "local_goal", LOCAL_GOAL, "o", (-6, -20),
        )
    if frontier_world is not None:
        frontier_local = world_to_body(frontier_world[None, :], position, orientation)[0]
        annotate_goal(
            ax, projector, frontier_local, "frontier_goal", FRONTIER, "D", (10, 20)
        )
    else:
        frontier_local = selected_local[-1]
    if mission_world is not None:
        mission_local = world_to_body(mission_world[None, :], position, orientation)[0]
        direction = mission_local - frontier_local
        direction /= max(np.linalg.norm(direction), 1e-9)
        arrow_end = frontier_local + 7.5 * direction
        start_x, start_y, _ = projector(frontier_local[None, :])
        end_x, end_y, _ = projector(arrow_end[None, :])
        ax.annotate(
            "", xy=(end_x[0], end_y[0]), xytext=(start_x[0], start_y[0]),
            arrowprops=dict(arrowstyle="-|>", color=MISSION, lw=1.2), zorder=15,
        )
        ax.annotate(
            "mission_goal", xy=(end_x[0], end_y[0]), xytext=(7, -14),
            textcoords="offset points", fontsize=7.8, color=MISSION,
            fontweight="bold", bbox=LABEL_BOX, zorder=16,
        )

    highest = int(np.argmax(active_scores))
    ax.annotate(
        r"depth-clipped semantic evidence ($D_t\geq20$ m)",
        xy=(projected_x[highest], projected_y[highest]),
        xytext=(-14, 19), textcoords="offset points", ha="right",
        fontsize=7.4, color=RISK_HIGH, fontweight="bold", bbox=LABEL_BOX,
        arrowprops=dict(arrowstyle="->", color=RISK_HIGH, lw=0.9), zorder=16,
    )
    ax.text(
        0.21, 0.71, r"(a)",
        transform=ax.transAxes, fontsize=9.2, color=INK,
        fontweight="bold", bbox=LABEL_BOX, zorder=18,
    )
    ax.text(
        0.52, 0.34, r"(b)",
        transform=ax.transAxes, fontsize=9.2, color=INK,
        fontweight="bold", bbox=LABEL_BOX, zorder=18,
    )

    score_axis = ax.inset_axes([0.76, 0.89, 0.18, 0.022])
    score_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=exposure_norm, cmap=exposure_map),
        cax=score_axis, orientation="horizontal", ticks=[0.0, 0.75],
    )
    score_bar.outline.set_visible(False)
    score_bar.ax.tick_params(labelsize=5.7, length=1.2, pad=1)
    score_bar.ax.set_title("semantic exposure", fontsize=6.2, color=INK, pad=1.5)

    legend_handles = [
        Line2D([], [], color=GRID_BLUE, lw=1.2, label="image grid"),
        Line2D([], [], color=ANCHOR, lw=1.6, label=r"fixed-$Z$ semantic ray"),
        Line2D([], [], color=RISK_LOW, lw=1.5, marker="o", markersize=3.1,
               markerfacecolor=RISK_HIGH, markeredgecolor="white",
               label="raycast-valid witnesses (semantic exposure)"),
        Line2D([], [], color=TOPOLOGY, lw=0, marker="o", markersize=4.0,
               markeredgecolor="white", label="semantic graph node"),
        Line2D([], [], color=SELECTED, lw=2.8, label="selected witness"),
    ]
    ax.legend(
        handles=legend_handles, loc="lower center", bbox_to_anchor=(0.50, -0.06),
        ncol=5, fontsize=6.5, frameon=False, handlelength=2.0,
        columnspacing=1.0, borderaxespad=0.0,
    )

    plane_corners = plane_points_from_pixels(
        np.asarray([0.0, rgb.shape[1], rgb.shape[1], 0.0]),
        np.asarray([0.0, 0.0, rgb.shape[0], rgb.shape[0]]),
        rgb.shape[1], rgb.shape[0],
    )
    content = np.vstack((plane_corners, camera_origin, selected_local))
    bounds_x, bounds_y, _ = projector(content)
    horizontal_padding = 0.075 * (bounds_x.max() - bounds_x.min())
    vertical_padding = 0.075 * (bounds_y.max() - bounds_y.min())
    ax.set_xlim(bounds_x.min() - horizontal_padding, bounds_x.max() + horizontal_padding)
    ax.set_ylim(bounds_y.min() - vertical_padding, bounds_y.max() + vertical_padding)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_facecolor(BACKGROUND)
    for spine in ax.spines.values():
        spine.set_visible(False)


def main() -> None:
    args = parse_args()
    session = args.session.resolve()
    events = load_events(session)
    rgb_event = exact_event(events, "rgb", args.rgb)
    stamp_ns = int(rgb_event["stamp_ns"])
    semantic_event = nearest_event(events["semantic"], stamp_ns)
    depth_event = nearest_event(events["depth"], stamp_ns)
    graph_event = nearest_event(events["graph"], stamp_ns)
    odom_event = nearest_event(events["odom"], stamp_ns)

    sync_events = (semantic_event, depth_event, graph_event, odom_event)
    max_sync_ms = max(
        abs(int(event["stamp_ns"]) - stamp_ns) for event in sync_events
    ) / 1e6
    if max_sync_ms > 50.0:
        raise ValueError(
            f"selected frame is not synchronized: max delta {max_sync_ms:.1f} ms"
        )

    rgb = np.asarray(Image.open(session / rgb_event["file"]).convert("RGB"))
    semantic = np.clip(read_scalar_image(session, semantic_event), 0.0, 1.0)
    depth = read_scalar_image(session, depth_event)
    pose = odom_event["data"]
    position = np.asarray(pose["position"], dtype=float)
    pixels, _, calibrated, projected_world = max_pool_and_project(
        semantic, position, pose["orientation"]
    )
    clipped_at_pixels = np.asarray([
        depth[pixel_v, pixel_u] >= DEPTH_CLIP_M for pixel_u, pixel_v in pixels
    ])
    graph = load_graph(session, graph_event)
    truth_world = load_ply_xyz(SCRIPT_DIR / "map2_ground_truth.ply")

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 8,
        "figure.facecolor": BACKGROUND,
        "savefig.facecolor": BACKGROUND,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    figure = plt.figure(figsize=(9.4, 4.15), facecolor=BACKGROUND)
    draw_scene(
        figure, rgb, semantic, depth, truth_world, graph, position,
        pose["orientation"], pixels, calibrated, projected_world,
        clipped_at_pixels,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output.with_suffix(".pdf"), dpi=300)
    figure.savefig(args.output.with_suffix(".png"), dpi=300)
    plt.close(figure)
    active_count = int(np.count_nonzero(
        (calibrated >= ACTIVE_SCORE) & clipped_at_pixels
    ))
    print(
        f"wrote {args.output.with_suffix('.pdf')} and .png from {session.name}; "
        f"frame={rgb_event['file']}; semantic={semantic_event['file']}; "
        f"depth={depth_event['file']}; graph={graph_event['file']}; "
        f"max_sync={max_sync_ms:.3f} ms; active_depth-clipped_rays={active_count}"
    )


if __name__ == "__main__":
    main()
