#!/usr/bin/env python3
"""Visualize one real ScaleNav semantic-to-graph decision.

The two panels use one synchronized Map2 state. Numbered image maxima correspond
to fixed-optical-Z endpoints in the graph view, while the graph witnesses are
colored from endpoint-to-witness distance exactly as used by the planner.
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
from matplotlib.patches import Circle
from PIL import Image, ImageFilter


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
    marker_points,
    marker_points_with_colors,
    marker_pose_position,
    nearest_event,
    quat_rotate,
    read_scalar_image,
)


DEFAULT_SESSION = REPO_ROOT / "log_scalenav/session_20260826_192224_661"
DEFAULT_RGB = "rgb/rgb_224.ppm"
DEFAULT_GRAPH = "graph/graph_267.json"
DEFAULT_OUTPUT = SCRIPT_DIR / "semantic_risk_field_paper"

CAMERA_TRANSLATION_FLU = np.asarray([0.5, 0.0, -0.1])
HORIZONTAL_FOV_DEG = 90.0
VERTICAL_FOV_DEG = 60.0
VIRTUAL_OPTICAL_Z_M = 30.0
PATCH_COLS = 5
PATCH_ROWS = 3
ACTIVE_SCORE = 0.35
RISK_RADIUS_M = 5.0
RISK_SIGMA_M = 0.5 * RISK_RADIUS_M

GRID_BLUE = "#48A4D2"
DEPTH_BLUE = "#657985"
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
    parser.add_argument("--graph", default=DEFAULT_GRAPH,
                        help="First logged graph update with the resulting route change")
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
        self.camera = np.asarray([-18.0, -35.0, 18.0])
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
        # Match YOPO's composition without viewing the RGB plane from its back.
        horizontal = self.focal * (relative @ self.right) / safe_depth
        vertical = self.focal * (relative @ self.up) / safe_depth
        return horizontal, vertical, depth


def build_plane_texture(
    rgb: np.ndarray,
    semantic: np.ndarray,
    score_vmax: float | None = None,
) -> np.ndarray:
    """Overlay the semantic heatmap directly on the synchronized RGB frame."""
    rgb_float = np.asarray(rgb, dtype=float) / 255.0

    risk_map = LinearSegmentedColormap.from_list(
        "image_semantic_risk", ["#F4C95D", "#ED7D3A", "#C43C39"]
    )
    vmin = float(np.min(semantic))
    vmax = max(float(score_vmax or np.max(semantic)), vmin + 1e-3)
    threshold = max(0.35, float(np.quantile(semantic, 0.82)))
    risk_rgb = risk_map(np.clip((semantic - vmin) / (vmax - vmin), 0.0, 1.0))[..., :3]
    # Show only measured high-response regions. Low responses retain the
    # synchronized RGB instead of washing the whole camera image with color.
    risk_alpha = np.clip((semantic - threshold) / max(vmax - threshold, 1e-3),
                         0.0, 0.58)[..., None]
    texture = rgb_float * (1.0 - risk_alpha) + risk_rgb * risk_alpha
    return np.clip(texture, 0.0, 1.0)


def build_depth_texture(depth: np.ndarray) -> np.ndarray:
    """Render measured depth while leaving clipped pixels nearly white."""
    depth_map = LinearSegmentedColormap.from_list(
        "measured_depth", ["#102B38", "#3E8796", "#DCE6E9"]
    )
    texture = depth_map(np.clip(depth / DEPTH_CLIP_M, 0.0, 1.0))[..., :3]
    clipped = depth >= DEPTH_CLIP_M
    texture[clipped] = np.asarray([0.88, 0.94, 0.96])
    row_index, column_index = np.indices(depth.shape)
    hatch = clipped & (((row_index + column_index) % 7) == 0)
    texture[hatch] = np.asarray([0.56, 0.72, 0.78])
    return texture


def plane_points_from_pixels(
    pixel_u: np.ndarray,
    pixel_v: np.ndarray,
    width: int,
    height: int,
    optical_z: float = VIRTUAL_OPTICAL_Z_M,
    offset_y: float = 0.0,
    offset_z: float = 0.0,
) -> np.ndarray:
    normalized_u = np.asarray(pixel_u, dtype=float) / width
    normalized_v = np.asarray(pixel_v, dtype=float) / height
    horizontal_tangent = math.tan(math.radians(HORIZONTAL_FOV_DEG / 2.0))
    vertical_tangent = math.tan(math.radians(VERTICAL_FOV_DEG / 2.0))
    return np.column_stack((
        np.full(np.size(normalized_u), CAMERA_TRANSLATION_FLU[0] + optical_z),
        CAMERA_TRANSLATION_FLU[1] + offset_y
        - (2.0 * normalized_u.ravel() - 1.0) * horizontal_tangent * optical_z,
        CAMERA_TRANSLATION_FLU[2] + offset_z
        - (2.0 * normalized_v.ravel() - 1.0) * vertical_tangent * optical_z,
    ))


def draw_image_plane(
    ax: plt.Axes,
    projector: PerspectiveProjector,
    texture: np.ndarray,
    *,
    optical_z: float = VIRTUAL_OPTICAL_Z_M,
    offset_y: float = 0.0,
    offset_z: float = 0.0,
    border_color: str = GRID_BLUE,
    draw_grid: bool = True,
    alpha: float = 0.84,
    zorder: float = 1.0,
    screen_scale: float = 1.0,
    screen_offset: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    """Rasterize one sensor image as a perspective paper-like plane."""
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
    plane = plane_points_from_pixels(
        grid_u, grid_v, width, height, optical_z, offset_y, offset_z
    )
    screen_u, screen_v, _ = projector(plane)
    screen_u = screen_u.reshape(render_height + 1, render_width + 1)
    screen_v = screen_v.reshape(render_height + 1, render_width + 1)
    center_u = float(np.mean(screen_u))
    center_v = float(np.mean(screen_v))
    screen_u = center_u + screen_scale * (screen_u - center_u) + screen_offset[0]
    screen_v = center_v + screen_scale * (screen_v - center_v) + screen_offset[1]

    def transform_screen(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (
            center_u + screen_scale * (x - center_u) + screen_offset[0],
            center_v + screen_scale * (y - center_v) + screen_offset[1],
        )

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
        alpha=alpha,
        rasterized=True,
        zorder=zorder,
    ))

    columns = range(PATCH_COLS + 1) if draw_grid else (0, PATCH_COLS)
    rows = range(PATCH_ROWS + 1) if draw_grid else (0, PATCH_ROWS)
    for column in columns:
        u = np.full(80, column * width / PATCH_COLS)
        v = np.linspace(0.0, height, 80)
        x, y, _ = projector(plane_points_from_pixels(
            u, v, width, height, optical_z, offset_y, offset_z
        ))
        x, y = transform_screen(x, y)
        ax.plot(x, y, color=border_color, lw=1.35, alpha=0.96, zorder=zorder + 1)
    for row in rows:
        u = np.linspace(0.0, width, 120)
        v = np.full(120, row * height / PATCH_ROWS)
        x, y, _ = projector(plane_points_from_pixels(
            u, v, width, height, optical_z, offset_y, offset_z
        ))
        x, y = transform_screen(x, y)
        ax.plot(x, y, color=border_color, lw=1.35, alpha=0.96, zorder=zorder + 1)

    corners = plane_points_from_pixels(
        np.asarray([0.0, width, width, 0.0]),
        np.asarray([0.0, 0.0, height, height]),
        width, height, optical_z, offset_y, offset_z,
    )
    corner_x, corner_y, _ = projector(corners)
    corner_x, corner_y = transform_screen(corner_x, corner_y)
    return np.column_stack((corner_x, corner_y))


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
        -(distances ** 2) / (2.0 * RISK_SIGMA_M ** 2)
    )
    influence[distances > RISK_RADIUS_M] = 0.0
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
    graph: list[dict],
    position: np.ndarray,
    orientation: list[float],
    pixels: np.ndarray,
    calibrated: np.ndarray,
    projected_world: np.ndarray,
    clipped_at_pixels: np.ndarray,
) -> None:
    ax = figure.add_axes([0.02, 0.12, 0.96, 0.84])
    projector = PerspectiveProjector()
    plane_corners = draw_image_plane(
        ax, projector, build_plane_texture(rgb, semantic),
        border_color=GRID_BLUE, draw_grid=True, alpha=0.88, zorder=2.0,
    )

    active = (calibrated >= ACTIVE_SCORE) & clipped_at_pixels
    active_world = projected_world[active]
    active_local = world_to_body(active_world, position, orientation)
    active_scores = calibrated[active]

    selected_world = marker_points(graph, "epic_selected_witness_path")
    # The persistent graph stores witness samples at the vehicle flight layer.
    # For an image-plane trajectory they must be intersected with the scene
    # ground; otherwise the route projects almost horizontally at the horizon.
    selected_ground_world = selected_world.copy()
    selected_ground_world[:, 2] = 0.0
    selected_local = world_to_body(selected_ground_world, position, orientation)
    witness_world = marker_points(graph, "epic_edge_witness_paths")
    witness_segments_world = witness_world[: len(witness_world) // 2 * 2].reshape(-1, 2, 3)
    witness_centers_local = world_to_body(
        witness_segments_world.mean(axis=1), position, orientation
    )
    witness_endpoint_distance = distance_to_polyline(
        witness_segments_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2)
    near_selected = np.max(witness_endpoint_distance, axis=1) <= 3.2
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
        linewidths=1.55,
        alpha=0.78,
        zorder=7,
    ))

    edge_world = marker_points(graph, "epic_skeleton_edges")
    edge_segments_world = edge_world[: len(edge_world) // 2 * 2].reshape(-1, 2, 3)
    edge_centers_local = world_to_body(edge_segments_world.mean(axis=1), position, orientation)
    edge_endpoint_distance = distance_to_polyline(
        edge_segments_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2)
    edge_near = np.max(edge_endpoint_distance, axis=1) <= 4.0
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
        (node_distance <= 4.0)
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
            color=ANCHOR, lw=0.75 + 0.70 * float(score), alpha=0.78,
            linestyle=(0, (3.0, 2.0)), solid_capstyle="round", zorder=6,
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
            ax, projector, frontier_local,
            "frontier_goal", FRONTIER, "D", (8, 17)
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
        r"projected semantic evidence",
        xy=(projected_x[highest], projected_y[highest]),
        xytext=(-12, 17), textcoords="offset points", ha="right",
        fontsize=6.8, color=RISK_HIGH, fontweight="bold", bbox=LABEL_BOX,
        arrowprops=dict(arrowstyle="->", color=RISK_HIGH, lw=0.85), zorder=16,
    )
    ax.text(
        0.25, 0.58, r"(a)",
        transform=ax.transAxes, fontsize=9.2, color=INK,
        fontweight="bold", bbox=LABEL_BOX, zorder=18,
    )
    ax.text(
        0.70, 0.26, r"(b)",
        transform=ax.transAxes, fontsize=9.2, color=INK,
        fontweight="bold", bbox=LABEL_BOX, zorder=18,
    )

    score_axis = ax.inset_axes([0.76, 0.76, 0.18, 0.026])
    score_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=exposure_norm, cmap=exposure_map),
        cax=score_axis, orientation="horizontal", ticks=[0.0, 0.75],
    )
    score_bar.outline.set_visible(False)
    score_bar.ax.tick_params(labelsize=5.7, length=1.2, pad=1)
    score_bar.ax.set_title("semantic exposure", fontsize=5.8, color=INK, pad=1.5)

    legend_handles = [
        Line2D([], [], color=GRID_BLUE, lw=1.4, label="RGB + PEARL image grid"),
        Line2D([], [], color=ANCHOR, lw=1.4, linestyle=(0, (3.0, 2.0)),
               label=r"fixed-$Z$ semantic projection"),
        Line2D([], [], color=RISK_LOW, lw=1.5, marker="o", markersize=3.1,
               markerfacecolor=RISK_HIGH, markeredgecolor="white",
               label="raycast-valid witnesses (semantic exposure)"),
        Line2D([], [], color=TOPOLOGY, lw=0, marker="o", markersize=4.0,
               markeredgecolor="white", label="semantic graph node"),
        Line2D([], [], color=SELECTED, lw=2.8, label="selected witness"),
    ]
    ax.legend(
        handles=legend_handles, loc="lower center", bbox_to_anchor=(0.50, -0.07),
        ncol=5, fontsize=6.1, frameon=False, handlelength=1.8,
        columnspacing=0.8, borderaxespad=0.0,
    )

    graph_content = [camera_origin, selected_local, active_local]
    if len(witness_segments_local):
        graph_content.append(witness_segments_local.reshape(-1, 3))
    if len(edge_segments_local):
        graph_content.append(edge_segments_local.reshape(-1, 3))
    if np.any(node_mask):
        graph_content.append(nodes_local[node_mask])
    for optional_goal in (local_goal_world, frontier_world, mission_world):
        if optional_goal is not None:
            graph_content.append(world_to_body(optional_goal[None, :], position, orientation))
    graph_x, graph_y, _ = projector(np.vstack(graph_content))
    bounds_x = np.concatenate((plane_corners[:, 0], graph_x))
    bounds_y = np.concatenate((plane_corners[:, 1], graph_y))
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


def draw_scene_2d(
    figure: plt.Figure,
    rgb: np.ndarray,
    semantic: np.ndarray,
    graph: list[dict],
    position: np.ndarray,
    orientation: list[float],
    calibrated: np.ndarray,
    projected_world: np.ndarray,
    clipped_at_pixels: np.ndarray,
) -> None:
    """Reproject one synchronized graph decision into the real RGB image."""
    ax = figure.add_axes([0.018, 0.115, 0.964, 0.865])
    height, width = rgb.shape[:2]
    semantic_vmin = float(np.min(semantic))
    semantic_vmax = float(np.max(semantic))

    # The logger stores a 160x96 RGB stream. Upscale once with a restrained
    # unsharp mask so the paper raster stays legible without inventing detail.
    display_rgb = Image.fromarray(np.asarray(rgb, dtype=np.uint8)).resize(
        (width * 6, height * 6), Image.Resampling.LANCZOS
    ).filter(ImageFilter.UnsharpMask(radius=1.0, percent=90, threshold=3))
    ax.imshow(display_rgb, extent=(0, width, height, 0), interpolation="nearest",
              zorder=0)

    risk_map = LinearSegmentedColormap.from_list(
        "pearl_score", ["#F4C95D", "#ED7D3A", "#C43C39"]
    )
    risk_threshold = max(0.35, float(np.quantile(semantic, 0.82)))
    risk_alpha = np.clip(
        (semantic - risk_threshold) / max(semantic_vmax - risk_threshold, 1e-3),
        0.0, 0.58,
    )
    ax.imshow(
        semantic, cmap=risk_map,
        norm=Normalize(semantic_vmin, semantic_vmax),
        alpha=risk_alpha, extent=(0, width, height, 0),
        interpolation="bilinear", zorder=2,
    )

    # Keep the deployed 5x3 pooling visible but subordinate to the graph.
    for column in range(1, PATCH_COLS):
        x = column * width / PATCH_COLS
        ax.plot([x, x], [0, height], color="white", lw=0.45,
                alpha=0.32, zorder=3)
    for row in range(1, PATCH_ROWS):
        y = row * height / PATCH_ROWS
        ax.plot([0, width], [y, y], color="white", lw=0.45,
                alpha=0.32, zorder=3)

    horizontal_tangent = math.tan(math.radians(HORIZONTAL_FOV_DEG / 2.0))
    vertical_tangent = math.tan(math.radians(VERTICAL_FOV_DEG / 2.0))

    def project_to_image(points_local: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        points_local = np.asarray(points_local, dtype=float).reshape(-1, 3)
        # The calibrated pinhole model is centered at the optical camera, not
        # at the vehicle origin.  Remove the logged FLU camera translation
        # before applying the optical-to-FLU axis convention.
        camera_relative = points_local - CAMERA_TRANSLATION_FLU
        forward = camera_relative[:, 0]
        safe_forward = np.maximum(forward, 1e-6)
        u = width * 0.5 * (
            1.0 - camera_relative[:, 1] / (safe_forward * horizontal_tangent)
        )
        v = height * 0.5 * (
            1.0 - camera_relative[:, 2] / (safe_forward * vertical_tangent)
        )
        return u, v, forward

    selected_world = marker_points(graph, "epic_selected_witness_path")
    ground_z = 0.0

    def ground_project(points_world: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        points_ground = np.asarray(points_world, dtype=float).copy().reshape(-1, 3)
        points_ground[:, 2] = ground_z
        return project_to_image(world_to_body(points_ground, position, orientation))

    selected_u, selected_v, selected_forward = ground_project(selected_world)

    witness_world = marker_points(graph, "epic_edge_witness_paths")
    witness_segments_world = witness_world[: len(witness_world) // 2 * 2].reshape(-1, 2, 3)
    witness_endpoint_distance = distance_to_polyline(
        witness_segments_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2)
    candidate_mask = np.max(witness_endpoint_distance, axis=1) <= 3.4
    candidate_world = witness_segments_world[candidate_mask]
    candidate_u, candidate_v, candidate_forward = ground_project(
        candidate_world.reshape(-1, 3)
    )
    candidate_u = candidate_u.reshape(-1, 2)
    candidate_v = candidate_v.reshape(-1, 2)
    candidate_forward = candidate_forward.reshape(-1, 2)
    visible_candidate = (
        np.all(candidate_forward > 0.45, axis=1)
        & np.all(candidate_u >= -0.08 * width, axis=1)
        & np.all(candidate_u <= 1.08 * width, axis=1)
        & np.all(candidate_v >= -0.08 * height, axis=1)
        & np.all(candidate_v <= 1.08 * height, axis=1)
    )
    candidate_screen_length = np.hypot(
        candidate_u[:, 1] - candidate_u[:, 0],
        candidate_v[:, 1] - candidate_v[:, 0],
    )
    visible_candidate &= candidate_screen_length <= 32.0
    candidate_world = candidate_world[visible_candidate]
    candidate_u = candidate_u[visible_candidate]
    candidate_v = candidate_v[visible_candidate]
    if len(candidate_world) > 55:
        keep = np.linspace(0, len(candidate_world) - 1, 55, dtype=int)
        candidate_world = candidate_world[keep]
        candidate_u = candidate_u[keep]
        candidate_v = candidate_v[keep]
    edge_segments = [np.column_stack((u, v)) for u, v in zip(candidate_u, candidate_v)]
    ax.add_collection(LineCollection(
        edge_segments, colors="#49616E", linewidths=0.9, alpha=0.58, zorder=6,
    ))

    nodes_world, node_colors = marker_points_with_colors(
        graph, "epic_skeleton_nodes"
    )
    node_distance = distance_to_polyline(nodes_world, selected_world)
    node_u, node_v, node_forward = ground_project(nodes_world)
    node_mask = (
        (node_distance <= 3.4) & (node_forward > 0.45)
        & (node_u >= 0.0) & (node_u <= width)
        & (node_v >= 0.0) & (node_v <= height)
    )
    visible_node_indices = np.flatnonzero(node_mask)
    if len(visible_node_indices) > 36:
        order = np.argsort(node_distance[visible_node_indices])[:36]
        visible_node_indices = visible_node_indices[order]
    visible_node_colors = (
        node_colors[visible_node_indices, :3]
        if node_colors is not None else TOPOLOGY
    )
    ax.scatter(
        node_u[visible_node_indices], node_v[visible_node_indices], s=19,
        color=visible_node_colors, edgecolor="white", linewidth=0.55,
        alpha=0.98, zorder=9,
    )

    selected_visible = (
        (selected_forward > 0.45)
        & (selected_u >= 0.0) & (selected_u <= width)
        & (selected_v >= 0.0) & (selected_v <= height)
    )
    selected_segments = []
    for index in range(len(selected_world) - 1):
        if selected_visible[index] and selected_visible[index + 1]:
            selected_segments.append(np.asarray([
                [selected_u[index], selected_v[index]],
                [selected_u[index + 1], selected_v[index + 1]],
            ]))
    for linewidth, color, zorder in ((7.0, "white", 11), (4.2, SELECTED, 12)):
        ax.add_collection(LineCollection(
            selected_segments, colors=[color] * len(selected_segments),
            linewidths=linewidth, capstyle="round", zorder=zorder,
        ))
    if selected_segments:
        direction_segment = selected_segments[-1]
        ax.annotate(
            "", xy=direction_segment[1], xytext=direction_segment[0],
            arrowprops=dict(arrowstyle="-|>", color=SELECTED, lw=1.35,
                            shrinkA=0, shrinkB=0), zorder=13,
        )

    def image_goal(
        point_world: np.ndarray | None,
        label: str,
        color: str,
        marker: str,
        offset: tuple[float, float],
    ) -> None:
        if point_world is None:
            return
        goal_u, goal_v, goal_forward = ground_project(point_world[None, :])
        if (goal_forward[0] <= 0.45 or goal_u[0] < 0 or goal_u[0] > width
                or goal_v[0] < 0 or goal_v[0] > height):
            return
        ax.scatter(goal_u, goal_v, marker=marker, s=95, color=color,
                   edgecolor="white", linewidth=1.2, zorder=14)
        ax.annotate(
            label, xy=(goal_u[0], goal_v[0]), xytext=offset,
            textcoords="offset points", fontsize=8.1, color=color,
            fontweight="bold", bbox=LABEL_BOX,
            arrowprops=dict(arrowstyle="-", color=color, lw=0.7), zorder=15,
        )

    image_goal(
        marker_pose_position(graph, "epic_local_goal", "epic_yopo_next_goal"),
        "local_goal", LOCAL_GOAL, "o", (7, 14),
    )
    image_goal(
        marker_pose_position(graph, "epic_frontier_goal", "epic_route_terminal"),
        "frontier_goal", FRONTIER, "D", (7, -17),
    )
    score_axis = ax.inset_axes([0.755, 0.055, 0.205, 0.027])
    score_bar = figure.colorbar(
        plt.cm.ScalarMappable(
            norm=Normalize(semantic_vmin, semantic_vmax), cmap=risk_map
        ),
        cax=score_axis, orientation="horizontal",
        ticks=[semantic_vmin, semantic_vmax],
    )
    score_bar.outline.set_visible(False)
    score_bar.ax.tick_params(labelsize=5.8, length=1.2, pad=1)
    score_bar.ax.set_title("PEARL score", fontsize=6.2, color=INK, pad=1.5)

    legend_handles = [
        Line2D([], [], color="#ED7D3A", lw=5.0, alpha=0.65,
               label="semantic risk"),
        Line2D([], [], color="#49616E", lw=0.9, marker="o", markersize=4.2,
               markeredgecolor="white", label="raycast-valid graph"),
        Line2D([], [], color=SELECTED, lw=3.2, label="selected witness"),
        Line2D([], [], color=LOCAL_GOAL, lw=0, marker="o", markersize=5,
               markeredgecolor="white", label="local_goal"),
        Line2D([], [], color=FRONTIER, lw=0, marker="D", markersize=5,
               markeredgecolor="white", label="frontier_goal"),
    ]
    ax.legend(
        handles=legend_handles, loc="lower left", bbox_to_anchor=(0.01, -0.115),
        ncol=5, fontsize=6.2, frameon=False, handlelength=1.8,
        columnspacing=1.0, borderaxespad=0.0,
    )
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def draw_yopo_scene(
    figure: plt.Figure,
    rgb: np.ndarray,
    semantic: np.ndarray,
    graph: list[dict],
    position: np.ndarray,
    orientation: list[float],
    pixels: np.ndarray,
    calibrated: np.ndarray,
    projected_world: np.ndarray,
    clipped_at_pixels: np.ndarray,
) -> None:
    """Compose a YOPO-style image plane behind the true world-space graph."""
    ax = figure.add_axes([0.02, 0.12, 0.96, 0.84])
    projector = PerspectiveProjector()
    # Keep the image plane and graph in one metric camera/body frame. Any
    # independent screen-space scaling would turn the dashed lines below into
    # illustrative connectors rather than the implemented pinhole rays.
    plane_scale = 1.0
    plane_offset = (0.0, 0.0)
    semantic_vmin = float(np.min(semantic))
    semantic_vmax = float(np.max(semantic))
    plane_corners = draw_image_plane(
        ax, projector, build_plane_texture(rgb, semantic, semantic_vmax),
        border_color=GRID_BLUE, draw_grid=True, alpha=0.94, zorder=2.0,
        screen_scale=plane_scale, screen_offset=plane_offset,
    )

    height, width = semantic.shape
    render_width = 80
    render_height = max(1, round(height * render_width / width))
    center_grid_u, center_grid_v = np.meshgrid(
        np.linspace(0.0, width, render_width + 1),
        np.linspace(0.0, height, render_height + 1),
    )
    center_plane = plane_points_from_pixels(
        center_grid_u, center_grid_v, width, height
    )
    center_x, center_y, _ = projector(center_plane)
    plane_center = (float(np.mean(center_x)), float(np.mean(center_y)))

    def transform_plane(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (
            plane_center[0] + plane_scale * (x - plane_center[0]) + plane_offset[0],
            plane_center[1] + plane_scale * (y - plane_center[1]) + plane_offset[1],
        )

    semantic_local = world_to_body(projected_world, position, orientation)
    anchor_x, anchor_y, _ = projector(semantic_local)
    anchor_x, anchor_y = transform_plane(anchor_x, anchor_y)

    height, width = semantic.shape
    normalized_u = (pixels[:, 0] + 0.5) / width
    normalized_v = (pixels[:, 1] + 0.5) / height
    fov_radius = np.maximum(
        np.abs(2.0 * normalized_u - 1.0),
        np.abs(2.0 * normalized_v - 1.0),
    )
    fov_confidence = 1.0 - 0.35 * np.square(fov_radius)
    row_confidence = np.empty_like(calibrated)
    for patch_u in range(PATCH_COLS):
        column = calibrated[patch_u::PATCH_COLS]
        for patch_v in range(PATCH_ROWS):
            index = patch_v * PATCH_COLS + patch_u
            support_threshold = max(ACTIVE_SCORE, 0.65 * calibrated[index])
            row_support = int(np.count_nonzero(column >= support_threshold))
            row_confidence[index] = 0.65 + 0.35 * max(row_support - 1, 0) / max(
                PATCH_ROWS - 1, 1
            )
    below_layer = position[2] - projected_world[:, 2]
    ground_confidence = np.where(
        below_layer <= 0.5,
        1.0,
        np.clip(1.0 - (below_layer - 0.5) / 5.0, 0.25, 1.0),
    )
    semantic_confidence = np.clip(
        fov_confidence * row_confidence * ground_confidence, 0.05, 1.0
    )
    confidence_scores = calibrated * semantic_confidence

    ax.scatter(
        anchor_x, anchor_y, s=20 + 42 * calibrated,
        facecolors=ANCHOR, edgecolors="white", linewidths=0.65,
        alpha=0.42 + 0.50 * calibrated, zorder=6,
    )
    clipped_high = (calibrated >= ACTIVE_SCORE) & clipped_at_pixels
    ax.scatter(
        anchor_x[clipped_high], anchor_y[clipped_high], s=60,
        facecolors="none", edgecolors=RISK_HIGH, linewidths=1.2, zorder=7,
    )

    selected_world = marker_points(graph, "epic_selected_witness_path")
    selected_local = world_to_body(selected_world, position, orientation)
    witness_world = marker_points(graph, "epic_edge_witness_paths")
    witness_world = witness_world[: len(witness_world) // 2 * 2].reshape(-1, 2, 3)
    endpoint_distance = distance_to_polyline(
        witness_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2)
    semantic_distance = np.min(
        segment_point_distance(projected_world, witness_world), axis=1
    )
    centers_local = world_to_body(witness_world.mean(axis=1), position, orientation)
    candidate_mask = (
        ((np.max(endpoint_distance, axis=1) <= 5.0)
         | (semantic_distance <= RISK_RADIUS_M))
        & (centers_local[:, 0] >= -2.0) & (centers_local[:, 0] <= 40.0)
        & (np.abs(centers_local[:, 1]) <= 35.0)
        & (centers_local[:, 2] >= -12.0) & (centers_local[:, 2] <= 18.0)
    )
    candidate_world = witness_world[candidate_mask]
    candidate_local = world_to_body(
        candidate_world.reshape(-1, 3), position, orientation
    ).reshape(-1, 2, 3)
    candidate_exposure = segment_semantic_exposure(
        candidate_world, projected_world, confidence_scores
    )
    if len(candidate_world) > 72:
        order = np.argsort(candidate_exposure)
        keep = np.unique(np.concatenate((
            order[np.linspace(0, len(order) - 1, 56, dtype=int)],
            np.argsort(np.max(endpoint_distance[candidate_mask], axis=1))[:16],
        )))
        candidate_world = candidate_world[keep]
        candidate_local = candidate_local[keep]
        candidate_exposure = candidate_exposure[keep]

    exposure_vmax = max(float(np.max(candidate_exposure)), 1e-4)
    exposure_map = LinearSegmentedColormap.from_list(
        "graph_exposure", ["#AFC2CF", "#E6A33A", "#C43C39"]
    )
    exposure_norm = Normalize(0.0, exposure_vmax)
    ax.add_collection(LineCollection(
        projected_line_segments(projector, candidate_local),
        colors=exposure_map(exposure_norm(candidate_exposure)),
        linewidths=1.25, alpha=0.82, zorder=7,
    ))

    nodes_world, node_colors = marker_points_with_colors(
        graph, "epic_skeleton_nodes"
    )
    node_distance = distance_to_polyline(nodes_world, selected_world)
    node_semantic_distance = np.min(
        np.linalg.norm(
            nodes_world[:, None, :] - projected_world[None, :, :], axis=2
        ),
        axis=1,
    )
    nodes_local = world_to_body(nodes_world, position, orientation)
    node_mask = (
        ((node_distance <= 5.0) | (node_semantic_distance <= RISK_RADIUS_M))
        & (nodes_local[:, 0] >= -2.0) & (nodes_local[:, 0] <= 40.0)
        & (np.abs(nodes_local[:, 1]) <= 35.0)
        & (nodes_local[:, 2] >= -12.0) & (nodes_local[:, 2] <= 18.0)
    )
    node_indices = np.flatnonzero(node_mask)
    if len(node_indices) > 42:
        node_indices = node_indices[
            np.argsort(node_distance[node_indices])[:42]
        ]
    node_x, node_y, _ = projector(nodes_local[node_indices])
    visible_colors = (
        node_colors[node_indices, :3] if node_colors is not None else TOPOLOGY
    )
    ax.scatter(
        node_x, node_y, c=visible_colors, s=18,
        edgecolors="white", linewidths=0.5, zorder=9,
    )

    selected_x, selected_y, _ = projector(selected_local)
    ax.plot(selected_x, selected_y, color="white", lw=6.0,
            solid_capstyle="round", zorder=10)
    ax.plot(selected_x, selected_y, color=SELECTED, lw=3.8,
            solid_capstyle="round", zorder=11)
    vehicle_x, vehicle_y, _ = projector(np.zeros((1, 3)))
    camera_x, camera_y, _ = projector(CAMERA_TRANSLATION_FLU[None, :])
    for endpoint_x, endpoint_y, score in zip(anchor_x, anchor_y, calibrated):
        ax.plot(
            [camera_x[0], endpoint_x], [camera_y[0], endpoint_y],
            color=ANCHOR, lw=0.45 + 0.55 * float(score),
            alpha=0.16 + 0.44 * float(score),
            linestyle=(0, (3.0, 2.2)), zorder=5,
        )
    ax.scatter(camera_x, camera_y, marker="s", s=25, color=ANCHOR,
               edgecolor="white", linewidth=0.7, zorder=12)
    ax.scatter(vehicle_x, vehicle_y, marker="^", s=88, color=UAV,
               edgecolor="white", linewidth=1.0, zorder=13)

    local_goal = marker_pose_position(graph, "epic_local_goal", "epic_yopo_next_goal")
    frontier_goal = marker_pose_position(
        graph, "epic_frontier_goal", "epic_route_terminal"
    )
    if local_goal is not None:
        annotate_goal(
            ax, projector, world_to_body(local_goal[None, :], position, orientation)[0],
            "local_goal", LOCAL_GOAL, "o", (7, -18),
        )
    if frontier_goal is not None:
        annotate_goal(
            ax, projector, world_to_body(frontier_goal[None, :], position, orientation)[0],
            "frontier_goal", FRONTIER, "D", (7, 12),
        )

    ax.text(
        float(np.mean(plane_corners[:, 0])), float(np.max(plane_corners[:, 1])) + 0.025,
        "RGB + PEARL score", ha="center", va="bottom", fontsize=8.0,
        color=INK, fontweight="bold", bbox=LABEL_BOX, zorder=15,
    )
    ax.annotate(
        "raycast-valid graph", xy=(vehicle_x[0], vehicle_y[0]),
        xytext=(-2, -20), textcoords="offset points", ha="right",
        fontsize=7.4, color=INK, fontweight="bold", bbox=LABEL_BOX, zorder=15,
    )

    score_axis = ax.inset_axes([0.765, 0.82, 0.175, 0.025])
    score_bar = figure.colorbar(
        plt.cm.ScalarMappable(
            norm=Normalize(semantic_vmin, semantic_vmax),
            cmap=LinearSegmentedColormap.from_list(
                "pearl_bar", ["#F4C95D", "#ED7D3A", RISK_HIGH]
            ),
        ),
        cax=score_axis, orientation="horizontal",
        ticks=[semantic_vmin, semantic_vmax],
    )
    score_bar.outline.set_visible(False)
    score_bar.ax.tick_params(labelsize=5.7, length=1.2, pad=1)
    score_bar.ax.set_title("raw PEARL response", fontsize=5.9, color=INK, pad=1.5)

    exposure_axis = ax.inset_axes([0.055, 0.82, 0.175, 0.025])
    exposure_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=exposure_norm, cmap=exposure_map),
        cax=exposure_axis, orientation="horizontal",
        ticks=[0.0, exposure_vmax],
    )
    exposure_bar.outline.set_visible(False)
    exposure_bar.ax.tick_params(labelsize=5.7, length=1.2, pad=1)
    exposure_bar.ax.set_title(
        r"current-frame field $s c\,e^{-d^2/(2\sigma^2)}$",
        fontsize=5.9, color=INK, pad=1.5,
    )

    legend_handles = [
        Line2D([], [], color=ANCHOR, lw=1.1, linestyle=(0, (3.0, 2.2)),
               marker="o", markersize=3.8, markeredgecolor="white",
               label=r"15 fixed-$Z$ semantic rays"),
        Line2D([], [], color=RISK_HIGH, lw=0, marker="o", markersize=5.2,
               markerfacecolor="none", label="high response beyond depth support"),
        Line2D([], [], color="#AFC2CF", lw=1.5, marker="o", markersize=3.8,
               markeredgecolor="white", label="low-exposure witness"),
        Line2D([], [], color="#C43C39", lw=1.5, marker="o", markersize=3.8,
               markeredgecolor="white", label="high-exposure witness"),
        Line2D([], [], color=SELECTED, lw=3.2, label="selected witness"),
        Line2D([], [], color=LOCAL_GOAL, lw=0, marker="o", markersize=5,
               markeredgecolor="white", label="local_goal"),
        Line2D([], [], color=FRONTIER, lw=0, marker="D", markersize=5,
               markeredgecolor="white", label="frontier_goal"),
    ]
    ax.legend(
        handles=legend_handles, loc="lower center", bbox_to_anchor=(0.50, -0.075),
        ncol=4, fontsize=5.9, frameon=False, handlelength=1.7,
        columnspacing=0.85, borderaxespad=0.0,
    )

    graph_points = [selected_local, candidate_local.reshape(-1, 3)]
    if len(node_indices):
        graph_points.append(nodes_local[node_indices])
    graph_x, graph_y, _ = projector(np.vstack(graph_points))
    bounds_x = np.concatenate((plane_corners[:, 0], graph_x))
    bounds_y = np.concatenate((plane_corners[:, 1], graph_y))
    pad_x = 0.06 * (bounds_x.max() - bounds_x.min())
    pad_y = 0.09 * (bounds_y.max() - bounds_y.min())
    ax.set_xlim(bounds_x.min() - pad_x, bounds_x.max() + pad_x)
    ax.set_ylim(bounds_y.min() - pad_y, bounds_y.max() + pad_y)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_facecolor(BACKGROUND)
    for spine in ax.spines.values():
        spine.set_visible(False)


def semantic_confidences(
    pixels: np.ndarray,
    calibrated: np.ndarray,
    projected_world: np.ndarray,
    image_shape: tuple[int, int],
    graph_layer_z: float,
) -> np.ndarray:
    """Reproduce the deployed FOV, row-support, and ground confidences."""
    height, width = image_shape
    normalized_u = (pixels[:, 0] + 0.5) / width
    normalized_v = (pixels[:, 1] + 0.5) / height
    fov_radius = np.maximum(
        np.abs(2.0 * normalized_u - 1.0),
        np.abs(2.0 * normalized_v - 1.0),
    )
    fov_confidence = 1.0 - 0.35 * np.square(np.clip(fov_radius, 0.0, 1.0))

    row_confidence = np.empty_like(calibrated)
    for patch_u in range(PATCH_COLS):
        column = calibrated[patch_u::PATCH_COLS]
        for patch_v in range(PATCH_ROWS):
            index = patch_v * PATCH_COLS + patch_u
            support_threshold = max(ACTIVE_SCORE, 0.65 * calibrated[index])
            row_support = int(np.count_nonzero(column >= support_threshold))
            row_confidence[index] = 0.65 + 0.35 * max(row_support - 1, 0) / max(
                PATCH_ROWS - 1, 1
            )

    below_layer = graph_layer_z - projected_world[:, 2]
    ground_confidence = np.where(
        below_layer <= 0.5,
        1.0,
        np.clip(1.0 - (below_layer - 0.5) / 5.0, 0.25, 1.0),
    )
    return np.clip(fov_confidence * row_confidence * ground_confidence, 0.05, 1.0)


def draw_algorithm_panels(
    figure: plt.Figure,
    rgb: np.ndarray,
    semantic: np.ndarray,
    graph: list[dict],
    position: np.ndarray,
    orientation: list[float],
    pixels: np.ndarray,
    raw_scores: np.ndarray,
    calibrated: np.ndarray,
    projected_world: np.ndarray,
    clipped_at_pixels: np.ndarray,
) -> None:
    """Show image evidence and graph consequences without conflating rays."""
    image_ax = figure.add_axes([0.018, 0.18, 0.385, 0.75])
    graph_ax = figure.add_axes([0.425, 0.13, 0.56, 0.82])
    height, width = semantic.shape
    risk_map = LinearSegmentedColormap.from_list(
        "pearl_image", ["#F8D878", "#EE8A3A", RISK_HIGH]
    )
    raw_norm = Normalize(float(np.min(semantic)), float(np.max(semantic)))

    display_rgb = Image.fromarray(np.asarray(rgb, dtype=np.uint8)).resize(
        (width * 7, height * 7), Image.Resampling.LANCZOS
    ).filter(ImageFilter.UnsharpMask(radius=1.0, percent=80, threshold=3))
    image_ax.imshow(
        display_rgb, extent=(0, width, height, 0), interpolation="nearest", zorder=0
    )
    baseline_index = int(math.floor(0.25 * (len(raw_scores) - 1)))
    frame_background = min(0.25, max(0.0, float(np.sort(raw_scores)[baseline_index])))
    overlay_threshold = frame_background + ACTIVE_SCORE * (1.0 - frame_background)
    calibrated_image = np.clip(
        (semantic - frame_background) / max(1e-3, 1.0 - frame_background),
        0.0, 1.0,
    )
    semantic_rgba = risk_map(raw_norm(semantic))
    semantic_rgba[..., 3] = np.where(
        semantic >= overlay_threshold,
        0.16 + 0.46 * calibrated_image,
        0.0,
    )
    image_ax.imshow(
        semantic_rgba, extent=(0, width, height, 0),
        interpolation="nearest", zorder=1,
    )
    for column in range(PATCH_COLS + 1):
        x = column * width / PATCH_COLS
        image_ax.plot([x, x], [0, height], color="white", lw=0.75,
                      alpha=0.78, zorder=2)
    for row in range(PATCH_ROWS + 1):
        y = row * height / PATCH_ROWS
        image_ax.plot([0, width], [y, y], color="white", lw=0.75,
                      alpha=0.78, zorder=2)
    image_ax.scatter(
        pixels[:, 0], pixels[:, 1], c=raw_scores, cmap=risk_map, norm=raw_norm,
        s=29, edgecolors="white", linewidths=0.8, zorder=4,
    )

    clipped_high = (calibrated >= ACTIVE_SCORE) & clipped_at_pixels
    highlighted_indices = np.flatnonzero(clipped_high)
    image_ax.scatter(
        pixels[clipped_high, 0], pixels[clipped_high, 1], s=100,
        facecolors="none", edgecolors=RISK_HIGH, linewidths=1.45, zorder=5,
    )
    image_label_offsets = ((-15, -12), (15, -12), (-16, 14), (17, 14))
    for label_index, point_index in enumerate(highlighted_indices, 1):
        pixel_u, pixel_v = pixels[point_index]
        image_ax.annotate(
            str(label_index), xy=(pixel_u, pixel_v),
            xytext=image_label_offsets[label_index - 1],
            textcoords="offset points",
            ha="center", va="center", color=RISK_HIGH, fontsize=7.4,
            fontweight="bold", bbox=dict(
                boxstyle="circle,pad=0.17", facecolor="white",
                edgecolor=RISK_HIGH, linewidth=0.85,
            ), arrowprops=dict(arrowstyle="-", color=RISK_HIGH, lw=0.55),
            zorder=6,
        )
    image_ax.text(
        0.0, 1.035, "(a) Image evidence", transform=image_ax.transAxes,
        ha="left", va="bottom", fontsize=8.6, color=INK, fontweight="bold",
    )
    image_ax.text(
        1.0, 1.035, r"RGB + PEARL, $5\!\times\!3$ maxima",
        transform=image_ax.transAxes, ha="right", va="bottom",
        fontsize=7.4, color=INK,
    )
    image_ax.set_xlim(0, width)
    image_ax.set_ylim(height, 0)
    image_ax.set_aspect("equal", adjustable="box")
    image_ax.set_xticks([])
    image_ax.set_yticks([])
    for spine in image_ax.spines.values():
        spine.set_visible(False)

    selected_world = marker_points(graph, "epic_selected_witness_path")
    graph_layer_z = float(np.median(selected_world[:, 2]))
    confidence = semantic_confidences(
        pixels, calibrated, projected_world, semantic.shape, graph_layer_z
    )
    endpoint_strength = calibrated * confidence

    def top_down(points_world: np.ndarray) -> np.ndarray:
        local = world_to_body(points_world, position, orientation)
        return np.column_stack((-local[:, 1], local[:, 0]))

    witness_points = marker_points(graph, "epic_edge_witness_paths")
    witness_world = witness_points[: len(witness_points) // 2 * 2].reshape(-1, 2, 3)
    selected_distance = distance_to_polyline(
        witness_world.reshape(-1, 3), selected_world
    ).reshape(-1, 2).min(axis=1)
    endpoint_distance = segment_point_distance(projected_world, witness_world).min(axis=1)
    witness_local = world_to_body(
        witness_world.mean(axis=1), position, orientation
    )
    candidate_mask = (
        ((selected_distance <= 5.0) | (endpoint_distance <= RISK_RADIUS_M))
        & (witness_local[:, 0] >= -4.0) & (witness_local[:, 0] <= 39.0)
        & (np.abs(witness_local[:, 1]) <= 36.0)
    )
    candidate_world = witness_world[candidate_mask]
    exposure = segment_semantic_exposure(
        candidate_world, projected_world, endpoint_strength
    )
    exposure_vmax = max(float(np.max(exposure)), 0.05)
    exposure_map = LinearSegmentedColormap.from_list(
        "graph_exposure", ["#C7D2D8", "#E6A33A", RISK_HIGH]
    )
    exposure_norm = Normalize(0.0, exposure_vmax)
    graph_segments = [top_down(segment) for segment in candidate_world]
    graph_ax.add_collection(LineCollection(
        graph_segments, colors=exposure_map(exposure_norm(exposure)),
        linewidths=1.05, alpha=0.82, zorder=2,
    ))

    nodes_world, _ = marker_points_with_colors(graph, "epic_skeleton_nodes")
    node_selected_distance = distance_to_polyline(nodes_world, selected_world)
    node_endpoint_distance = np.linalg.norm(
        nodes_world[:, None, :] - projected_world[None, :, :], axis=2
    ).min(axis=1)
    nodes_local = world_to_body(nodes_world, position, orientation)
    node_mask = (
        ((node_selected_distance <= 5.0) | (node_endpoint_distance <= RISK_RADIUS_M))
        & (nodes_local[:, 0] >= -4.0) & (nodes_local[:, 0] <= 39.0)
        & (np.abs(nodes_local[:, 1]) <= 36.0)
    )
    visible_nodes = nodes_world[node_mask]
    visible_node_xy = top_down(visible_nodes)
    node_exposure = np.zeros(len(visible_nodes))
    if len(visible_nodes):
        node_distances = np.linalg.norm(
            visible_nodes[:, None, :] - projected_world[None, :, :], axis=2
        )
        node_fields = endpoint_strength[None, :] * np.exp(
            -(node_distances ** 2) / (2.0 * RISK_SIGMA_M ** 2)
        )
        node_fields[node_distances > RISK_RADIUS_M] = 0.0
        node_exposure = np.max(node_fields, axis=1)
    graph_ax.scatter(
        visible_node_xy[:, 0], visible_node_xy[:, 1], c=node_exposure,
        cmap=exposure_map, norm=exposure_norm, s=15,
        edgecolors="white", linewidths=0.45, alpha=0.96, zorder=4,
    )

    endpoint_xy = top_down(projected_world)
    endpoint_label_offsets = ((-10, 11), (0, 15), (-13, -11), (13, -11))
    for label_index, point_index in enumerate(highlighted_indices, 1):
        center = endpoint_xy[point_index]
        graph_ax.add_patch(Circle(
            center, RISK_RADIUS_M, facecolor=RISK_HIGH, edgecolor=RISK_HIGH,
            linewidth=0.8, linestyle=(0, (3.0, 2.0)), alpha=0.075, zorder=1,
        ))
        graph_ax.scatter(
            center[0], center[1], s=68, marker="X", color=ANCHOR,
            edgecolor="white", linewidth=0.8, zorder=7,
        )
        graph_ax.annotate(
            str(label_index), xy=center,
            xytext=endpoint_label_offsets[label_index - 1],
            textcoords="offset points",
            ha="center", va="center", color=RISK_HIGH, fontsize=7.2,
            fontweight="bold", bbox=dict(
                boxstyle="circle,pad=0.16", facecolor="white",
                edgecolor=RISK_HIGH, linewidth=0.8,
            ), arrowprops=dict(arrowstyle="-", color=RISK_HIGH, lw=0.55),
            zorder=8,
        )

    selected_xy = top_down(selected_world)
    graph_ax.plot(selected_xy[:, 0], selected_xy[:, 1], color="white", lw=6.0,
                  solid_capstyle="round", zorder=5)
    graph_ax.plot(selected_xy[:, 0], selected_xy[:, 1], color=SELECTED, lw=3.4,
                  solid_capstyle="round", zorder=6)
    if len(selected_xy) >= 2:
        graph_ax.annotate(
            "", xy=selected_xy[-1], xytext=selected_xy[-2],
            arrowprops=dict(arrowstyle="-|>", color=SELECTED, lw=1.2,
                            shrinkA=0, shrinkB=0), zorder=7,
        )

    vehicle_xy = top_down(position[None, :])[0]
    graph_ax.scatter(
        vehicle_xy[0], vehicle_xy[1], marker="^", s=86, color=UAV,
        edgecolor="white", linewidth=1.0, zorder=8,
    )

    def top_down_goal(
        point_world: np.ndarray | None,
        label: str,
        color: str,
        marker: str,
        offset: tuple[float, float],
    ) -> None:
        if point_world is None:
            return
        point = top_down(point_world[None, :])[0]
        graph_ax.scatter(
            point[0], point[1], marker=marker, s=82, color=color,
            edgecolor="white", linewidth=1.0, zorder=9,
        )
        graph_ax.annotate(
            label, xy=point, xytext=offset, textcoords="offset points",
            fontsize=7.5, color=color, fontweight="bold", bbox=LABEL_BOX,
            zorder=10,
        )

    top_down_goal(
        marker_pose_position(graph, "epic_local_goal", "epic_yopo_next_goal"),
        "local_goal", LOCAL_GOAL, "o", (7, -16),
    )
    top_down_goal(
        marker_pose_position(graph, "epic_frontier_goal", "epic_route_terminal"),
        "frontier_goal", FRONTIER, "D", (7, 8),
    )
    graph_ax.text(
        0.0, 1.015, "(b) Graph decision", transform=graph_ax.transAxes,
        ha="left", va="bottom", fontsize=8.6, color=INK, fontweight="bold",
    )
    graph_ax.text(
        1.0, 1.015, "first logged route update, top-down body frame",
        transform=graph_ax.transAxes,
        ha="right", va="bottom", fontsize=7.4, color=INK,
    )
    graph_ax.annotate(
        "forward", xy=(0.08, 0.20), xytext=(0.08, 0.08),
        xycoords="axes fraction", textcoords="axes fraction",
        arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9),
        ha="center", va="center", fontsize=6.6, color=INK,
    )

    graph_ax.set_xlim(-24.0, 36.0)
    graph_ax.set_ylim(-4.0, 39.0)
    graph_ax.set_aspect("equal", adjustable="box")
    graph_ax.set_xticks([])
    graph_ax.set_yticks([])
    graph_ax.set_facecolor(BACKGROUND)
    for spine in graph_ax.spines.values():
        spine.set_visible(False)

    pearl_axis = figure.add_axes([0.055, 0.095, 0.27, 0.018])
    pearl_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=raw_norm, cmap=risk_map),
        cax=pearl_axis, orientation="horizontal",
        ticks=[float(np.min(semantic)), float(np.max(semantic))],
    )
    pearl_bar.outline.set_visible(False)
    pearl_bar.ax.tick_params(labelsize=5.8, length=1.0, pad=1)
    pearl_bar.ax.set_title("raw PEARL response", fontsize=6.2, color=INK, pad=1.5)

    exposure_axis = figure.add_axes([0.745, 0.055, 0.19, 0.018])
    exposure_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=exposure_norm, cmap=exposure_map),
        cax=exposure_axis, orientation="horizontal", ticks=[0.0, exposure_vmax],
    )
    exposure_bar.outline.set_visible(False)
    exposure_bar.ax.tick_params(labelsize=5.8, length=1.0, pad=1)
    exposure_bar.ax.set_title(
        "current-frame witness exposure", fontsize=6.2, color=INK, pad=1.5
    )

    legend_handles = [
        Line2D([], [], color=RISK_HIGH, lw=0, marker="o", markersize=5.5,
               markerfacecolor="none", label="high response / no depth return"),
        Line2D([], [], color=ANCHOR, lw=0, marker="X", markersize=5.2,
               markeredgecolor="white", label=r"fixed-$Z$ endpoint"),
        Line2D([], [], color=RISK_HIGH, lw=0.9, linestyle=(0, (3.0, 2.0)),
               label="5 m 3-D cutoff (plan view)"),
        Line2D([], [], color="#C7D2D8", lw=1.4, marker="o", markersize=3.5,
               markeredgecolor="white", label="raycast-valid graph"),
        Line2D([], [], color=SELECTED, lw=3.0, label="selected witness"),
        Line2D([], [], color=LOCAL_GOAL, lw=0, marker="o", markersize=5,
               markeredgecolor="white", label="local_goal"),
        Line2D([], [], color=FRONTIER, lw=0, marker="D", markersize=5,
               markeredgecolor="white", label="frontier_goal"),
    ]
    figure.legend(
        handles=legend_handles, loc="lower center", bbox_to_anchor=(0.50, 0.002),
        ncol=7, fontsize=5.8, frameon=False, handlelength=1.55,
        columnspacing=0.78, borderaxespad=0.0,
    )


def main() -> None:
    args = parse_args()
    session = args.session.resolve()
    events = load_events(session)
    rgb_event = exact_event(events, "rgb", args.rgb)
    stamp_ns = int(rgb_event["stamp_ns"])
    semantic_event = nearest_event(events["semantic"], stamp_ns)
    depth_event = nearest_event(events["depth"], stamp_ns)
    graph_event = exact_event(events, "graph", args.graph)
    odom_event = nearest_event(events["odom"], stamp_ns)

    sync_events = (semantic_event, depth_event, odom_event)
    max_sync_ms = max(
        abs(int(event["stamp_ns"]) - stamp_ns) for event in sync_events
    ) / 1e6
    if max_sync_ms > 50.0:
        raise ValueError(
            f"selected frame is not synchronized: max delta {max_sync_ms:.1f} ms"
        )
    graph_delay_ms = (int(graph_event["stamp_ns"]) - stamp_ns) / 1e6
    if graph_delay_ms < 0.0 or graph_delay_ms > 1000.0:
        raise ValueError(
            f"graph update is not a causal near-term state: delay {graph_delay_ms:.1f} ms"
        )

    rgb = np.asarray(Image.open(session / rgb_event["file"]).convert("RGB"))
    semantic = np.clip(read_scalar_image(session, semantic_event), 0.0, 1.0)
    depth = read_scalar_image(session, depth_event)
    pose = odom_event["data"]
    position = np.asarray(pose["position"], dtype=float)
    pixels, raw_scores, calibrated, projected_world = max_pool_and_project(
        semantic, position, pose["orientation"]
    )
    clipped_at_pixels = np.asarray([
        depth[pixel_v, pixel_u] >= DEPTH_CLIP_M for pixel_u, pixel_v in pixels
    ])
    graph = load_graph(session, graph_event)
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 8,
        "figure.facecolor": BACKGROUND,
        "savefig.facecolor": BACKGROUND,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    figure = plt.figure(figsize=(9.4, 4.55), facecolor=BACKGROUND)
    draw_algorithm_panels(
        figure, rgb, semantic, graph, position, pose["orientation"],
        pixels, raw_scores, calibrated, projected_world, clipped_at_pixels,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output.with_suffix(".pdf"), dpi=300,
                   bbox_inches="tight", pad_inches=0.04)
    figure.savefig(args.output.with_suffix(".png"), dpi=300,
                   bbox_inches="tight", pad_inches=0.04)
    plt.close(figure)
    active_count = int(np.count_nonzero(
        (calibrated >= ACTIVE_SCORE) & clipped_at_pixels
    ))
    print(
        f"wrote {args.output.with_suffix('.pdf')} and .png from {session.name}; "
        f"frame={rgb_event['file']}; semantic={semantic_event['file']}; "
        f"depth={depth_event['file']}; graph={graph_event['file']}; "
        f"sensor_sync={max_sync_ms:.3f} ms; graph_delay={graph_delay_ms:.3f} ms; "
        f"active_depth-clipped_rays={active_count}"
    )


if __name__ == "__main__":
    main()
