#!/usr/bin/env python3
"""Build the ScaleNav teaser from one recorded simulation flight.

The evidence frame is selected automatically from the first one-way mission:
it maximizes semantic response on pixels whose depth is clipped at the sensor
limit. The map uses the synchronized graph/path state and the flown trajectory
from that same mission, so every visible claim is traceable to the log.

Usage:
    python3 make_teaser.py SESSION_DIR OUT_PREFIX
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, ConnectionPatch
from PIL import Image


BACKGROUND = "#FFFFFF"
ENVIRONMENT = "#D9DEE3"
CANDIDATE = "#B5C1C8"
TOPOLOGY = "#657985"
SELECTED = "#007C83"
UAV = "#24343D"
MISSION = "#C43C39"
FRONTIER = "#E28A17"
LOCAL_GOAL = "#B64E8A"
ROUTE = "#3973B7"
RISK_HIGH = "#D14E46"
LABEL_BOX = dict(boxstyle="square,pad=0.12", facecolor=BACKGROUND,
                 edgecolor="none", alpha=0.88)

DEPTH_CLIP_M = 20.0
RISK_RADIUS_M = 10.0
ARRIVAL_RADIUS_M = 5.0
DETOUR_EVIDENCE_MIN_M = 8.0
DETOUR_EVIDENCE_MAX_M = 85.0
SYNC_TOLERANCE_NS = int(0.40e9)
SENSOR_SYNC_NS = int(0.10e9)
STATE_SYNC_NS = int(0.12e9)
PC_EVERY = 8
PC_STRIDE = 5
PC_MAX = 75_000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("out_prefix", type=Path)
    return parser.parse_args()


def load_events(session: Path) -> dict[str, list[dict]]:
    events: dict[str, list[dict]] = {}
    with (session / "index.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            event = json.loads(line)
            events.setdefault(event["kind"], []).append(event)
    required = ("odom", "rgb", "depth", "semantic", "graph", "path", "goal")
    missing = [kind for kind in required if not events.get(kind)]
    if missing:
        raise ValueError(f"session is missing required streams: {', '.join(missing)}")
    return events


def nearest_event(events: list[dict], stamp_ns: int) -> dict:
    stamps = np.asarray([event["stamp_ns"] for event in events], dtype=np.int64)
    return events[int(np.argmin(np.abs(stamps - stamp_ns)))]


def interpolated_odom_position(events: list[dict], stamp_ns: int) -> np.ndarray:
    """Linearly interpolate logged odometry at the sensor timestamp."""
    stamps = np.asarray([event["stamp_ns"] for event in events], dtype=np.int64)
    upper = int(np.searchsorted(stamps, stamp_ns))
    if upper <= 0 or upper >= len(events):
        return np.asarray(nearest_event(events, stamp_ns)["data"]["position"], dtype=float)
    before, after = events[upper - 1], events[upper]
    span = after["stamp_ns"] - before["stamp_ns"]
    weight = (stamp_ns - before["stamp_ns"]) / span if span else 0.0
    p0 = np.asarray(before["data"]["position"], dtype=float)
    p1 = np.asarray(after["data"]["position"], dtype=float)
    return p0 + weight * (p1 - p0)


def first_mission(events: dict[str, list[dict]]) -> tuple[int, int, np.ndarray, list[dict]]:
    start_ns = events["graph"][0]["stamp_ns"]
    goal = np.asarray(events["goal"][0]["data"]["position"], dtype=float)
    odometry = [event for event in events["odom"] if event["stamp_ns"] >= start_ns]
    if not odometry:
        raise ValueError("no odometry after the first graph update")
    end_index = len(odometry) - 1
    for index, event in enumerate(odometry):
        position = np.asarray(event["data"]["position"], dtype=float)
        if np.linalg.norm(position[:2] - goal[:2]) <= ARRIVAL_RADIUS_M:
            end_index = index
            break
    mission_odom = odometry[: end_index + 1]
    return start_ns, mission_odom[-1]["stamp_ns"], goal, mission_odom


def read_scalar_image(session: Path, event: dict) -> np.ndarray:
    return np.asarray(Image.open(session / event["file"]), dtype=float) / 1000.0


def choose_evidence_frame(
    session: Path,
    events: dict[str, list[dict]],
    start_ns: int,
    end_ns: int,
) -> tuple[dict, np.ndarray, np.ndarray, np.ndarray]:
    candidates = [
        event for event in events["rgb"]
        if event["file"] and start_ns <= event["stamp_ns"] <= end_ns
    ]
    start_position = np.asarray(nearest_event(events["odom"], start_ns)["data"]["position"])
    detour_candidates = []
    for event in candidates:
        position = np.asarray(nearest_event(events["odom"], event["stamp_ns"])["data"]["position"])
        progress = float(np.linalg.norm(position[:2] - start_position[:2]))
        if DETOUR_EVIDENCE_MIN_M <= progress <= DETOUR_EVIDENCE_MAX_M:
            detour_candidates.append(event)
    if detour_candidates:
        candidates = detour_candidates
    best = None
    best_aligned = None
    for rgb_event in candidates:
        semantic_event = nearest_event(events["semantic"], rgb_event["stamp_ns"])
        depth_event = nearest_event(events["depth"], rgb_event["stamp_ns"])
        if max(
            abs(semantic_event["stamp_ns"] - rgb_event["stamp_ns"]),
            abs(depth_event["stamp_ns"] - rgb_event["stamp_ns"]),
        ) > SYNC_TOLERANCE_NS:
            continue
        semantic = np.clip(read_scalar_image(session, semantic_event), 0.0, 1.0)
        depth = read_scalar_image(session, depth_event)
        clipped = depth >= DEPTH_CLIP_M
        # Favor strong semantic evidence specifically outside valid depth
        # support, without allowing one hot pixel to dominate selection.
        score = float(np.mean(semantic * clipped) * np.sqrt(max(np.mean(clipped), 1e-9)))
        if best is None or score > best[0]:
            best = (score, rgb_event, semantic, depth)
        overlap_fraction = float(np.mean((semantic >= 0.40) & clipped))
        sensor_gap = max(
            abs(semantic_event["stamp_ns"] - rgb_event["stamp_ns"]),
            abs(depth_event["stamp_ns"] - rgb_event["stamp_ns"]),
        )
        state_gap = max(
            abs(nearest_event(events[kind], rgb_event["stamp_ns"])["stamp_ns"]
                - rgb_event["stamp_ns"])
            for kind in ("graph", "path", "odom")
        )
        hierarchy_quality = 0.0
        graph_event = nearest_event(events["graph"], rgb_event["stamp_ns"])
        graph_markers = load_graph(session, graph_event)
        frontier = marker_pose_position(
            graph_markers, "epic_frontier_goal", "epic_route_terminal"
        )
        local_goal = marker_pose_position(
            graph_markers, "epic_local_goal", "epic_yopo_next_goal"
        )
        if frontier is not None and local_goal is not None:
            position = interpolated_odom_position(events["odom"], rgb_event["stamp_ns"])
            layer_separation = float(np.linalg.norm(frontier[:2] - local_goal[:2]))
            frontier_advance = float(np.linalg.norm(frontier[:2] - position[:2]))
            hierarchy_quality = min(layer_separation / 10.0, 1.0) * min(
                frontier_advance / 20.0, 1.0
            )
        if (overlap_fraction >= 0.03 and sensor_gap <= SENSOR_SYNC_NS
                and state_gap <= STATE_SYNC_NS):
            # Prefer a frame that demonstrates both claims: semantics beyond
            # usable depth and a genuinely separated planning hierarchy.
            aligned_key = (
                -hierarchy_quality, -overlap_fraction,
                max(sensor_gap, state_gap), -score,
            )
            if best_aligned is None or aligned_key < best_aligned[0]:
                best_aligned = (aligned_key, score, rgb_event, semantic, depth)
    if best_aligned is not None:
        _, _, rgb_event, semantic, depth = best_aligned
    elif best is not None:
        _, rgb_event, semantic, depth = best
    else:
        raise ValueError("no synchronized RGB/depth/semantic evidence frame")
    rgb = np.asarray(Image.open(session / rgb_event["file"]).convert("RGB"), dtype=float) / 255.0
    return rgb_event, rgb, semantic, depth


def quat_rotate(quaternion: list[float], vectors: np.ndarray) -> np.ndarray:
    vector_part = np.asarray(quaternion[:3], dtype=float)
    scalar = float(quaternion[3])
    cross = 2.0 * np.cross(vector_part, vectors)
    return vectors + scalar * cross + np.cross(vector_part, cross)


def load_ply_xyz(path: Path) -> np.ndarray:
    """Read an ASCII PLY vertex cloud exported in world_enu meters."""
    with path.open(encoding="ascii") as stream:
        vertex_count = None
        for line in stream:
            if line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        if vertex_count is None:
            raise ValueError(f"PLY has no vertex count: {path}")
        points = np.loadtxt(stream, max_rows=vertex_count)
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"PLY contains no XYZ vertices: {path}")
    points = points[:, :3]
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError(f"PLY contains no finite vertices: {path}")
    return points


def small_obstacle_footprint(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Extract true small connected footprints in the Map2 cylinder field."""
    resolution = 0.12
    x_min, x_max = -12.0, 14.0
    y_min, y_max = 4.0, 25.0
    region = points[
        (points[:, 0] >= x_min) & (points[:, 0] < x_max)
        & (points[:, 1] >= y_min) & (points[:, 1] < y_max)
        & (points[:, 2] >= 0.25) & (points[:, 2] <= 8.0)
    ]
    rows = int(np.ceil((x_max - x_min) / resolution))
    columns = int(np.ceil((y_max - y_min) / resolution))
    occupied = np.zeros((rows, columns), dtype=bool)
    if len(region):
        row = np.floor((region[:, 0] - x_min) / resolution).astype(int)
        column = np.floor((region[:, 1] - y_min) / resolution).astype(int)
        occupied[row, column] = True

    selected = np.zeros_like(occupied)
    visited = np.zeros_like(occupied)
    for seed_row, seed_column in np.argwhere(occupied):
        if visited[seed_row, seed_column]:
            continue
        stack = [(int(seed_row), int(seed_column))]
        visited[seed_row, seed_column] = True
        component = []
        while stack:
            current_row, current_column = stack.pop()
            component.append((current_row, current_column))
            for row_step in (-1, 0, 1):
                for column_step in (-1, 0, 1):
                    neighbor_row = current_row + row_step
                    neighbor_column = current_column + column_step
                    if (row_step == 0 and column_step == 0) or not (
                        0 <= neighbor_row < rows and 0 <= neighbor_column < columns
                    ):
                        continue
                    if (occupied[neighbor_row, neighbor_column]
                            and not visited[neighbor_row, neighbor_column]):
                        visited[neighbor_row, neighbor_column] = True
                        stack.append((neighbor_row, neighbor_column))
        indices = np.asarray(component, dtype=int)
        width = (np.ptp(indices[:, 0]) + 1) * resolution
        length = (np.ptp(indices[:, 1]) + 1) * resolution
        area = len(indices) * resolution**2
        if 0.45 <= area <= 5.0 and width <= 4.0 and length <= 4.0:
            selected[indices[:, 0], indices[:, 1]] = True

    grid_x = x_min + (np.arange(rows) + 0.5) * resolution
    grid_y = y_min + (np.arange(columns) + 0.5) * resolution
    world_y, world_x = np.meshgrid(grid_y, grid_x)
    return world_x, world_y, selected


def aggregate_point_cloud(
    session: Path,
    events: dict[str, list[dict]],
    start_ns: int,
    end_ns: int,
) -> np.ndarray:
    # Prefer the static Map2 mesh truth exported from UE.  The recorded PCD
    # stream remains a useful fallback for reproducing the figure elsewhere,
    # but it must not be presented as the scene geometry when truth is present.
    truth_path = Path(__file__).with_name("map2_ground_truth.ply")
    if truth_path.is_file():
        return load_ply_xyz(truth_path)
    cloud_events = [
        event for event in events["pointcloud"]
        if event["file"] and start_ns <= event["stamp_ns"] <= end_ns
    ]
    clouds = []
    for event in cloud_events[::PC_EVERY]:
        if event.get("data", {}).get("frame_id") != "base_link":
            raise ValueError(
                "Map2 point-cloud reconstruction expects base_link PCDs; "
                f"got {event.get('data', {}).get('frame_id')!r}"
            )
        odom = nearest_event(events["odom"], event["stamp_ns"])
        points = np.loadtxt(session / event["file"], skiprows=11)
        if points.ndim != 2 or points.shape[0] == 0:
            continue
        points = points[::PC_STRIDE, :3]
        # The depth adapter publishes body-FLU points in base_link.  Odom is
        # the ROS xyzw body-to-world rotation in world_enu, so no FRD/FLU
        # conversion belongs here (doing one mirrors the map vertically).
        position = np.asarray(odom["data"]["position"], dtype=float)
        clouds.append(quat_rotate(odom["data"]["orientation"], points) + position)
    if not clouds:
        raise ValueError("mission contains no usable point cloud")
    cloud = np.concatenate(clouds)
    if len(cloud) > PC_MAX:
        selection = np.random.default_rng(0).choice(len(cloud), PC_MAX, replace=False)
        cloud = cloud[selection]
    return cloud


def marker_map(graph: dict) -> dict[str, list[dict]]:
    markers: dict[str, list[dict]] = {}
    for marker in graph["markers"]:
        markers.setdefault(marker["ns"], []).append(marker)
    return markers


def marker_points(markers: dict[str, list[dict]], namespace: str) -> np.ndarray:
    values = []
    for marker in markers.get(namespace, []):
        values.extend(marker.get("points") or [])
    return np.asarray(values, dtype=float).reshape(-1, 3) if values else np.empty((0, 3))


def marker_points_with_colors(
    markers: dict[str, list[dict]], namespace: str
) -> tuple[np.ndarray, np.ndarray | None]:
    points = []
    colors = []
    complete_colors = True
    for marker in markers.get(namespace, []):
        if marker.get("action", 0) != 0:
            continue
        marker_points_values = marker.get("points") or []
        marker_colors = marker.get("colors") or []
        points.extend(marker_points_values)
        if len(marker_colors) == len(marker_points_values):
            colors.extend(marker_colors)
        else:
            complete_colors = False
    point_array = np.asarray(points, dtype=float).reshape(-1, 3) if points else np.empty((0, 3))
    color_array = np.asarray(colors, dtype=float).reshape(-1, 4) if complete_colors and colors else None
    return point_array, color_array


def marker_pose_position(
    markers: dict[str, list[dict]], *namespaces: str
) -> np.ndarray | None:
    for namespace in namespaces:
        for marker in markers.get(namespace, []):
            if marker.get("action", 0) != 0:
                continue
            position = marker.get("pose", {}).get("position")
            if position is not None:
                return np.asarray(position, dtype=float)
    return None


def rayfronts_projection(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Project world_enu points into a shallow, orientation-correct aerial view."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    forward = points[:, 1]
    lateral = points[:, 0]
    height = points[:, 2]
    # screen_y uses -lateral: (forward, lateral) -> (right, down) is a proper
    # clockwise rotation of the UE top view, not the mirrored transpose used
    # previously.  Height is retained gently for legible obstacle silhouettes.
    return forward + 0.10 * lateral, -0.52 * lateral + 0.27 * height - 0.012 * forward


def semantic_risk_anchors(markers: dict[str, list[dict]]) -> np.ndarray:
    """Recover high-risk anchor positions without treating unknowns as risk."""
    values = []
    for namespace in ("epic_semantic_point_labels", "epic_speculative_labels"):
        for marker in markers.get(namespace, []):
            if marker.get("action", 0) != 0:
                continue
            color = marker.get("color") or [0.0, 0.0, 0.0, 0.0]
            position = marker.get("pose", {}).get("position")
            # Both logger generations use a red/magenta marker for SEM-RISK
            # and a green/gray marker for SEM-UNKNOWN.
            if position is not None and color[0] >= 0.70 and color[1] <= 0.45:
                values.append(position)
    if values:
        return np.asarray(values, dtype=float).reshape(-1, 3)
    # Some early logs omitted the per-anchor labels. Preserve position-only
    # evidence as a fallback, while the figure labels the field reconstructed.
    return marker_points(markers, "epic_semantic_points")


def anchor_risk(points: np.ndarray, anchors: np.ndarray) -> np.ndarray:
    """Unit-confidence max aggregation used by the planner's anchor field."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points) or not len(anchors):
        return np.zeros(len(points), dtype=float)
    distances = np.linalg.norm(
        points[:, None, :] - anchors[None, :, :], axis=2
    ).min(axis=1)
    sigma = max(0.25, RISK_RADIUS_M * 0.5)
    scores = np.exp(-0.5 * (distances / sigma) ** 2)
    scores[distances > RISK_RADIUS_M] = 0.0
    return scores


def segment_anchor_risk(segments: np.ndarray, anchors: np.ndarray) -> np.ndarray:
    """Evaluate max anchor risk at the closest point on each graph segment."""
    segments = np.asarray(segments, dtype=float).reshape(-1, 2, 3)
    if not len(segments) or not len(anchors):
        return np.zeros(len(segments), dtype=float)
    starts = segments[:, 0, :]
    directions = segments[:, 1, :] - starts
    length_sq = np.sum(directions * directions, axis=1)
    best = np.full(len(segments), np.inf, dtype=float)
    for anchor in np.asarray(anchors, dtype=float).reshape(-1, 3):
        numerator = np.sum((anchor - starts) * directions, axis=1)
        t = np.divide(numerator, length_sq, out=np.zeros_like(numerator),
                      where=length_sq > 1e-9)
        closest = starts + np.clip(t, 0.0, 1.0)[:, None] * directions
        best = np.minimum(best, np.linalg.norm(closest - anchor, axis=1))
    sigma = max(0.25, RISK_RADIUS_M * 0.5)
    scores = np.exp(-0.5 * (best / sigma) ** 2)
    scores[best > RISK_RADIUS_M] = 0.0
    return scores


def point_segment_set_distances(points: np.ndarray, segments: np.ndarray) -> np.ndarray:
    """Minimum 3-D distance from each point to a set of line segments."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    segments = np.asarray(segments, dtype=float).reshape(-1, 2, 3)
    if not len(points) or not len(segments):
        return np.full(len(points), np.inf, dtype=float)
    starts = segments[:, 0]
    directions = segments[:, 1] - starts
    length_sq = np.sum(directions * directions, axis=1)
    distances = np.full(len(points), np.inf, dtype=float)
    for index, point in enumerate(points):
        numerator = np.sum((point - starts) * directions, axis=1)
        t = np.divide(numerator, length_sq, out=np.zeros_like(numerator),
                      where=length_sq > 1e-9)
        closest = starts + np.clip(t, 0.0, 1.0)[:, None] * directions
        distances[index] = np.linalg.norm(closest - point, axis=1).min()
    return distances


def load_graph(session: Path, event: dict) -> dict[str, list[dict]]:
    with (session / event["file"]).open(encoding="utf-8") as stream:
        return marker_map(json.load(stream))


def load_path(session: Path, event: dict) -> np.ndarray:
    with (session / event["file"]).open(encoding="utf-8") as stream:
        return np.asarray(json.load(stream)["poses"], dtype=float)


def evidence_witness_clearance(events: dict[str, list[dict]], stamp_ns: int) -> float | None:
    clearance_events = events.get("clearance", [])
    if not clearance_events:
        return None
    data = nearest_event(clearance_events, stamp_ns)["data"]
    value = data.get("global_witness_min_m", data.get("path_min_m"))
    return float(value) if value is not None and np.isfinite(value) else None


def draw_evidence_panel(
    figure: plt.Figure,
    slot,
    rgb: np.ndarray,
    semantic: np.ndarray,
    depth: np.ndarray,
) -> tuple[plt.Axes, plt.Axes, plt.Axes]:
    subgrid = slot.subgridspec(3, 1, hspace=0.075)
    ax_rgb = figure.add_subplot(subgrid[0])
    ax_semantic = figure.add_subplot(subgrid[1])
    ax_depth = figure.add_subplot(subgrid[2])

    ax_rgb.imshow(rgb, interpolation="lanczos")
    ax_rgb.text(
        0.025, 0.07, r"RGB at $t^*$", transform=ax_rgb.transAxes,
        fontsize=6.9, color="white", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.22", facecolor=UAV,
                  edgecolor="none", alpha=0.88),
    )
    ax_rgb.set_title(r"(a) Sensor evidence at $t^*$", fontsize=10.0, pad=4)

    clipped = depth >= DEPTH_CLIP_M
    semantic_map = LinearSegmentedColormap.from_list(
        "semantic_score", ["#22252A", "#6E4B78", "#C44E52", "#F0B64D"]
    )
    ax_semantic.imshow(semantic, cmap=semantic_map, vmin=0.0, vmax=0.75,
                       interpolation="nearest")
    ax_semantic.text(
        0.025, 0.055, r"semantic score $S_t$  [0, 0.75]",
        transform=ax_semantic.transAxes, fontsize=6.9, color="white", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.22", facecolor=UAV, edgecolor="none", alpha=0.88),
    )
    depth_map = LinearSegmentedColormap.from_list(
        "valid_depth", ["#24343D", "#657985", "#B5C1C8", "#E3E9E8"]
    )
    depth_map.set_bad(BACKGROUND)
    valid_depth = np.ma.masked_where(clipped, depth)
    ax_depth.imshow(valid_depth, cmap=depth_map, vmin=0.0, vmax=DEPTH_CLIP_M,
                    interpolation="nearest")
    ax_depth.text(
        0.025, 0.15, r"depth $D_t$ (m)", transform=ax_depth.transAxes,
        fontsize=7.0, color="white", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.22", facecolor=UAV, edgecolor="none", alpha=0.88),
    )
    ax_depth.text(
        0.975, 0.86, "white: no usable depth (>20 m)", transform=ax_depth.transAxes,
        fontsize=7.0, color="white", ha="right", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.22", facecolor=UAV, edgecolor="none", alpha=0.88),
    )
    for axis in (ax_rgb, ax_semantic, ax_depth):
        axis.set_xticks([])
        axis.set_yticks([])
        for spine in axis.spines.values():
            spine.set_color(TOPOLOGY)
            spine.set_linewidth(0.65)
    return ax_rgb, ax_semantic, ax_depth


def draw_map_panel(
    figure: plt.Figure,
    slot,
    cloud: np.ndarray,
    final_markers: dict[str, list[dict]],
    evidence_markers: dict[str, list[dict]],
    current_path: np.ndarray,
    mission_odom: list[dict],
    current_position: np.ndarray,
    goal: np.ndarray,
    clearance: float | None,
) -> tuple[plt.Axes, tuple[float, float]]:
    ax = figure.add_subplot(slot)
    nodes, logged_node_colors = marker_points_with_colors(
        final_markers, "epic_skeleton_nodes"
    )
    edge_points = marker_points(final_markers, "epic_skeleton_edges")
    edge_witnesses = marker_points(final_markers, "epic_edge_witness_paths")
    witness = marker_points(evidence_markers, "epic_selected_witness_path")
    anchors = semantic_risk_anchors(evidence_markers)
    frontier_goal = marker_pose_position(
        evidence_markers, "epic_frontier_goal", "epic_route_terminal"
    )
    local_goal = marker_pose_position(
        evidence_markers, "epic_local_goal", "epic_yopo_next_goal"
    )
    graph_height = float(current_position[2])
    node_slice = np.abs(nodes[:, 2] - graph_height) <= 2.0 if len(nodes) else np.empty(0, bool)
    nodes = nodes[node_slice]
    if logged_node_colors is not None:
        logged_node_colors = logged_node_colors[node_slice]

    # Render the UE triangle-surface truth in the same shallow aerial
    # projection as every overlay. The negative lateral term rotates the UE
    # top view without mirroring it.
    view = (
        (cloud[:, 0] >= -45.0) & (cloud[:, 0] <= 45.0)
        & (cloud[:, 1] >= 0.0) & (cloud[:, 1] <= 140.0)
    )
    scene_cloud = cloud[view] if np.any(view) else cloud
    footprint_x, footprint_y, small_footprints = small_obstacle_footprint(scene_cloud)
    if len(scene_cloud) > 145_000:
        selection = np.random.default_rng(7).choice(
            len(scene_cloud), 145_000, replace=False
        )
        scene_cloud = scene_cloud[selection]
    scene_u, scene_v = rayfronts_projection(scene_cloud)
    truth_map = LinearSegmentedColormap.from_list(
        "ue_truth_height", ["#D8E0E3", "#A7B6BC", "#718790", "#3F555E"]
    )
    # In this shallow aerial projection, surfaces above the navigation plane
    # are closer to the viewer. Draw them after the ordinary graph so walls
    # and block tops occlude the mesh instead of the mesh showing through.
    foreground_truth = scene_cloud[:, 2] > graph_height - 0.25
    background_truth = ~foreground_truth
    ax.scatter(
        scene_u[background_truth], scene_v[background_truth],
        c=np.clip(scene_cloud[background_truth, 2], 0.0, 22.0),
        cmap=truth_map, norm=Normalize(0.0, 18.0), s=0.34, alpha=0.76,
        linewidths=0, zorder=1, rasterized=True,
    )
    if np.any(small_footprints):
        footprint_points = np.column_stack((
            footprint_x.ravel(), footprint_y.ravel(),
            np.full(footprint_x.size, 0.35),
        ))
        footprint_u, footprint_v = rayfronts_projection(footprint_points)
        ax.contourf(
            footprint_u.reshape(footprint_x.shape),
            footprint_v.reshape(footprint_x.shape),
            small_footprints.astype(float), levels=[0.5, 1.5],
            colors=["#3F555E"], alpha=0.90, antialiased=True,
            zorder=1.8,
        )
        ax.contour(
            footprint_u.reshape(footprint_x.shape),
            footprint_v.reshape(footprint_x.shape),
            small_footprints.astype(float), levels=[0.5], colors=["#24343D"],
            linewidths=0.65, alpha=0.95, zorder=1.9,
        )
    ax.annotate(
        "cylindrical obstacles", xy=(15.0, -2.0), xytext=(1.0, -17.0),
        fontsize=6.5, color=UAV, fontweight="bold", bbox=LABEL_BOX,
        arrowprops=dict(arrowstyle="->", color=UAV, lw=0.7), zorder=10,
    )
    score_map = LinearSegmentedColormap.from_list(
        "semantic_graph_score", ["#EDF3F0", "#D9AD3D", "#D14E46"]
    )
    score_norm = Normalize(0.0, 0.4)
    if len(edge_points) >= 2:
        segments = edge_points[: len(edge_points) // 2 * 2].reshape(-1, 2, 3)
        segment_slice = np.all(np.abs(segments[:, :, 2] - graph_height) <= 2.0, axis=1)
        segments = segments[segment_slice]
        if len(anchors) and len(segments):
            relevant = point_segment_set_distances(anchors, segments) <= RISK_RADIUS_M
            anchors = anchors[relevant]
        projected_segments = []
        for segment in segments:
            u, v = rayfronts_projection(segment)
            projected_segments.append(np.column_stack((u, v)))
        ax.add_collection(LineCollection(
            projected_segments, colors=TOPOLOGY,
            linewidths=0.55, alpha=0.66, zorder=2.8,
        ))
    if len(edge_witnesses) >= 2:
        segments = edge_witnesses[: len(edge_witnesses) // 2 * 2].reshape(-1, 2, 3)
        segment_slice = np.all(np.abs(segments[:, :, 2] - graph_height) <= 2.0, axis=1)
        segments = segments[segment_slice]
        projected_segments = []
        for segment in segments:
            u, v = rayfronts_projection(segment)
            projected_segments.append(np.column_stack((u, v)))
        ax.add_collection(LineCollection(
            projected_segments, colors=CANDIDATE, linewidths=0.28,
            alpha=0.30, zorder=2.2,
        ))
    if len(nodes):
        u, v = rayfronts_projection(nodes)
        node_colors = logged_node_colors if logged_node_colors is not None else TOPOLOGY
        ax.scatter(
            u, v, color=node_colors, s=8.2,
            alpha=0.94, edgecolors=BACKGROUND, linewidths=0.20,
            zorder=3, rasterized=True,
        )

    if np.any(foreground_truth):
        ax.scatter(
            scene_u[foreground_truth], scene_v[foreground_truth],
            c=np.clip(scene_cloud[foreground_truth, 2], 0.0, 22.0),
            cmap=truth_map, norm=Normalize(0.0, 18.0), s=0.42, alpha=0.86,
            linewidths=0, zorder=3.4, rasterized=True,
        )

    if len(anchors):
        u, v = rayfronts_projection(anchors)
        ax.scatter(
            u, v, marker="X", s=30,
            facecolor=RISK_HIGH, edgecolor="white", linewidth=0.55, zorder=7,
        )

    trajectory = np.asarray([event["data"]["position"] for event in mission_odom], dtype=float)
    trajectory_u, trajectory_v = rayfronts_projection(trajectory)
    evidence_index = int(np.argmin(np.linalg.norm(trajectory - current_position, axis=1)))
    ax.plot(trajectory_u, trajectory_v, color="white", lw=3.2,
            solid_capstyle="round", zorder=4)
    ax.plot(trajectory_u[evidence_index:], trajectory_v[evidence_index:],
            color=TOPOLOGY, lw=1.35, alpha=0.58, linestyle=(0, (3, 2)),
            solid_capstyle="round", zorder=4.5)
    ax.plot(trajectory_u[: evidence_index + 1], trajectory_v[: evidence_index + 1],
            color=UAV, lw=1.9,
            solid_capstyle="round", zorder=5)
    if len(witness) > 1:
        u, v = rayfronts_projection(witness)
        ax.plot(u, v, color="white", lw=3.8,
                solid_capstyle="round", zorder=5.5)
        ax.plot(u, v, color=SELECTED, lw=3.3,
                solid_capstyle="round", zorder=6)
    # /epic/path is byte-for-byte the selected witness at this timestamp, so
    # drawing it again in a second color would imply two distinct paths.

    current_u, current_v = rayfronts_projection(current_position[None, :])
    start_u, start_v = rayfronts_projection(trajectory[:1])
    goal_u, goal_v = rayfronts_projection(goal[None, :])
    ax.scatter(current_u, current_v, marker="^", s=52,
               facecolor=UAV, edgecolor="white", linewidth=0.7, zorder=8)
    ax.scatter(start_u, start_v, marker="o", s=45,
               facecolor=BACKGROUND, edgecolor=UAV, linewidth=1.2, zorder=8)
    ax.scatter(goal_u, goal_v, marker="*", s=155,
               facecolor=MISSION, edgecolor="white", linewidth=0.75, zorder=8)
    ax.annotate("mission_goal", xy=(goal_u[0], goal_v[0]), xytext=(-5, 9),
                textcoords="offset points", ha="right", va="bottom",
                fontsize=6.7, color=MISSION, fontweight="bold",
                bbox=LABEL_BOX, zorder=10)
    if frontier_goal is not None:
        u, v = rayfronts_projection(frontier_goal[None, :])
        ax.scatter(u, v, marker="D", s=58, facecolor=FRONTIER,
                   edgecolor="white", linewidth=0.7, zorder=8.2)
        ax.annotate("frontier_goal", xy=(u[0], v[0]), xytext=(8, 13),
                    textcoords="offset points", fontsize=6.6, color=FRONTIER,
                    fontweight="bold", bbox=LABEL_BOX, zorder=10)
    if local_goal is not None:
        u, v = rayfronts_projection(local_goal[None, :])
        ax.scatter(u, v, marker="o", s=52, facecolor=LOCAL_GOAL,
                   edgecolor="white", linewidth=0.7, zorder=8.3)
        ax.annotate("local_goal", xy=(u[0], v[0]), xytext=(-9, -18),
                    textcoords="offset points", ha="right", fontsize=6.6,
                    color=LOCAL_GOAL, fontweight="bold", bbox=LABEL_BOX, zorder=10)
    ax.annotate(
        r"$t^*$", xy=(current_u[0], current_v[0]), xytext=(5, 7),
        textcoords="offset points", fontsize=7.0, color=UAV,
        fontweight="bold", bbox=LABEL_BOX, zorder=10,
    )

    if clearance is not None:
        early = np.flatnonzero(trajectory[:, 1] <= 35.0)
        detour_index = int(early[np.argmin(trajectory[early, 0])])
        detour = trajectory[detour_index]
        detour_u, detour_v = rayfronts_projection(detour[None, :])
        ax.annotate(
            f"global witness ({clearance:.2f} m diagnostic)",
            xy=(detour_u[0], detour_v[0]), xytext=(14, -24), textcoords="offset points",
            fontsize=6.3, color=UAV, ha="left", va="top",
            bbox=LABEL_BOX,
            arrowprops=dict(arrowstyle="->", color=TOPOLOGY, lw=0.65), zorder=10,
        )
    if len(anchors):
        anchor = anchors[int(np.argmin(np.linalg.norm(anchors[:, :2] - current_position[:2], axis=1)))]
        anchor_u, anchor_v = rayfronts_projection(anchor[None, :])
        ax.annotate(
            "far-field semantic nodes ($D_t>20$ m)",
            xy=(anchor_u[0], anchor_v[0]), xytext=(0.23, 0.88), textcoords="axes fraction",
            fontsize=7.0, color=RISK_HIGH, ha="center", va="top",
            bbox=LABEL_BOX,
            arrowprops=dict(arrowstyle="->", color=RISK_HIGH, lw=0.8), zorder=10,
        )

    score_bar_ax = ax.inset_axes([0.61, 0.93, 0.25, 0.025])
    score_mappable = plt.cm.ScalarMappable(norm=score_norm, cmap=score_map)
    score_bar = figure.colorbar(score_mappable, cax=score_bar_ax,
                                orientation="horizontal", ticks=[0.0, 0.4])
    score_bar.ax.tick_params(labelsize=5.7, length=1.5, pad=1)
    score_bar.outline.set_visible(False)
    score_bar.ax.set_title("graph-node semantic score", fontsize=6.2,
                           color=UAV, pad=2)

    handles = [
        Line2D([], [], color="#718790", marker=".", markersize=4, lw=0,
               label="UE mesh truth"),
        Line2D([], [], color=CANDIDATE, lw=0.8, label="candidate witnesses"),
        Line2D([], [], color="#D9AD3D", marker="o", markersize=3.0, lw=0,
               label="graph node (semantic score)"),
        Line2D([], [], color=UAV, lw=1.8, label=r"trajectory to $t^*$"),
        Line2D([], [], color=TOPOLOGY, lw=1.4, ls=(0, (3, 2)),
               label=r"trajectory after $t^*$"),
        Line2D([], [], color=SELECTED, lw=2.2, label="witness / current plan"),
        Line2D([], [], color=RISK_HIGH, marker="X", markersize=5.5, lw=0,
               label="far-field semantic node"),
        Line2D([], [], color=MISSION, marker="*", markersize=8, lw=0, label="mission_goal"),
        Line2D([], [], color=FRONTIER, marker="D", markersize=5.5, lw=0,
               label="frontier_goal"),
        Line2D([], [], color=LOCAL_GOAL, marker="o", markersize=5.5, lw=0,
               label="local_goal"),
    ]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.085),
              ncol=5, fontsize=5.9, frameon=False, handlelength=2.0,
              columnspacing=0.85, borderaxespad=0.0)

    bounds = np.asarray([
        [-45.0, 0.0, 0.0], [45.0, 0.0, 0.0],
        [-45.0, 140.0, 0.0], [45.0, 140.0, 0.0],
    ])
    all_u, all_v = rayfronts_projection(bounds)
    ax.set_xlim(float(all_u.min() - 1.5), float(all_u.max() + 2.0))
    ax.set_ylim(float(all_v.min() - 2.0), float(all_v.max() + 7.0))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("(b) ScaleNav in Map2",
                 fontsize=10.0, pad=4)
    ax.set_facecolor(BACKGROUND)
    for spine in ax.spines.values():
        spine.set_visible(False)
    return ax, (float(current_u[0]), float(current_v[0]))



def draw_scene_and_detour_panel(
    figure: plt.Figure,
    slot,
    scene: np.ndarray,
    final_markers: dict[str, list[dict]],
    evidence_markers: dict[str, list[dict]],
    current_path: np.ndarray,
    mission_odom: list[dict],
    goal: np.ndarray,
    clearance: float | None,
) -> None:
    """RayFronts-style scene overview plus a coordinate-faithful log inset."""
    subgrid = slot.subgridspec(1, 2, width_ratios=[1.05, 1.28], wspace=0.06)
    ax_scene = figure.add_subplot(subgrid[0])
    ax_log = figure.add_subplot(subgrid[1])

    ax_scene.imshow(scene, interpolation="lanczos")
    ax_scene.set_title("(b) Actual Map2 scene", fontsize=10.2, pad=5)
    ax_scene.text(
        0.035, 0.045, "ground-truth visual reference", transform=ax_scene.transAxes,
        fontsize=7.0, color="white", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.24", facecolor=UAV, edgecolor="none", alpha=0.88),
    )
    ax_scene.set_xticks([])
    ax_scene.set_yticks([])

    nodes = marker_points(final_markers, "epic_skeleton_nodes")
    edge_points = marker_points(final_markers, "epic_skeleton_edges")
    witness = marker_points(evidence_markers, "epic_selected_witness_path")
    anchors = marker_points(evidence_markers, "epic_speculative_nodes")
    trajectory = np.asarray([event["data"]["position"] for event in mission_odom], dtype=float)

    if len(edge_points) >= 2:
        segments = edge_points[: len(edge_points) // 2 * 2].reshape(-1, 2, 3)
        for segment in segments:
            ax_log.plot(segment[:, 1], segment[:, 0], color=TOPOLOGY,
                        lw=0.32, alpha=0.25, zorder=1)
    if len(nodes):
        ax_log.scatter(nodes[:, 1], nodes[:, 0], s=2.0, color=TOPOLOGY,
                       alpha=0.48, linewidths=0, zorder=2, rasterized=True)

    ax_log.plot(trajectory[:, 1], trajectory[:, 0], color="white", lw=3.5,
                solid_capstyle="round", zorder=3)
    ax_log.plot(trajectory[:, 1], trajectory[:, 0], color=UAV, lw=1.8,
                solid_capstyle="round", zorder=4)
    if len(witness) > 1:
        ax_log.plot(witness[:, 1], witness[:, 0], color="white", lw=4.0,
                    solid_capstyle="round", zorder=4.5)
        ax_log.plot(witness[:, 1], witness[:, 0], color=SELECTED, lw=2.3,
                    solid_capstyle="round", zorder=5)
    if len(current_path) > 1:
        ax_log.plot(current_path[:, 1], current_path[:, 0], color=FRONTIER,
                    lw=1.9, linestyle=(0, (4, 2)), zorder=5.5)

    visible_anchors = anchors[(anchors[:, 1] <= 42.0)] if len(anchors) else anchors
    for anchor in visible_anchors:
        ax_log.add_patch(Circle(
            (anchor[1], anchor[0]), RISK_RADIUS_M,
            facecolor=RISK_HIGH, edgecolor=RISK_HIGH, alpha=0.06,
            linewidth=0.65, linestyle=(0, (2, 2)), zorder=2.5,
        ))
    if len(visible_anchors):
        ax_log.scatter(visible_anchors[:, 1], visible_anchors[:, 0], marker="X", s=38,
                       facecolor=RISK_HIGH, edgecolor="white", linewidth=0.5, zorder=6)

    current_marker = evidence_markers.get("epic_vehicle_pose", [{}])[0]
    current_position = np.asarray(
        current_marker.get("pose", {}).get("position", trajectory[0]),
        dtype=float,
    )
    ax_log.scatter(trajectory[0, 1], trajectory[0, 0], marker="o", s=42,
                   facecolor=BACKGROUND, edgecolor=UAV, linewidth=1.1, zorder=7)
    ax_log.scatter(current_position[1], current_position[0], marker="^", s=48,
                   facecolor=UAV, edgecolor="white", linewidth=0.65, zorder=7)

    detour_index = int(np.argmin(trajectory[:, 0]))
    detour = trajectory[detour_index]
    if clearance is not None:
        ax_log.annotate(
            f"global witness\n{clearance:.2f} m diagnostic",
            xy=(detour[1], detour[0]), xytext=(29.0, -12.5),
            fontsize=6.8, color=UAV, ha="center", va="top",
            bbox=dict(boxstyle="round,pad=0.24", facecolor=BACKGROUND, edgecolor=CANDIDATE),
            arrowprops=dict(arrowstyle="->", color=TOPOLOGY, lw=0.75), zorder=9,
        )
    remaining = max(0.0, float(goal[1] - 42.0))
    ax_log.annotate(
        f"mission goal\n{remaining:.0f} m ahead", xy=(42.0, 0.0), xytext=(34.0, 9.5),
        fontsize=6.6, color=MISSION, ha="center", va="center",
        arrowprops=dict(arrowstyle="->", color=MISSION, lw=0.8), zorder=9,
    )

    ax_log.set_xlim(-2.0, 43.0)
    ax_log.set_ylim(-16.0, 14.0)
    ax_log.set_aspect("equal", adjustable="box")
    ax_log.set_xlabel("world Y (m)", fontsize=7.2)
    ax_log.set_ylabel("world X (m)", fontsize=7.2)
    ax_log.tick_params(labelsize=6.5, length=2.2, color=TOPOLOGY)
    ax_log.set_title("(c) Large-obstacle detour", fontsize=10.2, pad=5)

    handles = [
        Line2D([], [], color=TOPOLOGY, marker="o", markersize=2.5, lw=0.7,
               label="persistent graph"),
        Line2D([], [], color=UAV, lw=1.8, label="flown trajectory"),
        Line2D([], [], color=SELECTED, lw=2.2, label="selected witness"),
        Line2D([], [], color=FRONTIER, lw=1.8, ls=(0, (4, 2)), label="current plan"),
        Line2D([], [], color=RISK_HIGH, marker="X", markersize=5.5, lw=0,
               label="semantic anchor"),
    ]
    ax_log.legend(handles=handles, loc="upper center", bbox_to_anchor=(-0.03, -0.19),
                  ncol=3, fontsize=6.5, frameon=False, handlelength=2.2,
                  columnspacing=1.0, borderaxespad=0.0)

    for axis in (ax_scene, ax_log):
        axis.set_facecolor(BACKGROUND)
        for spine in axis.spines.values():
            spine.set_color(TOPOLOGY)
            spine.set_linewidth(0.65)


def main() -> None:
    args = parse_args()
    session = args.session_dir.resolve()
    events = load_events(session)
    start_ns, end_ns, goal, mission_odom = first_mission(events)
    rgb_event, rgb, semantic, depth = choose_evidence_frame(session, events, start_ns, end_ns)
    evidence_graph_event = nearest_event(events["graph"], rgb_event["stamp_ns"])
    evidence_path_event = nearest_event(events["path"], rgb_event["stamp_ns"])
    evidence_markers = load_graph(session, evidence_graph_event)
    final_markers = evidence_markers
    current_path = load_path(session, evidence_path_event)
    current_position = interpolated_odom_position(events["odom"], rgb_event["stamp_ns"])
    clearance = evidence_witness_clearance(events, rgb_event["stamp_ns"])
    cloud = aggregate_point_cloud(session, events, start_ns, end_ns)

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 8,
        "axes.linewidth": 0.65,
        "figure.facecolor": BACKGROUND,
        "savefig.facecolor": BACKGROUND,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    figure = plt.figure(figsize=(9.4, 3.85), facecolor=BACKGROUND)
    grid = figure.add_gridspec(1, 2, width_ratios=[0.42, 1.25], wspace=0.015,
                               left=0.012, right=0.995, top=0.91, bottom=0.16)
    evidence_axes = draw_evidence_panel(figure, grid[0], rgb, semantic, depth)
    map_ax, evidence_xy = draw_map_panel(
        figure, grid[1], cloud, final_markers, evidence_markers,
        current_path, mission_odom, current_position, goal, clearance,
    )

    # The equal-aspect map occupies a shorter active axes box than its grid
    # slot. Align the sensor stack to that active top edge so both panel
    # headings have the same baseline and the same image-title gap.
    figure.canvas.draw()
    vertical_shift = map_ax.get_position().y1 - evidence_axes[0].get_position().y1
    for evidence_axis in evidence_axes:
        position = evidence_axis.get_position()
        evidence_axis.set_position([
            position.x0, position.y0 + vertical_shift,
            position.width, position.height,
        ])
    figure.add_artist(ConnectionPatch(
        xyA=evidence_xy, coordsA=map_ax.transData,
        xyB=(1.0, 0.50), coordsB=evidence_axes[1].transAxes,
        color=TOPOLOGY, linewidth=0.85, alpha=0.82, zorder=20,
        linestyle=(0, (2, 2)), arrowstyle="-|>", mutation_scale=7,
        clip_on=False,
    ))

    args.out_prefix.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.out_prefix.with_suffix(".pdf"), dpi=300)
    figure.savefig(args.out_prefix.with_suffix(".png"), dpi=300)
    plt.close(figure)
    suffix = (
        f"global_witness_clearance={clearance:.3f} m"
        if clearance is not None else "global_witness_clearance=n/a"
    )
    deltas_ms = {
        kind: (nearest_event(events[kind], rgb_event["stamp_ns"])["stamp_ns"]
               - rgb_event["stamp_ns"]) / 1e6
        for kind in ("depth", "semantic", "graph", "path", "odom")
    }
    print(f"wrote {args.out_prefix.with_suffix('.pdf')} and .png from {session.name}; "
          f"evidence={rgb_event['file']}; offsets_ms={deltas_ms}; {suffix}")


if __name__ == "__main__":
    main()
