from __future__ import annotations

import argparse
import base64
import json
from functools import lru_cache
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import plotly.offline
import rtoml
import torch

from data.snapshot_dataset import read_ascii_point_cloud_ply
from graph.depth_query import DepthSafeVolumeQuery
from graph.replay import load_scene_frame, run_frame


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = PROJECT_ROOT / "tools"
STATE_COLORS = {
    "CERTIFIED": "#25a55f",
    "UNVALIDATED": "#e2a21b",
    "INVALID": "#d84a4a",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Browse the sparse depth Graph on Map2 data.")
    parser.add_argument("--data", default=str(PROJECT_ROOT / "data" / "Map2GraphData"))
    parser.add_argument("--goal-distance", type=float, default=20.0)
    parser.add_argument("--robot-radius", type=float, default=0.6)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--pearl-root", default=str(PROJECT_ROOT / "third_party" / "PEARL"))
    parser.add_argument("--pearl-checkpoint", default="ViT-B/16")
    parser.add_argument("--pearl-prompt", default="obstacle")
    parser.add_argument("--pearl-device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--obstacle-max-points", type=int, default=15000)
    parser.add_argument("--frgraph", action="store_true")
    return parser.parse_args()


def _png_data_uri(image: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("Failed to encode depth preview")
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:image/png;base64,{payload}"


def _pearl_png_data_uri(probability: np.ndarray) -> str:
    values = np.asarray(probability, dtype=np.float32)
    finite = np.isfinite(values)
    if not finite.any():
        raise ValueError("PEARL heatmap contains no finite values")
    lo, hi = np.percentile(values[finite], (1.0, 99.5))
    if hi <= lo:
        hi = lo + 1e-6
    normalized = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
    # TURBO is continuous and makes low/high probability differences visible.
    image = cv2.applyColorMap(np.rint(normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    return _png_data_uri(image)


def _obstacle_trace(
    scene_dir: Path,
    origin_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    max_points: int,
) -> tuple[dict | None, int]:
    path = scene_dir / "tree.ply"
    if not path.is_file():
        return None, 0
    points = read_ascii_point_cloud_ply(path)
    count = int(len(points))
    if max_points > 0 and count > max_points:
        # PLY vertices are often grouped by mesh part; random sampling gives a
        # much more uniform forest view than taking every Nth vertex.
        indices = np.random.default_rng(0).choice(count, max_points, replace=False)
        points = points[indices]
    points_body = (rotation_body_to_world.T @ (points.astype(np.float64) - origin_world).T).T
    finite = np.isfinite(points_body).all(axis=1)
    points_body = points_body[finite]
    trace = {
        "type": "scatter3d",
        "mode": "markers",
        "name": "Obstacle map",
        "legendgroup": "obstacles",
        "x": points_body[:, 0].astype(float).tolist(),
        "y": points_body[:, 1].astype(float).tolist(),
        "z": points_body[:, 2].astype(float).tolist(),
        "marker": {"size": 2.2, "color": "#788389", "opacity": 0.30},
        "hoverinfo": "skip",
    }
    return trace, count


def _depth_obstacle_trace(
    depth_m: np.ndarray,
    query: DepthSafeVolumeQuery,
    max_points: int,
) -> tuple[dict | None, int]:
    """Back-project non-clipped DepthPlanar pixels into the body-FLU view."""
    depth = np.asarray(depth_m, dtype=np.float32)
    rows, columns = np.indices(depth.shape, dtype=np.float32)
    valid = np.isfinite(depth) & (depth >= query.min_depth_m) & (
        depth < query.far_depth_m - 1e-3
    )
    if not valid.any():
        return None, 0
    forward = depth[valid]
    left = -(columns[valid] - query.cx) * forward / query.fx
    up = -(rows[valid] - query.cy) * forward / query.fy
    points_grid = np.zeros((*depth.shape, 3), dtype=np.float32)
    points_grid[..., 0] = depth
    points_grid[..., 1] = -(columns - query.cx) * depth / query.fx
    points_grid[..., 2] = -(rows - query.cy) * depth / query.fy
    # Keep the organized image grid so neighboring depth pixels form a
    # continuous wall surface instead of visually misleading scan stripes.
    step = 1
    valid_grid = valid
    if max_points > 0 and int(valid.sum()) > max_points:
        step = int(np.ceil(np.sqrt(valid.sum() / max_points)))
        valid_grid = np.zeros_like(valid)
        valid_grid[::step, ::step] = valid[::step, ::step]
        points_grid = points_grid[::step, ::step]
    rows_count, cols_count = valid_grid[::step, ::step].shape
    valid_grid = valid_grid[::step, ::step]
    flat_valid = valid_grid.reshape(-1)
    points = points_grid.reshape(-1, 3)[flat_valid]
    count = int(valid.sum())
    index_grid = np.full(valid_grid.shape, -1, dtype=np.int32)
    index_grid[valid_grid] = np.arange(len(points), dtype=np.int32)
    faces_i: list[int] = []
    faces_j: list[int] = []
    faces_k: list[int] = []
    for row in range(rows_count - 1):
        for column in range(cols_count - 1):
            a = int(index_grid[row, column])
            b = int(index_grid[row, column + 1])
            c = int(index_grid[row + 1, column])
            d = int(index_grid[row + 1, column + 1])
            if a < 0 or b < 0 or c < 0 or d < 0:
                continue
            faces_i.extend((a, b))
            faces_j.extend((b, d))
            faces_k.extend((c, c))
            faces_i.append(b)
            faces_j.append(d)
            faces_k.append(c)
    if not faces_i:
        return None, count
    return {
        "type": "mesh3d",
        "name": "Observed depth",
        "legendgroup": "observed-depth",
        "x": points[:, 0].astype(float).tolist(),
        "y": points[:, 1].astype(float).tolist(),
        "z": points[:, 2].astype(float).tolist(),
        "i": faces_i,
        "j": faces_j,
        "k": faces_k,
        "color": "#59666d",
        "opacity": 0.52,
        "flatshading": True,
        "hoverinfo": "skip",
    }, count


def _body_position(
    position_world: list[float] | np.ndarray,
    origin_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
) -> list[float]:
    value = rotation_body_to_world.T @ (np.asarray(position_world) - origin_world)
    return [float(item) for item in value]


def _line_trace(
    name: str,
    segments: list[tuple[list[float], list[float]]],
    color: str,
    *,
    width: int,
    dash: str = "solid",
    legend_group: str | None = None,
) -> dict:
    x: list[float | None] = []
    y: list[float | None] = []
    z: list[float | None] = []
    for start, end in segments:
        x.extend((start[0], end[0], None))
        y.extend((start[1], end[1], None))
        z.extend((start[2], end[2], None))
    return {
        "type": "scatter3d",
        "mode": "lines",
        "name": name,
        "legendgroup": legend_group or name,
        "x": x,
        "y": y,
        "z": z,
        "line": {"color": color, "width": width, "dash": dash},
        "hoverinfo": "name",
    }


def make_graph_figure(
    result: dict,
    origin_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    obstacle_trace: dict | None = None,
    observed_depth_trace: dict | None = None,
) -> dict:
    graph = result["graph"]
    body_by_id = {
        node["id"]: _body_position(node["positionWorld"], origin_world, rotation_body_to_world)
        for node in graph["nodes"]
    }
    traces: list[dict] = []
    if obstacle_trace is not None:
        traces.append(obstacle_trace)
    if observed_depth_trace is not None:
        traces.append(observed_depth_trace)

    frgraph = result.get("frgraph", {})
    region_segments = []
    for region in frgraph.get("regions", []):
        distance = min(5.0, float(region["depthLimitM"]))
        center_elev = np.deg2rad(float(region["centerElevDeg"]))
        for yaw_deg in (
            region["yawMinDeg"],
            region["centerYawDeg"],
            region["yawMaxDeg"],
        ):
            yaw = np.deg2rad(float(yaw_deg))
            direction = np.array(
                [
                    np.cos(center_elev) * np.cos(yaw),
                    np.cos(center_elev) * np.sin(yaw),
                    np.sin(center_elev),
                ]
            )
            region_segments.append((np.zeros(3), direction * distance))
    if region_segments:
        traces.append(
            _line_trace(
                "FRGraph free region",
                region_segments,
                "#00a6b2",
                width=3,
                dash="dot",
            )
        )

    for state in ("CERTIFIED", "UNVALIDATED", "INVALID"):
        segments = [
            (body_by_id[edge["source"]], body_by_id[edge["target"]])
            for edge in graph["edges"]
            if edge["state"] == state
        ]
        if segments:
            traces.append(
                _line_trace(
                    f"{state} edge",
                    segments,
                    STATE_COLORS[state],
                    width=5 if state == "CERTIFIED" else 3,
                    dash="dot" if state == "UNVALIDATED" else "solid",
                    legend_group=state,
                )
            )

    for state in ("CERTIFIED", "UNVALIDATED", "INVALID"):
        nodes = [node for node in graph["nodes"] if node["state"] == state]
        if not nodes:
            continue
        positions = [body_by_id[node["id"]] for node in nodes]
        labels = [
            (
                f"Node {node['id']}<br>{state}<br>"
                f"clearance: {node['clearanceM']:.2f} m"
                if node["clearanceM"] is not None
                else f"Node {node['id']}<br>{state}<br>clearance: --"
            )
            for node in nodes
        ]
        traces.append(
            {
                "type": "scatter3d",
                "mode": "markers+text",
                "name": f"{state} node",
                "legendgroup": state,
                "x": [point[0] for point in positions],
                "y": [point[1] for point in positions],
                "z": [point[2] for point in positions],
                "text": [str(node["id"]) for node in nodes],
                "textposition": "top center",
                "hovertext": labels,
                "hoverinfo": "text",
                "marker": {
                    "size": 7,
                    "color": STATE_COLORS[state],
                    "line": {"color": "#ffffff", "width": 1},
                },
            }
        )

    def path_trace(path_key: str, name: str, color: str, dash: str) -> None:
        path = result[path_key]
        if len(path) < 2:
            return
        points = [body_by_id[node_id] for node_id in path]
        traces.append(
            {
                "type": "scatter3d",
                "mode": "lines",
                "name": name,
                "x": [point[0] for point in points],
                "y": [point[1] for point in points],
                "z": [point[2] for point in points],
                "line": {"color": color, "width": 10, "dash": dash},
                "hoverinfo": "name",
            }
        )

    path_trace("optimisticPath", "Optimistic path", "#e2a21b", "dash")
    path_trace("certifiedPath", "Certified path", "#087f5b", "solid")

    current_id = graph["currentNodeId"]
    current = body_by_id[current_id]
    goal = result["goalBody"]
    marker_points = [("Current", current, "#1769aa", "diamond", 10)]
    marker_points.append(("Goal", goal, "#212529", "x", 10))
    waypoint = result["certifiedWaypointBody"] or result["optimisticWaypointBody"]
    if waypoint is not None:
        marker_points.append(("Waypoint", waypoint, "#00a6b2", "diamond-open", 12))
    for name, point, color, symbol, size in marker_points:
        traces.append(
            {
                "type": "scatter3d",
                "mode": "markers",
                "name": name,
                "x": [point[0]],
                "y": [point[1]],
                "z": [point[2]],
                "marker": {"size": size, "color": color, "symbol": symbol},
                "hovertemplate": (
                    f"{name}<br>forward %{{x:.2f}} m<br>left %{{y:.2f}} m"
                    "<br>up %{z:.2f} m<extra></extra>"
                ),
            }
        )

    return {
        "data": traces,
        "layout": {
            "margin": {"l": 0, "r": 0, "t": 8, "b": 0},
            "paper_bgcolor": "#ffffff",
            "plot_bgcolor": "#ffffff",
            "showlegend": True,
            "legend": {"orientation": "h", "x": 0.01, "y": 0.99, "bgcolor": "rgba(255,255,255,.82)"},
            "scene": {
                "aspectmode": "data",
                "camera": {"eye": {"x": 1.35, "y": -1.55, "z": 0.85}},
                "xaxis": {"title": "Forward (m)", "gridcolor": "#dfe4e7", "zerolinecolor": "#8c969d"},
                "yaxis": {"title": "Left (m)", "gridcolor": "#dfe4e7", "zerolinecolor": "#8c969d"},
                "zaxis": {"title": "Up (m)", "gridcolor": "#dfe4e7", "zerolinecolor": "#8c969d"},
            },
            "uirevision": "graph-camera",
        },
    }


def make_depth_overlay(
    depth_m: np.ndarray,
    query: DepthSafeVolumeQuery,
    graph: dict,
    origin_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    waypoint_body: list[float] | None,
    max_depth_m: float,
) -> str:
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(depth) & (depth >= query.min_depth_m)
    clipped = np.clip(depth, 0.0, max_depth_m)
    gray = np.zeros(depth.shape, dtype=np.uint8)
    # A fixed square-root tone curve preserves distance ordering while making
    # near and middle ranges readable in the small 160x96 preview.
    gray[valid] = np.rint(255.0 * np.sqrt(clipped[valid] / max_depth_m)).astype(np.uint8)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # Draw the same 3D edges used by the validator in image space. This makes
    # an edge's red INVALID state explainable when it crosses a visible wall.
    for edge in graph["edges"]:
        start = _body_position(
            next(node for node in graph["nodes"] if node["id"] == edge["source"])["positionWorld"],
            origin_world,
            rotation_body_to_world,
        )
        end = _body_position(
            next(node for node in graph["nodes"] if node["id"] == edge["target"])["positionWorld"],
            origin_world,
            rotation_body_to_world,
        )
        projected: list[tuple[int, int]] = []
        for progress in np.linspace(0.0, 1.0, 25):
            projection = query.project(
                np.asarray(start) + progress * (np.asarray(end) - np.asarray(start))
            )
            if projection is not None:
                projected.append(tuple(int(round(value)) for value in projection))
        if len(projected) < 2:
            continue
        color_hex = STATE_COLORS[edge["state"]].lstrip("#")
        color = tuple(int(color_hex[index : index + 2], 16) for index in (4, 2, 0))
        for first, second in zip(projected, projected[1:]):
            cv2.line(image, first, second, color, 1, cv2.LINE_AA)

    node_by_id = {node["id"]: node for node in graph["nodes"]}
    for node in graph["nodes"]:
        point = _body_position(node["positionWorld"], origin_world, rotation_body_to_world)
        projection = query.project(np.asarray(point))
        if projection is None:
            continue
        u, v = (int(round(value)) for value in projection)
        color_hex = STATE_COLORS[node["state"]].lstrip("#")
        color = tuple(int(color_hex[index : index + 2], 16) for index in (4, 2, 0))
        cv2.circle(image, (u, v), 4, color, -1, cv2.LINE_AA)
        cv2.circle(image, (u, v), 5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, str(node["id"]), (u + 6, v - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)

    if waypoint_body is not None:
        projection = query.project(np.asarray(waypoint_body))
        if projection is not None:
            u, v = (int(round(value)) for value in projection)
            cv2.drawMarker(image, (u, v), (255, 255, 0), cv2.MARKER_DIAMOND, 12, 2, cv2.LINE_AA)
    return _png_data_uri(image)


class GraphViewerEngine:
    def __init__(
        self,
        data_root: str | Path,
        goal_distance: float = 20.0,
        robot_radius: float = 0.6,
        pearl_root: str | Path | None = None,
        pearl_checkpoint: str = "ViT-B/16",
        pearl_prompt: str = "obstacle",
        pearl_device: str = "auto",
        obstacle_max_points: int = 15000,
        use_frgraph: bool = False,
    ) -> None:
        self.data_root = Path(data_root).expanduser().resolve()
        self.goal_distance = float(goal_distance)
        self.robot_radius = float(robot_radius)
        self.pearl_root = (
            Path(pearl_root).expanduser().resolve()
            if pearl_root
            else PROJECT_ROOT / "third_party" / "PEARL"
        )
        self.pearl_checkpoint = pearl_checkpoint
        self.pearl_prompt = pearl_prompt.strip() or "obstacle"
        self.pearl_device = pearl_device
        self.obstacle_max_points = int(obstacle_max_points)
        self.use_frgraph = bool(use_frgraph)
        self._pearl = None
        self._pearl_error: str | None = None
        if not self.data_root.is_dir():
            raise FileNotFoundError(f"Graph data directory not found: {self.data_root}")
        if self.goal_distance <= 0.0 or self.robot_radius <= 0.0:
            raise ValueError("goal distance and robot radius must be positive")
        self.catalog = self._build_catalog()
        if not self.catalog["scenes"]:
            raise FileNotFoundError(f"No scene data.toml found below {self.data_root}")

    def _get_pearl(self):
        if self._pearl is not None or self._pearl_error is not None:
            return self._pearl
        try:
            from text_tracker.pearl_adapter import PEARLHeatmapEncoder

            if self.pearl_root is None:
                raise FileNotFoundError("PEARL root is not configured")
            if self.pearl_device == "cuda" or (
                self.pearl_device == "auto" and torch.cuda.is_available()
            ):
                device = torch.device("cuda")
            else:
                device = torch.device("cpu")
            self._pearl = PEARLHeatmapEncoder(
                str(self.pearl_root), checkpoint=self.pearl_checkpoint, device=device
            )
            self._pearl.prepare_prompt(self.pearl_prompt)
        except Exception as error:  # Viewer remains usable without optional PEARL assets.
            self._pearl_error = f"{type(error).__name__}: {error}"
        return self._pearl

    def _pearl_payload(self, scene_dir: Path, record: dict) -> dict:
        default = {
            "pearlPrompt": self.pearl_prompt,
            "frgraph": self.use_frgraph,
            "pearlAvailable": False,
            "pearlImage": None,
            "pearlPeak": None,
            "pearlMean": None,
            "pearlError": self._pearl_error,
        }
        rgb_name = record.get("rgbFileName")
        if not rgb_name:
            default["pearlError"] = default["pearlError"] or "RGB frame is missing"
            return default
        encoder = self._get_pearl()
        if encoder is None:
            default["pearlError"] = self._pearl_error
            return default
        try:
            probability = encoder.encode(str(scene_dir / "Textures" / rgb_name), self.pearl_prompt)
            default.update(
                pearlAvailable=True,
                pearlImage=_pearl_png_data_uri(probability),
                pearlPeak=float(np.nanmax(probability)),
                pearlMean=float(np.nanmean(probability)),
                pearlError=None,
            )
        except Exception as error:
            default["pearlError"] = f"{type(error).__name__}: {error}"
        return default

    def _build_catalog(self) -> dict:
        scenes = []
        for document_path in sorted(self.data_root.glob("Scene_*/data.toml")):
            document = rtoml.load(document_path)
            frames = [
                {"index": index, "id": int(record.get("frameIndex", index))}
                for index, record in enumerate(document.get("dataArray", []))
            ]
            if frames:
                scenes.append({"name": document_path.parent.name, "frames": frames})
        return {
            "dataRoot": str(self.data_root),
            "goalDistanceM": self.goal_distance,
            "robotRadiusM": self.robot_radius,
            "pearlPrompt": self.pearl_prompt,
            "frameCount": sum(len(scene["frames"]) for scene in scenes),
            "scenes": scenes,
        }

    @lru_cache(maxsize=64)
    def frame_payload(self, scene: str, frame_index: int) -> dict:
        scene_names = {item["name"] for item in self.catalog["scenes"]}
        if scene not in scene_names:
            raise ValueError(f"Unknown scene: {scene}")
        depth, position, rotation, horizontal_fov, vertical_fov, record = load_scene_frame(
            self.data_root, scene, frame_index
        )
        query = DepthSafeVolumeQuery(
            depth,
            horizontal_fov_deg=horizontal_fov,
            vertical_fov_deg=vertical_fov,
            robot_radius_m=self.robot_radius,
        )
        _, result = run_frame(
            depth,
            position_world=position,
            rotation_body_to_world=rotation,
            goal_body=np.array([self.goal_distance, 0.0, 0.0]),
            horizontal_fov_deg=horizontal_fov,
            vertical_fov_deg=vertical_fov,
            robot_radius_m=self.robot_radius,
            use_frgraph=self.use_frgraph,
        )
        graph = result["graph"]
        scene_dir = self.data_root / scene
        obstacle_trace, obstacle_count = _obstacle_trace(
            scene_dir, position, rotation, self.obstacle_max_points
        )
        observed_depth_trace, observed_depth_count = _depth_obstacle_trace(
            depth, query, self.obstacle_max_points
        )
        node_details = [
            {
                "id": node["id"],
                "state": node["state"],
                "positionBody": _body_position(
                    node["positionWorld"], position, rotation
                ),
                "clearanceM": node["clearanceM"],
            }
            for node in graph["nodes"]
        ]
        edge_details = [
            {
                "source": edge["source"],
                "target": edge["target"],
                "state": edge["state"],
            }
            for edge in graph["edges"]
        ]
        edge_counts = {
            state: sum(edge["state"] == state for edge in graph["edges"])
            for state in STATE_COLORS
        }
        waypoint = result["certifiedWaypointBody"] or result["optimisticWaypointBody"]
        max_depth = float(record.get("depthMaxMeters", np.nanmax(depth)))
        if not np.isfinite(max_depth) or max_depth <= 0.0:
            max_depth = 20.0
        return {
            "scene": scene,
            "frame": int(record.get("frameIndex", frame_index)),
            "frameIndex": frame_index,
            "pose": {
                "positionNed": [float(value) for value in position],
                "yawDeg": float(record.get("yawStart", 0.0)),
                "horizontalFovDeg": horizontal_fov,
                "verticalFovDeg": vertical_fov,
            },
            "nodeCount": len(graph["nodes"]),
            "edgeCount": len(graph["edges"]),
            "stateCounts": result["stateCounts"],
            "edgeStateCounts": edge_counts,
            "nodeDetails": node_details,
            "edgeDetails": edge_details,
            "certifiedPath": result["certifiedPath"],
            "optimisticPath": result["optimisticPath"],
            "certifiedWaypointBody": result["certifiedWaypointBody"],
            "optimisticWaypointBody": result["optimisticWaypointBody"],
            "figure": make_graph_figure(
                result, position, rotation, obstacle_trace, observed_depth_trace
            ),
            "obstaclePointCount": obstacle_count,
            "observedDepthPointCount": observed_depth_count,
            **self._pearl_payload(scene_dir, record),
            "depthImage": make_depth_overlay(
                depth, query, graph, position, rotation, waypoint, max_depth
            ),
        }


class GraphViewerServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], engine: GraphViewerEngine) -> None:
        super().__init__(address, GraphViewerHandler)
        self.engine = engine
        self.page = (TOOLS_DIR / "graph_viewer.html").read_bytes()
        self.plotly = plotly.offline.get_plotlyjs().encode("utf-8")


class GraphViewerHandler(BaseHTTPRequestHandler):
    server: GraphViewerServer

    def do_GET(self) -> None:
        request = urlparse(self.path)
        try:
            if request.path in ("/", "/index.html"):
                self.send_bytes(self.server.page, "text/html; charset=utf-8")
                return
            if request.path == "/plotly.min.js":
                self.send_bytes(self.server.plotly, "application/javascript; charset=utf-8")
                return
            if request.path == "/api/index":
                self.send_json(self.server.engine.catalog)
                return
            if request.path == "/api/frame":
                params = parse_qs(request.query)
                scene = params.get("scene", [None])[0]
                frame = params.get("frame", [None])[0]
                if scene is None or frame is None:
                    raise ValueError("scene and frame are required")
                self.send_json(self.server.engine.frame_payload(scene, int(frame)))
                return
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
        except (ValueError, IndexError, FileNotFoundError) as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_bytes(body, "application/json; charset=utf-8", status)

    def send_bytes(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args) -> None:
        return


def main() -> None:
    args = parse_args()
    engine = GraphViewerEngine(
        args.data,
        args.goal_distance,
        args.robot_radius,
        pearl_root=args.pearl_root,
        pearl_checkpoint=args.pearl_checkpoint,
        pearl_prompt=args.pearl_prompt,
        pearl_device=args.pearl_device,
        obstacle_max_points=args.obstacle_max_points,
        use_frgraph=args.frgraph,
    )
    server = GraphViewerServer((args.host, args.port), engine)
    print(f"OpenSeek sparse Graph viewer: http://{args.host}:{args.port}")
    print(f"Data: {engine.data_root} ({engine.catalog['frameCount']} frames)")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
