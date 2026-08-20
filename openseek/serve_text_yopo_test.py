from __future__ import annotations

import argparse
import json
import threading
from functools import lru_cache
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np
import plotly.offline
import plotly.io as pio
import rtoml
import torch

from text_tracker.dataset import TextYopoDataset
from text_tracker.heatmap import pearl_similarity
from text_tracker.loss import TextYopoGuidanceLoss
from tools.visualize_text_yopo_test import (
    DEFAULT_DATA,
    DEFAULT_MODEL,
    choose_device,
    depth_point_cloud,
    depth_image,
    file_data_uri,
    frame_id,
    heatmap_peak_body_vector,
    make_figure,
    move_sample,
    png_data_uri,
    sample_trajectories,
    scene_name,
    signed_heatmap_image,
    target_body_vector,
)


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Browse OpenSeek baseline predictions on the testing dataset."
    )
    parser.add_argument("--data", default=str(DEFAULT_DATA))
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument(
        "--mode", choices=("auto", "search", "approach"), default="auto"
    )
    parser.add_argument("--pearl-enter-threshold", type=float, default=0.08)
    parser.add_argument("--search-goal-distance", type=float, default=10.0)
    parser.add_argument("--semantic-weight", type=float, default=1.2)
    parser.add_argument("--radius", type=float, default=15.0)
    parser.add_argument("--max-points", type=int, default=12000)
    parser.add_argument("--trajectory-points", type=int, default=80)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    return parser.parse_args()


class TestEngine:
    def __init__(self, args: argparse.Namespace) -> None:
        if (
            args.radius <= 0.0
            or args.max_points <= 0
            or args.trajectory_points < 2
            or args.search_goal_distance <= 0.0
        ):
            raise ValueError(
                "--radius/--max-points must be positive and --trajectory-points >= 2"
            )
        self.mode = args.mode
        self.radius = args.radius
        self.max_points = args.max_points
        self.trajectory_points = args.trajectory_points
        self.device = choose_device(args.device)
        approach_probability = 0.0 if self.mode == "search" else 1.0
        pearl_threshold = (
            0.0 if self.mode == "approach" else args.pearl_enter_threshold
        )
        self.dataset = TextYopoDataset(
            args.data,
            seed=100000,
            approach_probability=approach_probability,
            pearl_enter_threshold=pearl_threshold,
        )
        self.model_path = Path(args.model).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"OpenSeek baseline TorchScript model not found: {self.model_path}"
            )
        self.model = torch.jit.load(
            str(self.model_path), map_location=self.device
        ).eval()
        self.cost = TextYopoGuidanceLoss(
            self.dataset.scene_obstacles,
            semantic_weight=args.semantic_weight,
            device=self.device,
        ).to(self.device)
        self.lock = threading.Lock()
        self.catalog = self._build_catalog()

    def _build_catalog(self) -> dict:
        scenes: dict[str, list[dict]] = {}
        for index, record in enumerate(self.dataset.records):
            metadata = record["metadata"]
            scenes.setdefault(scene_name(record), []).append(
                {
                    "index": index,
                    "id": frame_id(record),
                    "visible": bool(record["target_visible"]),
                    "prompt": str(metadata.get("targetPrompt", "")),
                }
            )
        return {
            "mode": self.mode,
            "model": str(self.model_path),
            "frameCount": len(self.dataset),
            "scenes": [
                {"name": name, "frames": frames}
                for name, frames in scenes.items()
            ],
        }

    @lru_cache(maxsize=48)
    def frame_payload(self, index: int) -> dict:
        if index < 0 or index >= len(self.dataset):
            raise IndexError(f"Frame index out of range: {index}")
        record = self.dataset.records[index]
        sample = self.dataset[index]
        target_visible = bool(sample["target_visible"].item())
        active_mode = "visible-goal" if target_visible else "random-goal"
        batch = move_sample(sample, self.device)
        with self.lock, torch.inference_mode():
            output = self.model(batch["image"], batch["obs"])
            if not isinstance(output, (tuple, list)) or len(output) != 2:
                raise ValueError(
                    "Incompatible model: expected current (endstate, score) output"
                )
            endstate, predicted_score = output
            if endstate.ndim != 4 or endstate.shape[1] != 9 or predicted_score.ndim != 3:
                raise ValueError(
                    f"Unexpected model outputs: {tuple(endstate.shape)}, "
                    f"{tuple(predicted_score.shape)}"
                )
            true_cost, component_tensors = self.cost.trajectory_costs(
                endstate, batch
            )

        endstates = (
            endstate[0].permute(1, 2, 0).reshape(-1, 9).detach().cpu().numpy()
        )
        predicted_scores = predicted_score[0].reshape(-1).detach().cpu().numpy()
        true_costs = true_cost[0].reshape(-1).detach().cpu().numpy()
        components = {
            name: values[0].reshape(-1).detach().cpu().numpy()
            for name, values in component_tensors.items()
        }
        predicted_best = int(predicted_scores.argmin())
        oracle_best = int(true_costs.argmin())
        trajectories = sample_trajectories(
            endstates,
            sample["obs"].numpy(),
            self.trajectory_points,
        )

        scene_dir = Path(record["depth_path"]).parent.parent
        document = self.scene_document(str(scene_dir / "data.toml"))
        far_clip = float(document.get("depthCameraFarClipPlane", 20.0))
        # This preview is frame-aligned. The static PLY remains the collision
        # source for training, while DepthPlanar gives the visible geometry
        # even when the UE map uses instanced foliage meshes.
        points = depth_point_cloud(
            Path(record["depth_path"]),
            float(record["horizontal_fov"]),
            float(document.get("depthCameraVerticalFOV", 60.0)),
            self.radius,
            self.max_points,
            far_clip,
        )
        metadata = record["metadata"]
        target = target_body_vector(sample, metadata, active_mode)
        network_heatmap = sample["image"][1].numpy()
        display_heatmap = network_heatmap
        if record["semantic_path"] is not None:
            raw_heatmap = np.load(record["semantic_path"]).astype(np.float32)
            if raw_heatmap.ndim == 2 and np.isfinite(raw_heatmap).all():
                display_heatmap = raw_heatmap
        guidance = heatmap_peak_body_vector(
            display_heatmap,
            float(record["horizontal_fov"]),
        )
        figure = make_figure(
            points,
            trajectories,
            predicted_best,
            oracle_best,
            predicted_scores,
            true_costs,
            target,
            self.radius,
            target_label=("PEARL 峰值 3D Goal" if target_visible else "原版随机 3D Goal"),
            guidance=None,
        )
        figure_json = json.loads(pio.to_json(figure, validate=False))

        _, depth_panel = depth_image(Path(record["depth_path"]), far_clip)
        heatmap_panel = signed_heatmap_image(display_heatmap)
        rgb_name = metadata.get("rgbFileName")
        if not rgb_name:
            raise FileNotFoundError("The selected frame has no RGB image")
        target_distance = float(np.linalg.norm(target))
        pearl_confidence = 0.0
        if record["semantic_path"] is not None:
            pearl_confidence = pearl_similarity(np.load(record["semantic_path"]))
        strength = float(display_heatmap.max())
        candidates = []
        for candidate in range(predicted_scores.size):
            candidates.append(
                {
                    "id": candidate,
                    "predicted": float(predicted_scores[candidate]),
                    "total": float(components["total"][candidate]),
                    "smooth": float(components["smooth"][candidate]),
                    "safety": float(components["safety"][candidate]),
                    "acceleration": float(components["acceleration"][candidate]),
                    "goalCost": float(components["goal"][candidate]),
                    "endpointDistance": float(components["endpoint_distance"][candidate]),
                    "predictedBest": candidate == predicted_best,
                    "oracleBest": candidate == oracle_best,
                }
            )
        return {
            "index": index,
            "scene": scene_name(record),
            "frame": frame_id(record),
            "mode": active_mode,
            "requestedMode": self.mode,
            "prompt": str(metadata.get("targetPrompt", "")),
            "visible": target_visible,
            "targetDistance": target_distance,
            "targetVectorBody": [float(value) for value in target],
            "heatmapPeakBody": [float(value) for value in guidance],
            "heatmapStrength": strength,
            "heatmapResolution": [int(display_heatmap.shape[1]), int(display_heatmap.shape[0])],
            "networkHeatmapResolution": [
                int(network_heatmap.shape[1]), int(network_heatmap.shape[0])
            ],
            "pearlConfidence": pearl_confidence,
            "predictedBest": predicted_best,
            "oracleBest": oracle_best,
            "pointCount": int(points.shape[0]),
            "images": {
                "rgb": file_data_uri(scene_dir / "Textures" / rgb_name),
                "depth": png_data_uri(depth_panel),
                "heatmap": png_data_uri(heatmap_panel),
            },
            "figure": figure_json,
            "candidates": candidates,
        }

    @staticmethod
    @lru_cache(maxsize=16)
    def scene_document(path: str) -> dict:
        with Path(path).open("r", encoding="utf-8") as file:
            return rtoml.load(file)


class TestServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], engine: TestEngine) -> None:
        super().__init__(address, TestHandler)
        self.engine = engine
        self.page = (TOOLS_DIR / "text_yopo_test.html").read_bytes()
        self.plotly = plotly.offline.get_plotlyjs().encode("utf-8")


class TestHandler(BaseHTTPRequestHandler):
    server: TestServer

    def do_GET(self) -> None:
        request = urlparse(self.path)
        try:
            if request.path in ("/", "/index.html"):
                self.send_bytes(self.server.page, "text/html; charset=utf-8")
                return
            if request.path == "/plotly.min.js":
                self.send_bytes(
                    self.server.plotly,
                    "application/javascript; charset=utf-8",
                )
                return
            if request.path == "/api/index":
                self.send_json(self.server.engine.catalog)
                return
            if request.path == "/api/frame":
                params = parse_qs(request.query)
                raw_index = params.get("index", [None])[0]
                if raw_index is None:
                    raise ValueError("Missing frame index")
                self.send_json(
                    self.server.engine.frame_payload(int(raw_index))
                )
                return
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
        except (ValueError, IndexError, FileNotFoundError) as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)

    def send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
            "utf-8"
        )
        self.send_bytes(body, "application/json; charset=utf-8", status)

    def send_bytes(
        self,
        body: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
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
    engine = TestEngine(args)
    server = TestServer((args.host, args.port), engine)
    print(f"OpenSeek baseline offline test: http://{args.host}:{args.port}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
