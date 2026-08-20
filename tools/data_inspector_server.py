#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from functools import lru_cache
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

try:
    import cv2
    import numpy as np
    import rtoml
except ImportError as exc:
    raise SystemExit(
        "缺少数据查看依赖。请使用：\n"
        "  PYTHON=/path/to/python bash scripts/inspect_data.sh\n"
        f"原始错误：{exc}"
    ) from exc


TOOLS_DIR = Path(__file__).resolve().parent
DEFAULT_DATA = TOOLS_DIR.parent / "data" / "TrainingData"


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


class DatasetCatalog:
    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.frames: dict[tuple[str, str], dict[str, Any]] = {}
        self.payload: dict[str, Any] = {}
        self.reload()

    def reload(self) -> None:
        if not self.root.is_dir():
            raise FileNotFoundError(f"数据目录不存在：{self.root}")

        scenes: list[dict[str, Any]] = []
        frame_lookup: dict[tuple[str, str], dict[str, Any]] = {}
        total_visible = 0
        total_frames = 0
        total_complete = 0

        for scene_dir in sorted(self.root.glob("Scene_*"), key=scene_sort_key):
            toml_path = scene_dir / "data.toml"
            if not toml_path.is_file():
                continue
            document = rtoml.load(toml_path)
            texture_dir = scene_dir / "Textures"
            scene_frames: list[dict[str, Any]] = []

            for ordinal, item in enumerate(document.get("dataArray", [])):
                rgb_name = item.get("rgbFileName")
                depth_name = item.get("depthFileName")
                if not rgb_name:
                    rgb_name = find_named_file(item.get("imageFileNameList", []), "rgb_")
                if not depth_name:
                    depth_name = find_named_file(item.get("imageFileNameList", []), "depth_")
                index = frame_index(rgb_name, ordinal)
                semantic_names = (
                    f"semantic_pearl_{index}.npy",
                    f"semantic_raw_{index}.npy",
                )
                semantic_path = next(
                    (texture_dir / name for name in semantic_names if (texture_dir / name).is_file()),
                    None,
                )
                paths = {
                    "rgb": texture_dir / rgb_name if rgb_name else None,
                    "depth": texture_dir / depth_name if depth_name else None,
                    "semantic": semantic_path,
                }
                available = {
                    name: bool(path and path.is_file()) for name, path in paths.items()
                }
                complete = all(available.values())
                visible = bool(item.get("targetVisible", False))
                frame = {
                    "id": str(index),
                    "ordinal": ordinal,
                    "scene": scene_dir.name,
                    "visible": visible,
                    "complete": complete,
                    "available": available,
                    "prompt": str(item.get("targetPrompt", "")),
                    "confidence": float(item.get("semanticConfidence", 0.0)),
                    "position": json_safe(item.get("posStart", [])),
                    "yaw": float(item.get("yawStart", 0.0)),
                    "targetPixel": json_safe(item.get("targetPixel", [])),
                    "targetImageSize": [
                        int(item.get("targetImageWidth", 0)),
                        int(item.get("targetImageHeight", 0)),
                    ],
                    "targetCamera": json_safe(item.get("targetPositionCamera", [])),
                    "targetWorld": json_safe(item.get("targetPositionWorld", [])),
                    "metadata": json_safe(item),
                    "_paths": paths,
                }
                public_frame = {key: value for key, value in frame.items() if key != "_paths"}
                scene_frames.append(public_frame)
                frame_lookup[(scene_dir.name, str(index))] = frame
                total_frames += 1
                total_visible += int(visible)
                total_complete += int(complete)

            scenes.append(
                {
                    "name": scene_dir.name,
                    "frames": scene_frames,
                    "frameCount": len(scene_frames),
                    "visibleCount": sum(int(frame["visible"]) for frame in scene_frames),
                    "completeCount": sum(int(frame["complete"]) for frame in scene_frames),
                    "hasTreePly": (scene_dir / "tree.ply").is_file(),
                    "hasTargetPly": (scene_dir / "target.ply").is_file(),
                    "camera": {
                        "farClip": document.get("depthCameraFarClipPlane"),
                        "horizontalFov": document.get("depthCameraHorizontalFOV"),
                    },
                }
            )

        self.frames = frame_lookup
        self.payload = {
            "root": str(self.root),
            "name": self.root.name,
            "sceneCount": len(scenes),
            "frameCount": total_frames,
            "visibleCount": total_visible,
            "completeCount": total_complete,
            "scenes": scenes,
        }

    def frame(self, scene: str, frame_id: str) -> dict[str, Any]:
        try:
            return self.frames[(scene, frame_id)]
        except KeyError as exc:
            raise FileNotFoundError(f"找不到帧：{scene}/{frame_id}") from exc


def find_named_file(names: list[str], prefix: str) -> str | None:
    return next((name for name in names if str(name).startswith(prefix)), None)


def frame_index(rgb_name: str | None, fallback: int) -> str:
    if rgb_name:
        stem = Path(rgb_name).stem
        if stem.startswith("rgb_"):
            return stem[4:]
    return str(fallback)


def scene_sort_key(path: Path) -> tuple[int, str]:
    suffix = path.name.removeprefix("Scene_")
    return (int(suffix), path.name) if suffix.isdigit() else (2**31 - 1, path.name)


@lru_cache(maxsize=512)
def render_asset(kind: str, path_text: str, far_clip: float) -> bytes:
    path = Path(path_text)
    if kind == "rgb":
        return path.read_bytes()

    if kind == "thumb":
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"无法读取 RGB：{path}")
        height, width = image.shape[:2]
        target_width = 240
        target_height = max(1, round(height * target_width / max(width, 1)))
        image = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)
        return encode_png(image)

    if kind == "depth":
        depth = cv2.imread(str(path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        if depth is None:
            raise ValueError(f"无法读取 Depth EXR：{path}")
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        depth = np.nan_to_num(depth.astype(np.float32), nan=far_clip, posinf=far_clip, neginf=0.0)
        finite_max = float(np.max(depth)) if depth.size else 0.0
        normalized = np.clip(
            depth if finite_max <= 1.5 else depth / max(far_clip, 1e-6),
            0.0,
            1.0,
        )
        colored = cv2.applyColorMap(((1.0 - normalized) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        return encode_png(colored)

    if kind in ("semantic", "semantic_overlay"):
        semantic = np.load(path).astype(np.float32)
        if semantic.ndim > 2:
            semantic = np.squeeze(semantic)
        semantic = np.nan_to_num(semantic, nan=0.0, posinf=1.0, neginf=0.0)
        normalized = (np.clip(semantic, 0.0, 1.0) * 255).astype(np.uint8)
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_INFERNO)
        if kind == "semantic_overlay":
            colored = cv2.cvtColor(colored, cv2.COLOR_BGR2BGRA)
            colored[:, :, 3] = normalized
        return encode_png(colored)

    raise ValueError(f"未知图像类型：{kind}")


def encode_png(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("PNG 编码失败")
    return encoded.tobytes()


@lru_cache(maxsize=512)
def semantic_stats(path_text: str) -> dict[str, float]:
    semantic = np.load(path_text).astype(np.float32)
    finite = semantic[np.isfinite(semantic)]
    if finite.size == 0:
        return {"min": 0.0, "max": 0.0, "mean": 0.0}
    return {
        "min": float(finite.min()),
        "max": float(finite.max()),
        "mean": float(finite.mean()),
    }


class InspectorServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], catalog: DatasetCatalog) -> None:
        super().__init__(address, InspectorHandler)
        self.catalog = catalog


class InspectorHandler(SimpleHTTPRequestHandler):
    server: InspectorServer

    def do_GET(self) -> None:
        request = urlparse(self.path)
        try:
            if request.path == "/api/index":
                params = parse_qs(request.query)
                if params.get("refresh") == ["1"]:
                    self.server.catalog.reload()
                    render_asset.cache_clear()
                    semantic_stats.cache_clear()
                self.send_json(self.server.catalog.payload)
                return
            if request.path == "/api/asset":
                self.send_asset(parse_qs(request.query))
                return
            if request.path == "/api/frame-stats":
                self.send_frame_stats(parse_qs(request.query))
                return
            if request.path in ("/", "/index.html"):
                self.send_file(TOOLS_DIR / "data_inspector.html", "text/html; charset=utf-8")
                return
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
        except FileNotFoundError as exc:
            self.send_error(HTTPStatus.NOT_FOUND, str(exc))
        except (ValueError, OSError) as exc:
            # HTTP reason phrases are Latin-1 in BaseHTTPRequestHandler; keep
            # the status line ASCII-safe while preserving the diagnostic text.
            detail = str(exc).encode("ascii", "backslashreplace").decode("ascii")
            self.send_error(HTTPStatus.UNPROCESSABLE_ENTITY, "invalid request", detail)

    def send_asset(self, params: dict[str, list[str]]) -> None:
        scene = first_param(params, "scene")
        frame_id = first_param(params, "frame")
        kind = first_param(params, "kind")
        frame = self.server.catalog.frame(scene, frame_id)
        source_kind = "rgb" if kind == "thumb" else kind
        if kind == "semantic_overlay":
            source_kind = "semantic"
        if source_kind not in ("rgb", "depth", "semantic"):
            raise ValueError(f"未知图像类型：{kind}")
        path = frame["_paths"].get(source_kind)
        if not path or not path.is_file():
            raise FileNotFoundError(f"缺少 {source_kind}：{scene}/{frame_id}")
        scene_data = next(
            item for item in self.server.catalog.payload["scenes"] if item["name"] == scene
        )
        far_clip = float(scene_data["camera"].get("farClip") or 20.0)
        body = render_asset(kind, str(path), far_clip)
        content_type = "image/png"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_frame_stats(self, params: dict[str, list[str]]) -> None:
        scene = first_param(params, "scene")
        frame_id = first_param(params, "frame")
        frame = self.server.catalog.frame(scene, frame_id)
        path = frame["_paths"].get("semantic")
        if not path or not path.is_file():
            raise FileNotFoundError(f"缺少 semantic：{scene}/{frame_id}")
        self.send_json({"semantic": semantic_stats(str(path))})

    def send_json(self, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path: Path, content_type: str) -> None:
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message: str, *args: Any) -> None:
        if self.path.startswith("/api/asset") and len(args) > 1 and str(args[1]) == "200":
            return
        if self.path == "/favicon.ico":
            return
        print(f"[{self.log_date_time_string()}] {message % args}")


def first_param(params: dict[str, list[str]], name: str) -> str:
    values = params.get(name)
    if not values or not values[0]:
        raise ValueError(f"缺少参数：{name}")
    return values[0]


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenSeek 数据可视化检查工具")
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA, help="TrainingData 或 TestingData 目录")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    catalog = DatasetCatalog(args.data)
    server = InspectorServer((args.host, args.port), catalog)
    print(f"数据目录：{catalog.root}")
    print(
        f"已索引：{catalog.payload['sceneCount']} 个场景，"
        f"{catalog.payload['frameCount']} 帧，"
        f"{catalog.payload['visibleCount']} 帧目标可见"
    )
    print(f"浏览器地址：http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n数据检查工具已停止。")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
