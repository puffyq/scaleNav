from __future__ import annotations

import argparse
import base64
import html
import os
from functools import lru_cache
from pathlib import Path

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import cv2
import numpy as np
import open3d as o3d
import plotly.graph_objects as go
import plotly.io as pio
import rtoml
import torch

from config.config import cfg
from policy.poly_solver import Poly5Solver
from text_tracker.dataset import TextYopoDataset
from text_tracker.heatmap import pearl_similarity
from text_tracker.loss import TextYopoGuidanceLoss


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = PROJECT_ROOT / "data" / "TestingData"
DEFAULT_MODEL = PROJECT_ROOT / "saved" / "map4_goal" / "best" / "text_yopo.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an interactive OpenSeek baseline visualization."
    )
    parser.add_argument("--data", default=str(DEFAULT_DATA))
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--scene", help="Scene name, for example Scene_10")
    parser.add_argument("--frame", help="Frame number from depth_N.exr")
    parser.add_argument("--mode", choices=("search", "approach"), default="approach")
    parser.add_argument("--semantic-weight", type=float, default=1.2)
    parser.add_argument("--radius", type=float, default=15.0)
    parser.add_argument("--max-points", type=int, default=12000)
    parser.add_argument("--trajectory-points", type=int, default=80)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--output", default="text_yopo_offline_test.html")
    return parser.parse_args()


def choose_device(name: str) -> torch.device:
    if name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if name == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    return torch.device(name)


def frame_id(record: dict) -> str:
    return Path(record["depth_path"]).stem.removeprefix("depth_")


def scene_name(record: dict) -> str:
    return Path(record["depth_path"]).parent.parent.name


def select_record(dataset: TextYopoDataset, scene: str | None, frame: str | None) -> int:
    available_scenes = sorted({scene_name(record) for record in dataset.records})
    selected_scene = scene or available_scenes[0]
    candidates = [
        index
        for index, record in enumerate(dataset.records)
        if scene_name(record) == selected_scene
    ]
    if not candidates:
        raise ValueError(
            f"Scene {selected_scene!r} was not found; available: {', '.join(available_scenes)}"
        )
    if frame is not None:
        match = next(
            (index for index in candidates if frame_id(dataset.records[index]) == str(frame)),
            None,
        )
        if match is None:
            raise ValueError(f"Frame {frame!r} was not found in {selected_scene}")
        return match
    return next(
        (
            index
            for index in candidates
            if dataset.records[index]["metadata"].get("targetVisible", False)
        ),
        candidates[0],
    )


def move_sample(sample: dict[str, torch.Tensor], device: torch.device) -> dict[str, torch.Tensor]:
    return {key: value.unsqueeze(0).to(device) for key, value in sample.items()}


def load_prediction(
    model_path: Path,
    batch: dict[str, torch.Tensor],
    device: torch.device,
) -> tuple[torch.Tensor, torch.Tensor]:
    if not model_path.is_file():
        raise FileNotFoundError(
            f"OpenSeek baseline TorchScript model not found: {model_path}\n"
            "Train the current dual-heatmap model first or pass --model."
        )
    model = torch.jit.load(str(model_path), map_location=device).eval()
    with torch.inference_mode():
        output = model(batch["image"], batch["obs"])
    if not isinstance(output, (tuple, list)) or len(output) != 2:
        raise ValueError(
            "The model is not compatible with the current Text YOPO interface. "
            "Expected (endstate, score); old person/objectness checkpoints cannot be used."
        )
    endstate, score = output
    if endstate.ndim != 4 or endstate.shape[1] != 9 or score.ndim != 3:
        raise ValueError(
            f"Unexpected model outputs: endstate={tuple(endstate.shape)}, "
            f"score={tuple(score.shape)}"
        )
    return endstate, score


def sample_trajectories(endstates: np.ndarray, motion: np.ndarray, count: int) -> list[np.ndarray]:
    times = np.linspace(0.0, float(cfg["sgm_time"]), count)
    trajectories = []
    for endstate in endstates:
        axes = [
            Poly5Solver(
                0.0,
                float(motion[axis]),
                float(motion[axis + 3]),
                float(endstate[axis]),
                float(endstate[axis + 3]),
                float(endstate[axis + 6]),
                float(cfg["sgm_time"]),
            )
            for axis in range(3)
        ]
        trajectories.append(
            np.stack(
                [[solver.get_position(time) for time in times] for solver in axes],
                axis=1,
            ).astype(np.float32)
        )
    return trajectories


def nearby_point_cloud(
    path: Path,
    position: np.ndarray,
    rotation_world_body: np.ndarray,
    radius: float,
    max_points: int,
) -> np.ndarray:
    points = read_point_cloud(str(path))
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    local = (points - position[None, :]) @ rotation_world_body
    mask = (
        (np.linalg.norm(local[:, :2], axis=1) <= radius)
        & (local[:, 2] >= -2.0)
        & (local[:, 2] <= max(8.0, radius * 0.75))
    )
    local = local[mask]
    if local.shape[0] > max_points:
        indices = np.random.default_rng(0).choice(
            local.shape[0], size=max_points, replace=False
        )
        local = local[indices]
    return local


def depth_point_cloud(
    path: Path,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    radius: float,
    max_points: int,
    far_clip_m: float = 20.0,
) -> np.ndarray:
    """Back-project an AirSim DepthPlanar image into body-FLU coordinates."""
    depth = cv2.imread(str(path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    if depth is None:
        raise FileNotFoundError(path)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    depth = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    height, width = depth.shape
    if width < 2 or height < 2:
        return np.empty((0, 3), dtype=np.float32)
    fx = (width - 1) * 0.5 / np.tan(np.deg2rad(horizontal_fov_deg) * 0.5)
    fy = (height - 1) * 0.5 / np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    rows, columns = np.indices((height, width), dtype=np.float32)
    valid = (depth > 0.05) & (depth <= max(float(far_clip_m), 0.05))
    camera_forward = depth
    camera_right = (columns - cx) * camera_forward / fx
    camera_down = (rows - cy) * camera_forward / fy
    # AirSim camera FRD -> YOPO body FLU: x stays forward, right/down flip.
    local = np.stack((camera_forward, -camera_right, -camera_down), axis=-1)
    valid &= np.linalg.norm(local[:, :, :2], axis=2) <= radius
    valid &= (local[:, :, 2] >= -2.0) & (local[:, :, 2] <= max(8.0, radius * 0.75))
    local = local[valid]
    if local.shape[0] > max_points:
        indices = np.random.default_rng(0).choice(
            local.shape[0], size=max_points, replace=False
        )
        local = local[indices]
    return local.astype(np.float32, copy=False)


@lru_cache(maxsize=16)
def read_point_cloud(path: str) -> np.ndarray:
    cloud = o3d.io.read_point_cloud(path)
    return np.asarray(cloud.points, dtype=np.float32).copy()


def target_body_vector(sample: dict[str, torch.Tensor], metadata: dict, mode: str) -> np.ndarray:
    return sample["numeric_goal"].numpy().astype(np.float32)


def heatmap_peak_body_vector(
    heatmap: np.ndarray,
    horizontal_fov_deg: float,
    distance: float = 10.0,
) -> np.ndarray:
    """Convert the strongest positive heatmap pixel to a YOPO body direction."""
    values = np.asarray(heatmap, dtype=np.float32)
    if values.ndim != 2 or values.size == 0 or not np.isfinite(values).all():
        return np.zeros(3, dtype=np.float32)
    _, column = np.unravel_index(int(values.argmax()), values.shape)
    center = (values.shape[1] - 1) * 0.5
    focal = center / np.tan(np.deg2rad(horizontal_fov_deg) * 0.5)
    azimuth = np.arctan2(center - float(column), focal)
    return np.array(
        [distance * np.cos(azimuth), distance * np.sin(azimuth), 0.0],
        dtype=np.float32,
    )


def png_data_uri(image: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("Could not encode visualization image")
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:image/png;base64,{payload}"


def file_data_uri(path: Path) -> str:
    payload = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:image/png;base64,{payload}"


def depth_image(path: Path, far_clip: float) -> tuple[np.ndarray, np.ndarray]:
    depth = cv2.imread(str(path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    if depth is None:
        raise FileNotFoundError(path)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    depth = np.nan_to_num(
        depth.astype(np.float32), nan=1.0, posinf=1.0, neginf=0.0
    )
    normalized = depth if float(depth.max()) <= 1.5 else depth / max(far_clip, 1e-6)
    normalized = np.clip(normalized, 0.0, 1.0)
    colored = cv2.applyColorMap(
        ((1.0 - normalized) * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO
    )
    return normalized, colored


def signed_heatmap_image(heatmap: np.ndarray) -> np.ndarray:
    values = np.nan_to_num(
        heatmap.astype(np.float32), nan=0.0, posinf=1.0, neginf=-1.0
    )
    # PEARL is an absolute probability (usually below 0.2), while search mode
    # uses the signed [-1, 1] numeric goal map. Keep both contracts readable.
    if float(values.min()) >= -1e-6:
        probability = np.clip(values, 0.0, 1.0)
        scale = np.clip(probability / 0.2, 0.0, 1.0)
        return cv2.applyColorMap(
            (scale * 255.0).astype(np.uint8), cv2.COLORMAP_INFERNO
        )
    values = np.clip(values, -1.0, 1.0)
    positive = np.clip(values, 0.0, 1.0)[..., None]
    negative = np.clip(-values, 0.0, 1.0)[..., None]
    neutral = np.full((*values.shape, 3), 245.0, dtype=np.float32)
    red = np.array([45.0, 70.0, 220.0], dtype=np.float32)
    blue = np.array([210.0, 95.0, 35.0], dtype=np.float32)
    bgr = neutral * (1.0 - positive) + red * positive
    bgr = bgr * (1.0 - negative) + blue * negative
    return np.clip(bgr, 0.0, 255.0).astype(np.uint8)


def make_figure(
    point_cloud: np.ndarray,
    trajectories: list[np.ndarray],
    predicted_best: int,
    oracle_best: int,
    predicted_scores: np.ndarray,
    true_costs: np.ndarray,
    target: np.ndarray,
    radius: float,
    *,
    target_label: str = "目标真值方向",
    guidance: np.ndarray | None = None,
) -> go.Figure:
    figure = go.Figure()
    if point_cloud.size:
        figure.add_trace(
            go.Scatter3d(
                x=point_cloud[:, 0],
                y=point_cloud[:, 1],
                z=point_cloud[:, 2],
                mode="markers",
                name="深度点云（机体系 FLU）",
                marker={"size": 1.4, "color": "#59636b", "opacity": 0.38},
                hoverinfo="skip",
            )
        )
    for index, trajectory in enumerate(trajectories):
        figure.add_trace(
            go.Scatter3d(
                x=trajectory[:, 0],
                y=trajectory[:, 1],
                z=trajectory[:, 2],
                mode="lines",
                name=f"候选 {index}",
                line={"width": 3, "color": "#a7afb5"},
                opacity=0.7,
                text=(
                    f"候选 {index}<br>预测 score={predicted_scores[index]:.4f}"
                    f"<br>真实 cost={true_costs[index]:.4f}"
                ),
                hovertemplate="%{text}<extra></extra>",
                showlegend=False,
            )
        )
    predicted = trajectories[predicted_best]
    figure.add_trace(
        go.Scatter3d(
            x=predicted[:, 0],
            y=predicted[:, 1],
            z=predicted[:, 2],
            mode="lines",
            name=f"预测最优 #{predicted_best}",
            line={"width": 8, "color": "#d33f49"},
        )
    )
    oracle = trajectories[oracle_best]
    figure.add_trace(
        go.Scatter3d(
            x=oracle[:, 0],
            y=oracle[:, 1],
            z=oracle[:, 2],
            mode="lines",
            name=f"真实代价 oracle #{oracle_best}",
            line={"width": 7, "color": "#00897b", "dash": "dash"},
        )
    )
    target_distance = float(np.linalg.norm(target))
    if target_distance > 1e-6:
        target_display = target * min(1.0, radius / target_distance)
        figure.add_trace(
            go.Scatter3d(
                x=[0.0, target_display[0]],
                y=[0.0, target_display[1]],
                z=[0.0, target_display[2]],
                mode="lines+markers",
                name=f"{target_label} ({target_distance:.2f} m)",
                line={"width": 7, "color": "#e58b19", "dash": "dot"},
                marker={"size": [3, 7], "symbol": ["circle", "diamond"]},
            )
        )
    if guidance is not None:
        guidance_distance = float(np.linalg.norm(guidance))
        if guidance_distance > 1e-6:
            guidance_display = guidance * min(1.0, radius / guidance_distance)
            figure.add_trace(
                go.Scatter3d(
                    x=[0.0, guidance_display[0]],
                    y=[0.0, guidance_display[1]],
                    z=[0.0, guidance_display[2]],
                    mode="lines+markers",
                    name="网络 Heatmap 峰值方向",
                    line={"width": 6, "color": "#2563a6", "dash": "dash"},
                    marker={"size": [3, 6], "symbol": ["circle", "circle"]},
                )
            )
    figure.add_trace(
        go.Scatter3d(
            x=[0.0],
            y=[0.0],
            z=[0.0],
            mode="markers",
            name="当前位置",
            marker={"size": 7, "color": "#15191c", "symbol": "diamond"},
        )
    )
    figure.update_layout(
        height=720,
        margin={"l": 0, "r": 0, "t": 14, "b": 0},
        paper_bgcolor="#ffffff",
        plot_bgcolor="#ffffff",
        legend={"orientation": "h", "y": 1.02, "x": 0.0},
        scene={
            "xaxis": {"title": "X 前方 (m)", "range": [-radius * 0.15, radius]},
            "yaxis": {"title": "Y 左方 (m)", "range": [-radius * 0.65, radius * 0.65]},
            "zaxis": {"title": "Z 上方 (m)", "range": [-2.0, max(8.0, radius * 0.75)]},
            "aspectmode": "data",
            # Default top-down view keeps body-frame X-forward/Y-left directions
            # visually unambiguous; the Plotly toolbar still allows 3-D rotation.
            "camera": {
                "eye": {"x": 0.0, "y": 0.0, "z": 2.4},
                "up": {"x": 0.0, "y": 1.0, "z": 0.0},
                "projection": {"type": "orthographic"},
            },
        },
    )
    return figure


def score_table(
    predicted: np.ndarray,
    components: dict[str, np.ndarray],
    predicted_best: int,
    oracle_best: int,
) -> str:
    rows = []
    for index in range(predicted.size):
        classes = []
        if index == predicted_best:
            classes.append("predicted")
        if index == oracle_best:
            classes.append("oracle")
        rows.append(
            "<tr class=\"{}\"><td>{}</td><td>{:.4f}</td><td>{:.4f}</td>"
            "<td>{:.4f}</td><td>{:.4f}</td><td>{:.4f}</td><td>{:.4f}</td>"
            "<td>{:.4f}</td><td>{:.4f}</td></tr>".format(
                " ".join(classes),
                index,
                predicted[index],
                components["total"][index],
                components["endpoint_distance"][index],
                components["smooth"][index],
                components["safety"][index],
                components["acceleration"][index],
                components["heatmap_value"][index],
                components["semantic_cost"][index],
            )
        )
    return "".join(rows)


def render_html(
    *,
    plot_html: str,
    rgb_uri: str,
    depth_uri: str,
    heatmap_uri: str,
    rows: str,
    metadata: dict,
    scene: str,
    frame: str,
    mode: str,
    predicted_best: int,
    oracle_best: int,
    heatmap: np.ndarray,
    target: np.ndarray,
    point_count: int,
    model_path: Path,
) -> str:
    prompt = html.escape(str(metadata.get("targetPrompt", "")))
    visible = "是" if metadata.get("targetVisible", False) else "否"
    agreement = "一致" if predicted_best == oracle_best else "不一致"
    target_distance = float(np.linalg.norm(target))
    confidence = pearl_similarity(heatmap) if mode == "approach" else float(heatmap.max())
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Text YOPO 离线测试</title>
<style>
  :root {{ color-scheme: light; font-family: Inter, "Noto Sans SC", system-ui, sans-serif; color: #202529; background: #f4f6f7; }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; }}
  header {{ display: flex; align-items: baseline; justify-content: space-between; gap: 20px; padding: 18px 24px 14px; background: #fff; border-bottom: 1px solid #dce1e4; }}
  h1 {{ margin: 0; font-size: 20px; letter-spacing: 0; }}
  .source {{ color: #687178; font: 12px ui-monospace, monospace; overflow-wrap: anywhere; }}
  main {{ max-width: 1680px; margin: 0 auto; padding: 18px 24px 28px; }}
  .metrics {{ display: grid; grid-template-columns: repeat(8, minmax(110px, 1fr)); gap: 1px; background: #dce1e4; border: 1px solid #dce1e4; border-radius: 6px; overflow: hidden; margin-bottom: 16px; }}
  .metric {{ min-width: 0; padding: 10px 12px; background: #fff; }}
  .metric span {{ display: block; color: #707980; font-size: 11px; margin-bottom: 4px; }}
  .metric strong {{ display: block; font-size: 14px; overflow-wrap: anywhere; }}
  .inputs {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 14px; margin-bottom: 16px; }}
  figure {{ margin: 0; background: #fff; border: 1px solid #dce1e4; border-radius: 6px; overflow: hidden; }}
  figure img {{ display: block; width: 100%; height: 230px; object-fit: contain; background: #111; }}
  #heatmapImage {{ object-fit: fill; image-rendering: pixelated; background: #f5f5f5; }}
  figcaption {{ padding: 9px 12px; font-size: 13px; border-top: 1px solid #e4e8ea; }}
  .workspace {{ display: grid; grid-template-columns: minmax(0, 1fr) 430px; gap: 14px; align-items: start; }}
  .plot, .scores {{ background: #fff; border: 1px solid #dce1e4; border-radius: 6px; overflow: hidden; }}
  .scores h2 {{ margin: 0; padding: 13px 14px; font-size: 15px; border-bottom: 1px solid #e4e8ea; }}
  table {{ width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; font-size: 12px; }}
  th, td {{ padding: 8px 7px; text-align: right; border-bottom: 1px solid #edf0f2; white-space: nowrap; }}
  th:first-child, td:first-child {{ text-align: center; }}
  th {{ color: #667078; font-weight: 600; background: #fafbfb; }}
  tr.predicted td {{ background: #fff0f1; }}
  tr.oracle td {{ box-shadow: inset 0 -2px #00897b; }}
  .note {{ margin: 0; padding: 12px 14px; color: #59636b; font-size: 12px; line-height: 1.55; }}
  @media (max-width: 1100px) {{
    .metrics {{ grid-template-columns: repeat(4, 1fr); }}
    .workspace {{ grid-template-columns: 1fr; }}
  }}
  @media (max-width: 720px) {{
    header {{ align-items: flex-start; flex-direction: column; }}
    main {{ padding: 12px; }}
    .metrics {{ grid-template-columns: repeat(2, 1fr); }}
    .inputs {{ grid-template-columns: 1fr; }}
    figure img {{ height: auto; max-height: 320px; }}
    .scores {{ overflow-x: auto; }}
  }}
</style>
</head>
<body>
<header><h1>Text YOPO 离线测试</h1><div class="source">{html.escape(str(model_path))}</div></header>
<main>
  <section class="metrics">
    <div class="metric"><span>场景 / 帧</span><strong>{html.escape(scene)} / {html.escape(frame)}</strong></div>
    <div class="metric"><span>模式</span><strong>{html.escape(mode)}</strong></div>
    <div class="metric"><span>Prompt</span><strong>{prompt or '-'}</strong></div>
    <div class="metric"><span>目标可见真值</span><strong>{visible}</strong></div>
    <div class="metric"><span>目标距离</span><strong>{target_distance:.2f} m</strong></div>
    <div class="metric"><span>Heatmap 强度</span><strong>{confidence:.4f}</strong></div>
    <div class="metric"><span>预测 / Oracle</span><strong>#{predicted_best} / #{oracle_best} ({agreement})</strong></div>
    <div class="metric"><span>深度点云（机体系 FLU）</span><strong>{point_count:,} 点</strong></div>
  </section>
  <section class="inputs">
    <figure><img src="{rgb_uri}" alt="RGB input"><figcaption>前视 RGB（仅 PEARL 使用）</figcaption></figure>
    <figure><img src="{depth_uri}" alt="Depth input"><figcaption>Depth 输入，固定范围；暖色更近</figcaption></figure>
    <figure><img src="{heatmap_uri}" alt="Heatmap input"><figcaption>网络 Heatmap 输入，固定范围 [-1, 1]</figcaption></figure>
  </section>
  <section class="workspace">
    <div class="plot">{plot_html}</div>
    <aside class="scores">
      <h2>候选轨迹分数</h2>
      <table><thead><tr><th>#</th><th>预测</th><th>真实</th><th>终点距离(m)</th><th>平滑</th><th>安全</th><th>加速度</th><th>Heatmap值</th><th>Heatmap代价</th></tr></thead><tbody>{rows}</tbody></table>
      <p class="note">红色行为模型预测最低 score；绿色下划线行为按 ESDF、平滑、加速度和 heatmap 重新计算后的最低真实代价。测试集没有保存专家轨迹，因此绿色 3-D 轨迹是候选集合内的 cost oracle，不是人工轨迹标签。</p>
    </aside>
  </section>
</main>
</body>
</html>"""


def main() -> None:
    args = parse_args()
    if args.radius <= 0.0 or args.max_points <= 0 or args.trajectory_points < 2:
        raise ValueError("--radius/--max-points must be positive and --trajectory-points >= 2")
    device = choose_device(args.device)
    approach_probability = 1.0 if args.mode == "approach" else 0.0
    dataset = TextYopoDataset(
        args.data,
        approach_probability=approach_probability,
        pearl_enter_threshold=0.0,
    )
    index = select_record(dataset, args.scene, args.frame)
    record = dataset.records[index]
    sample = dataset[index]
    batch = move_sample(sample, device)

    endstate, predicted_score = load_prediction(Path(args.model), batch, device)
    cost = TextYopoGuidanceLoss(
        dataset.scene_obstacles,
        semantic_weight=args.semantic_weight,
        device=device,
    ).to(device)
    with torch.inference_mode():
        true_cost, component_tensors = cost.trajectory_costs(endstate, batch)

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
        endstates, sample["obs"].numpy(), args.trajectory_points
    )

    scene_dir = Path(record["depth_path"]).parent.parent
    points = depth_point_cloud(
        Path(record["depth_path"]),
        float(record["horizontal_fov"]),
        60.0,
        args.radius,
        args.max_points,
        20.0,
    )
    metadata = record["metadata"]
    target = target_body_vector(sample, metadata, args.mode)
    figure = make_figure(
        points,
        trajectories,
        predicted_best,
        oracle_best,
        predicted_scores,
        true_costs,
        target,
        args.radius,
    )
    plot_html = pio.to_html(
        figure,
        full_html=False,
        include_plotlyjs=True,
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )

    document = rtoml.load(scene_dir / "data.toml")
    far_clip = float(document.get("depthCameraFarClipPlane", 20.0))
    _, depth_panel = depth_image(Path(record["depth_path"]), far_clip)
    heatmap = sample["image"][1].numpy()
    display_heatmap = heatmap
    semantic_path = record.get("semantic_path")
    if semantic_path is not None:
        raw_heatmap = np.load(semantic_path).astype(np.float32)
        if raw_heatmap.ndim == 2 and np.isfinite(raw_heatmap).all():
            display_heatmap = raw_heatmap
    heatmap_panel = signed_heatmap_image(display_heatmap)
    rgb_name = metadata.get("rgbFileName")
    if not rgb_name:
        raise FileNotFoundError("The selected frame has no RGB image")
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        render_html(
            plot_html=plot_html,
            rgb_uri=file_data_uri(scene_dir / "Textures" / rgb_name),
            depth_uri=png_data_uri(depth_panel),
            heatmap_uri=png_data_uri(heatmap_panel),
            rows=score_table(
                predicted_scores, components, predicted_best, oracle_best
            ),
            metadata=metadata,
            scene=scene_name(record),
            frame=frame_id(record),
            mode=args.mode,
            predicted_best=predicted_best,
            oracle_best=oracle_best,
            heatmap=display_heatmap,
            target=target,
            point_count=points.shape[0],
            model_path=Path(args.model).resolve(),
        ),
        encoding="utf-8",
    )
    print(output_path)


if __name__ == "__main__":
    main()
