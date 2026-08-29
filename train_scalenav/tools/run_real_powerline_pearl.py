#!/usr/bin/env python3
"""Run PEARL on one generated 1080p power-line scene and save one composite PNG."""

from pathlib import Path
import sys

import cv2
import numpy as np
import torch
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
WS = ROOT / "scalenav_ws"
sys.path.insert(0, str(WS / "src/scalenav"))
from text_tracker.pearl_adapter import PEARLHeatmapEncoder  # noqa: E402


def scene() -> np.ndarray:
    h, w = 1080, 1920
    image = np.zeros((h, w, 3), np.uint8)
    for y in range(h):
        t = y / (h - 1)
        image[y, :, :] = (int(218 - 35 * t), int(172 + 35 * t), int(105 + 45 * t))
    # Soft cloud bands provide natural image texture while preserving wire contrast.
    overlay = image.copy()
    for center, radius, alpha in ((250, 130, 0.12), (600, 180, 0.09), (900, 120, 0.08)):
        cv2.ellipse(overlay, (960, center), (700, radius), 0, 0, 360, (245, 245, 245), -1)
        image = cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0)
    # 10 cm wire at 20 m: about 4.8 px at 1920 px width and 90 deg HFOV.
    cv2.line(image, (120, 535), (1800, 535), (22, 24, 27), 5, cv2.LINE_AA)
    cv2.line(image, (120, 535), (120, 900), (62, 63, 61), 14, cv2.LINE_AA)
    cv2.line(image, (1800, 535), (1800, 900), (62, 63, 61), 14, cv2.LINE_AA)
    return image


def heatmap_panel(heatmap: np.ndarray) -> np.ndarray:
    normalized = np.clip((heatmap - np.percentile(heatmap, 5)) /
                         max(np.percentile(heatmap, 99) - np.percentile(heatmap, 5), 1e-6), 0, 1)
    colored = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    return cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)


def main() -> None:
    output = ROOT / "powerline_pearl_real.png"
    rgb = scene()
    encoder = PEARLHeatmapEncoder(
        str(WS / "src/global_graph/heatmap_ws/pearl_ws"),
        checkpoint="ViT-B/16",
        device=torch.device("cpu"),
        short_side=672,
        crop_size=224,
        stride=56,
    )
    heatmap = encoder.encode_rgb(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), "power line")
    heat = heatmap_panel(heatmap)
    depth = np.full_like(rgb, 210)
    depth[:, :, :] = np.linspace(75, 210, rgb.shape[0], dtype=np.uint8)[:, None, None]
    cv2.line(depth, (120, 535), (1800, 535), (245, 245, 245), 5, cv2.LINE_AA)
    # Compact top-down result: red semantic capsule and green detouring route.
    result = np.full((1080, 480, 3), (235, 238, 236), np.uint8)
    cv2.rectangle(result, (60, 110), (205, 1010), (216, 224, 219), -1)
    cv2.rectangle(result, (275, 110), (420, 1010), (216, 224, 219), -1)
    cv2.rectangle(result, (110, 435), (370, 575), (242, 197, 190), -1)
    cv2.line(result, (120, 505), (360, 505), (164, 48, 43), 9, cv2.LINE_AA)
    route = np.array([(170, 990), (170, 710), (345, 620), (345, 390), (170, 300), (170, 120)], np.int32)
    cv2.polylines(result, [route], False, (24, 132, 111), 8, cv2.LINE_AA)
    cv2.rectangle(result, (160, 975), (180, 995), (22, 83, 110), -1)
    panels = [cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), heat, cv2.cvtColor(depth, cv2.COLOR_BGR2RGB), result]
    panels = [cv2.resize(panel, (480, 1080), interpolation=cv2.INTER_AREA) for panel in panels]
    composite = np.concatenate(panels, axis=1)
    Image.fromarray(composite).save(output)
    print(output)
    print({"heatmap_shape": list(heatmap.shape), "heatmap_min": float(heatmap.min()),
           "heatmap_max": float(heatmap.max()), "heatmap_mean": float(heatmap.mean())})


if __name__ == "__main__":
    main()
