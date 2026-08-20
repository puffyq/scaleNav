import argparse
import os
from pathlib import Path

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import cv2
import numpy as np
import rtoml


def normalize(values: np.ndarray, invert: bool = False) -> np.ndarray:
    values = np.nan_to_num(values.astype(np.float32))
    finite = values[np.isfinite(values)]
    low, high = np.percentile(finite, [2.0, 98.0])
    scaled = np.clip((values - low) / max(float(high - low), 1e-6), 0.0, 1.0)
    if invert:
        scaled = 1.0 - scaled
    return (scaled * 255.0).astype(np.uint8)


def title(panel: np.ndarray, text: str) -> None:
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(
        panel, text, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
        (255, 255, 255), 2, cv2.LINE_AA
    )


def draw_target(panel: np.ndarray, metadata: dict) -> None:
    if not metadata.get("targetVisible", False):
        return
    pixel = metadata.get("targetPixel", [0.0, 0.0])
    x = int(float(pixel[0]) / 160.0 * panel.shape[1])
    y = int(float(pixel[1]) / 90.0 * panel.shape[0])
    cv2.drawMarker(
        panel, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 24, 3, cv2.LINE_AA
    )
    cv2.circle(panel, (x, y), 15, (0, 0, 0), 2, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize synchronized Text YOPO RGB, depth, and SCLIP data."
    )
    parser.add_argument("--scene", required=True)
    parser.add_argument("--index", type=int)
    parser.add_argument("--output", default="text_yopo_visualization.png")
    parser.add_argument("--hide-target", action="store_true")
    args = parser.parse_args()

    scene = Path(args.scene)
    document = rtoml.load(scene / "data.toml")
    records = document.get("dataArray", [])
    if args.index is None:
        metadata = next((item for item in records if item.get("targetVisible")), records[0])
    else:
        expected = f"rgb_{args.index}.png"
        metadata = next(
            (item for item in records if item.get("rgbFileName") == expected), None
        )
        if metadata is None:
            raise ValueError(f"Frame {args.index} was not found in {scene / 'data.toml'}")

    textures = scene / "Textures"
    rgb_name = metadata["rgbFileName"]
    depth_name = metadata["depthFileName"]
    frame_index = Path(rgb_name).stem.removeprefix("rgb_")
    semantic_path = textures / f"semantic_raw_{frame_index}.npy"

    rgb = cv2.imread(str(textures / rgb_name), cv2.IMREAD_COLOR)
    depth = cv2.imread(
        str(textures / depth_name), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH
    )
    semantic = np.load(semantic_path)
    if rgb is None or depth is None:
        raise FileNotFoundError("Could not load the synchronized RGB-D frame")
    if depth.ndim == 3:
        depth = depth[:, :, 0]

    size = (rgb.shape[1], rgb.shape[0])
    depth_image = cv2.applyColorMap(normalize(depth, invert=True), cv2.COLORMAP_TURBO)
    depth_image = cv2.resize(depth_image, size, interpolation=cv2.INTER_NEAREST)
    semantic_fixed = np.clip(semantic.astype(np.float32) / 0.35, 0.0, 1.0)
    semantic_image = cv2.applyColorMap(
        (semantic_fixed * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO
    )
    semantic_image = cv2.resize(semantic_image, size, interpolation=cv2.INTER_CUBIC)
    overlay = cv2.addWeighted(rgb, 0.55, semantic_image, 0.45, 0.0)

    panels = [rgb.copy(), depth_image, semantic_image, overlay]
    labels = [
        f"RGB | frame {frame_index}",
        "Depth | warm = near",
        "Raw SCLIP cosine | fixed [0, .35]",
        "RGB + SCLIP",
    ]
    for panel, label in zip(panels, labels):
        if not args.hide_target:
            draw_target(panel, metadata)
        title(panel, label)

    canvas = np.vstack(
        [np.hstack(panels[:2]), np.hstack(panels[2:])]
    )
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas):
        raise OSError(f"Could not write {output}")
    print(output)


if __name__ == "__main__":
    main()
