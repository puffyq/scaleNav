from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rtoml
import torch

from text_tracker.sclip_adapter import SCLIPHeatmapEncoder


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Precompute raw SCLIP patch-cosine heatmaps."
    )
    parser.add_argument("--data", required=True)
    parser.add_argument(
        "--sclip-root", default=str(PROJECT_ROOT / "third_party" / "SCLIP")
    )
    parser.add_argument("--checkpoint", default="ViT-B/16")
    parser.add_argument("--prompt", default="person")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def precompute(args: argparse.Namespace) -> None:
    encoder = SCLIPHeatmapEncoder(
        args.sclip_root,
        checkpoint=args.checkpoint,
        device=torch.device("cuda" if torch.cuda.is_available() else "cpu"),
    )
    for toml_path in sorted(Path(args.data).glob("Scene_*/data.toml")):
        document = rtoml.load(toml_path)
        texture_dir = toml_path.parent / "Textures"
        for item in document.get("dataArray", []):
            rgb_name = item.get("rgbFileName")
            if not rgb_name:
                continue
            index = Path(rgb_name).stem.removeprefix("rgb_")
            output_path = texture_dir / f"semantic_raw_{index}.npy"
            if output_path.exists() and not args.overwrite:
                continue
            prompt = item.get("targetPrompt", args.prompt)
            heatmap = encoder.encode(str(texture_dir / rgb_name), prompt)
            np.save(output_path, heatmap)
            print(output_path)


if __name__ == "__main__":
    precompute(parse_args())
