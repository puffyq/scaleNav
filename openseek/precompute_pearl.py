from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rtoml
import torch

from text_tracker.pearl_adapter import PEARLHeatmapEncoder


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Precompute frozen PEARL target-probability heatmaps."
    )
    parser.add_argument("--data", required=True)
    parser.add_argument(
        "--pearl-root", default=str(PROJECT_ROOT / "third_party" / "PEARL")
    )
    parser.add_argument("--checkpoint", default="ViT-B/16")
    parser.add_argument("--prompt", default="person")
    parser.add_argument(
        "--force-prompt",
        action="store_true",
        help="Use --prompt for every frame instead of targetPrompt from data.toml.",
    )
    parser.add_argument("--output-prefix", default="semantic_pearl")
    parser.add_argument("--crop-size", type=int, default=224)
    parser.add_argument("--stride", type=int, default=112)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def precompute(args: argparse.Namespace) -> None:
    encoder = PEARLHeatmapEncoder(
        args.pearl_root,
        checkpoint=args.checkpoint,
        device=torch.device("cuda" if torch.cuda.is_available() else "cpu"),
        crop_size=args.crop_size,
        stride=args.stride,
    )
    encoder.prepare_prompt(args.prompt)
    for toml_path in sorted(Path(args.data).glob("Scene_*/data.toml")):
        document = rtoml.load(toml_path)
        texture_dir = toml_path.parent / "Textures"
        for item in document.get("dataArray", []):
            rgb_name = item.get("rgbFileName")
            if not rgb_name:
                continue
            index = Path(rgb_name).stem.removeprefix("rgb_")
            output_path = texture_dir / f"{args.output_prefix}_{index}.npy"
            if output_path.exists() and not args.overwrite:
                continue
            prompt = args.prompt if args.force_prompt else item.get("targetPrompt", args.prompt)
            probability = encoder.encode(str(texture_dir / rgb_name), prompt)
            np.save(output_path, probability)
            print(output_path)


if __name__ == "__main__":
    precompute(parse_args())
