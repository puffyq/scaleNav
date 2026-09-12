from __future__ import annotations

import argparse
from pathlib import Path

import torch

from config.config import cfg
from text_tracker.network import (
    TextYopoNetwork,
    export_text_yopo_torchscript,
    load_original_yopo_weights,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export a Depth+PEARL, 9-D-goal Graph executor from original YOPO."
    )
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    args = parser.parse_args()
    if not args.source.is_file():
        parser.error(f"source model not found: {args.source}")
    if args.device == "cuda" and not torch.cuda.is_available():
        parser.error("CUDA is not available")

    model = TextYopoNetwork().to(args.device).eval()
    loaded = load_original_yopo_weights(model, args.source)
    output = export_text_yopo_torchscript(
        model,
        args.output,
        image_height=int(cfg["image_height"]),
        image_width=int(cfg["image_width"]),
    )
    print(f"exported {output} from {loaded} compatible tensors on {args.device}")
    print("contract: image=[B,2,96,160], state=[B,9]")


if __name__ == "__main__":
    main()
