#!/usr/bin/env python3
"""Generate PEARL/SCLIP semantic heatmap comparison figures.

Expected files in one directory:
  rgb_XXXXXX.png, semantic_pearl_XXXXXX.npy, semantic_sclip_XXXXXX.npy
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def normalize(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    lo, hi = np.nanpercentile(a, [2, 98])
    return np.clip((a - lo) / max(hi - lo, 1e-9), 0.0, 1.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path("paper/scalenav/pics/candidates/pearl_vs_sclip"))
    ap.add_argument("--count", type=int, default=6)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    pearl = {p.stem.removeprefix("semantic_pearl_"): p for p in args.data.glob("semantic_pearl_*.npy")}
    sclip = {p.stem.removeprefix("semantic_sclip_"): p for p in args.data.glob("semantic_sclip_*.npy")}
    keys = sorted(set(pearl) & set(sclip))[: args.count]
    if not keys:
        raise SystemExit("No paired semantic_pearl_*/semantic_sclip_*.npy files found")
    for key in keys:
        rgb_path = args.data / f"rgb_{key}.png"
        if not rgb_path.exists():
            continue
        rgb = np.asarray(Image.open(rgb_path).convert("RGB"))
        p = normalize(np.load(pearl[key]))
        s = normalize(np.load(sclip[key]))
        fig, ax = plt.subplots(1, 4, figsize=(15, 3.8), constrained_layout=True)
        ax[0].imshow(rgb); ax[0].set_title("RGB")
        ax[1].imshow(rgb); ax[1].imshow(p, cmap="turbo", alpha=.62, vmin=0, vmax=1); ax[1].set_title("PEARL")
        ax[2].imshow(rgb); ax[2].imshow(s, cmap="turbo", alpha=.62, vmin=0, vmax=1); ax[2].set_title("SCLIP")
        diff = np.abs(p - s)
        ax[3].imshow(diff, cmap="magma", vmin=0, vmax=1); ax[3].set_title("|PEARL − SCLIP|")
        for axis in ax: axis.axis("off")
        fig.savefig(args.out / f"pearl_vs_sclip_{key}.png", dpi=220, bbox_inches="tight")
        plt.close(fig)
    print(f"wrote {len(keys)} figures to {args.out}")


if __name__ == "__main__":
    main()
