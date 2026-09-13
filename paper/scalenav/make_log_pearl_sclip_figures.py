#!/usr/bin/env python3
"""Build PEARL-vs-SCLIP figures from a ScaleNav log session."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def read_pgm16(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path), dtype=float) / 1000.0


def norm(a: np.ndarray) -> np.ndarray:
    lo, hi = np.percentile(a, [2, 98])
    return np.clip((a - lo) / max(hi - lo, 1e-9), 0, 1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--sclip", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    records = []
    for line in (args.session / "index.jsonl").read_text().splitlines():
        try:
            records.append(json.loads(line, parse_constant=lambda _: None))
        except json.JSONDecodeError:
            continue
    rgbs = [r for r in records if r.get("kind") == "rgb"]
    sems = [r for r in records if r.get("kind") == "semantic"]
    sclip = {p.stem.removeprefix("semantic_sclip_").removesuffix(".npy"): p for p in args.sclip.glob("semantic_sclip_*.npy")}
    for key, spath in sorted(sclip.items(), key=lambda item: int(item[0])):
        rgb_id = int(key)
        rgb_rec = next((r for r in rgbs if Path(r["file"]).stem == f"rgb_{rgb_id}"), None)
        if rgb_rec is None: continue
        nearest = min(sems, key=lambda r: abs(r["stamp_ns"] - rgb_rec["stamp_ns"])) if sems else None
        if nearest is None: continue
        rgb = np.asarray(Image.open(args.session / rgb_rec["file"]).convert("RGB"))
        pearl = norm(read_pgm16(args.session / nearest["file"]))
        s = norm(np.load(spath))
        fig, ax = plt.subplots(1, 4, figsize=(15, 3.7), constrained_layout=True)
        ax[0].imshow(rgb); ax[0].set_title(f"RGB  frame {rgb_id}")
        ax[1].imshow(rgb); ax[1].imshow(pearl, cmap="turbo", alpha=.62, vmin=0, vmax=1); ax[1].set_title("PEARL")
        ax[2].imshow(rgb); ax[2].imshow(s, cmap="turbo", alpha=.62, vmin=0, vmax=1); ax[2].set_title("SCLIP")
        ax[3].imshow(np.abs(pearl - s), cmap="magma", vmin=0, vmax=1); ax[3].set_title("absolute difference")
        for axis in ax: axis.axis("off")
        fig.savefig(args.out / f"pearl_vs_sclip_{rgb_id:04d}.png", dpi=220, bbox_inches="tight")
        plt.close(fig)
    print(f"wrote {len(list(args.out.glob('pearl_vs_sclip_*.png')))} figures to {args.out}")


if __name__ == "__main__":
    main()
