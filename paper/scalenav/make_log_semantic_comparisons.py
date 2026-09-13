#!/usr/bin/env python3
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path("scalenav_ws/tmp/0903_replay")
OUT = Path("paper/scalenav/pics/candidates/log_pearl_comparisons")
OUT.mkdir(parents=True, exist_ok=True)
items = sorted(ROOT.glob("run_*/pearl_overlay.jpg"), key=lambda p: p.stat().st_mtime, reverse=True)
font = ImageFont.load_default()
for batch in range(3):
    group = items[batch * 6:(batch + 1) * 6]
    if not group: break
    thumbs = []
    for path in group:
        heat = path.with_name("pearl_heatmap.png")
        if not heat.exists(): continue
        a = Image.open(path).convert("RGB"); b = Image.open(heat).convert("RGB")
        w, h = 480, 300
        a.thumbnail((w, h)); b.thumbnail((w, h))
        cell = Image.new("RGB", (w * 2, h + 32), "white")
        cell.paste(a, (0, 32)); cell.paste(b, (w, 32))
        d = ImageDraw.Draw(cell); d.text((8, 8), path.parent.name + "  PEARL overlay", fill="black", font=font); d.text((w + 8, 8), "PEARL heatmap", fill="black", font=font)
        thumbs.append(cell)
    if not thumbs: continue
    canvas = Image.new("RGB", (960, len(thumbs) * (332)), "#eeeeee")
    for i, cell in enumerate(thumbs): canvas.paste(cell, (0, i * 332))
    canvas.save(OUT / f"pearl_log_montage_{batch + 1}.png")
print(f"wrote {len(list(OUT.glob('*.png')))} figures to {OUT}")
