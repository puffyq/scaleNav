#!/usr/bin/env python3
"""Fig. 8: semantic front-end cost and response quality.

Row 1: online resource comparison (horizontal bars on a log time axis).
Row 2: resolution sensitivity -- identical frames at 160x96 and 1080p,
       response heatmap inset at the lower right of each RGB cell.
Row 3: PEARL vs SCLIP overlays on 0903 real-flight frames.
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
RES_A = HERE / "pics/candidates/pearl_powerline_resolution_4grid_red_blue.png"
OUT = HERE / "pics/candidates/fig8_semantic_frontend"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 8.2,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

CELL_W = 500
GAP = 6
FONT = matplotlib.font_manager.findfont("DejaVu Sans:bold")


def font(size):
    return ImageFont.truetype(FONT, size)


def chip(img, text, size=30, left=True, fg="white", bg=(18, 26, 32, 190),
         pad=8, margin=8, y=None):
    """Draw a small dark label chip at the top-left/right corner."""
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    f = font(size)
    tw = d.textlength(text, font=f)
    th = size * 1.25
    x0 = margin if left else img.width - tw - 2 * pad - margin
    y0 = margin if y is None else y
    d.rounded_rectangle([x0, y0, x0 + tw + 2 * pad, y0 + th + 2 * pad - size * 0.25],
                        radius=6, fill=bg)
    d.text((x0 + pad, y0 + pad - size * 0.12), text, font=f, fill=fg)
    return Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")


# ---------------------------------------------------------------- row 1: chart
GROUPS = [
    ("PEARL heatmap by input resolution", [
        ("1080p",  840.0,  951.0, "#C46E13"),
        ("160x96",  49.0,   None, "#E8A33D"),
    ]),
    ("Route layer", [
        ("Graph rebuild",  116.5,  208.8, "#2D8C74"),
        ("Planning tick",   70.9,  164.4, "#3973B7"),
        ("A* search",       20.4,   59.8, "#7A6FB0"),
        ("FrontierGCN",      0.83,  1.97, "#D62828"),
    ]),
]

fig, ax = plt.subplots(figsize=(6.7, 1.8))
labels, positions = [], []
y = 0.0
for group, bars in GROUPS:
    for label, mean, p95, color in bars:
        ax.barh(y, mean, height=0.62, color=color, alpha=0.92)
        if p95 is not None:
            ax.plot([mean, p95], [y, y], color="#26363E", linewidth=1.1)
            ax.plot([p95], [y], marker="|", color="#26363E", markersize=7)
        ax.text((p95 if p95 is not None else mean) * 1.15, y, f"{mean:g} ms",
                va="center", fontsize=7.2, color="#26363E")
        labels.append(label)
        positions.append(y)
        y += 1.0
    y += 0.9
ax.set_yticks(positions)
ax.set_yticklabels(labels, fontsize=7.8)

n_pearl = len(GROUPS[0][1])
n_route = len(GROUPS[1][1])
sep_y = n_pearl + 0.45
ax.axhline(sep_y, color="#b9c0c6", linewidth=0.7, linestyle=(0, (3, 3)))
ax.text(2400, sep_y + n_route / 2, GROUPS[1][0], fontsize=7.2,
        color="#5A6167", style="italic", va="center", ha="right")
ax.text(2400, sep_y - n_pearl / 2 - 0.45, GROUPS[0][0], fontsize=7.2,
        color="#5A6167", style="italic", va="center", ha="right")

ax.set_xscale("log")
ax.set_xlim(0.5, 3000)
ax.set_xticks([1, 10, 100, 1000])
ax.set_xlabel("Wall time per invocation (ms, log scale; whisker: P95)",
              fontsize=7.6)
ax.grid(True, axis="x", color="#d9dde1", linewidth=0.45, alpha=0.85)
ax.set_axisbelow(True)
for spine in ("top", "right"):
    ax.spines[spine].set_visible(False)

fig.tight_layout(pad=0.3)
chart_path = OUT.with_name(OUT.name + "_chart.png")
fig.savefig(chart_path, dpi=300, bbox_inches="tight", pad_inches=0.02)
plt.close(fig)


# ---------------------------------------------------------- row 2: resolution
CROP_TOP = 46   # baked-in label strip of the source 2x2 grids
CROP_SIDE = 4   # black frame around the source grids


def split_cells(img):
    mx, my = img.width // 2, img.height // 2
    cs, ct = CROP_SIDE, CROP_TOP
    return [img.crop((cs, ct, mx - cs, my - cs)),
            img.crop((mx + cs, ct, img.width - cs, my - cs)),
            img.crop((cs, my + ct, mx - cs, img.height - cs)),
            img.crop((mx + cs, my + ct, img.width - cs, img.height - cs))]


def res_cell(rgb, heatmap, res_label, peak_label, query, w):
    h = round(rgb.height * w / rgb.width)
    cell = rgb.resize((w, h), Image.Resampling.LANCZOS)
    iw = round(w * 0.40)
    ih = round(heatmap.height * iw / heatmap.width)
    inset = heatmap.resize((iw, ih), Image.Resampling.LANCZOS)
    inset = chip(inset, peak_label, size=30, margin=6)
    x0, y0 = w - iw - 10, h - ih - 10
    frame = Image.new("RGB", (iw + 4, ih + 4), "white")
    frame.paste(inset, (2, 2))
    cell.paste(frame, (x0 - 2, y0 - 2))
    cell = chip(cell, res_label, size=42)
    cell = chip(cell, query, size=32, left=False)
    return cell


scene_a = split_cells(Image.open(RES_A).convert("RGB"))
q_line = '"powerline, line"'
RES_CELL_W = (4 * CELL_W + 3 * GAP - GAP) // 2
res_cells = [
    res_cell(scene_a[0], scene_a[1], "160x96", "peak 0.05", q_line,
             RES_CELL_W),
    res_cell(scene_a[2], scene_a[3], "1080p", "peak 0.68", q_line,
             RES_CELL_W),
]


# ------------------------------------------------------- row 3: PEARL vs SCLIP
SCLIP_DIR = HERE / "pics/candidates/0903_pearl_vs_sclip"
REPLAY = REPO / "scalenav_ws/tmp/0903_replay"


def heat(a):
    a = np.asarray(a, float)
    lo, hi = np.percentile(a, [2, 98])
    return np.clip((a - lo) / max(hi - lo, 1e-9), 0, 1)


def overlay_cell(run, kind, query):
    rgb = Image.open(REPLAY / run / "rgb_capture.jpg").convert("RGB")
    if kind == "PEARL":
        cell = Image.open(REPLAY / run / "pearl_overlay.jpg").convert("RGB")
    else:
        s = np.load(SCLIP_DIR / f"{run}_sclip.npy")
        cmap = plt.get_cmap("turbo")(heat(s))[..., :3]
        cmap_img = Image.fromarray((cmap * 255).astype(np.uint8))
        cell = Image.blend(rgb, cmap_img, 0.62)
    w = CELL_W
    h = round(cell.height * w / cell.width)
    cell = cell.resize((w, h), Image.Resampling.LANCZOS)
    cell = chip(cell, kind, size=30)
    cell = chip(cell, query, size=24, left=False)
    return cell


RUN_BUILDING = "run_20260905_164547_80534"
RUN_FOREST = "run_20260905_170525_201003"
sclip_cells = [
    overlay_cell(RUN_BUILDING, "PEARL", '"building"'),
    overlay_cell(RUN_BUILDING, "SCLIP", '"building"'),
    overlay_cell(RUN_FOREST, "PEARL", '"trees"'),
    overlay_cell(RUN_FOREST, "SCLIP", '"trees"'),
]


# ------------------------------------------------------------------- assemble
def make_row(cells):
    h = max(c.height for c in cells)
    w = sum(c.width for c in cells) + (len(cells) - 1) * GAP
    row = Image.new("RGB", (w, h), "white")
    x = 0
    for c in cells:
        row.paste(c, (x, 0))
        x += c.width + GAP
    return row


width = 4 * CELL_W + 3 * GAP
chart = Image.open(chart_path).convert("RGB")
chart = chart.resize((width, round(chart.height * width / chart.width)),
                     Image.Resampling.LANCZOS)
row_res = make_row(res_cells)
row_sclip = make_row(sclip_cells)

combo = Image.new("RGB", (width, chart.height + 2 * GAP + row_res.height
                          + row_sclip.height), "white")
combo.paste(chart, (0, 0))
combo.paste(row_res, (0, chart.height + GAP))
combo.paste(row_sclip, (0, chart.height + 2 * GAP + row_res.height))
combo.save(OUT.with_suffix(".png"), dpi=(300, 300))
chart_path.unlink()
print(f"wrote {OUT}.png  {combo.size}")
