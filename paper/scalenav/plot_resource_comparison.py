#!/usr/bin/env python3
"""Fig. 8: resolution 4-grid (top) + online resource comparison (bottom).

Bottom panel: horizontal bars on a log time axis -- PEARL heatmap inference
at 1080p and 160x96 (measured from flight logs), route-layer stages
(persistent mode, test_data/graph_resource_scaling.csv), and the
FrontierGCN policy.  Whiskers mark P95 where available.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
TOP_IMAGE = HERE / "pics/candidates/pearl_powerline_resolution_4grid_red_blue.png"
SECOND_IMAGE = HERE / "pics/candidates/rgb_heatmap_resolution_comparison_red_blue.png"
OUT = HERE / "pics/candidates/fig8_resolution_resource"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 8.2,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

# (label, mean ms, P95 ms or None, color)
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

fig, ax = plt.subplots(figsize=(6.6, 2.2))
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

# group separator and group tags: bars are drawn bottom-up, so group 1
# (PEARL) sits below the dashed line and group 2 (route layer) above it.
n_pearl = len(GROUPS[0][1])
n_route = len(GROUPS[1][1])
sep_y = n_pearl + 0.45  # gap midpoint between the two groups
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


def split_cells(img: Image.Image) -> list[Image.Image]:
    """Split a 2x2 comparison panel into its four cells."""
    mx, my = img.width // 2, img.height // 2
    return [img.crop((0, 0, mx, my)), img.crop((mx, 0, img.width, my)),
            img.crop((0, my, mx, img.height)), img.crop((mx, my, img.width, img.height))]


scene_a = split_cells(Image.open(TOP_IMAGE).convert("RGB"))
scene_b = split_cells(Image.open(SECOND_IMAGE).convert("RGB"))
# two rows x four columns: row 1 = low resolution, row 2 = high resolution;
# each scene keeps its RGB | PEARL pair side by side.
cw, ch = scene_a[0].width, scene_a[0].height
bottom = Image.new("RGB", (4 * cw, 2 * ch), "white")
for c, cells in enumerate((scene_a, scene_b)):
    bottom.paste(cells[0], (2 * c * cw, 0))        # low row
    bottom.paste(cells[1], ((2 * c + 1) * cw, 0))
    bottom.paste(cells[2], (2 * c * cw, ch))       # high row
    bottom.paste(cells[3], ((2 * c + 1) * cw, ch))

chart = Image.open(chart_path).convert("RGB")
width = bottom.width
chart_height = round(chart.height * width / chart.width)
chart = chart.resize((width, chart_height), Image.Resampling.LANCZOS)

from PIL import ImageDraw, ImageFont
strip_h = 70
strip = Image.new("RGB", (width, strip_h), "white")
draw = ImageDraw.Draw(strip)
try:
    font = ImageFont.truetype("DejaVuSans.ttf", 44)
except OSError:
    font = ImageFont.load_default()
label = "query: \"powerline, line\""
tw = draw.textlength(label, font=font)
draw.text(((width - tw) / 2, (strip_h - 44) / 2), label, fill="#26363E",
          font=font)

combo = Image.new("RGB", (width, bottom.height + chart.height + strip_h),
                  "white")
combo.paste(chart, (0, 0))
combo.paste(bottom, (0, chart.height))
combo.paste(strip, (0, chart.height + bottom.height))
combo.save(OUT.with_suffix(".png"), dpi=(300, 300))
chart_path.unlink()
print(f"wrote {OUT}.png")
