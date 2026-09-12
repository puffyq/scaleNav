#!/usr/bin/env python3
"""Enlarge v18's lower panel while keeping only the trajectory legend item."""

from pathlib import Path

from PIL import Image


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "pics/candidates/pearl_powerline_graph3d_candidate_v18.png"
OUTPUT = HERE / "pics/candidates/pearl_powerline_graph3d_candidate_v26"


def main() -> None:
    image = Image.open(SOURCE).convert("RGB")
    width, height = image.size

    # v18's top panel is kept pixel-identical.  The legend and map are split
    # before scaling so the large blank band between them does not survive in
    # the paper canvas.  Each visible part is still scaled uniformly.
    top_end = 810
    legend_source = image.crop((950, 1275, 2300, 1390))
    map_panel = image.crop((950, 1585, 2300, 1870))

    # Rebuild the legend margin from white, retaining only the red
    # ``flown trajectory`` entry and removing the other two entries entirely.
    legend = Image.new("RGB", legend_source.size, "white")
    legend.paste(legend_source.crop((120, 0, 485, 35)), (120, 0))

    scale = width / legend.width
    legend = legend.resize((width, round(legend.height * scale)),
                           Image.Resampling.LANCZOS)
    map_panel = map_panel.resize((width, round(map_panel.height * scale)),
                                 Image.Resampling.LANCZOS)

    gap = 12
    lower_height = legend.height + gap + map_panel.height
    output_height = max(height, top_end + lower_height)
    output = Image.new("RGB", (width, output_height), "white")
    output.paste(image.crop((0, 0, width, top_end)), (0, 0))
    output.paste(legend, (0, top_end))
    output.paste(map_panel, (0, top_end + legend.height + gap))
    output.save(OUTPUT.with_suffix(".png"), dpi=(300, 300))
    output.save(OUTPUT.with_suffix(".pdf"), resolution=300.0)
    print(f"wrote {OUTPUT}.png and {OUTPUT}.pdf; size={output.size}")


if __name__ == "__main__":
    main()
