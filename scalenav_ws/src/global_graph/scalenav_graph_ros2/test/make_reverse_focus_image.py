#!/usr/bin/env python3
"""Render one uncluttered static image from the map2 reverse replay."""

from __future__ import annotations

import argparse
import base64
import html
import json
import math
from pathlib import Path

import semantic_frontier_module_test as replay
from make_reverse_focus_report import focused_payload, world_cloud


def css_score_color(score: float | None) -> str:
    if score is None or not math.isfinite(float(score)):
        return "#7d8c96"
    value = max(0.0, min(1.0, float(score)))
    # Debug colormap, not the RViz runtime colormap:
    # low semantic risk is intentionally dark so free/passable areas do not
    # look "bright"; high semantic risk is yellow/red.
    anchors = ((18, 34, 48), (232, 190, 67), (225, 60, 50))
    scaled = value * 2.0
    index = min(1, int(scaled))
    fraction = scaled - index
    rgb = [
        round(anchors[index][i] * (1.0 - fraction) +
              anchors[index + 1][i] * fraction)
        for i in range(3)
    ]
    return "#" + "".join(f"{component:02x}" for component in rgb)


def image_href(uri: str) -> str:
    return uri


def svg_text(x: float, y: float, text: str, size: int = 16,
             color: str = "#edf4f7", weight: str = "400") -> str:
    return (f'<text x="{x:.1f}" y="{y:.1f}" fill="{color}" '
            f'font-size="{size}px" font-family="DejaVu Sans, sans-serif" '
            f'font-weight="{weight}">{html.escape(text)}</text>')


def make_svg(payload: dict) -> str:
    frame = payload["frame"]
    width, height = frame["image_size"]["width"], frame["image_size"]["height"]
    panel_w, panel_h = 500, 300
    margin = 28
    top_y = 70
    world_y = 410
    world_w, world_h = 1544, 700
    canvas_w, canvas_h = 1600, 1140
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_w}" '
        f'height="{canvas_h}" viewBox="0 0 {canvas_w} {canvas_h}">',
        '<rect width="100%" height="100%" fill="#0b1118"/>',
        svg_text(margin, 34, "Map2 reverse projection: ordinary Verified nodes", 25, "#ffffff", "700"),
        svg_text(margin, 57,
                 f'{payload["session"]} · frame {payload["frame_index"] + 1}/'
                 f'{payload["frame_count"]} · t={frame["time_s"]:.2f} s',
                 14, "#9db0bb"),
    ]

    images = (("RGB + 3x5 patches", frame["rgb"]),
              ("Depth + 3x5 patches", frame["depth"]),
              ("Heatmap + node back-projection", frame["heatmap"]))
    for index, (title, uri) in enumerate(images):
        x = margin + index * (panel_w + 22)
        out.append(f'<rect x="{x}" y="{top_y}" width="{panel_w}" height="{panel_h}" '
                   'rx="6" fill="#131e28" stroke="#2c4150"/>')
        out.append(svg_text(x + 12, top_y + 25, title, 16, "#ffffff", "700"))
        if uri:
            image_y = top_y + 38
            image_h = panel_h - 48
            out.append(f'<image href="{image_href(uri)}" x="{x + 10}" y="{image_y}" '
                       f'width="{panel_w - 20}" height="{image_h}" '
                       'preserveAspectRatio="none"/>')
            # Image-space grid. The report has 160x96 camera images.
            ix, iy, iw, ih = x + 10, image_y, panel_w - 20, image_h
            for row in range(4):
                yy = iy + ih * row / 3.0
                out.append(f'<line x1="{ix:.1f}" y1="{yy:.1f}" x2="{ix + iw:.1f}" '
                           f'y2="{yy:.1f}" stroke="#ffffff" stroke-opacity=".55"/>')
            for col in range(6):
                xx = ix + iw * col / 5.0
                out.append(f'<line x1="{xx:.1f}" y1="{iy:.1f}" x2="{xx:.1f}" '
                           f'y2="{iy + ih:.1f}" stroke="#ffffff" stroke-opacity=".55"/>')
            if index == 2:
                means = frame["patch_means"]
                for row in range(3):
                    for col in range(5):
                        value = means[row][col]
                        fill = css_score_color(value)
                        xx, yy = ix + iw * col / 5.0, iy + ih * row / 3.0
                        out.append(f'<rect x="{xx:.1f}" y="{yy:.1f}" '
                                   f'width="{iw / 5:.1f}" height="{ih / 3:.1f}" '
                                   f'fill="{fill}" fill-opacity=".22"/>')
                        label = "n/a" if value is None else f"{value:.3f}"
                        out.append(svg_text(xx + 5, yy + 19, label, 12, "#ffffff", "700"))
            # Show all valid reverse projections on each image, but without
            # labels; this makes their spatial relation visible at a glance.
            for node in frame["nodes"]:
                if (not node["visible"] or node.get("score") is None or
                        node.get("pu") is None or node.get("pv") is None):
                    continue
                # pixel_u/pixel_v are stored at pixel-center coordinates
                # (normalized * size - 0.5).  Add 0.5 before converting back
                # to the image canvas; otherwise points exactly on a 3x5
                # boundary are assigned to the preceding row/column.
                px = ix + (float(node["pu"]) + 0.5) / width * iw
                py = iy + (float(node["pv"]) + 0.5) / height * ih
                color = css_score_color(node["score"])
                radius = 6.5 if node.get("nearest_scored") else (5.5 if node["top"] else 3.5)
                stroke = "#55d6e8" if node.get("nearest_scored") else "#ffffff"
                stroke_width = 2 if node.get("nearest_scored") else 1
                out.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{radius:.1f}" '
                           f'fill="{color}" stroke="{stroke}" '
                           f'stroke-width="{stroke_width}"/>')
                # Explicitly report the 3x5 row used by this projection.  A
                # pixel row is counted top-to-bottom (1, 2, 3), exactly as
                # the ROS image message and the PGM file are stored.
                if index == 2 and (node["top"] or node.get("nearest_scored")):
                    row_number = min(3, max(1, int((float(node["pv"]) + 0.5) /
                                                    height * 3.0) + 1))
                    label_prefix = "near" if node.get("nearest_scored") else ""
                    out.append(svg_text(px + 8, py - 8,
                                        f'{label_prefix} N{node["i"]}: row {row_number}',
                                        11, "#ffffff", "700"))

    stats = frame["stats"]
    facts = [
        f'odom=({frame["odom"][0]:.1f}, {frame["odom"][1]:.1f})',
        f'visible={stats.get("visible_nodes", 0)}',
        f'annotated={stats.get("annotated_nodes", 0)}',
        f'score={stats.get("score_min", 0.0):.3f}…{stats.get("score_max", 0.0):.3f}',
        f'top=N{stats.get("top_node_index", "-")}',
    ]
    out.append(f'<rect x="{margin}" y="385" width="{world_w}" height="1" fill="#2c4150"/>')
    out.append(svg_text(margin, 405, "World XY zoom: obstacle surface vs replayed semantic node scores", 18, "#ffffff", "700"))
    out.append(svg_text(margin + 600, 405,
                        "low score = dark, high score = yellow/red; gray dots = depth point cloud surface",
                        13, "#9db0bb"))
    out.append(svg_text(margin, 1100, " · ".join(facts), 14, "#9db0bb"))

    # Keep the world panel local. Including the mission goal at y=140 in the
    # bounds would compress the actual obstacle/node cluster into a few pixels.
    # The mission direction is shown separately as a short arrow.
    points = list(frame["cloud"])
    points.extend([[n["x"], n["y"]] for n in frame["nodes"]])
    points.append([frame["odom"][0], frame["odom"][1]])
    if not points:
        return "".join(out) + "</svg>"
    min_x = min(p[0] for p in points) - 3
    max_x = max(p[0] for p in points) + 3
    min_y = min(p[1] for p in points) - 3
    max_y = max(p[1] for p in points) + 3
    scale = min((world_w - 34) / max(1e-6, max_x - min_x),
                (world_h - 34) / max(1e-6, max_y - min_y))
    center_x, center_y = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
    sx = lambda value: margin + world_w / 2.0 + (value - center_x) * scale
    sy = lambda value: world_y + world_h / 2.0 - (value - center_y) * scale
    out.append(f'<rect x="{margin}" y="{world_y}" width="{world_w}" height="{world_h}" '
               'rx="6" fill="#071016" stroke="#2c4150"/>')
    # Draw the logged obstacle surface first so semantic nodes remain visible.
    for x, y in frame["cloud"]:
        out.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="1.5" '
                   'fill="#8b989e" fill-opacity=".40"/>')
    for node in frame["nodes"]:
        if node["score"] is None:
            continue
        fill = css_score_color(node["score"])
        radius = 8 if node["top"] else (7 if node.get("nearest_scored") else 4.7)
        stroke = "#ffffff" if node["top"] else ("#55d6e8" if node.get("nearest_scored") else "#14232c")
        sw = 2 if (node["top"] or node.get("nearest_scored")) else 1
        out.append(f'<circle cx="{sx(node["x"]):.1f}" cy="{sy(node["y"]):.1f}" '
                   f'r="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')
    ox, oy = sx(frame["odom"][0]), sy(frame["odom"][1])
    gx, gy = sx(frame["goal"]["x"]), sy(frame["goal"]["y"])
    out.append(f'<circle cx="{ox:.1f}" cy="{oy:.1f}" r="9" fill="#55d6e8"/>')
    out.append(svg_text(ox + 12, oy - 10, "odom", 14, "#55d6e8", "700"))
    # Draw the actual body-forward direction from synchronized odom.  The
    # logged quaternion is ROS order [x, y, z, w] and maps body FLU to
    # world_enu.  Do not substitute +X: at this frame the vehicle is heading
    # roughly (-0.516, +0.850) in world XY.
    orientation = frame.get("odom_orientation", [0.0, 0.0, 0.0, 1.0])
    forward = replay.quat_rotate(orientation, [1.0, 0.0, 0.0])
    forward_norm = math.hypot(forward[0], forward[1])
    if forward_norm > 1e-6:
        forward_xy = (forward[0] / forward_norm, forward[1] / forward_norm)
    else:
        forward_xy = (1.0, 0.0)
    body_len = min(120.0, world_w * 0.11)
    bx, by = ox + body_len * forward_xy[0], oy - body_len * forward_xy[1]
    out.append(f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{bx:.1f}" y2="{by:.1f}" '
               'stroke="#55d6e8" stroke-width="5"/>')
    # Arrowhead points in the same screen-space direction (world +Y is up in
    # the XY top view, hence the sign on by above).
    left_x = bx - 12.0 * forward_xy[0] + 8.0 * forward_xy[1]
    left_y = by + 12.0 * forward_xy[1] + 8.0 * forward_xy[0]
    right_x = bx - 12.0 * forward_xy[0] - 8.0 * forward_xy[1]
    right_y = by + 12.0 * forward_xy[1] - 8.0 * forward_xy[0]
    out.append(f'<polygon points="{bx:.1f},{by:.1f} {left_x:.1f},{left_y:.1f} '
               f'{right_x:.1f},{right_y:.1f}" fill="#55d6e8"/>')
    out.append(svg_text(bx + 10, by - 8, "body forward", 14, "#55d6e8", "700"))

    # Mission direction is a separate diagnostic vector.  It is based on the
    # actual odom-to-goal displacement, not on vehicle attitude.
    mission_dx = float(frame["goal"]["x"]) - float(frame["odom"][0])
    mission_dy = float(frame["goal"]["y"]) - float(frame["odom"][1])
    mission_norm = math.hypot(mission_dx, mission_dy)
    if mission_norm > 1e-6:
        mission_xy = (mission_dx / mission_norm, mission_dy / mission_norm)
    else:
        mission_xy = (1.0, 0.0)
    goal_len = min(180.0, world_w * 0.16)
    mx, my = ox + goal_len * mission_xy[0], oy - goal_len * mission_xy[1]
    out.append(f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{mx:.1f}" y2="{my:.1f}" '
               'stroke="#f4c95d" stroke-width="4" stroke-dasharray="10 7"/>')
    mleft_x = mx - 14.0 * mission_xy[0] + 8.0 * mission_xy[1]
    mleft_y = my + 14.0 * mission_xy[1] + 8.0 * mission_xy[0]
    mright_x = mx - 14.0 * mission_xy[0] - 8.0 * mission_xy[1]
    mright_y = my + 14.0 * mission_xy[1] - 8.0 * mission_xy[0]
    out.append(f'<polygon points="{mx:.1f},{my:.1f} {mleft_x:.1f},{mleft_y:.1f} '
               f'{mright_x:.1f},{mright_y:.1f}" fill="#f4c95d"/>')
    out.append(svg_text(mx + 12, my - 10, "toward mission goal", 14, "#f4c95d", "700"))
    out.extend([
        svg_text(margin + 18, world_y + 27,
                 "gray = obstacle surface", 13, "#b6c2c7"),
        svg_text(margin + 160, world_y + 27,
                 "dark = low risk", 13, "#9db0bb"),
        svg_text(margin + 320, world_y + 27,
                 "yellow/red = high risk", 13, "#e13c32"),
    ])
    out.append("</svg>")
    return "".join(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--frame", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = focused_payload(args.session, args.frame)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    svg_path = args.output.with_suffix(".svg")
    svg_path.write_text(make_svg(payload), encoding="utf-8")
    # ImageMagick is already used by the log replay for image conversion. Keep
    # the SVG as the lossless artifact and also produce a convenient PNG.
    import subprocess
    subprocess.run(["convert", "-background", "#0b1118", str(svg_path),
                    "-resize", "1600x1140", str(args.output)], check=True)
    print(f"focused image: {args.output}")
    print(f"focused svg: {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
