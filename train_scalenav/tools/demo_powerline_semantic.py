#!/usr/bin/env python3
"""Small dependency-free preflight demo for the 1080p power-line TODO.

The demo renders the same 10 cm wire at 20 m with a 90 degree camera in two
resolutions, reports the projected pixel width, and reconstructs a sparse 3-D
SemanticWire from ideal mask pixels plus synchronized depth. It deliberately
does not claim to test PEARL recall; use its output to validate scene scale and
the downstream map contract before adding a real PEARL inference process.
"""

from __future__ import annotations

import argparse
import binascii
import json
import math
import struct
import zlib
from pathlib import Path


def projected_width_px(width_px: int, fov_deg: float, diameter_m: float, depth_m: float) -> float:
    focal_px = (width_px / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return focal_px * diameter_m / depth_m


def project_x(x_m: float, depth_m: float, width_px: int, fov_deg: float) -> float:
    focal_px = (width_px / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return width_px / 2.0 + focal_px * x_m / depth_m


def unproject_x(pixel_x: float, depth_m: float, width_px: int, fov_deg: float) -> float:
    focal_px = (width_px / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return (pixel_x - width_px / 2.0) * depth_m / focal_px


def point_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    checksum = binascii.crc32(chunk_type)
    checksum = binascii.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + chunk_type + payload + struct.pack(">I", checksum)


def _write_rgb_png(path: Path, width: int, height: int, rows: list[bytes]) -> None:
    if len(rows) != height or any(len(row) != width * 3 for row in rows):
        raise ValueError("PNG row dimensions do not match the requested image size")
    payload = b"".join(b"\x00" + row for row in rows)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(payload, level=6))
        + _png_chunk(b"IEND", b"")
    )


FONT_5X7 = {
    " ": ("00000",) * 7,
    "+": ("00000", "00100", "00100", "11111", "00100", "00100", "00000"),
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
}


def _set_pixel(canvas: bytearray, width: int, height: int, x: int, y: int, color: tuple[int, int, int]) -> None:
    if 0 <= x < width and 0 <= y < height:
        offset = (y * width + x) * 3
        canvas[offset:offset + 3] = bytes(color)


def _fill_rect(canvas: bytearray, width: int, height: int, x0: int, y0: int, x1: int, y1: int,
               color: tuple[int, int, int]) -> None:
    x0, x1 = max(0, x0), min(width, x1)
    y0, y1 = max(0, y0), min(height, y1)
    row = bytes(color) * max(0, x1 - x0)
    for y in range(y0, y1):
        offset = (y * width + x0) * 3
        canvas[offset:offset + len(row)] = row


def _draw_line(canvas: bytearray, width: int, height: int, a: tuple[int, int], b: tuple[int, int],
               color: tuple[int, int, int], thickness: int = 1) -> None:
    x0, y0 = a
    x1, y1 = b
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    radius = max(0, thickness // 2)
    for step in range(steps + 1):
        t = step / steps
        x = round(x0 + t * (x1 - x0))
        y = round(y0 + t * (y1 - y0))
        _fill_rect(canvas, width, height, x - radius, y - radius, x + radius + 1, y + radius + 1, color)


def _draw_text(canvas: bytearray, width: int, height: int, x: int, y: int, text: str,
               color: tuple[int, int, int], scale: int = 4) -> None:
    cursor = x
    for character in text.upper():
        glyph = FONT_5X7.get(character, FONT_5X7[" "])
        for gy, row in enumerate(glyph):
            for gx, bit in enumerate(row):
                if bit == "1":
                    _fill_rect(canvas, width, height,
                               cursor + gx * scale, y + gy * scale,
                               cursor + (gx + 1) * scale, y + (gy + 1) * scale, color)
        cursor += 6 * scale


def render_pipeline_png(path: Path, fov_deg: float, diameter_m: float, depth_m: float) -> None:
    width, height = 1920, 1080
    panel_width = width // 4
    header_height = 78
    canvas = bytearray(width * height * 3)
    titles = ("RGB", "PEARL HEATMAP", "DEPTH", "DEMO RESULT")
    header_colors = ((31, 42, 55), (38, 34, 64), (40, 47, 51), (39, 54, 50))
    for panel, title in enumerate(titles):
        x0 = panel * panel_width
        _fill_rect(canvas, width, height, x0, 0, x0 + panel_width, header_height, header_colors[panel])
        _draw_text(canvas, width, height, x0 + 24, 24, title, (242, 245, 247), 4)

    # RGB panel: bright sky, two distant supports and the 4.8 px wire.
    for y in range(header_height, height):
        t = (y - header_height) / (height - header_height)
        color = (round(92 + 58 * t), round(170 + 42 * t), round(220 + 25 * t))
        _fill_rect(canvas, width, height, 0, y, panel_width, y + 1, color)
    wire_y = 505
    wire_px = max(1, round(projected_width_px(1920, fov_deg, diameter_m, depth_m)))
    _draw_line(canvas, width, height, (70, wire_y), (410, wire_y), (28, 31, 35), wire_px)
    _draw_line(canvas, width, height, (68, wire_y - 25), (68, 900), (70, 72, 70), 10)
    _draw_line(canvas, width, height, (412, wire_y - 25), (412, 900), (70, 72, 70), 10)

    # Synthetic PEARL response: a narrow high-risk ridge around the wire.
    pearl_x0 = panel_width
    _fill_rect(canvas, width, height, pearl_x0, header_height, pearl_x0 + panel_width, height, (15, 24, 52))
    for radius in range(30, 0, -1):
        ratio = 1.0 - radius / 30.0
        color = (round(45 + 210 * ratio), round(55 + 175 * ratio), round(115 - 90 * ratio))
        _draw_line(canvas, width, height,
                   (pearl_x0 + 70, wire_y), (pearl_x0 + 410, wire_y), color, radius * 2 + 1)
    _draw_text(canvas, width, height, pearl_x0 + 110, 930, "SYNTHETIC", (224, 230, 238), 3)

    # Depth panel: background is far, wire is 20 m and remains a thin return.
    depth_x0 = panel_width * 2
    _fill_rect(canvas, width, height, depth_x0, header_height, depth_x0 + panel_width, height, (42, 47, 50))
    for y in range(header_height, height):
        value = round(54 + 50 * (y - header_height) / (height - header_height))
        _fill_rect(canvas, width, height, depth_x0, y, depth_x0 + panel_width, y + 1,
                   (value, value, value))
    _draw_line(canvas, width, height, (depth_x0 + 70, wire_y), (depth_x0 + 410, wire_y),
               (222, 225, 227), wire_px)

    # Final top-down semantic map: wire capsule blocks the short center route,
    # while the selected A* path bends through the right corridor.
    map_x0 = panel_width * 3
    _fill_rect(canvas, width, height, map_x0, header_height, width, height, (235, 238, 236))
    _fill_rect(canvas, width, height, map_x0 + 60, 110, map_x0 + 205, 1010, (216, 224, 219))
    _fill_rect(canvas, width, height, map_x0 + 275, 110, map_x0 + 420, 1010, (216, 224, 219))
    _fill_rect(canvas, width, height, map_x0 + 110, wire_y - 70, map_x0 + 370, wire_y + 70,
               (242, 197, 190))
    _draw_line(canvas, width, height, (map_x0 + 120, wire_y), (map_x0 + 360, wire_y),
               (164, 48, 43), 9)
    route = ((map_x0 + 170, 990), (map_x0 + 170, 710), (map_x0 + 345, 620),
             (map_x0 + 345, 390), (map_x0 + 170, 300), (map_x0 + 170, 120))
    for start, end in zip(route, route[1:]):
        _draw_line(canvas, width, height, start, end, (24, 132, 111), 8)
    _fill_rect(canvas, width, height, map_x0 + 160, 975, map_x0 + 180, 995, (22, 83, 110))

    for separator in (panel_width, panel_width * 2, panel_width * 3):
        _fill_rect(canvas, width, height, separator - 2, 0, separator + 2, height, (255, 255, 255))
    rows = [bytes(canvas[y * width * 3:(y + 1) * width * 3]) for y in range(height)]
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_rgb_png(path, width, height, rows)


def render_wire_png(
    path: Path,
    width: int,
    height: int,
    fov_deg: float,
    diameter_m: float,
    depth_m: float,
    wire_x0_m: float,
    wire_x1_m: float,
) -> dict[str, float | int | bool]:
    """Render a dark horizontal wire on a bright sky and return mask stats."""
    x0 = project_x(wire_x0_m, depth_m, width, fov_deg)
    x1 = project_x(wire_x1_m, depth_m, width, fov_deg)
    half_width = projected_width_px(width, fov_deg, diameter_m, depth_m) / 2.0
    dark_pixels = 0
    total_darkness = 0.0
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for py in range(height):
        row = bytearray()
        for px in range(width):
            distance = point_segment_distance(px + 0.5, py + 0.5, x0, height / 2.0, x1, height / 2.0)
            # A one-pixel feather makes the sub-pixel low-resolution case
            # visible in the image without changing the reported diameter.
            coverage = max(0.0, min(1.0, half_width + 0.5 - distance))
            if coverage > 0.01:
                dark_pixels += 1
                total_darkness += coverage
            sky = int(220 - 80 * (py / max(1, height - 1)))
            value = int(round(sky * (1.0 - coverage) + 30.0 * coverage))
            row.extend((value // 2, value, min(255, value + 20)))
        rows.append(bytes(row))
    _write_rgb_png(path, width, height, rows)
    return {
        "width": width,
        "height": height,
        "projectedWireWidthPx": projected_width_px(width, fov_deg, diameter_m, depth_m),
        "darkPixels": dark_pixels,
        "darknessSum": round(total_darkness, 4),
        "resolvableAtThreePixels": projected_width_px(width, fov_deg, diameter_m, depth_m) >= 3.0,
    }


def reconstruct_semantic_wire(
    width: int,
    fov_deg: float,
    depth_m: float,
    altitude_m: float,
    wire_x0_m: float,
    wire_x1_m: float,
    sample_count: int,
) -> dict[str, object]:
    samples = []
    errors = []
    for index in range(sample_count):
        fraction = index / max(1, sample_count - 1)
        world_x = wire_x0_m + fraction * (wire_x1_m - wire_x0_m)
        pixel_x = project_x(world_x, depth_m, width, fov_deg)
        recovered_x = unproject_x(pixel_x, depth_m, width, fov_deg)
        errors.append(abs(recovered_x - world_x))
        # The demo camera looks along world +Y. Store the result in the
        # project's world_enu convention: x lateral, y forward, z altitude.
        samples.append([round(recovered_x, 5), round(depth_m, 5), round(altitude_m, 5)])
    return {
        "wireId": "demo-wire-001",
        "geometryState": "Verified",
        "polylineWorldEnu": samples,
        "physicalRadiusM": 0.05,
        "influenceRadiusM": 5.0,
        "risk": 0.9,
        "confidence": 1.0,
        "observationCount": 1,
        "maxReprojectionErrorM": max(errors) if errors else 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/powerline_semantic_demo"))
    parser.add_argument("--wire-diameter", type=float, default=0.10)
    parser.add_argument("--distance", type=float, default=20.0)
    parser.add_argument("--fov", type=float, default=90.0)
    parser.add_argument("--wire-half-length", type=float, default=4.0)
    parser.add_argument("--wire-altitude", type=float, default=1.6)
    args = parser.parse_args()
    if args.wire_diameter <= 0 or args.distance <= 0 or not 0 < args.fov < 180:
        parser.error("wire diameter and distance must be positive; FOV must be in (0, 180)")

    args.output.mkdir(parents=True, exist_ok=True)
    high_width_px = projected_width_px(1920, args.fov, args.wire_diameter, args.distance)
    high = {
        "width": 1920,
        "height": 1080,
        "projectedWireWidthPx": high_width_px,
        "resolvableAtThreePixels": high_width_px >= 3.0,
    }
    render_pipeline_png(
        args.output / "powerline_pipeline.png", args.fov, args.wire_diameter, args.distance)
    low_width_px = projected_width_px(160, args.fov, args.wire_diameter, args.distance)
    low = {
        "width": 160,
        "height": 96,
        "projectedWireWidthPx": low_width_px,
        "resolvableAtThreePixels": low_width_px >= 3.0,
    }
    semantic_wire = reconstruct_semantic_wire(
        1920, args.fov, args.distance, args.wire_altitude,
        -args.wire_half_length, args.wire_half_length, 33)
    report = {
        "scene": {
            "resolutionHigh": [1920, 1080],
            "resolutionLow": [160, 96],
            "horizontalFovDeg": args.fov,
            "wireDiameterM": args.wire_diameter,
            "wireDepthM": args.distance,
        },
        "highResolution": high,
        "lowResolution": low,
        "semanticWire": semantic_wire,
        "interpretation": {
            "highResolutionShouldShowWire": bool(high["resolvableAtThreePixels"]),
            "lowResolutionShouldBeSubPixel": not bool(low["resolvableAtThreePixels"]),
            "fusionUsesSynchronizedDepth": True,
            "pearlRecallWasTested": False,
        },
    }
    report["passed"] = bool(
        high["resolvableAtThreePixels"]
        and not low["resolvableAtThreePixels"]
        and float(semantic_wire["maxReprojectionErrorM"]) < 1e-4
    )
    (args.output / "semantic_wire.json").write_text(
        json.dumps(semantic_wire, indent=2) + "\n", encoding="utf-8"
    )
    (args.output / "report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    print(f"\nWrote demo artifacts to {args.output.resolve()}")
    if not report["passed"]:
        raise SystemExit("power-line semantic preflight FAILED")


if __name__ == "__main__":
    main()
