#!/usr/bin/env python3
"""Build a dependency-free obstacle-density heatmap for one round trip."""

import argparse
import bisect
import json
import math
from pathlib import Path


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * q)]


def rotate_point(point, quaternion):
    x, y, z = point
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def load_index(session):
    odom = []
    pointclouds = []
    graphs = []
    with (session / "index.jsonl").open("r", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            kind = record.get("kind")
            if kind == "odom":
                odom.append((record["stamp_ns"], record["data"]["position"],
                             record["data"]["orientation"]))
            elif kind == "pointcloud":
                pointclouds.append((record["stamp_ns"], session / record["file"]))
            elif kind == "graph":
                graphs.append((record["stamp_ns"], session / record["file"]))
    return odom, pointclouds, graphs


def nearest_odom(odom, stamps, stamp):
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(odom):
        candidates.append(odom[index])
    if index:
        candidates.append(odom[index - 1])
    return min(candidates, key=lambda item: abs(item[0] - stamp))


def route(odom, start_ns, end_ns, stride=5):
    selected = [(stamp, pos) for stamp, pos, _ in odom if start_ns <= stamp <= end_ns]
    points = [tuple(item[1]) for item in selected[::stride]]
    if selected and (not points or points[-1] != tuple(selected[-1][1])):
        points.append(tuple(selected[-1][1]))
    return points


def load_occupied_cells(session, odom, pointclouds, start_ns, end_ns,
                        cell_size=0.5, sample_period_ns=500_000_000,
                        min_z=0.3, max_z=3.2):
    stamps = [item[0] for item in odom]
    occupied = set()
    sampled_frames = 0
    accepted_points = 0
    next_stamp = start_ns
    for stamp, path in pointclouds:
        if stamp < start_ns or stamp > end_ns or stamp < next_stamp:
            continue
        next_stamp = stamp + sample_period_ns
        _, position, orientation = nearest_odom(odom, stamps, stamp)
        sampled_frames += 1
        with path.open("r", encoding="ascii") as stream:
            for line in stream:
                if line.startswith("DATA"):
                    break
            for point_index, line in enumerate(stream):
                if point_index % 2:
                    continue
                fields = line.split()
                if len(fields) < 3:
                    continue
                local = (float(fields[0]), float(fields[1]), float(fields[2]))
                rotated = rotate_point(local, orientation)
                world = (rotated[0] + position[0], rotated[1] + position[1],
                         rotated[2] + position[2])
                if min_z <= world[2] <= max_z:
                    occupied.add((math.floor(world[0] / cell_size),
                                  math.floor(world[1] / cell_size)))
                    accepted_points += 1
    return occupied, sampled_frames, accepted_points


def high_risk_semantic_points(graph_path, flight_z=1.6, vertical_band=4.0):
    with graph_path.open("r", encoding="utf-8") as stream:
        graph = json.load(stream)
    for marker in graph.get("markers", []):
        if marker.get("ns") != "epic_semantic_points":
            continue
        points = marker.get("points", [])
        colors = marker.get("colors", [])
        result = []
        for point, color in zip(points, colors):
            is_high_risk = color[0] > 0.75 and color[0] > color[1] + 0.2
            if is_high_risk and abs(point[2] - flight_z) <= vertical_band:
                result.append(tuple(point))
        return result
    return []


def density_grid(occupied, cell_size, bounds, radius=3.0, output_cell=1.0):
    x_min, x_max, y_min, y_max = bounds
    offsets = []
    radius_cells = math.ceil(radius / cell_size)
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if math.hypot(dx * cell_size, dy * cell_size) <= radius:
                offsets.append((dx, dy))
    grid = {}
    y = y_min
    while y < y_max:
        x = x_min
        while x < x_max:
            center = (math.floor((x + output_cell / 2) / cell_size),
                      math.floor((y + output_cell / 2) / cell_size))
            hits = sum((center[0] + dx, center[1] + dy) in occupied for dx, dy in offsets)
            grid[(round(x, 6), round(y, 6))] = hits / len(offsets)
            x += output_cell
        y += output_cell
    return grid


def density_at(point, occupied, cell_size=0.5, radius=3.0):
    center = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
    radius_cells = math.ceil(radius / cell_size)
    hits = 0
    total = 0
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if math.hypot(dx * cell_size, dy * cell_size) <= radius:
                total += 1
                hits += (center[0] + dx, center[1] + dy) in occupied
    return hits / total


def route_metrics(points, occupied, semantic):
    densities = [density_at(point, occupied) for point in points]
    semantic_distances = []
    for point in points:
        if semantic:
            semantic_distances.append(min(math.hypot(point[0] - sem[0], point[1] - sem[1])
                                          for sem in semantic))
    path_length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return {
        "samples": len(points),
        "path_length_m": path_length,
        "obstacle_density_mean": sum(densities) / len(densities),
        "obstacle_density_p90": percentile(densities, 0.90),
        "obstacle_density_max": max(densities),
        "semantic_distance_mean_m": (sum(semantic_distances) / len(semantic_distances)
                                      if semantic_distances else None),
        "semantic_distance_min_m": min(semantic_distances) if semantic_distances else None,
        "semantic_within_5m_fraction": (sum(value <= 5.0 for value in semantic_distances) /
                                        len(semantic_distances) if semantic_distances else None),
        "mean_abs_lateral_m": sum(abs(point[0]) for point in points) / len(points),
        "max_abs_lateral_m": max(abs(point[0]) for point in points),
    }


def interpolate_color(value):
    stops = [
        (0.00, (248, 250, 247)),
        (0.20, (239, 228, 154)),
        (0.45, (226, 155, 94)),
        (0.70, (190, 77, 62)),
        (1.00, (91, 35, 49)),
    ]
    for (left, color_left), (right, color_right) in zip(stops, stops[1:]):
        if value <= right:
            ratio = (value - left) / (right - left)
            return tuple(round(a + ratio * (b - a)) for a, b in zip(color_left, color_right))
    return stops[-1][1]


def svg_heatmap(output_path, grid, bounds, outbound, inbound, semantic, metrics, meta):
    width, height = 1200, 920
    plot_x, plot_y, plot_w, plot_h = 110, 70, 360, 790
    x_min, x_max, y_min, y_max = bounds
    max_density = percentile(list(grid.values()), 0.98) or 1.0

    def screen(point):
        x = plot_x + (point[0] - x_min) / (x_max - x_min) * plot_w
        y = plot_y + plot_h - (point[1] - y_min) / (y_max - y_min) * plot_h
        return x, y

    def polyline(points):
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in map(screen, points))

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="1200" height="920" fill="#f4f5f2"/>',
        '<text x="70" y="34" font-family="DejaVu Sans" font-size="24" fill="#172026">Round-trip obstacle density and semantic risk</text>',
        '<text x="70" y="57" font-family="DejaVu Sans" font-size="13" fill="#53616a">Latest complete run, 2026-08-28 11:18 | flight-height occupancy, 3 m neighborhood</text>',
        f'<rect x="{plot_x}" y="{plot_y}" width="{plot_w}" height="{plot_h}" fill="#fff" stroke="#9aa4a9"/>',
    ]
    output_cell = 1.0
    cell_w = output_cell / (x_max - x_min) * plot_w
    cell_h = output_cell / (y_max - y_min) * plot_h
    for (x, y), density in grid.items():
        sx, sy = screen((x, y + output_cell))
        color = interpolate_color(min(1.0, density / max_density))
        svg.append(f'<rect x="{sx:.2f}" y="{sy:.2f}" width="{cell_w + .3:.2f}" height="{cell_h + .3:.2f}" fill="rgb{color}"/>')
    for y in range(math.ceil(y_min / 20) * 20, math.floor(y_max / 20) * 20 + 1, 20):
        _, sy = screen((x_min, y))
        svg.append(f'<line x1="{plot_x}" y1="{sy:.1f}" x2="{plot_x + plot_w}" y2="{sy:.1f}" stroke="#ffffff" stroke-opacity="0.65"/>')
        svg.append(f'<text x="{plot_x - 12}" y="{sy + 4:.1f}" text-anchor="end" font-family="DejaVu Sans" font-size="11" fill="#53616a">{y}</text>')
    for x in range(math.ceil(x_min / 10) * 10, math.floor(x_max / 10) * 10 + 1, 10):
        sx, _ = screen((x, y_min))
        svg.append(f'<line x1="{sx:.1f}" y1="{plot_y}" x2="{sx:.1f}" y2="{plot_y + plot_h}" stroke="#ffffff" stroke-opacity="0.45"/>')
        svg.append(f'<text x="{sx:.1f}" y="{plot_y + plot_h + 19}" text-anchor="middle" font-family="DejaVu Sans" font-size="11" fill="#53616a">{x}</text>')
    for point in semantic:
        if x_min <= point[0] <= x_max and y_min <= point[1] <= y_max:
            sx, sy = screen(point)
            svg.append(f'<path d="M {sx:.1f} {sy-5:.1f} L {sx+5:.1f} {sy:.1f} L {sx:.1f} {sy+5:.1f} L {sx-5:.1f} {sy:.1f} Z" fill="#7f2f72" stroke="#fff" stroke-width="0.8"/>')
    svg.extend([
        f'<polyline points="{polyline(outbound)}" fill="none" stroke="#087f5b" stroke-width="4.2" stroke-linejoin="round" opacity="0.94"/>',
        f'<polyline points="{polyline(inbound)}" fill="none" stroke="#1769aa" stroke-width="3.5" stroke-linejoin="round" opacity="0.92"/>',
    ])
    for point, color in ((outbound[0], "#087f5b"), (outbound[-1], "#172026")):
        sx, sy = screen(point)
        svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="{color}" stroke="#fff" stroke-width="1.5"/>')

    panel_x = 530
    svg.extend([
        f'<text x="{panel_x}" y="90" font-family="DejaVu Sans" font-size="18" font-weight="bold" fill="#172026">Spatial evidence</text>',
        f'<line x1="{panel_x}" y1="120" x2="{panel_x+42}" y2="120" stroke="#087f5b" stroke-width="4"/><text x="{panel_x+54}" y="125" font-family="DejaVu Sans" font-size="14" fill="#172026">Outbound</text>',
        f'<line x1="{panel_x+190}" y1="120" x2="{panel_x+232}" y2="120" stroke="#1769aa" stroke-width="4"/><text x="{panel_x+244}" y="125" font-family="DejaVu Sans" font-size="14" fill="#172026">Return</text>',
        f'<path d="M {panel_x+6} 151 l 6 6 l -6 6 l -6 -6 Z" fill="#7f2f72"/><text x="{panel_x+22}" y="162" font-family="DejaVu Sans" font-size="14" fill="#172026">High-risk semantic node</text>',
        f'<text x="{panel_x}" y="210" font-family="DejaVu Sans" font-size="16" font-weight="bold" fill="#172026">Route exposure</text>',
    ])
    rows = [
        ("Path length", "path_length_m", "m", 3),
        ("Mean obstacle density", "obstacle_density_mean", "", 4),
        ("P90 obstacle density", "obstacle_density_p90", "", 4),
        ("Mean semantic distance", "semantic_distance_mean_m", "m", 2),
        ("Within 5 m of semantic risk", "semantic_within_5m_fraction", "", 1),
        ("Mean |lateral x|", "mean_abs_lateral_m", "m", 2),
    ]
    svg.extend([
        f'<text x="{panel_x+270}" y="236" text-anchor="end" font-family="DejaVu Sans" font-size="12" fill="#087f5b">OUTBOUND</text>',
        f'<text x="{panel_x+430}" y="236" text-anchor="end" font-family="DejaVu Sans" font-size="12" fill="#1769aa">RETURN</text>',
    ])
    y = 270
    for label, key, unit, digits in rows:
        left = metrics["outbound"][key]
        right = metrics["return"][key]
        if key == "semantic_within_5m_fraction":
            left_text = f"{100 * left:.1f}%"
            right_text = f"{100 * right:.1f}%"
        else:
            left_text = f"{left:.{digits}f} {unit}".strip()
            right_text = f"{right:.{digits}f} {unit}".strip()
        svg.extend([
            f'<text x="{panel_x}" y="{y}" font-family="DejaVu Sans" font-size="14" fill="#53616a">{label}</text>',
            f'<text x="{panel_x+270}" y="{y}" text-anchor="end" font-family="DejaVu Sans Mono" font-size="14" fill="#172026">{left_text}</text>',
            f'<text x="{panel_x+430}" y="{y}" text-anchor="end" font-family="DejaVu Sans Mono" font-size="14" fill="#172026">{right_text}</text>',
            f'<line x1="{panel_x}" y1="{y+11}" x2="{panel_x+440}" y2="{y+11}" stroke="#d9dedc"/>',
        ])
        y += 48
    svg.extend([
        f'<text x="{panel_x}" y="590" font-family="DejaVu Sans" font-size="16" font-weight="bold" fill="#172026">Interpretation</text>',
        f'<text x="{panel_x}" y="622" font-family="DejaVu Sans" font-size="14" fill="#35434b">1. The return path is longer and has lower straight-line efficiency.</text>',
        f'<text x="{panel_x}" y="650" font-family="DejaVu Sans" font-size="14" fill="#35434b">2. Compare density exposure to separate obstacle avoidance from semantic detours.</text>',
        f'<text x="{panel_x}" y="678" font-family="DejaVu Sans" font-size="14" fill="#35434b">3. Semantic nodes are the final accumulated snapshot, not a causal replay.</text>',
        f'<text x="{panel_x}" y="730" font-family="DejaVu Sans" font-size="16" font-weight="bold" fill="#172026">Map construction</text>',
        f'<text x="{panel_x}" y="760" font-family="DejaVu Sans Mono" font-size="13" fill="#35434b">PCD frames sampled: {meta["sampled_pointcloud_frames"]}</text>',
        f'<text x="{panel_x}" y="786" font-family="DejaVu Sans Mono" font-size="13" fill="#35434b">Flight-height points read: {meta["accepted_points"]}</text>',
        f'<text x="{panel_x}" y="812" font-family="DejaVu Sans Mono" font-size="13" fill="#35434b">Occupied 0.5 m cells: {meta["occupied_cells"]}</text>',
        f'<text x="{panel_x}" y="838" font-family="DejaVu Sans Mono" font-size="13" fill="#35434b">High-risk semantic nodes: {meta["semantic_nodes"]}</text>',
        '<text x="285" y="905" text-anchor="middle" font-family="DejaVu Sans" font-size="12" fill="#53616a">world x (m)</text>',
        '<text x="24" y="465" text-anchor="middle" transform="rotate(-90 24 465)" font-family="DejaVu Sans" font-size="12" fill="#53616a">world y (m)</text>',
        '</svg>',
    ])
    output_path.write_text("\n".join(svg), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--outbound-start", type=int, required=True)
    parser.add_argument("--outbound-end", type=int, required=True)
    parser.add_argument("--return-start", dest="return_start", type=int, required=True)
    parser.add_argument("--return-end", dest="return_end", type=int, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()

    odom, pointclouds, graphs = load_index(args.session)
    outbound = route(odom, args.outbound_start, args.outbound_end)
    inbound = route(odom, args.return_start, args.return_end)
    occupied, frame_count, accepted_points = load_occupied_cells(
        args.session, odom, pointclouds, args.outbound_start, args.return_end)
    graph_candidates = [item for item in graphs if item[0] <= args.return_end + 2_000_000_000]
    graph_path = graph_candidates[-1][1]
    semantic = high_risk_semantic_points(graph_path)
    bounds = (-30.0, 30.0, -5.0, 145.0)
    grid = density_grid(occupied, 0.5, bounds)
    metrics = {
        "outbound": route_metrics(outbound, occupied, semantic),
        "return": route_metrics(inbound, occupied, semantic),
    }
    meta = {
        "session": str(args.session),
        "graph_snapshot": str(graph_path),
        "sampled_pointcloud_frames": frame_count,
        "accepted_points": accepted_points,
        "occupied_cells": len(occupied),
        "semantic_nodes": len(semantic),
        "density_definition": "occupied 0.5 m cells in a 3 m XY neighborhood; world z 0.3..3.2 m",
        "semantic_definition": "red high-risk points in final graph snapshot; abs(z-1.6) <= 4 m",
    }
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    svg_heatmap(args.output_prefix.with_suffix(".svg"), grid, bounds, outbound, inbound,
                semantic, metrics, meta)
    with args.output_prefix.with_name(args.output_prefix.name + "_metrics.json").open(
            "w", encoding="utf-8") as stream:
        json.dump({"meta": meta, "metrics": metrics}, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"meta": meta, "metrics": metrics}, indent=2))


if __name__ == "__main__":
    main()
