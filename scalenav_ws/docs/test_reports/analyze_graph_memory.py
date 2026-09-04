#!/usr/bin/env python3
"""Summarize graph growth and cumulative-map memory from a ScaleNav session.

The logger does not record process RSS.  This report therefore exposes payload
proxies explicitly: cumulative mapped points times the PCD point stride, the
latest serialized graph snapshot, and the cumulative on-disk point-cloud log.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def parse_json_line(line: str) -> dict:
    line = (line.replace(":-inf", ":-Infinity")
            .replace(":inf", ":Infinity")
            .replace(":nan", ":NaN"))
    return json.loads(line, parse_constant=lambda value: float(value))


def finite(value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def marker_count(markers, names):
    for marker in markers:
        if marker.get("ns") in names:
            return len(marker.get("points") or [])
    return 0


def graph_counts(path: Path) -> tuple[int, int, int]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return 0, 0, 0
    markers = data.get("markers") or []
    nodes = marker_count(markers, {"scalenav_skeleton_nodes", "epic_skeleton_nodes"})
    semantic = marker_count(
        markers, {"scalenav_semantic_points", "epic_semantic_points"}
    )
    edge_points = marker_count(
        markers, {"scalenav_skeleton_edges", "epic_skeleton_edges"}
    )
    # RViz LINE_LIST markers encode one segment with two consecutive points.
    return nodes, edge_points // 2, semantic


def load_records(session: Path):
    graphs = []
    cloud_timings = []
    background_timings = []
    pointcloud_files = []
    index_path = session / "index.jsonl"
    with index_path.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            record = parse_json_line(line)
            kind = record.get("kind")
            stamp = int(record.get("stamp_ns", 0))
            if kind == "graph":
                raw_file = record.get("file") or ""
                graph_path = session / raw_file
                nodes, edges, semantic = graph_counts(graph_path)
                graph_bytes = (
                    graph_path.stat().st_size
                    if graph_path.is_file()
                    else int(record.get("bytes", 0))
                )
                graphs.append({
                    "stamp_ns": stamp,
                    "file": raw_file,
                    "nodes": nodes,
                    "edges": edges,
                    "semantic_nodes": semantic,
                    "graph_snapshot_bytes": graph_bytes,
                })
            elif kind == "timing":
                data = record.get("data") or {}
                if data.get("module") == "cloud":
                    cloud_timings.append((stamp, data))
                elif data.get("module") == "background":
                    background_timings.append((stamp, data))
            elif kind == "pointcloud":
                raw_file = record.get("file") or ""
                pointcloud_path = session / raw_file
                pointcloud_files.append((stamp, pointcloud_path, int(record.get("bytes", 0))))
    graphs.sort(key=lambda row: row["stamp_ns"])
    cloud_timings.sort(key=lambda item: item[0])
    background_timings.sort(key=lambda item: item[0])
    pointcloud_files.sort(key=lambda item: item[0])
    return graphs, cloud_timings, background_timings, pointcloud_files


def prior_value(records, stamps, stamp, field, default=0):
    index = bisect.bisect_right(stamps, stamp) - 1
    if index < 0:
        return default
    return records[index][1].get(field, default)


def build_rows(session: Path, point_bytes: int):
    graphs, cloud, background, pointcloud_files = load_records(session)
    cloud_stamps = [item[0] for item in cloud]
    background_stamps = [item[0] for item in background]
    rows = []
    cumulative_log_bytes = 0
    pointcloud_index = 0
    for index, graph in enumerate(graphs):
        while (
            pointcloud_index < len(pointcloud_files)
            and pointcloud_files[pointcloud_index][0] <= graph["stamp_ns"]
        ):
            _, path, logged_bytes = pointcloud_files[pointcloud_index]
            cumulative_log_bytes += path.stat().st_size if path.is_file() else logged_bytes
            pointcloud_index += 1
        map_points = int(prior_value(cloud, cloud_stamps, graph["stamp_ns"], "map_points", 0))
        background_data = {}
        bg_index = bisect.bisect_right(background_stamps, graph["stamp_ns"]) - 1
        if bg_index >= 0:
            background_data = background[bg_index][1]
        map_payload_bytes = map_points * point_bytes
        graph_bytes = graph["graph_snapshot_bytes"]
        rows.append({
            **graph,
            "index": index + 1,
            "time_s": (graph["stamp_ns"] - graphs[0]["stamp_ns"]) / 1e9,
            "bubbles": int(background_data.get("bubbles", 0)),
            "new_nodes": int(background_data.get("new_nodes", 0)),
            "removed_nodes": int(background_data.get("removed_nodes", 0)),
            "map_points": map_points,
            "map_payload_bytes": map_payload_bytes,
            "current_payload_bytes": map_payload_bytes + graph_bytes,
            "cumulative_pointcloud_log_bytes": cumulative_log_bytes,
        })
    return rows


FIELDS = [
    "index", "time_s", "stamp_ns", "file", "nodes", "edges", "semantic_nodes",
    "bubbles", "new_nodes", "removed_nodes", "map_points",
    "graph_snapshot_bytes", "map_payload_bytes", "current_payload_bytes",
    "cumulative_pointcloud_log_bytes",
    "map_payload_mb", "graph_snapshot_mb", "current_payload_mb", "cumulative_log_mb",
]


def write_csv(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows({field: row[field] for field in FIELDS} for row in rows)


def esc(value):
    return (str(value).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def svg_report(path: Path, rows, session: Path, point_bytes: int):
    width, height = 1200, 790
    panels = [(75, 95, 485, 245), (635, 95, 485, 245),
              (75, 405, 485, 245), (635, 405, 485, 245)]
    colors = ["#1769aa", "#d94841", "#087f5b", "#7f2f72"]
    max_time = max((row["time_s"] for row in rows), default=1.0) or 1.0

    def line_points(rows_, key, panel, maximum):
        x, y, w, h = panel
        maximum = maximum or 1.0
        return " ".join(
            f"{x + row['time_s'] / max_time * w:.1f},{y + h - float(row[key]) / maximum * h:.1f}"
            for row in rows_
        )

    def panel(title, keys, labels, units, panel_index):
        x, y, w, h = panels[panel_index]
        maximum = max(max(float(row[key]) for row in rows) for key in keys) or 1.0
        svg = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#fff" stroke="#9aa4a9"/>']
        for tick in range(5):
            yy = y + h - tick * h / 4
            value = maximum * tick / 4
            svg.append(f'<line x1="{x}" y1="{yy:.1f}" x2="{x+w}" y2="{yy:.1f}" stroke="#e1e5e7"/>')
            value_text = f"{value:.2f}" if units == "MB" else f"{value:.0f}"
            svg.append(f'<text x="{x-8}" y="{yy+4:.1f}" text-anchor="end" font-size="11" fill="#53616a">{value_text}</text>')
        for tick in range(5):
            xx = x + tick * w / 4
            elapsed = max_time * tick / 4
            svg.append(f'<line x1="{xx:.1f}" y1="{y}" x2="{xx:.1f}" y2="{y+h}" stroke="#edf0f1"/>')
            svg.append(f'<text x="{xx:.1f}" y="{y+h+17}" text-anchor="middle" font-size="10" fill="#53616a">{elapsed:.0f}</text>')
        svg.append(f'<text x="{x+8}" y="{y+22}" font-size="16" font-weight="bold" fill="#172026">{esc(title)}</text>')
        for i, key in enumerate(keys):
            svg.append(f'<polyline points="{line_points(rows, key, (x,y,w,h), maximum)}" fill="none" stroke="{colors[i]}" stroke-width="2.5"/>')
            lx = x + 12 + i * 145
            svg.append(f'<line x1="{lx}" y1="{y+42}" x2="{lx+22}" y2="{y+42}" stroke="{colors[i]}" stroke-width="3"/>')
            svg.append(f'<text x="{lx+28}" y="{y+46}" font-size="11" fill="#172026">{esc(labels[i])}</text>')
        svg.append(f'<text x="{x+w/2}" y="{y+h+25}" text-anchor="middle" font-size="11" fill="#53616a">time since first graph snapshot (s)</text>')
        return svg

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="1200" height="790" fill="#f4f5f2"/>',
           '<text x="60" y="38" font-size="25" font-weight="bold" fill="#172026">ScaleNav graph growth and cumulative-map memory</text>',
           f'<text x="60" y="62" font-size="13" fill="#53616a">{esc(session)} | {len(rows)} graph snapshots | PCD payload stride {point_bytes} B/point</text>']
    svg += panel(
        "Graph vertices", ["nodes", "semantic_nodes"],
        ["graph nodes", "semantic nodes"], "count", 0,
    )
    svg += panel("Graph edges", ["edges"], ["witness edge segments"], "count", 1)
    svg += panel("Retained cumulative map", ["map_points"], ["retained map points"], "count", 2)
    svg += panel(
        "Payload memory proxies",
        ["map_payload_mb", "graph_snapshot_mb", "current_payload_mb"],
        ["map payload", "graph snapshot", "map + graph"], "MB", 3,
    )
    final_log_mb = rows[-1]["cumulative_log_mb"]
    svg.append('<text x="60" y="724" font-size="12" fill="#53616a">RSS was not logged. Runtime payload proxy = retained map points × point stride + latest serialized graph snapshot.</text>')
    svg.append(f'<text x="60" y="746" font-size="12" fill="#53616a">Saved point-cloud sequence is a separate on-disk cost: {final_log_mb:.2f} MB cumulative at the final snapshot; it is retained in the CSV but not mixed into the runtime plot.</text>')
    svg.append('</svg>')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(svg), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-svg", type=Path, required=True)
    parser.add_argument("--point-bytes", type=int, default=16,
                        help="payload bytes per retained map point (default: PCD point_step 16)")
    args = parser.parse_args()
    if args.point_bytes <= 0:
        parser.error("--point-bytes must be positive")
    rows = build_rows(args.session, args.point_bytes)
    if not rows:
        parser.error(f"no graph snapshots found in {args.session}")
    for row in rows:
        row["map_payload_mb"] = row["map_payload_bytes"] / 1e6
        row["graph_snapshot_mb"] = row["graph_snapshot_bytes"] / 1e6
        row["current_payload_mb"] = row["current_payload_bytes"] / 1e6
        row["cumulative_log_mb"] = row["cumulative_pointcloud_log_bytes"] / 1e6
    write_csv(args.output_csv, rows)
    svg_report(args.output_svg, rows, args.session, args.point_bytes)
    last = rows[-1]
    print(f"graph snapshots: {len(rows)}")
    print(
        "final nodes/edges/semantic: "
        f"{last['nodes']}/{last['edges']}/{last['semantic_nodes']}"
    )
    print(f"final map points: {last['map_points']} ({last['map_payload_mb']:.2f} MB payload)")
    print(f"final graph snapshot: {last['graph_snapshot_mb']:.2f} MB")
    print(f"final map + graph payload proxy: {last['current_payload_mb']:.2f} MB")
    print(f"cumulative point-cloud log: {last['cumulative_log_mb']:.2f} MB")


if __name__ == "__main__":
    main()
