#!/usr/bin/env python3
"""Spatial graph/Depth consistency report for an EPIC online run.

The graph snapshot stores world-frame TopoNode centers.  This tool joins the
nearest depth and odometry records from the same event log, transforms nodes
to body FLU, and reports spatial bins rather than only aggregate node counts.
It intentionally uses only the standard library so it can run on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def read_jsonl(path: Path, event: str | None = None) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    if not path.exists():
        return records
    for line in path.read_text(errors="replace").splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if event is None or item.get("event") == event:
            records.append(item)
    return records


def quat_rotate_inverse(q: list[float], v: list[float]) -> tuple[float, float, float]:
    # q is [x,y,z,w], and this is the conjugate rotation q^-1 * v * q.
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx - w * tx + (y * tz - z * ty),
        vy - w * ty + (z * tx - x * tz),
        vz - w * tz + (x * ty - y * tx),
    )


def nearest(records: list[dict[str, Any]], stamp: float) -> dict[str, Any] | None:
    if not records:
        return None
    return min(records, key=lambda item: abs(float(item.get("wall_time", 0.0)) - stamp))


def body_node(node: dict[str, Any], odom: dict[str, Any] | None) -> tuple[float, float, float]:
    center = node["center"]
    if not odom:
        return float(center[0]), float(center[1]), float(center[2])
    position = odom.get("position_world", [0.0, 0.0, 0.0])
    q = odom.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
    return quat_rotate_inverse(q, [center[i] - position[i] for i in range(3)])


def make_report(graph_path: Path, events_path: Path, tolerance_s: float = 0.75) -> dict[str, Any]:
    snapshots = read_jsonl(graph_path, "graph_snapshot")
    depths = read_jsonl(events_path, "depth")
    odoms = read_jsonl(events_path, "odom")
    if not snapshots:
        raise SystemExit(f"no graph_snapshot records in {graph_path}")
    snapshot = snapshots[-1]
    stamp = float(snapshot.get("wall_time", 0.0))
    depth = nearest(depths, stamp)
    odom = nearest(odoms, stamp)
    depth_delta = abs(float(depth.get("wall_time", 0.0)) - stamp) if depth else None
    odom_delta = abs(float(odom.get("wall_time", 0.0)) - stamp) if odom else None
    nodes = snapshot.get("nodes", [])
    body = [dict(node, body=list(body_node(node, odom))) for node in nodes]
    non_odom = [node for node in body if node.get("role") != "odom"]
    geometric = [node for node in non_odom if node.get("role") == "geometric"]
    speculative = [node for node in non_odom if node.get("role") == "speculative"]

    bins: dict[str, dict[str, int]] = {}
    for node in non_odom:
        x, y, _ = node["body"]
        bx = math.floor(x / 5.0) * 5
        by = math.floor(y / 2.5) * 2.5
        key = f"x={bx:g}..{bx + 5:g},y={by:g}..{by + 2.5:g}"
        entry = bins.setdefault(key, {"all": 0, "geometric": 0, "speculative": 0, "isolated": 0})
        entry["all"] += 1
        entry[node.get("role") if node.get("role") in ("geometric", "speculative") else "geometric"] += 1
        if int(node.get("degree", 0)) == 0:
            entry["isolated"] += 1

    duplicate_pairs: list[dict[str, Any]] = []
    for i, left in enumerate(non_odom):
        for right in non_odom[i + 1 :]:
            delta = math.dist(left["center"], right["center"])
            if delta < 0.25:
                duplicate_pairs.append({"left": left.get("persistent_id"), "right": right.get("persistent_id"), "distance_m": delta})

    forward_nodes = [node for node in non_odom if 0.0 < node["body"][0] <= 20.0 and abs(node["body"][1]) <= node["body"][0] * math.tan(math.radians(45.0))]
    depth_stats = (depth or {}).get("model_depth_m_p05_median_center", [])
    center_depth_m = float(depth_stats[1]) if len(depth_stats) > 1 else None
    beyond_center = [node for node in forward_nodes if center_depth_m is not None and node["body"][0] > center_depth_m + 1.0]
    report = {
        "source": {"graph": str(graph_path), "events": str(events_path), "snapshot_count": len(snapshots)},
        "join": {"snapshot_wall_time": stamp, "depth_delta_s": depth_delta, "odom_delta_s": odom_delta, "within_tolerance": bool(depth_delta is not None and odom_delta is not None and depth_delta <= tolerance_s and odom_delta <= tolerance_s)},
        "summary": {
            "node_count": len(nodes), "geometric": len(geometric), "speculative": len(speculative),
            "edge_count": snapshot.get("edge_count"), "directed_edge_count": snapshot.get("directed_edge_count"),
            "asymmetric_edge_count": snapshot.get("asymmetric_edge_count"), "dangling_neighbor_count": snapshot.get("dangling_neighbor_count"),
            "zero_degree_nodes": snapshot.get("zero_degree_nodes"), "duplicate_pairs_lt_25cm": len(duplicate_pairs),
            "forward_fov_nodes_0_20m": len(forward_nodes), "forward_nodes_beyond_center_depth": len(beyond_center),
        },
        "depth": {"record": depth, "center_median_m": center_depth_m},
        "spatial_bins": bins,
        "duplicate_pairs": duplicate_pairs[:200],
        "nodes_beyond_center_depth": [{"persistent_id": node.get("persistent_id"), "body": node["body"], "degree": node.get("degree"), "role": node.get("role")} for node in beyond_center[:200]],
    }
    return report


def html(report: dict[str, Any]) -> str:
    summary = report["summary"]
    rows = "".join(f"<tr><td>{key}</td><td>{value}</td></tr>" for key, value in summary.items())
    bins = "".join(f"<tr><td>{key}</td><td>{value['all']}</td><td>{value['geometric']}</td><td>{value['speculative']}</td><td>{value['isolated']}</td></tr>" for key, value in sorted(report["spatial_bins"].items()))
    duplicates = "".join(f"<tr><td>{item['left']}</td><td>{item['right']}</td><td>{item['distance_m']:.3f}</td></tr>" for item in report["duplicate_pairs"][:50])
    return f"""<!doctype html><meta charset='utf-8'><title>EPIC Graph/Depth Report</title>
<style>body{{font:14px system-ui;margin:24px;color:#20262a}}table{{border-collapse:collapse;margin:12px 0 24px}}td,th{{border:1px solid #ccd3d7;padding:5px 9px;text-align:left}}h1{{font-size:22px}}.bad{{color:#b52e38;font-weight:600}}.ok{{color:#18794e;font-weight:600}}code{{font-size:12px}}</style>
<h1>EPIC 图节点空间分布与深度核对</h1><p>graph: <code>{report['source']['graph']}</code><br>events: <code>{report['source']['events']}</code></p>
<h2>摘要</h2><table><tr><th>指标</th><th>值</th></tr>{rows}</table>
<p class='{'ok' if report['join']['within_tolerance'] else 'bad'}'>时间对齐：depth Δ={report['join']['depth_delta_s']} s，odom Δ={report['join']['odom_delta_s']} s；中心深度中位数={report['depth']['center_median_m']} m</p>
<h2>空间分箱（机体 FLU）</h2><table><tr><th>区域</th><th>节点</th><th>几何</th><th>语义候选</th><th>零度</th></tr>{bins}</table>
<h2>重复节点（中心距 &lt; 0.25 m）</h2><table><tr><th>persistent A</th><th>persistent B</th><th>距离 m</th></tr>{duplicates or '<tr><td colspan=3>无</td></tr>'}</table>
<h2>解释</h2><p>零度节点代表已进入持久图但没有成功边；重复节点代表 Bubble 聚类/增量匹配仍在产生重叠顶点。深度中心中位数只能做量程 sanity check，不能代替完整深度点云，因此“超出中心深度”的节点需要结合对应 FOV 像素再判定。</p>"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", type=Path, default=Path("log_event/epic_graph_snapshots.jsonl"))
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--json", type=Path, default=Path("log_event/epic_graph_depth_report.json"))
    parser.add_argument("--html", type=Path, default=Path("log_event/epic_graph_depth_report.html"))
    args = parser.parse_args()
    report = make_report(args.graph, args.events)
    args.json.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    args.html.write_text(html(report))
    print(json.dumps(report["summary"], ensure_ascii=False, sort_keys=True))
    print(f"json: {args.json}\nhtml: {args.html}")


if __name__ == "__main__":
    main()
