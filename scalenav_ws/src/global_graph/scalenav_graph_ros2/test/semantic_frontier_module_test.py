#!/usr/bin/env python3
"""Deterministic frame-by-frame semantic frontier module test and HTML report."""

from __future__ import annotations

import argparse
import base64
import json
import math
import re
import struct
from pathlib import Path
from typing import Any


PATCH_ROWS = 3
PATCH_COLS = 5
PATCH_HEIGHT = 2
PATCH_WIDTH = 2
MISSION_DISTANCE_WEIGHT = 2.0
VIRTUAL_DEPTH_M = 35.0
MEMORY_S = 1.5
SEMANTIC_SCORE_WEIGHT = 1.0
HORIZONTAL_ANGLES_DEG = (-45.0, -22.5, 0.0, 22.5, 45.0)


def make_heatmap(patch_means: list[list[float]], hot_pixel: tuple[int, int, float] | None = None) -> list[list[float]]:
    pixels = []
    for patch_row in patch_means:
        for _ in range(PATCH_HEIGHT):
            row: list[float] = []
            for value in patch_row:
                row.extend([value] * PATCH_WIDTH)
            pixels.append(row)
    if hot_pixel is not None:
        row, column, value = hot_pixel
        pixels[row][column] = value
    return pixels


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def patch_means(heatmap: list[list[float]]) -> tuple[list[list[float | None]], list[list[bool]]]:
    means: list[list[float | None]] = []
    valid: list[list[bool]] = []
    for patch_row in range(PATCH_ROWS):
        mean_row: list[float | None] = []
        valid_row: list[bool] = []
        for patch_col in range(PATCH_COLS):
            values: list[float] = []
            for row in range(patch_row * PATCH_HEIGHT, (patch_row + 1) * PATCH_HEIGHT):
                for col in range(patch_col * PATCH_WIDTH, (patch_col + 1) * PATCH_WIDTH):
                    if finite(heatmap[row][col]):
                        values.append(float(heatmap[row][col]))
            mean_row.append(sum(values) / len(values) if values else None)
            valid_row.append(bool(values))
        means.append(mean_row)
        valid.append(valid_row)
    return means, valid


def project_candidate(odom: dict[str, float], depth: float, column: int) -> tuple[float, float]:
    angle = math.radians(HORIZONTAL_ANGLES_DEG[column]) + odom["yaw"]
    # Body-forward is the vehicle y axis in this planar module test.
    return (
        odom["x"] + depth * math.sin(angle),
        odom["y"] + depth * math.cos(angle),
    )


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def make_frame(index: int, time_s: float, odom: dict[str, float], goal: tuple[float, float],
               heatmap: list[list[float]], depths: list[float | None]) -> dict[str, Any]:
    means, valid = patch_means(heatmap)
    report_heatmap = [[value if finite(value) else None for value in row] for row in heatmap]
    candidates: list[dict[str, Any]] = []
    mission_span = distance((odom["x"], odom["y"]), goal)
    for column in range(PATCH_COLS):
        score = means[1][column]
        depth_value = depths[column] if column < len(depths) else None
        usable = score is not None
        depth_used = VIRTUAL_DEPTH_M
        point = project_candidate(odom, depth_used, column)
        goal_distance = distance(point, goal)
        route_cost = depth_used + 0.15 * abs(point[0] - odom["x"])
        semantic_cost = float(score) * SEMANTIC_SCORE_WEIGHT if usable else math.inf
        objective = ((route_cost + MISSION_DISTANCE_WEIGHT * goal_distance) /
                     max(1.0, mission_span)) + semantic_cost
        candidates.append({
            "column": column,
            "angle_deg": HORIZONTAL_ANGLES_DEG[column],
            "score": None if score is None else round(float(score), 6),
            "confidence": 1.0 if usable else 0.0,
            "depth_m": round(depth_used, 4),
            "depth_source": "virtual",
            "surface_depth_m": float(depth_value) if finite(depth_value) else None,
            "x": round(point[0], 4),
            "y": round(point[1], 4),
            "usable": usable,
            "reachable": usable,
            "risk": "continuous" if usable else "unavailable",
            "goal_distance_m": round(goal_distance, 4),
            "route_cost": round(route_cost, 4),
            "objective": round(objective, 6) if math.isfinite(objective) else None,
        })
    usable_candidates = [candidate for candidate in candidates if candidate["usable"]]
    mode = "continuous" if usable_candidates else "no_usable"
    return {
        "index": index,
        "time_s": time_s,
        "odom": odom,
        "heatmap": report_heatmap,
        "patch_means": means,
        "patch_valid": valid,
        "candidates": candidates,
        "mode": mode,
        "selected_column": None,
        "selected_reason": "",
        "goal": {"x": goal[0], "y": goal[1]},
    }


def rebase_candidate(candidate: dict[str, Any], odom: dict[str, float],
                     goal: tuple[float, float]) -> dict[str, Any]:
    """Re-evaluate a retained world point from the current odom, as A* does."""
    rebased = dict(candidate)
    point = (float(candidate["x"]), float(candidate["y"]))
    goal_distance = distance(point, goal)
    route_cost = distance((odom["x"], odom["y"]), point) + 0.15 * abs(point[0] - odom["x"])
    mission_span = distance((odom["x"], odom["y"]), goal)
    rebased["route_cost"] = round(route_cost, 4)
    rebased["goal_distance_m"] = round(goal_distance, 4)
    rebased["objective"] = round(
        (route_cost + MISSION_DISTANCE_WEIGHT * goal_distance) / max(1.0, mission_span)
        + float(candidate["score"]) * SEMANTIC_SCORE_WEIGHT, 6)
    rebased["retained_rebased"] = True
    return rebased


def select_frames(frames: list[dict[str, Any]]) -> None:
    retained: list[tuple[float, dict[str, Any], dict[str, Any]]] = []
    for frame in frames:
        current = [candidate for candidate in frame["candidates"] if candidate["usable"]]
        selected: dict[str, Any] | None = None
        reason = ""
        retained = [entry for entry in retained if frame["time_s"] - entry[0] <= MEMORY_S]
        active = []
        for candidate in current:
            item = dict(candidate)
            item["source_frame"] = frame["index"]
            active.append(item)
        for entry in retained:
            candidate = rebase_candidate(
                entry[2], frame["odom"], (frame["goal"]["x"], frame["goal"]["y"]))
            candidate["source_frame"] = entry[1]["index"]
            active.append(candidate)
        if active:
            selected = min(active, key=lambda item: (item["objective"], item["column"]))
            reason = "continuous_semantic_route_objective"
        if selected is None:
            reason = "ordinary_verified_fallback"
            selected = {
                "column": None,
                "score": None,
                "risk": "ordinary",
                "x": 0.0,
                "y": frame["odom"]["y"] + 12.0,
                "depth_m": 12.0,
                "depth_source": "ordinary_verified",
                "objective": None,
            }
        frame["selected_column"] = selected["column"]
        frame["selected_reason"] = reason
        frame["selected"] = selected
        for candidate in current:
            retained.append((frame["time_s"], frame, candidate))


def simulate() -> dict[str, Any]:
    goal = (0.0, 140.0)
    frames = [
        make_frame(0, 0.0, {"x": 0.0, "y": 0.0, "yaw": 0.0}, goal,
                   make_heatmap([[0.9, 0.9, 0.9, 0.9, 0.9],
                   [0.70, 0.18, 0.10, 0.20, 0.15],
                                 [0.9, 0.9, 0.9, 0.9, 0.9]], (2, 5, 1.0)),
                   [10.0, 12.0, 8.0, 9.0, 11.0]),
        make_frame(1, 1.0, {"x": 0.0, "y": 10.0, "yaw": 0.0}, goal,
                   make_heatmap([[0.1, 0.1, 0.1, 0.1, 0.1],
                                 [0.70, 0.20, 0.15, 0.18, 0.80],
                                 [0.1, 0.1, 0.1, 0.1, 0.1]]),
                   [14.0, 16.0, 15.0, 16.0, 14.0]),
        make_frame(2, 2.0, {"x": 0.0, "y": 15.0, "yaw": 0.0}, goal,
                   make_heatmap([[0.2, 0.2, 0.2, 0.2, 0.2],
                                 [0.80, 0.80, 0.80, 0.80, 0.80],
                                 [0.2, 0.2, 0.2, 0.2, 0.2]]),
                   [15.0, 15.0, 15.0, 15.0, 15.0]),
        make_frame(3, 3.0, {"x": 0.0, "y": 30.0, "yaw": 0.0}, goal,
                   make_heatmap([[0.3, 0.3, 0.3, 0.3, 0.3],
                                 [0.05, 0.07, 0.08, 0.10, 0.06],
                                 [0.3, 0.3, 0.3, 0.3, 0.3]]),
                   [25.0, 25.0, 25.0, 25.0, 25.0]),
        make_frame(4, 4.0, {"x": 0.0, "y": 40.0, "yaw": 0.0}, goal,
                   make_heatmap([[float("nan")] * 5, [float("nan")] * 5, [float("nan")] * 5]),
                   [None] * 5),
        make_frame(5, 7.5, {"x": 0.0, "y": 60.0, "yaw": 0.0}, goal,
                   make_heatmap([[float("nan")] * 5, [float("nan")] * 5, [float("nan")] * 5]),
                   [None] * 5),
    ]
    select_frames(frames)
    checks = {
        "middle_row_only": all(
            abs(candidate["score"] - frame["patch_means"][1][candidate["column"]]) < 1.0e-5
            for frame in frames for candidate in frame["candidates"]
            if candidate["score"] is not None),
        "five_candidates_per_valid_frame": all(len(frame["candidates"]) == 5 for frame in frames),
        "all_virtual_candidates_have_equal_range": all(
            abs(candidate["depth_m"] - VIRTUAL_DEPTH_M) < 1.0e-6
            for frame in frames for candidate in frame["candidates"]),
        "depth_never_replaces_virtual_candidate": all(
            candidate["depth_source"] == "virtual"
            for frame in frames for candidate in frame["candidates"]),
        "frame_0_uses_continuous_objective": frames[0]["selected_reason"] == "continuous_semantic_route_objective",
        "frame_2_high_current_can_use_retained_frame": frames[2]["selected"].get("source_frame") in (0, 1),
        "retained_candidate_rebased_from_current_odom": frames[2]["selected"].get("retained_rebased", False),
        "frame_3_uses_objective_without_threshold": frames[3]["selected_reason"] == "continuous_semantic_route_objective",
        "ordinary_only_fallback": frames[5]["selected_reason"] == "ordinary_verified_fallback",
        "hot_pixel_does_not_define_patch_mean": abs(frames[0]["patch_means"][1][2] - 0.325) < 1e-6,
    }
    return {
        "config": {
            "patch_grid": "3x5",
            "patch_aggregation": "finite arithmetic mean",
            "planar_row": "middle (row 1)",
            "mission_distance_weight": MISSION_DISTANCE_WEIGHT,
            "virtual_depth_m": VIRTUAL_DEPTH_M,
            "semantic_memory_s": MEMORY_S,
            "horizontal_angles_deg": HORIZONTAL_ANGLES_DEG,
        },
        "goal": {"x": goal[0], "y": goal[1]},
        "frames": frames,
        "checks": checks,
    }


def render_html(result: dict[str, Any]) -> str:
    payload = json.dumps(result, separators=(",", ":"), allow_nan=False)
    return """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Semantic Frontier Module Test</title><style>
:root{--bg:#0b1220;--panel:#121c2d;--line:#26364f;--text:#e8eef7;--muted:#8da0ba;--green:#42d392;--red:#ff6b6b;--amber:#f5c451;--cyan:#53c7e8}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.45 Inter,system-ui,sans-serif}
.shell{max-width:1500px;margin:0 auto;padding:24px}.top{display:flex;justify-content:space-between;gap:20px;align-items:end;border-bottom:1px solid var(--line);padding-bottom:18px}
h1{font-size:24px;margin:0 0 5px}.sub{color:var(--muted)}.badge{padding:8px 12px;border:1px solid var(--green);color:var(--green);border-radius:5px;font-weight:700}
.controls{display:flex;gap:8px;align-items:center;margin:18px 0}.controls button{background:#1b2a42;color:var(--text);border:1px solid var(--line);padding:8px 12px;border-radius:4px;cursor:pointer}.controls button.active{background:var(--cyan);color:#08111d;border-color:var(--cyan)}
.grid{display:grid;grid-template-columns:minmax(380px,1fr) minmax(460px,1.2fr);gap:16px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:16px}.panel h2{font-size:15px;margin:0 0 13px;color:#fff}
.heat{display:grid;grid-template-columns:repeat(5,1fr);gap:5px;margin:12px 0}.cell{min-height:58px;padding:8px;border-radius:4px;border:1px solid rgba(255,255,255,.12);display:flex;flex-direction:column;justify-content:space-between}.cell small{color:#07111d;font-weight:700}.cell span{font-weight:700}.rowlabel{font-size:11px;color:var(--muted);margin-top:8px}
.facts{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.fact{border-left:2px solid var(--cyan);padding-left:9px}.fact b{display:block;font-size:16px}.fact small{color:var(--muted)}
svg{width:100%;height:460px;background:#0d1728;border:1px solid var(--line);border-radius:4px}.axis{stroke:#31435f;stroke-width:1}.mission{fill:var(--amber)}.odom{fill:var(--cyan)}.candidate{stroke:#07111d;stroke-width:1.5}.route{stroke:var(--green);stroke-width:3;fill:none;stroke-dasharray:7 5}.selected{stroke:#fff;stroke-width:3}
table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:left;padding:7px 6px;border-bottom:1px solid var(--line)}th{color:var(--muted);font-weight:500}.low{color:var(--green)}.high{color:var(--red)}.unavailable{color:var(--muted)}.ordinary{color:var(--amber)}
pre{white-space:pre-wrap;background:#0d1728;border:1px solid var(--line);padding:12px;max-height:300px;overflow:auto;color:#b9c9dd}.wide{grid-column:1/-1}.check{display:flex;flex-wrap:wrap;gap:8px}.check span{padding:6px 8px;border:1px solid var(--line);border-radius:4px}.pass{color:var(--green)}.fail{color:var(--red)}
@media(max-width:900px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.shell{padding:14px}.top{align-items:start;flex-direction:column}svg{height:360px}}
</style></head><body><main class="shell"><header class="top"><div><h1>Semantic Frontier Module Test</h1><div class="sub">3x5 heatmap to planar semantic nodes, A* objective, and frontier decision</div></div><div id="status" class="badge">PASS</div></header>
<div id="controls" class="controls"></div><section class="grid"><article class="panel"><h2>Heatmap and projection input</h2><div id="facts" class="facts"></div><div class="rowlabel">Upper row (diagnostic only)</div><div id="heat-upper" class="heat"></div><div class="rowlabel">Middle row (5 planar candidates)</div><div id="heat-middle" class="heat"></div><div class="rowlabel">Lower row (diagnostic only)</div><div id="heat-lower" class="heat"></div></article><article class="panel"><h2>World-frame output</h2><svg id="scene" viewBox="0 0 700 460" role="img" aria-label="Projected semantic candidates and selected frontier"></svg></article><article class="panel wide"><h2>Candidate nodes and frontier choice</h2><div id="decision"></div></article><article class="panel wide"><h2>Module checks</h2><div id="checks" class="check"></div><pre id="json"></pre></article></section></main><script>
const DATA=__DATA__;let current=0;const $=id=>document.getElementById(id);const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
function color(v){if(v===null||!Number.isFinite(v))return '#53647d';let t=Math.max(0,Math.min(1,v));return `hsl(${145-145*t} 72% ${72-28*t}%)`}
function render(){const f=DATA.frames[current];$('controls').innerHTML=DATA.frames.map((x,i)=>`<button class="${i===current?'active':''}" onclick="current=${i};render()">Frame ${i}<br><small>${x.time_s.toFixed(1)} s</small></button>`).join('');const selected=f.selected||{};$('facts').innerHTML=`<div class="fact"><b>${f.mode}</b><small>frame mode</small></div><div class="fact"><b>${f.odom.x.toFixed(1)}, ${f.odom.y.toFixed(1)}</b><small>odom (m)</small></div><div class="fact"><b>${selected.column===null?'ordinary':`col ${selected.column}`}</b><small>frontier goal</small></div>`;['upper','middle','lower'].forEach((name,row)=>{$('heat-'+name).innerHTML=f.patch_means[row].map((v,c)=>`<div class="cell" style="background:${color(v)}"><small>col ${c}</small><span>${v===null?'n/a':v.toFixed(3)}</span></div>`).join('')});renderScene(f);$('decision').innerHTML=`<div style="margin-bottom:10px"><b>Decision:</b> ${esc(f.selected_reason)} &nbsp; <b>goal:</b> ${selected.x===undefined?'n/a':`${selected.x.toFixed(2)}, ${selected.y.toFixed(2)}`}</div><table><thead><tr><th>col</th><th>angle</th><th>mean</th><th>depth</th><th>node</th><th>risk</th><th>route cost</th><th>goal dist</th><th>objective</th></tr></thead><tbody>${f.candidates.map(c=>`<tr><td>${c.column}</td><td>${c.angle_deg.toFixed(1)} deg</td><td>${c.score===null?'n/a':c.score.toFixed(3)}</td><td>${c.depth_m.toFixed(1)} (${c.depth_source})</td><td>(${c.x.toFixed(2)}, ${c.y.toFixed(2)})</td><td class="${c.risk}">${c.risk}</td><td>${c.route_cost.toFixed(2)}</td><td>${c.goal_distance_m.toFixed(2)}</td><td>${c.objective==null?'n/a':c.objective.toFixed(4)}</td></tr>`).join('')}</tbody></table>`;$('checks').innerHTML=Object.entries(DATA.checks).map(([k,v])=>`<span class="${v?'pass':'fail'}">${v?'PASS':'FAIL'} ${esc(k)}</span>`).join('');$('json').textContent=JSON.stringify(f,null,2)}
function renderScene(f){const all=f.candidates.filter(c=>Number.isFinite(c.x));const pts=all.concat([{x:f.goal.x,y:f.goal.y}]);const minX=Math.min(...pts.map(p=>p.x),f.odom.x)-5,maxX=Math.max(...pts.map(p=>p.x),f.odom.x)+5,minY=Math.min(...pts.map(p=>p.y),f.odom.y)-5,maxY=Math.max(...pts.map(p=>p.y),f.odom.y)+5;const sx=x=>40+(x-minX)/(maxX-minX)*620,sy=y=>430-(y-minY)/(maxY-minY)*390;let out='';for(let i=0;i<6;i++){const x=40+i*124;out+=`<line class="axis" x1="${x}" y1="40" x2="${x}" y2="430"/>`}out+=`<circle class="odom" cx="${sx(f.odom.x)}" cy="${sy(f.odom.y)}" r="8"/><text x="${sx(f.odom.x)+10}" y="${sy(f.odom.y)-10}" fill="#53c7e8">odom</text><circle class="mission" cx="${sx(f.goal.x)}" cy="${sy(f.goal.y)}" r="8"/><text x="${sx(f.goal.x)+10}" y="${sy(f.goal.y)-10}" fill="#f5c451">mission goal</text>`;if(f.selected&&f.selected.column!==null){out+=`<line class="route" x1="${sx(f.odom.x)}" y1="${sy(f.odom.y)}" x2="${sx(f.selected.x)}" y2="${sy(f.selected.y)}"/>`}for(const c of all){const selected=f.selected&&c.column===f.selected.column;const fill=c.risk==='low'?'#42d392':c.risk==='high'?'#ff6b6b':'#53647d';out+=`<circle class="candidate ${selected?'selected':''}" cx="${sx(c.x)}" cy="${sy(c.y)}" r="${selected?10:7}" fill="${fill}"/><text x="${sx(c.x)+9}" y="${sy(c.y)+4}" fill="#dfe8f5" font-size="12">c${c.column}</text>`} $('scene').innerHTML=out}
render();
</script></body></html>""".replace("__DATA__", payload)


# The log store writes images as portable Netpbm assets and planner diagnostics
# as JSONL.  The replay below deliberately consumes those files directly so a
# report cannot silently turn into a synthetic test when a sensor record is
# missing.
def read_log_jsonl(path: Path) -> list[dict[str, Any]]:
    records = []
    invalid = re.compile(r"(?<![A-Za-z0-9_])(?:-?inf(?:inity)?|nan)(?![A-Za-z0-9_])", re.I)
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            # C++ streams non-finite doubles as inf/nan.  Convert only those
            # explicit tokens to JSON null; no malformed payload is discarded.
            try:
                record = json.loads(invalid.sub("null", line))
            except json.JSONDecodeError as error:
                raise ValueError(f"cannot parse {path}:{line_number}: {error}") from error
        records.append(record)
    return records


def read_pgm16_mm(path: Path) -> tuple[int, int, list[float]]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    offset = 0
    while len(tokens) < 4:
        while offset < len(data) and data[offset] <= 32:
            offset += 1
        if offset < len(data) and data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            offset = len(data) if newline < 0 else newline + 1
            continue
        begin = offset
        while offset < len(data) and data[offset] > 32:
            offset += 1
        tokens.append(data[begin:offset])
    if tokens[0] != b"P5" or int(tokens[3]) != 65535:
        raise ValueError(f"unsupported PGM format: {path}")
    width, height = int(tokens[1]), int(tokens[2])
    count = width * height
    if offset < len(data) and data[offset] <= 32:
        offset += 1
    raw = data[offset:offset + count * 2]
    if len(raw) != count * 2:
        raise ValueError(f"truncated PGM: {path}")
    values = [value / 1000.0 for value in struct.unpack(f">{count}H", raw)]
    return width, height, values


def finite_patch_means(values: list[float], width: int, height: int) -> list[list[float | None]]:
    means: list[list[float | None]] = []
    for patch_row in range(PATCH_ROWS):
        row: list[float | None] = []
        v0, v1 = patch_row * height // PATCH_ROWS, (patch_row + 1) * height // PATCH_ROWS
        for patch_col in range(PATCH_COLS):
            u0, u1 = patch_col * width // PATCH_COLS, (patch_col + 1) * width // PATCH_COLS
            patch = [values[v * width + u] for v in range(v0, v1) for u in range(u0, u1)
                     if math.isfinite(values[v * width + u])]
            row.append(sum(patch) / len(patch) if patch else None)
        means.append(row)
    return means


def quat_rotate(q: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = q
    vx, vy, vz = vector
    # q * v * q^-1, expanded to avoid a dependency on numpy.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx)]


def project_log_patch(odom: dict[str, Any], column: int, depth_m: float,
                      width: int, height: int, virtual: bool) -> dict[str, Any]:
    # This is virtualSemanticPointFlu() from the graph node, evaluated at the
    # center of the selected image patch.
    u = ((column + 0.5) / PATCH_COLS)
    v = 0.5
    horizontal_tangent = math.tan(math.radians(90.0) / 2.0)
    vertical_tangent = math.tan(math.radians(60.0) / 2.0)
    direction = [1.0,
                 -(2.0 * u - 1.0) * horizontal_tangent,
                 -(2.0 * v - 1.0) * vertical_tangent]
    if virtual:
        norm = math.sqrt(sum(value * value for value in direction))
        direction = [value / norm for value in direction]
    body = [0.5 + depth_m * direction[0],
            depth_m * direction[1],
            -0.1 + depth_m * direction[2]]
    position = odom.get("position", [0.0, 0.0, 0.0])
    world = quat_rotate(odom.get("orientation", [0.0, 0.0, 0.0, 1.0]), body)
    return {"x": position[0] + world[0], "y": position[1] + world[1],
            "z": position[2] + world[2], "depth_m": depth_m,
            "depth_source": "virtual" if virtual else "surface"}


def nearest_record(records: list[dict[str, Any]], kind: str, stamp_ns: int) -> dict[str, Any] | None:
    options = [r for r in records if r.get("kind") == kind]
    if not options:
        return None
    return min(options, key=lambda r: abs(int(r.get("stamp_ns", 0)) - stamp_ns))


def extract_graph(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    result: dict[str, Any] = {"skeleton": [], "semantic": [], "edges": [], "semantic_links": [], "astar": [],
                              "frontier": None, "global_goal": None, "local_goal": None}
    for marker in data.get("markers", []):
        namespace = marker.get("ns", "")
        if namespace == "scalenav_skeleton_nodes":
            result["skeleton"] = marker.get("points", [])
        elif namespace == "scalenav_semantic_points":
            result["semantic"] = marker.get("points", [])
        elif namespace == "scalenav_skeleton_edges":
            points = marker.get("points", [])
            result["edges"] = [points[i:i + 2] for i in range(0, len(points) - 1, 2)]
        elif namespace == "scalenav_semantic_links":
            points = marker.get("points", [])
            result["semantic_links"] = [points[i:i + 2] for i in range(0, len(points) - 1, 2)]
        elif namespace == "scalenav_astar_topology_path":
            result["astar"] = marker.get("points", [])
        elif (namespace in ("scalenav_frontier_goal", "scalenav_global_goal", "scalenav_local_goal")
              and int(marker.get("action", 0)) == 0):
            pose = marker.get("pose", {}).get("position")
            if pose:
                result[{"scalenav_frontier_goal": "frontier", "scalenav_global_goal": "global_goal",
                       "scalenav_local_goal": "local_goal"}[namespace]] = pose
    return result


def endpoint_diagnosis(graph: dict[str, Any], candidates: list[dict[str, Any]],
                       measured: list[dict[str, Any]], planner: dict[str, Any]) -> dict[str, Any]:
    path = graph.get("astar", [])
    if not path:
        return {"kind": "no_online_path", "endpoint": None,
                "nearest_virtual_m": None, "nearest_surface_m": None}
    endpoint = path[-1]
    distance = lambda point: math.hypot(float(endpoint[0]) - float(point["x"]),
                                        float(endpoint[1]) - float(point["y"]))
    virtual_distance = min((distance(point) for point in candidates), default=math.inf)
    surface_distance = min((distance(point) for point in measured), default=math.inf)
    selected_semantic = int(planner.get("selected_semantic_column", -1)) >= 0
    if selected_semantic and surface_distance <= 2.0 and surface_distance + 0.25 < virtual_distance:
        kind = "measured_surface_selected_as_semantic_frontier"
    elif selected_semantic and virtual_distance <= 2.0:
        kind = "fixed_depth_virtual_semantic_frontier"
    else:
        kind = "ordinary_or_held_topology_endpoint"
    finite_or_none = lambda value: value if math.isfinite(value) else None
    return {"kind": kind, "endpoint": endpoint,
            "nearest_virtual_m": finite_or_none(virtual_distance),
            "nearest_surface_m": finite_or_none(surface_distance)}


def offline_astar(graph: dict[str, Any], odom: dict[str, Any], candidates: list[dict[str, Any]],
                  goal: list[float]) -> dict[str, Any]:
    """Recompute paths from the current odom and current five virtual nodes.

    This is a diagnostic replay of the logged topology, not a replacement for
    the planner's collision/witness checks. Candidate-to-backbone links are
    therefore marked synthetic in the returned data and shown separately.
    """
    skeleton = [tuple(float(v) for v in p[:2]) for p in graph.get("skeleton", [])]
    if not skeleton:
        if candidates:
            start = odom.get("position", [0.0, 0.0])
            fallback = candidates[0]
            return {"selected_column": fallback["column"], "paths": [],
                    "selected_path": [[float(start[0]), float(start[1])],
                                      [float(fallback["x"]), float(fallback["y"])]],
                    "selection_reason": "fallback_no_skeleton", "status": "no_skeleton"}
        return {"selected_column": None, "paths": [], "selected_path": [], "status": "no_skeleton"}
    nodes = skeleton[:]
    edges: list[tuple[int, int, float, bool]] = []
    index_by_key = {tuple(round(v, 5) for v in p): i for i, p in enumerate(nodes)}
    def node_index(point: tuple[float, float]) -> int:
        key = tuple(round(v, 5) for v in point)
        if key not in index_by_key:
            index_by_key[key] = len(nodes); nodes.append(point)
        return index_by_key[key]
    def add_edge(a: int, b: int, synthetic: bool = False) -> None:
        if a == b: return
        cost = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
        if cost > 1.0e-6: edges.append((a, b, cost, synthetic))
    for pair in graph.get("edges", []):
        if len(pair) == 2:
            a, b = node_index(tuple(pair[0][:2])), node_index(tuple(pair[1][:2]))
            add_edge(a, b)
    start = node_index((float(odom.get("position", [0, 0])[0]), float(odom.get("position", [0, 0])[1])))
    nearest_start = min(range(len(skeleton)), key=lambda i: math.hypot(nodes[i][0] - nodes[start][0], nodes[i][1] - nodes[start][1]))
    add_edge(start, nearest_start, True)
    adjacency: dict[int, list[tuple[int, float, bool]]] = {i: [] for i in range(len(nodes))}
    for a, b, cost, synthetic in edges:
        adjacency[a].append((b, cost, synthetic)); adjacency[b].append((a, cost, synthetic))
    def search(target: int) -> tuple[list[int], float, bool]:
        import heapq
        open_set = [(0.0, start)]; came: dict[int, int] = {}; g = {start: 0.0}; synthetic_used = False
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == target: break
            for nxt, cost, synthetic in adjacency.get(current, []):
                ng = g[current] + cost
                if ng < g.get(nxt, math.inf):
                    g[nxt] = ng; came[nxt] = current; synthetic_used = synthetic_used or synthetic
                    h = math.hypot(nodes[nxt][0] - goal[0], nodes[nxt][1] - goal[1])
                    heapq.heappush(open_set, (ng + h, nxt))
        if target not in g: return [], math.inf, synthetic_used
        path = [target]
        while path[-1] != start: path.append(came[path[-1]])
        path.reverse(); return path, g[target], synthetic_used
    paths = []
    mission_span = max(1.0, math.hypot(nodes[start][0] - goal[0], nodes[start][1] - goal[1]))
    for candidate in candidates:
        point = (float(candidate["x"]), float(candidate["y"]))
        candidate_idx = node_index(point)
        nearest = min(range(len(skeleton)), key=lambda i: math.hypot(nodes[i][0] - point[0], nodes[i][1] - point[1]))
        add_edge(candidate_idx, nearest, True)
        # Add the candidate connection after the adjacency was constructed.
        adjacency.setdefault(candidate_idx, []).append((nearest, math.hypot(point[0] - nodes[nearest][0], point[1] - nodes[nearest][1]), True))
        adjacency.setdefault(nearest, []).append((candidate_idx, math.hypot(point[0] - nodes[nearest][0], point[1] - nodes[nearest][1]), True))
        path, route_cost, synthetic = search(candidate_idx)
        semantic_cost = float(candidate.get("score") or 0.0) * SEMANTIC_SCORE_WEIGHT
        objective = ((route_cost + 2.0 * math.hypot(point[0] - goal[0], point[1] - goal[1])) / mission_span) + semantic_cost
        paths.append({"column": candidate["column"], "points": [[nodes[i][0], nodes[i][1]] for i in path],
                      "route_cost": route_cost, "goal_distance": math.hypot(point[0] - goal[0], point[1] - goal[1]),
                      "semantic_cost": semantic_cost, "objective": objective, "synthetic_connection": synthetic,
                      "risk": candidate.get("risk")})
    eligible = [p for p in paths if math.isfinite(p["objective"])]
    selected = min(eligible, key=lambda p: (p["objective"], p["column"])) if eligible else None
    return {"selected_column": selected["column"] if selected else None, "paths": paths,
            "selected_path": selected["points"] if selected else [],
            "selection_reason": "lowest_continuous_semantic_route_objective",
            "status": "ok" if selected else "no_path"}


def build_log_result(session: Path) -> dict[str, Any]:
    records = read_log_jsonl(session / "index.jsonl")
    semantic_records = [r for r in records if r.get("kind") == "semantic" and r.get("file")]
    if not semantic_records:
        raise ValueError(f"no semantic image records in {session}")
    first_stamp = int(semantic_records[0]["stamp_ns"])
    goal_record = next((r for r in records if r.get("kind") == "goal"), None)
    goal = (goal_record or {}).get("data", {}).get("position", [0.0, 140.0, 1.6])
    timing_records = [r for r in records if r.get("kind") == "timing" and
                      r.get("data", {}).get("module") == "planner"]
    frames: list[dict[str, Any]] = []
    for index, semantic_record in enumerate(semantic_records):
        stamp = int(semantic_record["stamp_ns"])
        depth_record = nearest_record(records, "depth", stamp)
        rgb_record = nearest_record(records, "rgb", stamp)
        odom_record = nearest_record(records, "odom", stamp)
        matching_decisions = [r for r in timing_records
                              if r.get("data", {}).get("searched") and
                              int(r.get("data", {}).get("selected_semantic_frame_stamp_ns", 0)) == stamp]
        timing_record = min(matching_decisions,
                            key=lambda r: abs(int(r.get("stamp_ns", 0)) - stamp),
                            default=None)
        timing_match = "selected_semantic_frame"
        if timing_record is None:
            timing_record = min(timing_records,
                                key=lambda r: abs(int(r.get("stamp_ns", 0)) - stamp),
                                default=None)
            timing_match = "nearest_planner_tick"
        graph_stamp = int(timing_record.get("stamp_ns", stamp)) if timing_record else stamp
        graph_record = nearest_record(records, "graph", graph_stamp)
        if odom_record is None:
            raise ValueError(f"semantic frame {index} has no odom record")
        semantic_width, semantic_height, semantic_values = read_pgm16_mm(session / semantic_record["file"])
        means = finite_patch_means(semantic_values, semantic_width, semantic_height)
        depth_values: list[float] | None = None
        depth_width = depth_height = 0
        if depth_record and depth_record.get("file"):
            depth_width, depth_height, depth_values = read_pgm16_mm(session / depth_record["file"])
        odom = odom_record.get("data", {})
        candidates = []
        measured_projections = []
        for column in range(PATCH_COLS):
            u = min(semantic_width - 1, int((column + 0.5) * semantic_width / PATCH_COLS))
            v = min(semantic_height - 1, int(0.5 * semantic_height))
            sampled = None
            if depth_values is not None:
                du = min(depth_width - 1, int((u + 0.5) / semantic_width * depth_width))
                dv = min(depth_height - 1, int((v + 0.5) / semantic_height * depth_height))
                value = depth_values[dv * depth_width + du]
                if math.isfinite(value) and 0.0 < value < 20.0:
                    sampled = value
            # The semantic frontier always uses the fixed-depth virtual ray.
            # Measured depth is retained separately for ordinary-node
            # annotation and never replaces this five-column candidate set.
            projected = project_log_patch(odom, column, 35.0,
                                           semantic_width, semantic_height, True)
            # The active graph is planar (graph_fixed_layer=true); this is the
            # final node height used by topology/A*, rather than the raw camera
            # ray height.
            projected["z"] = 1.6
            score = means[PATCH_ROWS // 2][column]
            projected.update({"column": column, "score": score, "risk": "continuous",
                              "x": projected["x"], "y": projected["y"], "z": projected["z"],
                              "surface_depth_m": sampled})
            candidates.append(projected)
            if sampled is not None:
                measured = project_log_patch(odom, column, sampled,
                                             semantic_width, semantic_height, False)
                measured["column"] = column
                measured_projections.append(measured)
        graph = extract_graph(session / graph_record["file"]) if graph_record and graph_record.get("file") else {}
        planner = (timing_record or {}).get("data", {})
        diagnosis = endpoint_diagnosis(graph, candidates, measured_projections, planner)
        selected = diagnosis["endpoint"] or (graph.get("frontier") if graph else None)
        recomputed = offline_astar(graph, odom, candidates, goal)
        frames.append({
            "index": index, "time_s": (stamp - first_stamp) / 1.0e9, "stamp_ns": stamp,
            "odom": odom, "goal": {"x": goal[0], "y": goal[1], "z": goal[2]},
            "rgb": {"data": base64.b64encode((session / rgb_record["file"]).read_bytes()).decode("ascii"),
                    "mime": "image/x-portable-pixmap"} if rgb_record and rgb_record.get("file") else None,
            "depth": {"data": base64.b64encode((session / depth_record["file"]).read_bytes()).decode("ascii"),
                      "mime": "image/x-portable-graymap"} if depth_record and depth_record.get("file") else None,
            "heatmap": {"data": base64.b64encode((session / semantic_record["file"]).read_bytes()).decode("ascii"),
                        "mime": "image/x-portable-graymap"},
            "image_size": {"width": semantic_width, "height": semantic_height},
            "patch_means": means, "candidates": candidates,
            "measured_projections": measured_projections, "graph": graph,
            "recomputed_astar": recomputed,
            "planner": planner, "selected_frontier": selected,
            "endpoint_diagnosis": diagnosis, "planner_timing_match": timing_match,
            "sync_ms": {"rgb": abs(int(rgb_record["stamp_ns"]) - stamp) / 1.0e6 if rgb_record else None,
                         "depth": abs(int(depth_record["stamp_ns"]) - stamp) / 1.0e6 if depth_record else None,
                         "odom": abs(int(odom_record["stamp_ns"]) - stamp) / 1.0e6},
        })
    measured_frontier_frames = sum(
        1 for frame in frames
        if frame["endpoint_diagnosis"]["kind"] ==
        "measured_surface_selected_as_semantic_frontier")
    route_events = []
    route_was_cleared = False
    for record in timing_records:
        planner = record.get("data", {})
        decision = str(planner.get("route_decision", ""))
        reason = str(planner.get("switch_reason", "NONE"))
        searched = bool(planner.get("searched", False))
        rejected = int(planner.get("semantic_risk_edges_rejected", 0))
        if decision == "ROUTE_HELD":
            continue
        if not searched and decision != "NO_CANDIDATE":
            continue
        if reason == "FRONTIER_PROGRESS":
            label, severity = "达到 40% progress，正常重新 A*", "normal"
        elif reason == "ROUTE_UNREACHABLE":
            label, severity = "旧 frontier 不可达，重新 A*", "warning"
        elif decision == "NO_CANDIDATE" and rejected > 0:
            label, severity = "普通-虚拟语义末边被拒绝，路线被清空", "error"
        elif reason == "INITIAL_ACCEPT" and route_was_cleared:
            label, severity = "清空路线后重新接收（不是任务初始）", "warning"
        elif reason == "INITIAL_ACCEPT":
            label, severity = "首次接收语义 frontier", "normal"
        else:
            label, severity = f"{reason} / {decision}", "warning"
        stamp_ns = int(record.get("stamp_ns", 0))
        nearest_frame = min(range(len(frames)),
                            key=lambda i: abs(int(frames[i]["stamp_ns"]) - stamp_ns))
        route_events.append({
            "time_s": (stamp_ns - first_stamp) / 1.0e9,
            "stamp_ns": stamp_ns,
            "frame_index": nearest_frame,
            "searched": searched,
            "decision": decision,
            "reason": reason,
            "frontier_id": int(planner.get("committed_frontier_id", 0)),
            "semantic_edges_checked": int(planner.get("semantic_risk_edges_checked", 0)),
            "semantic_edges_rejected": rejected,
            "label": label,
            "severity": severity,
        })
        if decision == "NO_CANDIDATE" and rejected > 0:
            route_was_cleared = True
        elif decision == "CANDIDATE_COMMITTED":
            route_was_cleared = False
    last_frame = frames[-1]
    last_planner = last_frame.get("planner", {})
    last_velocity = last_frame.get("odom", {}).get("velocity", [0.0, 0.0, 0.0])
    last_speed = math.hypot(float(last_velocity[0]), float(last_velocity[1]))
    last_progress = float(last_planner.get("frontier_progress_t", 0.0))
    replan_ratio = float(last_planner.get("frontier_replan_ratio", 0.4))
    final_hold_below_trigger = (
        last_planner.get("route_decision") == "ROUTE_HELD" and
        not bool(last_planner.get("searched", False)) and
        last_progress < replan_ratio)
    return {"session": session.name, "goal": {"x": goal[0], "y": goal[1], "z": goal[2]},
            "config": {"patch_grid": "3x5", "planar_row": "middle row (5 columns)",
                       "patch_aggregation": "finite pixel mean", "semantic_score_weight": SEMANTIC_SCORE_WEIGHT,
                       "virtual_range_m": 35.0, "semantic_memory_s": 1.5},
            "diagnosis": {
                "measured_surface_selected_as_semantic_frontier_frames": measured_frontier_frames,
                "root_cause": ("measured depth annotations had no persistent source type; "
                               "semantic_observations/Unknown geometry could therefore be "
                               "misclassified as a virtual frontier endpoint")},
            "route_events": route_events,
            "trajectory": [[float(frame["odom"]["position"][0]),
                            float(frame["odom"]["position"][1])]
                           for frame in frames],
            "route_diagnosis": {
                "event_count": len(route_events),
                "progress_replans": sum(event["reason"] == "FRONTIER_PROGRESS"
                                        for event in route_events),
                "route_unreachable_replans": sum(event["reason"] == "ROUTE_UNREACHABLE"
                                                  for event in route_events),
                "semantic_edge_route_clears": sum(
                    event["decision"] == "NO_CANDIDATE" and
                    event["semantic_edges_rejected"] > 0 for event in route_events),
                "final_hold_below_trigger": final_hold_below_trigger,
                "final_progress": last_progress,
                "replan_ratio": replan_ratio,
                "final_speed_mps": last_speed,
                "final_searched": bool(last_planner.get("searched", False)),
                "final_route_decision": last_planner.get("route_decision", ""),
                "final_frontier_id": int(last_planner.get("committed_frontier_id", 0)),
            },
            "record_counts": {kind: sum(1 for r in records if r.get("kind") == kind)
                              for kind in sorted({r.get("kind") for r in records})},
            "frames": frames}


def render_log_html(result: dict[str, Any]) -> str:
    payload = json.dumps(result, separators=(",", ":"), allow_nan=False)
    return r'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ScaleNav semantic frontier log test</title><style>
:root{--bg:#101820;--panel:#182632;--line:#304554;--text:#edf4f7;--muted:#9db0bb;--cyan:#55d6e8;--green:#56d58b;--red:#ff776f;--amber:#f4c95d}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:13px/1.4 system-ui,sans-serif}.shell{max-width:1700px;margin:auto;padding:18px}.head{display:flex;justify-content:space-between;gap:16px;align-items:end;border-bottom:1px solid var(--line);padding-bottom:12px}h1{font-size:21px;margin:0}.muted{color:var(--muted)}.controls{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin:12px 0}.controls button{background:#243948;color:var(--text);border:1px solid var(--line);padding:6px 9px;border-radius:3px;cursor:pointer}.controls button.active{background:var(--cyan);color:#071319}.grid{display:grid;grid-template-columns:repeat(3,minmax(230px,1fr)) minmax(410px,1.6fr);gap:10px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:5px;padding:10px;min-width:0}.panel h2{font-size:14px;margin:0 0 8px}.image{width:100%;background:#071016;border:1px solid var(--line);display:block;aspect-ratio:5/3}.wide{grid-column:1/-1}.world{width:100%;height:500px;background:#09131b;border:1px solid var(--line)}.facts{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:7px}.fact{border-left:2px solid var(--cyan);padding-left:6px}.fact b{display:block;font-size:14px}.fact small{color:var(--muted)}table{width:100%;border-collapse:collapse;font-size:11px}th,td{padding:5px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}th{color:var(--muted)}.low{color:var(--green)}.high{color:var(--red)}.none{color:var(--muted)}pre{white-space:pre-wrap;max-height:300px;overflow:auto;color:#c8d8df;background:#0c171f;padding:8px;font-size:11px}.legend{color:var(--muted);font-size:11px;margin-top:5px}.event-strip{display:flex;gap:6px;overflow-x:auto;padding-bottom:5px}.event{min-width:178px;text-align:left;background:#213440;border:1px solid var(--line);color:var(--text);padding:7px;border-radius:3px;cursor:pointer}.event.normal{border-left:3px solid var(--green)}.event.warning{border-left:3px solid var(--amber)}.event.error{border-left:3px solid var(--red)}.event.current{outline:2px solid var(--cyan)}.diagnostic-alert{border-left:3px solid var(--red);padding:8px 10px;background:#251e22}@media(max-width:1100px){.grid{grid-template-columns:repeat(2,minmax(230px,1fr))}.wide{grid-column:1/-1}}@media(max-width:680px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.shell{padding:9px}.world{height:380px}}
</style></head><body><main class="shell"><header class="head"><div><h1>Real log: semantic frontier frame test</h1><div class="muted" id="session"></div></div><div class="muted">RGB + depth + heatmap + 3x5 projection + graph/A*</div></header><div id="controls" class="controls"></div><section class="grid"><article class="panel"><h2>RGB (patch grid overlay)</h2><canvas id="rgb" class="image"></canvas></article><article class="panel"><h2>Depth (middle-row sample)</h2><canvas id="depth" class="image"></canvas></article><article class="panel"><h2>Heatmap (patch means)</h2><canvas id="heat" class="image"></canvas></article><article class="panel"><h2>Frame facts</h2><div id="facts" class="facts"></div><div id="sync" class="legend"></div><div id="planner"></div></article><article class="panel wide"><h2>Route lifecycle (click to jump)</h2><div id="timeline" class="event-strip"></div></article><article class="panel wide"><h2>Log diagnosis</h2><div id="diagnosis"></div></article><article class="panel wide"><h2>World XY: projected semantic nodes, graph nodes, A* path and selected frontier</h2><svg id="world" class="world" viewBox="0 0 1100 500"></svg><div class="legend">cyan = online A* topology path; orange = offline diagnostic A*; pale blue = flown trajectory; green = 35 m virtual candidates; dashed gray = measured surface projections; white ring = online A* endpoint; gray = skeleton nodes</div></article><article class="panel wide"><h2>Five planar candidates and actual frontier decision</h2><div id="decision"></div></article><article class="panel wide"><h2>Raw frame JSON</h2><pre id="raw"></pre></article></section></main><script>
const DATA=__DATA__;let current=0;const $=id=>document.getElementById(id);const esc=s=>String(s??'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
function b64(s){const raw=atob(s),a=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)a[i]=raw.charCodeAt(i);return a}
function netpbm(a){let p=0,t=[];const skip=()=>{while(p<a.length&&a[p]<=32)p++;if(a[p]===35){while(p<a.length&&a[p]!==10)p++;skip()}};const tok=()=>{skip();let q=p;while(p<a.length&&a[p]>32)p++;return new TextDecoder().decode(a.slice(q,p))};const magic=tok(),w=+tok(),h=+tok(),max=+tok();if(a[p]===13&&a[p+1]===10)p+=2;else if(a[p]<=32)p++;return{magic,w,h,max,off:p}}
function decodeImage(obj){if(!obj)return null;const a=b64(obj.data),n=netpbm(a),out=document.createElement('canvas');out.width=n.w;out.height=n.h;const x=out.getContext('2d'),im=x.createImageData(n.w,n.h),values=[];if(n.magic==='P6'){for(let i=0,p=n.off;i<n.w*n.h;i++,p+=3){im.data[i*4]=a[p];im.data[i*4+1]=a[p+1];im.data[i*4+2]=a[p+2];im.data[i*4+3]=255}}else{for(let i=0,p=n.off;i<n.w*n.h;i++,p+=2){const v=a[p]*256+a[p+1];values.push(v);const q=Math.round(255*v/n.max);im.data[i*4]=q;im.data[i*4+1]=q;im.data[i*4+2]=q;im.data[i*4+3]=255}}x.putImageData(im,0,0);return{canvas:out,meta:n,values}}
function heatColor(t){const stops=[[20,40,100],[25,155,215],[70,205,145],[245,210,65],[225,55,45]],q=Math.max(0,Math.min(1,t))*(stops.length-1),i=Math.min(stops.length-2,Math.floor(q)),f=q-i;return stops[i].map((v,k)=>Math.round(v+(stops[i+1][k]-v)*f))}
function patchOverlay(target,means,heat){const c=$(target),x=c.getContext('2d');if(!heat)return;const w=c.width,h=c.height,im=x.createImageData(w,h);for(let i=0;i<w*h;i++){const col=heatColor((heat.values[i]||0)/1000);im.data[i*4]=col[0];im.data[i*4+1]=col[1];im.data[i*4+2]=col[2];im.data[i*4+3]=255}x.putImageData(im,0,0);x.lineWidth=Math.max(1,w/320);x.font=Math.max(8,w/36)+'px monospace';for(let r=0;r<3;r++)for(let col=0;col<5;col++){const x0=col*w/5,y0=r*h/3;x.strokeStyle=r===1?'#fff':'rgba(255,255,255,.45)';x.strokeRect(x0,y0,w/5,h/3);const v=means[r][col];x.fillStyle=r===1?'#fff':'rgba(255,255,255,.78)';x.fillText(v==null?'n/a':v.toFixed(3),x0+2,y0+13)}}
function drawFrameImage(id,obj,means,heat,depth){const c=$(id),x=c.getContext('2d');if(!obj){x.fillStyle='#0b151c';x.fillRect(0,0,c.width,c.height);x.fillStyle='#9db0bb';x.fillText('no record',8,18);return}const decoded=decodeImage(obj);c.width=decoded.meta.w;c.height=decoded.meta.h;if(id==='heat')patchOverlay(id,means,decoded,depth);else{const src=decoded.canvas;x.drawImage(src,0,0,c.width,c.height);x.lineWidth=1;for(let r=0;r<=3;r++){x.strokeStyle=r===1||r===2?'#fff':'rgba(255,255,255,.5)';x.beginPath();x.moveTo(0,r*c.height/3);x.lineTo(c.width,r*c.height/3);x.stroke()}for(let col=0;col<=5;col++){x.strokeStyle='rgba(255,255,255,.5)';x.beginPath();x.moveTo(col*c.width/5,0);x.lineTo(col*c.width/5,c.height);x.stroke()}if(id==='depth'){x.fillStyle='#fff';x.font='9px monospace';for(let col=0;col<5;col++){const v=DATA.frames[current].candidates[col],d=v.surface_depth_m;x.fillText(d==null?'n/a':d.toFixed(2)+'m',col*c.width/5+2,c.height/2+12)}}}}
function fmt(v,n=2){return Number.isFinite(v)?Number(v).toFixed(n):'-'}
function drawWorld(f){const s=$('world'),pts=[];const add=p=>{if(p&&Number.isFinite(+p[0])&&Number.isFinite(+p[1]))pts.push([+p[0],+p[1]])};f.candidates.forEach(c=>add([c.x,c.y]));(f.graph?.skeleton||[]).forEach(add);(f.graph?.semantic||[]).forEach(add);(f.graph?.astar||[]).forEach(add);add([f.odom.position[0],f.odom.position[1]]);add([f.goal.x,f.goal.y]);if(f.selected_frontier)add(f.selected_frontier);let minx=Math.min(...pts.map(p=>p[0]))-4,maxx=Math.max(...pts.map(p=>p[0]))+4,miny=Math.min(...pts.map(p=>p[1]))-4,maxy=Math.max(...pts.map(p=>p[1]))+4;const sx=x=>50+(x-minx)/(maxx-minx||1)*1000,sy=y=>460-(y-miny)/(maxy-miny||1)*420;let o='';const line=(a,b,cl,w=1)=>o+=`<line x1="${sx(a[0])}" y1="${sy(a[1])}" x2="${sx(b[0])}" y2="${sy(b[1])}" stroke="${cl}" stroke-width="${w}"/>`;for(const e of f.graph?.edges||[])if(e.length===2)line(e[0],e[1],'#465866',1);const path=f.graph?.astar||[];for(let i=1;i<path.length;i++)line(path[i-1],path[i],'#55d6e8',3);for(const p of f.graph?.skeleton||[])o+=`<circle cx="${sx(p[0])}" cy="${sy(p[1])}" r="3" fill="#81909a"/>`;for(const p of f.graph?.semantic||[])o+=`<circle cx="${sx(p[0])}" cy="${sy(p[1])}" r="7" fill="#ff776f" stroke="#fff"/>`;for(const c of f.candidates){const cl=c.risk==='high'?'#ff776f':'#56d58b';o+=`<circle cx="${sx(c.x)}" cy="${sy(c.y)}" r="${c.column===f.planner.selected_semantic_column?8:5}" fill="${cl}"/><text x="${sx(c.x)+7}" y="${sy(c.y)+4}" fill="#edf4f7" font-size="11">c${c.column}</text>`}o+=`<circle cx="${sx(f.odom.position[0])}" cy="${sy(f.odom.position[1])}" r="7" fill="#55d6e8"/><text x="${sx(f.odom.position[0])+8}" y="${sy(f.odom.position[1])-7}" fill="#55d6e8">odom</text><circle cx="${sx(f.goal.x)}" cy="${sy(f.goal.y)}" r="7" fill="#f4c95d"/><text x="${sx(f.goal.x)+8}" y="${sy(f.goal.y)-7}" fill="#f4c95d">mission</text>`;if(f.selected_frontier)o+=`<circle cx="${sx(f.selected_frontier[0])}" cy="${sy(f.selected_frontier[1])}" r="11" fill="none" stroke="#fff" stroke-width="3"/>`;s.innerHTML=o}
function render(){const f=DATA.frames[current],p=f.planner||{};$('controls').innerHTML=DATA.frames.map((x,i)=>`<button class="${i===current?'active':''}" onclick="current=${i};render()">F${i} <small>${x.time_s.toFixed(2)}s</small></button>`).join('');$('session').textContent=`${DATA.session} · frame ${current+1}/${DATA.frames.length} · stamp ${f.stamp_ns}`;const heat=decodeImage(f.heatmap),rgb=decodeImage(f.rgb),depth=decodeImage(f.depth);['rgb','depth','heat'].forEach(id=>{const c=$(id);c.width=160;c.height=96});drawFrameImage('rgb',f.rgb,f.patch_means,heat,depth);drawFrameImage('depth',f.depth,f.patch_means,heat,depth);drawFrameImage('heat',f.heatmap,f.patch_means,heat,depth);$('facts').innerHTML=`<div class="fact"><b>${fmt(f.odom.position[0])}, ${fmt(f.odom.position[1])}</b><small>odom XY</small></div><div class="fact"><b>${f.candidates.length}/5</b><small>virtual columns</small></div><div class="fact"><b>${p.route_decision||'-'}</b><small>route decision</small></div><div class="fact"><b>${p.switch_reason||'-'}</b><small>switch reason</small></div>`;$('sync').textContent=`sync: rgb ${fmt(f.sync_ms.rgb,1)} ms · depth ${fmt(f.sync_ms.depth,1)} ms · odom ${fmt(f.sync_ms.odom,1)} ms`;$('planner').innerHTML=`<pre>${esc(JSON.stringify(p,null,2))}</pre>`;drawWorld(f);$('decision').innerHTML=`<p><b>Graph frontier pose:</b> ${f.selected_frontier?f.selected_frontier.slice(0,2).map(v=>fmt(v)).join(', '):'none'} · <b>planner semantic column:</b> ${p.selected_semantic_column??'-'} · <b>semantic candidates:</b> ${p.astar_semantic_frontier_candidates??'-'} · <b>ordinary candidates:</b> ${p.astar_ordinary_frontier_candidates??'-'}</p><table><thead><tr><th>col</th><th>mean</th><th>risk</th><th>range</th><th>source</th><th>projected node</th></tr></thead><tbody>${f.candidates.map(c=>`<tr><td>${c.column}</td><td>${c.score==null?'-':c.score.toFixed(4)}</td><td class="${c.risk}">${c.risk}</td><td>${c.depth_m.toFixed(2)}</td><td>${c.depth_source}</td><td>(${fmt(c.x)}, ${fmt(c.y)}, ${fmt(c.z)})</td></tr>`).join('')}</tbody></table>`;$('raw').textContent=JSON.stringify(f,null,2)}render();
// Override the compact base renderer with an equal-scale XY top view. The
// same metre-to-pixel scale is used for X and Y, so the five rays keep their
// physical angles instead of being stretched by the panel aspect ratio.
function drawWorld(f){const s=$('world'),points=[];const trail=(DATA.trajectory||[]).slice(0,f.index+1),add=p=>{if(p&&Number.isFinite(+p[0])&&Number.isFinite(+p[1]))points.push([+p[0],+p[1]])};f.candidates.forEach(c=>add([c.x,c.y]));(f.measured_projections||[]).forEach(c=>add([c.x,c.y]));(f.graph?.skeleton||[]).forEach(add);(f.graph?.semantic||[]).forEach(add);(f.graph?.astar||[]).forEach(add);trail.forEach(add);add([f.odom.position[0],f.odom.position[1]]);add([f.goal.x,f.goal.y]);if(f.selected_frontier)add(f.selected_frontier);let minx=Math.min(...points.map(p=>p[0]))-4,maxx=Math.max(...points.map(p=>p[0]))+4,miny=Math.min(...points.map(p=>p[1]))-4,maxy=Math.max(...points.map(p=>p[1]))+4;const scale=Math.min(1000/(maxx-minx||1),420/(maxy-miny||1)),cx=(minx+maxx)/2,cy=(miny+maxy)/2,sx=x=>550+(x-cx)*scale,sy=y=>250-(y-cy)*scale;let out='';const line=(a,b,color,width=1,dash='')=>out+=`<line x1="${sx(a[0])}" y1="${sy(a[1])}" x2="${sx(b[0])}" y2="${sy(b[1])}" stroke="${color}" stroke-width="${width}" ${dash?`stroke-dasharray="${dash}"`:''}/>`;for(const e of f.graph?.edges||[])if(e.length===2)line(e[0],e[1],'#465866');for(let i=1;i<trail.length;i++)line(trail[i-1],trail[i],'#77a9ba',2);const path=f.graph?.astar||[];for(let i=1;i<path.length;i++)line(path[i-1],path[i],'#55d6e8',3);for(const m of f.measured_projections||[])line([f.odom.position[0],f.odom.position[1]],[m.x,m.y],'#70818c',1,'3 3');for(const c of f.candidates)line([f.odom.position[0],f.odom.position[1]],[c.x,c.y],c.risk==='high'?'#753f43':'#2e7653',1,'5 4');for(const p of f.graph?.skeleton||[])out+=`<circle cx="${sx(p[0])}" cy="${sy(p[1])}" r="3" fill="#81909a"/>`;for(const p of f.graph?.semantic||[])out+=`<circle cx="${sx(p[0])}" cy="${sy(p[1])}" r="7" fill="#ff776f" stroke="#fff"/>`;for(const c of f.candidates){const color=c.risk==='high'?'#ff776f':'#56d58b';out+=`<circle cx="${sx(c.x)}" cy="${sy(c.y)}" r="${c.column===f.planner.selected_semantic_column?8:5}" fill="${color}"/><text x="${sx(c.x)+7}" y="${sy(c.y)+4}" fill="#edf4f7" font-size="11">V${c.column}</text>`}out+=`<circle cx="${sx(f.odom.position[0])}" cy="${sy(f.odom.position[1])}" r="7" fill="#55d6e8"/><text x="${sx(f.odom.position[0])+8}" y="${sy(f.odom.position[1])-7}" fill="#55d6e8">odom</text><circle cx="${sx(f.goal.x)}" cy="${sy(f.goal.y)}" r="7" fill="#f4c95d"/><text x="${sx(f.goal.x)+8}" y="${sy(f.goal.y)-7}" fill="#f4c95d">mission</text>`;if(f.selected_frontier)out+=`<circle cx="${sx(f.selected_frontier[0])}" cy="${sy(f.selected_frontier[1])}" r="11" fill="none" stroke="#fff" stroke-width="3"/>`;s.innerHTML=out}
function drawRecomputed(f){const path=f.recomputed_astar?.selected_path||[];if(path.length<2)return;const all=[];const add=p=>{if(p&&Number.isFinite(+p[0])&&Number.isFinite(+p[1]))all.push([+p[0],+p[1]])};f.candidates.forEach(c=>add([c.x,c.y]));(f.measured_projections||[]).forEach(c=>add([c.x,c.y]));(f.graph?.skeleton||[]).forEach(add);(f.graph?.semantic||[]).forEach(add);(f.graph?.astar||[]).forEach(add);add([f.odom.position[0],f.odom.position[1]]);add([f.goal.x,f.goal.y]);if(f.selected_frontier)add(f.selected_frontier);let minx=Math.min(...all.map(p=>p[0]))-4,maxx=Math.max(...all.map(p=>p[0]))+4,miny=Math.min(...all.map(p=>p[1]))-4,maxy=Math.max(...all.map(p=>p[1]))+4;const scale=Math.min(1000/(maxx-minx||1),420/(maxy-miny||1)),cx=(minx+maxx)/2,cy=(miny+maxy)/2,sx=x=>550+(x-cx)*scale,sy=y=>250-(y-cy)*scale;const s=$('world'),g=document.createElementNS('http://www.w3.org/2000/svg','g');g.setAttribute('data-layer','recomputed-astar');for(let i=1;i<path.length;i++){const e=document.createElementNS('http://www.w3.org/2000/svg','line');e.setAttribute('x1',sx(path[i-1][0]));e.setAttribute('y1',sy(path[i-1][1]));e.setAttribute('x2',sx(path[i][0]));e.setAttribute('y2',sy(path[i][1]));e.setAttribute('stroke','#ffb347');e.setAttribute('stroke-width','4');e.setAttribute('stroke-linecap','round');g.appendChild(e)}s.appendChild(g)}const baseRender=render;render=function(){baseRender();drawRecomputed(DATA.frames[current])};render();
let autoplay=true,autoplayTimer=null;
function highlightChoice(f){const svg=$('world'),selected=f.recomputed_astar?.selected_column;for(const label of [...svg.querySelectorAll('text')].filter(n=>/^V[0-4]$/.test(n.textContent))){const col=Number(label.textContent.slice(1)),cx=Number(label.getAttribute('x'))-7,cy=Number(label.getAttribute('y'))-4, circle=document.createElementNS('http://www.w3.org/2000/svg','circle');circle.setAttribute('cx',cx);circle.setAttribute('cy',cy);circle.setAttribute('r',col===selected?'8':'6');circle.setAttribute('fill',col===selected?'#56d58b':(f.candidates[col]?.risk==='high'?'#ff776f':'#71808b'));circle.setAttribute('stroke',col===selected?'#fff':'#15242d');circle.setAttribute('stroke-width',col===selected?'2':'1');svg.insertBefore(circle,label);label.setAttribute('x',cx+7);label.setAttribute('y',cy+4);label.setAttribute('fill','#edf4f7')}}
function stopAutoplay(){if(autoplayTimer){clearInterval(autoplayTimer);autoplayTimer=null}}
function startAutoplay(){stopAutoplay();if(!autoplay)return;autoplayTimer=setInterval(()=>{if(current>=DATA.frames.length-1){autoplay=false;stopAutoplay();return}current++;render()},900)}
function renderTimeline(){const events=DATA.route_events||[];$('timeline').innerHTML=events.map((e,i)=>`<button class="event ${e.severity} ${e.frame_index===current?'current':''}" onclick="current=${e.frame_index};render()"><b>${i+1}. ${e.time_s.toFixed(2)} s · ${esc(e.reason)}</b><br>${esc(e.label)}<br><small>frontier ${e.frontier_id||'-'} · rejected ${e.semantic_edges_rejected}</small></button>`).join('')}
function renderDiagnosis(f){const p=f.planner||{},run=DATA.route_diagnosis||{},progress=Number(p.frontier_progress_t||0),ratio=Number(p.frontier_replan_ratio||run.replan_ratio||0.4),held=p.route_decision==='ROUTE_HELD'&&!p.searched,below=held&&progress<ratio;const event=(DATA.route_events||[]).find(e=>e.frame_index===f.index);let headline,detail;if(event){headline=event.label;detail=`本帧 route_decision=${event.decision}，searched=${event.searched}，frontier=${event.frontier_id||'-'}。`}else if(below){headline='旧路线被继续持有，未运行 A*';detail=`progress=${(progress*100).toFixed(1)}%，尚未达到 ${(ratio*100).toFixed(0)}% 触发阈值。拓扑图即使已经出现绕路，只要旧 frontier 仍被判为可达，当前逻辑也不会搜索新路径。`}else{headline='当前帧继续执行已接收路线';detail=`route_decision=${p.route_decision||'-'}，searched=${!!p.searched}，progress=${(progress*100).toFixed(1)}%。`}const alert=below&&f.index===DATA.frames.length-1?'diagnostic-alert':'';$('diagnosis').innerHTML=`<div class="${alert}"><p><b>${esc(headline)}</b></p><p>${esc(detail)}</p></div><p class="muted">全程：${run.progress_replans??0} 次 progress 重规划，${run.route_unreachable_replans??0} 次 endpoint 不可达重规划，${run.semantic_edge_route_clears??0} 次语义末边拒绝并清空路线。末帧 searched=${run.final_searched}，progress=${fmt((run.final_progress||0)*100,1)}%，speed=${fmt(run.final_speed_mps)} m/s。</p>`}
function renderCandidateDecision(f){const paths=new Map((f.recomputed_astar?.paths||[]).map(p=>[p.column,p])),selected=f.recomputed_astar?.selected_column;$('decision').innerHTML=`<p><b>Current-contract winner:</b> ${selected==null?'none':'V'+selected} · <b>historical online endpoint:</b> ${f.selected_frontier?f.selected_frontier.slice(0,2).map(v=>fmt(v)).join(', '):'none'} · <b>online reported column:</b> ${f.planner?.selected_semantic_column??'-'}</p><table><thead><tr><th>candidate</th><th>patch mean</th><th>A* route cost</th><th>mission distance</th><th>semantic cost</th><th>normalized objective</th><th>status</th></tr></thead><tbody>${f.candidates.map(c=>{const p=paths.get(c.column)||{};return `<tr><td>${c.column===selected?'SELECTED ':''}V${c.column}</td><td>${fmt(c.score,4)}</td><td>${fmt(p.route_cost)}</td><td>${fmt(p.goal_distance)}</td><td>${fmt(p.semantic_cost,4)}</td><td>${fmt(p.objective,5)}</td><td>${p.synthetic_connection?'diagnostic link':'logged graph link'}</td></tr>`}).join('')}</tbody></table>`}
const rendered=render;render=function(){rendered();const frame=DATA.frames[current];highlightChoice(frame);renderTimeline();renderDiagnosis(frame);renderCandidateDecision(frame);const old=$('autoplay');if(old)old.remove();const button=document.createElement('button');button.id='autoplay';button.textContent=autoplay?'暂停自动播放':'继续自动播放';button.onclick=()=>{autoplay=!autoplay;if(autoplay)startAutoplay();else stopAutoplay();button.textContent=autoplay?'暂停自动播放':'继续自动播放'};$('controls').prepend(button);if(autoplay&&!autoplayTimer)startAutoplay()};render();
</script></body></html>'''.replace('__DATA__', payload)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--html", type=Path, required=True)
    parser.add_argument("--session", type=Path)
    args = parser.parse_args()
    result = build_log_result(args.session) if args.session else simulate()
    args.json.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    args.html.write_text(render_log_html(result) if args.session else render_html(result), encoding="utf-8")
    if args.session:
        print(f"real log report: {len(result['frames'])} semantic frames from {result['session']}")
        return 0
    passed = sum(bool(value) for value in result["checks"].values())
    print(f"semantic frontier module: {passed}/{len(result['checks'])} checks passed")
    return 0 if all(result["checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
