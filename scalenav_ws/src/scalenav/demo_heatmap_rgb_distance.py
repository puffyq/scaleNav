#!/usr/bin/env python3
"""Offline RGB-heatmap distance feasibility demo.

The estimator uses only the logged RGB-derived semantic heatmap and odometry.
Logged point clouds are used as a geometric reference, never as an estimator
input.  For every middle-row heatmap column, particles are sampled along the
current RGB observation ray and weighted by their distance to rays from recent
poses (WildOS-style multi-view localization).
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import random
import re
import struct
from dataclasses import dataclass
from pathlib import Path


COLS = 5
ROWS = 3


def parse_jsonl(path: Path) -> list[dict]:
    records = []
    nonfinite = re.compile(r"(?<![A-Za-z0-9_])(?:-?inf(?:inity)?|nan)(?![A-Za-z0-9_])", re.I)
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            records.append(json.loads(line, parse_constant=lambda _: None))
        except json.JSONDecodeError as exc:
            try:
                records.append(json.loads(nonfinite.sub("null", line)))
            except json.JSONDecodeError:
                raise ValueError(f"invalid JSON at {path}:{line_no}: {exc}") from exc
    return records


def read_pgm16_mm(path: Path) -> tuple[int, int, list[float]]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    offset = 0
    while len(tokens) < 4:
        while offset < len(data) and data[offset] <= 32:
            offset += 1
        if offset < len(data) and data[offset] == ord("#"):
            end = data.find(b"\n", offset)
            offset = len(data) if end < 0 else end + 1
            continue
        begin = offset
        while offset < len(data) and data[offset] > 32:
            offset += 1
        tokens.append(data[begin:offset])
    if tokens[0] != b"P5" or int(tokens[3]) != 65535:
        raise ValueError(f"unsupported PGM: {path}")
    width, height = int(tokens[1]), int(tokens[2])
    if offset < len(data) and data[offset] <= 32:
        offset += 1
    count = width * height
    raw = data[offset:offset + count * 2]
    if len(raw) != count * 2:
        raise ValueError(f"truncated PGM: {path}")
    return width, height, [x / 1000.0 for x in struct.unpack(f">{count}H", raw)]


def read_ppm_rgb(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    offset = 0
    while len(tokens) < 4:
        while offset < len(data) and data[offset] <= 32:
            offset += 1
        if offset < len(data) and data[offset] == ord("#"):
            end = data.find(b"\n", offset)
            offset = len(data) if end < 0 else end + 1
            continue
        begin = offset
        while offset < len(data) and data[offset] > 32:
            offset += 1
        tokens.append(data[begin:offset])
    if tokens[0] != b"P6" or int(tokens[3]) != 255:
        raise ValueError(f"unsupported PPM: {path}")
    if offset < len(data) and data[offset] <= 32:
        offset += 1
    width, height = int(tokens[1]), int(tokens[2])
    rgb = data[offset:offset + width * height * 3]
    if len(rgb) != width * height * 3:
        raise ValueError(f"truncated PPM: {path}")
    return width, height, rgb


def read_pcd(path: Path) -> list[tuple[float, float, float]]:
    lines = path.read_text(encoding="ascii").splitlines()
    try:
        start = next(i for i, line in enumerate(lines) if line.startswith("DATA"))
    except StopIteration as exc:
        raise ValueError(f"PCD has no DATA section: {path}") from exc
    if not lines[start].lower().endswith("ascii"):
        raise ValueError(f"demo requires ASCII PCD: {path}")
    points = []
    for line in lines[start + 1:]:
        fields = line.split()
        if len(fields) >= 3:
            try:
                point = tuple(float(fields[i]) for i in range(3))
            except ValueError:
                continue
            if all(math.isfinite(x) for x in point):
                points.append(point)
    return points


def qrotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = q
    vx, vy, vz = v
    # Quaternion-vector rotation, avoiding a dependency on numpy/scipy.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def scale(a, s):
    return tuple(x * s for x in a)


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(a):
    return math.sqrt(max(0.0, dot(a, a)))


@dataclass
class Ray:
    origin: tuple[float, float, float]
    direction: tuple[float, float, float]
    column: int
    score: float
    stamp_ns: int


def patch_means(values: list[float], width: int, height: int) -> list[float | None]:
    result: list[float | None] = []
    middle = ROWS // 2
    v0, v1 = middle * height // ROWS, (middle + 1) * height // ROWS
    for col in range(COLS):
        u0, u1 = col * width // COLS, (col + 1) * width // COLS
        patch = [values[v * width + u] for v in range(v0, v1) for u in range(u0, u1)
                 if math.isfinite(values[v * width + u])]
        result.append(sum(patch) / len(patch) if patch else None)
    return result


def ray_for_column(position, orientation, column: int, width: int = 160, height: int = 96):
    # Matches the current ScaleNav virtual projection: 90 x 60 deg FOV and
    # camera translation (0.5, 0, -0.1) in FLU body coordinates.
    u = ((column + 0.5) / COLS)
    v = 0.5
    tx = math.tan(math.radians(90.0) * 0.5)
    ty = math.tan(math.radians(60.0) * 0.5)
    camera_translation = (0.5, 0.0, -0.1)
    body_direction = (1.0, -(2.0 * u - 1.0) * tx, -(2.0 * v - 1.0) * ty)
    length = norm(body_direction)
    body_direction = scale(body_direction, 1.0 / length)
    return add(position, qrotate(orientation, camera_translation)), qrotate(orientation, body_direction)


def distance_to_ray(point, ray: Ray) -> tuple[float, float]:
    delta = sub(point, ray.origin)
    along = dot(delta, ray.direction)
    if along <= 0.0:
        return math.inf, along
    perpendicular = norm(sub(delta, scale(ray.direction, along)))
    return perpendicular, along


def estimate_distance(current: Ray, history: list[Ray], particles: int, rng: random.Random,
                      min_depth: float, max_depth: float) -> tuple[float | None, float, list[tuple[float, float, float]]]:
    usable = [ray for ray in history if ray.column == current.column]
    if len({ray.stamp_ns for ray in usable}) < 2:
        return None, 0.0, []
    hypotheses = []
    weights = []
    for _ in range(particles):
        depth = rng.uniform(min_depth, max_depth)
        point = add(current.origin, scale(current.direction, depth))
        weight = 0.0
        for ray in usable:
            perpendicular, along = distance_to_ray(point, ray)
            if math.isfinite(perpendicular):
                weight += max(0.05, ray.score) / (perpendicular + 0.25)
        hypotheses.append(point)
        weights.append(weight)
    total = sum(weights)
    if total <= 1.0e-9:
        return None, 0.0, []
    point = tuple(sum(hypotheses[i][axis] * weights[i] for i in range(particles)) / total for axis in range(3))
    depth = dot(sub(point, current.origin), current.direction)
    peak = max(weights)
    confidence = min(1.0, peak / max(1.0e-9, total / particles) / 20.0)
    return depth, confidence, hypotheses


def nearest_cloud_range(origin, direction, points, angular_radius_m: float = 0.35) -> float | None:
    best = None
    for point in points:
        perpendicular, along = distance_to_ray(point, Ray(origin, direction, 0, 1.0, 0))
        if math.isfinite(perpendicular) and perpendicular <= angular_radius_m and (best is None or along < best):
            best = along
    return best


def nearest_record(records, stamp_ns, kind):
    candidates = [r for r in records if r.get("kind") == kind and r.get("file")]
    if not candidates:
        return None
    return min(candidates, key=lambda r: abs(int(r.get("stamp_ns", 0)) - stamp_ns))


def metrics(rows, key):
    errors = [abs(r[key] - r["truth_m"]) for r in rows if r.get(key) is not None and r.get("truth_m") is not None]
    if not errors:
        return {"count": 0, "mae_m": None, "rmse_m": None}
    return {"count": len(errors), "mae_m": sum(errors) / len(errors),
            "rmse_m": math.sqrt(sum(e * e for e in errors) / len(errors))}


HTML = r'''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RGB heatmap distance feasibility</title>
<style>
body{margin:0;background:#101419;color:#e8edf2;font:14px system-ui,sans-serif}header{padding:14px 18px;border-bottom:1px solid #303944}main{display:grid;grid-template-columns:340px 1fr;gap:14px;padding:14px}.panel{background:#181e26;border:1px solid #303944;border-radius:6px;padding:14px}canvas{width:100%;height:auto;background:#0b0e12;border:1px solid #303944;border-radius:4px}button{margin:2px;padding:6px 10px;background:#263342;color:#eef;border:1px solid #46566a;border-radius:4px}input{width:100%}.metric{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #29313b}.good{color:#42d392}.bad{color:#ff8177}.muted{color:#9aa8b5}.heat{display:grid;grid-template-columns:repeat(5,1fr);gap:3px;margin:10px 0}.cell{padding:8px 2px;text-align:center;border-radius:3px;font-size:11px}.legend{line-height:1.8}.small{font-size:12px;color:#aeb9c4}@media(max-width:900px){main{display:block}.panel+ .panel{margin-top:12px}}
</style><header><b>RGB heatmap distance feasibility</b><div class="muted">WildOS-style multi-view rays vs logged point-cloud reference</div></header><main><aside class="panel"><button id="prev">上一帧</button><button id="next">下一帧</button><button id="play">播放</button><input id="slider" type="range" min="0" max="0" value="0"><div id="frame"></div><div id="metrics"></div><div class="heat" id="heat"></div><div class="legend"><span class="good">绿色</span>：RGB 多帧估计<br><span class="bad">红色</span>：固定 35 m 基线<br><span style="color:#ffd166">黄色</span>：点云真值参考</div><p class="small">真值为同步 logged PCD 中与当前相机射线最近的首个表面点；PCD 未进入估计器。</p></aside><section class="panel"><canvas id="rgb" width="640" height="384"></canvas><canvas id="plot" width="1000" height="600"></canvas></section></main>
<script>const D=__DATA__,frames=D.frames;let i=0,timer=null,$=x=>document.getElementById(x),fmt=x=>x==null?'-':Number(x).toFixed(2);function draw(){let f=frames[i];$('slider').value=i;$('frame').innerHTML=`<b>帧 ${i+1}/${frames.length}</b><br><span class="muted">${f.time_s.toFixed(2)} s · pose baseline ${f.history_baseline_m.toFixed(2)} m · ${f.rgb_name}</span>`;let es=f.rows.filter(x=>x.truth_m!=null),ae=es.filter(x=>x.estimate_m!=null),be=es.filter(x=>x.baseline_m!=null);let mae=a=>a.length?a.reduce((s,x)=>s+Math.abs(x.estimate_m-x.truth_m),0)/a.length:null;let bmae=be.length?be.reduce((s,x)=>s+Math.abs(x.baseline_m-x.truth_m),0)/be.length:null;$('metrics').innerHTML=`<div class="metric"><span>全局 RGB MAE</span><b class="good">${fmt(D.summary.rgb_multiview.mae_m)} m</b></div><div class="metric"><span>全局 35m 基线 MAE</span><b class="bad">${fmt(D.summary.baseline.mae_m)} m</b></div><div class="metric"><span>本帧有效真值列</span><b>${es.length}/5</b></div><div class="metric"><span>本帧 RGB估计 MAE</span><b class="good">${fmt(mae(ae))} m</b></div><div class="metric"><span>本帧 35m基线 MAE</span><b class="bad">${fmt(bmae)} m</b></div><div class="metric"><span>本帧 RGB有效估计</span><b>${ae.length}/5</b></div>`;$('heat').innerHTML=f.rows.map(x=>`<div class="cell" style="background:hsl(${120-120*Math.min(1,x.score*2)} 55% 30%)">c${x.column}<br>${fmt(x.score)}<br><span class="good">${fmt(x.estimate_m)}</span>/<span style="color:#ffd166">${fmt(x.truth_m)}</span></div>`).join('');drawRgb(f);drawPlot(f)}function drawRgb(f){let c=$('rgb'),ctx=c.getContext('2d'),raw=atob(f.rgb_b64),im=ctx.createImageData(f.rgb_width,f.rgb_height);for(let j=0;j<im.data.length/4;j++){im.data[4*j]=raw.charCodeAt(3*j);im.data[4*j+1]=raw.charCodeAt(3*j+1);im.data[4*j+2]=raw.charCodeAt(3*j+2);im.data[4*j+3]=255}let tmp=document.createElement('canvas');tmp.width=f.rgb_width;tmp.height=f.rgb_height;tmp.getContext('2d').putImageData(im,0,0);ctx.imageSmoothingEnabled=false;ctx.drawImage(tmp,0,0,c.width,c.height)}function drawPlot(f){let c=$('plot'),ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.fillStyle='#0b0e12';ctx.fillRect(0,0,w,h);let max=60,p=55,x=d=>p+d/max*(w-2*p),y=k=>h-p-k/max*(h-2*p);ctx.strokeStyle='#303944';for(let d=0;d<=max;d+=10){ctx.beginPath();ctx.moveTo(x(d),p);ctx.lineTo(x(d),h-p);ctx.stroke();ctx.fillStyle='#9aa8b5';ctx.fillText(d+'m',x(d)-8,h-p+18)}ctx.fillStyle='#e8edf2';ctx.fillText('当前相机射线方向上的距离',p,22);f.rows.forEach((r,j)=>{let yy=75+j*95;ctx.strokeStyle='#263342';ctx.beginPath();ctx.moveTo(p,yy);ctx.lineTo(w-p,yy);ctx.stroke();ctx.fillStyle='#e8edf2';ctx.fillText('col '+r.column,p-38,yy+4);if(r.truth_m!=null){ctx.fillStyle='#ffd166';ctx.beginPath();ctx.arc(x(r.truth_m),yy,8,0,Math.PI*2);ctx.fill()}if(r.estimate_m!=null){ctx.fillStyle='#42d392';ctx.beginPath();ctx.arc(x(r.estimate_m),yy,7,0,Math.PI*2);ctx.fill()}ctx.fillStyle='#ff8177';ctx.fillRect(x(35)-2,yy-2,4,4)})}function sel(n){i=Math.max(0,Math.min(frames.length-1,n));draw()}$('slider').max=Math.max(0,frames.length-1);$('slider').oninput=e=>sel(+e.target.value);$('prev').onclick=()=>sel(i-1);$('next').onclick=()=>sel(i+1);$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='播放'}else{$('play').textContent='暂停';timer=setInterval(()=>{if(i>=frames.length-1){clearInterval(timer);timer=null;$('play').textContent='播放'}else sel(i+1)},500)}};draw();</script>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, default=Path("scalenav_ws/tmp/heatmap_rgb_distance.html"))
    parser.add_argument("--particles", type=int, default=128)
    parser.add_argument("--history", type=int, default=8)
    parser.add_argument("--min-depth", type=float, default=1.0)
    parser.add_argument("--max-depth", type=float, default=35.0)
    args = parser.parse_args()
    if args.particles <= 0 or args.history < 2:
        raise SystemExit("particles must be positive and history must be >= 2")
    session = args.session.resolve()
    records = parse_jsonl(session / "index.jsonl")
    odoms = [r for r in records if r.get("kind") == "odom"]
    semantics = [r for r in records if r.get("kind") == "semantic" and r.get("file")]
    if not odoms or not semantics:
        raise SystemExit("session must contain odom and semantic records")
    rng = random.Random(42)
    history: list[Ray] = []
    frames = []
    for semantic in semantics:
        stamp = int(semantic.get("stamp_ns", 0))
        odom = min(odoms, key=lambda r: abs(int(r.get("stamp_ns", 0)) - stamp))
        odom_data = odom.get("data", {})
        position = tuple(float(x) for x in odom_data["position"])
        orientation = tuple(float(x) for x in odom_data["orientation"])
        width, height, values = read_pgm16_mm(session / semantic["file"])
        scores = patch_means(values, width, height)
        cloud_record = nearest_record(records, stamp, "pointcloud")
        cloud_body = read_pcd(session / cloud_record["file"]) if cloud_record else []
        cloud_world = [add(position, qrotate(orientation, p)) for p in cloud_body]
        current: list[Ray] = []
        for col, score in enumerate(scores):
            origin, direction = ray_for_column(position, orientation, col, width, height)
            current.append(Ray(origin, direction, col, float(score or 0.0), stamp))
        # Keep the current RGB observation in the estimator, just like the
        # official WildOS implementation includes the current view.
        rows = []
        for ray in current:
            prior = history + [ray]
            estimate, confidence, _ = estimate_distance(
                ray, prior, args.particles, rng, args.min_depth, args.max_depth)
            truth = nearest_cloud_range(ray.origin, ray.direction, cloud_world)
            rows.append({"column": ray.column, "score": ray.score,
                         "estimate_m": estimate, "confidence": confidence,
                         "truth_m": truth, "baseline_m": 35.0})
        history.extend(current)
        history = history[-args.history * COLS:]
        rgb_record = nearest_record(records, stamp, "rgb")
        rgb_b64 = ""
        rgb_name = "missing"
        rgb_width, rgb_height = 1, 1
        if rgb_record:
            rgb_width, rgb_height, rgb_data = read_ppm_rgb(session / rgb_record["file"])
            rgb_b64 = base64.b64encode(rgb_data).decode("ascii")
            rgb_name = rgb_record["file"]
        history_origins = {r.stamp_ns: r.origin for r in history}
        origins = list(history_origins.values())
        baseline = max((norm(sub(a, b)) for a in origins for b in origins), default=0.0)
        frames.append({"stamp_ns": stamp, "time_s": (stamp - int(semantics[0].get("stamp_ns", stamp))) * 1e-9,
                       "history_baseline_m": baseline, "rows": rows,
                       "rgb_b64": rgb_b64, "rgb_width": rgb_width,
                       "rgb_height": rgb_height, "rgb_name": rgb_name})
    flat = [row for frame in frames for row in frame["rows"]]
    summary = {"session": str(session), "frames": len(frames),
               "baseline": metrics(flat, "baseline_m"),
               "rgb_multiview": metrics(flat, "estimate_m"),
               "note": "truth is nearest logged point-cloud surface along the projected RGB ray"}
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(HTML.replace("__DATA__", json.dumps({"summary": summary, "frames": frames}, separators=(",", ":"))), encoding="utf-8")
    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", encoding="utf-8") as stream:
        stream.write("stamp_ns,time_s,column,score,estimate_m,confidence,truth_m,baseline_m\n")
        for frame in frames:
            for row in frame["rows"]:
                values = [frame["stamp_ns"], frame["time_s"], row["column"], row["score"],
                          row["estimate_m"], row["confidence"], row["truth_m"], row["baseline_m"]]
                stream.write(",".join("" if value is None else str(value) for value in values) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"html={output}")
    print(f"csv={csv_path}")


if __name__ == "__main__":
    main()
