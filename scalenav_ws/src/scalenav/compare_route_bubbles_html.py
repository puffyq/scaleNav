#!/usr/bin/env python3
"""Compare logged route-bubble sampling strategies in a self-contained HTML."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


ANCHORS = np.asarray([1, 2, 3, 4, 5, 6, 8, 10, 14, 18, 24, 30], dtype=np.float64)
ROBOT_RADIUS = 0.3
SAFETY_MARGIN = 0.2
RADIUS_CAP = 3.0


def arclength(points: np.ndarray) -> tuple[np.ndarray, float]:
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    return cumulative, float(cumulative[-1])


def interpolate(points: np.ndarray, cumulative: np.ndarray, distance: float) -> np.ndarray:
    distance = float(np.clip(distance, 0.0, cumulative[-1]))
    right = int(np.searchsorted(cumulative, distance, side="right"))
    if right <= 0:
        return points[0].copy()
    if right >= len(points):
        return points[-1].copy()
    left = right - 1
    span = float(cumulative[right] - cumulative[left])
    alpha = 0.0 if span <= 1.0e-9 else (distance - cumulative[left]) / span
    return points[left] + alpha * (points[right] - points[left])


def load_pair(path_file: Path, bubbles_file: Path) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        path = np.asarray(
            json.loads(path_file.read_text())["poses"],
            dtype=np.float64,
        )
        markers = json.loads(bubbles_file.read_text())["markers"]
    except (OSError, KeyError, json.JSONDecodeError, ValueError):
        return None
    route_centers = []
    route_radii = []
    for marker in markers:
        if marker.get("ns") != "scalenav_route_bubble_radius" or marker.get("action") != 0:
            continue
        route_centers.append(marker["pose"]["position"])
        route_radii.append(float(marker["scale"][0]) * 0.5)
    if path.ndim != 2 or path.shape[1:] != (3,) or len(path) < 2 or not route_centers:
        return None
    route_centers = np.asarray(route_centers, dtype=np.float64)
    route_radii = np.asarray(route_radii, dtype=np.float64)
    distance = np.linalg.norm(path[:, None, :] - route_centers[None, :, :], axis=2)
    nearest = np.argmin(distance, axis=1)
    nearest_distance = distance[np.arange(len(path)), nearest]
    if np.any(nearest_distance > 1.0):
        return None
    # Match the controller's body-radius conversion, but retain the raw values
    # for the HTML so the cost of that conversion remains visible.
    safe_radii = np.clip(route_radii[nearest] - ROBOT_RADIUS - SAFETY_MARGIN, 0.05, RADIUS_CAP)
    return path, safe_radii


def make_frame(frame: int, path: np.ndarray, safe_radii: np.ndarray, stamp_ns: int | None = None) -> dict:
    cumulative, length = arclength(path)
    valid_anchors = ANCHORS[ANCHORS <= length + 1.0e-5]
    sample_distances = np.concatenate((valid_anchors, ANCHORS[len(valid_anchors) :]))
    current_centers = np.asarray(
        [interpolate(path, cumulative, distance) for distance in sample_distances]
    )
    current_radii = np.asarray(
        [np.interp(min(float(distance), length), cumulative, safe_radii) for distance in sample_distances]
    )
    # Direct raw-bubble binding: no geometric or radius interpolation. For an
    # anchor between two route vertices, choose the closest original vertex.
    nearest_indices = np.abs(cumulative[:, None] - sample_distances[None, :]).argmin(axis=0)
    raw_centers = path[nearest_indices]
    raw_radii = safe_radii[nearest_indices]
    return {
        "frame": frame,
        "stamp_ns": stamp_ns,
        "path": path.round(4).tolist(),
        "length_m": length,
        "anchor_distances_m": sample_distances.round(4).tolist(),
        "current_centers": current_centers.round(4).tolist(),
        "current_radii": current_radii.round(4).tolist(),
        "raw_centers": raw_centers.round(4).tolist(),
        "raw_radii": raw_radii.round(4).tolist(),
        "center_jump_current_m": np.linalg.norm(np.diff(current_centers, axis=0), axis=1).round(4).tolist(),
        "center_jump_raw_m": np.linalg.norm(np.diff(raw_centers, axis=0), axis=1).round(4).tolist(),
    }


TEMPLATE = r'''<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Route bubble A/B offline replay</title>
<style>
body{margin:0;background:#101419;color:#e9eef2;font:14px system-ui,sans-serif}header{padding:14px 18px;border-bottom:1px solid #303944}main{display:grid;grid-template-columns:360px 1fr;gap:14px;padding:14px}.panel{background:#181e26;border:1px solid #303944;border-radius:6px;padding:14px}canvas{width:100%;height:auto;background:#0b0e12;border:1px solid #303944;border-radius:4px}label{display:block;color:#aeb9c4;margin:12px 0 5px}input[type=range]{width:100%}.metric{display:flex;justify-content:space-between;gap:8px;padding:6px 0;border-bottom:1px solid #29313b}.value{color:#fff;font-weight:600}.muted{color:#9aa8b5}.legend{line-height:1.8}.current{color:#46d6a2}.raw{color:#ff9e64}.route{color:#4fd1ff}.warn{color:#ffd166}pre{white-space:pre-wrap;word-break:break-word;font-size:12px;color:#cbd5df;max-height:280px;overflow:auto}@media(max-width:900px){main{display:block}.panel+ .panel{margin-top:12px}}
</style>
<header><b>Route bubble A/B offline replay</b><div class="muted">__SESSION__</div></header>
<main><aside class="panel"><button id="prev">上一帧</button> <button id="next">下一帧</button> <button id="play">播放</button><label>帧 <span id="frameNo" class="value"></span></label><input id="slider" type="range" min="0" max="0" value="0" step="1"><div id="metrics"></div><label>图例</label><div class="legend"><span class="route">青色</span>：记录的路线<br><span class="current">绿色</span>：当前弧长插值中心/半径<br><span class="raw">橙色</span>：原始路线 bubble 最近顶点绑定<br><span class="warn">黄色点</span>：路线起点</div><label>说明</label><pre id="detail"></pre></aside><section class="panel"><canvas id="plot" width="1200" height="760"></canvas><p class="muted">本页面重放路线和原始 bubble 数据，不包含未记录的 MPC 求解轨迹；圆的半径已扣除无人机半径 0.3 m 和安全余量 0.2 m，并保留 3 m 上限。</p></section></main>
<script>
const FRAMES=__FRAMES__;let index=0,timer=null;const $=id=>document.getElementById(id);
function fmt(x,d=2){return Number.isFinite(Number(x))?Number(x).toFixed(d):'-'}
function draw(){const f=FRAMES[index],c=$('plot'),ctx=c.getContext('2d');$('frameNo').textContent=`${index+1}/${FRAMES.length} · graph frame ${f.frame}`;$('slider').value=index;const all=[...f.path,...f.current_centers,...f.raw_centers];let minx=Math.min(...all.map(p=>p[0]))-2,maxx=Math.max(...all.map(p=>p[0]))+2,miny=Math.min(...all.map(p=>p[1]))-2,maxy=Math.max(...all.map(p=>p[1]))+2;const pad=55,s=Math.min((c.width-2*pad)/(maxx-minx||1),(c.height-2*pad)/(maxy-miny||1)),X=x=>pad+(x-minx)*s,Y=y=>c.height-pad-(y-miny)*s;ctx.fillStyle='#0b0e12';ctx.fillRect(0,0,c.width,c.height);ctx.strokeStyle='#252d36';ctx.lineWidth=1;for(let x=Math.ceil(minx);x<=maxx;x+=5){ctx.beginPath();ctx.moveTo(X(x),pad);ctx.lineTo(X(x),c.height-pad);ctx.stroke()}for(let y=Math.ceil(miny);y<=maxy;y+=5){ctx.beginPath();ctx.moveTo(pad,Y(y));ctx.lineTo(c.width-pad,Y(y));ctx.stroke()}function line(ps,col,w,dash=[]){if(ps.length<2)return;ctx.save();ctx.strokeStyle=col;ctx.lineWidth=w;ctx.setLineDash(dash);ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(X(p[0]),Y(p[1])):ctx.moveTo(X(p[0]),Y(p[1])));ctx.stroke();ctx.restore()}function bubbles(cs,rs,col){cs.forEach((p,i)=>{ctx.save();ctx.strokeStyle=col;ctx.globalAlpha=.65;ctx.lineWidth=2;ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),Math.max(3,rs[i]*s),0,Math.PI*2);ctx.stroke();ctx.globalAlpha=1;ctx.fillStyle=col;ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),4,0,Math.PI*2);ctx.fill();ctx.restore()})}line(f.path,'#4fd1ff',4);line(f.current_centers,'#46d6a2',2);line(f.raw_centers,'#ff9e64',2,[8,5]);bubbles(f.current_centers,f.current_radii,'#46d6a2');bubbles(f.raw_centers,f.raw_radii,'#ff9e64');ctx.fillStyle='#ffd166';ctx.beginPath();ctx.arc(X(f.path[0][0]),Y(f.path[0][1]),7,0,Math.PI*2);ctx.fill();$('metrics').innerHTML=`<div class="metric"><span>路线长度</span><b>${fmt(f.length_m)} m</b></div><div class="metric"><span>当前半径均值</span><b>${fmt(f.current_radii.reduce((a,b)=>a+b,0)/f.current_radii.length)} m</b></div><div class="metric"><span>原始绑定半径均值</span><b>${fmt(f.raw_radii.reduce((a,b)=>a+b,0)/f.raw_radii.length)} m</b></div><div class="metric"><span>当前最大中心跳变</span><b>${fmt(Math.max(...f.center_jump_current_m))} m</b></div><div class="metric"><span>原始最大中心跳变</span><b>${fmt(Math.max(...f.center_jump_raw_m))} m</b></div>`;$('detail').textContent=JSON.stringify({anchors_m:f.anchor_distances_m,current_radii_m:f.current_radii,raw_radii_m:f.raw_radii,current_center_jumps_m:f.center_jump_current_m,raw_center_jumps_m:f.center_jump_raw_m},null,2)}
function select(i){index=Math.max(0,Math.min(FRAMES.length-1,i));draw()}$('slider').max=Math.max(0,FRAMES.length-1);$('slider').oninput=e=>select(+e.target.value);$('prev').onclick=()=>select(index-1);$('next').onclick=()=>select(index+1);$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='播放'}else{$('play').textContent='暂停';timer=setInterval(()=>{if(index>=FRAMES.length-1){clearInterval(timer);timer=null;$('play').textContent='播放'}else select(index+1)},500)}};draw();
</script>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, default=Path("train_scalenav/tmp/route_bubble_ab.html"))
    args = parser.parse_args()
    graph = args.session / "graph"
    frames = []
    index_file = args.session / "index.jsonl"
    if index_file.is_file():
        records = [json.loads(line) for line in index_file.read_text().splitlines() if line.strip()]
        paths = {int(r["stamp_ns"]): r for r in records if r.get("kind") == "path" and r.get("file")}
        bubbles = {int(r["stamp_ns"]): r for r in records if r.get("kind") == "bubbles" and r.get("file")}
        for ordinal, stamp_ns in enumerate(sorted(set(paths) & set(bubbles)), start=1):
            pair = load_pair(args.session / paths[stamp_ns]["file"], args.session / bubbles[stamp_ns]["file"])
            if pair is not None:
                frames.append(make_frame(ordinal, *pair, stamp_ns=stamp_ns))
    else:
        for path_file in graph.glob("path_*.json"):
            stem = path_file.stem.removeprefix("path_")
            bubbles_file = graph / f"bubbles_{stem}.json"
            if not stem.isdigit() or not bubbles_file.is_file():
                continue
            pair = load_pair(path_file, bubbles_file)
            if pair is not None:
                frames.append(make_frame(int(stem), *pair))
    frames.sort(key=lambda item: item["frame"])
    if not frames:
        raise SystemExit("no paired path/bubble frames found")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    html = TEMPLATE.replace("__SESSION__", str(args.session.resolve()))
    html = html.replace("__FRAMES__", json.dumps(frames, separators=(",", ":")))
    output.write_text(html, encoding="utf-8")
    print(f"paired_frames={len(frames)}")
    print(output)


if __name__ == "__main__":
    main()
