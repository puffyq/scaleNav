#!/usr/bin/env python3
"""Create one uncluttered reverse semantic projection diagnostic frame."""

from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
from typing import Any

import semantic_frontier_module_test as replay


def read_pcd(path: Path) -> list[list[float]]:
    points: list[list[float]] = []
    data = path.read_text(encoding="utf-8", errors="replace").splitlines()
    try:
        start = data.index("DATA ascii") + 1
    except ValueError:
        return points
    for line in data[start:]:
        values = line.split()
        if len(values) < 3:
            continue
        try:
            point = [float(values[0]), float(values[1]), float(values[2])]
        except ValueError:
            continue
        if all(math.isfinite(value) for value in point):
            points.append(point)
    return points


def world_cloud(session: Path, frame: dict[str, Any]) -> list[list[float]]:
    stamp = int(frame["stamp_ns"])
    records = replay.read_log_jsonl(session / "index.jsonl")
    pointcloud = replay.nearest_record(records, "pointcloud", stamp)
    if not pointcloud or not pointcloud.get("file"):
        return []
    odom = frame["odom"]
    position = odom.get("position", [0.0, 0.0, 0.0])
    quaternion = odom.get("orientation", [0.0, 0.0, 0.0, 1.0])
    points = read_pcd(session / pointcloud["file"])
    transformed: list[list[float]] = []
    for point in points:
        rotated = replay.quat_rotate(quaternion, point)
        transformed.append([position[i] + rotated[i] for i in range(3)])
    return transformed


def image_uri(image: dict[str, Any] | None) -> str:
    if not image:
        return ""
    return f"data:{image['mime']};base64,{image['data']}"


def focused_payload(session: Path, frame_index: int) -> dict[str, Any]:
    result = replay.build_log_result(session)
    if not result["frames"]:
        raise ValueError("session has no semantic frames")
    frame_index = max(0, min(frame_index, len(result["frames"]) - 1))
    frame = result["frames"][frame_index]
    cloud = world_cloud(session, frame)
    odom = frame["odom"]["position"]
    z0 = float(odom[2])
    # Show the obstacle surfaces around the flight plane, not the ground or
    # ceiling. Downsample deterministically to keep the SVG readable.
    cloud = [
        [round(point[0], 3), round(point[1], 3)]
        for index, point in enumerate(cloud)
        if abs(point[2] - z0) <= 0.8 and
        math.hypot(point[0] - odom[0], point[1] - odom[1]) <= 28.0 and
        index % 3 == 0
    ]
    annotations = frame.get("reverse_projection", {}).get("nodes", [])
    skeleton = frame.get("graph", {}).get("skeleton", [])
    nodes = []
    for index, point in enumerate(skeleton):
        annotation = annotations[index] if index < len(annotations) else {}
        score = annotation.get("score")
        nodes.append({
            "i": index,
            "x": float(point[0]),
            "y": float(point[1]),
            "score": None if score is None else float(score),
            "visible": bool(annotation.get("visible")),
            "pu": annotation.get("pixel_u"),
            "pv": annotation.get("pixel_v"),
            "depth": annotation.get("projection_depth_m"),
            "radius": annotation.get("object_radius_m"),
            "top": index == frame.get("reverse_projection", {}).get(
                "stats", {}).get("top_node_index"),
        })
    nearest_index = None
    nearest_distance = math.inf
    for index, node in enumerate(nodes):
        if not node["visible"] or node["score"] is None:
            continue
        distance = math.hypot(node["x"] - float(odom[0]),
                              node["y"] - float(odom[1]))
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_index = index
    if nearest_index is not None:
        nodes[nearest_index]["nearest_scored"] = True
        nodes[nearest_index]["odom_distance_m"] = nearest_distance
    return {
        "session": result["session"],
        "frame_index": frame_index,
        "frame_count": len(result["frames"]),
        "frame": {
            "time_s": frame["time_s"],
            "odom": frame["odom"]["position"],
            # Keep the synchronized ROS odom attitude in the focused
            # artifact.  The previous report only carried position, which
            # forced the static renderer to draw a fake +X direction.
            "odom_orientation": frame["odom"].get(
                "orientation", [0.0, 0.0, 0.0, 1.0]),
            "goal": frame["goal"],
            "image_size": frame["image_size"],
            "rgb": image_uri(frame.get("rgb")),
            "depth": image_uri(frame.get("depth")),
            "heatmap": image_uri(frame.get("heatmap")),
            "patch_means": frame["patch_means"],
            "nodes": nodes,
            "cloud": cloud,
            "stats": frame.get("reverse_projection", {}).get("stats", {}),
        },
    }


HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Map2 reverse semantic projection — focused frame</title>
<style>
:root{--bg:#0b1118;--panel:#131e28;--line:#2c4150;--text:#edf4f7;--muted:#9db0bb}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);
font:14px/1.4 system-ui,sans-serif}.shell{max-width:1500px;margin:auto;padding:18px}
h1{font-size:22px;margin:0}.muted{color:var(--muted)}.head{display:flex;
justify-content:space-between;gap:20px;align-items:end;border-bottom:1px solid var(--line);
padding-bottom:12px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;
margin-top:14px}.panel{background:var(--panel);border:1px solid var(--line);
border-radius:6px;padding:10px}.wide{grid-column:1/-1}.image{width:100%;display:block;
background:#071016;border:1px solid var(--line);aspect-ratio:5/3}.facts{display:grid;
grid-template-columns:repeat(4,1fr);gap:8px}.fact{border-left:3px solid #55d6e8;padding-left:8px}
.fact b{display:block;font-size:17px}.fact small{color:var(--muted)}.legend{color:var(--muted);
font-size:12px;margin-top:7px}.world{width:100%;height:620px;background:#071016;
border:1px solid var(--line)}.heat{position:relative}.heat img{width:100%;display:block;
image-rendering:auto}.heat canvas{position:absolute;inset:0;width:100%;height:100%}
table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:left;
padding:6px;border-bottom:1px solid var(--line)}th{color:var(--muted)}.high{color:#ff776f}
.low{color:#56d58b}.top{font-weight:700;color:#fff}
@media(max-width:900px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}}
</style></head><body><main class="shell">
<header class="head"><div><h1>普通 Verified 节点反投影：单帧清晰验证</h1>
<div id="subtitle" class="muted"></div></div><div class="muted">真实 map2 日志 · 离线重算</div></header>
<section class="grid"><article class="panel"><h2>RGB（3×5 分块）</h2><canvas id="rgb" class="image"></canvas></article>
<article class="panel"><h2>Depth（3×5 分块）</h2><canvas id="depth" class="image"></canvas></article>
<article class="panel heat"><h2>Heatmap + 节点反投影位置</h2><img id="heatimg"><canvas id="heat"></canvas></article>
<article class="panel wide"><div id="facts" class="facts"></div>
<div class="legend">节点颜色是重新计算的 Gaussian 语义分数：浅色/绿色 = 低风险，橙色 = 中等，红色 = 高风险；白色外圈 = 本帧最高分节点。灰色点云 = 当前帧、飞行高度附近的障碍表面点。</div></article>
<article class="panel wide"><h2>世界 XY 俯视图（只保留颜色，不画密集标签）</h2>
<svg id="world" class="world" viewBox="0 0 1100 620"></svg>
<div class="legend">灰色点云 = 障碍物表面；圆点 = graph skeleton Verified 节点，颜色来自“3D → 热力图 → depth-aware Gaussian”；青色实箭头 = odom 机体前向，黄色虚线 = mission goal 方向，白圈 = 最高语义分数节点，青色外圈 = 距 odom 最近的已赋分节点。</div></article>
<article class="panel wide"><h2>本帧节点分数</h2><div id="stats"></div></article>
</section></main><script>
const D=__DATA__,F=D.frame,$=id=>document.getElementById(id);
function esc(s){return String(s??'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function scoreColor(v){if(!Number.isFinite(+v))return [125,140,150];let t=Math.max(0,Math.min(1,+v));
const a=[[237,243,240],[217,173,61],[209,78,70]],q=t*2,i=Math.min(1,Math.floor(q)),u=q-i;
return a[i].map((x,k)=>Math.round(x*(1-u)+a[i+1][k]*u))}
function loadImage(uri,cb){if(!uri){cb(null);return}const im=new Image();im.onload=()=>cb(im);im.src=uri}
function drawImage(id,uri,overlay){const c=$(id),x=c.getContext('2d');loadImage(uri,im=>{if(!im)return;c.width=im.naturalWidth;c.height=im.naturalHeight;
x.drawImage(im,0,0);x.strokeStyle='rgba(255,255,255,.55)';x.lineWidth=1;
for(let r=0;r<=3;r++){x.beginPath();x.moveTo(0,r*c.height/3);x.lineTo(c.width,r*c.height/3);x.stroke()}
for(let k=0;k<=5;k++){x.beginPath();x.moveTo(k*c.width/5,0);x.lineTo(k*c.width/5,c.height);x.stroke()}
if(overlay){for(const n of F.nodes){if(!n.visible||!Number.isFinite(n.score)||!Number.isFinite(n.pu))continue;
const col=scoreColor(n.score),px=n.pu*c.width,py=n.pv*c.height;x.beginPath();x.arc(px,py,4,0,Math.PI*2);
x.fillStyle=`rgb(${col.join(',')})`;x.fill();x.strokeStyle='#fff';x.stroke()}}})}
function drawHeat(){const c=$('heat'),x=c.getContext('2d');loadImage(F.heatmap,im=>{if(!im)return;c.width=im.naturalWidth;c.height=im.naturalHeight;
x.drawImage(im,0,0);x.strokeStyle='rgba(255,255,255,.7)';x.lineWidth=1;for(let r=0;r<=3;r++){x.beginPath();x.moveTo(0,r*c.height/3);x.lineTo(c.width,r*c.height/3);x.stroke()}
for(let k=0;k<=5;k++){x.beginPath();x.moveTo(k*c.width/5,0);x.lineTo(k*c.width/5,c.height);x.stroke()}
for(const n of F.nodes){if(!n.visible||!Number.isFinite(n.score)||!Number.isFinite(n.pu))continue;const col=scoreColor(n.score);
const px=n.pu*c.width,py=n.pv*c.height;x.beginPath();x.arc(px,py,5,0,Math.PI*2);x.fillStyle=`rgb(${col.join(',')})`;x.fill();x.strokeStyle='#fff';x.stroke()}})}
function drawWorld(){const s=$('world'),all=[...F.cloud.map(p=>[p[0],p[1]]),...F.nodes.map(n=>[n.x,n.y]),[F.odom[0],F.odom[1]],[F.goal.x,F.goal.y]];
let minx=Math.min(...all.map(p=>p[0]))-3,maxx=Math.max(...all.map(p=>p[0]))+3,miny=Math.min(...all.map(p=>p[1]))-3,maxy=Math.max(...all.map(p=>p[1]))+3;
const scale=Math.min(1040/(maxx-minx||1),560/(maxy-miny||1)),cx=(minx+maxx)/2,cy=(miny+maxy)/2,sx=x=>550+(x-cx)*scale,sy=y=>310-(y-cy)*scale;let o='';
for(const p of F.cloud)o+=`<circle cx="${sx(p[0]).toFixed(1)}" cy="${sy(p[1]).toFixed(1)}" r="1.3" fill="#6f7c83" opacity=".34"/>`;
for(const n of F.nodes){const col=scoreColor(n.score),fill=`rgb(${col.join(',')})`,r=n.top?7:4; o+=`<circle cx="${sx(n.x).toFixed(1)}" cy="${sy(n.y).toFixed(1)}" r="${r}" fill="${fill}" stroke="${n.top?'#fff':'#16232d'}" stroke-width="${n.top?2:1}"/>`;}
const ox=sx(F.odom[0]),oy=sy(F.odom[1]);o+=`<circle cx="${ox}" cy="${oy}" r="7" fill="#55d6e8"/><text x="${ox+10}" y="${oy-9}" fill="#55d6e8">odom</text>`;
const q=F.odom_orientation||[0,0,0,1],tx=0,ty=2*q[2],tz=-2*q[1],fw=[1+q[3]*tx+(q[1]*tz-q[2]*ty),q[3]*ty+(q[2]*tx-q[0]*tz),q[3]*tz+(q[0]*ty-q[1]*tx)],fn=Math.hypot(fw[0],fw[1])||1,flen=80,fx=ox+flen*fw[0]/fn,fy=oy-flen*fw[1]/fn;o+=`<line x1="${ox}" y1="${oy}" x2="${fx}" y2="${fy}" stroke="#55d6e8" stroke-width="4"/><text x="${fx+8}" y="${fy-8}" fill="#55d6e8">body forward</text>`;
const mdx=F.goal.x-F.odom[0],mdy=F.goal.y-F.odom[1],mn=Math.hypot(mdx,mdy)||1,mlen=120,mx=ox+mlen*mdx/mn,my=oy-mlen*mdy/mn;o+=`<line x1="${ox}" y1="${oy}" x2="${mx}" y2="${my}" stroke="#f4c95d" stroke-width="3" stroke-dasharray="8 6"/><text x="${mx+8}" y="${my-8}" fill="#f4c95d">toward mission goal</text>`;s.innerHTML=o}
function render(){const s=F.stats;$('subtitle').textContent=`${D.session} · frame ${D.frame_index+1}/${D.frame_count} · t=${F.time_s.toFixed(2)} s`;
$('facts').innerHTML=`<div class="fact"><b>${F.odom[0].toFixed(1)}, ${F.odom[1].toFixed(1)}</b><small>odom XY</small></div><div class="fact"><b>${s.annotated_nodes}/${s.visible_nodes}</b><small>annotated / visible</small></div><div class="fact"><b>${s.score_min.toFixed(3)}…${s.score_max.toFixed(3)}</b><small>score range</small></div><div class="fact"><b>N${s.top_node_index}</b><small>highest score</small></div>`;
$('stats').innerHTML=`<table><thead><tr><th>node</th><th>world XY</th><th>image pixel</th><th>depth</th><th>Gaussian radius</th><th>offline score</th><th>state</th></tr></thead><tbody>`+
F.nodes.filter(n=>Number.isFinite(n.score)).sort((a,b)=>b.score-a.score).map(n=>{const row=Number.isFinite(n.pv)?Math.min(3,Math.max(1,Math.floor(n.pv/F.image_size.height*3)+1)):'-';return `<tr class="${n.top?'top':''}"><td>N${n.i}</td><td>(${n.x.toFixed(2)}, ${n.y.toFixed(2)})</td><td>${n.pu.toFixed(1)}, ${n.pv.toFixed(1)}</td><td>row ${row}</td><td>${Number.isFinite(n.depth)?n.depth.toFixed(2)+' m':'-'}</td><td>${Number.isFinite(n.radius)?n.radius.toFixed(2)+' m':'-'}</td><td class="${n.score>=.35?'high':'low'}">${n.score.toFixed(4)}</td><td>${n.top?'最高分':''}</td></tr>`}).join('')+'</tbody></table>`;
drawImage('rgb',F.rgb,false);drawImage('depth',F.depth,false);drawHeat();drawWorld()}
render();
</script></body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--frame", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = focused_payload(args.session, args.frame)
    # Add image projection metadata used by the browser overlay.
    for node in payload["frame"]["nodes"]:
        # The data is already present in the replay annotations; keep this
        # helper robust when a node is not visible or has no score.
        source = payload["frame"]
        node.setdefault("pu", None)
        node.setdefault("pv", None)
        node.setdefault("depth", None)
        node.setdefault("radius", None)
    # Re-read annotations to preserve projection fields without duplicating
    # the full 30 MB replay JSON in this focused artifact.
    full = replay.build_log_result(args.session)["frames"][payload["frame_index"]]
    for node, annotation in zip(payload["frame"]["nodes"],
                                full.get("reverse_projection", {}).get("nodes", [])):
        node["pu"] = annotation.get("pixel_u")
        node["pv"] = annotation.get("pixel_v")
        node["depth"] = annotation.get("projection_depth_m")
        node["radius"] = annotation.get("object_radius_m")
    html = HTML.replace("__DATA__", json.dumps(payload, separators=(",", ":"),
                                                allow_nan=False))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html, encoding="utf-8")
    print(f"focused reverse-projection report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
