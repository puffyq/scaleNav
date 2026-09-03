#!/usr/bin/env python3
"""Create a self-contained HTML timeline viewer from a scalenav_log session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def nearest(records, stamp_ns):
    if not records:
        return None
    return min(records, key=lambda item: abs(item["stamp_ns"] - stamp_ns))


def load_path(session, record):
    if record is None:
        return []
    try:
        payload = json.loads((session / record["file"]).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, KeyError):
        return []
    return payload.get("poses", payload.get("points", []))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    output = (args.output or (session / "route_yopo_timeline.html")).resolve()
    records = [json.loads(line) for line in (session / "index.jsonl").open(encoding="utf-8") if line.strip()]
    statuses = [r for r in records if r.get("kind") == "route_yopo_status"]
    odom = [r for r in records if r.get("kind") == "odom"]
    controls = [r for r in records if r.get("kind") == "control"]
    paths = [r for r in records if r.get("kind") == "path" and r.get("file")]
    planned_paths = [
        r
        for r in records
        if r.get("kind") == "route_yopo_planned_path" and r.get("file")
    ]
    odom.sort(key=lambda r: r["stamp_ns"]); controls.sort(key=lambda r: r["stamp_ns"])
    paths.sort(key=lambda r: r["stamp_ns"])
    planned_paths.sort(key=lambda r: r["stamp_ns"])
    latest_path = None
    latest_planned_path = None
    frames = []
    for status in statuses:
        stamp = int(status["stamp_ns"])
        for path_record in paths:
            if path_record["stamp_ns"] <= stamp:
                latest_path = path_record
            else:
                break
        for path_record in planned_paths:
            if path_record["stamp_ns"] <= stamp:
                latest_planned_path = path_record
            else:
                break
        path = load_path(session, latest_path)
        planned_path = load_path(session, latest_planned_path)
        od = nearest(odom, stamp); ct = nearest(controls, stamp)
        frames.append({
            "t": stamp * 1e-9,
            "status": status.get("data", {}),
            "odom": None if od is None else od.get("data", {}),
            "control": None if ct is None else ct.get("data", {}),
            "path": path,
            "planned_path": planned_path,
        })
    if not frames:
        raise SystemExit("session contains no route_yopo_status records")
    html = TEMPLATE.replace("__SESSION__", json.dumps(str(session)))
    html = html.replace("__FRAMES__", json.dumps(frames, separators=(",", ":")))
    output.write_text(html, encoding="utf-8")
    print(output)


TEMPLATE = r'''<!doctype html>
<meta charset="utf-8"><title>Route-YOPO Log Timeline</title>
<style>
body{margin:0;background:#101318;color:#e8edf2;font:14px system-ui, sans-serif}header{padding:14px 20px;border-bottom:1px solid #303740}main{display:grid;grid-template-columns:340px 1fr;gap:14px;padding:14px}aside{background:#181d24;border:1px solid #303740;padding:14px;border-radius:6px}canvas{width:100%;height:520px;background:#0b0e12;border:1px solid #303740;border-radius:6px}label{display:block;color:#9da9b5;margin:12px 0 5px}input[type=range]{width:100%}button{background:#2b6cb0;color:white;border:0;border-radius:4px;padding:7px 12px;margin-right:6px;cursor:pointer}pre{white-space:pre-wrap;word-break:break-word;color:#c7d2de;font-size:12px}.value{color:#fff;font-weight:600}.warn{color:#ffbf69}.bad{color:#ff6b6b}.ok{color:#72e6a2}
</style>
<header><b>Route-YOPO Log Timeline</b><div id="session"></div></header>
<main><aside><button id="play">Play</button><button id="reset">Reset</button><label>Time <span id="time" class="value"></span></label><input id="slider" type="range" min="0" value="0" step="1"><div id="summary"></div><label>Status</label><pre id="status"></pre><label>Odometry</label><pre id="odom"></pre><label>Control</label><pre id="control"></pre></aside><section><canvas id="plot" width="1100" height="620"></canvas><p>Top view: <span style="color:#72e6a2">green = executed odometry</span>, <span style="color:#4fd1ff">cyan = ScaleNav guidance route</span>, <span style="color:#ff7b72">red = Route-YOPO predicted polynomial</span>, yellow dot = current pose.</p></section></main>
<script>
const SESSION=__SESSION__, FRAMES=__FRAMES__; let i=0, timer=null;
session.textContent=SESSION; slider.max=Math.max(0,FRAMES.length-1);
function fmt(v){return typeof v==='number'?v.toFixed(3):v}
function draw(f){
  const c=plot,ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);ctx.fillStyle='#0b0e12';ctx.fillRect(0,0,w,h);
  const all=[]; FRAMES.forEach(x=>{if(x.odom?.position)all.push(x.odom.position)}); if(f.path)f.path.forEach(p=>all.push(p)); if(f.planned_path)f.planned_path.forEach(p=>all.push(p));
  let minx=-10,maxx=10,miny=-10,maxy=10; if(all.length){minx=Math.min(...all.map(p=>p[0]));maxx=Math.max(...all.map(p=>p[0]));miny=Math.min(...all.map(p=>p[1]));maxy=Math.max(...all.map(p=>p[1]));}
  const pad=35, sx=(w-2*pad)/Math.max(1,maxx-minx), sy=(h-2*pad)/Math.max(1,maxy-miny), s=Math.min(sx,sy); const X=x=>pad+(x-minx)*s, Y=y=>h-pad-(y-miny)*s;
  ctx.strokeStyle='#242b34';ctx.lineWidth=1;for(let x=Math.ceil(minx);x<=maxx;x+=5){ctx.beginPath();ctx.moveTo(X(x),pad);ctx.lineTo(X(x),h-pad);ctx.stroke()}for(let y=Math.ceil(miny);y<=maxy;y+=5){ctx.beginPath();ctx.moveTo(pad,Y(y));ctx.lineTo(w-pad,Y(y));ctx.stroke()}
  function line(points,color,width){if(!points.length)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();points.forEach((p,n)=>n?ctx.lineTo(X(p[0]),Y(p[1])):ctx.moveTo(X(p[0]),Y(p[1])));ctx.stroke()}
  line(FRAMES.slice(0,i+1).filter(x=>x.odom?.position).map(x=>x.odom.position),'#72e6a2',3); line((f.path||[]),'#4fd1ff',3); line((f.planned_path||[]),'#ff7b72',4);
  if(f.odom?.position){ctx.fillStyle='#ffd166';ctx.beginPath();ctx.arc(X(f.odom.position[0]),Y(f.odom.position[1]),7,0,Math.PI*2);ctx.fill()}
  time.textContent=((f.t-FRAMES[0].t).toFixed(2)+' s'); slider.value=i;
  const st=f.status||{}, reason=st.reason||'', hasPlan=(f.planned_path||[]).length>0; summary.innerHTML=`<div>mode: <span class="value">${st.mode||''}</span></div><div>reason: <span class="${reason.includes('candidate')||reason.includes('invalid')?'warn':'ok'}">${reason}</span></div><div>primitive: <span class="value">${st.selected_primitive??'none'}</span></div><div>trajectory replaced: <span class="value">${st.trajectory_replaced??false}</span></div><div>predicted polynomial: <span class="${hasPlan?'ok':'bad'}">${hasPlan?f.planned_path.length+' samples':'not recorded'}</span></div>`;
  status.textContent=JSON.stringify({reason:st.reason,mode:st.mode,input_mode:st.input_mode,selected_primitive:st.selected_primitive,selection_policy:st.selection_policy,route_terminal_error_m:st.route_terminal_error_m},null,2);
  odom.textContent=JSON.stringify(f.odom?{position:f.odom.position,orientation:f.odom.orientation,velocity:f.odom.velocity}:null,null,2);
  control.textContent=JSON.stringify(f.control?{position:f.control.position_world||f.control.position,velocity:f.control.velocity_world||f.control.velocity,acceleration:f.control.acceleration_world||f.control.acceleration,yaw_deg:f.control.yaw_deg}:null,null,2);
}
slider.oninput=()=>{i=+slider.value;draw(FRAMES[i])};play.onclick=()=>{if(timer){clearInterval(timer);timer=null;play.textContent='Play'}else{play.textContent='Pause';timer=setInterval(()=>{i=(i+1)%FRAMES.length;draw(FRAMES[i])},100)}};reset.onclick=()=>{i=0;draw(FRAMES[0])};draw(FRAMES[0]);
</script>'''


if __name__ == "__main__":
    main()
