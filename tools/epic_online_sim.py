#!/usr/bin/env python3
"""Deterministic EPIC online timing/route simulation with a standalone HTML report.

The defaults mirror scripts/epic_online/start.sh:
  depth/free-rays 10 Hz, semantic heatmap 2 Hz, planner 5 Hz,
  skeleton rebuild 5 Hz, route decision 0.5 Hz.

This is a contract test for scheduling and observable behavior. It does not
pretend to be AirSim physics; the C++ integration test covers the real graph
and map data structures, while this script covers stream cadence, latency and
rebuild freshness.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path


PARAMETERS = {
    "duration_s": 12.0,
    "depth_hz": 10.0,
    "free_ray_hz": 10.0,
    "odom_hz": 50.0,
    "semantic_hz": 2.0,
    "planner_hz": 5.0,
    "skeleton_hz": 5.0,
    "route_hz": 0.5,
    "cloud_latency_ms": 100.0,
    "semantic_latency_ms": 20.0,
    # The latest graph timings are generally 20-100 ms. Keep a bounded
    # deterministic jitter to exercise the real 200 ms rebuild cadence.
    "skeleton_base_ms": 80.0,
    "skeleton_jitter_ms": 30.0,
    "route_latency_ms": 200.0,
    "depth_clip_m": 20.0,
    "speculative_range_m": 22.0,
}


def due(now: float, period: float, next_time: float) -> bool:
    return now + 1e-9 >= next_time


def simulate() -> dict:
    p = PARAMETERS
    dt = 0.01
    events = []
    depth_count = free_count = odom_count = semantic_count = 0
    planner_count = route_count = 0
    graph_version = 0
    graph_last_update = 0.0
    graph_rebuild_done = -1.0
    next_depth = next_free = next_odom = 0.0
    next_semantic = next_planner = next_route = 0.0
    next_skeleton = 0.0
    semantic_arrival = None
    semantic_applied = False
    route = "DIRECT"
    first_turn_time = None
    max_graph_age = 0.0
    missed_rebuilds = 0
    rebuilds = []

    steps = int(round(p["duration_s"] / dt))
    for tick in range(steps + 1):
        now = tick * dt

        if due(now, 1.0 / p["depth_hz"], next_depth):
            depth_count += 1
            events.append({"t": now, "kind": "depth", "value": depth_count})
            next_depth += 1.0 / p["depth_hz"]
        if due(now, 1.0 / p["free_ray_hz"], next_free):
            free_count += 1
            events.append({"t": now, "kind": "free_ray", "value": free_count})
            next_free += 1.0 / p["free_ray_hz"]
        if due(now, 1.0 / p["odom_hz"], next_odom):
            odom_count += 1
            next_odom += 1.0 / p["odom_hz"]
        if due(now, 1.0 / p["semantic_hz"], next_semantic):
            semantic_count += 1
            semantic_arrival = now + p["semantic_latency_ms"] / 1000.0
            events.append({"t": now, "kind": "semantic", "value": semantic_count})
            next_semantic += 1.0 / p["semantic_hz"]
        if semantic_arrival is not None and not semantic_applied and now >= semantic_arrival:
            semantic_applied = True
            events.append({"t": now, "kind": "semantic_applied", "value": 1})

        if due(now, 1.0 / p["skeleton_hz"], next_skeleton):
            if graph_rebuild_done >= now:
                missed_rebuilds += 1
                events.append({"t": now, "kind": "rebuild_missed", "value": missed_rebuilds})
            else:
                # Deterministic variation reflects the measured 20-100 ms
                # incremental update cost without using random state.
                jitter = p["skeleton_jitter_ms"] * math.sin(1.7 * now + 0.4)
                duration_ms = p["skeleton_base_ms"] + jitter
                duration_ms = max(30.0, duration_ms)
                graph_rebuild_done = now + duration_ms / 1000.0
                rebuilds.append({"start": now, "done": graph_rebuild_done,
                                 "duration_ms": duration_ms})
                events.append({"t": now, "kind": "rebuild_start", "value": duration_ms})
                graph_version += 1
                events.append({"t": graph_rebuild_done, "kind": "rebuild_done", "value": graph_version})
            next_skeleton += 1.0 / p["skeleton_hz"]

        if graph_rebuild_done >= 0.0 and graph_rebuild_done <= now:
            graph_last_update = max(graph_last_update, graph_rebuild_done)

        if due(now, 1.0 / p["planner_hz"], next_planner):
            planner_count += 1
            age = max(0.0, now - graph_last_update)
            max_graph_age = max(max_graph_age, age)
            events.append({"t": now, "kind": "planner", "value": age * 1000.0})
            next_planner += 1.0 / p["planner_hz"]

        if due(now, 1.0 / p["route_hz"], next_route):
            route_count += 1
            selected = "SIDE_SAFE" if semantic_applied else "DIRECT"
            if selected != route:
                route = selected
                if first_turn_time is None:
                    first_turn_time = now
                events.append({"t": now, "kind": "route_turn", "value": selected})
            else:
                events.append({"t": now, "kind": "route", "value": selected})
            next_route += 1.0 / p["route_hz"]

    counts = {
        "depth": depth_count,
        "free_ray": free_count,
        "odom": odom_count,
        "semantic": semantic_count,
        "planner": planner_count,
        "route": route_count,
    }
    expected = {
        "depth": int(round(p["duration_s"] * p["depth_hz"])) + 1,
        "free_ray": int(round(p["duration_s"] * p["free_ray_hz"])) + 1,
        "odom": int(round(p["duration_s"] * p["odom_hz"])) + 1,
        "semantic": int(round(p["duration_s"] * p["semantic_hz"])) + 1,
        "planner": int(round(p["duration_s"] * p["planner_hz"])) + 1,
        "route": int(round(p["duration_s"] * p["route_hz"])) + 1,
    }
    checks = [
        {"name": "stream cadence", "passed": counts == expected,
         "detail": f"counts={counts}, expected={expected}"},
        {"name": "semantic turn is early", "passed": first_turn_time is not None and first_turn_time <= 2.0,
         "detail": f"first_turn_time_s={first_turn_time}"},
        {"name": "graph rebuild has no backlog", "passed": missed_rebuilds == 0,
         "detail": f"missed_rebuilds={missed_rebuilds}"},
        {"name": "planner sees a fresh graph", "passed": max_graph_age <= 0.3,
         "detail": f"max_graph_age_s={max_graph_age:.3f}"},
    ]
    return {
        "parameters": p,
        "counts": counts,
        "expected_counts": expected,
        "checks": checks,
        "first_turn_time_s": first_turn_time,
        "max_graph_age_s": max_graph_age,
        "missed_rebuilds": missed_rebuilds,
        "rebuilds": rebuilds,
        "events": events,
        "route_points": [
            {"x": 0, "y": 0, "label": "start"},
            {"x": 18, "y": 0, "label": "risk ray"},
            {"x": 30, "y": 8 if route == "SIDE_SAFE" else 0, "label": route},
            {"x": 60, "y": 8 if route == "SIDE_SAFE" else 0, "label": "goal"},
        ],
    }


def render_html(result: dict) -> str:
    payload = json.dumps(result, separators=(",", ":"), ensure_ascii=True)
    passed = sum(1 for check in result["checks"] if check["passed"])
    total = len(result["checks"])
    rows = "".join(
        f"<tr><td>{html.escape(check['name'])}</td>"
        f"<td class={'pass' if check['passed'] else 'fail'}>"
        f"{'PASS' if check['passed'] else 'FAIL'}</td>"
        f"<td>{html.escape(check['detail'])}</td></tr>"
        for check in result["checks"]
    )
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>EPIC Online Simulation</title>
<style>
body{{font:14px system-ui,sans-serif;margin:24px;background:#111827;color:#e5e7eb}}
h1{{margin:0 0 6px}} .muted{{color:#9ca3af}} .grid{{display:grid;grid-template-columns:1fr 1fr;gap:16px}}
section{{background:#1f2937;border:1px solid #374151;border-radius:6px;padding:16px;margin-top:16px}}
table{{width:100%;border-collapse:collapse}}td,th{{padding:7px;border-bottom:1px solid #374151;text-align:left}}
.pass{{color:#34d399;font-weight:700}} .fail{{color:#f87171;font-weight:700}} canvas{{width:100%;height:280px;background:#111827}}
svg{{width:100%;height:280px;background:#111827}} code{{color:#93c5fd}}
</style></head><body>
<h1>EPIC Online Timing and Route Simulation</h1>
<div class="muted">Deterministic synthetic run using the current start.sh rates and observed callback latencies.</div>
<section><h2>Contract checks: {passed}/{total}</h2><table><tr><th>Contract</th><th>Result</th><th>Evidence</th></tr>{rows}</table></section>
<div class="grid"><section><h2>Stream timeline</h2><canvas id="timeline" width="900" height="280"></canvas></section>
<section><h2>Selected route</h2><svg id="route" viewBox="0 0 900 280" role="img" aria-label="simulated route"></svg></section></div>
<section><h2>Parameters</h2><pre id="params"></pre></section>
<script>
const result={payload};
document.getElementById('params').textContent=JSON.stringify(result.parameters,null,2);
const canvas=document.getElementById('timeline'),ctx=canvas.getContext('2d');
const W=canvas.width,H=canvas.height,T=result.parameters.duration_s;
const lanes=[['depth','#60a5fa'],['free_ray','#34d399'],['semantic','#fbbf24'],['rebuild_start','#f97316'],['planner','#c084fc'],['route_turn','#f43f5e']];
ctx.fillStyle='#111827';ctx.fillRect(0,0,W,H);ctx.font='12px system-ui';
lanes.forEach((lane,i)=>{{const y=28+i*38;ctx.fillStyle='#9ca3af';ctx.fillText(lane[0],8,y+4);ctx.strokeStyle='#374151';ctx.beginPath();ctx.moveTo(90,y);ctx.lineTo(W-10,y);ctx.stroke();ctx.fillStyle=lane[1];result.events.filter(e=>e.kind===lane[0]).forEach(e=>{{ctx.beginPath();ctx.arc(90+e.t/T*(W-100),y,4,0,Math.PI*2);ctx.fill()}})}});
ctx.fillStyle='#9ca3af';ctx.fillText('0 s',90,H-8);ctx.fillText(T.toFixed(1)+' s',W-45,H-8);
const svg=document.getElementById('route'),pts=result.route_points;const sx=x=>80+x*12,sy=y=>140-y*10;
for(let i=1;i<pts.length;i++){{const a=pts[i-1],b=pts[i];const line=document.createElementNS('http://www.w3.org/2000/svg','line');line.setAttribute('x1',sx(a.x));line.setAttribute('y1',sy(a.y));line.setAttribute('x2',sx(b.x));line.setAttribute('y2',sy(b.y));line.setAttribute('stroke',b.label==='DIRECT'?'#f87171':'#34d399');line.setAttribute('stroke-width','8');svg.appendChild(line)}}
pts.forEach(p=>{{const c=document.createElementNS('http://www.w3.org/2000/svg','circle');c.setAttribute('cx',sx(p.x));c.setAttribute('cy',sy(p.y));c.setAttribute('r','7');c.setAttribute('fill','#f9fafb');svg.appendChild(c);const t=document.createElementNS('http://www.w3.org/2000/svg','text');t.setAttribute('x',sx(p.x)+8);t.setAttribute('y',sy(p.y)-8);t.setAttribute('fill','#e5e7eb');t.textContent=p.label;svg.appendChild(t)}});
</script></body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=Path("log_event/epic_online_sim.json"))
    parser.add_argument("--html", type=Path, default=Path("log_event/epic_online_sim_report.html"))
    args = parser.parse_args()
    result = simulate()
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.html.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    args.html.write_text(render_html(result), encoding="utf-8")
    failed = [check for check in result["checks"] if not check["passed"]]
    print(f"simulation: {len(result['checks']) - len(failed)}/{len(result['checks'])} checks passed")
    print(f"json: {args.json}")
    print(f"html: {args.html}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
