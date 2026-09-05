#!/usr/bin/env python3
"""Build an HTML viewer for goal-aware GCN samples and A* paths."""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
from heapq import heappop, heappush

import torch
from PIL import Image

from train import build_model, session_groups
from collect_privileged_dataset import astar as grid_astar, build_occupancy


def lookahead_point(path, distance_m):
    if len(path) < 2:
        return None
    traveled = 0.0
    for current, following in zip(path, path[1:]):
        segment = math.dist(current, following)
        if traveled + segment >= distance_m:
            return following
        traveled += segment
    return None


def body_relative_column(point, position, yaw):
    """Map a world point to the same five body-relative columns as the GT builder."""
    dx = point[0] - position[0]
    dy = point[1] - position[1]
    angle = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                       math.cos(yaw) * dx + math.sin(yaw) * dy)
    return max(0, min(4, int(round(2.0 - angle / math.radians(20.0)))))


def read_image(path):
    if not path or not os.path.isfile(path):
        return ""
    try:
        image = Image.open(path).convert("RGB")
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode()
    except Exception:
        return ""


def read_pcd(path, limit=1600):
    points = []
    data = False
    if not path or not os.path.isfile(path):
        return points
    with open(path, encoding="ascii", errors="ignore") as stream:
        for line in stream:
            if line.lower().startswith("data"):
                data = True
                continue
            if data and len(points) < limit:
                fields = line.split()
                if len(fields) >= 3:
                    try:
                        points.append([float(fields[0]), float(fields[1]), float(fields[2])])
                    except ValueError:
                        pass
    return points


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return [vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx]


def graph_path(sample, position, orientation, goal):
    real_count = min(sample["frontier_index"].tolist())
    nodes = [[float(row[0]) * 25, float(row[1]) * 80]
             for row in sample["x"][:real_count]]
    edges = [(a, b) for a, b in sample["edge_index"][:, ::2].t().tolist()
             if a < real_count and b < real_count and a != b]
    adjacency = {i: [] for i in range(real_count)}
    for a, b in edges:
        weight = math.dist(nodes[a], nodes[b])
        adjacency[a].append((b, weight))
        adjacency[b].append((a, weight))
    start = min(range(real_count), key=lambda i: math.dist(nodes[i], position[:2]))
    goal_id = min(range(real_count), key=lambda i: math.dist(nodes[i], goal[:2]))
    distance = {i: float("inf") for i in range(real_count)}
    next_hop = {}
    distance[goal_id] = 0.0
    heap = [(0.0, goal_id)]
    while heap:
        cost, current = heappop(heap)
        if cost != distance[current]:
            continue
        for neighbor, weight in adjacency[current]:
            candidate = cost + weight
            if candidate < distance[neighbor]:
                distance[neighbor] = candidate
                next_hop[neighbor] = current
                heappush(heap, (candidate, neighbor))
    yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
                     1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
    costs = [float("inf")] * 5
    chosen = {}
    limit = math.radians(50)
    for neighbor, weight in adjacency[start]:
        dx, dy = nodes[neighbor][0] - nodes[start][0], nodes[neighbor][1] - nodes[start][1]
        angle = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                           math.cos(yaw) * dx + math.sin(yaw) * dy)
        if abs(angle) > limit or not math.isfinite(distance[neighbor]):
            continue
        column = max(0, min(4, int(round((angle / limit + 1) * 2))))
        total = weight + distance[neighbor]
        if total < costs[column]:
            costs[column] = total
            chosen[column] = neighbor
    target = min(range(5), key=lambda column: costs[column])
    if target not in chosen:
        return nodes, edges, [start], costs, yaw, start, goal_id, chosen
    first = chosen[target]
    path = [start, first]
    current = first
    while current != goal_id and current in next_hop:
        current = next_hop[current]
        path.append(current)
    return nodes, edges, path, costs, yaw, start, goal_id, chosen


def snapshot_route(snapshot, position, orientation, goal, target):
    marker = next((m for m in snapshot.get("markers", [])
                   if m.get("ns") == "scalenav_skeleton_nodes"), None)
    edge_marker = next((m for m in snapshot.get("markers", [])
                        if m.get("ns") == "scalenav_skeleton_edges"), None)
    if not marker or not marker.get("points"):
        return [], float("inf"), None
    nodes = [[float(p[0]), float(p[1])] for p in marker["points"]]
    edges = set()
    points = edge_marker.get("points", []) if edge_marker else []
    for i in range(0, len(points) - 1, 2):
        a = min(range(len(nodes)), key=lambda j: math.dist(points[i][:2], nodes[j]))
        b = min(range(len(nodes)), key=lambda j: math.dist(points[i + 1][:2], nodes[j]))
        if a != b: edges.add((a, b))
    adjacency = {i: [] for i in range(len(nodes))}
    for a, b in edges:
        w = math.dist(nodes[a], nodes[b]); adjacency[a].append((b, w)); adjacency[b].append((a, w))
    start = min(range(len(nodes)), key=lambda i: math.dist(nodes[i], position[:2]))
    goal_id = min(range(len(nodes)), key=lambda i: math.dist(nodes[i], goal[:2]))
    dist = {i: float("inf") for i in range(len(nodes))}; parent = {}; dist[goal_id] = 0.0; heap = [(0.0, goal_id)]
    while heap:
        d, u = heappop(heap)
        if d != dist[u]: continue
        for v, w in adjacency[u]:
            if d + w < dist[v]: dist[v] = d + w; parent[v] = u; heappush(heap, (dist[v], v))
    yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
                     1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
    limit = math.radians(50.0)
    candidates = []
    for neighbor, weight in adjacency[start]:
        dx, dy = nodes[neighbor][0] - nodes[start][0], nodes[neighbor][1] - nodes[start][1]
        angle = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                           math.cos(yaw) * dx + math.sin(yaw) * dy)
        column = max(0, min(4, int(round((angle / limit + 1.0) * 2.0))))
        if abs(angle) <= limit and column == target and math.isfinite(dist[neighbor]):
            candidates.append((weight + dist[neighbor], neighbor))
    if not candidates: return [], float("inf"), None
    route_cost, first = min(candidates)
    route = [start, first]
    while route[-1] != goal_id:
        if route[-1] not in parent: return nodes, float("inf"), None
        route.append(parent[route[-1]])
    return [nodes[i] for i in route], route_cost, nodes[first]


HTML = r'''<!doctype html><meta charset="utf-8"><title>ScaleNav goal-aware GCN</title>
<style>body{margin:0;background:#10141a;color:#e8edf3;font:14px system-ui}header{padding:12px 16px;border-bottom:1px solid #39434f;display:flex;gap:10px;align-items:center;flex-wrap:wrap}button,select{background:#202832;color:#fff;border:1px solid #536171;padding:6px 10px;border-radius:4px}main{display:grid;grid-template-columns:minmax(550px,1fr) 370px;gap:12px;padding:12px}.panel{background:#171d25;border:1px solid #39434f;padding:10px;border-radius:4px}canvas{width:100%;background:#111820;border:1px solid #39434f}.sensors{display:grid;grid-template-columns:1fr 1fr;gap:8px}.sensors img{width:100%;background:#0b1015}.stats{display:grid;grid-template-columns:1fr 1fr;gap:6px}.stats div{background:#202832;padding:7px}.label{color:#9eacbb;font-size:12px}.value{font-size:17px;font-weight:600}.hint{color:#9eacbb;font-size:12px;line-height:1.5}.row{padding:5px 0;border-bottom:1px solid #303944}@media(max-width:900px){main{grid-template-columns:1fr}}</style>
<header><b>ScaleNav goal-aware GCN</b><button id="prev">上一帧</button><button id="next">下一帧</button><select id="sample"></select><select id="session"><option value="">全部 Session</option></select><span id="count"></span></header>
<main><section class="panel"><canvas id="map" width="960" height="650"></canvas><div class="hint" style="display:flex;gap:14px;flex-wrap:wrap;padding-top:7px"><span style="color:#ef5b65">● 飞机</span><span style="color:#45d483">● GT 35 m 点</span><span style="color:#f4c95d">● mission goal</span><span style="color:#ba7cff">━ GCN 选择方向</span><span style="color:#f0a04a">━ Planner 选择方向</span><span style="color:#36c5d8">━ 点云栅格 A* 路径</span></div><h3>RGB / Depth / Heatmap</h3><div class="sensors"><img id="rgb"><img id="depth"><img id="semantic"></div><p class="hint">灰线=当前帧 skeleton；蓝灰点=当前帧局部点云。青线来自跨 session 统一 world 点云占据栅格；绿色点是沿完整 A* 路径累计 35 m 的 GT 方向点。</p></section><aside class="panel"><div id="title"></div><div class="stats" id="stats"></div><h3>五列</h3><div id="cols"></div><p class="hint">GT 将统一点云地图 A* 的 35 m look-ahead 方向映射到左前、左、中、右、右前五列。GCN 分数是五列 softmax 概率。</p></aside></main>
<script>
const D=__DATA__,S=document.querySelector('#sample'),F=document.querySelector('#session'),M=document.querySelector('#map'),C=M.getContext('2d');
[...new Set(D.samples.map(s=>s.session))].sort().forEach(s=>F.add(new Option(s,s)));
function list(){return D.samples.filter(s=>!F.value||s.session===F.value)}
function fill(){S.innerHTML='';list().forEach((s,i)=>S.add(new Option(i+': '+s.session+' seq='+s.seq,i)));document.querySelector('#count').textContent=list().length+' / '+D.total;draw()}
function step(d){let n=list().length;if(n){S.value=(+S.value+d+n)%n;draw()}}
function draw(){const a=list(),s=a[+S.value||0];if(!s)return;['rgb','depth','semantic'].forEach(k=>document.querySelector('#'+k).src=s[k]||'');let X=s.nodes.map(p=>p[0]),Y=s.nodes.map(p=>p[1]);s.cloud.forEach(p=>{X.push(p[0]);Y.push(p[1])});(s.global_path||[]).forEach(p=>{X.push(p[0]);Y.push(p[1])});X.push(s.goal[0]);Y.push(s.goal[1]);let ax=Math.min(...X),bx=Math.max(...X),ay=Math.min(...Y),by=Math.max(...Y),q=38,k=Math.min((M.width-2*q)/(bx-ax||1),(M.height-2*q)/(by-ay||1)),px=v=>q+(v-ax)*k,py=v=>M.height-q-(v-ay)*k;C.clearRect(0,0,M.width,M.height);C.fillStyle='#66788b';s.cloud.forEach(p=>{C.fillRect(px(p[0]),py(p[1]),2,2)});C.strokeStyle='#3b4755';C.lineWidth=1;s.edges.forEach(e=>{C.beginPath();C.moveTo(px(s.nodes[e[0]][0]),py(s.nodes[e[0]][1]));C.lineTo(px(s.nodes[e[1]][0]),py(s.nodes[e[1]][1]));C.stroke()});C.strokeStyle='#36c5d8';C.lineWidth=5;C.beginPath();(s.global_path||[]).forEach((p,j)=>j?C.lineTo(px(p[0]),py(p[1])):C.moveTo(px(p[0]),py(p[1])));C.stroke();C.fillStyle='#f4c95d';C.beginPath();C.arc(px(s.goal[0]),py(s.goal[1]),9,0,7);C.fill();C.strokeStyle='#ef5b65';C.lineWidth=3;let u=px(s.position[0]),v=py(s.position[1]),hx=Math.cos(s.yaw),hy=Math.sin(s.yaw);C.beginPath();C.moveTo(u,v);C.lineTo(u+hx*32,v-hy*32);C.stroke();C.fillStyle='#ef5b65';C.beginPath();C.arc(u,v,7,0,7);C.fill();if(s.gt_point){C.fillStyle='#45d483';C.strokeStyle='#0b1015';C.lineWidth=2;C.beginPath();C.arc(px(s.gt_point[0]),py(s.gt_point[1]),8,0,7);C.fill();C.stroke();C.fillStyle='#45d483';C.font='bold 12px system-ui';C.fillText('GT 35m',px(s.gt_point[0])+9,py(s.gt_point[1])-9)}let rays=[40,20,0,-20,-40].map(d=>{let z=s.yaw+d*Math.PI/180;return [s.position[0]+Math.cos(z)*18,s.position[1]+Math.sin(z)*18]});rays.forEach((p,i)=>{C.strokeStyle=i===s.model?'#ba7cff':i===s.target?'#45d483':i===s.planner?'#f0a04a':'#536171';C.lineWidth=i===s.model?4:2;C.beginPath();C.moveTo(u,v);C.lineTo(px(p[0]),py(p[1]));C.stroke()});document.querySelector('#title').innerHTML='<b>'+s.session+'</b> seq='+s.seq;document.querySelector('#stats').innerHTML=[['GCN 输出',s.model],['Map 真值列',s.target],['GT 35m','A* look-ahead'],['Planner',s.planner],['A* 路径长度',s.path_cost.toFixed(2)+' m'],['A* 节点数',(s.global_path||[]).length],['点云数',s.cloud.length],['飞机 yaw',(s.yaw*180/Math.PI).toFixed(1)+'°'],['GCN 命中',s.model===s.target?'是':'否']].map(v=>'<div><span class="label">'+v[0]+'</span><br><span class="value">'+v[1]+'</span></div>').join('');document.querySelector('#cols').innerHTML=s.scores.map((v,i)=>'<div class="row">column '+i+'　GCN <b style="color:#ba7cff">'+v.toFixed(3)+'</b>　路径代价 '+(s.costs[i]===null?'不可达':s.costs[i].toFixed(2))+(i===s.model?'　← GCN':'')+(i===s.target?'　← 真值':'')+(i===s.planner?'　← Planner':'')+'</div>').join('')}
F.onchange=fill;S.onchange=draw;prev.onclick=()=>step(-1);next.onclick=()=>step(1);document.onkeydown=e=>{if(e.key==='ArrowLeft')step(-1);if(e.key==='ArrowRight')step(1)};fill();
</script>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='train_gcn/dataset_goal.pt')
    parser.add_argument('--model', default='train_gcn/frontier_gcn_goal.pt')
    parser.add_argument('--output', default='train_gcn/goal_dataset_viewer.html')
    parser.add_argument('--max-samples', type=int, default=300)
    parser.add_argument('--occupancy-cache', default='')
    args = parser.parse_args()
    bundle = torch.load(args.dataset, weights_only=False)
    raw = bundle['samples']
    privileged = str(bundle.get('schema', '')).startswith('scalenav_gcn_privileged')
    chosen = raw if args.max_samples <= 0 else raw[:args.max_samples]
    checkpoint = torch.load(args.model, map_location='cpu', weights_only=False)
    model = build_model(int(chosen[0]['x'].shape[1]), checkpoint.get('architecture', 'legacy'))
    model.load_state_dict(checkpoint['model'])
    model.eval(); predictions = {}; probabilities = {}
    for _, group in session_groups(chosen).items():
        hidden = None; previous = -1
        for sample in group:
            with torch.no_grad(): logits, hidden = model(sample, hidden, previous)
            predictions[id(sample)] = int(logits.argmax()); probabilities[id(sample)] = torch.softmax(logits, 0).tolist(); previous = predictions[id(sample)]; hidden = hidden.detach() if hidden is not None else None
    global_blocked = None
    global_resolution = 0.75
    if args.occupancy_cache:
        occupancy_bundle = torch.load(args.occupancy_cache, weights_only=False)
        global_blocked = set(map(tuple, occupancy_bundle['blocked']))
        global_resolution = float(occupancy_bundle.get('resolution', global_resolution))
    cache = {}; occupancy_cache = {}; output = []; gt_visual_mismatches = 0
    for sample in chosen:
        root = os.path.join('log_scalenav', sample['session']); records = cache.setdefault(root, [])
        if not records:
            with open(os.path.join(root, 'index.jsonl'), encoding='utf-8') as stream:
                for line in stream:
                    try: records.append(json.loads(line, parse_constant=float))
                    except json.JSONDecodeError: pass
        timing = next((r for r in records if r.get('kind') == 'timing' and r.get('seq') == sample['seq']), {}); stamp = timing.get('stamp_ns', 0)
        odom = min((r for r in records if r.get('kind') == 'odom'), key=lambda r: abs(r['stamp_ns'] - stamp), default={}); position = odom.get('data', {}).get('position', sample['position']); orientation = odom.get('data', {}).get('orientation', [0, 0, 0, 1]); goal_record = next((r for r in records if r.get('kind') == 'goal'), {}); goal = goal_record.get('data', {}).get('position', [0, 140, 1.6])
        final_graph = max((r for r in records if r.get('kind') == 'graph' and r.get('file')), key=lambda r: r.get('stamp_ns', 0), default=None)
        final_snapshot = {}
        if final_graph:
            try:
                with open(os.path.join(root, final_graph['file']), encoding='utf-8') as stream: final_snapshot = json.load(stream)
            except (OSError, json.JSONDecodeError): pass
        assets = {}
        for kind in ('rgb', 'depth', 'semantic', 'pointcloud'):
            candidates = [r for r in records if r.get('kind') == kind and r.get('file')]
            if candidates: assets[kind] = os.path.join(root, min(candidates, key=lambda r: abs(r['stamp_ns'] - stamp))['file'])
        nodes, edges, path, costs, yaw, start, goal_id, chosen = graph_path(sample, position, orientation, goal); cloud = []
        global_path, global_cost, gt_point = snapshot_route(final_snapshot, position, orientation, goal, sample['target'])
        if privileged:
            if root not in occupancy_cache:
                try:
                    resolution = global_resolution
                    blocked = global_blocked if global_blocked is not None else build_occupancy(root, records, resolution, 1.2, 100)
                    odom_positions = [r.get('data', {}).get('position', [0, 0, 0]) for r in records if r.get('kind') == 'odom']
                    xy = [(int(math.floor(goal[0] / resolution)), int(math.floor(goal[1] / resolution)))]
                    xy += [(int(math.floor(p[0] / resolution)), int(math.floor(p[1] / resolution))) for p in odom_positions]
                    margin = max(8, math.ceil(40.0 / resolution))
                    bounds = (min(p[0] for p in xy) - margin, max(p[0] for p in xy) + margin,
                              min(p[1] for p in xy) - margin, max(p[1] for p in xy) + margin)
                    occupancy_cache[root] = (blocked, bounds, resolution)
                except (OSError, ValueError):
                    occupancy_cache[root] = None
            if occupancy_cache[root] is not None:
                blocked, bounds, resolution = occupancy_cache[root]
                start_cell = (int(math.floor(position[0] / resolution)), int(math.floor(position[1] / resolution)))
                goal_cell = (int(math.floor(goal[0] / resolution)), int(math.floor(goal[1] / resolution)))
                cells = grid_astar(start_cell, goal_cell, blocked, bounds)
                if cells:
                    global_path = [[(x + 0.5) * resolution, (y + 0.5) * resolution] for x, y in cells]
                    global_cost = sum(math.dist(global_path[i], global_path[i + 1]) for i in range(len(global_path) - 1))
                    gt_point = lookahead_point(global_path, 35.0)
                    if gt_point is not None and body_relative_column(
                            gt_point, position, yaw) != sample['target']:
                        gt_visual_mismatches += 1
        for point in read_pcd(assets.get('pointcloud')):
            rotated = rotate(orientation, point); cloud.append([rotated[0] + position[0], rotated[1] + position[1], rotated[2] + position[2]])
        gt_node = path[1] if len(path) > 1 else None
        output.append({'nodes': nodes, 'edges': edges, 'path': path, 'global_path': global_path, 'gt_point': gt_point, 'gt_node': gt_node, 'path_cost': global_cost, 'costs': [None if not math.isfinite(v) else round(v, 2) for v in costs], 'goal': goal[:2], 'cloud': cloud, 'model': predictions[id(sample)], 'scores': probabilities[id(sample)], 'target': sample['target'], 'planner': sample['planner_target'], 'safe': sample['safe_columns'].tolist(), 'session': sample['session'], 'seq': sample['seq'], 'position': position, 'yaw': yaw})
    # Keep the static obstacle footprint once at page level instead of
    # duplicating thousands of points into every sample record.
    map_obstacles = []
    if global_blocked is not None:
        map_obstacles = [[(x + 0.5) * global_resolution, (y + 0.5) * global_resolution]
                         for x, y in global_blocked]
    page = HTML.replace('__DATA__', json.dumps({'total': len(raw), 'samples': output,
                                                'map_obstacles': map_obstacles}, separators=(',', ':')))
    page = page.replace("let X=s.nodes.map(p=>p[0]),Y=s.nodes.map(p=>p[1]);",
                        "let X=s.nodes.map(p=>p[0]),Y=s.nodes.map(p=>p[1]);D.map_obstacles.forEach(p=>{X.push(p[0]);Y.push(p[1])});")
    page = page.replace("C.clearRect(0,0,M.width,M.height);C.fillStyle='#66788b';",
                        "C.clearRect(0,0,M.width,M.height);C.fillStyle='#8d647d';D.map_obstacles.forEach(p=>{C.fillRect(px(p[0])-1,py(p[1])-1,3,3)});C.fillStyle='#66788b';")
    page = page.replace('点云栅格 A* 路径</span>', '点云栅格 A* 路径</span><span style="color:#8d647d">■ 静态障碍膨胀栅格</span>')
    page = page.replace('蓝灰点=当前帧局部点云。', '蓝灰点=当前帧局部点云；紫灰点=完整静态 Map2 障碍膨胀栅格。')
    path = os.path.abspath(args.output); os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as stream: stream.write(page)
    print(f'wrote={path} embedded={len(output)} total={len(raw)} '
          f'gt_visual_mismatches={gt_visual_mismatches}')


if __name__ == '__main__': main()
