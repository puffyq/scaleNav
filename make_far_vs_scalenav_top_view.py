import json
from pathlib import Path

root = Path('/mnt/code/lab/yopo/OpenSeek')
out = root / 'scalenav_ws/results/far_vs_scalenav_20260912'
out.mkdir(parents=True, exist_ok=True)
sources = {
    'ScaleNav (best)': (root / 'log_scalenav/session_20260912_230232_120', '#16a34a'),
    'FAR (worst)': (root / 'log_scalenav/session_20260912_220910_714', '#dc2626'),
}
traces = []
for label, (session, color) in sources.items():
    points = []
    for line in (session / 'index.jsonl').open(encoding='utf-8'):
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get('kind') != 'odom':
            continue
        p = record['data']['position']
        if not points or sum((p[i] - points[-1][i]) ** 2 for i in range(2)) > 0.25 ** 2:
            points.append((float(p[0]), float(p[1])))
    traces.append((label, color, points))
    (out / (label.split()[0].lower() + '.csv')).write_text('x,y\n' + ''.join(f'{x:.6f},{y:.6f}\n' for x,y in points), encoding='utf-8')
width, height, margin = 1400, 1100, 100
all_points = [p for _,_,points in traces for p in points]
min_x, max_x = min(p[0] for p in all_points), max(p[0] for p in all_points)
min_y, max_y = min(p[1] for p in all_points), max(p[1] for p in all_points)
scale = min((width-2*margin)/(max_x-min_x), (height-2*margin)/(max_y-min_y))
def project(p): return margin+(p[0]-min_x)*scale, height-margin-(p[1]-min_y)*scale
elements = []
for label, color, points in traces:
    sampled = points[::max(1, len(points)//900)]
    path = ' '.join(f'{project(p)[0]:.1f},{project(p)[1]:.1f}' for p in sampled)
    sx,sy = project(points[0]); gx,gy = project(points[-1])
    elements.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="3"/>')
    elements.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="8" fill="{color}"/>')
    elements.append(f'<polygon points="{gx:.1f},{gy-12:.1f} {gx+11:.1f},{gy+9:.1f} {gx-11:.1f},{gy+9:.1f}" fill="{color}"/>')
elements += ['<text x="100" y="45" font-size="30" font-family="sans-serif">RealCitySF top view: FAR vs ScaleNav</text>', '<text x="100" y="1070" font-size="20" font-family="sans-serif">green = ScaleNav best (1244.88 m) | red = FAR worst (1342.86 m) | circle=start, triangle=goal</text>']
svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}"><rect width="100%" height="100%" fill="white"/><g>{"".join(elements)}</g></svg>'
output = out / 'far_vs_scalenav_top_view.svg'
output.write_text(svg, encoding='utf-8')
print(output)
