import csv
from pathlib import Path

root = Path('/mnt/code/lab/yopo/OpenSeek/scalenav_ws/results/far_20260912_220907')
traces = []
for trial, color in ((1, '#00a6ff'), (2, '#ff6b35')):
    with (root / f'trial_{trial}.csv').open(encoding='utf-8') as stream:
        points = [(float(row['x']), float(row['y'])) for row in csv.DictReader(stream)]
    traces.append((trial, color, points))
width, height, margin = 1200, 1000, 90
all_points = [p for _, _, points in traces for p in points]
min_x, max_x = min(p[0] for p in all_points), max(p[0] for p in all_points)
min_y, max_y = min(p[1] for p in all_points), max(p[1] for p in all_points)
scale = min((width-2*margin)/(max_x-min_x), (height-2*margin)/(max_y-min_y))
def project(point):
    return margin+(point[0]-min_x)*scale, height-margin-(point[1]-min_y)*scale
lines = []
for trial, color, points in traces:
    sampled = points[::max(1, len(points)//600)]
    path = ' '.join(f'{project(p)[0]:.1f},{project(p)[1]:.1f}' for p in sampled)
    sx, sy = project(points[0]); gx, gy = project(points[-1])
    lines.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="2"/>')
    lines.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="{color}"/>')
    lines.append(f'<polygon points="{gx:.1f},{gy-10:.1f} {gx+9:.1f},{gy+8:.1f} {gx-9:.1f},{gy+8:.1f}" fill="{color}"/>')
lines.append('<text x="90" y="45" font-size="28" font-family="sans-serif">RealCitySF top view: FAR successful trajectories</text>')
lines.append('<text x="90" y="980" font-size="18" font-family="sans-serif">circle=start, triangle=goal | blue=trial 1, orange=trial 2</text>')
svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}"><rect width="100%" height="100%" fill="white"/><g>{"".join(lines)}</g></svg>'
output = root / 'far_trajectories_top_view.svg'
output.write_text(svg, encoding='utf-8')
print(output)
