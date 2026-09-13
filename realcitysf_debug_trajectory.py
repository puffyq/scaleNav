import csv
from pathlib import Path
import unreal

MAP = "/Game/RealCitySF/Maps/San_Francisco_sunny_01"
DATA = Path("/mnt/code/lab/yopo/OpenSeek/scalenav_ws/results/far_vs_scalenav_20260912")
unreal.EditorLevelLibrary.load_level(MAP)
world = unreal.EditorLevelLibrary.get_editor_world()
colors = [(16, 180, 80, 255), (220, 30, 30, 255)]
all_points = []
for trial, color in (("scalenav", colors[0]), ("far", colors[1])):
    rows = []
    with (DATA / f"{trial}.csv").open(encoding="utf-8") as stream:
        rows = [(float(r["x"]) * 100, float(r["y"]) * 100, 160) for r in csv.DictReader(stream)]
    rows = rows[::max(1, len(rows) // 250)]
    all_points.extend(rows)
    for start, end in zip(rows, rows[1:]):
        unreal.SystemLibrary.draw_debug_line(world, unreal.Vector(*start), unreal.Vector(*end), unreal.LinearColor(color[0]/255, color[1]/255, color[2]/255, 1), 3600.0, 12.0)
if all_points:
    min_x, max_x = min(p[0] for p in all_points), max(p[0] for p in all_points)
    min_y, max_y = min(p[1] for p in all_points), max(p[1] for p in all_points)
    center = unreal.Vector((min_x + max_x) / 2, (min_y + max_y) / 2, 0)
    span = max(max_x - min_x, max_y - min_y)
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(unreal.Vector(center.x, center.y, max(10000, span * 0.72)), unreal.Rotator(-90, 0, 0))
unreal.log("FAR vs ScaleNav debug trajectory drawn in top view")

