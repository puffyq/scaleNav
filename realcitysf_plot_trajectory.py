import csv
import math
from pathlib import Path
import unreal


MAP = "/Game/RealCitySF/Maps/San_Francisco_sunny_01"
DATA = Path("/mnt/code/lab/yopo/OpenSeek/scalenav_ws/results/far_20260912_220907")
unreal.EditorLevelLibrary.load_level(MAP)

for actor in unreal.EditorLevelLibrary.get_all_level_actors():
    if actor.get_actor_label().startswith("FAR_20260912_Trajectory"):
        unreal.EditorLevelLibrary.destroy_actor(actor)

all_points = []
for trial in (1, 2):
    rows = []
    with (DATA / f"trial_{trial}.csv").open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            point = unreal.Vector(float(row["x"]) * 100.0, float(row["y"]) * 100.0, float(row["z"]) * 100.0)
            rows.append(point)
            all_points.append(point)
    if len(rows) < 2:
        continue
    mesh = unreal.load_asset("/Engine/BasicShapes/Cube")
    for index in range(0, len(rows) - 1, 50):
        start = rows[index]
        end = rows[min(index + 10, len(rows) - 1)]
        delta = end - start
        length = math.sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z)
        if length < 1.0:
            continue
        midpoint = (start + end) * 0.5
        horizontal = math.sqrt(delta.x * delta.x + delta.y * delta.y)
        rotation = unreal.Rotator(
            math.degrees(math.atan2(-delta.z, horizontal)),
            math.degrees(math.atan2(delta.y, delta.x)),
            0.0)
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor, midpoint, rotation)
        actor.set_actor_label(f"FAR_20260912_Trajectory_Trial{trial}_{index:05d}")
        component = actor.get_static_mesh_component()
        component.set_static_mesh(mesh)
        component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        actor.set_actor_scale3d(unreal.Vector(length / 100.0, 0.035, 0.035))

if all_points:
    min_x = min(point.x for point in all_points)
    max_x = max(point.x for point in all_points)
    min_y = min(point.y for point in all_points)
    max_y = max(point.y for point in all_points)
    center = unreal.Vector((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, 0.0)
    span = max(max_x - min_x, max_y - min_y)
    camera_location = unreal.Vector(center.x, center.y, max(10000.0, span * 0.72))
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(camera_location, unreal.Rotator(-90.0, 0.0, 0.0))

unreal.EditorLevelLibrary.save_current_level()
unreal.log("FAR trajectory splines and top view saved")
