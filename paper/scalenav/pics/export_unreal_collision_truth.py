#!/usr/bin/env python3
"""Export a horizontal collision-truth occupancy map from an Unreal level.

Run this script through UnrealEditor-Cmd's PythonScript commandlet.  It queries
the level collision scene directly; no AirSim vehicle or camera is moved.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import unreal


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--level", default="/Game/FlyingCPP/Maps/FlyingExampleMapV4")
    parser.add_argument("--output", required=True)
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--x", type=float, nargs=2, default=(-50.0, 50.0))
    parser.add_argument("--y", type=float, nargs=2, default=(-10.0, 150.0))
    parser.add_argument("--z", type=float, nargs=2, default=(0.6, 2.6))
    parser.add_argument("--pie", action="store_true",
                        help="query the currently running PIE world")
    parser.add_argument("--game", action="store_true",
                        help="query a standalone UnrealEditor -game world")
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--api-only", action="store_true")
    return parser.parse_args()


def actor_summary(actor: unreal.Actor) -> dict:
    location = actor.get_actor_location()
    return {
        "label": actor.get_actor_label(),
        "class": actor.get_class().get_name(),
        "location_cm": [location.x, location.y, location.z],
        "component_classes": sorted({
            component.get_class().get_name()
            for component in actor.get_components_by_class(unreal.ActorComponent)
        }),
    }


def main() -> None:
    args = parse_args()
    if args.resolution <= 0.0:
        raise ValueError("resolution must be positive")

    if args.api_only:
        print(json.dumps({
            "world_api": sorted(name for name in dir(unreal) if "world" in name.lower()),
            "object_api": sorted(name for name in dir(unreal)
                                 if "object" in name.lower() or "iterator" in name.lower()),
        }))
        return

    if args.game:
        worlds = [item for item in unreal.ObjectIterator(unreal.World)]
        game_worlds = [item for item in worlds if item.get_name() == "FlyingExampleMapV4"]
        if not game_worlds:
            raise RuntimeError(f"no Map4 game world among {[item.get_name() for item in worlds]}")
        actor_sets = [
            list(unreal.GameplayStatics.get_all_actors_of_class(item, unreal.Actor))
            for item in game_worlds
        ]
        world_index = max(range(len(game_worlds)), key=lambda index: len(actor_sets[index]))
        world = game_worlds[world_index]
        actors = actor_sets[world_index]
    elif args.pie:
        worlds = list(unreal.EditorLevelLibrary.get_pie_worlds(False))
        if not worlds:
            raise RuntimeError("no running PIE world")
        world = worlds[0]
        actors = list(unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor))
    else:
        if not unreal.EditorLevelLibrary.load_level(args.level):
            raise RuntimeError(f"failed to load level: {args.level}")
        world = unreal.EditorLevelLibrary.get_editor_world()
        actors = list(unreal.EditorLevelLibrary.get_all_level_actors())
    summaries = [actor_summary(actor) for actor in actors]
    interesting = [
        item for item in summaries
        if any(token in item["class"].lower() or token in item["label"].lower()
               for token in ("playerstart", "landscape", "foliage", "pcg", "power"))
    ]
    print(json.dumps({
        "level": args.level,
        "world": world.get_name(),
        "actor_count": len(actors),
        "actor_classes": {
            class_name: sum(item["class"] == class_name for item in summaries)
            for class_name in sorted({item["class"] for item in summaries})
        },
        "interesting_actors": interesting,
    }, ensure_ascii=True))
    if args.inspect_only:
        return

    player_starts = [
        actor for actor in actors
        if isinstance(actor, unreal.PlayerStart)
    ]
    if len(player_starts) != 1:
        raise RuntimeError(f"expected one PlayerStart, found {len(player_starts)}")
    origin = player_starts[0].get_actor_transform()
    origin_location = origin.translation
    origin_yaw = math.radians(origin.rotation.rotator().yaw)

    # Convert world_enu (x, y, z up) into UE centimetres.  AirSim global NED
    # uses UE X/Y axes directly; the logged ENU-like frame swaps horizontal
    # axes and flips only vertical: ENU x = UE Y, ENU y = UE X.  PlayerStart
    # yaw sets the initial vehicle attitude, not the global NED axes.
    def to_ue(x_enu: float, y_enu: float, z_enu: float) -> unreal.Vector:
        return unreal.Vector(
            origin_location.x + y_enu * 100.0,
            origin_location.y + x_enu * 100.0,
            origin_location.z + z_enu * 100.0,
        )

    object_types = [
        unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY1,
        unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY2,
        unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY3,
        unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY4,
        unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY5,
        unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY6,
    ]
    ignored = [actor for actor in actors if isinstance(actor, unreal.Pawn)]
    half_xy = args.resolution * 50.0
    half_z = (args.z[1] - args.z[0]) * 50.0
    center_z = (args.z[0] + args.z[1]) * 0.5
    nx = int(math.ceil((args.x[1] - args.x[0]) / args.resolution))
    ny = int(math.ceil((args.y[1] - args.y[0]) / args.resolution))
    occupied = []
    for iy in range(ny):
        y = args.y[0] + (iy + 0.5) * args.resolution
        for ix in range(nx):
            x = args.x[0] + (ix + 0.5) * args.resolution
            components = unreal.SystemLibrary.box_overlap_components(
                world,
                to_ue(x, y, center_z),
                unreal.Vector(half_xy, half_xy, half_z),
                object_types,
                unreal.PrimitiveComponent,
                ignored,
            ) or []
            components = [
                component for component in components
                if component.get_owner() not in ignored
            ]
            if components:
                occupied.append([x, y, center_z])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii") as stream:
        stream.write(
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(occupied)}\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n"
        )
        for point in occupied:
            stream.write(f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f}\n")
    metadata = {
        "source": "Unreal collision scene box-overlap truth",
        "level": args.level,
        "frame": "world_enu",
        "player_start_location_ue_cm": [origin_location.x, origin_location.y, origin_location.z],
        "player_start_yaw_deg": math.degrees(origin_yaw),
        "resolution_m": args.resolution,
        "bounds_enu_m": {"x": args.x, "y": args.y, "z": args.z},
        "dimensions_xy": [nx, ny],
        "occupied_cells": len(occupied),
        "ply": str(output.resolve()),
    }
    output.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, ensure_ascii=True))


if __name__ == "__main__":
    main()
