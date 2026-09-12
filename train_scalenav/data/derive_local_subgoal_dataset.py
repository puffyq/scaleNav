"""Derive a V2 route dataset with a fixed witness-relative local subgoal."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np

from .route_contract import (
    ROUTE_DATASET_VERSION,
    RouteRecord,
    load_route_table,
    local_subgoal_on_witness,
    pack_route_records,
    save_route_table,
)


def derive_local_subgoal_dataset(
    source: Path,
    output: Path,
    *,
    local_subgoal_distance_m: float = 10.0,
    overwrite: bool = False,
) -> Path:
    source = Path(source).resolve()
    output = Path(output).resolve()
    if source == output:
        raise ValueError("source and output datasets must differ")
    if not source.is_dir():
        raise FileNotFoundError(source)
    if output.exists():
        if not overwrite:
            raise FileExistsError(output)
        shutil.rmtree(output)

    # Immutable RGB-D and point-cloud assets are hard-linked; route tables and
    # reports are replaced below, so the source batch can never be modified.
    shutil.copytree(source, output, copy_function=os.link)
    route_count = 0
    actual_distances: list[float] = []
    for route_path in sorted(output.glob("Scene_*/routes.npz")):
        table = load_route_table(route_path)
        records: list[RouteRecord] = []
        distances: list[float] = []
        for index in range(len(table)):
            witness, clearance, radii = table.path(index)
            if float(table.arrays["path_length_m"][index]) + 1.0e-5 < local_subgoal_distance_m:
                continue
            _, distance = local_subgoal_on_witness(
                witness, local_subgoal_distance_m
            )
            distances.append(distance)
            topo_centers, topo_radii, topo_ids = table.topology(index)
            records.append(
                RouteRecord(
                    frame_index=int(table.arrays["frame_index"][index]),
                    mission_goal_world=table.arrays["mission_goal_world"][index],
                    frontier_goal_world=table.arrays["frontier_goal_world"][index],
                    path_points_world=witness,
                    path_clearance_m=clearance,
                    path_bubble_radius_m=radii,
                    topo_centers_world=topo_centers,
                    topo_bubble_radius_m=topo_radii,
                    topo_persistent_id=topo_ids,
                    route_valid=bool(table.arrays["route_valid"][index]),
                    route_quality_flags=int(table.arrays["route_quality_flags"][index]),
                    route_quality_weight=float(table.arrays["route_quality_weight"][index]),
                    route_seed=int(table.arrays["route_seed"][index]),
                    route_search_detour_ratio=float(
                        table.arrays.get("route_search_detour_ratio", np.ones(len(table)))[index]
                    ),
                    route_centerline_gain_m=float(
                        table.arrays.get("route_centerline_gain_m", np.zeros(len(table)))[index]
                    ),
                    local_subgoal_distance_m=distance,
                )
            )
        save_route_table(route_path, pack_route_records(records))
        route_count += len(records)
        actual_distances.extend(distances)

    if route_count == 0:
        raise FileNotFoundError(f"no Scene_*/routes.npz under {source}")
    report_path = output / "generation_report.json"
    report = (
        json.loads(report_path.read_text(encoding="utf-8"))
        if report_path.is_file()
        else {}
    )
    # copytree used hard links for large immutable assets; reports are mutable.
    report_path.unlink(missing_ok=True)
    report["derived_from"] = str(source)
    report["route_dataset_version"] = ROUTE_DATASET_VERSION
    report["local_subgoal_contract"] = {
        "requested_distance_m": float(local_subgoal_distance_m),
        "minimum_actual_distance_m": float(min(actual_distances)),
        "maximum_actual_distance_m": float(max(actual_distances)),
        "route_count": route_count,
        "short_witnesses_filtered": True,
        "definition": "arclength interpolation on the complete witness",
    }
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    for generated in (output / "viewer", output / "route_previews"):
        if generated.exists():
            shutil.rmtree(generated)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--distance", type=float, default=10.0)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    print(
        derive_local_subgoal_dataset(
            args.source,
            args.output,
            local_subgoal_distance_m=args.distance,
            overwrite=args.overwrite,
        )
    )


if __name__ == "__main__":
    main()
