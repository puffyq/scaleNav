#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-dataset/pilot_002}"
TREE_PLY="${TREE_PLY:-/mnt/code/lab/yopo/YOPO-Simple/Simulator/src/pointcloud/tree.ply}"

if [[ ! -f "$DATA/generation_report.json" ]]; then
  "$PYTHON" -m data.ground_truth_dataset \
    --output "$DATA" --scenes 3 --frames 250 --routes-per-frame 3 \
    --scene-styles yopo_forest,yopo_real_forest,blocks --obstacles 40 \
    --seed 502002 --yopo-tree-ply "$TREE_PLY" --preview-routes 100 \
    --dataset-role train --overwrite
fi

"$PYTHON" -m data.validate_snapshot_dataset "$DATA" --require-routes
"$PYTHON" -m data.build_dataset_viewer "$DATA" --overwrite
