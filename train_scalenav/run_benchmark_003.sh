#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-dataset/benchmark_003}"
OUTPUT="${OUTPUT:-dataset/benchmark_003/comparison_002}"
TREE_PLY="${TREE_PLY:-/mnt/code/lab/yopo/YOPO-Simple/Simulator/src/pointcloud/tree.ply}"

if [[ ! -f "$DATA/generation_report.json" ]]; then
  "$PYTHON" -m data.ground_truth_dataset \
    --output "$DATA" --scenes 3 --frames 200 --routes-per-frame 3 \
    --scene-styles yopo_forest,yopo_real_forest,blocks --obstacles 40 \
    --seed 820003 --yopo-tree-ply "$TREE_PLY" --preview-routes 60 \
    --dataset-role offline_test --overwrite
fi

"$PYTHON" -m data.validate_snapshot_dataset "$DATA" --require-routes
"$PYTHON" -m data.build_dataset_viewer "$DATA" --overwrite
PYTHONPATH="$ROOT" "$PYTHON" compare_yopo.py \
  --data "$DATA" \
  --route-checkpoint "${ROUTE_CHECKPOINT:-saved/YOPO_6/best.pth}" \
  --previous-route-checkpoint "${PREVIOUS_ROUTE_CHECKPOINT:-saved/YOPO_3/best.pth}" \
  --simple-checkpoint "${SIMPLE_CHECKPOINT:-/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth}" \
  --output "$OUTPUT" --batch-size "${BATCH_SIZE:-32}" \
  --workers "${WORKERS:-0}" --device "${DEVICE:-cpu}"
