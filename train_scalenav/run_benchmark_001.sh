#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-dataset/benchmark_001}"
OUTPUT="${OUTPUT:-dataset/benchmark_001/comparison_001}"

if [[ ! -f "$DATA/generation_report.json" ]]; then
  "$PYTHON" -m data.ground_truth_dataset \
    --output "$DATA" --scenes 2 --frames 200 --routes-per-frame 3 \
    --scene-styles yopo_forest,blocks --seed 120001 \
    --preview-routes 40 --dataset-role offline_test --overwrite
fi

PYTHONPATH="$ROOT" "$PYTHON" compare_yopo.py \
  --data "$DATA" \
  --route-checkpoint "${ROUTE_CHECKPOINT:-saved/YOPO_3/best.pth}" \
  --simple-checkpoint "${SIMPLE_CHECKPOINT:-/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth}" \
  --output "$OUTPUT" --batch-size "${BATCH_SIZE:-32}" --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cpu}"
