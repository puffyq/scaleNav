#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
SOURCE_DATA="${SOURCE_DATA:-dataset/benchmark_003}"
DATA="${DATA:-dataset/benchmark_004}"
OUTPUT="${OUTPUT:-dataset/benchmark_004/comparison_003}"

if [[ ! -f "$DATA/generation_report.json" ]]; then
  "$PYTHON" -m data.derive_local_subgoal_dataset \
    --source "$SOURCE_DATA" --output "$DATA" --distance 10 --overwrite
fi
"$PYTHON" -m data.validate_snapshot_dataset "$DATA" --require-routes
PYTHONPATH="$ROOT" "$PYTHON" compare_yopo.py \
  --data "$DATA" \
  --route-checkpoint "${ROUTE_CHECKPOINT:-saved/YOPO_10/epoch12.pth}" \
  --previous-route-checkpoint "${PREVIOUS_ROUTE_CHECKPOINT:-saved/YOPO_9/epoch5.pth}" \
  --simple-checkpoint "${SIMPLE_CHECKPOINT:-/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth}" \
  --output "$OUTPUT" --batch-size "${BATCH_SIZE:-64}" \
  --workers "${WORKERS:-0}" --device "${DEVICE:-cuda}"
