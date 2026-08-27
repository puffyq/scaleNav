#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-dataset/benchmark_004_esdf_001}"
OUTPUT="${OUTPUT:-$DATA/comparison_019_repro}"

PYTHONPATH="$ROOT" "$PYTHON" compare_yopo.py \
  --data "$DATA" \
  --route-checkpoint "${ROUTE_CHECKPOINT:-saved/YOPO_31/best.pth}" \
  --previous-route-checkpoint "${PREVIOUS_ROUTE_CHECKPOINT:-saved/YOPO_30/best.pth}" \
  --simple-checkpoint "${SIMPLE_CHECKPOINT:-/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth}" \
  --output "$OUTPUT" \
  --batch-size "${BATCH_SIZE:-64}" \
  --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cuda}"
