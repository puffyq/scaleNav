#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"

# Balanced ESDF-like bubble attraction.  The score head uses only detached
# YOPO total-cost regression; no independent safety ranking is enabled.
PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data "${DATA:-dataset/pilot_esdf_001}" \
  --output "${OUTPUT:-saved}" \
  --checkpoint "${CHECKPOINT:-saved/YOPO_29/best.pth}" \
  --finetune \
  --epochs "${EPOCHS:-12}" \
  --batch-size "${BATCH_SIZE:-32}" \
  --learning-rate "${LEARNING_RATE:-3e-5}" \
  --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cuda}" \
  --seed "${SEED:-502031}" \
  --freeze-backbone-epochs 0 \
  --save-interval 2 \
  --bubble-weight "${BUBBLE_WEIGHT:-0.01}" \
  --path-mse-weight "${PATH_MSE_WEIGHT:-0.2}" \
  --progress-weight "${PROGRESS_WEIGHT:-0.1}" \
  --progress-floor-weight 0.0 \
  --safety-ranking-weight 0
