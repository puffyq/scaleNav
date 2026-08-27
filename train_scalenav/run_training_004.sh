#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"

# Safety-first score calibration from the balanced YOPO_14 candidate.
PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data dataset/pilot_003 \
  --output saved \
  --checkpoint saved/YOPO_14/best.pth \
  --finetune \
  --epochs "${EPOCHS:-8}" \
  --batch-size "${BATCH_SIZE:-32}" \
  --learning-rate "${LEARNING_RATE:-1.5e-5}" \
  --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cuda}" \
  --seed "${SEED:-502015}" \
  --save-interval 2 \
  --progress-weight 1.2 \
  --progress-floor-m 6.8 \
  --progress-floor-weight 0.7 \
  --safety-weight 1.2 \
  --safety-peak-weight 1.5 \
  --safety-collision-margin-weight 5.0 \
  --safety-ranking-weight 8.0 \
  --safety-ranking-target-margin 0.0001
