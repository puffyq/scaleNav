#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"

# Fixed-altitude Route contract: training loss evaluates the same projected
# terminal state that the online controller certifies and executes.
PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data "${DATA:-dataset/train_large_001}" \
  --output "${OUTPUT:-saved_fixed_altitude}" \
  --checkpoint "${CHECKPOINT:-saved_large/YOPO_0/best.pth}" \
  --finetune \
  --epochs "${EPOCHS:-12}" \
  --batch-size "${BATCH_SIZE:-32}" \
  --learning-rate "${LEARNING_RATE:-1e-5}" \
  --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cuda}" \
  --seed "${SEED:-602042}" \
  --freeze-backbone-epochs 0 \
  --save-interval 3 \
  --bubble-weight "${BUBBLE_WEIGHT:-0.01}" \
  --path-mse-weight "${PATH_MSE_WEIGHT:-0.2}" \
  --centerline-weight "${CENTERLINE_WEIGHT:-0.1}" \
  --route-incompatible-score-cost "${ROUTE_INCOMPATIBLE_SCORE_COST:-10.0}" \
  --progress-weight "${PROGRESS_WEIGHT:-0.1}" \
  --progress-floor-weight 0.0 \
  --score-ranking-weight "${SCORE_RANKING_WEIGHT:-1.0}" \
  --safety-ranking-weight "${SAFETY_RANKING_WEIGHT:-1.0}" \
  --safety-ranking-target-margin "${SAFETY_RANKING_TARGET_MARGIN:-0.001}"
