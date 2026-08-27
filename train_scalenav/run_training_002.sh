#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-dataset/pilot_002}"
INITIAL_CHECKPOINT="${INITIAL_CHECKPOINT:-saved/YOPO_3/best.pth}"

PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data "$DATA" --output saved --checkpoint "$INITIAL_CHECKPOINT" --finetune \
  --epochs 30 --batch-size 16 --learning-rate 1.5e-4 --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cpu}" --seed 502002 --freeze-backbone-epochs 3 --save-interval 5 \
  --progress-weight 0.8

if [[ -z "${STAGE1_CHECKPOINT:-}" ]]; then
  STAGE1_CHECKPOINT="$(find saved -maxdepth 2 -type f -path 'saved/YOPO_*/best.pth' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
fi
test -f "$STAGE1_CHECKPOINT"

PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data "$DATA" --output saved --checkpoint "$STAGE1_CHECKPOINT" --finetune \
  --epochs 15 --batch-size 16 --learning-rate 7.5e-5 --workers "${WORKERS:-0}" \
  --device "${DEVICE:-cpu}" --seed 502003 --freeze-backbone-epochs 0 --save-interval 5 \
  --progress-weight 1.2
