#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"
SOURCE_DATA="${SOURCE_DATA:-dataset/pilot_002}"
DATA="${DATA:-dataset/pilot_003}"
INITIAL_CHECKPOINT="${INITIAL_CHECKPOINT:-saved/YOPO_6/best.pth}"
DEVICE="${DEVICE:-cuda}"

"$PYTHON" -m data.derive_local_subgoal_dataset \
  --source "$SOURCE_DATA" --output "$DATA" --distance 10 --overwrite
"$PYTHON" -m data.validate_snapshot_dataset "$DATA" --require-routes

PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data "$DATA" --output saved --checkpoint "$INITIAL_CHECKPOINT" --finetune \
  --epochs 25 --batch-size 32 --learning-rate 7.5e-5 --workers "${WORKERS:-0}" \
  --device "$DEVICE" --seed 502006 --freeze-backbone-epochs 0 --save-interval 5 \
  --progress-weight 1.2 --progress-floor-m 6.4 --progress-floor-weight 0.5

STAGE1_DIR="$(find saved -maxdepth 1 -type d -name 'YOPO_*' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
test -f "$STAGE1_DIR/epoch5.pth"

PYTHONPATH="$ROOT" "$PYTHON" train_yopo.py \
  --data "$DATA" --output saved --checkpoint "$STAGE1_DIR/epoch5.pth" --finetune \
  --epochs 15 --batch-size 32 --learning-rate 5e-5 --workers "${WORKERS:-0}" \
  --device "$DEVICE" --seed 502007 --freeze-backbone-epochs 0 --save-interval 3 \
  --progress-weight 1.2 --progress-floor-m 6.8 --progress-floor-weight 0.7

echo "Select epoch12.pth from the second stage using run_benchmark_004.sh."
