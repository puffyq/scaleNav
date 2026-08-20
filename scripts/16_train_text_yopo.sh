#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
TRAIN_DATA="${TRAIN_DATA:-$PROJECT_ROOT/data/TrainingData}"
TEST_DATA="${TEST_DATA:-$PROJECT_ROOT/data/TestingData}"
OUTPUT="${OUTPUT:-$PROJECT_ROOT/saved/map4_person}"
EPOCHS="${EPOCHS:-50}"
BATCH_SIZE="${BATCH_SIZE:-8}"
WORKERS="${WORKERS:-0}"
PRECISION="${PRECISION:-fp32}"
EVAL_EVERY="${EVAL_EVERY:-5}"
CHECKPOINT_EVERY="${CHECKPOINT_EVERY:-5}"
PRETRAINED_BACKBONE="${PRETRAINED_BACKBONE:-$PROJECT_ROOT/models/depth_backbone/epoch50.pth}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$TRAIN_DATA" ]] || { echo "错误: 训练集不存在: $TRAIN_DATA" >&2; exit 1; }
[[ -d "$TEST_DATA" ]] || { echo "错误: 测试集不存在: $TEST_DATA" >&2; exit 1; }
[[ "$PRECISION" == "fp32" || "$PRECISION" == "amp" ]] || { echo "错误: PRECISION 只能是 fp32 或 amp" >&2; exit 1; }

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1

args=(
  "$PROJECT_ROOT/openseek/train_text_yopo.py"
  --train-data "$TRAIN_DATA"
  --test-data "$TEST_DATA"
  --output "$OUTPUT"
  --epochs "$EPOCHS"
  --batch-size "$BATCH_SIZE"
  --workers "$WORKERS"
  --precision "$PRECISION"
  --eval-every "$EVAL_EVERY"
  --checkpoint-every "$CHECKPOINT_EVERY"
)
if [[ -n "$PRETRAINED_BACKBONE" ]]; then
  [[ -f "$PRETRAINED_BACKBONE" ]] || { echo "错误: 深度预训练权重不存在: $PRETRAINED_BACKBONE" >&2; exit 1; }
  args+=(--pretrained-backbone "$PRETRAINED_BACKBONE")
fi

echo "开始训练: train=$TRAIN_DATA test=$TEST_DATA epochs=$EPOCHS batch=$BATCH_SIZE"
echo "模型输出: $OUTPUT"
exec "$PYTHON" "${args[@]}" "$@"

