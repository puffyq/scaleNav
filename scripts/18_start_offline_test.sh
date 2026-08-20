#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DATA_DIR="${DATA_DIR:-$PROJECT_ROOT/data/TestingData}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8766}"
DEVICE="${DEVICE:-auto}"
MODE="${MODE:-auto}"
DEFAULT_MODEL="$PROJECT_ROOT/saved/map4_goal/best/text_yopo.pt"
if [[ ! -f "$DEFAULT_MODEL" && -f "$PROJECT_ROOT/saved/map4_goal_smoke/best/text_yopo.pt" ]]; then
  DEFAULT_MODEL="$PROJECT_ROOT/saved/map4_goal_smoke/best/text_yopo.pt"
fi
MODEL="${MODEL:-$DEFAULT_MODEL}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$DATA_DIR" ]] || { echo "错误: 测试集不存在: $DATA_DIR" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "错误: 模型不存在，请先运行 scripts/16_train_text_yopo.sh: $MODEL" >&2; exit 1; }

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1

echo "启动离线测试: http://$HOST:$PORT"
echo "测试集: $DATA_DIR"
echo "模型: $MODEL"
exec "$PYTHON" "$PROJECT_ROOT/openseek/serve_text_yopo_test.py" \
  --data "$DATA_DIR" \
  --model "$MODEL" \
  --device "$DEVICE" \
  --mode "$MODE" \
  --host "$HOST" \
  --port "$PORT" \
  "$@"
