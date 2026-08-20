#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
SPLIT="${SPLIT:-train}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8765}"

case "$SPLIT" in
  train) DEFAULT_DATA="$PROJECT_ROOT/data/TrainingData" ;;
  test) DEFAULT_DATA="$PROJECT_ROOT/data/TestingData" ;;
  *) echo "错误: SPLIT 只能是 train 或 test" >&2; exit 1 ;;
esac
DATA_DIR="${DATA_DIR:-$DEFAULT_DATA}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$DATA_DIR" ]] || { echo "错误: 数据目录不存在: $DATA_DIR" >&2; exit 1; }

echo "启动数据可视化: http://$HOST:$PORT"
echo "数据目录: $DATA_DIR"
exec "$PYTHON" "$PROJECT_ROOT/tools/data_inspector_server.py" \
  --data "$DATA_DIR" --host "$HOST" --port "$PORT" "$@"

