#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-$PROJECT_ROOT/data/Map2GraphData}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8767}"
GOAL_DISTANCE="${GOAL_DISTANCE:-20.0}"
ROBOT_RADIUS="${ROBOT_RADIUS:-0.6}"
PEARL_ROOT="${PEARL_ROOT:-$PROJECT_ROOT/third_party/PEARL}"
PEARL_PROMPT="${PEARL_PROMPT:-obstacle}"
PEARL_DEVICE="${PEARL_DEVICE:-auto}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$DATA" ]] || { echo "错误: Graph 数据不存在: $DATA" >&2; exit 1; }

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1

echo "启动 Sparse Graph 可视化: http://$HOST:$PORT"
echo "数据: $DATA"
exec "$PYTHON" "$PROJECT_ROOT/openseek/serve_graph_viewer.py" \
  --data "$DATA" \
  --goal-distance "$GOAL_DISTANCE" \
  --robot-radius "$ROBOT_RADIUS" \
  --pearl-root "$PEARL_ROOT" \
  --pearl-prompt "$PEARL_PROMPT" \
  --pearl-device "$PEARL_DEVICE" \
  --host "$HOST" \
  --port "$PORT" \
  "$@"
