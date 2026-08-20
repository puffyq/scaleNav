#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
LOG_DIR="${LOG_DIR:-$PROJECT_ROOT/log_event}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8765}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$LOG_DIR" ]] || { echo "错误: 日志目录不存在: $LOG_DIR" >&2; exit 1; }

echo "启动 OpenSeek 日志分析器: http://$HOST:$PORT"
echo "日志目录: $LOG_DIR"
exec "$PYTHON" "$PROJECT_ROOT/tools/log_viewer/server.py" \
  --log-dir "$LOG_DIR" \
  --host "$HOST" \
  --port "$PORT" \
  "$@"
