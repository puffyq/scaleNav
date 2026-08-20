#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
SOURCE="${SOURCE:-$PROJECT_ROOT/models/original_yopo_simple/model.pt}"
OUTPUT="${OUTPUT:-$PROJECT_ROOT/saved/graph_executor/text_yopo.pt}"
DEVICE="${DEVICE:-cuda}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -f "$SOURCE" ]] || { echo "错误: 原版 YOPO 模型不存在: $SOURCE" >&2; exit 1; }
export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"

exec "$PYTHON" -m openseek.export_graph_executor \
  --source "$SOURCE" --output "$OUTPUT" --device "$DEVICE" "$@"
