#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-$PROJECT_ROOT/data/Map2GraphData}"
SCENE="${SCENE:-Scene_0001}"
FRAME="${FRAME:-0}"
OUTPUT="${OUTPUT:-$PROJECT_ROOT/data/map2_graph_result.json}"
SYNTHETIC="${SYNTHETIC:-0}"

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1
if [[ "$SYNTHETIC" == "1" ]]; then
  exec "$PYTHON" -m graph.replay --synthetic-wall --frame "$FRAME" --output "$OUTPUT" "$@"
fi
[[ -f "$DATA/$SCENE/data.toml" ]] || {
  echo "错误: Map2 数据不存在，请先打开 FlyingExampleMapV2 并运行 scripts/21_collect_map2_graph_data.sh" >&2
  echo "无需 UE 的大墙回归测试: SYNTHETIC=1 bash scripts/22_test_map2_graph.sh" >&2
  exit 1
}
exec "$PYTHON" -m graph.replay --data "$DATA" --scene "$SCENE" --frame "$FRAME" --output "$OUTPUT" "$@"
