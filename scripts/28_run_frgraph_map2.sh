#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DATA="${DATA:-$PROJECT_ROOT/data/Map2GraphData}"
SCENE="${SCENE:-Scene_0002}"
FRAME="${FRAME:-0}"
OUTPUT="${OUTPUT:-$PROJECT_ROOT/data/frgraph_map2_result.json}"
ROBOT_RADIUS="${ROBOT_RADIUS:-0.6}"

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1

exec "$PYTHON" -m graph.replay \
  --data "$DATA" \
  --scene "$SCENE" \
  --frame "$FRAME" \
  --robot-radius "$ROBOT_RADIUS" \
  --frgraph \
  --output "$OUTPUT"
