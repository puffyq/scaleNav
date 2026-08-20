#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DATA_ROOT="${DATA_ROOT:-$PROJECT_ROOT/data}"
STRIDE="${STRIDE:-4}"
MAX_POINTS_PER_FRAME="${MAX_POINTS_PER_FRAME:-1500}"

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1

for split in TrainingData TestingData; do
  "$PYTHON" -m openseek.data.rebuild_depth_map \
    --data "$DATA_ROOT/$split" \
    --stride "$STRIDE" \
    --max-points-per-frame "$MAX_POINTS_PER_FRAME"
done
