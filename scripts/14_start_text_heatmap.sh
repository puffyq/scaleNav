#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
PROMPT="${PROMPT:-person}"
INPUT_TOPIC="${INPUT_TOPIC:-/camera/color/image}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/openseek/text_heatmap}"
DEVICE="${DEVICE:-cuda}"
UPDATE_RATE="${UPDATE_RATE:-10}"
PEARL_ROOT="${PEARL_ROOT:-$PROJECT_ROOT/third_party/PEARL}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "错误: 未找到 ROS2 Humble" >&2; exit 1; }
[[ -f "$PEARL_ROOT/pearl/prop.py" ]] || { echo "错误: PEARL 源码不完整: $PEARL_ROOT" >&2; exit 1; }

export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PROJECT_ROOT/openseek${PYTHONPATH:+:$PYTHONPATH}"

echo "启动 PEARL 热力图: prompt=$PROMPT input=$INPUT_TOPIC output=$OUTPUT_TOPIC rate=$UPDATE_RATE device=$DEVICE"
exec "$PYTHON" "$PROJECT_ROOT/openseek/text_heatmap_ros2.py" \
  --prompt "$PROMPT" \
  --input-topic "$INPUT_TOPIC" \
  --output-topic "$OUTPUT_TOPIC" \
  --device "$DEVICE" \
  --update-rate "$UPDATE_RATE" \
  --pearl-root "$PEARL_ROOT" \
  "$@"
