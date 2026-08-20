#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
COLOSSEUM_ROOT="${COLOSSEUM_ROOT:-/mnt/code/lab/airsim/Colosseum}"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
ROS_INSTALL="$COLOSSEUM_ROOT/ros2/${ROS_INSTALL_BASE:-install}"
HOST="${RPC_HOST:-127.0.0.1}"
PORT="${RPC_PORT:-41451}"
RATE="${RATE:-5.0}"
MARKER_TOPIC="${MARKER_TOPIC:-/openseek/graph_markers}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "错误: 未找到 ROS2 Humble" >&2; exit 1; }
[[ -f "$ROS_INSTALL/setup.bash" ]] || { echo "错误: 未找到 Colosseum ROS2 环境: $ROS_INSTALL" >&2; exit 1; }

export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
set +u
source /opt/ros/humble/setup.bash
source "$ROS_INSTALL/setup.bash"
set -u
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$COLOSSEUM_ROOT/PythonClient:$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"

echo "启动在线 Graph -> UE 可视化: topic=$MARKER_TOPIC RPC=$HOST:$PORT rate=${RATE}Hz"
echo "请确认 UE/AirSim 已经运行，并且规划器使用 GRAPH_VISUALIZATION=1。"
exec "$PYTHON" "$PROJECT_ROOT/openseek/online_graph_ue_bridge.py" \
  --marker-topic "$MARKER_TOPIC" \
  --rate "$RATE" \
  --rpc-host "$HOST" \
  --rpc-port "$PORT" \
  "$@"
