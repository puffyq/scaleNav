#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/src/config/scalenav_graph.rviz}"
FIXED_FRAME="${RVIZ_FIXED_FRAME:-world_enu}"

usage() {
  echo "用法: $0 [--config FILE] [--fixed-frame FRAME] [rviz2 参数...]"
  echo "默认配置: $RVIZ_CONFIG"
  echo "默认固定坐标系: $FIXED_FRAME"
  echo "MPC 轨迹: /scalenav/route_yopo/mpc_path"
  echo "MPC bubbles: /scalenav/route_yopo/mpc_bubbles"
}

while (($#)); do
  case "$1" in
    --config)
      (($# >= 2)) || { echo "错误: --config 需要文件路径" >&2; exit 2; }
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --fixed-frame)
      (($# >= 2)) || { echo "错误: --fixed-frame 需要坐标系名称" >&2; exit 2; }
      FIXED_FRAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $SCRIPT_DIR/build.sh first" >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "RViz config not found: $RVIZ_CONFIG" >&2; exit 1; }
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
echo "RViz fixed frame: $FIXED_FRAME"
echo "MPC trajectory topic: /scalenav/route_yopo/mpc_path"
echo "MPC bubble topic: /scalenav/route_yopo/mpc_bubbles"
exec rviz2 -f "$FIXED_FRAME" -d "$RVIZ_CONFIG" "$@"
