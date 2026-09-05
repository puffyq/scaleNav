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
  echo "GCN 方向箭头: /scalenav/gcn_selected"
  echo "GCN 方向列号: /scalenav/gcn_frontier_column"
  echo "Graph/A*: /scalenav/graph, /scalenav/path"
  echo "障碍物点云: /depth/points (free rays: /depth/free_rays)"
  echo "Depth/RGB/heatmap: /camera/depth/image, /camera/color/image, /scalenav/text_heatmap"
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
echo "GCN selected direction: /scalenav/gcn_selected (column: /scalenav/gcn_frontier_column)"
echo "Graph/A* topics: /scalenav/graph /scalenav/path"
echo "Obstacle cloud: /depth/points (free rays: /depth/free_rays)"
echo "Images: depth=/camera/depth/image rgb=/camera/color/image heatmap=/scalenav/text_heatmap"
if command -v ros2 >/dev/null 2>&1; then
  graph_publishers="$(ros2 topic info /scalenav/graph 2>/dev/null | awk '/Publisher count:/ {print $3; exit}' || true)"
  gcn_publishers="$(ros2 topic info /scalenav/gcn_selected 2>/dev/null | awk '/Publisher count:/ {print $3; exit}' || true)"
  if [[ "${graph_publishers:-0}" == "0" || "${gcn_publishers:-0}" == "0" ]]; then
    echo "提示: 当前 graph=${graph_publishers:-0} 个 publisher, GCN=${gcn_publishers:-0} 个 publisher。"
    echo "请先运行 bash $WS/scripts/start_gcn_online.sh，再启动本脚本。"
  fi
fi
exec rviz2 -f "$FIXED_FRAME" -d "$RVIZ_CONFIG" "$@"
