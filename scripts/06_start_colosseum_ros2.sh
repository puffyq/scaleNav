#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
COLOSSEUM_ROOT="${COLOSSEUM_ROOT:-/mnt/code/lab/airsim/Colosseum}"
ROS_ROOT="$COLOSSEUM_ROOT/ros2"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
ROS_INSTALL="$ROS_ROOT/${ROS_INSTALL_BASE:-install}"
RPC_IP="${RPC_IP:-127.0.0.1}"
VEHICLE_NAME="${VEHICLE_NAME:-drone_1}"
CAMERA_NAME="${CAMERA_NAME:-camera_0}"
AUTO_TAKEOFF="${AUTO_TAKEOFF:-1}"
TAKEOFF_TIMEOUT="${TAKEOFF_TIMEOUT:-30}"
STREAM_READY_TIMEOUT="${STREAM_READY_TIMEOUT:-45}"
STREAM_READY_STABILIZE_SEC="${STREAM_READY_STABILIZE_SEC:-2}"
[[ -f "$ROS_SETUP" ]] || { echo "错误: 未找到 ROS2: $ROS_SETUP" >&2; exit 1; }
[[ -f "$ROS_INSTALL/setup.bash" ]] || { echo "错误: 请先运行 03_build_colosseum_ros2.sh" >&2; exit 1; }
[[ "$AUTO_TAKEOFF" == "0" || "$AUTO_TAKEOFF" == "1" ]] || { echo "错误: AUTO_TAKEOFF 只能是 0 或 1" >&2; exit 1; }

export PATH="/usr/bin:/bin:$PATH"
set +u
source "$ROS_SETUP"
source "$ROS_INSTALL/setup.bash"
set -u

bridge_pid=""
tf_compat_pid=""
foreground_pid=""

stop_child_group() {
  local pid="$1"
  local attempt
  [[ -n "$pid" ]] || return 0

  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true

  # ros2 run starts the real node as a child and does not forward TERM.
  for ((attempt = 0; attempt < 20; attempt++)); do
    kill -0 -- "-$pid" 2>/dev/null || return 0
    sleep 0.05
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
}

start_child_group() {
  /usr/bin/setsid "$@" &
  foreground_pid=$!
}

run_interruptible() {
  start_child_group "$@"
  local status=0
  wait "$foreground_pid" || status=$?
  foreground_pid=""
  return "$status"
}

cleanup() {
  local status=$?
  trap - EXIT
  trap '' INT TERM
  stop_child_group "$foreground_pid"
  stop_child_group "$tf_compat_pid"
  stop_child_group "$bridge_pid"
  exit "$status"
}
trap cleanup EXIT INT TERM

prefix="/colosseum_node/$VEHICLE_NAME/$CAMERA_NAME"
args=(
  --ros-args
  -p "host_ip:=$RPC_IP"
  -p coordinate_system_enu:=true
  -p world_frame_id:=world_enu
  -p odom_frame_id:=odom_local_enu
  -p is_vulkan:=false
  -p update_colosseum_img_response_every_n_sec:=0.05
  -p update_colosseum_control_every_n_sec:=0.01
  -p update_lidar_every_n_sec:=0.01
  -p publish_clock:=false
  -r "/colosseum_node/$VEHICLE_NAME/odom_local_enu:=/sim/odom"
  -r "$prefix/Scene:=/camera/color/image"
  -r "$prefix/DepthPlanar:=/camera/depth/image"
  -r "$prefix/DepthPlanar/camera_info:=/camera/depth/camera_info"
  -r "/colosseum_node/$VEHICLE_NAME/lidar/front_lidar:=/lidar/front/points"
)
echo "启动 Colosseum 官方 ROS2 bridge"
echo "  camera:   $CAMERA_NAME"
echo "  source:   $prefix/{Scene,DepthPlanar}"
echo "  output:   /sim/odom, /camera/color/image, /camera/depth/image, /lidar/front/points"
/usr/bin/setsid ros2 run colosseum_ros_pkgs colosseum_node "${args[@]}" "$@" &
bridge_pid=$!

/usr/bin/setsid /usr/bin/python3 "$PROJECT_ROOT/openseek/colosseum_tf_compat_ros2.py" \
  --vehicle-frame "$VEHICLE_NAME" \
  >/tmp/openseek_colosseum_tf_compat.log 2>&1 &
tf_compat_pid=$!

if [[ "$AUTO_TAKEOFF" == "1" ]]; then
  echo "等待 RGBD 数据流就绪..."
  if ! run_interruptible timeout "$STREAM_READY_TIMEOUT" /usr/bin/python3 "$PROJECT_ROOT/scripts/check_rgbd_stream.py" --wait-until-ready --timeout "$STREAM_READY_TIMEOUT"; then
    echo "错误: RGBD 数据流未就绪，取消自动起飞。请确认 UE 已按 Play，且相机/ForwardRGBD 正常工作。" >&2
    exit 1
  fi
  run_interruptible sleep "$STREAM_READY_STABILIZE_SEC"
  echo "等待 takeoff 服务并自动起飞..."
  takeoff_output="$(mktemp /tmp/openseek_takeoff.XXXXXX)"
  if ! run_interruptible timeout "$TAKEOFF_TIMEOUT" ros2 service call "/colosseum_node/$VEHICLE_NAME/takeoff" colosseum_interfaces/srv/Takeoff "{wait_on_last_task: true}" >"$takeoff_output" 2>&1; then
    cat "$takeoff_output" >&2
    rm -f "$takeoff_output"
    echo "错误: 自动起飞失败。RGBD 已就绪，但 takeoff RPC 未完成；请检查 UE 是否仍在 Play、无人机是否可控。" >&2
    exit 1
  fi
  rm -f "$takeoff_output"
  # The upstream callback runs takeoffAsync(), but never assigns
  # response.success, so ROS always prints success=False even on completion.
  echo "自动起飞 RPC 已完成（官方 bridge 的 success 字段未实现）。"
fi

wait "$bridge_pid"
