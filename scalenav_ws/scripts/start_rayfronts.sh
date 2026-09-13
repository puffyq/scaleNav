#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"
RAYFRONTS_ROOT="${RAYFRONTS_ROOT:-$ROOT/bc/third_party/compare/RayFronts-main}"
PYTHON="${RAYFRONTS_PYTHON:-$ROOT/../YOPO-Rally/.venv/bin/python}"
DEVICE="${DEVICE:-cuda}"
LAYER_Z="${GRAPH_LAYER_Z:-1.6}"
PROMPT="${PROMPT:-blocks, walls, box}"
MAX_DEPTH="${RAYFRONTS_MAX_DEPTH_M:-20.0}"

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "RayFronts Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$RAYFRONTS_ROOT/rayfronts/mapping_server.py" ]] || {
  echo "RayFronts source not found: $RAYFRONTS_ROOT" >&2; exit 1;
}

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="$RAYFRONTS_ROOT:$WS/src/scalenav:${PYTHONPATH:-}"
RAYFRONTS_CPP="$RAYFRONTS_ROOT/rayfronts/csrc/build"
if [[ -d "$RAYFRONTS_CPP" ]]; then
  export PYTHONPATH="$RAYFRONTS_CPP:$PYTHONPATH"
fi
LOCAL_DEPS="$ROOT/.rayfronts_local"
if [[ -d "$LOCAL_DEPS" ]]; then
  # Put the extracted OpenVDB Python binding before the local compatibility
  # shim in scalenav_ws/src/scalenav/openvdb.py.  RayFronts imports openvdb
  # directly and otherwise resolves the shim first, which then fails to find
  # pyopenvdb and exits the whole stack with status 1.
  export PYTHONPATH="$LOCAL_DEPS/usr/lib/python3/dist-packages:$PYTHONPATH"
  export LD_LIBRARY_PATH="$LOCAL_DEPS/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi

PIDS=()
run() { setsid stdbuf -oL -eL "$@" & PIDS+=("$!"); }
stop() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 0.2
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap stop EXIT INT TERM

run ros2 launch airsim_renderer controller_airsim.launch.py maximum_linear_speed:="${MAX_SPEED:-6.0}" ignore_collision:=false
run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py
run "$PYTHON" "$WS/src/scalenav/rayfronts_odom_pose_bridge.py"
run "$PYTHON" -m rayfronts.mapping_server \
  dataset=ros2zedx \
  dataset.rgb_topic=/camera/color/image \
  dataset.depth_topic=/camera/depth/image \
  dataset.pose_topic=/rayfronts/pose \
  dataset.intrinsics_topic=/camera/depth/camera_info \
  dataset.disparity_topic=null \
  dataset.src_coord_system=flu \
  mapping=frontier_vdb_map \
  encoder=base_encoder vis=ros messaging_service=ros \
  mapping.vox_size="${RAYFRONTS_VOX_SIZE:-0.3}" \
  mapping.max_depth_sensing="$MAX_DEPTH" \
  messaging_service.text_query_topic=/rayfronts/msg_serv/new_text_query
run "$PYTHON" "$WS/src/scalenav/rayfronts_frontier_planner.py" \
  --ros-args -p layer_z:="$LAYER_Z" -p prompt:="$PROMPT"
run "$PYTHON" "$WS/src/scalenav/online_planner_ros2.py" \
  --model "$WS/src/models/original_yopo_simple/model.pt" \
  --device "$DEVICE" --config-file "$WS/src/config/config.yaml" --control \
  --original-goal-input --goal-topic /scalenav/local_goal \
  --mission-goal-topic /goal_pose --world-frame world_enu \
  --odom-twist-frame body --fixed-altitude --plan-from-reference \
  --disable-event-log --maximum-trajectory-speed-mps "${MAX_SPEED:-6.0}" \
  --model-image-width 160 --model-image-height 96 --model-vertical-num 3
echo "started RayFronts-only mapping backend; root=$RAYFRONTS_ROOT layer_z=$LAYER_Z max_depth=${MAX_DEPTH}m prompt=$PROMPT"
wait -n "${PIDS[@]}"
