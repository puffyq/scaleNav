#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_INSTALL="$PROJECT_ROOT/install"
CONTROL="${CONTROL:-1}"
EPIC_ONLINE="${EPIC_ONLINE:-0}"
START_RENDERER="${START_RENDERER:-1}"
START_SEMANTIC="${START_SEMANTIC:-0}"
SEMANTIC_PROMPT="${SEMANTIC_PROMPT:-tree}"
SEMANTIC_UPDATE_RATE="${SEMANTIC_UPDATE_RATE:-5}"
COLOSSEUM_ROOT="${COLOSSEUM_ROOT:-/mnt/code/lab/airsim/Colosseum}"
ROS_INSTALL="$COLOSSEUM_ROOT/ros2/${ROS_INSTALL_BASE:-install}"
MODEL="${MODEL:-$PROJECT_ROOT/models/original_yopo_simple/model.pt}"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DEVICE="${DEVICE:-cuda}"
SEARCH_DISTANCE="${SEARCH_DISTANCE:-10.0}"
HEATMAP_SIGMA="${HEATMAP_SIGMA:-7.5}"
DEPTH_TIMEOUT="${DEPTH_TIMEOUT:-0.5}"
GOAL_TOPIC="${GOAL_TOPIC:-/goal_pose}"
GOAL_TOLERANCE="${GOAL_TOLERANCE:-2.0}"
EPIC_GLOBAL_GOAL_TOPIC="${EPIC_GLOBAL_GOAL_TOPIC:-/goal_pose}"
EPIC_NEXT_GOAL_TOPIC="${EPIC_NEXT_GOAL_TOPIC:-/epic/yopo_goal}"
MISSION_GOAL_TOPIC="${MISSION_GOAL_TOPIC:-}"
MISSION_GOAL_TOLERANCE="${MISSION_GOAL_TOLERANCE:-0.5}"
MISSION_STOP_SPEED="${MISSION_STOP_SPEED:-0.3}"
FINAL_SUBGOAL_TOLERANCE="${FINAL_SUBGOAL_TOLERANCE:-0.25}"
EPIC_VISUALIZATION_FRAME="${EPIC_VISUALIZATION_FRAME:-odom}"
EPIC_TRAJECTORY_SPEED_COLOR_MAX_MPS="${EPIC_TRAJECTORY_SPEED_COLOR_MAX_MPS:-8.0}"
EPIC_TRAJECTORY_MAX_POINTS="${EPIC_TRAJECTORY_MAX_POINTS:-50000}"
EPIC_FLIGHT_STATISTICS_FILE="${EPIC_FLIGHT_STATISTICS_FILE:-$PROJECT_ROOT/log_event/epic_flight_statistics.csv}"
EPIC_GRAPH_FIXED_LAYER="${EPIC_GRAPH_FIXED_LAYER:-true}"
EPIC_GRAPH_LAYER_Z="${EPIC_GRAPH_LAYER_Z:-1.6}"
EPIC_REUSE_GRAPH_ON_GOAL="${EPIC_REUSE_GRAPH_ON_GOAL:-true}"
EPIC_MAP_MARGIN="${EPIC_MAP_MARGIN:-20.0}"
EPIC_MAP_VOXEL_SIZE="${EPIC_MAP_VOXEL_SIZE:-0.25}"
EPIC_MAP_HISTORY_RADIUS_M="${EPIC_MAP_HISTORY_RADIUS_M:-20.0}"
EPIC_MAP_MAX_POINTS="${EPIC_MAP_MAX_POINTS:-20000}"
EPIC_MAP_PRUNE_DISTANCE_M="${EPIC_MAP_PRUNE_DISTANCE_M:-0.5}"
EPIC_YOPO_GOAL_TOLERANCE="${EPIC_YOPO_GOAL_TOLERANCE:-0.5}"
EPIC_UPDATE_PERIOD_MS="${EPIC_UPDATE_PERIOD_MS:-100}"
EPIC_SKELETON_REBUILD_PERIOD_MS="${EPIC_SKELETON_REBUILD_PERIOD_MS:-500.0}"
EPIC_LOCAL_GOAL_MIN_ADVANCE_M="${EPIC_LOCAL_GOAL_MIN_ADVANCE_M:-0.75}"
EPIC_LOCAL_GOAL_LOOKAHEAD_M="${EPIC_LOCAL_GOAL_LOOKAHEAD_M:-10.0}"
EPIC_ROUTE_PLAN_PERIOD_MS="${EPIC_ROUTE_PLAN_PERIOD_MS:-2000}"
EPIC_LOCAL_GOAL_RESERVE_M="${EPIC_LOCAL_GOAL_RESERVE_M:-5.0}"
EPIC_USE_EDGE_WITNESS_PATH="${EPIC_USE_EDGE_WITNESS_PATH:-true}"
EPIC_RAYCAST_SHORTCUT_SAMPLE_STEP_M="${EPIC_RAYCAST_SHORTCUT_SAMPLE_STEP_M:-0.25}"
EPIC_RAYCAST_SHORTCUT_CLEARANCE_MARGIN_M="${EPIC_RAYCAST_SHORTCUT_CLEARANCE_MARGIN_M:-0.05}"
EPIC_GOAL_PATH_COST_WEIGHT="${EPIC_GOAL_PATH_COST_WEIGHT:-0.2}"
EPIC_SEMANTIC_COST_WEIGHT="${EPIC_SEMANTIC_COST_WEIGHT:-1.0}"
EPIC_SEMANTIC_NODE_EMA_ALPHA="${EPIC_SEMANTIC_NODE_EMA_ALPHA:-0.3}"
EPIC_SEMANTIC_VISUALIZATION_MAX_SCORE="${EPIC_SEMANTIC_VISUALIZATION_MAX_SCORE:-1.0}"
EPIC_SEMANTIC_ASSOCIATION_RADIUS_M="${EPIC_SEMANTIC_ASSOCIATION_RADIUS_M:-1.5}"
EPIC_SEMANTIC_DEPTH_CLIP_M="${EPIC_SEMANTIC_DEPTH_CLIP_M:-20.0}"
EPIC_SEMANTIC_DEPTH_SYNC_TOLERANCE_MS="${EPIC_SEMANTIC_DEPTH_SYNC_TOLERANCE_MS:-250.0}"
EPIC_SEMANTIC_PATCH_COLS="${EPIC_SEMANTIC_PATCH_COLS:-5}"
EPIC_SEMANTIC_PATCH_ROWS="${EPIC_SEMANTIC_PATCH_ROWS:-3}"
EPIC_SPECULATIVE_ENABLED="${EPIC_SPECULATIVE_ENABLED:-true}"
EPIC_SPECULATIVE_MIN_SCORE="${EPIC_SPECULATIVE_MIN_SCORE:-0.35}"
EPIC_SPECULATIVE_FORWARD_M="${EPIC_SPECULATIVE_FORWARD_M:-22.0}"
EPIC_SPECULATIVE_PATCH_SEPARATION_M="${EPIC_SPECULATIVE_PATCH_SEPARATION_M:-1.5}"
EPIC_SPECULATIVE_RADIUS_M="${EPIC_SPECULATIVE_RADIUS_M:-0.75}"
EPIC_SPECULATIVE_MAX_NODES="${EPIC_SPECULATIVE_MAX_NODES:-16}"
EPIC_SPECULATIVE_CONNECT_TIMEOUT_MS="${EPIC_SPECULATIVE_CONNECT_TIMEOUT_MS:-20.0}"
EPIC_CLEARANCE_COST_WEIGHT="${EPIC_CLEARANCE_COST_WEIGHT:-2.0}"
EPIC_CLEARANCE_TARGET_M="${EPIC_CLEARANCE_TARGET_M:-1.2}"
EPIC_PREVIOUS_PATH_COST_FACTOR="${EPIC_PREVIOUS_PATH_COST_FACTOR:-0.0}"
EPIC_ROUTE_REMAP_DISTANCE_M="${EPIC_ROUTE_REMAP_DISTANCE_M:-1.25}"
EPIC_ROUTE_REUSE_HORIZON_M="${EPIC_ROUTE_REUSE_HORIZON_M:-6.0}"
EPIC_ROUTE_REUSE_LATERAL_DISTANCE_M="${EPIC_ROUTE_REUSE_LATERAL_DISTANCE_M:-1.5}"
EPIC_ROUTE_TERMINAL_RELEASE_DISTANCE_M="${EPIC_ROUTE_TERMINAL_RELEASE_DISTANCE_M:-1.0}"
EPIC_GOAL_CONNECT_DISTANCE_M="${EPIC_GOAL_CONNECT_DISTANCE_M:-6.0}"
EPIC_GOAL_CONNECT_TIMEOUT_MS="${EPIC_GOAL_CONNECT_TIMEOUT_MS:-20.0}"
EPIC_ODOM_RECONNECT_DISTANCE_M="${EPIC_ODOM_RECONNECT_DISTANCE_M:-1.0}"
EPIC_ODOM_RECONNECT_YAW_DEG="${EPIC_ODOM_RECONNECT_YAW_DEG:-20.0}"
EPIC_ODOM_FALLBACK_RADIUS_M="${EPIC_ODOM_FALLBACK_RADIUS_M:-15.0}"
EPIC_ODOM_FALLBACK_CANDIDATES="${EPIC_ODOM_FALLBACK_CANDIDATES:-24}"
EPIC_ODOM_CONNECT_TIMEOUT_MS="${EPIC_ODOM_CONNECT_TIMEOUT_MS:-3.0}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/lidar/front/points}"
WORLD_FRAME="${WORLD_FRAME:-world_enu}"
MAX_YAW_RATE="${MAX_YAW_RATE:-1.5}"
if [[ -z "${ODOM_TWIST_FRAME+x}" ]]; then
  if [[ "$START_RENDERER" == "1" ]]; then
    # openseek_uav_sim follows nav_msgs/Odometry and publishes twist in base_link.
    ODOM_TWIST_FRAME="body"
  else
    ODOM_TWIST_FRAME="world"
  fi
fi
REFERENCE_RESET_POSITION_ERROR="${REFERENCE_RESET_POSITION_ERROR:-0.75}"
REFERENCE_RESET_VELOCITY_ERROR="${REFERENCE_RESET_VELOCITY_ERROR:-1.5}"
MINIMUM_TRAJECTORY_ALTITUDE="${MINIMUM_TRAJECTORY_ALTITUDE:-0.15}"
TRAJECTORY_ALTITUDE_MARGIN="${TRAJECTORY_ALTITUDE_MARGIN:-0.10}"
FIXED_ALTITUDE="${FIXED_ALTITUDE:-1}"
DIRECT_GOAL_DISTANCE="${DIRECT_GOAL_DISTANCE:-3.5}"
EVENT_LOG_DIR="${EVENT_LOG_DIR:-$PROJECT_ROOT/log_event}"
SAVE_DEPTH_PNG="${SAVE_DEPTH_PNG:-0}"
PLAN_FROM_REFERENCE="${PLAN_FROM_REFERENCE:-1}"
GRAPH_VISUALIZATION="${GRAPH_VISUALIZATION:-0}"
# Empty means the local SO3 controller's trajectory topic. Set this explicitly
# only when running the legacy official Colosseum velocity bridge.
export COLOSSEUM_CONTROL_TOPIC="${COLOSSEUM_CONTROL_TOPIC-}"
[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "错误: 在线模型不存在: $MODEL" >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "错误: 未找到 ROS2 Humble" >&2; exit 1; }
[[ -f "$ROS_INSTALL/setup.bash" ]] || { echo "错误: 请先运行 03_build_colosseum_ros2.sh" >&2; exit 1; }
[[ -f "$WORKSPACE_INSTALL/setup.bash" ]] || { echo "错误: 请先在 OpenSeek 根目录运行 colcon build --symlink-install" >&2; exit 1; }
[[ "$CONTROL" == "0" || "$CONTROL" == "1" ]] || { echo "错误: CONTROL 只能是 0 或 1" >&2; exit 1; }
[[ "$EPIC_ONLINE" == "0" || "$EPIC_ONLINE" == "1" ]] || { echo "错误: EPIC_ONLINE 只能是 0 或 1" >&2; exit 1; }
[[ "$START_RENDERER" == "0" || "$START_RENDERER" == "1" ]] || { echo "错误: START_RENDERER 只能是 0 或 1" >&2; exit 1; }
[[ "$START_SEMANTIC" == "0" || "$START_SEMANTIC" == "1" ]] || { echo "错误: START_SEMANTIC 只能是 0 或 1" >&2; exit 1; }
[[ "$SAVE_DEPTH_PNG" == "0" || "$SAVE_DEPTH_PNG" == "1" ]] || { echo "错误: SAVE_DEPTH_PNG 只能是 0 或 1" >&2; exit 1; }
[[ "$PLAN_FROM_REFERENCE" == "0" || "$PLAN_FROM_REFERENCE" == "1" ]] || { echo "错误: PLAN_FROM_REFERENCE 只能是 0 或 1" >&2; exit 1; }
[[ "$GRAPH_VISUALIZATION" == "0" || "$GRAPH_VISUALIZATION" == "1" ]] || { echo "错误: GRAPH_VISUALIZATION 只能是 0 或 1" >&2; exit 1; }

export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
set +u
source /opt/ros/humble/setup.bash
source "$ROS_INSTALL/setup.bash"
source "$WORKSPACE_INSTALL/setup.bash"
set -u
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PROJECT_ROOT/openseek${PYTHONPATH:+:$PYTHONPATH}"

require_single_publisher() {
  local topic="$1"
  local info=""
  local count="0"
  local attempt
  for ((attempt = 0; attempt < 20; attempt++)); do
    info="$(timeout 3 ros2 topic info "$topic" 2>/dev/null || true)"
    count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$info")"
    count="${count:-0}"
    if [[ "$count" == "1" ]]; then
      echo "输入检查通过: $topic 只有 1 个发布者"
      return 0
    fi
    if (( count > 1 )); then
      echo "错误: $topic 有 $count 个发布者，存在两套 AirSim/renderer，EPIC 位姿会被污染。" >&2
      ros2 topic info "$topic" --verbose >&2 || true
      echo "请在旧的 08/09/40 启动终端按 Ctrl-C，只保留 06 bridge，然后重新运行 40。" >&2
      return 1
    fi
    sleep 0.5
  done
  echo "错误: $topic 没有发布者。请确认 UE 已按 Play，且 06 bridge 正在运行。" >&2
  return 1
}

require_no_publishers() {
  local topic="$1"
  local info=""
  local count="0"
  info="$(timeout 3 ros2 topic info "$topic" 2>/dev/null || true)"
  count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$info")"
  count="${count:-0}"
  if (( count == 0 )); then
    return 0
  fi
  echo "错误: 启动本地 SO3 前，$topic 已有 $count 个发布者。" >&2
  ros2 topic info "$topic" --verbose >&2 || true
  echo "请停止 06 bridge 以及旧的 08/09/40；在线 SO3 模式只能由 40 提供这些话题。" >&2
  return 1
}

if [[ "$EPIC_ONLINE" == "1" ]]; then
  if [[ "$START_RENDERER" == "0" ]]; then
    require_single_publisher /sim/odom
    require_single_publisher /camera/depth/image
  else
    require_no_publishers /sim/odom
    require_no_publishers /camera/depth/image
  fi
  GOAL_TOPIC="$EPIC_NEXT_GOAL_TOPIC"
  GOAL_TOLERANCE="$EPIC_YOPO_GOAL_TOLERANCE"
  MISSION_GOAL_TOPIC="$EPIC_GLOBAL_GOAL_TOPIC"
fi

args=(
  "$PROJECT_ROOT/openseek/online_planner_ros2.py"
  --model "$MODEL"
  --device "$DEVICE"
  --search-distance "$SEARCH_DISTANCE"
  --heatmap-sigma "$HEATMAP_SIGMA"
  --depth-timeout "$DEPTH_TIMEOUT"
  --original-goal-input
  --goal-topic "$GOAL_TOPIC"
  --mission-goal-topic "$MISSION_GOAL_TOPIC"
  --mission-goal-tolerance "$MISSION_GOAL_TOLERANCE"
  --mission-stop-speed "$MISSION_STOP_SPEED"
  --final-subgoal-tolerance "$FINAL_SUBGOAL_TOLERANCE"
  --goal-tolerance "$GOAL_TOLERANCE"
  --lidar-topic "$LIDAR_TOPIC"
  --world-frame "$WORLD_FRAME"
  --max-yaw-rate "$MAX_YAW_RATE"
  --odom-twist-frame "$ODOM_TWIST_FRAME"
  --reference-reset-position-error "$REFERENCE_RESET_POSITION_ERROR"
  --reference-reset-velocity-error "$REFERENCE_RESET_VELOCITY_ERROR"
  --minimum-trajectory-altitude "$MINIMUM_TRAJECTORY_ALTITUDE"
  --altitude-margin "$TRAJECTORY_ALTITUDE_MARGIN"
  --model-image-width 160
  --model-image-height 96
  --model-vertical-num 3
  --direct-goal-distance "$DIRECT_GOAL_DISTANCE"
  --event-log-dir "$EVENT_LOG_DIR"
)
if [[ "$FIXED_ALTITUDE" == "1" ]]; then
  args+=(--fixed-altitude)
fi
if [[ "$PLAN_FROM_REFERENCE" == "1" ]]; then
  args+=(--plan-from-reference)
fi
if [[ "$GRAPH_VISUALIZATION" == "1" ]]; then
  args+=(--graph-visualization)
fi
if [[ "$CONTROL" == "1" ]]; then
  args+=(--control --control-topic "$COLOSSEUM_CONTROL_TOPIC")
fi
if [[ "$SAVE_DEPTH_PNG" == "1" ]]; then
  args+=(--save-depth-png)
fi
control_topic_display="${COLOSSEUM_CONTROL_TOPIC:-/openseek/trajectory_point}"
echo "启动 YOPO-Simple: control=$CONTROL renderer=$START_RENDERER odom_twist=$ODOM_TWIST_FRAME plan_from_reference=$PLAN_FROM_REFERENCE graph_visualization=$GRAPH_VISUALIZATION save_depth_png=$SAVE_DEPTH_PNG topic=$control_topic_display goal=$GOAL_TOPIC frame=$WORLD_FRAME log=$EVENT_LOG_DIR"

launch_pid=""
planner_pid=""
frgraph_pid=""
epic_pid=""
semantic_pid=""

# ros2 launch is a wrapper process and owns several child nodes.  Signalling
# only the wrapper leaves those nodes alive after Ctrl-C, which can make an
# old planner continue printing "waiting for ..." after the terminal prompt
# has returned.
kill_process_tree() {
  local root_pid="$1"
  local signal="$2"
  [[ -n "$root_pid" ]] || return 0
  kill -0 "$root_pid" 2>/dev/null || return 0
  local child_pid
  while read -r child_pid; do
    [[ -n "$child_pid" ]] || continue
    kill_process_tree "$child_pid" "$signal"
  done < <(pgrep -P "$root_pid" 2>/dev/null || true)
  kill -"$signal" "$root_pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  local pids=()
  for pid in "$planner_pid" "$semantic_pid" "$epic_pid" "$frgraph_pid" "$launch_pid"; do
    [[ -z "$pid" ]] || pids+=("$pid")
  done
  for pid in "${pids[@]}"; do
    kill_process_tree "$pid" INT
  done
  # Some Python/ROS processes handle SIGINT as a request to shut down and can
  # take a moment; do not leave a detached planner behind indefinitely.
  sleep 0.3
  for pid in "${pids[@]}"; do
    kill_process_tree "$pid" TERM
  done
  sleep 0.3
  for pid in "${pids[@]}"; do
    kill_process_tree "$pid" KILL
  done
  [[ -z "$planner_pid" ]] || wait "$planner_pid" 2>/dev/null || true
  [[ -z "$semantic_pid" ]] || wait "$semantic_pid" 2>/dev/null || true
  [[ -z "$epic_pid" ]] || wait "$epic_pid" 2>/dev/null || true
  [[ -z "$frgraph_pid" ]] || wait "$frgraph_pid" 2>/dev/null || true
  [[ -z "$launch_pid" ]] || wait "$launch_pid" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

if [[ "$EPIC_ONLINE" == "1" ]]; then
  ros2 launch openseek_frgraph_ros2 depth_planar_to_pointcloud.launch.py \
    > /tmp/openseek_frgraph_online.log 2>&1 &
  frgraph_pid=$!
  ros2 launch openseek_epic_ros2 epic_graph.launch.py \
    "goal_topic:=$EPIC_GLOBAL_GOAL_TOPIC" \
    "next_goal_topic:=$EPIC_NEXT_GOAL_TOPIC" \
    "visualization_frame:=$EPIC_VISUALIZATION_FRAME" \
    "odom_twist_frame:=${ODOM_TWIST_FRAME:-world}" \
    "flight_statistics_file:=$EPIC_FLIGHT_STATISTICS_FILE" \
    "trajectory_speed_color_max_mps:=$EPIC_TRAJECTORY_SPEED_COLOR_MAX_MPS" \
    "trajectory_max_points:=$EPIC_TRAJECTORY_MAX_POINTS" \
    "graph_fixed_layer:=$EPIC_GRAPH_FIXED_LAYER" \
    "graph_layer_z:=$EPIC_GRAPH_LAYER_Z" \
    "reuse_graph_on_goal:=$EPIC_REUSE_GRAPH_ON_GOAL" \
    "map_margin:=$EPIC_MAP_MARGIN" \
    "map_voxel_size:=$EPIC_MAP_VOXEL_SIZE" \
    "map_history_radius_m:=$EPIC_MAP_HISTORY_RADIUS_M" \
    "map_max_points:=$EPIC_MAP_MAX_POINTS" \
    "map_prune_distance_m:=$EPIC_MAP_PRUNE_DISTANCE_M" \
    "update_period_ms:=$EPIC_UPDATE_PERIOD_MS" \
    "skeleton_rebuild_period_ms:=$EPIC_SKELETON_REBUILD_PERIOD_MS" \
    "local_goal_min_advance_m:=$EPIC_LOCAL_GOAL_MIN_ADVANCE_M" \
    "local_goal_lookahead_m:=$EPIC_LOCAL_GOAL_LOOKAHEAD_M" \
    "route_plan_period_ms:=$EPIC_ROUTE_PLAN_PERIOD_MS" \
    "local_goal_reserve_m:=$EPIC_LOCAL_GOAL_RESERVE_M" \
    "use_edge_witness_path:=$EPIC_USE_EDGE_WITNESS_PATH" \
    "raycast_shortcut_sample_step_m:=$EPIC_RAYCAST_SHORTCUT_SAMPLE_STEP_M" \
    "raycast_shortcut_clearance_margin_m:=$EPIC_RAYCAST_SHORTCUT_CLEARANCE_MARGIN_M" \
    "goal_path_cost_weight:=$EPIC_GOAL_PATH_COST_WEIGHT" \
    "semantic_cost_weight:=$EPIC_SEMANTIC_COST_WEIGHT" \
    "semantic_node_ema_alpha:=$EPIC_SEMANTIC_NODE_EMA_ALPHA" \
    "semantic_visualization_max_score:=$EPIC_SEMANTIC_VISUALIZATION_MAX_SCORE" \
    "semantic_association_radius_m:=$EPIC_SEMANTIC_ASSOCIATION_RADIUS_M" \
    "semantic_depth_clip_m:=$EPIC_SEMANTIC_DEPTH_CLIP_M" \
    "semantic_depth_sync_tolerance_ms:=${EPIC_SEMANTIC_DEPTH_SYNC_TOLERANCE_MS:-250.0}" \
    "semantic_patch_cols:=$EPIC_SEMANTIC_PATCH_COLS" \
    "semantic_patch_rows:=$EPIC_SEMANTIC_PATCH_ROWS" \
    "speculative_enabled:=${EPIC_SPECULATIVE_ENABLED:-true}" \
    "speculative_min_score:=${EPIC_SPECULATIVE_MIN_SCORE:-0.35}" \
    "speculative_forward_m:=${EPIC_SPECULATIVE_FORWARD_M:-22.0}" \
    "speculative_patch_separation_m:=${EPIC_SPECULATIVE_PATCH_SEPARATION_M:-1.5}" \
    "speculative_radius_m:=${EPIC_SPECULATIVE_RADIUS_M:-0.75}" \
    "speculative_max_nodes:=${EPIC_SPECULATIVE_MAX_NODES:-16}" \
    "bubble_topo/clearance_cost_weight:=$EPIC_CLEARANCE_COST_WEIGHT" \
    "bubble_topo/clearance_target_m:=$EPIC_CLEARANCE_TARGET_M" \
    "previous_path_cost_factor:=$EPIC_PREVIOUS_PATH_COST_FACTOR" \
    "route_remap_distance_m:=$EPIC_ROUTE_REMAP_DISTANCE_M" \
    "route_reuse_horizon_m:=$EPIC_ROUTE_REUSE_HORIZON_M" \
    "route_reuse_lateral_distance_m:=$EPIC_ROUTE_REUSE_LATERAL_DISTANCE_M" \
    "route_terminal_release_distance_m:=$EPIC_ROUTE_TERMINAL_RELEASE_DISTANCE_M" \
    "goal_connect_distance_m:=$EPIC_GOAL_CONNECT_DISTANCE_M" \
    "goal_connect_timeout_ms:=$EPIC_GOAL_CONNECT_TIMEOUT_MS" \
    "odom_reconnect_distance_m:=$EPIC_ODOM_RECONNECT_DISTANCE_M" \
    "odom_reconnect_yaw_deg:=$EPIC_ODOM_RECONNECT_YAW_DEG" \
    "odom_fallback_radius_m:=$EPIC_ODOM_FALLBACK_RADIUS_M" \
    "odom_fallback_candidates:=$EPIC_ODOM_FALLBACK_CANDIDATES" \
    "odom_connect_timeout_ms:=$EPIC_ODOM_CONNECT_TIMEOUT_MS" \
    > /tmp/openseek_epic_online.log 2>&1 &
  epic_pid=$!
  echo "EPIC 在线链路已启动: global=$EPIC_GLOBAL_GOAL_TOPIC next=$EPIC_NEXT_GOAL_TOPIC update=${EPIC_UPDATE_PERIOD_MS}ms rebuild=${EPIC_SKELETON_REBUILD_PERIOD_MS}ms"
  if [[ "$START_SEMANTIC" == "1" ]]; then
    "$PYTHON" "$PROJECT_ROOT/openseek/text_heatmap_ros2.py" \
      --prompt "$SEMANTIC_PROMPT" \
      --input-topic /camera/color/image \
      --output-topic /openseek/text_heatmap \
      --device "$DEVICE" \
      --update-rate "$SEMANTIC_UPDATE_RATE" \
      --pearl-root "$PROJECT_ROOT/third_party/PEARL" \
      > /tmp/openseek_text_heatmap_online.log 2>&1 &
    semantic_pid=$!
    echo "PEARL 语义融合已启动: prompt=$SEMANTIC_PROMPT rate=${SEMANTIC_UPDATE_RATE}Hz"
  fi
fi

if [[ "$START_RENDERER" == "1" ]]; then
  ros2 launch openseek_airsim_renderer controller_airsim.launch.py &
  launch_pid=$!
else
  echo "使用外部 AirSim/Colosseum bridge，不启动本地 renderer。"
fi
"$PYTHON" "${args[@]}" "$@" &
planner_pid=$!

set +e
critical_pids=("$planner_pid")
for pid in "$launch_pid" "$frgraph_pid" "$epic_pid" "$semantic_pid"; do
  [[ -z "$pid" ]] || critical_pids+=("$pid")
done
wait -n "${critical_pids[@]}"
status=$?
set -e
exit "$status"
