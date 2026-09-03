#!/usr/bin/env bash
set -Eeo pipefail

# ── Tuning ─────────────────────────────────────────────────────────────────
PROMPT="${PROMPT:-tree, blocks, wall}"   # PEARL prompt (higher score = higher risk)
SEMANTIC=1                    # 1=text heatmap on, 0=geometry only
SEMANTIC_COST_WEIGHT="${SEMANTIC_COST_WEIGHT:-2.0}"    # A* semantic repulsion; 0=off
SEMANTIC_ROUTE_INFLUENCE_M="${SEMANTIC_ROUTE_INFLUENCE_M:-8.0}"
SEMANTIC_POINT_INFLUENCE_M="${SEMANTIC_POINT_INFLUENCE_M:-8.0}"
FRONTIER_GOAL_DISTANCE_WEIGHT="${FRONTIER_GOAL_DISTANCE_WEIGHT:-2.0}"
WAIT_FOR_INITIAL_SEMANTIC="${WAIT_FOR_INITIAL_SEMANTIC:-true}"
GRAPH_FIXED_LAYER="${GRAPH_FIXED_LAYER:-true}"  # graph topology: true=single layer, false=3D
DEVICE="cuda"
RATE="2"                      # heatmap Hz
SAVE_DEPTH=0
IGNORE_COLLISION="${IGNORE_COLLISION:-false}"
AIRSIM_RESET_ON_START="${AIRSIM_RESET_ON_START:-true}"
AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
LOG_ROOT=""                   # empty → $SCALENAV_LOG_DIR or ws/../log_scalenav
# ───────────────────────────────────────────────────────────────────────────

# Standalone online entry point. All project files are below this workspace.
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SRC="$WS/src"
LOG_ROOT="${LOG_ROOT:-${SCALENAV_LOG_DIR:-$WS/../log_scalenav}}"

PYTHON="$WS/../../YOPO-Rally/.venv/bin/python"
MODEL="$SRC/models/original_yopo_simple/model.pt"
CONFIG="$SRC/config/config.yaml"

while (($#)); do
  case "$1" in
    --prompt)
      if (($# < 2)); then echo "--prompt needs a value" >&2; exit 2; fi
      PROMPT="$2"
      shift 2
      ;;
    --rate)
      if (($# < 2)); then echo "--rate needs a value" >&2; exit 2; fi
      RATE="$2"
      shift 2
      ;;
    --semantic-cost-weight)
      if (($# < 2)); then echo "--semantic-cost-weight needs a value" >&2; exit 2; fi
      SEMANTIC_COST_WEIGHT="$2"
      shift 2
      ;;
    --semantic-route-influence-m)
      if (($# < 2)); then echo "--semantic-route-influence-m needs a value" >&2; exit 2; fi
      SEMANTIC_ROUTE_INFLUENCE_M="$2"
      shift 2
      ;;
    --semantic-point-influence-m)
      if (($# < 2)); then echo "--semantic-point-influence-m needs a value" >&2; exit 2; fi
      SEMANTIC_POINT_INFLUENCE_M="$2"
      shift 2
      ;;
    --no-semantic) SEMANTIC=0; shift ;;
    --capture-depth) SAVE_DEPTH=1; shift ;;
    -h|--help)
      echo "Usage: $0 [--prompt TEXT] [--rate HZ] [--no-semantic] [--capture-depth]"
      echo "       [--semantic-cost-weight VALUE] [--semantic-route-influence-m VALUE]"
      echo "       [--semantic-point-influence-m VALUE]"
      echo "       GRAPH_FIXED_LAYER=true|false $0 ... (default: true)"
      echo "       SCALENAV_LOG_DIR=/path/to/logs $0 ... (default: $WS/../log_scalenav)"
      exit 0
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# A geometry-only run has no heatmap producer, so it must bypass the semantic
# startup gate. Semantic runs keep the default one-shot wait enabled.
if ((SEMANTIC == 0)); then
  WAIT_FOR_INITIAL_SEMANTIC=false
fi

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $SCRIPT_DIR/build.sh first" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "model not found: $MODEL" >&2; exit 1; }
[[ -f "$CONFIG" ]] || { echo "config not found: $CONFIG" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="$SRC/scalenav:$SRC:$PYTHONPATH"
MAXIMUM_TRAJECTORY_SPEED_MPS="$(
  "$PYTHON" -c \
    'import sys; from trajectory_timing import load_maximum_trajectory_speed; print(load_maximum_trajectory_speed(sys.argv[1]))' \
    "$CONFIG"
)"

if [[ "$AIRSIM_RESET_ON_START" == "true" ]]; then
  "$PYTHON" "$SCRIPT_DIR/reset_airsim.py" \
    --host "$AIRSIM_HOST" --port "$AIRSIM_PORT"
fi

PIDS=""
LAST_PID=""
run() {
  setsid stdbuf -oL -eL "$@" &
  LAST_PID="$!"
  PIDS="$PIDS $LAST_PID"
}
stop() {
  trap - EXIT INT TERM
  local pid
  for pid in $PIDS; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 0.5
  for pid in $PIDS; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap stop EXIT INT TERM

run ros2 launch scalenav_log scalenav_log.launch.py output_dir:="$LOG_ROOT"
run ros2 launch airsim_renderer controller_airsim.launch.py \
  maximum_linear_speed:="$MAXIMUM_TRAJECTORY_SPEED_MPS" \
  ignore_collision:="$IGNORE_COLLISION"
run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py
run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
  graph_fixed_layer:="$GRAPH_FIXED_LAYER" \
  goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
  next_goal_frame:=world_enu visualization_frame:=world_enu \
  odom_twist_frame:=body semantic_heatmap_topic:=/scalenav/text_heatmap_raw \
  flight_statistics_file:=/dev/null graph_log_file:=/dev/null \
  trajectory_speed_color_max_mps:="$MAXIMUM_TRAJECTORY_SPEED_MPS" \
  semantic_cost_weight:="$SEMANTIC_COST_WEIGHT" \
  semantic_route_influence_m:="$SEMANTIC_ROUTE_INFLUENCE_M" \
  semantic_point_influence_m:="$SEMANTIC_POINT_INFLUENCE_M" \
  wait_for_initial_semantic:="$WAIT_FOR_INITIAL_SEMANTIC" \
  frontier_goal_distance_weight:="$FRONTIER_GOAL_DISTANCE_WEIGHT"

if ((SEMANTIC)); then
  run "$PYTHON" "$SRC/scalenav/text_heatmap_ros2.py" \
    --prompt "$PROMPT" --input-topic /camera/color/image \
    --output-topic /scalenav/text_heatmap --device "$DEVICE" \
    --update-rate "$RATE" --pearl-root "$SRC/global_graph/heatmap_ws/pearl_ws"
fi

start_planner() {
  run "$PYTHON" "$SRC/scalenav/online_planner_ros2.py" \
    --model "$MODEL" --device "$DEVICE" \
    --config-file "$CONFIG" \
    --control --original-goal-input --goal-topic /scalenav/local_goal \
    --mission-goal-topic /goal_pose --world-frame world_enu --odom-twist-frame body \
    --model-image-width 160 --model-image-height 96 --model-vertical-num 3 \
    --trajectory-speed-color-max-mps "$MAXIMUM_TRAJECTORY_SPEED_MPS" \
    --fixed-altitude --disable-event-log "$@"
}

if ((SAVE_DEPTH)); then
  start_planner --save-depth-png
else
  start_planner
fi

echo "started; goal=/goal_pose semantic=$SEMANTIC prompt=$PROMPT semantic_cost_weight=$SEMANTIC_COST_WEIGHT semantic_route_influence_m=$SEMANTIC_ROUTE_INFLUENCE_M semantic_point_influence_m=$SEMANTIC_POINT_INFLUENCE_M graph_fixed_layer=$GRAPH_FIXED_LAYER maximum_trajectory_speed_mps=$MAXIMUM_TRAJECTORY_SPEED_MPS ignore_collision=$IGNORE_COLLISION airsim_reset=$AIRSIM_RESET_ON_START recorded_logs=$LOG_ROOT"
wait -n $PIDS
