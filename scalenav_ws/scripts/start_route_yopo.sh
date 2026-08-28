#!/usr/bin/env bash
set -Eeo pipefail

PROMPT="tree, blocks, wall"
SEMANTIC=1
SEMANTIC_COST_WEIGHT="2.0"
GRAPH_FIXED_LAYER="${GRAPH_FIXED_LAYER:-true}"
DEVICE="${DEVICE:-auto}"
RATE="2"
ATTACH=0
LOG_ROOT=""
IGNORE_COLLISION="${IGNORE_COLLISION:-false}"

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"
SRC="$WS/src"
TRAIN_ROOT="$ROOT/train_scalenav"
PYTHON="$ROOT/../YOPO-Rally/.venv/bin/python"
MODEL="$TRAIN_ROOT/saved_corrected/YOPO_5/best.pth"
LOG_ROOT="${LOG_ROOT:-${SCALENAV_LOG_DIR:-$ROOT/log_scalenav}}"

while (($#)); do
  case "$1" in
    --attach) ATTACH=1; shift ;;
    --prompt)
      if (($# < 2)); then echo "--prompt needs a value" >&2; exit 2; fi
      PROMPT="$2"; shift 2
      ;;
    --rate)
      if (($# < 2)); then echo "--rate needs a value" >&2; exit 2; fi
      RATE="$2"; shift 2
      ;;
    --device)
      if (($# < 2)); then echo "--device needs auto, cpu, or cuda" >&2; exit 2; fi
      DEVICE="$2"; shift 2
      ;;
    --no-semantic) SEMANTIC=0; shift ;;
    -h|--help)
      echo "Usage: $0 [--attach] [--device auto|cpu|cuda] [--prompt TEXT] [--rate HZ] [--no-semantic]"
      echo ""
      echo "Default: start simulator, depth adapter, ScaleNav, semantic perception, and Route-YOPO control."
      echo "--attach: attach Route-YOPO control to an existing ROS2/ScaleNav session. Stop the old planner first."
      echo "Control output: /scalenav/trajectory_point at 50 Hz. This script does not invoke scripts/start.sh."
      exit 0
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $SCRIPT_DIR/build.sh first" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "Route-YOPO checkpoint not found: $MODEL" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="$TRAIN_ROOT:$SRC/scalenav:$SRC:${PYTHONPATH:-}"

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

if ((ATTACH == 0)); then
  run ros2 launch scalenav_log scalenav_log.launch.py output_dir:="$LOG_ROOT"
  run ros2 launch airsim_renderer controller_airsim.launch.py \
    ignore_collision:="$IGNORE_COLLISION"
  run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py
  run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
    graph_fixed_layer:="$GRAPH_FIXED_LAYER" \
    goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
    next_goal_frame:=world_enu visualization_frame:=world_enu \
    odom_twist_frame:=body semantic_heatmap_topic:=/scalenav/text_heatmap_raw \
    flight_statistics_file:=/dev/null graph_log_file:=/dev/null \
    semantic_cost_weight:="$SEMANTIC_COST_WEIGHT"

  if ((SEMANTIC)); then
    run "$PYTHON" "$SRC/scalenav/text_heatmap_ros2.py" \
      --prompt "$PROMPT" --input-topic /camera/color/image \
      --output-topic /scalenav/text_heatmap --device cuda \
      --update-rate "$RATE" --pearl-root "$SRC/global_graph/heatmap_ws/pearl_ws"
  fi
fi

run "$PYTHON" "$SRC/scalenav/route_yopo_control_ros2.py" \
  --model "$MODEL" --train-root "$TRAIN_ROOT" --device "$DEVICE" \
  --odom-twist-frame body --world-frame world_enu

echo "started Route-YOPO control; model=$MODEL attach=$ATTACH control_output=/scalenav/trajectory_point@50Hz"
echo "status=/scalenav/route_yopo/status path=/scalenav/route_yopo/planned_path"
wait -n $PIDS
