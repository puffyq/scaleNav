#!/usr/bin/env bash
set -Eeuo pipefail

# Single entry point for the AirSim + EPIC + YOPO online chain.
# Configuration is local to the child process; it is not exported into the
# caller's shell or persisted in a shell startup file.
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PROMPT="blocks"
SEMANTIC_RATE="2"
START_SEMANTIC="1"

while (($# > 0)); do
  case "$1" in
    --prompt)
      (($# >= 2)) || { echo "--prompt requires a value" >&2; exit 2; }
      PROMPT="$2"
      shift 2
      ;;
    --rate)
      (($# >= 2)) || { echo "--rate requires a value" >&2; exit 2; }
      SEMANTIC_RATE="$2"
      shift 2
      ;;
    --no-semantic)
      START_SEMANTIC="0"
      shift
      ;;
    -h|--help)
      printf 'Usage: %s [--prompt TEXT] [--rate HZ] [--no-semantic]\n' "$0"
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      exit 2
      ;;
  esac
done

echo "Starting online ScaleNav: prompt=${PROMPT@Q}, PEARL=${START_SEMANTIC}, rate=${SEMANTIC_RATE}Hz"
echo "UE/AirSim must already be running and set to the selected map."

exec env \
  CONTROL=1 \
  EPIC_ONLINE=1 \
  START_RENDERER=1 \
  START_SEMANTIC="$START_SEMANTIC" \
  SEMANTIC_PROMPT="$PROMPT" \
  SEMANTIC_UPDATE_RATE="$SEMANTIC_RATE" \
  DEVICE=cuda \
  GRAPH_VISUALIZATION=0 \
  PLAN_FROM_REFERENCE=1 \
  SAVE_DEPTH_PNG=0 \
  ODOM_TWIST_FRAME=body \
  REFERENCE_RESET_POSITION_ERROR=0.75 \
  REFERENCE_RESET_VELOCITY_ERROR=1.5 \
  MINIMUM_TRAJECTORY_ALTITUDE=0.15 \
  TRAJECTORY_ALTITUDE_MARGIN=0.10 \
  FIXED_ALTITUDE=1 \
  DIRECT_GOAL_DISTANCE=3.5 \
  MISSION_GOAL_TOLERANCE=0.5 \
  MISSION_STOP_SPEED=0.3 \
  FINAL_SUBGOAL_TOLERANCE=0.25 \
  EPIC_UPDATE_PERIOD_MS=200 \
  EPIC_SKELETON_REBUILD_PERIOD_MS=1000.0 \
  EPIC_MAP_VOXEL_SIZE=0.25 \
  EPIC_MAP_HISTORY_RADIUS_M=20.0 \
  EPIC_MAP_MAX_POINTS=20000 \
  EPIC_MAP_PRUNE_DISTANCE_M=0.5 \
  EPIC_LOCAL_GOAL_MIN_ADVANCE_M=0.75 \
  EPIC_LOCAL_GOAL_LOOKAHEAD_M=10.0 \
  EPIC_ROUTE_PLAN_PERIOD_MS=2000 \
  EPIC_LOCAL_GOAL_RESERVE_M=5.0 \
  EPIC_USE_EDGE_WITNESS_PATH=false \
  EPIC_RAYCAST_SHORTCUT_SAMPLE_STEP_M=0.25 \
  EPIC_RAYCAST_SHORTCUT_CLEARANCE_MARGIN_M=0.05 \
  EPIC_GOAL_PATH_COST_WEIGHT=0.2 \
  EPIC_SEMANTIC_COST_WEIGHT=1.0 \
  EPIC_SEMANTIC_NODE_EMA_ALPHA=0.3 \
  EPIC_PREVIOUS_PATH_COST_FACTOR=0.0 \
  EPIC_ROUTE_REMAP_DISTANCE_M=1.25 \
  EPIC_ROUTE_REUSE_HORIZON_M=6.0 \
  EPIC_ROUTE_REUSE_LATERAL_DISTANCE_M=1.5 \
  EPIC_ROUTE_TERMINAL_RELEASE_DISTANCE_M=1.0 \
  EPIC_GOAL_CONNECT_DISTANCE_M=6.0 \
  EPIC_GOAL_CONNECT_TIMEOUT_MS=20.0 \
  EPIC_ODOM_RECONNECT_DISTANCE_M=1.0 \
  EPIC_ODOM_RECONNECT_YAW_DEG=20.0 \
  EPIC_ODOM_FALLBACK_RADIUS_M=15.0 \
  EPIC_ODOM_FALLBACK_CANDIDATES=8 \
  EPIC_ODOM_CONNECT_TIMEOUT_MS=3.0 \
  EPIC_YOPO_GOAL_TOLERANCE=0.5 \
  EPIC_GLOBAL_GOAL_TOPIC=/goal_pose \
  EPIC_NEXT_GOAL_TOPIC=/epic/yopo_goal \
  EPIC_VISUALIZATION_FRAME=world_enu \
  EPIC_GRAPH_FIXED_LAYER=true \
  EPIC_GRAPH_LAYER_Z=1.6 \
  COLOSSEUM_CONTROL_TOPIC= \
  bash "$ROOT_DIR/scripts/08_start_openseek_planner.sh"
