#!/usr/bin/env bash
set -Eeuo pipefail

# ScaleNav online defaults. Select Map2 or Map4 in UE before pressing Play.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export START_YOPO=1
export YOPO_CONTROL=1
export CONTROL=1
export EPIC_ONLINE=1
export START_RENDERER=1
export START_SEMANTIC="${START_SEMANTIC:-1}"
export SEMANTIC_PROMPT="${SEMANTIC_PROMPT:-tree}"
export SEMANTIC_UPDATE_RATE="${SEMANTIC_UPDATE_RATE:-5}"
export DEVICE=cuda
# EPIC already owns the online graph and RViz output. The legacy Python graph
# costs hundreds of milliseconds per depth frame and starves the 50 Hz control loop.
export GRAPH_VISUALIZATION=0
export PLAN_FROM_REFERENCE=1
export SAVE_DEPTH_PNG=0
export ODOM_TWIST_FRAME=body
export REFERENCE_RESET_POSITION_ERROR=0.75
export REFERENCE_RESET_VELOCITY_ERROR=1.5
export MINIMUM_TRAJECTORY_ALTITUDE=0.15
export TRAJECTORY_ALTITUDE_MARGIN=0.10
export FIXED_ALTITUDE=1
export DIRECT_GOAL_DISTANCE=3.5
export MISSION_GOAL_TOLERANCE=0.5
export MISSION_STOP_SPEED=0.3
export FINAL_SUBGOAL_TOLERANCE=0.25

# Conservative first online run: rolling graph search at 5 Hz, Bubble rebuild at 1 Hz.
# YOPO follows the incoming RGB-D rate and its 50 Hz controller runs separately.
export EPIC_UPDATE_PERIOD_MS=200
export EPIC_SKELETON_REBUILD_PERIOD_MS=1000.0
export EPIC_MAP_VOXEL_SIZE=0.25
export EPIC_MAP_HISTORY_RADIUS_M=20.0
export EPIC_MAP_MAX_POINTS=20000
export EPIC_MAP_PRUNE_DISTANCE_M=0.5
export EPIC_LOCAL_GOAL_MIN_ADVANCE_M=0.75
export EPIC_LOCAL_GOAL_LOOKAHEAD_M=5.0
# Persistent rolling search over real EPIC TopoNodes. Previous route edges are
# discounted inside the graph search so the chosen side of a building remains committed.
export EPIC_GOAL_PATH_COST_WEIGHT=0.2
export EPIC_PREVIOUS_PATH_COST_FACTOR=0.0
export EPIC_ROUTE_REMAP_DISTANCE_M=1.25
export EPIC_ROUTE_REUSE_HORIZON_M=6.0
export EPIC_ROUTE_REUSE_LATERAL_DISTANCE_M=1.5
export EPIC_ROUTE_TERMINAL_RELEASE_DISTANCE_M=1.0
export EPIC_GOAL_CONNECT_DISTANCE_M=6.0
export EPIC_GOAL_CONNECT_TIMEOUT_MS=20.0
export EPIC_ODOM_RECONNECT_DISTANCE_M=1.0
export EPIC_ODOM_RECONNECT_YAW_DEG=20.0
export EPIC_ODOM_FALLBACK_RADIUS_M=15.0
export EPIC_ODOM_FALLBACK_CANDIDATES=24
export EPIC_YOPO_GOAL_TOLERANCE=0.5
export EPIC_GLOBAL_GOAL_TOPIC=/goal_pose
export EPIC_NEXT_GOAL_TOPIC=/epic/yopo_goal
export EPIC_VISUALIZATION_FRAME=world_enu
export EPIC_GRAPH_FIXED_LAYER=true
export EPIC_GRAPH_LAYER_Z=1.6
export COLOSSEUM_CONTROL_TOPIC=

# UE/AirSim is external; this process owns SO3 dynamics, RGB-D, EPIC and YOPO.
echo "在线 ScaleNav: YOPO -> SO3 -> /sim/odom -> AirSim renderer"
echo "本脚本不会启动 UE，也不使用官方 Colosseum bridge。"
echo "请先运行 05_open_blocks_v2.sh，并在 UE 中按 Play。"
echo "频率: 滚动全局路径 5 Hz, Bubble 后台重建 1 Hz, YOPO 路径前视 5.0 m, PEARL ${SEMANTIC_UPDATE_RATE} Hz prompt=${SEMANTIC_PROMPT}"
exec bash "${SCRIPT_DIR}/08_start_openseek_planner.sh" "$@"
