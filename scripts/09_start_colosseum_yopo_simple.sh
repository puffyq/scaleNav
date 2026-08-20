#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${BUILD:-0}"
SETUP_SETTINGS="${SETUP_SETTINGS:-1}"
RPC_IP="${RPC_IP:-127.0.0.1}"
RPC_PORT="${RPC_PORT:-41451}"
RPC_TIMEOUT="${RPC_TIMEOUT:-180}"
START_YOPO="${START_YOPO:-0}"
YOPO_CONTROL="${YOPO_CONTROL:-0}"
ue_pid=""
bridge_pid=""

if [[ "$SETUP_SETTINGS" == "1" ]]; then bash "$SCRIPT_DIR/01_setup_colosseum_settings.sh"; fi
if [[ "$BUILD" == "1" ]]; then
  bash "$SCRIPT_DIR/02_build_colosseum.sh"
  bash "$SCRIPT_DIR/03_build_colosseum_ros2.sh"
  bash "$SCRIPT_DIR/04_build_blocks_v2.sh"
fi
[[ "$START_YOPO" == "0" || "$START_YOPO" == "1" ]] || { echo "错误: START_YOPO 只能是 0 或 1" >&2; exit 1; }
[[ "$YOPO_CONTROL" == "0" || "$YOPO_CONTROL" == "1" ]] || { echo "错误: YOPO_CONTROL 只能是 0 或 1" >&2; exit 1; }
cleanup() {
  local status=$?
  if [[ -n "$bridge_pid" ]] && kill -0 "$bridge_pid" 2>/dev/null; then kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; fi
  if [[ -n "$ue_pid" ]] && kill -0 "$ue_pid" 2>/dev/null; then kill "$ue_pid" 2>/dev/null || true; wait "$ue_pid" 2>/dev/null || true; fi
  exit "$status"
}
trap cleanup EXIT INT TERM

UE_ROOT="${UE_ROOT:-/mnt/code/lab/ue5_7/Linux_Unreal_Engine_5.7.1}"
BLOCKS_PROJECT="${BLOCKS_PROJECT:-/mnt/code/lab/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject}"
UE_EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor"
"$UE_EDITOR" "$BLOCKS_PROJECT" -log & ue_pid=$!
echo "UE 已启动，请在编辑器中按 Play；等待 RPC $RPC_IP:$RPC_PORT（${RPC_TIMEOUT}s）"
deadline=$((SECONDS + RPC_TIMEOUT))
while (( SECONDS < deadline )); do
  kill -0 "$ue_pid" 2>/dev/null || { echo "错误: UE 提前退出" >&2; exit 1; }
  if (echo >/dev/tcp/"$RPC_IP"/"$RPC_PORT") 2>/dev/null; then
    if [[ "$START_YOPO" == "1" ]]; then
      bash "$SCRIPT_DIR/06_start_colosseum_ros2.sh" &
      bridge_pid=$!
      sleep 3
      CONTROL="$YOPO_CONTROL" bash "$SCRIPT_DIR/08_start_openseek_planner.sh" "$@"
      exit $?
    fi
    exec bash "$SCRIPT_DIR/06_start_colosseum_ros2.sh" "$@"
  fi
  sleep 1
done
echo "错误: RPC 等待超时，请确认 UE 已按 Play。" >&2
exit 1
