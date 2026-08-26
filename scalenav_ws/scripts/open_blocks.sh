#!/usr/bin/env bash
set -Eeuo pipefail

UE_ROOT="${UE_ROOT:-/mnt/code/lab/ue5_7/Linux_Unreal_Engine_5.7.1}"
BLOCKS_PROJECT="${BLOCKS_PROJECT:-/mnt/code/lab/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject}"
UE_EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor"
BUILD="${BUILD:-0}"
[[ -x "$UE_EDITOR" ]] || { echo "错误: 找不到 UnrealEditor: $UE_EDITOR" >&2; exit 1; }
[[ -f "$BLOCKS_PROJECT" ]] || { echo "错误: 找不到 BlocksV2 工程: $BLOCKS_PROJECT" >&2; exit 1; }
if [[ "$BUILD" == "1" ]]; then
  bash "$(dirname -- "$0")/04_build_blocks_v2.sh"
elif [[ "$BUILD" != "0" ]]; then
  echo "错误: BUILD 只能是 0 或 1。" >&2
  exit 1
fi

cd -- "$UE_ROOT"
echo "启动 BlocksV2；进入编辑器后按 Play。"
exec "$UE_EDITOR" "$BLOCKS_PROJECT" "$@"
