#!/usr/bin/env bash
set -Eeuo pipefail

UE_ROOT="${UE_ROOT:-/mnt/code/lab/ue5_7/Linux_Unreal_Engine_5.7.1}"
BLOCKS_PROJECT="${BLOCKS_PROJECT:-/mnt/code/lab/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject}"
UE_BUILD_SCRIPT="$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh"
[[ -x "$UE_BUILD_SCRIPT" ]] || { echo "错误: 找不到 UE Build.sh: $UE_BUILD_SCRIPT" >&2; exit 1; }
[[ -f "$BLOCKS_PROJECT" ]] || { echo "错误: 找不到 BlocksV2 工程: $BLOCKS_PROJECT" >&2; exit 1; }

cd -- "$UE_ROOT"
exec "$UE_BUILD_SCRIPT" BlocksV2Editor Linux Development \
  "-Project=$BLOCKS_PROJECT" -NoHotReloadFromIDE
