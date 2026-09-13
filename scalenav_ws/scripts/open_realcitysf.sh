#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
UE_LAUNCHER="${UE_LAUNCHER:-/mnt/code/lab/airsim/Colosseum/Unreal/Environments/BlocksV2/open_realcitysf.sh}"
MAP="${REALCITYSF_MAP:-/Game/RealCitySF/Maps/San_Francisco_sunny_01}"

usage() {
  cat <<EOF
用法: $0 [地图路径] [UnrealEditor 参数...]

默认地图:
  $MAP

环境变量:
  REALCITYSF_DAYLIGHT=true|false  城市地图加亮，默认 true
  REALCITYSF_UNLIT=true|false    使用 Unlit 应急显示，默认 false
  REALCITYSF_NULLRHI=true|false  无渲染诊断，默认 false
  UE_LAUNCHER=PATH                覆盖 UE 启动脚本路径
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (($# >= 1)) && [[ "$1" == /Game/* ]]; then
  MAP="$1"
  shift
fi

[[ -x "$UE_LAUNCHER" ]] || {
  echo "错误: 找不到 UE 启动脚本: $UE_LAUNCHER" >&2
  exit 1
}

echo "打开 RealCitySF: $MAP"
exec "$UE_LAUNCHER" "$MAP" "$@"

