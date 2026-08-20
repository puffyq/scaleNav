#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
AIRSIM_ROOT="${AIRSIM_ROOT:-/mnt/code/lab/airsim/Colosseum/PythonClient}"
OUTPUT="${OUTPUT:-$PROJECT_ROOT/data/Map2GraphData}"
COUNT="${COUNT:-20}"
SEED="${SEED:-2201}"
OVERWRITE="${OVERWRITE:-0}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$AIRSIM_ROOT" ]] || { echo "错误: Colosseum PythonClient 不存在: $AIRSIM_ROOT" >&2; exit 1; }
[[ "$OVERWRITE" == "0" || "$OVERWRITE" == "1" ]] || { echo "错误: OVERWRITE 只能是 0 或 1" >&2; exit 1; }
if [[ -d "$OUTPUT/Scene_0001" && "$OVERWRITE" == "0" ]]; then
  echo "错误: Map2 Graph 数据已存在；覆盖时设置 OVERWRITE=1" >&2
  exit 1
fi

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1
args=()
[[ "$OVERWRITE" == "0" ]] || args+=(--overwrite)
echo "请确认 UE 正在运行 FlyingExampleMapV2，且 AirSim RPC 41451 已连接。"
exec "$PYTHON" -m openseek.data.snapshot_dataset \
  --output "$OUTPUT" --scene-id 0001 --count "$COUNT" --seed "$SEED" \
  --prompt person --export-static-meshes --airsim-root "$AIRSIM_ROOT" "${args[@]}"
