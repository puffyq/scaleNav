#!/usr/bin/env bash
set -Eeuo pipefail

COLOSSEUM_ROOT="${COLOSSEUM_ROOT:-/mnt/code/lab/airsim/Colosseum}"
[[ -f "$COLOSSEUM_ROOT/build.sh" ]] || { echo "错误: 找不到 $COLOSSEUM_ROOT/build.sh" >&2; exit 1; }
[[ -f "$COLOSSEUM_ROOT/ColosseumLib/src/api/RpcLibClientBase.cpp" ]] || {
  echo "错误: ColosseumLib/src 源码不完整。请在 Colosseum 目录执行：" >&2
  echo "  git restore --source=HEAD -- ColosseumLib/src" >&2
  exit 1
}
command -v cmake >/dev/null || { echo "错误: 未找到 cmake" >&2; exit 1; }
command -v make >/dev/null || { echo "错误: 未找到 make" >&2; exit 1; }

cd -- "$COLOSSEUM_ROOT"
echo "编译 Colosseum native libraries（不要使用 sudo）"
if [[ "${CLEAN_BUILD:-1}" == "1" ]]; then
  echo "清理旧的 CMake build 缓存"
  bash ./clean.sh
elif [[ "${CLEAN_BUILD:-1}" != "0" ]]; then
  echo "错误: CLEAN_BUILD 只能是 0 或 1。" >&2
  exit 1
fi
exec bash ./build.sh
