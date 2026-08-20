#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DESTINATION="${DESTINATION:-$PROJECT_ROOT/third_party/repro}"

mkdir -p "$DESTINATION"

fetch_repo() {
  local name="$1"
  local url="$2"
  local branch="$3"
  local commit="$4"
  local marker="$5"
  local target="$DESTINATION/$name"

  if [[ -f "$target/$marker" ]]; then
    echo "已存在: $name ($(git -C "$target" rev-parse --short HEAD 2>/dev/null || echo archive))"
    return
  fi
  if [[ -d "$target" ]]; then
    rmdir "$target" 2>/dev/null || {
      echo "错误: $target 已存在但不是完整仓库，请先人工检查。" >&2
      return 1
    }
  fi

  echo "拉取: $name ($branch @ ${commit:0:12})"
  git clone --filter=blob:none --no-checkout --branch "$branch" "$url" "$target"
  git -C "$target" checkout --detach "$commit"
}

# Peer-reviewed, single-frame depth collision checking. Reproduce first.
fetch_repo \
  RAPPIDS \
  https://github.com/nlbucki/RAPPIDS.git \
  master \
  44625049430fdaa58e70f930e30e11be7fa8cac7 \
  CMakeLists.txt

# Peer-reviewed depth-history proximity queries. Core code is ROS-independent.
fetch_repo \
  nanomap_ros \
  https://github.com/peteflorence/nanomap_ros.git \
  master \
  650e80b02f29c229d4f3b0a8514fab5cf9233b47 \
  src/nanomap.cc

# Peer-reviewed visibility Graph; use the branch matching local ROS2 Humble.
fetch_repo \
  far_planner \
  https://github.com/MichaelFYang/far_planner.git \
  humble-jazzy \
  29fb215886df4b0c53c27f487c51041f808c5eba \
  src/far_planner/CMakeLists.txt

# Map-based ESDF baseline, not the intended OpenSeek runtime representation.
fetch_repo \
  FIESTA \
  https://github.com/HKUST-Aerial-Robotics/FIESTA.git \
  master \
  d01ce1b4602340a417a68ec7bb5f6b5a6790207e \
  CMakeLists.txt

echo "完成: $DESTINATION"
