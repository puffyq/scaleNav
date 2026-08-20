#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
RAPPIDS_ROOT="$PROJECT_ROOT/third_party/repro/RAPPIDS"
DEPTH="${DEPTH:-$PROJECT_ROOT/data/Map2GraphData/Scene_0002/Textures/depth_000000.exr}"
BUILD_DIR="${BUILD_DIR:-/tmp/openseek_rappids_map2}"
# RAPPIDS paper defaults; override these to test the current OpenSeek body.
PHYSICAL_RADIUS="${PHYSICAL_RADIUS:-0.26}"
PLANNING_RADIUS="${PLANNING_RADIUS:-0.46}"
MIN_CHECK_DISTANCE="${MIN_CHECK_DISTANCE:-1.0}"

[[ -f "$DEPTH" ]] || { echo "错误: 深度文件不存在: $DEPTH" >&2; exit 1; }

cmake -S "$RAPPIDS_ROOT" -B "$RAPPIDS_ROOT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$RAPPIDS_ROOT/build" -j"$(nproc)"
mkdir -p "$BUILD_DIR"

g++ -std=c++11 -O3 \
  -I"$RAPPIDS_ROOT/include" \
  $(pkg-config --cflags opencv4) \
  "$PROJECT_ROOT/tools/rappids_map2_compare.cpp" \
  "$RAPPIDS_ROOT/build/src/libRAPPIDS.a" \
  "$RAPPIDS_ROOT/build/src/libRapidQuadcopterTrajectories.a" \
  "$RAPPIDS_ROOT/build/src/libQuartic.a" \
  $(pkg-config --libs opencv4) \
  -o "$BUILD_DIR/rappids_map2_compare"

export OPENCV_IO_ENABLE_OPENEXR=1
"$BUILD_DIR/rappids_map2_compare" "$DEPTH" \
  "$PHYSICAL_RADIUS" "$PLANNING_RADIUS" "$MIN_CHECK_DISTANCE"
