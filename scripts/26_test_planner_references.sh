#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
REFERENCE_ROOT="${REFERENCE_ROOT:-$PROJECT_ROOT/third_party/repro}"
BUILD_ROOT="${BUILD_ROOT:-/tmp/openseek_planner_repro}"

bash "$PROJECT_ROOT/scripts/25_fetch_planner_references.sh"
mkdir -p "$BUILD_ROOT"

echo "[1/3] RAPPIDS: build and upstream benchmark"
cmake -S "$REFERENCE_ROOT/RAPPIDS" -B "$REFERENCE_ROOT/RAPPIDS/build" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$REFERENCE_ROOT/RAPPIDS/build" -j"$(nproc)"
mkdir -p "$BUILD_ROOT/rappids_run/data"
(
  cd "$BUILD_ROOT/rappids_run"
  "$REFERENCE_ROOT/RAPPIDS/build/test/Benchmarker" \
    --test_type 2 -n 20 \
    --w 160 --h 96 --f 80 --cx 79.5 --cy 47.5 \
    --vehicleRadius 0.6 --planningRadius 0.75 \
    --numCompTimesForTCTest 2
)

echo "[2/3] NanoMap: build ROS-independent core and run upstream tests"
mkdir -p "$BUILD_ROOT/nanomap_shim/pcl_conversions" "$BUILD_ROOT/nanomap"
printf '#pragma once\n' > "$BUILD_ROOT/nanomap_shim/pcl_conversions/pcl_conversions.h"
nanomap_flags=(
  -std=c++14 -O2 -pthread -include deque
  -I"$BUILD_ROOT/nanomap_shim"
  -I"$REFERENCE_ROOT/nanomap_ros/src"
  -I/usr/include/eigen3
  -I/usr/include/pcl-1.12
)
for source in \
  nanomap.cc \
  pose_manager.cc \
  structured_point_cloud_chain.cc \
  structured_point_cloud.cc \
  fov_evaluator.cc; do
  g++ "${nanomap_flags[@]}" -c "$REFERENCE_ROOT/nanomap_ros/src/$source" \
    -o "$BUILD_ROOT/nanomap/${source%.cc}.o"
done
g++ "${nanomap_flags[@]}" \
  "$REFERENCE_ROOT/nanomap_ros/test/test_nanomap.cpp" \
  "$BUILD_ROOT"/nanomap/*.o -lgtest -lpthread \
  -o "$BUILD_ROOT/nanomap/test_nanomap"
"$BUILD_ROOT/nanomap/test_nanomap"

echo "[3/3] FAR Planner: build the ROS2 Humble branch"
# ROS Humble's code generators require Ubuntu's Python 3.10, not an active
# Conda Python. Prepend /usr/bin for this isolated build only.
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
colcon --log-base "$BUILD_ROOT/far_log" build \
  --base-paths "$REFERENCE_ROOT/far_planner" \
  --build-base "$BUILD_ROOT/far_build" \
  --install-base "$BUILD_ROOT/far_install" \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Reproduction checks passed. FIESTA is retained as a ROS1 map baseline."
