#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OUTPUT="${WS}/bags/rgbd_odom_$(date +%Y%m%d_%H%M%S)"
ODOM_TOPIC="/sim/odom"
RGB_TOPIC="/camera/color/image"
DEPTH_TOPIC="/camera/depth/image"
DURATION=""

usage() {
  cat <<'EOF'
Usage: record_rgbd_odom.sh [options]

Record exactly odometry, RGB and Depth into a ROS 2 bag.

Options:
  --output DIR       Bag directory (default: ws/bags/rgbd_odom_TIMESTAMP)
  --duration SEC     Stop automatically after SEC seconds
  --odom-topic NAME  Default: /sim/odom
  --rgb-topic NAME   Default: /camera/color/image
  --depth-topic NAME Default: /camera/depth/image
EOF
}

while (($#)); do
  case "$1" in
    --output) [[ $# -ge 2 ]] || { echo "--output needs a value" >&2; exit 2; }; OUTPUT="$2"; shift 2 ;;
    --duration) [[ $# -ge 2 ]] || { echo "--duration needs a value" >&2; exit 2; }; DURATION="$2"; shift 2 ;;
    --odom-topic) [[ $# -ge 2 ]] || { echo "--odom-topic needs a value" >&2; exit 2; }; ODOM_TOPIC="$2"; shift 2 ;;
    --rgb-topic) [[ $# -ge 2 ]] || { echo "--rgb-topic needs a value" >&2; exit 2; }; RGB_TOPIC="$2"; shift 2 ;;
    --depth-topic) [[ $# -ge 2 ]] || { echo "--depth-topic needs a value" >&2; exit 2; }; DEPTH_TOPIC="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
source /opt/ros/humble/setup.bash
[[ ! -e "$OUTPUT" ]] || { echo "output already exists: $OUTPUT" >&2; exit 1; }
mkdir -p "$(dirname -- "$OUTPUT")"

command=(ros2 bag record --output "$OUTPUT" --storage sqlite3)
command+=("$ODOM_TOPIC" "$RGB_TOPIC" "$DEPTH_TOPIC")
printf 'Recording three topics to %s:\n' "$OUTPUT"
printf '  %s\n' "$ODOM_TOPIC" "$RGB_TOPIC" "$DEPTH_TOPIC"

if [[ -n "$DURATION" ]]; then
  [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "duration must be seconds" >&2; exit 2; }
  (( $(awk "BEGIN { print ($DURATION > 0) }") )) || { echo "duration must be positive" >&2; exit 2; }
  timeout --foreground --signal=INT "${DURATION}s" "${command[@]}" || {
    status=$?
    [[ $status -eq 124 ]] || exit "$status"
  }
else
  exec "${command[@]}"
fi
echo "Bag written to $OUTPUT"
