#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd -- "$WS/.." && pwd)"
LOG_ROOT="${SCALENAV_LOG_DIR:-$PROJECT_ROOT/log_scalenav}"
PORT="${SCALENAV_LOG_VIEWER_PORT:-8765}"
WEB_ROOT="$WS/install/scalenav_log/share/scalenav_log/web"
VIEWER="$WS/install/scalenav_log/lib/scalenav_log/scalenav_log_viewer"

usage() {
  echo "用法: $0 [--root DIR] [--port PORT] [--web-root DIR]"
  echo "默认日志目录: $PROJECT_ROOT/log_scalenav"
  echo "默认地址: http://127.0.0.1:$PORT"
}

while (($#)); do
  case "$1" in
    --root)
      (($# >= 2)) || { echo "错误: --root 需要目录" >&2; exit 2; }
      LOG_ROOT="$2"
      shift 2
      ;;
    --port)
      (($# >= 2)) || { echo "错误: --port 需要端口" >&2; exit 2; }
      PORT="$2"
      shift 2
      ;;
    --web-root)
      (($# >= 2)) || { echo "错误: --web-root 需要目录" >&2; exit 2; }
      WEB_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误: 未知参数 $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "错误: 未找到 ROS2 Humble" >&2
  exit 1
}
[[ -f "$WS/install/setup.bash" ]] || {
  echo "错误: 未找到工作区安装环境，请先运行 $SCRIPT_DIR/build.sh" >&2
  exit 1
}
[[ -x "$VIEWER" ]] || {
  echo "错误: 未找到日志 viewer，请先构建 scalenav_log" >&2
  exit 1
}
[[ -f "$WEB_ROOT/index.html" ]] || {
  echo "错误: 未找到回放网页: $WEB_ROOT/index.html" >&2
  exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

mkdir -p "$LOG_ROOT"
echo "ScaleNav log viewer: http://127.0.0.1:$PORT"
echo "日志目录: $LOG_ROOT"
exec "$VIEWER" --root "$LOG_ROOT" --web-root "$WEB_ROOT" --port "$PORT"
