#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_ROOT="${SCALENAV_LOG_DIR:-$ROOT/log_scalenav}"
MODEL="${ROUTE_YOPO_MODEL:-$ROOT/train_scalenav/saved_route_balanced_w05_train_large_001/YOPO_0/best.pth}"
PORT="${LOG_REPLAY_PORT:-8766}"

exec python3 "$ROOT/scalenav_ws/src/scalenav/log_replay_server.py" \
  --root "$LOG_ROOT" --model "$MODEL" --train-root "$ROOT/train_scalenav" \
  --port "$PORT" "$@"
