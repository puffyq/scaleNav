#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python}"

if [[ ! -x "$PYTHON" ]]; then
  echo "ERROR: Python client environment not found: $PYTHON" >&2
  exit 1
fi

exec "$PYTHON" "$SCRIPT_DIR/plot_far_vs_scalenav_airsim.py" "$@"
