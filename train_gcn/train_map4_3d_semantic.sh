#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

PROMPT="${PROMPT_3D:-line}"
LOGS_ROOT="${LOGS_ROOT:-$ROOT/log_scalenav}"
OCCUPANCY="${OCCUPANCY_3D:-$ROOT/paper/scalenav/pics/map4_mesh_truth_3d_20260906.npz}"
DATASET="${DATASET_3D:-$ROOT/train_gcn/dataset_privileged_map4_3d_${PROMPT}.pt}"
MODEL="${MODEL_3D:-$ROOT/train_gcn/frontier_gcn_map4_3d_${PROMPT}.pt}"
EPOCHS="${EPOCHS_3D:-30}"
DEVICE="${DEVICE:-cuda}"
FORCE_RECOLLECT="${FORCE_RECOLLECT:-1}"

case "${PROMPT,,}" in
  line|trees) ;;
  *)
    echo "PROMPT_3D must be line or trees" >&2
    exit 2
    ;;
esac

if [[ "$FORCE_RECOLLECT" == "1" || ! -f "$DATASET" ]]; then
  echo "collecting 3-D semantic dataset: $DATASET" >&2
  echo "collecting prompt-conditioned 3-D semantic data from $LOGS_ROOT" >&2
  "$ROOT/../YOPO-Rally/.venv/bin/python" "$SCRIPT_DIR/collect_3d_semantic_dataset.py" \
    --logs-root "$LOGS_ROOT" \
    --output "$DATASET" \
    --occupancy "$OCCUPANCY" \
    --prompt "$PROMPT"
fi

exec "$ROOT/../YOPO-Rally/.venv/bin/python" "$SCRIPT_DIR/train.py" \
  --dataset "$DATASET" \
  --architecture 3d \
  --device "$DEVICE" \
  --epochs "$EPOCHS" \
  --save "$MODEL"
