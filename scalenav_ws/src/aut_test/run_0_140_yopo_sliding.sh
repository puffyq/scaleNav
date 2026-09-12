#!/usr/bin/env bash
set -Eeuo pipefail

# Graph + YOPO with a local sliding map. The graph and point-cloud history
# stay bounded around the vehicle instead of retaining the explored route.
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"

export STACK="scalenav"
export LOCAL_SLIDING_GRAPH="true"
export LOCAL_SLIDING_GRAPH_RADIUS_M="40.0"
export MAP_HISTORY_RADIUS_M="40.0"

exec bash "$SCRIPT_DIR/run_0_140_continuous.sh" \
  --stack scalenav \
  --prompt "${PROMPT:-blocks, walls}" \
  --timeout "${TIMEOUT_S:-150}" \
  "$@"
