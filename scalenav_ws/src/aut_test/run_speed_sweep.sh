#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PROJECT_ROOT="$(cd -- "$WS/.." && pwd)"
CONFIG="$WS/src/config/config.yaml"
RESULTS_ROOT="$SCRIPT_DIR/results/speed_sweep_$(date +%Y%m%d_%H%M%S)_$$"
COUNT="${COUNT:-10}"
TIMEOUT="${TIMEOUT:-90}"
COOLDOWN="${COOLDOWN:-3}"

[[ -f "$CONFIG" ]] || { echo "missing config: $CONFIG" >&2; exit 1; }
[[ "$COUNT" =~ ^[1-9][0-9]*$ ]] || { echo "COUNT must be a positive integer" >&2; exit 2; }

original_config="$(mktemp)"
cp "$CONFIG" "$original_config"
cleanup() {
  cp "$original_config" "$CONFIG"
  rm -f "$original_config"
}
trap cleanup EXIT INT TERM

mkdir -p "$RESULTS_ROOT"
printf 'speed_mps,result_dir,summary_csv\n' > "$RESULTS_ROOT/index.csv"

for speed in 6 8 10; do
  sed -E -i "s/(maximum_trajectory_speed_mps:[[:space:]]*)[0-9]+(\.[0-9]+)?/\\1${speed}.0/" "$CONFIG"
  actual="$(awk '/maximum_trajectory_speed_mps:/{print $2; exit}' "$CONFIG")"
  [[ "$actual" == "${speed}.0" ]] || { echo "failed to set speed $speed (got $actual)" >&2; exit 1; }
  group_root="$RESULTS_ROOT/${speed}ms"
  mkdir -p "$group_root"
  cp "$CONFIG" "$group_root/config.yaml"
  echo "=== ScaleNav speed ${speed} m/s: ${COUNT} trials ===" | tee "$group_root/run.log"
  set +e
  /usr/bin/python3 "$SCRIPT_DIR/run_repeated_test.py" \
    --count "$COUNT" --timeout "$TIMEOUT" --cooldown "$COOLDOWN" \
    --stack scalenav --results-root "$group_root" \
    --log-root "$PROJECT_ROOT/log_scalenav" 2>&1 | tee -a "$group_root/run.log"
  status=${PIPESTATUS[0]}
  set -e
  summary="$(find "$group_root" -mindepth 2 -maxdepth 2 -name summary.csv -print -quit)"
  printf '%s,%s,%s\n' "$speed" "$group_root" "$summary" >> "$RESULTS_ROOT/index.csv"
  if ((status != 0)); then
    echo "speed ${speed} m/s batch exited with status ${status}" | tee -a "$RESULTS_ROOT/run.log"
  fi
done

echo "speed sweep complete: $RESULTS_ROOT"
