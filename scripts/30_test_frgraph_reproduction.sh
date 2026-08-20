#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
FRGRAPH_ROOT="$PROJECT_ROOT/third_party/FRGraph"
BUILD_DIR="${BUILD_DIR:-/tmp/openseek_frgraph_decomp_build}"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"

cd "$PROJECT_ROOT"

echo "[1/3] FRGraph upstream DecompUtil"
cmake -S "$FRGRAPH_ROOT/src/DecompROS/DecompUtil" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j"$(nproc)"
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "[2/3] OpenSeek FRGraph adapter tests"
export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1
"$PYTHON" -m unittest -v openseek.tests.test_frgraph_adapter

echo "[3/3] Map2 integration"
bash "$PROJECT_ROOT/scripts/28_run_frgraph_map2.sh" >/tmp/openseek_frgraph_map2.log
"$PYTHON" - <<'PY'
import json
from pathlib import Path

result = json.loads(Path("data/frgraph_map2_result.json").read_text())
assert result["frgraph"]["regionCount"] == 1, result["frgraph"]
assert result["optimisticPath"] == [0, 2, 1], result["optimisticPath"]
edge_states = {
    (edge["source"], edge["target"]): edge["state"]
    for edge in result["graph"]["edges"]
}
assert edge_states[(0, 2)] == "CERTIFIED", edge_states
assert edge_states[(2, 1)] == "UNVALIDATED", edge_states
print("FRGraph Map2: region=1 path=0->2->1 PASS")
PY
