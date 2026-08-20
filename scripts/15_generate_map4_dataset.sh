#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_ROOT/../YOPO-Rally/.venv/bin/python}"
DATA_ROOT="${DATA_ROOT:-$PROJECT_ROOT/data}"
TRAIN_DATA="$DATA_ROOT/TrainingData"
TEST_DATA="$DATA_ROOT/TestingData"
AIRSIM_ROOT="${AIRSIM_ROOT:-/mnt/code/lab/airsim/Colosseum/PythonClient}"
PERSON_POSITIONS="${PERSON_POSITIONS:-/mnt/code/lab/airsim/Colosseum/Unreal/Environments/BlocksV2/Saved/PersonSpawner/generated_people.json}"
PEARL_ROOT="${PEARL_ROOT:-$PROJECT_ROOT/third_party/PEARL}"
SCENE_IDS="${SCENE_IDS:-${SCENE_ID:-0001}}"
PROMPT="${PROMPT:-person}"
TRAIN_COUNT="${TRAIN_COUNT:-800}"
TEST_COUNT="${TEST_COUNT:-200}"
TRAIN_SEED="${TRAIN_SEED:-1001}"
TEST_SEED="${TEST_SEED:-2001}"
OVERWRITE="${OVERWRITE:-0}"

[[ -x "$PYTHON" ]] || { echo "错误: Python 环境不存在: $PYTHON" >&2; exit 1; }
[[ -d "$AIRSIM_ROOT" ]] || { echo "错误: Colosseum PythonClient 不存在: $AIRSIM_ROOT" >&2; exit 1; }
[[ -f "$PERSON_POSITIONS" ]] || { echo "错误: 请先在 UE 人物面板中生成人物: $PERSON_POSITIONS" >&2; exit 1; }
[[ -f "$PEARL_ROOT/pearl/prop.py" ]] || { echo "错误: PEARL 源码不完整: $PEARL_ROOT" >&2; exit 1; }
[[ "$TRAIN_COUNT" =~ ^[1-9][0-9]*$ ]] || { echo "错误: TRAIN_COUNT 必须是正整数" >&2; exit 1; }
[[ "$TEST_COUNT" =~ ^[1-9][0-9]*$ ]] || { echo "错误: TEST_COUNT 必须是正整数" >&2; exit 1; }
[[ "$TRAIN_SEED" =~ ^-?[0-9]+$ ]] || { echo "错误: TRAIN_SEED 必须是整数" >&2; exit 1; }
[[ "$TEST_SEED" =~ ^-?[0-9]+$ ]] || { echo "错误: TEST_SEED 必须是整数" >&2; exit 1; }
[[ "$OVERWRITE" == "0" || "$OVERWRITE" == "1" ]] || { echo "错误: OVERWRITE 只能是 0 或 1" >&2; exit 1; }
IFS=',' read -r -a scene_ids <<< "$SCENE_IDS"
(( ${#scene_ids[@]} > 0 )) || { echo "错误: SCENE_IDS 不能为空" >&2; exit 1; }
for scene_id in "${scene_ids[@]}"; do
  [[ "$scene_id" =~ ^[0-9]+$ ]] || { echo "错误: 场景编号必须是数字: $scene_id" >&2; exit 1; }
done

export PYTHONPATH="$PROJECT_ROOT/openseek:$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OPENCV_IO_ENABLE_OPENEXR=1

overwrite_args=()
if [[ "$OVERWRITE" == "1" ]]; then
  overwrite_args+=(--overwrite)
else
  for scene_id in "${scene_ids[@]}"; do
    if [[ -d "$TRAIN_DATA/Scene_$scene_id" || -d "$TEST_DATA/Scene_$scene_id" ]]; then
      echo "错误: Scene_$scene_id 已经存在。要重新采集请显式设置 OVERWRITE=1，或设置新的 DATA_ROOT。" >&2
      exit 1
    fi
  done
fi

echo "采集前请确认："
echo "  1. UE 已打开 FlyingExampleMapV4_PersonTest 并按下 Play"
echo "  2. 人物面板已按所需百分比生成人物"
echo "  3. AirSim RPC 端口 41451 可连接"
echo "数据目录: $DATA_ROOT"
echo "场景: ${scene_ids[*]}"
echo "每个场景训练/测试: $TRAIN_COUNT/$TEST_COUNT，query=$PROMPT"

scene_index=0
for scene_id in "${scene_ids[@]}"; do
  scene_train_seed=$((TRAIN_SEED + scene_index * 1000003))
  scene_test_seed=$((TEST_SEED + scene_index * 1000003))
  echo "采集训练场景 Scene_$scene_id"
  "$PYTHON" -m openseek.data.snapshot_dataset \
    --output "$TRAIN_DATA" \
    --scene-id "$scene_id" \
    --count "$TRAIN_COUNT" \
    --seed "$scene_train_seed" \
    --prompt "$PROMPT" \
    --person-positions "$PERSON_POSITIONS" \
    --export-static-meshes \
    --airsim-root "$AIRSIM_ROOT" \
    "${overwrite_args[@]}"

  # Reuse the merged Map4 + person point cloud so both splits have identical
  # collision geometry without adding the same person capsules twice.
  echo "采集测试场景 Scene_$scene_id"
  "$PYTHON" -m openseek.data.snapshot_dataset \
    --output "$TEST_DATA" \
    --scene-id "$scene_id" \
    --count "$TEST_COUNT" \
    --seed "$scene_test_seed" \
    --prompt "$PROMPT" \
    --obstacle-ply "$TRAIN_DATA/Scene_$scene_id/tree.ply" \
    --airsim-root "$AIRSIM_ROOT" \
    "${overwrite_args[@]}"
  scene_index=$((scene_index + 1))
done

for split in "$TRAIN_DATA" "$TEST_DATA"; do
  pearl_args=()
  [[ "$OVERWRITE" == "0" ]] || pearl_args+=(--overwrite)
  "$PYTHON" "$PROJECT_ROOT/openseek/precompute_pearl.py" \
    --data "$split" \
    --pearl-root "$PEARL_ROOT" \
    --prompt "$PROMPT" \
    --force-prompt \
    "${pearl_args[@]}"
  "$PYTHON" -m openseek.data.validate_snapshot_dataset \
    "$split" --require-semantic
done

echo "数据集完成:"
echo "  $TRAIN_DATA"
echo "  $TEST_DATA"
