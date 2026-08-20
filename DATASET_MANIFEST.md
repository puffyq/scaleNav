# Map4 Person Snapshot Dataset

这是基于复制地图 `FlyingExampleMapV4_PersonTest` 采集的 YOPO-Simple 数据集。
它是可以直接送进当前 Text-YOPO 训练器的完整试验集：每一帧同时包含 RGB、深度、位姿和 PEARL 语义热力图；不包含 YOPO-Rally 专家轨迹，也不需要 `data_opt.toml`。

## 数据位置

| split | 路径 | 帧数 | 内容 |
|---|---|---:|---|
| train | `data/TrainingData/Scene_0001` | 800 | 800 RGB PNG + 800 Depth EXR + 800 `semantic_pearl_*.npy` |
| test | `data/TestingData/Scene_0001` | 200 | 200 RGB PNG + 200 Depth EXR + 200 `semantic_pearl_*.npy` |

每个 scene 目录还包含：

- `data.toml`：相机 FOV、深度上限、位置、姿态、yaw 和 `targetPrompt="person"`；
- `tree.ply`：静态地图点云、DepthPlanar 反投影点云，以及生成的人体胶囊碰撞点，统一使用世界 NED 米制坐标。

## 采集配置

- 地图：`/Game/FlyingCPP/Maps/FlyingExampleMapV4_PersonTest`
- 原始 Map4 未修改；UE 人物密度：30%；实际生成：45 人
- 训练随机种子：`1001`
- 测试随机种子：`2001`
- 采样区域：NED `x,y in [-30, 30]`，相机高度 `1.6 m`
- 深度：米制 float32 EXR，`depthMaxMeters=20.0`
- 文本 query：`person`
- 图像/热力图原始尺寸：`160x96`
- YOPO 输入尺寸：`[2,96,160]`（Depth + PEARL），输出网格：`3x5=15` 条候选轨迹

当前是 1000 帧的可训练 pilot dataset，不宣称是最终的 10 万帧生产规模。扩充时保持相同目录格式，并使用不同 seed 和不重叠的采样序列。

## Map2 Graph 验收数据

`data/Map2GraphData` 不是训练集，而是稀疏 Graph 的离线回放数据：

- `Scene_0001`：10 个 Map2 随机 RGB-D 快照；
- `Scene_0002`：1 个固定大障碍快照，位置约
  `(24.25, 12.07, -1.6)`、yaw 约 `87 deg`；
- 固定帧预期产生 3 个 CERTIFIED 和 3 个 INVALID 节点，并选择侧向
  body-FLU waypoint 约 `(4.10, -2.87, 0)`。

```bash
SCENE=Scene_0002 FRAME=0 bash scripts/22_test_map2_graph.sh
```

## Bash 流程

以下脚本都从仓库根目录执行。采集前需要打开复制的 UE 地图、按下
Play、在人物面板中生成指定密度的人物，并让 AirSim RPC 可连接。

```bash
# 采集 RGB-D + tree.ply，计算 PEARL，并验证 train/test
bash scripts/15_generate_map4_dataset.sh

# 正式训练（默认 50 epoch，输出 saved/map4_person）
bash scripts/16_train_text_yopo.sh

# 查看训练集 RGB / Depth / PEARL / 元数据
bash scripts/17_start_data_viewer.sh

# 查看测试集上的模型候选轨迹、代价和 PEARL 引导结果
bash scripts/18_start_offline_test.sh

# 修复旧数据的 tree.ply（保留 .tree.ply.before_depth 备份）
bash scripts/19_rebuild_depth_maps.sh
```

常用覆盖参数示例：

```bash
# 改人物采集数量和随机种子；不覆盖已有目录
TRAIN_COUNT=2000 TEST_COUNT=500 TRAIN_SEED=3001 TEST_SEED=4001 \
  bash scripts/15_generate_map4_dataset.sh

# 一次生成多个 Scene_*；每个 scene 都有自己的 train/test 帧
SCENE_IDS=0002,0003 bash scripts/15_generate_map4_dataset.sh

# 确认要重采集已有 scene 时显式覆盖
OVERWRITE=1 bash scripts/15_generate_map4_dataset.sh

# 训练冒烟测试
EPOCHS=1 BATCH_SIZE=4 CHECKPOINT_EVERY=0 \
  bash scripts/16_train_text_yopo.sh
```

## 验证

在仓库根目录执行：

```bash
PYTHONPATH=openseek:. /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  -m openseek.data.validate_snapshot_dataset data/TrainingData --require-semantic
PYTHONPATH=openseek:. /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  -m openseek.data.validate_snapshot_dataset data/TestingData --require-semantic
```

预期输出分别是 `{"Scene_0001": 800}` 和 `{"Scene_0001": 200}`。

当前数据加载器会按 PEARL 置信度自动选择 semantic approach 或 numeric/search 模式；低置信度帧不会被丢弃。

本数据构建出的 ESDF 有效点云范围为：`x=(-48.90, 40.35)`、`y=(-40.16, 39.77)`、`z=(-5.52, 0.11)` 米。

## 训练入口

```bash
PYTHONPATH=openseek:. /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  openseek/train_text_yopo.py \
  --train-data data/TrainingData --test-data data/TestingData \
  --output saved/map4_person_smoke --epochs 1 --batch-size 4 \
  --workers 0 --precision fp32 --eval-every 1 --checkpoint-every 0
```

该冒烟训练已在本机完整通过。输出位于 `saved/map4_person_smoke`：

- `text_yopo.pt`：最终 TorchScript 模型；
- `text_yopo_state.pth`：最终训练状态；
- `best/text_yopo.pt`：本轮测试集最优模型；
- `tensorboard/`：训练和测试指标。

本次坐标和 160x96 配置修正后的测试结果：`total=1.1808`、`selection_top1=0.0900`、`selected_total_cost=1.0139`。这些数值只证明整条数据和训练链路可运行，不代表 1 epoch 模型已经收敛。
