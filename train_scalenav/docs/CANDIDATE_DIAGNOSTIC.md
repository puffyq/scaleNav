# 15-Primitive 3D Candidate Diagnostic

该诊断保留 YOPO 网络输出的全部 15 个 primitive，而不是只保存 score 最低的一个。
每个候选保存 101 个 XYZ 轨迹采样点、body/world 终态 `p/v/a`，以及 score、碰撞、
最小净空、中心线距离、走廊偏差和路线进度。Route-YOPO 与 YOPO-Simple 使用同一 depth、
零 motion、同一 10 m local goal 和同一世界姿态。

运行：

```bash
cd /mnt/code/lab/yopo/OpenSeek/train_scalenav
PYTHONPATH=. /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python evaluate_candidates.py \
  --data dataset/benchmark_004_esdf_001 \
  --checkpoint saved/YOPO_54/best.pth \
  --simple-checkpoint /mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth \
  --output dataset/benchmark_004_esdf_001/candidate_diagnostic_003 \
  --batch-size 64 --workers 0 --device cuda
```

产物：

- `candidate_diagnostic_003/viewer/index.html`：XY、XZ、YZ 同步交互 viewer；
- `candidate_diagnostic_report.json`：selected/oracle aggregate 指标；
- `candidate_predictions.json`：每条路线全部候选的详细结果。

viewer 中可分别开关两个模型的全部 15 条轨迹、安全候选和 selected-only，并查看每条候选
的 world `end xyz/vxyz/axyz` 与 z 范围。Bubble 在三个正交投影中均按球的投影圆显示。

## 3D 合同对拍

`tests/test_3d_alignment.py` 直接检查：

- 原版 3x5 lattice 角度、旋转矩阵、展平及 flip 顺序；
- 15 个随机 primitive 的完整 position/velocity/acceleration 解码；
- 含 z 分量的 velocity/acceleration/goal 输入变换；
- 本地 XYZ 轨迹采样与上游 `Poly5Solver`；
- 非零 yaw/pitch/roll 下 body-FLU 到 world-ENU 的三维变换；
- 显式期望排列：输出三行 pitch 为 `+20/0/-20 deg`，每行 yaw 为
  `+36/+18/0/-18/-36 deg`。

结果未发现 primitive/三维坐标不一致。1479 条 route 的候选统计也保持明确的上下层：
Route-YOPO 三行平均 body z 为 `+0.442/+0.024/-0.397 m`，YOPO-Simple 为
`+0.712/+0.405/-0.434 m`。

该批 1479 个 local goal 的 body z 全部为 `0`。selected 结果中，Route-YOPO 平均终点 body z
为 `+0.099 m`，三行选择比例为 `11.7%/84.9%/3.4%`；YOPO-Simple 平均为 `+0.451 m`，
比例为 `17.2%/82.8%/0%`。因此原版 checkpoint 在这批水平目标上存在明显向上选择偏置，
但下层候选本身仍具有正确的负 z/vz，属于模型输出/score 分布而非坐标变换错误。

## 发现的独立缺陷

原版 YOPO-Simple head 输入顺序是 `[observation(9), depth(64)]`。历史 Route-YOPO 使用了
`[depth(64), observation(9), route(60)]`，导致从原版初始化时第一层语义错位。当前网络已
修正为 `[observation, depth, route]`。读取无 `feature_order` 元数据的 133-channel 历史
checkpoint 时，loader 会置换首层 `0:9` 与 `9:73` 权重，使旧模型函数保持完全一致；旧
optimizer state 不允许直接 resume，只能 `--finetune` 或从原版重新初始化。

## 完整批次结果

- Route-YOPO selected：碰撞 `0.203%`，平均 3D 中心线距离 `0.220 m`，进度 `6.545 m`。
- YOPO-Simple selected：碰撞 `8.046%`，平均 3D 中心线距离 `0.413 m`，进度 `6.225 m`。
- Route-YOPO score-selected 与无进度约束的 centerline oracle 匹配率为 `17.2%`；该 oracle
  平均进度只有 `4.551 m`，不能单凭匹配率判定 score head 错误。

原 viewer 中：

- `score selected` 是网络 score 最低的 primitive；
- `centerline oracle` 是安全候选中平均 3D witness 中心线距离最小的 primitive；
- `selectionCenterlineGapM` 是两者的中心线距离差；
- `All 15 candidates` 可显示全部候选，`Safe candidates only` 可隐藏碰撞候选。

解释规则：若 oracle 明显优于 selected，问题主要在 score head 排序；若两者都偏离中心线，
问题主要在 end-state/trajectory 表达或 path loss 梯度。
