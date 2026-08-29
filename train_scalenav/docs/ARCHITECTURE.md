# ScaleNav Route-Conditioned YOPO 训练架构

| 项目 | 内容 |
|---|---|
| 文档版本 | `V1.0` |
| 日期 | `2026-08-27` |
| 上游设计 | `scalenav_ws/docs/ALGORITHM_DESIGN.md` |
| 范围 | 数据生成、witness corridor、模型训练、离线评测 |
| 不包含 | ROS2 推理部署、飞控执行、EPIC 图性能优化 |

## 1. 目的和系统边界

ScaleNav/EPIC 负责选择拓扑路线，并输出经过碰撞检查的 accepted witness；YOPO 负责在
该路线的局部安全走廊内生成动力学可执行轨迹。YOPO 不重新决定全局左右绕行，也不把
witness 当作带时间参数的专家轨迹进行行为克隆。

在线系统的唯一路线真值是 EPIC accepted witness。合成训练场景可以使用 Python 真值
规划器，但其路线目标、clearance 定义、平滑和质量门必须与 ScaleNav M4 等价，并通过
固定场景对拍验证。不能因为合成场景方便，就在数据生成器中形成另一套路线语义。

```mermaid
flowchart LR
    Sensor[深度 / 点云 / Odom] --> Map[ScaleNav M1 局部几何]
    Map --> Topology[ScaleNav M2 Bubble / TopoGraph]
    Topology --> Search[ScaleNav M4 路线搜索]
    Search --> Accepted[Accepted witness]
    Accepted --> Refine[Witness 中心线优化]
    Refine --> Corridor[有序 corridor bubbles]
    Corridor --> Contract[RouteCondition / routes.npz]
    Contract --> Dataset[YOPODataset]
    Dataset --> Model[Route-Conditioned YOPO]
    Dataset --> Loss[ESDF + route loss]
    Model --> Loss
    Loss --> Checkpoint[Checkpoint]
```

## 2. 三种 Bubble 必须分开

| 名称 | 所属模块 | 中心 | 半径 | 用途 |
|---|---|---|---|---|
| 地图 Bubble | ScaleNav M2 | 自由空间采样点 | 到障碍安全空间 | 构建 TopoGraph |
| TopoNode 代表 Bubble | ScaleNav M2 | 拓扑节点中心 | 代表性半径 | 图搜索、诊断 |
| Witness corridor Bubble | M4 输出适配层 | 优化后 witness 中心线采样点 | `clearance - robot_radius - margin` | YOPO 输入和 loss |

Witness corridor Bubble 不是把地图 Bubble 全量送入模型，也不是在原 witness 旁边随意
移动一组只供训练使用的圆。若需要扩大局部细腰，必须先优化 witness 中心线，再从新
中心线重新计算 clearance 和 Bubble。否则 corridor loss 使用一条线，progress/tangent
使用另一条线，训练目标会自相矛盾。

## 3. 目标架构

### 3.1 S0 ClearanceField

统一提供以下纯几何接口：

```text
clearance(point) -> distance_to_nearest_obstacle
gradient(point)  -> clearance ascent direction
segment_min_clearance(a, b, max_step) -> continuous lower bound
is_safe(point, required_clearance) -> bool
```

在线实现读取 ScaleNav `LIOInterface`；合成实现读取解析障碍/ESDF。两者使用相同的
`robot_radius + safety_margin` 定义。点采样间隔不能代替连续线段审计。

### 3.2 S1 路线搜索

搜索不能只最小化栅格长度。对同一 homotopy 内的可行路线，采用受限 widest-shortest
目标：

```text
硬约束:
  clearance(p) >= robot_radius + safety_margin
  route_length <= detour_ratio * shortest_safe_length

排序:
  1. 最大化 r_min = min_p(clearance(p) - robot_radius - safety_margin)
  2. 最小化 integral clearance-risk
  3. 最小化 route_length
```

推荐的连续风险项与 ScaleNav 总体设计保持一致：

```text
D = w_clearance * ds * (d_target / (d_target + safe_radius))^2
```

`r_min` 是 bottleneck 指标，避免出现单个极小 Bubble；`detour_ratio` 防止 widest path 为了
追求宽度无限绕行。不同拓扑分支仍由 EPIC/TopoGraph 选择，不由 YOPO 数据层选择。

### 3.3 S2 Witness 中心线优化

搜索得到离散 witness 后，在不改变拓扑分支的前提下做局部中心化：

1. 固定起点、frontier 和受保护转折点。
2. 对中间点沿 clearance 梯度尝试小步移动。
3. 同时加入长度、曲率和偏离原 witness 的正则项。
4. 只有 `r_min` 或 clearance-risk 改善且所有线段连续安全时才接受。
5. 重新采样并迭代，直到改善低于阈值或达到迭代上限。

建议目标：

```text
J_center = -w_bottleneck * softmin(safe_radius)
           + w_risk * integral clearance-risk
           + w_length * path_length
           + w_curvature * curvature_cost
           + w_anchor * distance_to_original_witness
```

真实狭窄通道两侧梯度会相互抵消，优化不能虚假扩大半径；只有路径贴单侧障碍时，中心线
才会向更大的自由空间移动。

### 3.4 S3 Corridor 构造

从优化后的同一条 witness 中心线生成稠密 corridor：

```text
center_i = refined_witness(s_i)
raw_clearance_i = ClearanceField.clearance(center_i)
safe_radius_i = raw_clearance_i - robot_radius - safety_margin
```

要求：

- 常规采样步长不大于 `0.25 m`，高曲率/高梯度区不大于 `0.1 m`。
- 相邻 Bubble 必须覆盖中间 witness segment，而不只检查端点。
- 保存原始 clearance，不用裁剪后的模型半径代替几何审计值。
- 模型输入可将半径裁剪到配置上限；loss 和质量门使用未裁剪真值。
- 12 个模型 Bubble 的降采样不得制造新的细腰；负责区间半径取保守最小值。

### 3.5 S4 RouteQualityGate

质量门同时记录以下指标：

```text
minimum_safe_radius
safe_radius_p05
neck_length_below_target
continuous_min_clearance
maximum_curvature
bubble_overlap_margin
center_refinement_gain
```

硬拒绝只针对不可执行路线，例如 `minimum_safe_radius <= 0`、连续碰撞、Bubble 断连或曲率
不可达。对于安全但较窄的真实通道，保留样本并降低质量权重；对于存在明显更宽替代路
线却仍产生细腰的路线，标记为 labeler/search 缺陷，不应进入训练。

### 3.6 S5 数据契约

`routes.npz` 保存原始、可审计数据：

```text
frontier_goal_world
path_points_world              # refined accepted witness
path_clearance_m               # 原始障碍距离
path_bubble_radius_m           # 可用安全半径
route_min_clearance_m
route_min_safe_radius_m
route_neck_length_m
route_quality_flags / weight
route_seed / route_id
```

真实日志只消费 EPIC accepted witness。合成数据保存搜索前后指标，使一个 route 可以回答
“细腰来自真实通道，还是来自搜索/平滑”。

### 3.7 S6 Dataset 和模型输入

```text
Depth             [B, 1, 96, 160]
Velocity/Accel    [B, 6] body FLU
Frontier goal     [B, 3] body FLU
Ordered bubbles   [B, 12, 4] = center_xyz + safe_radius
```

Dataset 只能做坐标变换、固定 K 采样和归一化，不负责重新搜索、移动圆心或修改
几何真值。训练和在线必须共用同一固定 K 采样规则。

### 3.8 S7 模型和 S8 Loss

模型仍输出 `3 x 5` primitive 的动力学终态和 score。路线损失使用同一条 refined witness：

```text
L = L_safety + L_smooth + L_acceleration + L_frontier
    + L_bubble_field + L_route_path_mse
```

`L_bubble_field` 使用与 ESDF safety 相同形状的 signed-distance 指数场：轨迹在 Bubble 外时被
推回安全体积，进入 Bubble 后仍保留向球心的连续梯度。它与真实障碍 ESDF safety 相加，安全球
半径必须来自同一 ESDF。`L_route_path_mse` 只按 witness 弧长提供有序中心线引导，避免在 Bubble
联合体内部走捷径。Loss 不负责修正错误标签。

### 3.9 S9 评测

除碰撞、平均 clearance、route progress 外，必须增加：

- 按 `minimum_safe_radius` 分桶的碰撞率和进度。
- neck 前停车率、穿越率和最大 corridor violation。
- 搜索前/中心化后 `r_min`、长度、曲率变化。
- Route-YOPO 与 YOPO-Simple 使用完全相同 depth、motion、frontier 的配对结果。
- 树林和大方块分别报告，不只汇总平均值。

树林评测分成两个独立场景，不得混为同一种数据：

- `yopo_forest`：保持 4 m 布局分布的解析圆柱近似，用于快速回归。
- `yopo_real_forest`：读取 YOPO-Simple 原始 `tree.ply`，保留树干、树枝、树冠及随机三轴
  姿态；occupancy、clearance、深度和导出点云均来自实例化后的真实树点云。

离线批次 `benchmark_002` 固定包含 `yopo_forest,yopo_real_forest,blocks` 三个场景，两个
模型仍使用逐样本完全相同的 depth、motion 和 frontier goal。

## 4. 当前实现映射

| 阶段 | 当前实现 | 当前行为 | 与目标差异 |
|---|---|---|---|
| ClearanceField | `GroundTruthScene.clearance_m`、KD-tree | 有真值距离 | 缺统一 gradient/segment 接口 |
| 搜索 | `GroundTruthScene._build_navigation_graph` | 8 邻域，边权仅为长度 | 没有 bottleneck/clearance 目标 |
| 平滑 | `smooth_grid_path` | 折线短化、Chaikin、Spline | 后两步使用更低的 smoothing margin |
| Corridor | `build_witness_corridor` | 路径重采样后直接查询半径 | 不优化 witness 中心线 |
| 质量门 | `RouteQualityGate.evaluate` | `radius > 0`、Bubble overlap | 没有 neck 指标和避免伪细腰判定 |
| 固定 K 输入 | `sample_route_bubbles` | 窄点优先，区间半径取最小 | 会忠实放大上游细腰影响 |
| Dataset | `YOPODataset.__getitem__` | 坐标变换、归一化 | 职责正确，不应在此修路 |
| Loss | `RouteLoss.forward` | Bubble 并集 + progress + tangent | 会正确惩罚细腰外轨迹 |
| 模型 | `YopoNetwork` | route 条件 + 15 primitives | 不是细腰产生模块 |

## 5. 当前问题定位

截图中的局部小 Bubble 首先属于数据路线生成链路，而不是模型：

1. **主因：合成搜索模块。** `ground_truth_dataset.py::_build_navigation_graph()` 的边权只有
   欧氏步长，A* 只找短路，不最大化最小安全半径。
2. **次因：平滑模块。** 原始 A* 使用较大的 `planning_occupancy`，但 Chaikin/Spline 接受
   条件切换到较小的 `smoothing_occupancy`，可能将路径重新拉向障碍。
3. **放大器：Corridor 构造。** `build_witness_corridor()` 直接把路径点当圆心，不进行
   中心线优化，因此贴障点原样成为小 Bubble。
4. **放大器：固定 K 采样。** `sample_route_bubbles()` 主动优先保留局部半径极小点，并对
   负责区间取最小半径，所以模型一定能看到该细腰。
5. **结果而非根因：Loss。** `RouteLoss` 正确使用 Bubble 并集。它使模型在细腰处保守或
   提前停止，但不应通过放宽 loss 来掩盖错误 corridor。

在线 ScaleNav 也需要注意两级代价的差异：TopoGraph 的 `routeEdgeCost()` 已包含
`edgeClearancePenalty()`，但 `ParallelBubbleAstar::search()` 内部的 `g_score` 仍只有步长，
`collisionCheck_shortenPath()` 只验证 Bubble overlap。因此拓扑分支会偏好宽路，不代表
每条 edge witness 已沿通道中心。目标实现应在 ScaleNav M4 的 witness 生成/短化层加入
bottleneck-aware 中心线优化，再由离线合成实现对齐，而不是只改 Python 数据。

## 6. 推荐实施顺序

1. 在固定截图场景上保存原始 occupancy、最短路径、每点 clearance 和 `r_min` fixture。
2. 为 S0 增加连续线段 clearance 和梯度接口及单元测试。
3. 在合成搜索中实现受限 widest-shortest，并验证不会无限绕路。
4. 实现 witness 中心线优化，端点固定，输出优化前后 `r_min/length/curvature`。
5. 统一平滑阶段安全门，不允许平滑降低到另一套 margin。
6. 扩展 `RouteQualityGate` 的 neck 指标和 reason code。
7. 在 ScaleNav M4 实现等价逻辑，并做 C++/Python 固定地图对拍。
8. 重新生成新批次数据，不覆盖 `pilot_001/test_001/benchmark_001`。
9. 先用旧 checkpoint 离线评估新 corridor，再决定是否重新训练。
10. 新训练只使用通过新质量门的数据，并重新执行树林/大方块配对基准。

## 7. 验收条件

- 固定伪细腰场景的 `r_min` 明显增加，路径长度增长不超过配置的 `detour_ratio`。
- 固定真实窄通道场景不得报告虚假的半径增加。
- 起点/frontier 不移动，优化前后属于同一拓扑分支。
- 所有 witness segment 连续安全，相邻 corridor Bubble 无未覆盖间隙。
- Python 合成 labeler 与 ScaleNav M4 在固定地图上的 route 分支和 bottleneck 指标一致。
- 新数据报告包含 `r_min`、P05、neck length 及优化增益分布。
- 模型在窄度分桶上的进度改善不能以碰撞率上升为代价。
