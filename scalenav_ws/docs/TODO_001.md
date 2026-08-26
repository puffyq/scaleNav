# TODO

| 项目 | 内容 |
|---|---|
| 批次号 | `001` |
| 主题 | Route-Conditioned YOPO |
| 状态 | 正式设计已采纳，待实施 |

## 1. 目标

在保留 YOPO-Simple 原有深度感知、轨迹 primitive 和物理代价训练方式的基础上，
将 EPIC 输出的 `frontier_goal` 和 accepted witness path 作为 YOPO 的条件输入，使
YOPO 不仅朝局部目标前进，还能沿全局规划器选定的安全走廊生成动力学可执行轨迹。

第一版按当前在线系统采用固定高度规划，但数据格式、坐标和模型接口保留完整 3-D
字段，避免后续扩展 3-D 时重新制作数据。

## 2. 已确认的现状

### 2.1 YOPO-Simple 数据与训练机制

YOPO-Simple 不依赖专家轨迹标签。原始流程是：

1. 为每个环境生成或加载点云地图。
2. 在距离障碍物超过 `safe_dist` 的位置随机采样相机位姿。
3. 随机采样 roll、pitch 和 yaw，并渲染归一化深度图。
4. 每个环境保存一份点云，每个图像保存对应位置和四元数。
5. Dataset 在训练时随机生成速度、加速度和 goal。
6. 网络预测所有 primitive 的轨迹终态和 score。
7. 使用点云生成的 ESDF、平滑度、加速度和 goal cost 进行自监督训练。

参考实现：

- `/mnt/code/lab/yopo/YOPO-Simple/Simulator/src/src/dataset_generator.cpp`
- `/mnt/code/lab/yopo/YOPO-Simple/YOPO/policy/yopo_dataset.py`
- `/mnt/code/lab/yopo/YOPO-Simple/YOPO/loss/safety_loss.py`

### 2.2 ScaleNav 已有能力

- `data/snapshot_dataset.py` 已能从 AirSim 采集 RGB-D、位姿和场景 `tree.ply`。
- EPIC 已通过 `/epic/path` 发布 accepted witness polyline。
- `frontier_goal` 已存在于 EPIC 内部状态和诊断日志，但没有独立、原子化的模型输入接口。
- `TopoGraph::goalDirectedSearch()`、edge `paths_` 和 `getBubbleSnapshot()` 已可用于离线生成路线。
- 当前 YOPO 输入仍是 Depth 和 9-D `[velocity, acceleration, goal]`。

## 3. 总体决定

### 3.1 第一版输入采用 witness corridor bubbles

第一版以 accepted witness path 为路线骨架，并将路径重采样点转换成一串有序的
`witness corridor bubbles`。每个 Bubble 包含中心 `[x, y, z]` 和安全半径
`safe_radius`。暂不把整个局部图的 Bubble 集合作为模型输入。

```text
witness corridor bubble = [center_x, center_y, center_z, safe_radius]
safe_radius = max(0, distance_to_obstacle - robot_radius - safety_margin)
```

这里的 witness corridor bubble 和拓扑构建阶段的原始 `BubbleNode` 必须明确区分：

- 原始 `BubbleNode` 是拓扑生成过程量，聚类成 `TopoNode` 后其集合会被清空。
- `TopoNode::bubble_radius_` 只保留该拓扑节点的一个代表性 Bubble 半径。
- edge witness 是碰撞验证后的折线，不直接保存逐点 Bubble。
- witness corridor bubble 是沿最终 accepted witness 重新查询距离场得到的有序安全管道。

原因：

- witness path 有序，直接表达 EPIC 选中的路线。
- 全量 Bubble 无序且数量不固定，包含大量与当前 accepted route 无关的空间。
- edge witness 不严格对应一串原始 Bubble；沿 witness 查询安全半径更接近实际安全走廊。
- 有序 `[center, safe_radius]` 同时保留路线方向、转弯位置和允许偏离宽度。
- 路径上的 TopoNode 代表 Bubble 仍应保存，用于验证 corridor bubbles 与拓扑路线一致。

### 3.2 不重新实现另一套 A*

离线路线标注器必须复用当前 EPIC 的 TopoGraph、Bubble A* 和 witness 拼接逻辑。
训练标签和在线部署若使用两套搜索逻辑，会产生不可控的路径分布偏差。

### 3.3 Witness 同时进入模型输入和 loss

witness corridor bubbles 不是只作为网络条件输入，还必须直接参与 trajectory loss 和
score label 的计算，否则模型没有足够的训练信号学习服从路线。

具体契约：

- 网络输入包含当前 route 的有序 witness corridor bubbles。
- 将每条预测五次多项式轨迹离散成采样点。
- 使用 witness corridor bubbles 计算轨迹的走廊越界代价 `L_path_corridor`。
- 使用 witness 的弧长方向计算前向进度代价 `L_path_progress`。
- 使用 witness 的局部切向计算末端速度方向代价 `L_path_tangent`。
- 上述路线代价与安全、平滑和加速度代价相加，形成 trajectory total cost。
- trajectory total cost 的 detached 值作为对应 primitive 的 score label。

这里不采用的是“将 witness 折点按时间对齐后作为专家轨迹坐标进行逐点回归”的传统
行为克隆。witness 是几何安全走廊，不包含速度、加速度和时间参数；YOPO 仍负责在该
走廊内优化动力学可执行轨迹。

### 3.4 统一 RouteQualityGate

EPIC accepted route 是上游规划结果，不自动等于可用于训练或执行的合格 route。数据
标注器、Dataset 和在线 YOPO 必须复用同一套 `RouteQualityGate`。以下任一硬条件失败，
V1 训练样本必须拒绝，在线新 route 也不得替换当前有效 route：

- EPIC 报告 `found=false`、`blocked=true` 或 route 未正式提交。
- route、frontier 或 Bubble 半径包含非有限值，或 witness 少于两个不同点。
- witness 起点无法与当前轨迹起点连续连接，或末端与 frontier 不一致。
- 对完整 witness 折线连续审计后，任一点的 clearance 小于机器人半径与安全裕量之和。
- 相邻 witness corridor bubbles 无法覆盖两者之间的 witness segment。
- 前向可执行长度不足一个 YOPO 规划时域，且当前 route 不是合法近终点短路线。
- 前缀转角、曲率或相对机体方向超出当前 YOPO lattice 和动力学可达范围。
- 在线 route 已过期、`route_id` 回退，或与发布时 odom 的时空偏差超过阈值。
- 新旧 route 的受保护前缀发生超过阈值的横向跳变，且 EPIC 未提供硬阻塞或合法切换原因。

除硬拒绝外，可为接近阈值但仍合格的 route 保存 `[0, 1]` 质量权重，供后续消融；V1
默认只训练通过全部硬门槛的样本。每次拒绝必须记录稳定的 reason code，不能只输出
自由文本日志。

### 3.5 与图侧性能工作的边界

Route-Conditioned YOPO 不解决 TopoGraph persistence、窗口更新和 `边 x 语义` 冗余等
图侧性能问题。图性能优化使用独立 TODO 批次、独立指标和独立验收，不作为本批次模型
实验的隐式收益，也不因本批次开始而暂停。两条工作流只通过稳定的 `RouteCondition`
契约和 EPIC accepted-route 质量门槛耦合。

## 4. P0 数据制作

### 4.1 场景和坐标契约

- [ ] 定义数据版本，例如 `routeDatasetVersion = 1`。
- [ ] 数据统一使用 `world_enu` 世界坐标和 `body_flu` 机体坐标。
- [ ] 实现并测试 AirSim NED/FRD 到 ENU/FLU 的位置、四元数和点云转换。
- [ ] 固定高度 V1 使用在线默认高度 `z = 1.6 m`，但不得删除 z 字段。
- [ ] 相机 FOV、最大深度和外参写入场景元数据，不依赖训练代码中的隐式默认值。
- [ ] 训练与在线推理共用同一个 route 裁剪、重采样和坐标转换实现。

### 4.2 修正快照位姿采样

当前 `PoseSampler` 只在矩形范围内均匀采样，尚未像 YOPO-Simple 一样检查最近障碍物。

- [ ] 从 `tree.ply` 建立 KD-tree。
- [ ] 拒绝 clearance 小于 `safe_dist` 的采样位置。
- [ ] 拒绝地图边界、地面高度、非有限值和无有效深度的样本。
- [ ] 保存采样 seed，保证数据集可复现。
- [ ] 输出采样尝试数、障碍拒绝数、边界拒绝数和最终接受率。

建议 pilot 配置：

```text
scene_count:       2
frames_per_scene:  500
routes_per_frame:  3
safe_dist:         >= 0.6 m
altitude:          1.6 m
```

完整训练集规模在 pilot 验证后提高到约 50,000 至 100,000 个 route-conditioned 样本。

### 4.3 提取可复用 EPIC 核心

当前 topology 源码直接编译进 `epic_graph_node` 和测试程序，尚未形成可链接库。

- [ ] 将 `TOPO_SOURCES` 建成共享的 `scalenav_topology` CMake target。
- [ ] 保持 `epic_graph_node` 和现有测试链接新 target，确认行为不变。
- [ ] 从 ROS2 node 中提取纯路线拼接函数，输入 topology path，输出去重后的 edge witness。
- [ ] 为正向/反向 edge witness、重复端点和空 edge path 增加单元测试。
- [ ] 为 frontier、witness 末端和 route terminal 一致性增加契约测试。

### 4.4 新增离线路线标注器

新增 C++ 可执行程序 `epic_route_labeler`：

- [ ] 每个场景只加载和体素化一次 `tree.ply`。
- [ ] 使用与在线节点一致的地图、Bubble、A*、安全距离和 frontier 参数。
- [ ] 从数据帧读取起点位置、姿态和相机朝向。
- [ ] 为每帧采样多个 mission goal。
- [ ] 调用 `updateOdomNode()` 和 `goalDirectedSearch()`。
- [ ] 拼接 edge `paths_` 得到 accepted witness polyline。
- [ ] 对 witness 进行连续性、碰撞、有限值和最小长度检查。
- [ ] 沿 witness 以不大于 `0.25 m` 的步长连续查询 clearance。
- [ ] 在高曲率、clearance 接近硬阈值或 clearance 梯度较大的区段加密到不大于 `0.1 m`。
- [ ] 使用 `clearance - robot_radius - safety_margin` 生成 witness corridor bubble 半径。
- [ ] 半径小于等于零的 witness bubble 必须使该 route 标注失败。
- [ ] 保存 selected topology path 上每个 TopoNode 的 center、代表性 `bubble_radius_` 和 persistent id。
- [ ] 验证相邻 corridor bubbles 有足够重叠，不能在安全管道中产生未覆盖间隙。
- [ ] 调用统一 `RouteQualityGate`，输出 hard reject 和 soft quality 的全部分项。
- [ ] 保留规划失败记录及失败原因，不能只静默丢弃失败样本。

goal 采样至少覆盖：

- 直行长路线。
- 左转和右转路线。
- 障碍物绕行路线。
- 接近 mission goal 的短路线。
- 初始路线方向接近 YOPO lattice 可达边界的困难样本。

不能给 YOPO 生成当前 primitive 集合物理上无法执行的反向路线。当前水平 primitive
终点覆盖约 `[-51 deg, 51 deg]`，超出范围的样本应单独统计，并在扩展 lattice 前排除
出 V1 训练集。

### 4.5 路线文件格式

每个场景继续保留：

```text
Scene_0001/
  data.toml
  tree.ply
  Textures/
    depth_000000.exr
    rgb_000000.png
```

新增 `routes.npz`，避免 NumPy object array 和 `allow_pickle=True`：

```text
frame_index           int64    [R]
mission_goal_world    float32  [R, 3]
frontier_goal_world   float32  [R, 3]
path_offsets          int64    [R + 1]
path_points_world     float32  [P, 3]
path_clearance_m      float32  [P]
path_bubble_radius_m  float32  [P]
topo_offsets          int64    [R + 1]
topo_centers_world    float32  [T, 3]
topo_bubble_radius_m  float32  [T]
topo_persistent_id    uint64   [T]
path_length_m         float32  [R]
route_valid           uint8    [R]
route_quality_flags   uint32   [R]
route_quality_weight  float32  [R]
route_min_clearance_m float32  [R]
route_max_curvature   float32  [R]
route_seed            int64    [R]
```

其中第 `r` 条路线的点范围为：

```text
path_points_world[path_offsets[r]:path_offsets[r + 1]]
```

原始 witness 必须完整保存。模型使用的固定数量 waypoint 在 Dataset 中生成，不能写死
在离线标签文件中。

`path_clearance_m` 是到最近障碍物的原始距离，供数据审计和重新配置安全裕量使用；
`path_bubble_radius_m` 是按生成时机器人半径和安全裕量计算后的可用安全半径。两者不能
混用。`topo_*` 字段保存 selected topology path 的代表 Bubble，仅用于诊断、可视化和
后续消融，V1 模型默认读取 `path_points_world + path_bubble_radius_m`。

### 4.6 路线多样性与反事实样本

- [ ] 每张深度图至少保留 2 至 4 条不同路线。
- [ ] 尽量生成同一 depth、相近 frontier、不同左/右绕行 path 的样本。
- [ ] 统计前 10 m 的转角、曲率、路线长度和 clearance 分布。
- [ ] 防止绝大多数样本都是正前方直线，否则网络会忽略 path 输入。
- [ ] train/validation/test 按场景或空间区域划分，不按相邻帧随机划分。

### 4.7 ESDF 分块

原始 `SafetyLoss` 为整张点云建立稠密 ESDF。ScaleNav 场景较大时可能造成内存或显存
不可接受。

- [ ] 测量每个实际场景在 `0.2 m` ESDF 分辨率下的内存需求。
- [ ] 超出预算时，将场景预计算成带 halo 的局部 ESDF tile。
- [ ] 每个 route 样本保存 `sdf_tile_id`。
- [ ] tile 必须覆盖起点周围至少一个完整 YOPO 规划半径及插值余量。
- [ ] 验证 tile 边缘的距离查询与完整场景 ESDF 一致。

### 4.8 数据验证与可视化

- [ ] 扩展 `validate_snapshot_dataset.py` 检查 `routes.npz`。
- [ ] 检查每条有效路线至少包含 2 个不同点。
- [ ] 检查 path 起点靠近采样位姿，末端靠近 frontier。
- [ ] 检查点间距、路线连续性、最小 clearance、Bubble 半径和坐标有限性。
- [ ] 检查相邻 witness corridor bubbles 的覆盖连续性。
- [ ] 对原始 witness 每个 segment 做连续 clearance 审计，不能只检查已保存的 Bubble 中心。
- [ ] 使用加密采样复查高曲率和窄走廊区段，确保降采样没有隐藏局部最小 clearance。
- [ ] 对每个 `RouteQualityGate` reason code 构造正反测试样本。
- [ ] 新增三维查看器，同时显示点云、相机视锥、机体轴、mission goal、frontier、witness、路径 TopoNode Bubble 和 witness corridor bubbles。
- [ ] pilot 数据必须人工抽查至少 100 条路线后才能进入模型开发。

## 5. P1 Dataset 和模型

### 5.1 固定大小路线表示

Dataset 从起点在 witness 上向前裁剪。模型输入使用可配置数量 `K` 的 Bubble，pilot
默认 `K=12`，并按非均匀弧长提供基础锚点：

```text
s = [1, 2, 3, 4, 5, 6, 8, 10, 14, 18, 24, 30] m
```

路线不足对应距离时重复 terminal，并通过 `route_mask` 标记无效尾部。`K=8/12/16`
必须作为 pilot 消融，而不是在接口中永久写死 8 点。

- [ ] 将 waypoint 转成相对于轨迹起点的 body-FLU 向量。
- [ ] waypoint 方向按对应弧长归一化，避免近点数值过小、远点数值过大。
- [ ] 重采样必须保留显著转角点和局部最小 clearance 点；固定弧长锚点不足时优先替换冗余直线点。
- [ ] 输入 Bubble 半径使用其负责弧长区间内的最小 safe radius，不只使用中心点 clearance。
- [ ] frontier 使用现有 goal 归一化方式，保留近终点距离信息。
- [ ] clearance 裁剪到固定上限后归一化。
- [ ] 对路径截断、重复 terminal、零长度路径和近终点路径增加单元测试。

Batch 接口：

```text
depth             [B, 1, 96, 160]
motion            [B, 6]
frontier_body     [B, 3]
route_bubbles     [B, K, 4]    # body-FLU center xyz + normalized safe radius
route_mask        [B, K]
```

训练 loss 还需要世界坐标位姿、稠密 route、dense route mask 和 `sdf_tile_id`。Loss
不得使用模型输入的 K 点近似代替稠密 witness，否则会漏掉窄走廊和急转弯监督。

### 5.2 Route-Conditioned YOPO V1

- [ ] 保留 `[velocity, acceleration, frontier]` 作为原来的 9-D observation。
- [ ] 使用 `StateTransform` 将 frontier 和每个 witness bubble center 转换到每个 primitive frame。
- [ ] 将 witness bubble safe radius 和 route mask broadcast 到 `vertical_num x horizon_num`。
- [ ] 与 64 通道 depth feature 拼接。
- [ ] 扩展 YOPO head 的第一层输入通道，保持输出仍为 9-D endstate 和 1-D score。
- [ ] 保持 15 个 primitive 的排列和图像网格顺序不变。
- [ ] 为 forward、inference、TorchScript export 和 reload 增加 shape/finite 测试。

不建议 V1 直接使用全局 path embedding。当前 YOPO head 是共享的 1x1 convolution，
若路线特征没有转换到 primitive frame，开阔场景中各 candidate 很难获得不同的路线关系。

### 5.3 在线原子接口

`/epic/path` 和单独的 frontier topic 可能来自不同规划版本。正式在线接口应使用单条
原子消息，例如：

```text
RouteCondition.msg
  std_msgs/Header header
  uint64 route_id
  uint32 quality_flags
  geometry_msgs/Point frontier_goal
  geometry_msgs/Point[] witness_bubble_centers
  float32[] witness_bubble_radii
  float32 remaining_length
  bool valid
```

- [ ] EPIC 每次提交 accepted route 时递增 `route_id`。
- [ ] 同一消息发布 frontier 和有序 witness corridor bubbles。
- [ ] EPIC 在消息中明确发布 blocked、切换原因和质量标志，不让 YOPO 从几何变化猜状态。
- [ ] YOPO 使用统一 `RouteQualityGate` 拒绝过期、空、非连续、突跳或距离当前无人机过远的 route。
- [ ] 在线预处理必须调用与训练 Dataset 相同的 route transform。
- [ ] 新 route 失败时，只有上一 route 仍新鲜、连续、未阻塞且剩余长度充分，才允许继续沿用。
- [ ] 所有模式切换发布结构化事件，包含 mode、route_id 和 reason code。

在线降级必须是显式状态机：

```text
ROUTE
  -> 当前 route 合格：frontier + witness corridor 正常推理
  -> 新 route 不合格但上一 route 仍合格：保持上一 route

FRONTIER_ONLY
  -> 无合格 route，但 frontier 新鲜且有限：route_mask 全零，仅使用 frontier 和 depth

SAFETY_HOLD
  -> route 与 frontier 均不合格：受控减速/悬停，不得静默跟随过期走廊
```

`FRONTIER_ONLY` 是正式模型契约，训练集需要加入 route dropout 样本；`SAFETY_HOLD`
优先使用确定性的制动/悬停控制，不把无目标的 YOPO primitive 选择冒充安全保证。

## 6. P1 损失函数

总损失建议为：

```text
L = L_safety
  + L_smooth
  + L_acceleration
  + w_path * L_path_corridor
  + w_progress * L_path_progress
  + w_tangent * L_path_tangent
  + w_frontier * L_frontier
```

- [ ] 复用 `SafetyLoss` 中五次多项式的 30 个轨迹采样点。
- [ ] 实现采样点到 witness segment 的可微最近距离。
- [ ] `L_path_corridor` 使用 witness bubble safe radius 构造可变宽度走廊，并限制最大宽度，避免开放区域完全失去路线约束。
- [ ] `L_path_progress` 约束末端沿 witness 前进，防止低速停留在起点附近。
- [ ] `L_path_tangent` 约束末端速度与路线局部切向一致。
- [ ] `L_frontier` 只保留低权重长程方向约束，避免朝 frontier 直线切弯。
- [ ] score target 使用所有真实 cost 的 detached 总和。
- [ ] 分别记录各 loss、selected cost、oracle cost、selection regret 和 top-1。
- [ ] `FRONTIER_ONLY` 样本关闭 route loss，只保留安全、平滑、加速度和 frontier guidance。

## 7. P2 训练方案

### 7.1 权重初始化

- [ ] 从现有 YOPO-Simple checkpoint 加载 depth backbone。
- [ ] 保留原 head 中 depth、velocity、acceleration 和 goal 对应权重。
- [ ] 将 frontier 放在旧 goal 的 3 个通道，最大化预训练权重复用。
- [ ] 新 route、clearance 和 mask 通道使用小值初始化。
- [ ] 输出 checkpoint 时同时保存数据版本、route 参数和 loss 权重。

### 7.2 分阶段训练

阶段 A：接口和新通道预热。

- 冻结 depth backbone 3 至 5 epoch。
- 使用直线和低曲率 route 为主。
- 验证 path 输入改变时网络输出会相应变化。

阶段 B：全量微调。

- 解冻全部网络。
- 加入完整转弯、绕行和近终点样本。
- 训练约 30 至 50 epoch。
- 根据 validation `selected_total_cost` 和 route 指标保存 best，而不是默认使用最后一轮。

阶段 C：困难样本。

- 增加低 clearance、强转弯和 lattice 边界样本。
- 加入深度噪声、缺失值和轻微位姿扰动。
- 加入明确标记的 route dropout 样本，训练 `FRONTIER_ONLY` 降级模式。
- 不对 route geometry 和 depth 使用相互不一致的数据增强。

### 7.3 关键消融

- [ ] Depth + frontier，不输入 path。
- [ ] Depth + frontier + path centerline。
- [ ] Depth + frontier + path centerline + witness bubble safe radius。
- [ ] `K=8/12/16` witness bubble 数量消融。
- [ ] 去掉 `L_path_tangent`。
- [ ] 去掉 `L_frontier`，检查是否影响长程稳定性。

## 8. 验收指标

### 8.1 数据质量

- 有效路线比例和各种失败原因。
- 路线长度、前 10 m 转角、曲率、clearance 和 witness bubble safe radius 分布。
- 左转、右转、直行、绕行、近终点样本比例。
- 坐标转换和路径连续性错误数必须为零。
- RouteQualityGate 各 reason code 的计数、占比和空间分布。
- 稠密 clearance 审计值与降采样 Bubble 半径之间不得出现非保守误差。

### 8.2 离线模型

- collision cost 和最小 ESDF 距离。
- 到 witness 的轨迹平均距离和最大距离。
- 沿 witness 的实际 progress。
- 末端速度与 path tangent 的夹角。
- selected total cost、oracle total cost、score regret 和 top-1。
- 同一 depth 下切换不同 path 时，预测轨迹必须产生可解释的方向变化。

### 8.3 闭环仿真

- 任务到达率。
- 碰撞率和 emergency stop 次数。
- 飞行轨迹到 accepted witness 的平均和最大偏差。
- frontier/path 更新时的控制连续性。
- 路线切换后的响应延迟。
- YOPO 推理耗时和全链路控制频率。

## 9. 实施顺序

严格按以下顺序推进：

1. 坐标和 `routes.npz` 数据契约。
2. 快照安全位姿采样。
3. `scalenav_topology` 可复用库。
4. `epic_route_labeler`。
5. `RouteQualityGate`、pilot 数据生成、稠密 clearance 审计和三维可视化。
6. Route Dataset 与固定 waypoint 预处理。
7. Route-Conditioned YOPO V1。
8. route loss 和训练指标。
9. pilot 训练及消融。
10. 原子 ROS2 route 接口、三级降级状态机和闭环仿真。

在步骤 5 通过前，不进入模型训练；在离线反事实 path 测试通过前，不接入在线控制。
