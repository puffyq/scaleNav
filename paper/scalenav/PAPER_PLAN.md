# ScaleNav 论文计划

## 1. 核心定位

ScaleNav 不是新的局部轨迹规划器，而是位于感知与现有局部规划器之间的
**即插即用语义拓扑地图层**。它将 RGB-D、里程计和文本查询转换为持久的
自由空间拓扑、远场语义风险以及滚动局部路线；拓扑 GCN 直接在在线 skeleton
上学习 35 m 前瞻的五方向 frontier policy，使只具备局部感知能力的
YOPO、EGO-Planner 和 SUPER 能够提前选择绕过长距离、大尺度或细小障碍物的
路线。

当前标题：

> **ScaleNav: A Plug-and-Play Topological Route Layer for Language-Guided Aerial Navigation**

核心英文表述：

> ScaleNav is a plug-and-play semantic topological mapping layer that equips
> existing local planners with persistent, long-range route guidance for
> bypassing large and visually recognizable obstacles, while a topology-graph
> GCN learns long-range frontier selection directly on the online skeleton.

## 2. 要解决的问题

现有局部规划器主要受到三类限制：

1. 感知和规划窗口有限，无法在长墙、建筑群或大面积树林前及时决定从哪一侧绕行。
2. 局部状态缺少持久路线记忆，异步更新后容易改变 homotopy 或左右振荡。
3. 深度对远距离和细小目标不可靠，但 RGB 仍可能识别电线、围栏和树枝等风险。

ScaleNav 的目标不是替代局部规划器的动态可行性和避碰能力，而是向其补充持久、
远距离、语言条件化的路线信息。

## 3. 论文贡献

### 3.1 即插即用的地图接口

- 输入：RGB-D、odometry、文本查询。
- 对局部规划器的唯一输出：沿 accepted witness 滑动的 world-frame local goal。
- `frontier_goal` 是 ScaleNav 内部路线锚点和诊断量，不是当前闭环 executor 输入。
- YOPO、EGO-Planner 和 SUPER 通过轻量 adapter 使用相同地图层。
- 当前闭环不向局部规划器传递 corridor、bubble sequence、route mask 或 graph feature。
- 不修改 planner core，不重新训练 YOPO；但 EGO/SUPER 仍需要 goal/cloud/command topic adapter。

### 3.2 面向大尺度绕障的持久自由空间拓扑

- 使用 collision-checked free-space bubbles 和 witness paths 表示可执行连接。
- 保留已经探索过的拓扑，使路线决策可以超出单帧深度和局部规划范围。
- 使用 world-frame witness 和 persistent identity 跨异步图替换保持路线一致性。

### 3.3 Topology-Graph GCN Frontier Policy

- 在当前在线 skeleton 上插入五个虚拟方向候选，通过两层 weighted GCN 聚合拓扑邻域。
- 使用 global mean pooling 和上一帧方向的 GRU 状态，在统一图上下文中比较五个候选。
- 使用完整 Map2 静态点云 A* 生成 35 m privileged direction label，并按 session 划分数据。
- GCN 是核心的学习式长距离决策模块；它只输出 frontier proposal，A*、witness 和碰撞检查继续负责几何有效性。

### 3.4 基于 RGB 的远场和细障碍语义

- 将开放词汇热力图投影成远场语义节点和连续 witness risk。
- 深度尚未返回时，RGB 语义仍可提前改变路线偏好。
- 地图接口与热力图分辨率解耦，可接收任意校准分辨率的语义图。
- 不能写成“识别效果不受分辨率影响”；实际识别率仍由语义前端和目标像素宽度决定。

### 3.5 明确的安全边界

- 语义只改变候选路线代价，不改变几何 edge validity。
- 语义假设不能把未知空间或占据空间认证为自由空间。
- 当前深度、几何检查和下游局部规划器继续负责最终执行安全。

### 3.6 可控的在线工作集

- Rolling A* 只在 40 m 局部窗口内展开节点。
- 每条边只评估固定数量的附近语义候选。
- 点缓冲区和 virtual semantic nodes 具有显式上限。
- Verified persistent backbone 当前没有全局节点硬上限，因此只能声称
  **在线计算工作集可控**，不能声称全局持久图存储严格有界。

## 4. 主张与证据

| 论文主张 | 必需证据 | 当前状态 |
| --- | --- | --- |
| 拓扑路由层可接入不同局部规划器 | YOPO/EGO/SUPER 与对应 ScaleNav 版本的同场景交错对比 | 三种 executor 均已完成同一 Map2、同一起终点的 10 次独立批次：0%→100%、0%→70%、0%→80%；仍缺交错、逐 trial 对照 |
| GCN 能从在线拓扑学习长距离方向选择 | 独立测试集分类结果、手工 ranking 对照和闭环结果 | 最新可复现评估在 707 帧测试集上 exact/macro 为 89.3%/89.4%，within-one 为 99.4%，column MAE 为 0.115，验证集选择 penalty 0.00；GCN 为 10/10、手工 ranking 为 24/27，两者任务完全相同但批次独立采集、样本量不同 |
| 能改善长距离绕障 | 障碍尺度逐级增加，比较成功率、决策距离和路径长度 | 缺失 |
| 语义确实改变路线 | 固定几何和 graph snapshot 的 semantic on/off 以及 matched/irrelevant query 对比 | 已有同协议闭环 on/off 和 matched/irrelevant query（irrelevant prompt: `trees`）结果；固定 snapshot、路线切换和严格无关 query 仍缺失 |
| 能利用深度范围外的 RGB 信息 | 记录 RGB 语义出现、深度无返回、首次换路之间的同步时间和距离 | 只有定性图，缺少统计 |
| 能绕过电线等细障碍 | Depth-only 与 RGB-semantic 在细线场景中的闭环对比 | 缺失 |
| 热力图接口对分辨率解耦 | 多种输入分辨率下的路线一致性、召回率和运行时间 | 缺失 |
| 路线记忆减少振荡 | 去掉 world-frame route memory 的交错消融 | 缺失 |
| 在线资源适合大尺度运行 | 不同航程下的局部窗口、展开节点、延迟和真实 RSS | 已有 persistent/sliding 工作集与延迟对照；RSS、长航程压力测试缺失 |

## 5. 实验计划

### E1. Planner 插件式增强

对每个局部规划器进行成对比较：

- YOPO-Simple vs. ScaleNav+YOPO
- EGO-Planner vs. ScaleNav+EGO
- SUPER vs. ScaleNav+SUPER

协议：

- 至少包含长墙、建筑群/blocks、树林三个场景。
- 每个条件至少 10 次，使用同一静态地图、起点、终点和障碍序列，并按条件交错运行。
- 失败试验保留在 success/collision/timeout 中。
- 完成时间、完成路径和速度只统计成功试验；失败另报 truncated observed path。

主要指标：success、collision、timeout、成功路径长度、完成时间、平均速度和最大速度。

当前阻塞项：ScaleNav+EGO 最新回归为 7/10，ScaleNav+SUPER 为 8/10；
仍需同场景交错采集的逐 trial comparison，不能据此宣称对所有执行器都有统计显著的提升。

### E2. 障碍尺度与规划距离

构造几何相同但长度不同的长障碍，例如 20、40、60、80 m，覆盖和超过局部规划范围。

比较：

- 原始局部规划器
- 仅几何持久图
- 完整 ScaleNav

指标：成功率、碰撞率、首次稳定分支决策距离、路线切换次数、detour ratio 和完成时间。

该实验直接支撑“适合大尺度绕障”，比单一 140 m 航线更有解释力。

### E3. 语义条件与因果验证

在完全相同的几何、graph snapshots 和初始状态下交错比较：

- No query / semantic disabled
- 正确查询：avoid buildings、avoid trees
- 无关查询或负控制
- 未见类别：avoid power lines

除常规飞行指标外，必须报告：分支选择、query compliance、首次决策距离、route
switches 和 semantic exposure。

离线固定回放：

- 首先比较 `lambda_sem = 0` 与 `lambda_sem = 2`。
- 再测试 `lambda_sem = {1, 2, 4}`。
- 测试 `R_s = {4, 8, 12}` m，并保持 `sigma_s = R_s / 2`。

### E4. Topology-GCN 策略验证

- 保留当前独立测试集的 exact、macro、within-one-column accuracy 和混淆矩阵。
- 在相同 reset、语义配置和路线参数下交错比较 hand-designed ranking 与 GCN policy。
- 增加跨地图零样本测试，报告方向准确率、切换率、候选 witness 否决率和闭环结果。
- 将 GCN 贡献表述为 learned frontier selection，不把下游 A* 的安全检查归入网络输出。

### E5. 远场和细障碍实验

场景至少包含电线、围栏或细树枝，并保证一部分观察满足“RGB 可见、深度无有效返回”。

比较：

- Depth-only
- RGB semantic without persistence
- RGB semantic with persistent far-field nodes

指标：语义召回率、首次识别距离、首次换路距离、最小风险距离、碰撞率和成功率。

分辨率实验使用若干实际可部署的 RGB/heatmap 分辨率，并报告：目标像素宽度、检测
召回率、最终分支一致性、PEARL 延迟和 dropped/coalesced frames。该实验支撑
“resolution-agnostic interface”，而不是“resolution-independent perception”。

### E6. 组件消融

使用 current scene 和相同 reset 条件交错完成：

- Full ScaleNav
- w/o semantic front end
- w/o far-field semantic nodes
- w/o continuous witness risk，仅使用 endpoint score
- w/o world-frame route memory

语义前端关闭条件可以与 E3 的 No query 批次复用，但其他配置必须完全一致。

### E7. 持久性与资源

- 至少 10 次 outbound/return，或一条显著长于当前 140 m 的连续航程。
- 报告局部窗口节点、active A* 展开节点、edge evaluations、Mean/P95 时间。
- 增加真实 process RSS、peak GPU memory、YOPO latency 和 PEARL latency 日志。
- 将持久存储增长与在线工作集分开报告。
- 如果希望声称全局存储有界，必须先实现 verified backbone 的裁剪、压缩或硬上限。

## 6. 最低提交集

在投稿前至少完成：

- [x] 完成 ScaleNav+EGO 十次回归（7/10 成功，3/10 碰撞）；仍需交错对照。
- [x] 重测 ScaleNav+SUPER（8/10 成功）；仍需与基线交错对照。
- [ ] 完成三组 planner 的交错 plug-in comparison。
- [x] 完成 GCN 独立测试集评估和当前闭环批次；仍需 hand-designed/GCN 的交错对照和跨地图测试。
- [x] 完成同协议 semantic on/off 和 matched/irrelevant query（irrelevant prompt: `trees`）闭环对比；仍需固定 snapshot、路线切换和严格无关 query 对比。
- [ ] 完成 `lambda_sem=0` vs. `2` 的固定 snapshot 因果回放。
- [ ] 完成 far-field nodes、route memory 和 witness risk 消融。
- [x] 完成 persistent geometry 与 40-m sliding geometry 的在线工作集和延迟对照；仍需交错性能消融。
- [ ] 完成至少一个电线或其他细障碍闭环场景。
- [ ] 增加真实 RSS、PEARL latency 和 YOPO latency 记录。

增强项：

- [ ] 多障碍长度尺度实验。
- [ ] 多 RGB/heatmap 分辨率实验。
- [ ] 10 次往返或长航程持久性实验。
- [ ] 完整参数敏感性网格。

## 7. 论文结构建议

1. **Introduction**：局部规划器缺少远距离路线记忆；RGB 可识别深度范围外风险。
2. **Related Work**：局部规划、持久拓扑地图、开放词汇语义导航。
3. **Plug-and-Play Route Interface**：moving local goal、planner adapters、安全边界。
4. **Persistent Free-Space Topology**：bubbles、witness、异步替换和路线身份。
5. **Topology-Graph GCN Frontier Policy**：weighted GCN、GRU、35 m privileged labels 和 A* 安全边界。
6. **Semantic Foresight**：heatmap projection、far-field nodes、continuous witness risk。
7. **Bounded Online Search**：局部窗口、候选上限、复杂度和资源边界。
8. **Experiments**：跨 planner 闭环、GCN policy、语义消融和资源统计。
9. **Limitations**：跨地图泛化、全局 verified graph 存储增长、固定高度和语义误检。

## 8. 结果展示建议

- 主表：三组原始 planner 与 ScaleNav 增强版本的常规飞行指标。
- 主图：同一长障碍下原始 planner 与 ScaleNav 的成对轨迹。
- 语义因果图：同步 RGB、heatmap、depth、far-field nodes 和 branch change。
- 尺度图：成功率和决策距离随障碍长度变化。
- 细障碍图：不同 RGB 分辨率或 depth-only/RGB semantic 的电线绕行结果。
- 资源图：参考 EGO-Planner，使用模块化 Mean/P95 柱状比较，不使用全局节点作为
  rolling-A* workload 横轴。

## 9. 行文约束

在证据完成前不得使用以下表述：

- “ScaleNav makes all tested planners succeed.”
- “Performance is independent of RGB resolution.”
- “The complete persistent graph has bounded memory.”
- “Semantic observations guarantee safety.”

当前可以使用的准确表述：

- ScaleNav+YOPO 在最新 Map2 批次中完成 10/10 次任务。
- ScaleNav+SUPER 的最新批次为 8/10 成功，相比原始 SUPER 的 0/10 碰撞批次有改善，
  但两批次并非交错采集。
- ScaleNav+EGO 的最新十次回归为 7/10 成功、3/10 碰撞；该批次替代了早期 3/3 小样本回归，仍不足以证明统计上的性能改善。
- 同协议 GCN 语义消融为 semantic-on 10/10 成功、semantic-off 6/10 成功；两批次独立采集，不能作为逐 trial 配对估计。
- Rolling A* 使用局部空间窗口，语义风险不会改变几何有效性。
- 当前闭环只向 planner 发布 moving local goal，不使用 corridor-conditioned network 或 MPC refinement variant。

## 10. 数据与文稿位置

- 主文稿：`paper/scalenav/root.tex`
- 已归档闭环数据：`paper/scalenav/test_data/closed_loop/`
- 聚合指标：`paper/scalenav/test_data/aggregate_metrics.csv`
- 资源分析：`scalenav_ws/docs/test_reports/plot_graph_resource_scaling.py`
- 当前资源图：`paper/scalenav/pics/experiments/map2_0_140_1p6/graph_resource_scaling.pdf`
