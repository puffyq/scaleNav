# ScaleNav / EPIC 软件测试规格说明书

## 1. 测试范围与判定

本页规定 [ALGORITHM_DESIGN.md](ALGORITHM_DESIGN.md) 所述软件的单元测试、模块测试和集成测试。第 2 章逐项覆盖详细设计第 4 章的全部算法函数。重载函数必须对每一种参数类型执行相同断言；表中的“测试次数”是单次 CI/验收运行内的最少重复次数，“运行频率”是线上调用节奏。

Route-Conditioned YOPO 的训练数据、Dataset、模型、loss、离线评测和在线 route 接口另见
[YOPO_TRAINING_INTEGRATION_DESIGN.md](YOPO_TRAINING_INTEGRATION_DESIGN.md#5-测试规格)。训练侧
保留 `UT-RC/MT-RC/IT-RC/PERF-RC` 编号，不并入本页 M1-M6 的 151 项在线统计。

重要性定义：

- `P0`：飞行安全、路线正确性或核心数据一致性，失败阻断发布。
- `P1`：主要功能或实时性，失败阻断版本验收。
- `P2`：诊断、兼容或低风险辅助能力，允许单独排期修复。

测试状态定义：`已有测试` 表示已有 GTest、集成检查、在线检查或仿真证据；`测试设计已定义` 表示输入、输出、频率、判定、次数和重要性已经规定，但尚未归档执行证据；`已有测试，待场景复核` 表示已有函数级测试，仍需在目标场景中复核；`已测试通过`、`已测试失败` 和 `部分通过` 表示已有本页判定口径下的实际执行或日志复核结论。后续新增函数必须同时在算法设计和本页增加条目。

### 1.1 测试层级

| 层级 | 测试对象 | 隔离方式 | 入口 | 退出准则 |
|---|---|---|---|---|
| 单元测试 | 单个函数、重载、边界分支 | 内存夹具、确定性点集/图、mock ROS 参数 | 每次提交 | 全部 P0/P1 函数用例通过 |
| 模块测试 | M1-M6 各模块内部函数协作 | 合成消息流、合成地图、模块真实依赖 | 每次合并 | 模块接口、状态和时限均满足 |
| 集成测试 | ROS2 话题、并发 rebuild、EPIC-YOPO、仿真飞行 | 完整节点或 AirSim/Colosseum | 发布候选/版本验收 | 端到端安全、功能、频率和性能指标满足 |

### 1.2 用例规模

| 测试层级 | 用例数 | 覆盖范围 |
|---|---:|---|
| 单元测试 | 108 | 详细设计中的函数接口；重载、语义和 frontier 回归问题拆分测试 |
| 模块测试 | 26 | M1、M2 各 3 项，M3 6 项，M4 5 项，M5 5 项，M6 4 项，新增 loss 排序场景 |
| 集成测试 | 17 | ROS2 接口/并发 7 项，飞行场景 10 项 |
| 合计 | 151 | 功能、异常、频率、并发、实时性和任务活性 |

## 2. 单元测试用例

### 2.1 M1 感知同步与局部地图

| 用例 ID | 函数 | 输入 | 预期输出 | 运行频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---|---:|---|---:|---:|---|
| TC-M1-001 | `LIOInterface::init` | 缺省参数及测试 ROS node | 初始化完成 | 启动 1 次 | 参数可读，空 KD-tree 查询不崩溃 | 1 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M1-002 | `LIOInterface::IsInBox`（`Vector3f`） | 盒内、边界、盒外、禁区各 1 点 | bool | 按点 | 盒内/边界 true，盒外/禁区 false | 4 | P1 | 已测试失败：禁区返回 true（2026-08-27） |
| TC-M1-003 | `LIOInterface::IsInBox`（`PointType`） | 与 TC-M1-002 同坐标 | bool | 按点 | 与 `Vector3f` 重载逐项一致 | 4 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M1-004 | `LIOInterface::IsInMap`（`Vector3f`） | 地图内、内边界附近、外部点 | bool | 按点 | 仅满足 `1e-4` 内缩边界的点为 true | 6 | P1 | 已测试失败：未执行内缩（2026-08-27） |
| TC-M1-005 | `LIOInterface::IsInMap`（`PointType`） | 与 TC-M1-004 同坐标 | bool | 按点 | 与 `Vector3f` 重载逐项一致 | 6 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M1-006 | `LIOInterface::getDisToOcc`（3 重载） | 已知障碍集与同一查询点的三种类型 | 最近距离 | A*/安全空间热点 | 三种结果误差 `<1e-5 m`，等于几何真值 | 30/重载 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M1-007 | `LIOInterface::KNN` | 10 个已知点，`k=1,3,20` | 点及距离数组 | Bubble 生成 | 数量为 `min(k,N)`，按距离非降序且点匹配 | 10/配置 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M1-008 | `LIOInterface::boxSearch` | 已知点集及相交/空 AABB | 盒内点集 | 区域更新 | 仅返回闭区间内点；空盒返回空集 | 20 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M1-009 | `LIOInterface::updateCloudMapOdometry` | 重复帧、同体素多点、40 m 内外点 | 更新后的点云/KD-tree | 约 10 Hz | 重复帧无增长；保留近体素中心点；窗内点仅作为 M1 跨帧缓存，窗外点删除且不转为 M2 persistent node | 100 帧 | P0 | 已测试失败：静止后窗外点重入（2026-08-27） |
| TC-M1-010 | `ScaleNavGraphNode::onOdom` | 非单位四元数、递增时间戳轨迹 | 当前状态、姿态历史 | odom 频率 | 姿态归一化，位置正确，历史不超容量 | 1000 消息 | P1 | 部分通过：5491 条日志姿态有限且单位化；非单位输入和内部容量未直接观测（2026-08-27） |
| TC-M1-011 | `ScaleNavGraphNode::poseForCloud` | 精确、容差内、容差外时间戳 | pose 与 bool | 每点云/语义帧 | 前两者成功且姿态正确；超时返回 false | 100/场景 | P0 | 部分通过：日志覆盖 0/10-40/50-90 ms 分支，未直接比对返回 pose（2026-08-27） |
| TC-M1-012 | `ScaleNavGraphNode::onCloud` | 含 NaN、近点、40 m 外点的机体系点云 | 世界地图更新 | 约 10 Hz | 非法/窗外点不入图，合法点坐标变换正确 | 120 帧 | P0 | 部分通过：410 帧真实点云正常处理；NaN/40 m 外点未独立注入（2026-08-27） |
| TC-M1-013 | `ScaleNavGraphNode::onFreeRays` | 只含自由端点的消息 | M1 局部占据点数不变 | 约 10 Hz | 自由射线端点不写入跨帧占据缓存，也不生成 M2 persistent node；最近障碍距离不变 | 120 帧 | P0 | 已测试通过（GTest 120 帧；日志 410 帧，2026-08-27） |
| TC-M1-014 | `ScaleNavGraphNode::onCloud` | 同一体素多点、相邻体素点、叶尺寸 `0.1 m` 及低于 `0.05 m` 的配置、采样后为空的点云 | `latestCloudSnapshot`、M1 局部地图及诊断字段 | 约 10 Hz | 每体素至多保留一个代表点；M1 地图更新只使用采样结果；叶尺寸低于下限时钳位到 `0.05 m`；空采样帧不更新图且被丢弃 | 120 帧/配置 | P0 | 测试设计已定义 |

M1 单元测试执行明细、失败定位和日志证据见
[`TEST_REPORT_2026-08-27_M1_UNIT.md`](test_reports/TEST_REPORT_2026-08-27_M1_UNIT.md)。

### 2.2 M2 Bubble 与拓扑图

| 用例 ID | 函数 | 输入 | 预期输出 | 运行频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---|---:|---|---:|---:|---|
| TC-M2-001 | `projectGraphPoint` | 同一点，planar 开/关及层高 `1.6` | 投影点 | 每观测点 | 开启时 x/y 不变、z=1.6；关闭时完全不变 | 1000 | P0 | 已测试通过 |
| TC-M2-002 | `TopoGraph::init` | 合法地图与 A* 对象 | 初始化图 | 每次建图 | 区域参数、依赖指针和空图状态正确 | 10 | P1 | 已有测试，待场景复核 |
| TC-M2-003 | `TopoGraph::getIndex` | 区域中心、边界两侧、负坐标 | 区域索引 | 每节点/点 | 索引与配置分辨率的 floor 结果一致 | 1000 | P1 | 已测试通过 |
| TC-M2-004 | `TopoGraph::index2boundary` | 合法首/末索引及越界索引 | AABB 与 bool | 每更新区域 | 合法盒尺寸正确；越界返回 false | 100 | P1 | 已测试失败 |
| TC-M2-005 | `TopoGraph::getRegionNode` | 32 线程重复请求同一索引 | 唯一 RegionNode | 按需 | 所有返回指针相同，map 仅增加 1 项 | 100 轮 | P0 | 已测试通过 |
| TC-M2-006 | `TopoGraph::getRegionsToUpdate` | 占据/自由更新与前向 goal | 区域列表 | 每次重建 | 包含相关区域、优先前向且不超上限 | 50 | P1 | 部分通过 |
| TC-M2-007 | `TopoGraph::generateBubble` | 空旷、近墙和完全占据 AABB | Bubble 集 | 每更新区域 | Bubble 中心自由且半径不越过最近障碍 | 100/场景 | P0 | 部分通过 |
| TC-M2-008 | `TopoGraph::splitCubeBubbleGeneration` | 部分覆盖的大立方体 | 细分 Bubble | 每更新区域 | 未覆盖子区得到 Bubble，无中心落入障碍 | 100 | P0 | 测试设计已定义 |
| TC-M2-009 | `TopoGraph::supplementCubeBubbleGeneration` | AABB 与已有主 Bubble | 补充 Bubble | 每更新区域 | 覆盖缺口且不生成与主 Bubble 完全重复项 | 100 | P1 | 测试设计已定义 |
| TC-M2-010 | `TopoGraph::isCubeCoveredByBubble` | 全覆盖、部分覆盖、空集合 | bool | Bubble 热点 | 仅全覆盖返回 true | 1000/场景 | P1 | 测试设计已定义 |
| TC-M2-011 | `BubbleUnionSet::updateRegionNode` | 两簇 Bubble 的 RegionNode | 更新后的 topo 集 | 每更新区域 | 生成 2 个连通拓扑节点且归属正确 | 100 | P1 | 测试设计已定义 |
| TC-M2-012 | `BubbleUnionSet::unionSetCluster` | 相交链、分离簇、空集合 | TopoNode 集 | 每更新区域 | 传递相交合并；分离簇不合并；空输入为空 | 100/场景 | P0 | 已测试通过 |
| TC-M2-013 | `TopoGraph::removeNodes` | 含双向边与 witness 的删除集 | 更新后的图 | topology diff | 节点、反向邻接、路径和权重全部移除 | 100 | P0 | 已测试通过 |
| TC-M2-014 | `TopoGraph::updateRemainedConnections` | 安全、碰撞、超时的已有边 | 保留/修复/删除统计 | topology diff | 安全保留，碰撞删除，单次超时软重试 | 100/状态 | P0 | 部分通过 |
| TC-M2-015 | `TopoGraph::insertNodes` | 可连接、被墙隔断、超时节点 | 插入节点及边 | topology diff | 节点可存在；仅成功 witness 建立边 | 100/状态 | P0 | 部分通过 |
| TC-M2-016 | `TopoGraph::insertNode` | 新节点、2 邻居、2 witness | 双向邻接与路径 | 每新节点 | 两侧 neighbors/path/weight 一致且无半边 | 100 | P0 | 已测试通过 |
| TC-M2-017 | `TopoGraph::removeNode` | 已存在节点及同一节点再次删除 | 更新后的图 | 按需 | 首次完全删除，第二次无异常且无额外变化 | 100 | P0 | 已测试通过 |
| TC-M2-018 | `TopoGraph::deduplicateNearbyNodes` | 距离 `0.04 m` 重复点和 `0.20 m` 分支 | 删除数量及图 | 每次重建后 | 仅合并重复点，保留分支及全部 incident edges | 100 | P0 | 已测试通过 |
| TC-M2-019 | `TopoGraph::normalizeConnectivity` | 双向边和单向半边混合图 | 删除数量及图 | 每次重建后 | 半边删除，合法双向边/witness 保留 | 100 | P0 | 已测试通过 |
| TC-M2-020 | `TopoGraph::copyPersistentNodesFrom` | 含几何、语义、odom 节点的旧图 | 新图快照 | detached rebuild | 跨 rebuild 复制 persistent 节点/边/witness/语义，不复制临时 odom 或 M1 局部点云状态 | 100 | P0 | 已测试通过 |
| TC-M2-021 | `TopoGraph::updateSkeleton` | 连续地图快照：稳定、增障碍、移窗口 | 图与 timing | 重建周期 | 稳态不漂移；新障碍断开碰撞边；过程不崩溃 | 1000 次 | P0 | 部分通过 |
| TC-M2-022 | `TopoGraph::getBubbleSnapshot` | rebuild 并发读写 | Bubble 副本 | 发布周期 | 无竞态/崩溃，快照对象离开锁后可读 | 10000 次 | P1 | 测试设计已定义 |
| TC-M2-023 | `TopoGraph::searchPathWithBoundary` | 开放、墙阻断、极短预算的节点对 | 状态码、witness、剩余预算 | 每次边重验/插入 | 成功路径位于边界盒内；失败状态正确；预算只减不增 | 100/场景 | P0 | 测试设计已定义 |

### 2.3 M3 语义风险

| 用例 ID | 函数 | 输入 | 预期输出 | 运行频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---|---:|---|---:|---:|---|
| TC-M3-001 | `semanticFrameBaseline` | 正常值、NaN/Inf、空数组，q=`0,0.25,1` | `[0,1]` 基线 | 每语义帧 | 非有限值忽略；空为 0；分位值正确 | 100/配置 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M3-002 | `calibrateSemanticScore` | score/baseline 边界及非有限值 | `[0,1]` 风险 | 每 patch | 非有限为 0，基线扣除与上下界钳位正确 | 1000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M3-003 | `isSemanticRiskAnchor` | 分数/置信度恰低于、等于、高于门槛 | bool | 每语义节点 | 两值均达到门槛才 true；NaN/Inf false | 1000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M3-004 | `virtualSemanticPointFlu` | 中心/四角 patch，FOV 90x60，depth 30 m | FLU 点 | 每 patch | 所有点 optical x 增量 30 m；三行 z 有序且边角欧氏距离更大 | 1500 点 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M3-005 | `ScaleNavGraphNode::onSemanticHeatmap` | mono8、32FC1、错误编码、带姿态帧 | 世界风险点帧 | 约 2 Hz | 合法帧产生 15 patch；非法/无姿态帧丢弃 | 100/编码 | P0 | 部分通过：在线 32FC1 帧均产生 15 patch；mono8、错误编码和无姿态分支未受控注入（2026-08-27） |
| TC-M3-006 | `TopoGraph::insertSemanticNodes` | 新点、2.5 m 内重复点、被墙隔断点 | 插入/更新数及节点 | 每语义帧 | 近点复用 id；分数更新；失败连接无可执行边 | 100/场景 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M3-007 | `TopoGraph::updateNodeSemantic` | 观测 `0,0.8,1.2`，alpha `0,0.5,1`，递增时间 | 节点语义字段 | 每匹配点 | EMA 与钳位正确，次数递增，时间更新 | 100/配置 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M3-008 | `TopoGraph::semanticNodes` | 局部/远端、有/无有效证据节点 | 节点列表 | 搜索/发布周期 | 半径查询排除远点；仅返回有语义观测节点 | 1000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M3-009 | `TopoGraph::semanticMemorySnapshot` | 3 条不同 id 记录 | 记录副本 | 重建/语义周期 | 字段完整，修改副本不改变原记忆 | 100 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M3-010 | `TopoGraph::loadSemanticMemory` | 重复 id、新旧时间混合记录 | 图内记忆 | 每次建图 | 每 id 唯一且采用预期记录，无数量膨胀 | 100 | P1 | 已测试失败：同 id 的旧记录覆盖新记录（GTest，2026-08-27） |
| TC-M3-011 | `TopoGraph::semanticMemorySize` | 空、加载、更新后的记忆 | size | 诊断 | 始终等于 snapshot 条目数 | 1000 | P2 | 已测试通过（GTest，2026-08-27） |
| TC-M3-012 | `TopoGraph::restoreNodeSemanticMemory` | 同 id、近点、远点、冲突 id 节点 | 恢复数量/属性 | 每次重建 | 同 id/允许近点恢复，远点和 unavailable id 不恢复 | 100/场景 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M3-013 | `ScaleNavGraphNode::mergeSemanticMemory` | 同 id 的新旧时间记录 | 全局记忆 | 重建/语义周期 | 新记录覆盖旧记录，旧时间不反向覆盖 | 100 | P1 | 测试设计已定义 |
| TC-M3-014 | `ScaleNavGraphNode::semanticMemorySnapshot` | 与 merge 并发读 | 记录副本 | 重建周期 | 无数据竞争，字段自洽且 id 唯一 | 10000 次 | P1 | 测试设计已定义 |
| TC-M3-015 | `ScaleNavGraphNode::semanticRiskAlongRoute` | witness 上/边缘/带外风险点；上/中/下 row；FOV 中心/边缘；地面型响应；不同观测时间 | `[0,1]` 路线风险及分项诊断 | 规划/语义周期 | 风险同时考虑 row、FOV、地面可能性和时间置信度；风险随距离衰减；带外为 0；地面型响应降权但不直接清除空中风险 | 1000/场景 | P0 | 部分通过：距离衰减、影响带和置信度链路已有测试/日志；row、FOV、地面与时间矩阵未完整执行（2026-08-27） |
| TC-M3-016 | `ScaleNavGraphNode::updateTopoSemanticMemory` | 新鲜/过期/重复时间帧与安全 incumbent | 图更新及 replan latch | 规划周期 | 新鲜帧应用一次；过期/重复忽略；请求不清空路线 | 100/场景 | P0 | 部分通过：在线新鲜帧应用且 incumbent 持续保留；过期和重复帧未受控注入（2026-08-27） |
| TC-M3-017 | `ScaleNavGraphNode::updateTopoSemanticMemory` | 连续两帧相同 5x3 风险图；无人机沿前向移动 `2.65 m`；两帧 30 m 投影足迹重叠 | persistent id、节点数、观测次数 | 语义 2 Hz | 重叠区对应射线复用 persistent id，观测次数增加；节点数不增加 15 个 | 1000 对帧 | P0 | 部分通过：同名 GTest 实际只插入单点并做 `0.001 m` 级扰动，未执行 5x3、多帧移动、重建及 global/local/A* 工作集；见 [最新日志复核报告](test_reports/TEST_REPORT_2026-08-27_LATEST_LOG_REVIEW.md) |
| TC-M3-018 | `ScaleNavGraphNode::updateTopoSemanticMemory` | 同列上/中/下三 patch 分数 `0.2/0.6/0.9`，fixed-layer 开/关各一组 | 三个语义中心及对应分数 | 语义 2 Hz | 原始三维位置保持 row 间 z 差异；fixed-layer 不回写语义节点 z；上/中/下行信息均保留，不因排序或 `1.5 m` 去重随机丢行 | 1000 帧/模式 | P0 | 部分通过：投影 GTest 及真实快照均保留多高度；fixed-layer 开/关完整回调矩阵未执行（2026-08-27） |
| TC-M3-019 | `semanticObservationConfidence` | 单行、多行一致、多行冲突、FOV 中心/边缘、地面型射线 | 观测置信度 `[0,1]` | 每语义目标 | 输入差异只改变置信度和诊断项，不产生路线 hard reject；置信度不得把物理可行候选直接判为不可行 | 1000/场景 | P0 | 测试设计已定义 |
| TC-M3-020 | `semanticVerticalAggregation` | 同一目标由单行或上/中/下三行表达，含地面行和空中行 | 聚合风险、row 权重和置信度 | 每语义目标 | 三行不将风险简单累加三倍；地面行降权；上/中行高风险仍能贡献路线 loss；冲突行降低置信度而非取消资格 | 1000/场景 | P0 | 测试设计已定义 |
| TC-M3-021 | `TopoGraph::semanticNodes` | 当前 `semantic_stamp` 的 `Unknown` 虚拟节点、上一帧 `Unknown` 节点、历史 `Verified` 节点及语义流过期状态 | 搜索语义工作集及路线风险 | A*/发布周期 | 当前代 `Unknown` 参与；历史 `Unknown` 仍保存在 persistent memory 但不参与；历史 `Verified` 持续参与；语义流过期时所有 `Unknown` 均退出 | 1000/场景 | P0 | 已测试通过（GTest 及在线工作集日志，2026-08-27） |

M3 单元测试执行明细、失败定位和在线证据见
[`TEST_REPORT_2026-08-27_M3_UNIT.md`](test_reports/TEST_REPORT_2026-08-27_M3_UNIT.md)。

### 2.4 M4 搜索与统一代价

| 用例 ID | 函数 | 输入 | 预期输出 | 运行频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---|---:|---|---:|---:|---|
| TC-M4-001 | `ParallelBubbleAstar::init` | 缺省/平面参数及 LIO | 初始化搜索器 | 每次建图 | 分辨率倒数、安全空间和图层参数一致 | 10 | P1 | 部分通过：构建及搜索夹具完成初始化；10 组缺省/平面参数未逐项断言（2026-08-27） |
| TC-M4-002 | `ParallelBubbleAstar::reset` | 完成一次搜索后的缓存 | 空搜索状态 | 每次搜索 | safe/danger/open 状态不影响下一搜索 | 1000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M4-003 | `ParallelBubbleAstar::posToIndex` | 栅格中心、正负边界点 | 索引 | 搜索热点 | floor 映射正确 | 10000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M4-004 | `ParallelBubbleAstar::IndexToPos` | 正负索引 | 栅格中心点 | 搜索热点 | 与索引中心一致；往返误差不超半栅格 | 10000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M4-005 | `ParallelBubbleAstar::graphClearance` | 层上障碍、层下地面、空地图查询点 | 距离 | 每扩展节点 | 平面模式忽略地面；层上障碍距离正确 | 10000 | P0 | 部分通过：开放长边安全空间查询通过；层下地面与空地图未独立注入（2026-08-27） |
| TC-M4-006 | `ParallelBubbleAstar::isNodeSafe` | 安全、安全空间不足、AABB 外节点 | bool 与缓存 | 每扩展节点 | 仅安全且盒内节点 true；缓存重复结果一致 | 10000/场景 | P0 | 测试设计已定义 |
| TC-M4-007 | `ParallelBubbleAstar::search` | 开放空间、墙阻断、非法端点、极短 timeout | 状态码与 path | 每候选连接 | 分别返回成功/无路/端点失败/超时；成功 path 有序 | 100/场景 | P0 | 已有测试，待场景复核 |
| TC-M4-008 | `ParallelBubbleAstar::collisionCheck_shortenPath` | 可直连折线、绕障折线、碰撞折线 | bool 与短化 path | 每成功搜索 | 可见点被短化；绕障保留必要折点；碰撞 false | 100/场景 | P0 | 部分通过：开放长边检查通过；绕障和碰撞矩阵未完成规定次数（2026-08-27） |
| TC-M4-009 | `ParallelBubbleAstar::calculatePathCost` | 空、直线、多段折线 | cost | 每成功搜索 | 空路径有限安全；多段代价等于定义且随长度单调 | 1000 | P1 | 已测试失败：多段折线正确，空路径触发越界崩溃（2026-08-27） |
| TC-M4-010 | `TopoGraph::semanticRiskForEdge` | 中心直线安全但 witness 靠近风险点的边 | 风险值 | A* 每边 | 风险由 witness 折线决定且明显大于远离风险的边 | 1000 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M4-011 | `TopoGraph::clearanceCostForEdge` | 零、等于目标值、高于目标值及趋于无穷大的安全空间 witness | 安全空间代价 | A* 每边 | 代价等于 `w_clearance * edge_length * (target / (target + clearance))^2`；安全空间增大时单调递减；零安全空间时为 `w_clearance * edge_length`，等于目标值时为该值的 `1/4`，仅在安全空间趋于无穷大时趋近 0 | 1000 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M4-012 | `TopoGraph::routeEdgeCost` | 已知长度、语义风险、安全空间、incumbent 连续性及各权重 | 总边代价与分项代价 | A* 每边 | 统一代价同时覆盖几何、语义、安全空间和 incumbent 连续性；逐项改变输入时总 loss 单调且分项可追溯 | 1000/权重 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M4-013 | `TopoGraph::graphSearch` | 连通图、断图、终点在 35 m 外、超时 | bool 与节点路径 | incumbent 恢复 | 连通返回正确最小代价路径；其余失败且 path 不冒充成功 | 100/场景 | P0 | 部分通过：连通搜索及窗外拒绝通过；断图和受控超时未完成规定次数（2026-08-27） |
| TC-M4-014 | `TopoGraph::goalDirectedSearch` | 双走廊不同长度、任务方向、FOV、语义风险、安全空间和 smoothness 的物理可行 terminal | bool 与 terminal path | 候选重规划 | 所有物理可行 terminal 使用统一 loss 排序；较短但更安全的候选可以胜出；不得按距离单项或最远节点单项选择 | 1000/场景 | P0 | 部分通过：几何/语义/方向权重与安全候选排序通过；完整场景矩阵未达规定次数（2026-08-27） |
| TC-M4-015 | `TopoGraph::getPathLength` | 带 witness 的多边路径及空路径 | 总长度 | 路线评估 | 等于 witness 段总长；空/单节点为 0 | 1000 | P1 | 已测试失败：空路径和单节点路径均触发越界崩溃（2026-08-27） |
| TC-M4-016 | `ScaleNavGraphNode::connectTerminalToGoal` | 窗内开放、超距、墙阻断及 A* 超时的 goal | bool、extension 与发布 path | 目标接近时 | 仅窗内且搜索和碰撞复核均通过时追加 extension，端点误差 `<=0.5 m`；墙阻断或超时时保持于已验证 terminal，发布 path 不得拼接未经检查的 terminal-to-goal 直线 | 100/场景 | P0 | 已有测试，待场景复核 |
| TC-M4-017 | `TopoGraph::goalDirectedSearch` | 35 m 图内加入 235 个活动语义节点；候选路线几何/语义代价分别约 `34/47` 和 `31/50` | frontier terminal 与节点路径 | 候选重规划 | `31.5 m`、任务方向偏差和 FOV 位置均作为 loss 项，不作为 hard qualification；所有物理可行 terminal 均参与统一 loss，选择结果可由权重解释 | 1000 图种子 | P0 | 部分通过：径向安全绕行和较近安全 terminal 可胜出；未执行 1000 个日志规模图种子（2026-08-27） |
| TC-M4-018 | `TopoGraph::goalDirectedSearch` | 同一几何图分别加入 11 个和 235 个表达相同风险场的语义节点 | 两组 frontier terminal | 候选重规划 | 两组选择同一等价风险场下的同一排序；节点数量变化不改变排序，terminal 进度差 `<1 m` | 1000 图种子 | P0 | 部分通过：1 与 235 个等价风险节点重复 1000 次排序一致；未覆盖 1000 个不同图种子（2026-08-27） |
| TC-M4-019 | `frontierCandidateLoss` | 分别改变进度短缺、任务方向偏差、FOV 位置、语义、安全空间、smoothness | 总 loss 及分项 loss | 每候选 terminal | 每个 loss 项随对应输入单调变化；权重为 0 时该项不影响总 loss；总 loss 为各项加权和 | 2000/权重 | P0 | 部分通过：几何、语义、安全空间和方向权重单调性通过；FOV/smoothness 分项未逐项穷举（2026-08-27） |
| TC-M4-020 | `goalDirectedSearchSoftPreferences` | `27 m` 安全候选与 `32 m` 高风险候选；交换进度/语义/安全空间权重 | terminal 选择 | 候选重规划 | 默认权重下较短安全候选可胜出；交换权重后结果按预期可预测变化 | 1000/权重 | P0 | 部分通过：安全候选及交换几何/语义权重结果通过；未完成规定次数和全部权重组合（2026-08-27） |
| TC-M4-021 | `goalDirectedSearchPhysicalConstraints` | 断连、碰撞、安全空间不足、无有效 witness/local goal；另含后向、FOV 外、单行语义候选 | terminal 接受/拒绝结果 | 候选重规划 | 仅断连、碰撞、安全空间不足和无法形成有效 witness/local goal 属 hard reject；后向、FOV 外、单行语义只能增加 loss 或降低置信度，不能单独拒绝 | 1000/场景 | P0 | 已测试失败：有邻接和权重但无 witness 的节点仍被接受为 terminal（2026-08-27） |
| TC-M4-022 | `TopoGraph::goalDirectedSearch` | 从 odom 可达的 `Verified` 安全球、可达/不可达的 `Unknown` 虚拟语义点及不同语义方向风险 | terminal、节点路径及终端几何状态 | 候选重规划 | terminal 必须是 odom 可达的 `Verified` 安全球；`Unknown` 虚拟语义点只能改变候选路线或方向的语义 loss，不得直接成为 terminal；无可达真实安全球时不得输出虚拟 terminal | 1000/场景 | P0 | 已测试通过（GTest，2026-08-27） |

M4 单元测试执行明细、失败定位和历史变更证据见
[`TEST_REPORT_2026-08-27_M4_UNIT.md`](test_reports/TEST_REPORT_2026-08-27_M4_UNIT.md)。

### 2.5 M5 路线记忆与目标选择

| 用例 ID | 函数 | 输入 | 预期输出 | 运行频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---|---:|---|---:|---:|---|
| TC-M5-001 | `pointSegmentDistance` | 投影在线内、端点外、退化线段的点 | 距离 | 路线热点 | 分别等于垂距、端点距、点距 | 10000/场景 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M5-002 | `pointPathDistance` | 空、单点、多段路径及查询点 | 距离 | 路线热点 | 空为 infinity；其余为所有线段最小距离 | 10000/场景 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M5-003 | `forwardRouteWindow` | 无人机在首段中部，horizon 10 m | 限长前向折线 | 规划周期 | 首点是无人机投影，身后点删除，总长 `<=10 m` | 1000 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M5-004 | `forwardRouteFromPosition` | 无人机越过首点但仍在路线内 | 完整剩余折线 | 规划周期 | 首点是投影、末点与原 terminal 相同 | 1000 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M5-005 | `isContinuousForwardRoute` | 路线上、容差边界、平行走廊外、末端后无人机 | bool | 规划周期 | 仅仍有前向路线且在横向容差内时 true | 1000/场景 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M5-006 | `shouldSwitchRoute` | incumbent/candidate 完整 loss（几何、语义、安全空间、连续性、任务方向、FOV、smoothness）及硬失效状态 | bool 与切换原因 | 候选产生时 | 路线切换依据完整 candidate loss 和滞回；任务方向/FOV 只能进入 loss，不能单独 hard switch；硬阻塞仍可强制切换 | 1000/场景 | P0 | 已测试通过：聚合 loss 滞回、进度门槛和硬失效分支通过（GTest，2026-08-27） |
| TC-M5-007 | `edgeFollowsRoute` | 与路线重合、端点近但中点偏离、平行远边 | bool | A* 每相关边 | 仅起终点及中点均在容差内时 true | 1000/场景 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M5-008 | `routeLength` | 空、直线、多段、含非有限段路线 | 长度 | 路线评估 | 有限段正确累计，空路线为 0 | 1000 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M5-009 | `candidateExtendsAcceptedRoute` | 同走廊更长、近车换道、长度不足候选 | bool | 候选产生时 | 仅同走廊且达到最小增益者 true | 1000/场景 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M5-010 | `shouldReuseTerminal` | 距 terminal 大于、等于、小于释放距离 | bool | 规划周期 | 仅严格大于释放距离时 true | 1000/场景 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M5-011 | `canReuseForwardRoute` | 路线中段、离线、越过 terminal、剩余不足 | bool | 规划周期 | 仅在线且剩余距离大于释放阈值时 true | 1000/场景 | P0 | 已测试通过（GTest，2026-08-27） |
| TC-M5-012 | `semanticRiskIncreaseRequiresReplan` | 上升小于/等于/大于阈值、下降、NaN | bool | 每语义更新 | 达到阈值的有限上升 true，其余 false | 1000/场景 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M5-013 | `semanticRiskChangeRequiresReplan` | 与 TC-M5-012 相同 | bool | 每语义更新 | 与 increase 函数结果逐项一致 | 1000/场景 | P2 | 已测试通过（GTest，2026-08-27） |
| TC-M5-014 | `semanticRouteResetRequested` | 开关开/关及显著/微小风险上升 | bool | 每语义更新 | 仅开关开启且风险触发时 true | 1000/场景 | P1 | 已测试通过（GTest，2026-08-27） |
| TC-M5-015 | `ScaleNavGraphNode::nearestPersistentNode` | 正确 id、id 丢失近点、远点 | 节点或 null | graph swap/规划 | id 优先；后备不超距离；远点 null | 100/场景 | P0 | 测试设计已定义 |
| TC-M5-016 | `ScaleNavGraphNode::ensureOdomConnectivity` | odom 已连接、近邻开放、近邻被墙隔断图 | 新增连接数 | 规划周期 | 已连为 0；仅向安全邻居建立带 witness 双向边 | 100/场景 | P0 | 部分通过：在线覆盖连接成功及失败诊断；三场景函数级矩阵未完成（2026-08-27） |
| TC-M5-017 | `ScaleNavGraphNode::buildRememberedEdges` | 无人机位于 witness 中段的拓扑路径 | 边集合及数量 | 规划周期 | 只包含投影前方 accepted route 边 | 1000 | P0 | 部分通过：前向裁剪和 edge 匹配 GTest 通过，在线 remembered edge 持续更新；私有函数未直接执行 1000 次（2026-08-27） |
| TC-M5-018 | `ScaleNavGraphNode::selectNextGoal` | 平面 accepted path、无人机位置、10 m lookahead | goal 与 bool | 成功规划周期 | goal 在前方约 10 m；近 mission goal 时直接选 goal | 1000 | P0 | 部分通过：折线顺序前视 GTest 1000 次及在线 local goal 通过；近 mission goal 私有分支未受控执行 1000 次（2026-08-27） |
| TC-M5-019 | `ScaleNavGraphNode::update` | accepted witness 且未 blocked、terminal 可重映射；无人机相对 witness 横向偏离分别小于/大于 `route_reuse_lateral_distance_m` | terminal id、remembered edges、local goal 及切换原因 | 默认 10 Hz | 仅横向偏离不得清除 accepted witness、触发 `NO_ACCEPTED_ROUTE` 或切换 terminal；仍按 witness 前向投影发布 local goal | 1000/场景 | P0 | 部分通过：日志中横向偏离至 `5.81 m` 时仍保留 RECOVERED incumbent；未完成双边界 1000 tick（2026-08-27） |
| TC-M5-020 | `ScaleNavGraphNode::update` | accepted witness 执行进度 `<50%`、`=50%`、`>50%`；候选分别为兼容延伸、低收益分叉及显著更低 loss 分叉 | `frontier_half_replan`、候选搜索、提交结果及切换原因 | 默认 10 Hz | 未过半不因固定 `31.5 m` 或 horizon 触发刷新；过半触发候选搜索；兼容延伸可用 `FRONTIER_HALF` 提交；分叉仍须按总 loss 和滞回胜出 | 1000/场景 | P0 | 部分通过：日志覆盖 9 次兼容 FRONTIER_HALF、1 次 LOWER_LOSS 和 5 次候选滞回拒绝；精确 50% 边界矩阵未完成（2026-08-27） |
| TC-M5-021 | `ScaleNavGraphNode::update` | mission goal 进入 `35 m` 窗口，存在安全 incumbent；分别覆盖未过半、已过半和 blocked | terminal、切换原因及 candidate search | 默认 10 Hz | goal in window 只影响 candidate target/loss，不得单独清空 incumbent、强制重搜或 hard switch；只有过半、blocked、incumbent 丢失等状态可改变 terminal | 1000/场景 | P0 | 部分通过：goal 距离 30.28 m 及 0 m 时 incumbent 均可保持；未完成 blocked/过半组合 1000 tick（2026-08-27） |

M5 单元测试执行明细和在线证据见
[`TEST_REPORT_2026-08-27_M5_UNIT.md`](test_reports/TEST_REPORT_2026-08-27_M5_UNIT.md)。

### 2.6 M6 在线调度与发布

| 用例 ID | 函数 | 输入 | 预期输出 | 运行频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---|---:|---|---:|---:|---|
| TC-M6-001 | `ScaleNavGraphNode::configureMapBounds` | 地图、中心 `(0,0,1.6)`、margin 50 m | 初始边界 | 首目标/首帧 | 中心在边界内，各方向余量满足配置 | 20 | P1 | 测试设计已定义 |
| TC-M6-002 | `ScaleNavGraphNode::expandMapBounds` | 已有边界内点和边界外新点 | bool 与新边界 | 新目标/点云 | 内点 false 且不变；外点 true 且只扩不缩，点云保留 | 100 | P1 | 已有测试 |
| TC-M6-003 | `ScaleNavGraphNode::onGoal` | 首目标、重复目标、远端新目标 | mission goal 与规划状态 | 事件触发 | 首目标初始化；重复目标幂等；新目标清除旧 terminal 状态但按配置保留图 | 20/场景 | P0 | 测试设计已定义 |
| TC-M6-004 | `ScaleNavGraphNode::startSkeletonRebuild` | 100 Hz 并发触发、单次构建耗时 80 ms | rebuild 次数与交换图 | 默认约 10 Hz | 同时最多一个 worker，无 backlog，交换图可规划 | 1000 触发 | P0 | 已有测试 |
| TC-M6-005 | `ScaleNavGraphNode::update` | 正常、储备不足、语义上升、remap 失败、route blocked 状态 | 路线状态迁移 | 默认 10 Hz | 安全 incumbent 保留；兼容延伸提交；硬阻塞不发布旧路线 | 每场景 600 tick | P0 | 已有测试 |
| TC-M6-006 | `ScaleNavGraphNode::publish` | accepted、未接受 candidate、阻塞路线、hold timeout 状态 | ROS 输出与统计 | 规划周期 | 仅发布 accepted 安全 witness；local goal 连续；超时后停止保底 | 每场景 600 tick | P0 | 测试设计已定义 |
| TC-M6-007 | `ScaleNavGraphNode::update` | 连续 140 m 任务的图快照序列；每 tick 更新无人机位置并注入高语义代价 | frontier 序列、任务进度、完成状态 | 默认 10 Hz | 无硬阻塞时 frontier 的 mission-direction 累计进度单调增加；goal 进入 35 m 后选 goal terminal；最终进入到达容差 | 100 次任务 | P0 | 测试设计已定义 |

## 3. 模块测试用例

模块测试使用模块真实实现，不替换被测模块内部协作函数；模块外部依赖使用确定性夹具或 ROS2 测试节点。

### 3.1 M1 感知同步与局部地图

| 用例 ID | 测试接口/输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| MT-M1-001 | 10 Hz 点云 + 100 Hz odom，持续直线移动 60 s | 无人机中心滑动地图 | depth 10 Hz、odom 100 Hz | 每帧取得容差内姿态；世界点位置误差 `<0.05 m`；无积压 | 10 轮 | P0 | 测试设计已定义 |
| MT-M1-002 | 40 m 内外混合点、重复体素、容量超限点云 | 点云/KD-tree 查询 | 10 Hz | 窗外点从 M1 局部地图删除、体素唯一、容量固定、近障碍可查询；不得把 M1 点云当作跨 rebuild 持久图 | 600 帧 | P0 | 已有测试 |
| MT-M1-003 | 自由射线与占据点端点重叠输入，随后保持静止并移动出 40 m 窗口 | M1 局部占据点云/KD-tree | 两路各 10 Hz | 自由射线不作为新增障碍；真实占据点在 M1 中跨帧保留，移出窗口后删除；不得仅因写入 M1 而生成 M2 persistent node | 600 帧 | P0 | 已有测试 |

### 3.2 M2 Bubble 与拓扑图

| 用例 ID | 测试接口/输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| MT-M2-001 | 开放走廊点云，执行完整 `updateSkeleton()` | Bubble、节点、双向边、witness | rebuild 10 Hz | 图连通；每条边双向且 witness 通过碰撞检查 | 100 rebuild | P0 | 已有测试 |
| MT-M2-002 | 稳态图后插入墙面，再移出滑动窗口 | topology diff | rebuild 10 Hz | M1 窗口外点可删除；M2 已确认的 persistent 节点、边和 witness 按拓扑差分保留或安全删除；移窗后图可恢复连接 | 100 轮 | P0 | 测试设计已定义 |
| MT-M2-003 | graph rebuild 中注入边搜索 TIME_OUT/NO_PATH | 图与 timing 统计 | rebuild 10 Hz | TIME_OUT 保留安全旧边并重试；NO_PATH 新边不插入；无半边 | 1000 次 | P0 | 测试设计已定义 |

### 3.3 M3 语义风险

| 用例 ID | 测试接口/输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| MT-M3-001 | 5x3 热力图 + 对齐 odom + measured depth 5-20 m | 15 个世界语义点 | semantic 2 Hz | optical Z 恒为 30 m，与 measured depth 无关 | 200 帧 | P0 | 已有测试 |
| MT-M3-002 | 同方向重复风险观测，随后真实 Bubble 到达 | persistent semantic node | semantic 2 Hz、rebuild 10 Hz | id 不膨胀；EMA 正确；Unknown 提升为 Verified 且语义保留 | 200 帧 | P0 | 测试设计已定义 |
| MT-M3-003 | 风险点由影响带外移至 witness 上并保持稳定 | 路线风险/replan latch | semantic 2 Hz | 风险按距离上升；越阈值只锁存一次；释放阈值后可再次触发 | 100 周期 | P0 | 测试设计已定义 |
| MT-M3-004 | 无人机以 `5.3 m/s` 前进 60 s，输入稳定 5x3 语义图 | semantic graph、memory size、persistent id 序列 | semantic 2 Hz、planner 10 Hz | 相邻帧重叠足迹复用节点；增长量由新进入的空间决定，不得接近 `120 x 15`；无单帧 15 点爆发式累积 | 20 轮 | P0 | 测试设计已定义 |
| MT-M3-005 | 单行目标、三行一致/冲突目标及地面型响应，持续 200 帧并改变 row 分数 | 三维语义节点、聚合风险和置信度 | semantic 2 Hz | 单行和多行均贡献风险；多行一致性只调整置信度，不决定路线资格；地面型响应降权；fixed-layer 不压平原始节点 | 20 轮 | P0 | 测试设计已定义 |
| MT-M3-006 | 连续注入地面单行高响应、三行一致目标和 FOV 边缘目标 | 路线风险、candidate loss、local goal 偏航 | semantic 2 Hz、planner 10 Hz | 地面响应不会造成持续错误偏航；三行一致目标稳定提高 loss；FOV 边缘只降低置信度/增加软代价 | 20 轮 x 600 tick | P0 | 测试设计已定义 |

### 3.4 M4 搜索与统一代价

| 用例 ID | 测试接口/输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| MT-M4-001 | 双走廊图：短高风险与长低风险，逐级调整权重 | topology path + witness | 每规划 tick | 权重为 0 时选几何短路；语义权重提高后切换到安全路 | 100/权重 | P0 | 已有测试 |
| MT-M4-002 | 窄/宽走廊与相同长度、风险 | 路径及 clearance cost | 每规划 tick | 安全空间权重开启后选择宽走廊；关闭后代价只含几何/语义 | 100/配置 | P0 | 已有测试 |
| MT-M4-003 | 起终点、墙和 timeout 故障矩阵 | Bubble A* 与 TopoGraph A* 状态 | 每规划 tick | 底层状态不被误报为成功，上层不接收空 witness | 100/状态 | P0 | 测试设计已定义 |
| MT-M4-004 | 复现日志规模：活动图 436 节点、活动语义节点 235 个，语义代价高于几何代价 | frontier、完整 loss 分解、物理可行性 | planner 10 Hz | 删除任务前向带资格；所有物理可行 terminal 由统一 loss 排序，语义密度只通过等价风险场影响 loss | 100 轮 x 600 tick | P0 | 测试设计已定义 |
| MT-M4-005 | frontier 集合含近/远、前/侧/后、FOV 内外及不同语义风险候选 | 全候选 loss 表、最终 terminal | planner 10 Hz | 除物理不可行项外，所有候选参与同一 loss 排序；最终选择等于全量候选最小 loss，权重调整结果可预测 | 1000 集合 | P0 | 测试设计已定义 |

### 3.5 M5 路线记忆与目标选择

| 用例 ID | 测试接口/输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| MT-M5-001 | 无人机沿弯折 witness 越过多个首点 | forward route + local goal | planner 10 Hz | 持续裁掉身后路线，terminal 不变，local goal 无倒退 | 600 tick | P0 | 已有测试 |
| MT-M5-002 | incumbent 与同走廊延伸、平行换道、低收益 candidate | 接受/拒绝结果 | candidate 事件 | 延伸接受；换道与低收益拒绝；accepted witness 仅提交时改变 | 100/场景 | P0 | 已有测试 |
| MT-M5-003 | graph swap 后 terminal id 存在/丢失/临时不可达 | 恢复状态与输出路线 | rebuild 10 Hz | id 优先恢复；近点后备；旧 witness 安全时继续发布 | 100/场景 | P0 | 测试设计已定义 |
| MT-M5-004 | 连续局部图覆盖 140 m 路线，含必须侧向/短期后向绕障段，无人机跟随 local goal 更新 | accepted frontier/witness、滑窗任务进度和切换 loss | planner/rebuild 10 Hz | 允许统一 loss 选择必要的短期任务方向偏离；长期滑窗任务进度为正，路线不左右振荡，不停留在已消费 terminal | 100 次任务 | P0 | 测试设计已定义 |
| MT-M5-005 | witness 由已通过前缀、当前 FOV 内前向段和 FOV 外前向段组成；分别在三段投放障碍 | `route_blocked`、`route_probe_points` 及 accepted route | planner 10 Hz、depth 10 Hz | 已通过前缀与 FOV 外前向段障碍不得触发 blocked；当前 FOV 内且在 lookahead 内的障碍必须触发；探测在 FOV 边界停止 | 1000/场景 | P0 | 测试设计已定义 |

### 3.6 M6 在线调度与发布

| 用例 ID | 测试接口/输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| MT-M6-001 | 合成 M1-M5 状态依次触发 NO_ROUTE/TRACKING/BLOCKED | ROS 输出与状态日志 | planner 10 Hz | 状态转换符合详设；阻塞时不发布旧 local goal | 每状态 600 tick | P0 | 测试设计已定义 |
| MT-M6-002 | rebuild 耗时 80 ms、触发周期 100 ms | rebuild/graph age 统计 | 10 Hz | 同时最多一个 worker；无 backlog；最大图龄 `<=300 ms` | 10000 tick | P0 | 部分通过：worker/合成节奏检查有证据；真实日志 update 峰值 `198.386 ms`，未完成真实规模端到端预算验证 |
| MT-M6-003 | 规划间歇失败 300 ms/500 ms | local goal 输出 | planner 10 Hz | 300 ms 内安全目标可保底；500 ms 超时后停止保底 | 100/时长 | P0 | 测试设计已定义 |
| MT-M6-004 | 闭环执行：用每次 local goal 更新下一 tick 无人机位置，mission goal 距离 140 m | 完成标志、耗时、frontier/local goal 时序 | planner 10 Hz | 在规定最大时间内进入 mission goal 容差；若未完成，用例必须因 no-progress 或错误 frontier 失败 | 100 次任务 | P0 | 测试设计已定义 |

## 4. 集成测试用例

### 4.1 ROS2 接口与进程内集成

| 用例 ID | 场景输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| IT-ROS-001 | 启动 `scalenav_graph_node`，发布 odom/depth/goal | `/scalenav/path`、`/scalenav/local_goal` | odom 100 Hz、depth/planner 10 Hz | QoS 匹配；首次有效图后 2 s 内输出有限目标 | 20 次启动 | P0 | 测试设计已定义 |
| IT-ROS-002 | 时间戳乱序、语义延迟 300 ms、点云延迟 300 ms | 路线与告警 | 各输入额定频率 | 超容差数据丢弃，节点继续运行，accepted route 不被空数据清除 | 100 批次 | P0 | 测试设计已定义 |
| IT-ROS-003 | 规划、语义、点云和 rebuild 并发 30 min | 全部话题及 sanitizer | 10/2/10/10 Hz | 无死锁、数据竞争、崩溃和持续 backlog | 3 次 x 30 min | P0 | 测试设计已定义 |
| IT-ROS-004 | 动态改变 goal 到地图边界外 | 扩展地图、重规划路径 | goal 0.2 Hz | 边界只扩不缩，旧地图/图不损坏，新 local goal 朝新任务方向 | 50 个 goal | P1 | 测试设计已定义 |
| IT-ROS-005 | 发布 120 帧固定 5x3 热力图并同步发布匀速 odom，采集 graph snapshot | 节点数、id、z、score 时序 | semantic 2 Hz、odom 100 Hz | 节点增长非逐帧 15 个；重叠观测 id 连续；每列三行 z/score 对应稳定 | 20 次 x 60 s | P0 | 测试设计已定义 |
| IT-ROS-006 | ROS2 闭环测试节点跟随 `/scalenav/local_goal` 更新 odom，并注入 235 个局部语义节点 | `/scalenav/path`、frontier、完整 loss、local goal、到达事件 | odom 100 Hz、planner 10 Hz、semantic 2 Hz | 统一 loss 下 frontier 不长期停滞，允许短期偏离，最终收敛到 mission goal 并在超时前发布到达状态 | 20 次任务 | P0 | 测试设计已定义 |
| IT-ROS-007 | 闭环注入单行/多行风险及地面误检 | 路线 risk/loss、local goal、切换次数、任务状态 | odom 100 Hz、planner 10 Hz、semantic 2 Hz | local goal 连续；路线不因单帧、单行或地面误检左右振荡；最终完成任务 | 20 次任务 | P0 | 测试设计已定义 |
| IT-ROS-008 | 独立启动 Route-YOPO 控制，注入同步及超时的 odom/depth/path/frontier/clearance，并注入第二控制 publisher | status、route_condition、15 候选、planned path 和 `/scalenav/trajectory_point` | odom 100 Hz、depth/EPIC 10 Hz、模型 5 Hz、控制 50 Hz | 兼容输入标记 `compat_non_atomic`；三级降级正确；仅执行安全候选；无安全候选时位置保持；第二 publisher 出现时停止发布 | 每状态 100 tick、10 次启动 | P0 | 部分通过：14 项函数测试和 RTX 3090 合成 P95 通过；尚未执行真实 DDS 输入同步和闭环飞行 |

### 4.2 飞行场景集成

| 用例 ID | 场景输入 | 预期输出 | 输入频率 | 预期结果/判定 | 测试次数 | 重要性 | 测试状态 |
|---|---|---|---:|---|---:|---:|---|
| IT-FLT-001 | 空旷 140 m 往返任务 | 连续 frontier/local goal | depth 10 Hz、semantic 2 Hz、planner 10 Hz | 到达两个 mission goal，无 emergency stop，无路线倒退 | 3 次完整往返 | P0 | 已有测试 |
| IT-FLT-002 | 大墙逐步进入 40 m 窗口 | 安全空间下降、绕行或阻塞 | 同上 | 最迟 2 个点云/规划周期响应，不生成穿墙 witness | 10 次 | P0 | 测试设计已定义 |
| IT-FLT-003 | 稳态高风险走廊与安全旁路，叠加单帧及单行噪声 | 候选完整 loss、切换原因和稳定路线 | 同上 | 路线切换满足总 loss 改善和滞回；不受单帧或单行噪声误导；无左右振荡 | 10 次 | P0 | 测试设计已定义 |
| IT-FLT-004 | graph rebuild 时无人机越过 witness 首点 | 连续 accepted route/local goal | rebuild 10 Hz、planner 10 Hz | 无 discontinuous rejection，terminal id 可恢复 | 100 次 rebuild | P0 | 已测试失败（最新日志）：出现 8 次 `discontinuous witness` 拒绝，并有 stale terminal/无可达真实安全球告警；受控 100 次场景尚未执行 |
| IT-FLT-005 | 5x3 高风险 patch，measured depth 小于 30 m | 15 个固定 optical Z 语义投影 | semantic 2 Hz | 每有效帧 points>0，中心/边缘 optical Z 都为 30 m | 100 帧 | P0 | 已有测试，待场景复核 |
| IT-FLT-006 | 候选搜索连续超时 300 ms，旧 witness 安全 | incumbent 和短时 local goal | planner 10 Hz | hold 窗内继续安全路线；超过 400 ms 无安全输出则停止保底 | 30 次 | P0 | 测试设计已定义 |
| IT-FLT-007 | EPIC 输出接入 Route-YOPO 控制器，含窄门与转弯 | 飞行轨迹、候选安全状态和 50 Hz 控制状态 | 全链路额定频率 | RouteCondition 坐标系/层高正确；控制连续；无碰撞和 emergency stop；失效时位置保持 | 10 次路线 | P0 | 部分通过：入口、模型、安全门和控制函数已有自动化测试；10 次闭环路线待执行 |
| IT-FLT-008 | 长航线持续输入上/中/下分层语义目标及地面响应 | graph snapshot、row/FOV/地面/时间置信度、路线 loss | 全链路额定频率 | 三行空间信息保留；地面响应被抑制；不同 row 对路线 loss 的影响稳定、可重复，单行高风险仍有效 | 10 次 x 5 min | P0 | 测试设计已定义 |
| IT-FLT-009 | 复现未到终点会话的地图、语义密度和代价比例 | 完整飞行轨迹、frontier 序列、loss 分解、任务完成事件 | 全链路额定频率 | 不复现 frontier 停滞/循环并按时到达；每次切换由总 loss 改善或物理失效解释，不要求一定是前向延伸 | 10 次完整任务 | P0 | 测试设计已定义 |
| IT-FLT-010 | 同一进程执行 `(0,0) -> (0,140) -> (0,0)`；去程建立 graph，回程不清空 graph/semantic memory | goal graph 状态、background `inserted/remained`、route switch、update/rebuild 时延、完成时间、odom 轨迹长度、障碍密度热图及高风险语义距离 | depth/planner 10 Hz、semantic 2 Hz、odom 100 Hz | 回程必须复用 graph 且 `inserted/rebuild` 至少下降 30%；10 次往返中回程时间中位数至少降低 5%，轨迹和 update/rebuild P95 不得回退超过 5%，切换次数不增加；语义影响须以相同日志、语义权重开/关 A/B 区分于几何避障 | 10 次完整往返 | P1 | 部分通过（2 次日志实验）：graph 复用、几何新增量和切换次数通过；轮次 2 回程平均/P90 障碍密度下降 8.2%/28.6%，高风险语义 5 m 内暴露从 8.7% 降至 0%；但回程计算未加速、轨迹更长，且尚无语义权重 A/B 因果证据；见 [往返复用实验报告](test_reports/TEST_REPORT_2026-08-28_ROUND_TRIP_GRAPH_REUSE.md) |

## 5. 执行命令与通过标准

```bash
source /opt/ros/humble/setup.bash
cd /mnt/code/lab/yopo/OpenSeek/scalenav_ws
colcon build --packages-select scalenav_graph_ros2
colcon test --packages-select scalenav_graph_ros2
colcon test-result --verbose
```

通过标准：

1. 所有 `P0`、`P1` 测试用例完成规定判定，无崩溃、超时积压或数据竞争。
2. 数值比较默认绝对误差 `1e-5`；几何端点误差按具体用例规定。
3. P0 随机场景使用固定种子，表中次数全部通过，不接受概率性放行。
4. ASan/TSan 构建中不得出现 use-after-free、越界或 graph/semantic snapshot 数据竞争。
5. 真实仿真用例保留日志、参数快照和统计文件，便于复现。

## 6. 当前执行证据缺口

最近一次归档报告记录了 64 项 GTest、4 项在线节奏检查和 1 项按条件跳过的 rebuild 日志场景。当前源码已定义 67 项 GTest（其中 rebuild 日志场景仍可按条件跳过），新增测试项尚未形成新的归档执行证据。本规格中的测试设计条目仍需逐项保留输入、输出、频率、判定、次数和重要性。执行证据归档顺序为：

最新 ROS 会话的覆盖复核见 [`TEST_REPORT_2026-08-27_LATEST_LOG_REVIEW.md`](test_reports/TEST_REPORT_2026-08-27_LATEST_LOG_REVIEW.md)。该报告将函数级通过、部分覆盖、日志失败和仅有设计四种状态分开，不能用单元测试结果替代闭环或性能验收。

去程建图与回程效率的独立对照见 [`TEST_REPORT_2026-08-28_ROUND_TRIP_GRAPH_REUSE.md`](test_reports/TEST_REPORT_2026-08-28_ROUND_TRIP_GRAPH_REUSE.md)。当前 2 次完整往返证明 M2 graph 被复用，但尚未证明回程端到端计算和轨迹效率提高。

现有日志包测试 `scalenav_log/test_log_storage` 已通过，覆盖 session 不因容量滚动且旧 session 不被自动删除；该结果属于日志包测试证据，不新增本页 `TC/MT/IT` 编号。

1. `P0` 的 ROS 回调、时间同步、发布抑制和在线状态机直接测试。
2. Bubble 生成、拓扑插入/删除、A* 状态码的边界及故障注入测试。
3. 并发 graph snapshot、语义记忆和后台 rebuild 的 TSan 压力测试。
4. `P1/P2` 的索引换算、查询重载和诊断一致性测试。
