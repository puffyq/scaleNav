# M4 搜索与统一代价单元测试报告

## 1. 测试范围

- 测试依据：[`FUNCTION_TEST_CASES.md`](../FUNCTION_TEST_CASES.md) 的 `TC-M4-001` 至
  `TC-M4-022`。
- 测试日期：2026-08-27。
- 代码范围：`ParallelBubbleAstar`、`TopoGraph` 搜索、边代价及 frontier terminal 选择。
- 主要测试入口：`test_topo_semantic`。
- 辅助测试入口：`test_epic_integration`。
- 历史证据：[`CHANGELOG.md`](../CHANGELOG.md) 的 CHG-0011、CHG-0017 至 CHG-0026，
  以及 [`TEST_REPORT_2026-08-26.md`](TEST_REPORT_2026-08-26.md)。

本轮只判定 M4 函数级用例。M5 路线切换状态机、ROS2 闭环和真实飞行不计入 M4 单元
测试。没有达到规格规定的场景集合或次数时，即使相关自动化检查通过，也只记为“部分
通过”或“已有测试，待场景复核”。

## 2. 执行结果

| 用例 | 结果 | 证据 | 判定摘要 |
|---|---|---|---|
| TC-M4-001 | 部分通过 | Release 构建及现有搜索夹具 | 初始化链路可用；未按 10 组参数逐项核对分辨率倒数、平面图层和 LIO 指针 |
| TC-M4-002 | 通过 | `TcM4002ResetClearsSearchCaches` | 1000 次 reset 后 safe/danger 缓存均为空，前次搜索状态不泄漏 |
| TC-M4-003 | 通过 | `TcM4003And004GridIndexRoundTrip` | 10000 个正负坐标的 floor 索引均与定义一致 |
| TC-M4-004 | 通过 | `TcM4003And004GridIndexRoundTrip` | 10000 个索引中心正确，点到中心的逐轴误差不超过半栅格 |
| TC-M4-005 | 部分通过 | `OpenLongBubbleEdgeIsNotRejectedByAnArbitraryTwoMeterCap` | 开放长边的安全空间查询和碰撞复核通过；层下地面及空地图查询未独立注入 |
| TC-M4-006 | 测试设计已定义 | 无完整边界夹具 | 尚未完成安全、安全空间不足、AABB 外节点及缓存一致性矩阵 |
| TC-M4-007 | 已有测试，待场景复核 | 现有 A*、重建及 Changelog 包级测试 | 成功和失败分支曾执行；本轮未对四种状态码各执行 100 次受控输入 |
| TC-M4-008 | 部分通过 | `OpenLongBubbleEdgeIsNotRejectedByAnArbitraryTwoMeterCap` | 开放 6 m 折线检查通过；绕障保留折点与碰撞 false 未完成完整矩阵 |
| TC-M4-009 | 失败 | `TcM4009PathCostHandlesPolylineAndEmptyInput` | 1000 次多段折线均得到 17 m；空输入因无符号下溢进入循环并收到 SIGSEGV |
| TC-M4-010 | 通过 | `RiskUsesTheExecutedWitnessPolyline` | 1000 次均证明风险按实际弯折 witness 计算，明显高于远离风险点的边 |
| TC-M4-011 | 通过 | `TcM4011ClearanceCostUsesContinuousFormula` | 1000 次核对零安全空间、目标值、目标值以上及大安全空间；数值和单调性均符合现行公式 |
| TC-M4-012 | 通过 | `TcM4012RouteEdgeCostCombinesAndDiscountsTerms` | 1000 次核对几何、语义、安全空间和 incumbent 折扣；基础值 18、折扣值 12，语义风险使总 loss 增加 |
| TC-M4-013 | 部分通过 | `GraphSearchRejectsEndOutsideLocalWindow` 及现有连通搜索 | 窗外终点被拒绝且 path 为空；断图和极短超时未完成每场景 100 次 |
| TC-M4-014 | 部分通过 | `AstarTurnsAwayFromNearbySemanticRisk`、`GoalDirectedSearchPrefersSaferTerminalOverFartherRisk` 等 | 多条真实安全球候选按组合 loss 排序；完整双走廊参数矩阵未达规定次数 |
| TC-M4-015 | 失败 | `TcM4015EmptyAndSingleNodePathsAreZero` | 空路径和单节点路径均因 `size()-1` 无符号下溢收到 SIGSEGV，不满足返回 0 的契约 |
| TC-M4-016 | 已有测试，待场景复核 | CHG-0019 及 2026-08-26 报告 | 代码和历史回归已删除失败后的直线 fallback；本轮未直接驱动私有函数完成三场景各 100 次 |
| TC-M4-017 | 部分通过 | `RadialFrontierAllowsWideSemanticDetour`、`GoalDirectedSearchPrefersSaferTerminalOverFartherRisk` | 低于 31.5 m 的安全候选可以胜出，径向/任务方向没有作为硬资格；未执行 1000 个日志规模图种子 |
| TC-M4-018 | 部分通过 | `TcM4018EquivalentRiskDensityKeepsTerminalRanking` | 同一风险场由 1 个和 235 个重合语义点表达时，1000 次均选择同一 terminal；尚未覆盖 1000 个不同图种子 |
| TC-M4-019 | 部分通过 | `TcM4012RouteEdgeCostCombinesAndDiscountsTerms`、`GeometryWeightBalancesSemanticFrontierRisk` | 几何、语义、安全空间和方向权重变化符合单调性；FOV 和 smoothness 尚未逐项穷举 |
| TC-M4-020 | 部分通过 | `GeometryWeightBalancesSemanticFrontierRisk`、`GoalDirectedSearchPrefersSaferTerminalOverFartherRisk` | 较近安全候选可胜出，交换几何/语义权重后选择可预测变化；未达到每组 1000 次 |
| TC-M4-021 | 失败 | `TcM4021RejectsConnectedNodeWithoutWitness` | 只有双向邻接和有限权重、完全没有 witness 的 `Verified` 节点仍被搜索接受并输出为 terminal |
| TC-M4-022 | 通过 | `TcM4022OnlyVerifiedBubbleCanBeTerminal` | 1000 轮均只选择可达 `Verified` 安全球；仅剩 `Unknown` 虚拟语义点时返回 false 且 path 为空 |

汇总：22 条用例中，7 条通过、3 条失败、9 条部分通过、2 条已有测试待场景复核、
1 条测试设计已定义。

## 3. 自动化执行记录

构建命令：

```bash
colcon build --packages-select scalenav_graph_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

构建通过。

M4 主要测试：

```bash
./test_topo_semantic --gtest_color=no \
  --gtest_filter='TopoGraphM4Contract.*:TopoSemanticCost.*:TopoSearchRadius.*'
```

共执行 30 项：27 项通过、3 项失败。失败目标分别对应 TC-M4-009、TC-M4-015 和
TC-M4-021；死亡测试将越界崩溃隔离在子进程内，因此其余断言继续执行。

`test_topo_semantic` 全目标共执行 58 项，54 项通过、4 项失败。除上述 3 个 M4 失败外，
另 1 项为 M2 报告已经记录的 `TC-M2-004` 越界索引失败；没有新增其他模块回归。

辅助测试：

```bash
./test_epic_integration --gtest_color=no \
  --gtest_filter='EpicIntegration.SemanticUpdateChangesTheNextPlannerDecision:EpicIntegration.OpenLongBubbleEdgeIsNotRejectedByAnArbitraryTwoMeterCap'
```

2 项均通过。第一项证明语义更新会在下一规划周期改变中间走廊，同时 terminal 保持为
外侧真实安全球；第二项证明开放 6 m 边不会被任意 2 m 上限误拒绝。

## 4. 失败定位

### 4.1 TC-M4-009 空路径代价越界

`ParallelBubbleAstar::calculatePathCost()` 使用 `int i` 与 `path.size() - 1` 比较。空 vector
的 `size()` 为无符号零，减一后变成极大值，循环错误进入并读取 `path[0]`、`path[1]`，
最终收到 SIGSEGV。非空多段折线的数值计算正确。

### 4.2 TC-M4-015 空及单节点拓扑路径越界

`TopoGraph::getPathLength()` 有相同的 `topo_path.size() - 1` 无符号下溢。空路径直接访问
不存在的拓扑节点；单节点路径在展开完成后，又对空的几何 path 使用同样的循环条件，
两者都崩溃，未返回规格要求的 0。

### 4.3 TC-M4-021 无 witness 的边仍可执行

`goalDirectedSearch()` 展开邻居时只要求 `neighbors_` 中存在节点且 `weight_` 有有限值，
没有要求 `paths_` 中存在至少两个有限点组成的 witness。候选筛选又只检查节点为
`Verified`，因此测试构造的无 witness terminal 被返回。该结果违反“无法形成有效
witness/local goal 必须拒绝”的物理约束。

## 5. 旧测试口径调整

CHG-0026 已回退“语义点直接充当 frontier terminal”。本轮将两条旧断言改为现行口径：

- `UnknownSemanticNodeCannotBecomeFrontierTerminal`：`Unknown` 虚拟语义点不能成为 terminal。
- `VerifiedLowRiskBubbleRemainsAValidSafeBranch`：真实 `Verified` 安全球仍可参与候选排序。

另有三条旧测试仍按两节点 path 断言。当前搜索返回完整的
`start -> branch -> outer terminal`，因此断言改为检查中间走廊和最终真实安全球，不改变
算法行为。

本轮没有修改生产实现。
