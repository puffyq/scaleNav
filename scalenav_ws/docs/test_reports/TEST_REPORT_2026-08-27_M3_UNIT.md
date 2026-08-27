# M3 语义风险单元测试报告

## 1. 测试范围

- 测试依据：[`FUNCTION_TEST_CASES.md`](../FUNCTION_TEST_CASES.md) 的 `TC-M3-001` 至
  `TC-M3-021`。
- 测试日期：2026-08-27。
- 代码范围：语义分数处理、固定深度投影、拓扑语义节点、持久记忆、路线风险及
  当前代语义工作集。
- 主要测试入口：`test_topo_semantic`。
- 在线日志：`/home/puffy/.ros/log/epic_graph_node_1556390_1787814009528.log`。
- 结构化快照：`log_scalenav/session_20260827_150009_473/graph/graph_1.json`。

本报告只判定 M3 函数级用例。在线日志用于补充回调、规划周期和持久化行为证据，不能
替代私有函数的受控边界输入。未完成用例规定的输入集合或次数时记为“部分通过”。

## 2. 执行结果

| 用例 | 结果 | 证据 | 判定摘要 |
|---|---|---|---|
| TC-M3-001 | 通过 | `TcM3001BaselineCoversQuantilesAndNonFiniteInput` | 100 轮覆盖空数组、仅非有限值、q=0/0.25/1 和输入钳位，结果符合分位基线定义 |
| TC-M3-002 | 通过 | `TcM3002CalibrationIsFiniteClampedAndMonotonic` | 1000 轮覆盖非有限值、上下界、基线扣除及风险单调性 |
| TC-M3-003 | 通过 | `TcM3003RiskAnchorUsesInclusiveThresholds` | 1000 轮验证分数和置信度均采用包含边界的双门槛，NaN/Inf 均拒绝 |
| TC-M3-004 | 通过 | `TcM3004ProjectionPreservesThreeVerticalRows` | 1500 个三行点保持 30 m optical x 增量且 z 有序，另以 500 轮验证边角距离大于中心 |
| TC-M3-005 | 部分通过 | 21 条在线 `[EPIC semantic]` 日志 | 每条均为 `image=160x96 patches=5x3 points=15 virtual_depth=30.00 m` 且姿态同步成功；mono8、错误编码、无姿态三类输入未受控执行 |
| TC-M3-006 | 通过 | `TcM3006SemanticInsertionReusesNearbyIdentity` | 100 轮验证新点插入、2.4 m 近点复用 id、分数及观测次数更新；无可执行连接时邻接为空 |
| TC-M3-007 | 通过 | `TcM3007SemanticEmaAndClamping` | 100 轮验证 alpha=0/0.5/1、分数与置信度钳位、观测次数及时间戳更新 |
| TC-M3-008 | 通过 | `TcM3008SemanticQueryFiltersRangeAndEvidence` | 1000 轮仅返回半径内且有观测证据的语义节点 |
| TC-M3-009 | 通过 | `TcM3009SnapshotIsDetachedFromMemory` | 100 轮修改快照副本不影响图内三条持久记录 |
| TC-M3-010 | 失败 | `TcM3010LoadKeepsNewestRecordPerIdentity` | 100 轮均由 `stamp=100` 的旧记录覆盖同 id、`stamp=200` 的新记录；条目数仍为 1，但中心、分数和时间戳错误 |
| TC-M3-011 | 通过 | `TcM3011SizeAlwaysMatchesSnapshot` | 空记忆及加载两条记录后，1000 轮 size 均等于 snapshot 条目数 |
| TC-M3-012 | 通过 | `TcM3012RestoreHonorsDistanceAndUnavailableIds` | 100 轮验证近点恢复、已占用 id 不恢复、远点不恢复 |
| TC-M3-013 | 测试设计已定义 | 私有 `mergeSemanticMemory` 无直接测试入口 | 实现含时间戳比较，但尚未以同 id 乱序记录执行 100 轮函数级测试 |
| TC-M3-014 | 测试设计已定义 | 无并发快照测试 | merge 与 snapshot 并发 10000 次的字段自洽和 id 唯一性尚未执行 |
| TC-M3-015 | 部分通过 | 现有语义代价测试、在线路线风险日志及代码复核 | 路径距离衰减、影响带、置信度乘积和过期代际过滤链路已执行；row、FOV、地面和时间输入矩阵未完整执行 |
| TC-M3-016 | 部分通过 | 在线 semantic route/update 日志 | 新鲜帧持续写入，重规划请求不清除既有 witness；尚未受控注入过期帧和重复时间戳各 100 次 |
| TC-M3-017 | 部分通过 | `TcM3017OverlappingFramesReusePersistentIdentity` | GTest 仅构造 1 个点并做 `0.001 m` 级扰动，1000 次均复用原 persistent id；未执行规格要求的 5x3、多帧移动、graph rebuild 及 global/local/A* 工作集场景 |
| TC-M3-018 | 部分通过 | `TcM3018FixedLayerDoesNotFlattenSemanticRows`；`graph_1.json` | GTest 1000 轮保持上、中、下行 z 有序且行间距离大于 1.5 m；真实快照 14 个语义点含 9 个高度，z 范围 `-4.4539..18.6401 m`；fixed-layer 开/关完整回调矩阵未执行 |
| TC-M3-019 | 测试设计已定义 | 逻辑内嵌于热力图回调 | 当前没有名为 `semanticObservationConfidence` 的独立函数，单行、一致、冲突、FOV 边缘和地面型输入矩阵尚未执行 |
| TC-M3-020 | 测试设计已定义 | 未形成独立聚合函数 | 单行/三行等价风险、地面行降权和冲突置信度矩阵尚未执行 |
| TC-M3-021 | 通过 | `TcM3021OnlyCurrentUnknownGenerationParticipates`；在线 update 日志 | 1000 轮验证当前 `Unknown` 和历史 `Verified` 参与、历史及过期 `Unknown` 退出；在线日志同时显示持久记录 800 条时 A* 仅使用 16 条，另有 378 条局部非活动虚拟节点 |

汇总：21 条用例中，11 条通过、1 条失败、5 条部分通过、4 条测试设计已定义。

## 3. 自动化执行记录

构建命令：

```bash
colcon build --packages-select scalenav_graph_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

Release 构建通过。

M3 相关测试：

```bash
./test_topo_semantic --gtest_color=no \
  --gtest_filter='TopoGraphM3Contract.*:TopoNodeModel.*:TopoSemanticMemory.*'
```

共执行 25 项：24 项通过、1 项失败。25 项由 14 项 M3 编号契约测试和 11 项既有节点/
记忆回归测试组成；唯一失败为 `TC-M3-010`。其中 `TC-M3-001` 至 `TC-M3-004`、
`TC-M3-006` 至 `TC-M3-012`、`TC-M3-017`、`TC-M3-018` 和 `TC-M3-021` 均具有编号化
断言，但 `TC-M3-018` 因未执行完整回调双模式，最终状态仍为“部分通过”。

完整目标：

```bash
./test_topo_semantic --gtest_color=no \
  --gtest_output=xml:/tmp/scalenav_m3_full.xml
```

共执行 72 项：67 项通过、5 项失败。除 `TC-M3-010` 外，其余失败均已由既有报告记录：
M2 的 `TC-M2-004`，以及 M4 的 `TC-M4-009`、`TC-M4-015`、`TC-M4-021`。本轮未发现
新的跨模块回归。

## 4. 在线证据复核

### 4.1 语义回调与三维位置

ROS 日志包含 21 条 `[EPIC semantic]` 记录，每条均接收 160x96 的 32FC1 热力图，并输出
5x3、15 个固定深度 30 m 的语义点，`pose_sync=0.0 ms`。该证据只覆盖合法编码及同步
姿态分支。

首个结构化 graph 快照的 `epic_semantic_points` 含 14 个点、9 个不同 z 值，最小值
`-4.45392513 m`，最大值 `18.6400871 m`。这证明该会话中的语义点没有全部写回
`graph_layer_z`；它不能替代 fixed-layer 开、关两种模式的受控回调测试。

### 4.2 持久记忆与规划工作集

在线 update 日志末段记录：

| 指标 | 数值 |
|---|---:|
| persistent semantic records | 800 |
| global semantic nodes | 798 |
| global Verified semantic nodes | 16 |
| global virtual semantic nodes | 782 |
| local semantic nodes | 16 |
| local inactive virtual semantic nodes | 377 |
| A* semantic nodes | 16 |
| A* inactive virtual semantic nodes | 378 |

持久记忆持续保留历史记录，而当前规划周期只把当前代 `Unknown` 和历史 `Verified` 纳入
工作集。该结果与 `TC-M3-021` 的函数级断言一致，也说明持久化数量不能直接当作每次 A*
实际参与数量。

## 5. 失败定位

### 5.1 TC-M3-010 旧记录覆盖新记录

测试按“新记录、旧记录”的顺序向 `TopoGraph::loadSemanticMemory()` 输入同一
`node_id=42`：

| 字段 | 新记录 | 旧记录 | 实际保留 |
|---|---:|---:|---:|
| stamp_ns | 200 | 100 | 100 |
| score | 0.9 | 0.2 | 0.2 |
| center.x | 2.0 | 1.0 | 1.0 |

`loadSemanticMemory()` 当前对每条合法记录直接执行
`semantic_memory_[record.node_id] = record`，没有比较已有记录时间戳。因此条目数没有
膨胀，但内容会随输入顺序回退。相邻的 `EpicGraphNode::mergeSemanticMemory()` 已采用
`record.stamp_ns >= existing.stamp_ns` 判定，两处加载语义不一致。

本轮没有修改生产实现。

## 6. 后续测试项

- TC-M3-005：为热力图回调补 mono8、错误编码、无姿态及 32FC1 四组受控输入。
- TC-M3-013、TC-M3-014：为 Epic 私有持久记忆接口建立测试 peer，分别执行乱序覆盖和
  并发快照测试。
- TC-M3-015、TC-M3-019、TC-M3-020：把 row、FOV、地面可能性和时间置信度拆成可直接
  断言的函数接口，再执行规格中的输入矩阵。
- TC-M3-016：受控注入新鲜、过期、重复时间帧并检查 replan latch 与 incumbent 连续性。
- TC-M3-018：通过完整回调分别运行 fixed-layer 开、关模式，并逐项关联三行分数、位置和
  persistent id。
