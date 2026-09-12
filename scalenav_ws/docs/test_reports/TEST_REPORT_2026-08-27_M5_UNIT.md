# M5 路线记忆与目标选择单元测试报告

## 1. 测试范围

- 测试依据：[`FUNCTION_TEST_CASES.md`](../FUNCTION_TEST_CASES.md) 的 `TC-M5-001` 至
  `TC-M5-021`。
- 测试日期：2026-08-27。
- 代码范围：路线距离、前向裁剪、切换滞回、terminal 复用、语义重规划、路线恢复和
  local goal 选择。
- 主要测试入口：`test_route_memory`。
- 在线日志：`/home/puffy/.ros/log/epic_graph_node_1556390_1787814009528.log`，并以历史
  `epic_graph_node_3919091_1787583196533.log` 补充 odom 连接成功分支。

本报告只判定 M5 函数级用例。`RouteMemoryM5Contract` 直接调用公开纯函数；
`EpicGraphNode` 私有函数和 `update()` 状态机使用在线日志补充行为证据。日志不能替代受控
边界输入，未完成规格规定的场景集合或次数时记为“部分通过”。

## 2. 执行结果

| 用例 | 结果 | 证据 | 判定摘要 |
|---|---|---|---|
| TC-M5-001 | 通过 | `TcM5001PointSegmentDistanceCoversProjectionAndDegenerateSegment` | 每场景 10000 轮覆盖线内投影、首尾端点外及退化线段，结果分别为垂距、端点距和点距 |
| TC-M5-002 | 通过 | `TcM5002PointPathDistanceCoversEmptySingleAndPolyline` | 每场景 10000 轮验证空路径 infinity、单点距离和多段最小距离 |
| TC-M5-003 | 通过 | `TcM5003ForwardRouteWindowStartsAtProjectionAndHonorsHorizon` | 1000 轮均从无人机投影点开始、删除身后点并在 10 m 弧长处截断 |
| TC-M5-004 | 通过 | `TcM5004ForwardRouteFromPositionKeepsCompleteSuffix` | 1000 轮均保留投影点到原 terminal 的完整前向后缀 |
| TC-M5-005 | 通过 | `TcM5005ContinuousRouteCoversBoundaryOffsetAndPassedTerminal` | 1000 轮覆盖线上、0.5 m 边界、0.51 m 外及越过 terminal，返回值符合契约 |
| TC-M5-006 | 通过 | `TcM5006SwitchUsesAggregateLossHysteresisAndHardFailure` | 1000 轮验证硬失效强制切换、风险门槛、聚合 objective 比例、进度门槛和滞回边界 |
| TC-M5-007 | 通过 | `TcM5007EdgeFollowsRouteChecksEndpointsAndMidpoint` | 每场景 1000 轮验证重合边、端点近但中点偏离的 U 形跨接边及平行远边 |
| TC-M5-008 | 通过 | `TcM5008RouteLengthSkipsNonFiniteSegments` | 1000 轮验证空、直线、多段及含 NaN 段路线，只累计有限段 |
| TC-M5-009 | 通过 | `TcM5009OnlyCompatibleLongerCandidateIsAnExtension` | 每场景 1000 轮仅接受同走廊且长度增益达标候选，拒绝近端换道及增益不足候选 |
| TC-M5-010 | 通过 | `TcM5010TerminalReuseUsesStrictReleaseBoundary` | 每场景 1000 轮验证仅距离严格大于释放距离时复用 terminal |
| TC-M5-011 | 通过 | `TcM5011ForwardRouteReuseNeedsAlignmentAndRemainingDistance` | 每场景 1000 轮验证路线中段复用，离线、越过 terminal 和剩余不足均拒绝 |
| TC-M5-012 | 通过 | `TcM5012SemanticRiskIncreaseUsesInclusiveFiniteThreshold` | 每场景 1000 轮验证小于/等于/大于阈值、下降和 NaN，阈值采用包含边界 |
| TC-M5-013 | 通过 | `TcM5013SemanticRiskChangeMatchesIncreaseContract` | 六组输入各 1000 轮，change 与 increase 返回值逐项一致 |
| TC-M5-014 | 通过 | `TcM5014SemanticResetRequiresEnableAndRiskTrigger` | 每场景 1000 轮验证开关、微小/边界/显著上升和 NaN |
| TC-M5-015 | 测试设计已定义 | 私有 `nearestPersistentNode` 无直接测试入口 | id 优先、id 丢失近点后备和远点 null 三场景尚未函数级执行 |
| TC-M5-016 | 部分通过 | 在线 `[EPIC odom diagnosis]` | 历史日志含 `connected=2` 的成功连接，也覆盖无路、起点失败和 timeout；已连接早退及墙隔断三场景未各执行 100 次 |
| TC-M5-017 | 部分通过 | TC-M5-003、TC-M5-007 及在线 `remembered_edges` | 前向路线裁剪和边归属核心函数通过；在线周期持续生成 remembered edge，但私有函数未直接执行 1000 次 |
| TC-M5-018 | 部分通过 | `TcM5018RouteLookaheadFollowsPolylineOrder`；在线 `[EPIC goals]` | 1000 轮验证 local goal 沿折线顺序前视而不跳段；在线目标通常位于约 10 m 前方，近 mission goal 私有分支未受控执行 1000 次 |
| TC-M5-019 | 部分通过 | 最新在线 update 日志 | `route_lateral_error=5.81 m` 时仍为 `incumbent=RECOVERED`、`switch_reason=NONE`，未因横向偏离清空 terminal；小于/大于阈值双场景未完成 1000 tick |
| TC-M5-020 | 部分通过 | 最新在线 route switch/update 日志 | 记录 9 次 `FRONTIER_HALF` 且均 `compatible_extension=1`、1 次 `LOWER_LOSS`，另有 5 次候选找到但未接受；精确 50% 边界及全部候选组合未完成规定次数 |
| TC-M5-021 | 部分通过 | 最新在线 update 日志 | mission goal 距离 30.28 m 时 incumbent 保持且候选被拒；terminal 到 goal 为 0 m 时连续三个周期仍保持同一 RECOVERED terminal；blocked/过半组合未完成 1000 tick |

汇总：21 条用例中，14 条通过、6 条部分通过、1 条测试设计已定义；本轮没有 M5 失败。

## 3. 自动化执行记录

构建命令：

```bash
colcon build --packages-select scalenav_graph_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

Release 构建通过。PCL 的 pcap/png 可选功能警告不影响本测试目标。

编号化 M5 测试：

```bash
./test_route_memory --gtest_color=no \
  --gtest_filter='RouteMemoryM5Contract.*'
```

共执行 15 项，15 项全部通过。前 14 项对应 `TC-M5-001` 至 `TC-M5-014`，第 15 项验证
`TC-M5-018` 的折线顺序前视核心。

完整路线记忆目标：

```bash
./test_route_memory --gtest_color=no \
  --gtest_output=xml:/tmp/scalenav_m5_route_memory.xml
```

共执行 27 项，27 项全部通过：15 项编号化 M5 测试和 12 项已有路线记忆回归测试。

CTest 注册入口：

```bash
ctest -R '^test_route_memory$' --output-on-failure
```

CTest 结果为 1 个测试目标通过；该目标内部实际包含上述 27 项 GTest。

本包全量回归：

```bash
ctest --output-on-failure
```

6 个测试目标中 4 个通过，`test_lidar_map` 和 `test_topo_semantic` 失败。失败明细为 M1 的
`TC-M1-002`、`TC-M1-004`、`TC-M1-009`，M2 的 `TC-M2-004`，M3 的 `TC-M3-010`，以及
M4 的 `TC-M4-009`、`TC-M4-015`、`TC-M4-021`，均已在对应模块报告中记录。本轮没有
新增 M5 失败或其他跨模块失败。

## 4. 在线状态机证据

### 4.1 incumbent 与候选滞回

最新完整日志包含：

| 事件 | 次数 |
|---|---:|
| `incumbent=RECOVERED` | 21 |
| `FRONTIER_HALF` 路线延伸 | 9 |
| `LOWER_LOSS` 路线切换 | 1 |
| candidate found 但未接受 | 5 |

9 次 `FRONTIER_HALF` 均记录 `compatible_extension=1`。`LOWER_LOSS` 事件记录
`compatible_extension=0`，说明非兼容分叉没有借半程刷新直接提交，而是走完整 loss 比较。
多次 `route_compare=1` 时 candidate loss 高于 incumbent loss，候选保持未接受。

### 4.2 横向偏离不构成硬切换

日志在 `route_lateral_error=3.36 m`、`5.81 m`、`3.03 m` 和 `4.05 m` 时仍记录
`incumbent=RECOVERED`、`switch_reason=NONE`。其中 `5.81 m` 样本保持原 terminal 685，
没有触发 `NO_ACCEPTED_ROUTE`。这支持 TC-M5-019 的现行设计，但样本不是阈值两侧各
1000 tick 的受控测试。

### 4.3 进入 mission goal 窗口

mission goal 距离 30.28 m 时，日志记录 `incumbent=RECOVERED`，候选虽然找到但因
`incumbent_loss=42.06 < candidate_loss=49.44` 未切换。terminal 已到 mission goal、
`terminal_goal_distance=0.00 m` 后又连续三个周期保持同一 persistent terminal 1043，
`switch_reason=NONE`。goal-window 本身没有清除 incumbent 或成为硬切换命令。

## 5. 未完成的函数级测试

- TC-M5-015：需要为 `nearestPersistentNode` 建立测试 peer，直接验证 persistent id 优先级、
  近点后备距离和远点 null。
- TC-M5-016：需要可控开放/墙阻断地图，分别执行已连接、成功新增 witness 和连接失败。
- TC-M5-017：需要构造包含身后边、跨接边和前向边的 TopoGraph，直接读取 remembered
  edge 集合。
- TC-M5-018：需要驱动 `selectNextGoal` 的 near-goal 分支，并检查 fixed-layer 非平面路径
  的拒绝行为。
- TC-M5-019 至 TC-M5-021：需要可控 `update()` 状态夹具，覆盖横向阈值、精确 50% 进度、
  blocked、incumbent 丢失及 goal-window 组合。

本轮没有修改生产实现。
