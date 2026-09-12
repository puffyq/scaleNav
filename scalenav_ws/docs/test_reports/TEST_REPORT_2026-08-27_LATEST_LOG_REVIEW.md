# 最新日志复核：测试覆盖与实际故障的差距

## 1. 复核对象

- ROS 日志：`/home/puffy/.ros/log/epic_graph_node_2455540_1787829341547.log`
- 会话目录：`log_scalenav/session_20260827_191541_638/`
- 对照规格：[`FUNCTION_TEST_CASES.md`](../FUNCTION_TEST_CASES.md)
- 对照执行报告：M3、M4、M5 单元测试报告
- 本报告只修正测试证据和覆盖状态，不修改生产实现。

## 2. 日志事实

| 指标 | 日志观测 | 对验收的含义 |
|---|---:|---|
| planner update | 43 次 | 具备一个短时在线闭环样本，但不是规定次数的受控集成测试 |
| route switch | 89 次 | 路线选择并非稳定的单次决策过程 |
| `reason=BLOCKED` | 48 次 | 需要检查阻塞恢复、同 terminal 重选和 local goal 连续性 |
| `discontinuous witness` 拒绝 | 8 次 | `IT-FLT-004` 的“无拒绝”判定没有满足 |
| `candidate_accepted=1` | 5 次 | 仅有少量候选实际提交，不能证明候选集合和排序已被充分覆盖 |
| persistent semantic records | 14 -> 1910 | 语义记录持续增长，现场问题不是单帧 patch 数量，而是跨帧持久化合并失效或输入未复用 |
| global virtual semantic nodes | 14 -> 1813 | 虚拟语义节点进入全局图，已影响活动拓扑和 A* 工作集 |
| 单次 update 最大耗时 | 198.386 ms | 超过 10 Hz 的 100 ms 周期预算；性能验收不能标记为通过 |

日志还出现了 `stale terminal behind vehicle` 和 `no reachable real Bubble topology`。最终出现
`mission complete` 不抵消中途的路线切换、阻塞和 witness 拒绝；任务完成事件只能作为结果字段，
不能替代过程约束。

## 3. 为什么已有测试没有测出来

### 3.1 M3 语义跨帧合并的输入弱于规格

规格中的 `TC-M3-017` 要求连续两帧、每帧 5x3 共 15 个 patch、无人机移动 `2.65 m`、30 m
投影足迹重叠，并观察 persistent/global/local/A* 工作集。实际 GTest
`test_topo_semantic.cpp` 中的同名测试只：

1. 首先插入 1 个点；
2. 循环 1000 次，每次仍只插入 1 个点；
3. 位置只作 `0.001 m` 级扰动，虽然传入了一个 odom 位移参数，但没有生成 5x3 投影、点云重建、后台 worker 或 planner update；
4. 最终只断言 `semanticNodes().size()==1`、同一 persistent id 和观测次数。

这证明的是单对象近邻复用，不是现场的 15 点跨帧合并。现场每帧新写入的候选、无人机运动
造成的投影变化、重建恢复和全局记忆增长没有进入该夹具，因此单元测试通过并不矛盾于日志中的
`14 -> 1910`。

### 3.2 M4/M5 没有把候选选择串成在线状态机

`TC-M4-012`、`TC-M4-014`、`TC-M4-020` 和 `TC-M5-006` 主要验证纯函数或小图夹具中的
loss、排序和滞回。它们没有连续驱动下面的状态链：

```text
candidate search
    -> incumbent recovery
    -> FRONTIER_HALF / BLOCKED
    -> terminal extension
    -> 无人机实际位置更新
    -> 下一次 planner update
```

因此没有对 89 次切换、48 次 BLOCKED、同一 terminal 反复切换、stale terminal、witness 点数
增长和不连续 witness 拒绝设置过程断言。`TC-M4-021` 已在单元测试中暴露“无 witness 节点仍可能
被接受”，但对应闭环场景尚未执行，故该类问题仍能出现在在线日志中。

### 3.3 M6 性能测试没有测真实 update

已有 `TC-M6-004`/`MT-M6-002` 证据主要是合成 worker 时长、单 worker 约束或已有日志节奏检查。
它们没有把真实点云规模、持久语义记录、region select、skeleton、锁等待、A* 和 publish 串成
10 Hz 端到端统计，也没有把 `total` 的最大值作为失败条件。最新日志的 198.386 ms 因而没有被
现有性能用例捕获。

## 4. 覆盖状态修正

| 用例 | 原状态 | 修正状态 | 修正原因 |
|---|---|---|---|
| `TC-M3-017` | 已测试通过 | 部分通过 | GTest 是单点微扰复用；5x3、多帧、移动、重建和工作集未执行 |
| `IT-FLT-004` | 已有测试 | 已测试失败（日志） | 日志出现 8 次 `discontinuous witness`，并出现 stale terminal/无可达真实 Bubble 拓扑 |
| `MT-M6-002` | 已有测试 | 部分通过 | worker/合成节奏检查有证据，但真实 update 峰值 198.386 ms，未满足 100 ms 周期预算 |
| `IT-ROS-006` | 测试设计已定义 | 测试设计已定义 | 没有闭环注入 235 个局部语义节点并验证最终 mission goal 的受控报告 |
| `IT-ROS-007` | 测试设计已定义 | 测试设计已定义 | 没有单行/多行/地面误检的连续闭环证据 |
| `IT-FLT-009` | 测试设计已定义 | 测试设计已定义 | 最新日志可作为复现输入，但尚未完成 10 次完整任务及切换原因审计 |

状态含义固定为：

- **已测试通过**：实际输入、频率、次数和判定均有可追溯执行证据；
- **部分通过**：只覆盖了函数子集、弱化输入或部分在线指标；不能外推为场景通过；
- **已测试失败**：已有受控测试或真实日志直接违反判定；
- **测试设计已定义**：只有规格，没有本地执行证据。

## 5. 必须补做的测试

1. `TC-M3-017` 的真实夹具：每帧生成 15 个投影点，按 2 Hz 注入 60 s，同步 100 Hz odom，移动 `2.65 m`，并在 graph rebuild 期间采集 persistent、global、local 和 A* 工作集曲线；判定记录不得逐帧增加 15 个独立持久记录。
2. `IT-ROS-006`/`IT-FLT-009` 闭环复现：记录每次 update 的 incumbent/candidate loss、terminal id、route switch reason、witness 点数、local goal 进度和 mission goal 距离；任何无解释切换、重复 terminal 振荡或 no-progress 都应失败。
3. `IT-FLT-004`：在 rebuild 与 odom 越过 witness 首点的同时，要求 0 次不连续 witness 拒绝；stale terminal 和无可达真实安全球必须有明确失败计数。
4. `MT-M6-002`：使用最新点云和语义规模，统计 update P50/P95/P99/max、rebuild backlog、锁等待和 graph age；max 超过 100 ms 时不能标记通过。

## 6. 结论

不是“测试通过但现场偶发”这么简单，而是测试规格、实际夹具和在线验收口径不一致：规格描述了
多帧、多模块、闭环和性能约束，执行时却主要运行了单点函数测试或合成节奏检查。最新日志已经
给出明确的场景级失败证据；在补做上述闭环和真实规模性能测试前，只能保留函数级通过，不能宣称
M3/M4/M5/M6 的在线行为已通过。
