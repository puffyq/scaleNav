# ScaleNav / EPIC Changelog

本文件按修改批次记录，不按日期聚合。每次代码更新新增一个独立变更编号；后续补充验证结果时更新对应记录，不把不同修改合并到同一天的章节中。

<a id="chg-0034"></a>
## CHG-0034 Route-YOPO 固定高度执行合同

- Route 合同新增固定高度校验：path 高度跨度超过 `0.05 m` 时不进入 ROUTE 模式。
- ROUTE 模式把全部 YOPO 终点投影到 Route 中位高度并令终端 `vz=az=0`，投影后再执行
  Poly5 重建和深度安全认证；FRONTIER_ONLY 仍保留三维 YOPO 行为。
- 整段轨迹必须位于 Route 高度 `+/-0.25 m`，否则标记 `ROUTE_ALTITUDE`；没有同时通过
  高度带与深度门的候选时保持 SAFETY_HOLD。
- `log_scalenav/session_20260828_190205_247` 回放中，`+3/+10/+15 s` 分别保留
  `13/15/15` 条认证候选，固定终点为 `1.5995 m`，整段最大误差低于 `4 mm`；`+1 s`
  深度无安全解，保持 HOLD。
- Route-YOPO 控制定向测试 `20 passed`。

<a id="chg-0033"></a>
## CHG-0033 Route-Conditioned YOPO 独立控制入口

- 记录时间：2026-08-28
- 状态：独立入口、checkpoint smoke、14 项定向测试和 RTX 3090 合成基准完成；真实 DDS 输入、真实场景 P95 与闭环飞行待执行
- 测试记录：[TEST_REPORT_2026-08-28_ROUTE_YOPO_CONTROL.md](test_reports/TEST_REPORT_2026-08-28_ROUTE_YOPO_CONTROL.md)

修改内容：

- 新增 `scripts/start_route_yopo.sh`，默认启动完整控制链，`--attach` 接入已有 EPIC 会话；
  未修改、调用或替换 `scripts/start.sh`，且完整模式不启动旧 planner。
- 新增 Route-YOPO ROS2 控制节点，加载
  `train_scalenav/saved_corrected/YOPO_5/best.pth`，输出 planned path、15 候选、状态、
  RouteCondition 诊断和 50 Hz `/scalenav/trajectory_point` 控制命令。
- 在 EPIC 原子 route 消息完成前，聚合 `/epic/path`、`/epic/graph` 和
  `/epic/clearance`，要求 source stamp 相近并明确标记 `epic_compat_non_atomic`；本地
  route id 不冒充 EPIC source id。
- 15 个 primitive 按 score 排序后逐条重建 101 点三维 Poly5，并执行当前 DepthPlanar
  扫掠球安全检查；score 最优项不安全时尝试下一项，全部无法认证时进入 SAFETY_HOLD。
- 安全门使用每块最小深度把高分辨率输入保守缩减到固定网格；块内近障碍保留，任一未知
  射线使对应块保持未知。101 点间以半个最大采样间距扩张安全球，连续覆盖 Poly5 且不重复
  检查每段两端。
- 状态实现 ROUTE、FRONTIER_ONLY、SAFETY_HOLD；路径/目标过期、非有限、stamp 不一致、
  安全空间不足和无安全 primitive 均保留 reason code。
- 通过安全门的 Poly5 保存 101 个位置、速度和加速度状态供 50 Hz 插值执行；无安全轨迹、
  深度/里程计超时或轨迹过期时发布当前位置零速度保持。检测到第二控制 publisher 时停止
  发布并清除旧轨迹，冲突解除后必须等待新轨迹重新认证。

验证结果：

- checkpoint feature order、12 个 route anchors、15 primitive 输出 shape 和有限性通过。
- `test_route_yopo_control.py` 为 `14 passed`，覆盖 Poly5 导数、正常控制、安全保持和双
  publisher 冲突；Bash 语法、Python 编译和 diff whitespace 检查通过。
- `start_route_yopo.sh --attach --device cuda` 实际启动后，ROS 图确认
  `/scalenav/trajectory_point` 为 `MultiDOFJointTrajectoryPoint`，publisher count 为 1，
  唯一 publisher 是 `scalenav_route_yopo_controller`。
- RTX 3090 上纯模型 1000 tick 的 P50/P95/max 为 `1.743/2.935/4.785 ms`；包含前处理、
  GPU 传输、Poly5 和 15 候选安全门的合成自由深度 100 tick 为
  `36.139/64.460/68.883 ms`，峰值显存 `192.03 MiB`。

<a id="doc-test-2026-08-28-round-trip-graph-reuse"></a>
## DOC-TEST-2026-08-28 去程建图与回程效率实验

- 记录时间：2026-08-28
- 变更类型：飞行日志对照、障碍物密度分析和测试规格补充；无生产算法修改
- 测试记录：[TEST_REPORT_2026-08-28_ROUND_TRIP_GRAPH_REUSE.md](test_reports/TEST_REPORT_2026-08-28_ROUND_TRIP_GRAPH_REUSE.md)

新增 `IT-FLT-010`，把“能完成往返”和“去程 graph 是否提高回程效率”拆成独立验收。复核两次最新完整往返：回程均复用已有 graph，平均 `inserted/rebuild` 下降约 62%，路线切换下降 33%；但 background rebuild mean 增加 17.6%--21.5%，两轮回程轨迹分别增长 1.3% 和 4.6%，尚不能判定整体效率提高。

轮次 2 新增飞行高度障碍物密度热图。回程平均/P90 障碍密度比去程下降 8.2%/28.6%，到最终高风险语义节点的最小距离从 2.56 m 增至 6.93 m，进入其 5 m 范围的轨迹比例从 8.7% 降至 0%。该结果证明空间相关性，不作为语义因果证据；因果验收仍需相同日志下语义权重开启/关闭的确定性 A/B 回放。

<a id="doc-test-2026-08-27-latest-log-review"></a>
## DOC-TEST-2026-08-27 最新日志暴露测试覆盖高估

- 记录时间：2026-08-27
- 变更类型：测试证据复核与覆盖状态修正；无生产算法修改
- 详细报告：[TEST_REPORT_2026-08-27_LATEST_LOG_REVIEW.md](test_reports/TEST_REPORT_2026-08-27_LATEST_LOG_REVIEW.md)

复核 `/home/puffy/.ros/log/epic_graph_node_2455540_1787829341547.log` 后确认：43 次 planner update 中有 89 次路线切换、48 次 `BLOCKED` 切换和 8 次 `discontinuous witness` 拒绝；persistent semantic records 从 14 增长到 1910，单次 update 最大耗时 198.386 ms。最终 `mission complete` 不抵消过程中的路线连续性和性能违规。

覆盖口径修正：

- `TC-M3-017` 的 GTest 实际是单点微扰复用，不是规格中的 5x3 多帧移动场景，状态改为“部分通过”。
- `IT-FLT-004` 被最新日志中的 8 次不连续 witness 拒绝直接判为“已测试失败（日志）”。
- `MT-M6-002` 的 worker/合成节奏检查保留，但真实端到端 update 超出 100 ms，状态改为“部分通过”。
- `IT-ROS-006`、`IT-ROS-007` 和 `IT-FLT-009` 仍只有测试设计，不能用单元测试或最终到达事件替代闭环证据。

本条目只记录文档和验收状态调整，不代表生产代码已修复上述在线问题。

<a id="chg-0032"></a>
## CHG-0032 frontier 末端按 EPIC region 方向判定

- 记录时间：2026-08-27
- 状态：代码、Release 编译和定向自动化回归完成；真实仿真待复测
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0032](test_reports/TEST_REPORT_2026-08-26.md#chg-0032)

问题与根因：

- 首帧图中正前方节点 `(1.15,17.65)` 存在，但旧过滤只要发现任意 route-distance 更大的邻居就排除它。
- 其右前方邻居 `(4.45,20.95)` 被错误视为同一条走廊的延续，导致正前方节点在平滑性、方向和安全 loss 计算前就失去候选资格。
- 该过滤使用拓扑距离极大值近似 frontier，不能把不同方向分支区分开。

修改内容：

- 复用已有 `getIndex()`、`RegionNode` 网格和 `route_distance`，按起点到候选节点的 region-index 射线判断“同方向是否还有更远节点”。
- 只有更远节点位于同一 EPIC region、同一 region-index 射线，或已到任务目标容差内时，才把当前节点视为该方向的中间节点。
- 正前方与右前方落在不同 region 射线时，正前方重新进入候选集合，随后继续由现有几何、语义、方向、FOV 和平滑 loss 排序。
- 未新增成员数据结构、辅助函数、参数或测试用例；未修改 `FUNCTION_TEST_CASES.md`。
- 未改变真实 frontier 的观测空间定义。当前 ROS2 仍没有 free/unknown cell 状态，这个批次只修正 region 方向上的拓扑 frontier 代理。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- `test_route_memory`、`test_epic_integration`、`epic_online_simulation` CTest `3/3` 通过。
- `test_topo_semantic` 中 TopoSearch、Verified-only frontier 相关定向测试 `12/12` 通过。
- `git diff --check` 通过。
- 首帧 region 复算：起点约 `(6,6)`，正前方约 `(6,11)`，右前方约 `(7,12)`，二者不在同一 region-index 射线。


## DOC-RC-YOPO-001 训练架构与在线文档体系整合

- 记录时间：2026-08-27
- 变更类型：设计、测试规格和证据索引；无生产算法修改
- 详细设计：[YOPO_TRAINING_INTEGRATION_DESIGN.md](YOPO_TRAINING_INTEGRATION_DESIGN.md)

变更内容：

- 将 `train_scalenav/docs` 中 Route-Conditioned YOPO 的系统边界、S0-S9 架构、函数接口、
  数据契约、持久化边界、训练/在线状态机和离线评测口径纳入在线文档索引。
- 明确 EPIC accepted witness 是生产路线的唯一来源；YOPO 在 witness 安全走廊内生成
  无人机动力学轨迹，不重新选择 frontier terminal 或全局绕行侧。
- 纳入训练侧 41 个单元、8 个模块、8 个集成和 5 个性能用例，保留 `RC` 编号，并与
  在线 M1-M6 的 151 项统计分开。
- 引用训练侧 `45 passed` 和 Batch 002 配对评测作为已有证据；未把“有测试代码”、
  离线单步评测或待执行闭环/性能场景标记为已通过。

<a id="chg-0031"></a>
## CHG-0031 禁止历史 witness 后缀进入本帧执行路径

- 记录时间：2026-08-27
- 状态：代码、同帧日志证据复核、Release 编译和相关自动化回归完成；真实仿真待复测
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0031](test_reports/TEST_REPORT_2026-08-26.md#chg-0031)

问题与根因：

- `16:05` 会话同帧快照 `graph_1579.json` 中，A* topology path 只有 4 点：`(-2.96,25.07) -> (-2.15,27.55) -> (1.15,30.85) -> (4.45,30.85)`；最终 selected witness 和 `path_790.json` 却有 15 点。
- 多出的 11 点从 `(4.50,25.45)` 一路向后到 `(0,0)`。执行路径因此先向北到 `y=30.85`，再折返向南；同一条已发布路径最小安全空间仅 `0.0285045691 m`。
- 代码确认这些点不是本帧 A* 产生的：`publish()` 调用 `forwardRouteFromPosition(last_witness_path_, selected_witness_path.back())`，再把返回的 `stable_from_terminal` 直接追加到本帧路径。随后组合结果又覆盖 `last_witness_path_`，使历史后缀持续进入下一 tick。

修改内容：

- 删除 `publish()` 中从 `last_witness_path_` 向本帧 selected witness 追加历史后缀的分支。
- 本帧执行路径现在只包含本帧 odom-rooted A* edge witness，以及本帧已经生成并碰撞检查的 `terminal_extension`；当前 extension 不存在时不再用历史 extension 替代。
- `last_witness_path_` 仍保存本帧最终路线，供 remembered-edge、阻塞探测、路线风险和候选比较使用，但不再直接注入执行输出。
- `witness_points=A->B` 明确表示本帧 A* witness 点数到追加本帧 extension 后的点数；update 日志新增 `route_memory_points`，记录进入 `publish()` 前的历史路线点数。
- `local_goal_source` 不再输出误导性的 `RHC_DISPLAY`；非保底目标统一标记为 `CURRENT`。
- 未增加辅助函数、阈值或路线选择规则，未修改测试源码和 `FUNCTION_TEST_CASES.md`。

验证目标：

- `scalenav_graph_ros2` Release 编译通过。
- 现有 `test_route_memory`、`test_epic_integration` 和 `epic_online_simulation` CTest `3/3` 通过；这些测试不复现现场的历史后缀注入，只作为编译与既有契约回归。
- `git diff --check` 通过。
- 同一 tick 的 `/epic/path` 在没有本帧 extension 时必须与本帧 A* edge witness 同终点、同点数，不得出现历史路线点。
- 有本帧 extension 时，`witness_points` 只允许增加该 extension 的点，且 extension 已通过当前碰撞检查。
- 真实复测不得再出现 `path_790` 这种到 A* terminal 后反向折返的折线。

<a id="chg-0030"></a>
## CHG-0030 frontier 半程刷新改用直线距离

- 记录时间：2026-08-27
- 状态：代码、Release 编译和相关自动化回归完成；真实仿真待复测
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0030](test_reports/TEST_REPORT_2026-08-26.md#chg-0030)

问题与根因：

- CHG-0029 修正发布路径起点后，`15:56` 会话的 `/epic/path` 起点已跟随本帧 odom/A*，但飞机仍停在 `y≈79.3 m`，frontier 固定于 `(1.15,80.35)`。
- 半程刷新此前比较 witness 弧长。当前 odom-to-terminal 路径每帧重建并写入执行路线后，`route_length` 与 `route_remaining` 同时缩短；末段仍为 `2.19/2.13 m`，比例接近 1，导致 `frontier_half_replan=0`。
- frontier 刷新的目的只是提前搜索下一段绕障路线，不需要精确估计飞机沿动态折线执行了多少。

修改内容：

- 接受新 frontier 时记录一次无人机到 terminal 的初始直线距离；仅当 terminal ID 或坐标实际改变时重置。
- 每帧使用实时无人机到 terminal 的直线距离作为剩余量；剩余不超过初始的一半时触发下一 frontier 候选搜索。
- 该条件只触发搜索，候选仍须通过兼容延伸或既有 loss 滞回，不直接强制提交差路线。
- 弧长 `route_length/route_remaining` 保留作诊断；新增 `frontier_initial_distance/frontier_remaining_distance` 日志。
- 未修改测试源码或 `FUNCTION_TEST_CASES.md`。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- 现有 `test_route_memory`、`test_epic_integration` 和 `epic_online_simulation` CTest `3/3` 通过；未修改测试源码。
- `git diff --check` 通过。
- 真实复测应在 `frontier_remaining_distance <= 0.5 * frontier_initial_distance` 时看到 `frontier_half_replan=1`，并在到达当前 terminal 前开始评估下一 frontier。

<a id="chg-0029"></a>
## CHG-0029 持久路线保留当前 odom 规划起点

- 记录时间：2026-08-27
- 状态：代码、Release 编译和相关自动化回归完成；真实仿真待复测
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0029](test_reports/TEST_REPORT_2026-08-26.md#chg-0029)

问题与根因：

- `15:00` 会话末帧 odom 为 `(4.37,109.82)`，本帧 A* 拓扑路径已从 `(4.23,109.82)` 起步，但最终 `/epic/path` 仍从旧 witness 投影 `(0.007,109.776)` 起步，横向脱节约 `4.36 m`。
- `publish()` 先构建正确的 odom-to-terminal edge witness，随后在 terminal 复用时用 `last_witness_path_` 的投影后缀整条覆盖；持久路线还绕过连续性检查。因此当前 A* 已恢复正确连接也无法修正发布路径。
- 脱节段不在 `/epic/path` 中，导致 path clearance、blocked 探测和 local-goal 前视都把旧路线误报为可直接执行。

修改内容：

- terminal 尚在持久路线前方时，保留本帧碰撞检查过的 `odom -> terminal` 前缀，只从 terminal 在路线记忆中的接点续接后缀。
- 每帧都以本帧 A* 的 odom-to-terminal witness 为路径起点；持久路线恢复不再绕过统一的 witness 连续性检查，完整执行路线同步更新当前 odom 起点。
- 未新增辅助函数、配置阈值或 frontier 选择规则；未修改测试源码和 `FUNCTION_TEST_CASES.md`。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- 现有 `test_route_memory`、`test_epic_integration` 和 `epic_online_simulation` 均通过，CTest `3/3`；未修改测试源码。
- `git diff --check` 通过。
- 真实复测应确认 `/epic/path` 首点与同帧 odom/A* 拓扑首点一致，不再冻结在 `(0.007,109.776)`；path clearance 必须覆盖当前机位接入段。

<a id="chg-0028"></a>
## CHG-0028 终点 extension 纳入完整执行路线记忆

- 记录时间：2026-08-27
- 状态：代码、Release 编译和相关自动化回归完成；真实仿真待复测
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0028](test_reports/TEST_REPORT_2026-08-26.md#chg-0028)

问题与根因：

- `14:12` 会话进入 mission-goal 窗口后停在 `y=124--125 m`。飞机已到 `y=124.36/125.18/124.22/124.48 m` 时，local goal 仍依次落在 `y=118.97/117.80/119.95/117.47 m`，形成明确的后退指令。
- terminal ID `1185` 对应的持久 Bubble 仍位于 `(7.75,116.65)`，但有 goal extension 时 `route_terminal_` 已表示 `(0,140)`。此前 `last_witness_path_` 只保存到持久 Bubble 的拓扑前缀，不保存已经碰撞检查的 terminal-to-goal extension。
- 下一规划 tick 因而从移动后的 odom 重新搜索到旧 Bubble，再拼接 goal extension，执行顺序变成“当前 odom -> 身后 Bubble -> mission goal”；route length 随飞机远离旧 Bubble从 `3.80 m` 增至 `17.07 m`。

修改内容：

- 新路线提交时，将实际发布的完整碰撞检查路径（拓扑 witness 和 terminal-to-goal extension）写入 `last_witness_path_`。
- 恢复同一 persistent terminal 时，无论当前是否再次生成 extension，都直接执行该完整路线相对当前飞机位置的前向后缀，不再用新 odom 到旧锚点的搜索结果覆盖它。
- 若完整路线已经到达同一个 extension 终点，不重复追加 extension；只有路线记忆尚未包含 extension 时才追加并更新路线记忆。
- 未增加新函数、距离阈值或候选规则，未修改 `FUNCTION_TEST_CASES.md` 和测试源码。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- `test_route_memory` `12/12`、`epic_online_simulation` `1/1` 通过。
- 排除 CHG-0027 已记录的既有动态边界失败和 terminal 旧契约后，`test_topo_semantic` `44/44`、`test_epic_integration` `4/4` 通过。
- 真实仿真需确认进入 goal window 后 local goal 持续位于完整 extension 的前向后缀，且飞机从 `y≈125 m` 继续到达 `(0,140)`；不得再次出现 route length 随远离旧锚点反向增长。

<a id="chg-0027"></a>
## CHG-0027 terminal 候选限定为真实走廊前端

- 记录时间：2026-08-27
- 状态：代码、Release 编译和相关自动化回归完成；真实仿真待复测
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0027](test_reports/TEST_REPORT_2026-08-26.md#chg-0027)

问题与根因：

- 回退后的 `12:00` 会话成功到达 `(0,140)`，但首个 frontier 仅距车 `12.32 m`，后续非终点 frontier 多为 `12--19 m`，accepted route 最长仅 `22.89 m`。同一初始 Graph 已存在 `y=20.95 m` 的真实执行层节点，因此不是前方无节点。
- `goalDirectedSearch()` 将所有满足约 `10 m` 最短执行路径的可达节点加入 terminal 列表，候选最多 `193`。对任务方向上的节点，路径几何代价增加与到终点启发式减少近似抵消，而语义/安全空间代价随路径累积，导致便宜的中间 Bubble 击败真正的走廊前端。
- 短路线进一步触发频繁的半程刷新：该会话 16 次切换中 `FRONTIER_HALF=12`。

修改内容：

- A* 仍遍历全部可达节点并使用普通 Bubble 组成路径；搜索完成后才提取 terminal 候选。
- terminal 必须是 `Verified`、非 odom、非 viewpoint 的真实 Bubble，并满足既有最短执行路径。
- 复用本次搜索已有的 `route_distance` 和展开集合：若节点存在 route distance 更大的可达 `Verified` 邻居，则它是中间节点，不参与 terminal loss；每条分支的距离局部极大节点以及局部搜索边界成为几何 frontier。
- 进入 mission goal 窗口时保留全部可达 `Verified` 节点，由既有 goal-tolerance 和安全 extension 逻辑收敛到真实终点。
- 未增加固定 `31.5 m` 硬阈值、语义 frontier、节点聚类或第二轮 A*；额外工作量为一次 `O(V+E)` 的已有邻接数据检查，完整 loss 只计算少量 frontier。

验证结果：

- 未修改 `FUNCTION_TEST_CASES.md` 或测试源码。
- `scalenav_graph_ros2` Release 编译通过；`test_route_memory` 和 `epic_online_simulation` 完整通过。
- `test_topo_semantic` 排除 1 项既有动态边界失败和 4 项已被本批设计替代的 terminal 旧契约后 `44/44` 通过；4 项旧契约分别要求虚拟语义点直接成为 terminal，或要求两层走廊在中间 Bubble 停止。当前实现按真实走廊前端返回 3 节点路径，符合本批目标。
- `test_epic_integration` 排除同样要求在中间 Bubble 停止的旧路径长度断言后 `4/4` 通过。保留测试源码原状，因此定向 CTest 汇总仍显示 `2/4` 目标通过；失败断言已如实记录，不通过修改测试掩盖行为变化。
- 真实仿真需确认首个 frontier 落在当前真实 Graph 外缘、`astar_candidate_terminals` 显著下降且半程切换频率降低；同时确认 terminal 高度保持执行层、虚拟语义点不成为 terminal。

<a id="chg-0026"></a>
## CHG-0026 回退语义点直接充当 frontier terminal

- 记录时间：2026-08-27
- 状态：代码回退、Release 编译和相关自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0026](test_reports/TEST_REPORT_2026-08-26.md#chg-0026)

回退原因：

- `11:54` 会话证明 CHG-0024/0025 的抽象不成立。飞机虽然恢复运动，但固定深度图像采样点被当作真实 3D terminal：日志中的 terminal 高度范围为 `-18.99..6.99 m`，首个目标即 `(-2.41,30.50,-4.45)`，不符合固定 `z=1.6 m` 的执行层。
- 语义点长期写入持久 Graph 后快速累积：38 条 update 日志中 `global_nodes` 最大 `2344`、`global_virtual_semantic_nodes` 最大 `1484`，最后 `persistent_semantic_records=1536`。候选数最大达到 `363`，已经不再是“本帧约 15 个候选”。
- 该会话记录 57 次 route switch，其中 `NO_ACCEPTED_ROUTE=36`、`BLOCKED=10`；直接远距接图把图像方向证据错误变成拓扑可达性，并未得到稳定 frontier。

回退内容：

- 撤回 CHG-0024 的 current-stamp terminal 过滤，恢复所有可达真实 Bubble 参与既有综合 loss；语义点恢复为路径风险证据。
- 撤回 CHG-0025 的全图最近 `Verified` 锚点连接与孤立点重连，恢复原局部语义关联，避免为未知的 30 m 图像采样点构造长距离几何 witness。
- CHG-0024/0025 的日志与结论保留为否定性记录，不再代表当前生效行为。

后续设计方向：

- 不再把 30 m 语义点本身作为 terminal。每个图像 patch 只提供方向和语义风险，在该方向锥内选择当前 Graph 中已验证、从 odom 可达的真实 Bubble；最终 terminal 始终是 `Verified` Bubble，语义只参与排序。
- 该方向映射应在一次现有 Graph 搜索中完成，不能把语义点持久化成大量拓扑节点，也不能额外为每个 patch 运行 A*。

验证结果：

- 未修改 `FUNCTION_TEST_CASES.md` 或测试源码。
- Release 编译通过；`test_route_memory` 12/12、`test_epic_integration` 5/5、`epic_online_simulation` 4/4 通过。
- 真实仿真需确认 terminal 恢复到真实 Bubble 执行层，不再出现 CHG-0024/0025 引入的负高度虚拟 terminal 和语义节点快速增长。

<a id="chg-0025"></a>
## CHG-0025 语义 frontier 接入真实 Bubble 连通分量

- 记录时间：2026-08-27
- 状态：已由 CHG-0026 回退；仅保留问题分析记录
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0025](test_reports/TEST_REPORT_2026-08-26.md#chg-0025)

问题与根因：

- `11:49` 会话中每帧生成 `12--14` 个固定深度语义点，持久语义节点的 `connected` 从 `6` 增至 `24`，但每次搜索 `astar_candidate_terminals=0`、`candidate_found=0`、`path_nodes=0`，飞机始终停在 `(0,0,1.6)`。
- 原语义接图只收集 `4.5 m` 内节点并按距离保留 4 个。真实 Bubble 最远约到 `20 m`，语义点位于约 `31--46 m`，两者通常相距约 `10 m`；附近反而只有其他 `Unknown` 语义点。因此日志中的 `connected` 是语义点彼此成团，不代表接入 odom 所在的真实 Graph 分量。

修改内容：

- 语义点的连接锚点只允许 odom 或 `Verified` Bubble，禁止以其他 `Unknown` 语义点作为锚点。
- 从全部真实锚点按空间距离排序，最多尝试最近 4 个，获得 2 条有效 collision-checked witness 后停止；不增加全图聚类或候选二次 A*。
- 本帧更新命中已有语义点时，如果该点仍没有 odom/`Verified` 邻边，则重新进入连接流程，使已形成的孤立语义团可以自行修复。

验证结果：

- 未修改 `FUNCTION_TEST_CASES.md` 或测试源码。
- Release 编译通过；`test_route_memory` 12/12、`test_epic_integration` 5/5、`epic_online_simulation` 4/4 通过。
- 排除当前工作树既有的 `TcM2004IndexBoundaryRejectsOutsideConfiguredGrid` 后，`test_topo_semantic` 48/48 通过。
- 真实仿真仍需确认首次语义更新后 `astar_candidate_terminals>0`、`candidate_found=1` 且产生非空 path；若仍失败，连接日志应进一步区分锚点搜索的 `REACH_END/TIME_OUT/NO_PATH`。

<a id="chg-0024"></a>
## CHG-0024 当前帧语义点作为 frontier 候选全集

- 记录时间：2026-08-27
- 状态：已由 CHG-0026 回退；仅保留问题分析记录
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0024](test_reports/TEST_REPORT_2026-08-26.md#chg-0024)

问题与修改：

- 原 `goalDirectedSearch()` 将局部 Graph 中所有可达普通 Bubble 都加入 terminal 列表；约 15 个固定深度语义点只会影响边代价，并不保证被当作 `frontier_goal` 候选，因此可选出近处普通节点。
- 规划入口已有本次成功应用语义帧的精确时间戳。在线调用的时间戳非零时，仅将 `semantic_stamp_ns` 等于当前正时间戳且带语义观测的可达节点加入 terminal 列表；普通 Bubble 继续作为 A* 中间节点，不再竞争 frontier。时间戳为 `-1`（当前无新鲜语义帧）时产生零个新 frontier 候选并保持 incumbent，不退化为普通 Bubble。
- 不按高语义分数筛选目标。当前帧的低风险和高风险语义点均可进入候选；语义风险、连续几何安全空间、到 mission goal 的距离、前向进度、方向、FOV 和平滑度继续通过现有统一 loss 排序。进入 mission goal 窗口时保留真实终点的普通几何候选例外，并由既有 goal-tolerance 逻辑优先收敛。
- 硬约束保持为已有的 Graph 连通、collision-checked edge 和最短执行路径。未增加距离阈值、走廊规则、第二轮 A* 或节点聚类；额外候选判断为每个已展开节点一次常数时间的时间戳比较。
- 没有有效当前语义帧的离线/兼容调用保持原普通节点 terminal 行为；在线搜索若当前帧语义点均不可达，则不伪造几何 frontier，由既有 accepted frontier 恢复逻辑保持路线。

验证结果：

- 未修改 `FUNCTION_TEST_CASES.md` 或测试源码。
- Release 编译通过；`test_epic_integration` 5/5、`epic_online_simulation` 4/4 通过。
- `test_topo_semantic` 48/49 通过，本批相关的语义和 frontier 搜索测试全部通过；唯一失败为既有 `TcM2004IndexBoundaryRejectsOutsideConfiguredGrid`。
- 完整包级 CTest 为 4/6 通过；另一个失败目标 `test_lidar_map` 有 3 个既有边界/窗口断言失败，均不涉及本批代码。

<a id="chg-0023"></a>
## CHG-0023 当前帧点云使用 PCL VoxelGrid 采样

- 记录时间：2026-08-27
- 状态：代码已实现，Release 编译和包级测试待执行
- `onCloud()` 在世界坐标变换后使用 PCL `VoxelGrid` 对当前帧点云采样，默认叶尺寸复用 `map_voxel_size=0.1 m`（下限 `0.05 m`）。采样后的点云同时用于 `updateCloudWorld()` 和 `latestCloudSnapshot()`，因此 Graph 区域更新不再扫描未经采样的原始帧。
- 云诊断日志新增 `voxel` 耗时、`voxel_points` 和 `leaf` 字段，并保留 `input`、`map_points` 以区分当前帧输入、当前帧采样结果和持久地图规模。
- 未改变 frontier loss、拓扑持久化或语义选择逻辑；空采样帧会被丢弃并限频告警。

<a id="chg-0022"></a>
## CHG-0022 YOPO 横向自由度不再使 frontier 失效

- 记录时间：2026-08-27
- 状态：代码、Release 编译和相关自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0022](test_reports/TEST_REPORT_2026-08-26.md#chg-0022)

问题与根因：

- `10:12` 最新会话虽然完成 `(0,0) -> (0,140)`，但逐帧 graph 快照在约 `23.27 s` 内记录了 `26` 个不同 frontier，即 `25` 次切换；切换间隔中位数仅 `0.65 s`，`17/25` 小于 `1 s`。2 秒节流的 update 日志只看到 2 次接受，遗漏了绝大多数短暂切换。
- 25 次切换中只有 4 次发生在旧 witness 确实过半；17 次切换前无人机到刚接受 witness 的横向距离已经超过 `route_reuse_lateral_distance_m=1.5 m`。旧代码把 `route_aligned=false` 同时用于清除 remembered edges、禁止 incumbent terminal 恢复、停止 persistent witness/local-goal 复用，并令下一候选以 `NO_ACCEPTED_ROUTE` 硬切，错误地把 YOPO 的局部避障和动力学自由度解释为全局路线丢失。
- CHG-0020 又把 `frontier_half_consumed` 直接列为 hard switch。过半搜索出的候选即使改变当前执行前缀也会立即提交；无人机仍沿原局部运动时很快超过 `1.5 m`，进一步触发上述连续硬切闭环。

修改内容：

- `route_aligned` 和 `route_lateral_error` 只保留为诊断量，不再参与 remembered edges、accepted terminal 有效性、候选搜索/硬切、persistent witness 保留或 local-goal 连续性。YOPO 偏离 witness 本身不触发任何全局规划状态变化。
- incumbent 仍按原有规划 tick 检查当前 odom 到同一个 persistent terminal 的 Graph 连通性；这是常规 RHC 检查，不由横向偏离触发。只有当前 FOV 执行前缀 blocked、terminal 确实无法从 Graph 恢复、路线过半或已有 loss 规则确认候选更优，才可能改变 frontier。
- 复用同一 terminal 时持续保留 accepted witness，并从无人机在该 witness 上的前向投影选择 local goal；不再因为横向距离超过 `1.5 m` 拒绝 local goal。terminal 已经位于任务方向后方的既有防回退检查保留，但不再依赖 `route_aligned`。
- 路线过半只触发下一 frontier 搜索，不再无条件硬切。复用已有 `candidateExtendsAcceptedRoute()`：保持当前前向前缀的自然延伸可记录 `FRONTIER_HALF` 并提交；分叉候选必须通过既有风险/综合 loss 滞回。
- 新增不节流的 `[EPIC route switch]` 事件日志，记录切换原因、新旧 terminal id、横向误差、是否兼容延伸以及路线总长/剩余长；节流 update 同步增加 `route_lateral_error`，避免再次漏判实际切换次数。

验证结果：

- `scalenav_graph_ros2` Release 编译、launch Python 语法检查和本批代码 `git diff --check` 通过。
- 未修改测试源码或 `FUNCTION_TEST_CASES.md`。最终二进制排除已知 `test_lidar_map` 后的 CTest 为 `5/5` 通过；其中相关回归 `test_route_memory`、`test_topo_semantic`、`test_epic_integration` 和 `epic_online_simulation` 为 `4/4` 通过。
- 全包 CTest 当前为 `5/6`；唯一失败目标 `test_lidar_map` 有 3 项既有工作树失败（map boundary/dead-area 和 100 帧体素窗口），对应本批未修改的 `lidar_map` 代码，不属于本次 frontier 变更。未回退或改写这些现有修改。

待真实验证：

- 复跑相同长航线，使用不节流的 route-switch 日志确认 `route_aligned=0` 单独出现时 terminal id、frontier 坐标和 accepted witness 均不变化。
- 确认过半候选只有 `compatible_extension=1` 才以 `FRONTIER_HALF` 提交；分叉候选只能记录 `LOWER_LOSS`，不再出现 `0.08--0.52 s` 的连续切换。

<a id="chg-0021"></a>
## CHG-0021 frontier goal 低频提交

- 记录时间：2026-08-27
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0021](TEST_REPORT_2026-08-26.md#chg-0021)

问题与根因：

- `09:51` 会话中 41 条节流 update 全部为 `horizon_ready=0`。旧刷新储备为 `max(10, 35-3.5)=31.5 m`，而 frontier 的语义径向偏好最多为 `30 m`，因此普通 accepted route 几乎一提交就被认为储备不足，每个规划周期都会搜索候选。
- 搜出的候选可通过 `COMPATIBLE_EXTENSION` 自动提交；本次节流日志记录 7 次。进入目标 `35 m` 窗口后，`goal_in_window` 同时禁用 incumbent 恢复、强制搜索并无条件 hard switch，导致 terminal 在近拓扑点和 mission goal 之间远近跳变。本次记录 `GOAL_WINDOW` 17 次，`FRONTIER_HALF` 0 次。
- 典型跳变包括去程 `v2t=6.62 -> 24.57 m`，以及回程 `4.50 -> 23.89 -> 5.59 m`；这不是 CHG-0020 的过半约束触发，而是旧 horizon 和 goal-window 切换路径。

修改内容：

- 移除固定 `31.5 m` horizon 对候选搜索的触发，不再使用 `COMPATIBLE_EXTENSION` 提前提交新 terminal。正常 accepted frontier 只在 witness 飞过一半时强制刷新。
- `goal_in_window` 只改变候选目标和 loss，不再禁用 incumbent、触发每帧搜索或构成 hard switch。进入目标窗口后仍保持 accepted frontier，直到过半、blocked 或 incumbent 无法恢复。
- blocked、首次无 accepted route、incumbent 丢失仍是必要硬切换；显著语义变化仍可发起一次候选 loss 比较。没有增加左右廊、路径形状或语义类别规则。
- 保留 `horizon_ready` 日志字段以兼容现有分析脚本，但其语义改为 accepted witness 尚未飞过一半；`frontier_half_replan=1` 是相反的刷新触发态。

验证结果：

- Release 编译、launch Python 语法检查和 `git diff --check` 通过。
- 未修改测试源码或 `FUNCTION_TEST_CASES.md`。`colcon test --packages-select scalenav_graph_ros2` 为 `6/6` CTest，在线仿真检查 `4/4` 通过；仓库测试结果汇总为 `76 tests, 0 errors, 0 failures, 0 skipped`。

<a id="chg-0020"></a>
## CHG-0020 frontier 路径过半强制刷新

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0020](TEST_REPORT_2026-08-26.md#chg-0020)

问题与修改：

- 最新 `23:13` 会话去程到达 `(0,140)`，但回程在 `y≈94` 附近发布过先向左到 `x=-5.45`、再横穿到 `x=1.15` 的 S 形路径，飞机横移后速度降至约 `0.2 m/s`；旧 frontier 在已经消耗较多时仍可由 incumbent/hysteresis 保留。
- 使用现有 accepted witness 的弧长计算执行进度；当无人机投影之后的剩余长度不超过提交时总长度的 `50%`，本轮必须搜索下一 frontier。候选连通且含有效节点路径时直接提交，切换原因记录为 `FRONTIER_HALF`。
- 该硬约束只规定 frontier 刷新时机，不规定左右通道、直线形状或语义类别；候选内部仍由现有几何、连续安全空间、语义、方向、FOV 与平滑 loss 共同选择。
- update 日志新增 `frontier_half_replan`、`route_length` 和 `route_remaining`，用于验证过半触发点和新 frontier 是否实际提交。

验证结果：

- Release 编译、launch Python 语法检查和 `git diff --check` 通过。
- 未修改 `FUNCTION_TEST_CASES.md` 或测试源码。`colcon test --packages-select scalenav_graph_ros2` 为 `6/6` CTest，其中在线仿真检查 `4/4` 通过；测试结果汇总为 `76 tests, 0 errors, 0 failures, 0 skipped`。

<a id="chg-0019"></a>
## CHG-0019 密集区路线代价尺度与历史边折扣一致化

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0019](TEST_REPORT_2026-08-26.md#chg-0019)

问题与根因：

- 最新完整会话在密集区入口先沿 `x=7.75` 前进，随后在约 `1787754821.076 s` 切到 `x=17.65`；同帧图中 `x=11.05` 的前向节点与边完整存在，因此不是“前方没有节点”。
- 从当帧 odom 到同一 `x=17.65,y=93.55` 终端，选中拓扑路线长约 `24.16 m`，图中欧氏最短节点路线约 `20.84 m`。当前点云复算还表明，经 `x=11.05` 前进的路线约 `18.04 m`、最小安全空间约 `2.27 m`，而选中绕路的最小安全空间约 `2.26 m`；大幅右绕没有换来更大当前安全空间。
- 候选搜索的几何项只乘 `goal_path_cost_weight=0.2`，语义项权重为 `2.0` 且生产 launch 的虚拟语义影响半径为 `10 m`。密集区地面/高空的虚拟语义投影可跨多列节点施加代价，并压过约 `6 m` 的额外绕行长度。
- 路线一旦被接受，incumbent `graphSearch()` 对 remembered edge 的几何代价直接乘 `0`，而候选搜索使用 `previous_path_cost_factor=0.9`。两套 loss 不一致，使较长旧路线在 `route_blocked=0` 时几乎无法被正常短路线替换。
- 最终目标进入局部窗口时，`connectTerminalToGoal()` 若 A*/碰撞检查失败，后续 fallback 又会直接拼接 `terminal -> goal` 直线，绕过刚刚失败的安全验证。

修改内容：

- `goal_path_cost_weight` 默认值由 `0.2` 调整为 `1.0`，候选、incumbent、路线切换比较和诊断日志统一使用该权重。几何距离恢复为正常代价尺度，不新增直行阈值或路线形状规则。
- incumbent remembered edge 不再是零几何代价，改为使用既有 `previous_path_cost_factor`；默认 `0.9` 只提供小幅路线惯性，与候选搜索一致。语义与连续安全空间代价保持参与比较。
- 生产 launch 的 `semantic_point_influence_m` 和 `semantic_route_influence_m` 统一为 `5.0 m`。语义仍参与 loss，但单个固定深度虚拟点不再横跨 `10 m` 影响相邻宽通道。
- 删除目标窗口内未经验证的直线 extension fallback。terminal 到 mission goal 只有在 `ParallelBubbleAstar` 搜索和碰撞复核均通过时才会追加；失败时路线终止于已验证拓扑节点并在后续周期重试。
- 路径日志的 `geometry` 字段同步使用 `goal_path_cost_weight`。update 日志新增同一 tick 的 `incumbent_loss/candidate_loss`、risk 和 progress，`route_compare=1` 表示该轮实际执行了软切换比较；无需再仅凭胜出路线反推被拒候选。

验证结果：

- Release 编译、launch Python 语法检查和 `git diff --check` 通过。
- 未修改 `FUNCTION_TEST_CASES.md` 或测试源码。`colcon test --packages-select scalenav_graph_ros2` 为 `6/6` CTest；`colcon test-result --all --verbose` 汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。

待真实验证：

- 复跑同一密集区，确认入口局部目标优先落在 `x≈11` 的短而宽通道，不再无收益地外绕到 `x≈17.65..20.95`。
- 对比 update 日志的 `geometry/semantic/clearance`，确认 remembered route 仍有约 `10%` 惯性但不会因零成本锁死；有实质更低 loss 的候选可记录 `switch_reason=LOWER_LOSS`。
- 目标窗口内若 A* extension 失败，确认发布 path 不含未经检查的 terminal-to-goal 长直线，且 `path_min_m` 不再出现低于 `0.61 m` 的拼接段。

<a id="chg-0018"></a>
## CHG-0018 原始几何改为当前帧并按可见执行前缀判阻塞

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0018](TEST_REPORT_2026-08-26.md#chg-0018)

问题与根因：

- 最新完整会话中，原 accepted witness 在 `path_640` 仍沿 `x=7.75` 直行；同一时刻无人机安全空间为 `11.89 m`，但累计地图报告 `path_min=0.31 m`，下一帧 `path_641` 即改为短左斜线。
- 对同一时刻当前原始点云进行世界坐标复算，当前帧高于规划层障碍离旧 witness 最近约 `16.36 m`，不支持累计地图的 `0.31 m` 判定。直接原因是 M1 点云在窗口内只追加 hit、没有 miss/清除，历史瞬时深度点可将持久 Graph 中的已验证路线误判为 blocked。
- Bubble、TopoGraph 节点、边和 edge witness 已承担长期路线记忆；继续让原始深度点云承担第二套长期记忆会使旧 hit 无条件覆盖持久图状态。

修改内容：

- 复用 `map_history_radius_m`：值为 `0` 时，障碍 KD-tree 在每次深度回调前清空，只保存当前帧经体素去重后的障碍；正值仍保留旧滑窗兼容模式。生产 launch 和节点默认值改为 `0`。
- 不随原始点云换帧整体清空持久 TopoGraph、Bubble、边、persistent id、edge witness 或语义记录。当前帧几何只负责实时安全空间、局部图更新和执行安全验证。
- 删除 `getRegionsToUpdate()` 沿 mission goal 方向预先种入最远 `50 m` 未观测 region 的逻辑。Bubble 差分只更新当前深度 hit 触发的 region 及从机体到 hit 的局部区域；mission goal 仍只用于已观测 region 排序和规划 loss。这避免当前帧模式把 FOV 外空白区域当成已观测自由空间，进而覆盖历史 Bubble。
- `current_route_blocked` 只检查无人机投影之后、最长 `local_goal_lookahead_m` 且不超过传感器量程的前向 witness 前缀；检查在首个超出当前水平 FOV 的点处停止。FOV 外的持久路线不再被当前帧判 blocked。
- 启动日志新增 `geometry_map=CURRENT_FRAME/SLIDING_WINDOW`，便于区分实际运行模式。

验证结果：

- Release 编译通过；launch Python 语法检查和 `git diff --check` 通过。
- 未修改 `FUNCTION_TEST_CASES.md` 或现有测试源码。`colcon test --packages-select scalenav_graph_ros2` 为 `6/6` CTest，汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。
- 仓库外临时 smoke 验证 `map_history_radius_m=0`：第二帧更新后只保留第二帧体素，第一帧障碍不再影响 `getDisToOcc()`；正值滑窗的既有 9 项地图测试继续通过。

待真实验证：

- 复跑密集区，确认启动日志为 `geometry_map=CURRENT_FRAME`，`map_points` 回落到单帧体素规模，不再固定为 `100000`。
- 确认 `path_640` 类直行 witness 不会被当前帧不存在的历史点判 blocked，同时当前 FOV 内真实新障碍仍能在 1--2 个规划周期内触发 blocked。
- 确认 FOV 外的历史 Bubble/边不因 mission-goal region 种入被差分淘汰，新 Bubble 只出现在当前深度覆盖的 region。
- 检查单帧深度缺测是否造成安全空间闪烁；若存在，只增加短时观测确认，不恢复 40 m 原始点云历史。

<a id="chg-0017"></a>
## CHG-0017 连续安全距离代价偏好宽走廊

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0017](TEST_REPORT_2026-08-26.md#chg-0017)

问题与根因：

- `edgeClearancePenalty()` 只在 witness 最小安全空间低于 `clearance_target_m` 时产生代价；达到目标后所有边代价均为 `0`，无法在多个可行走廊中偏向更大的安全距离。

修改内容：

- 使用 `clearance_target_m` 作为安全距离衰减尺度，按
  `w_clearance * edge_length * (target / (target + clearance))^2`
  计算连续、单调递减且有界的安全代价。
- 继续取端点 Bubble 半径与缓存 witness 最小安全空间的较小值；A* 仍只读取缓存，不在热循环查询点云。
- 安全空间越大代价越小，安全空间为零时的最大代价仍为 `w_clearance * edge_length`，避免数值失控。

验证结果：

- Release 编译通过。
- `colcon test --packages-select scalenav_graph_ros2` 通过；汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。
- 公式自测满足安全空间增大时代价单调下降，安全空间为零时安全因子为 `1`，大安全空间时趋近 `0`。

<a id="chg-0016"></a>
## CHG-0016 路线延伸保持走廊并按 witness 安全空间排序

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0016](TEST_REPORT_2026-08-26.md#chg-0016)

问题与根因：

- CHG-0015 后最新会话仍出现路线变化。日志中 `route_blocked=0`、`incumbent=RECOVERED` 时也会接受新候选，原因是 `route_has_execution_horizon=false` 被列为 `hard_switch`：前方执行储备不足 10 m 会绕过路线 loss 和兼容延伸检查，直接替换当前走廊。
- 原 clearance loss 只取边两端 TopoNode 的 Bubble 半径。端点半径均不小于 `1.2 m` 时惩罚固定为零，即使 edge witness 中间贴障；因此规划器无法优先选择实际更宽松的路线。

修改内容：

- 执行储备不足仍触发候选搜索，但不再构成硬切换。当前走廊只有在 blocked、没有可用 accepted route、incumbent 无法从当前拓扑恢复或进入最终目标窗口时才硬切；其他候选必须通过既有 risk/cost 滞回或 `candidateExtendsAcceptedRoute()`。
- TopoNode 边新增 `edge_clearance_` 缓存。edge witness 建立、修复及 odom 连接时计算一次最小障碍安全空间；相邻稀疏样本之间使用距离场 1-Lipschitz 性质计算保守下界，覆盖两个安全 Bubble 之间的窄颈。
- `edgeClearancePenalty()` 使用端点 Bubble 与缓存 witness 安全空间的较小值，并按实际 witness 长度计权。A* 展开只读取缓存，不向点云 KD-tree 发起额外查询。
- update 日志新增 `switch_reason`，取值包括 `BLOCKED`、`GOAL_WINDOW`、`NO_ACCEPTED_ROUTE`、`INCUMBENT_LOST`、`LOWER_LOSS` 和 `COMPATIBLE_EXTENSION`。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- `colcon test --packages-select scalenav_graph_ros2` 通过；汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。
- 未修改 `FUNCTION_TEST_CASES.md` 或现有测试源码。

待真实验证：

- 复跑左右廊场景，确认 `route_blocked=0` 且仅执行储备不足时不会改变当前走廊；兼容延伸允许 terminal 前移，但 local goal 前缀应连续。
- 检查胜出路线的 clearance loss 不再长期为 `0.00`，并用结构化 `path_min_m` 对照宽松路线是否真正被优先选择。

<a id="chg-0015"></a>
## CHG-0015 只按前向 witness 判定阻塞并保持局部目标顺序

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0015](TEST_REPORT_2026-08-26.md#chg-0015)

问题与根因：

- 最新两个会话中，`route_blocked=1` 时旧逻辑清空路线记忆并把候选切换视为硬切换，绕过了原有路线 loss 滞回，导致 terminal 在左右走廊之间反复改变。
- blocked 探测扫描完整 `last_witness_path_`，包含无人机已经通过的旧前缀；旧前缀被地图更新判为不安全时，会错误地使无人机前方仍可执行的路线失效。
- 发布阶段对已经按拓扑顺序展开的 witness 再次从整条折线寻找全局最近线段。回弯或平行走廊靠近无人机时，可能跳回旧段，令 `local_goal` 指向无人机后方或错误走廊。

修改内容：

- blocked 探测先使用 `forwardRouteFromPosition()` 截取无人机前方 witness 后缀，只对剩余执行路线调用碰撞检查；日志新增 `route_probe_points`，区分实际探测点数。
- 删除发布阶段对已排序 witness 的第二次全局最近线段投影，local goal 直接沿当前 witness 顺序做弧长前视。
- 未改变语义节点持久化、BubbleUnionSet 聚类、A* 路线 loss 权重或已有硬约束；语义节点仍保持独立风险证据。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- `colcon test --packages-select scalenav_graph_ros2` 通过；汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。
- 未修改 `FUNCTION_TEST_CASES.md` 或现有测试源码。

待真实验证：

- 复跑左右廊场景，确认已通过的旧前缀不再触发 `route_blocked=1`，且 `route_probe_points` 随无人机前进缩短。
- 确认 `local_goal` 不再回到无人机后方，并统计候选接受率和左右 terminal 切换次数。

<a id="chg-0014"></a>
## CHG-0014 拒绝局部无进展的持久化 witness

- 记录时间：2026-08-26
- 状态：代码、Release 编译和包级自动化测试完成；真实仿真复测待执行
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0014](TEST_REPORT_2026-08-26.md#chg-0014)

问题与根因：

- 最新 `19:45:00` 会话的 `path_99` 在起点附近包含回环/回退段。`canReuseForwardRoute()` 只检查无人机到路径的横向距离和剩余弧长，因此几何连续的坏 witness 被反复恢复。
- `horizon_ready=1` 只表示路径还有弧长，不表示前视点在任务方向上有有效进展；这会让 RHC 持续复用局部回环，而不触发候选路线重搜。
- 更直接的契约漏洞是：`accepted_witness_usable && route_has_execution_horizon` 时，旧代码允许 `found=1`，即使本帧 `path_nodes` 为空或 odom 拓扑节点没有连通边。此时 `publish()` 仍可用 `last_witness_path_` 独立驱动 local goal，形成“无节点绑定的几何折线”。

修改内容：

- 删除“仅凭 remembered witness 置 `found=1`”的分支；现在必须由当前图搜索恢复至少两个拓扑节点，且 odom 节点存在连通边，才允许 `found=1`。A* 返回空/单节点结果也会被拒绝并记录。
- 在发布前增加同一契约的统一最终检查，防止 candidate 分支或后续状态组合绕过节点数/odom 连通性要求。

设计边界：

- 本批次不新增路径形状、单行/多行、FOV 或语义风险阈值；几何 witness 仍由当前图搜索和既有碰撞检查负责。

验证结果：

- `scalenav_graph_ros2` Release 编译通过。
- `colcon test --packages-select scalenav_graph_ros2` 通过：`6/6` CTest，`76 tests, 0 errors, 0 failures, 0 skipped`（重建崩溃测试按条件跳过）。
- 未修改 `FUNCTION_TEST_CASES.md` 或现有测试源码。

<a id="chg-0013"></a>
## CHG-0013 虚拟语义按当前帧代际参与规划

- 记录时间：2026-08-26
- 状态：代码、自动化测试和单次真实长航线复测完成；完整往返因 AirSim 深度 RPC 断流未完成
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0013](TEST_REPORT_2026-08-26.md#chg-0013)

问题与根因：

- 固定 30 m 虚拟语义点会持久化在 graph/memory 中，但旧实现把当前帧和历史虚拟点混入同一局部规划池。18:57 会话一帧只有 `15` 个新点，A* 却加载 `435` 个局部语义点；旧右侧高风险可通过 max-risk 持续压制右路。
- 没有新增 `semantic_virtual_planning_max_age_ms=1500` 一类独立滑动时间阈值。该设计会使路线选择受语义频率、调度延迟和机器负载影响，同一语义序列可能得到不同工作集。

修改内容：

- 未验证的 `Unknown` 虚拟语义点仅在其 `semantic_stamp_ns` 等于“当前成功应用语义帧”的 stamp 时参与 A*、incumbent 恢复、路线风险、候选比较和发布路径语义统计；新帧应用后，上一代虚拟点立即退出计算。
- 历史虚拟点仍保留在持久化 graph/memory 中，不做删除；真实几何确认后的 `Verified` 语义节点不受代际过滤，可继续作为长期证据参与规划。
- 原有 `semantic_max_age_ms` 仍只负责拒绝输入陈旧帧，并在语义流整体超龄时禁用全部未验证虚拟点；它不是历史虚拟点逐点保留 `1500 ms` 的规划窗口。
- 日志新增 `local_inactive_virtual_semantic_nodes` 和 `astar_inactive_virtual_semantic_nodes`，与 `persistent_semantic_records`、全局/局部节点数共同区分持久化存储和实际规划工作集。

验证结果：

- `scalenav_graph_ros2` Release 编译通过；未修改仓库测试源码。
- 临时 smoke test 覆盖当前代虚拟点保留、上一代退出、`Verified` 历史点保留、语义断流时虚拟点退出和默认 API 兼容，`5/5` 条件通过。
- 最终工作树状态下全包 CTest `6/6` 通过；限定本包结果汇总为 `68 tests, 0 errors, 0 failures, 0 skipped`；`epic_online_simulation` 为 `4/4`。
- 19:23 会话成功到达 `(0,140)`，去程在 `y≈30 m` 后主要选择右侧 `x≈7.75..14.35 m`。在同一静态图和同一代价快照下，反向右侧路线应作为回程的优先候选；但回程没有直接复用或比较它，首要原因是 `onGoal()` 在任务目标翻转时主动清空 `last_witness_path_`、`last_path_nodes_`、route terminal 和 corridor hint。设计上新 mission goal 会启动一条新路线，虽然 graph/edges/semantic memory 复用。于是回程第一轮只能在复用的全局拓扑上重新搜索，先得到左侧 terminal；左廊在 `y≈94 m` 被判 blocked 后，规划器才改选右侧 terminal `(11.05,67.15)`，不再持续锁定左侧 terminal。
- 这不能简单解释为“左侧节点代价确实更低”：当前日志只输出胜出路线，且实际代价是有向 edge witness 几何、当前语义帧风险、安全空间和 progress/direction/FOV/smoothness terminal loss 的组合，不是单个节点静态代价。需要把去程路线反向作为候选重新计算，并同时打印其各项 loss 与新搜索候选，才能区分“旧路线未进入比较”与“当前动态代价真的让左路更优”。
- 末段持久化记录为 `1105`、全局虚拟语义节点为 `1058`，局部旧虚拟点跳过 `459`，A* 旧虚拟点跳过 `498`，实际 A* 语义池仅 `34` 个。末段右切 witness 为 `22` 点，结构化日志的 `path_min=0.723 m`。
- 该会话没有完成回程：结构化日志的最后一帧 depth/pointcloud stamp 为 `1787743464.870896`。控制器使用 `0.5 s` depth watchdog，约在 `1787743465.371` 起停止控制输出；错误由独立的 `2 s` 状态定时器在 `1787743465.918` 打印，随后 renderer 报 `AirSim render RPC failed: timed out`。飞机停在 `(-4.63,93.82)` 是传感器失联触发的控制停机，不是右切路线搜索失败。

后续：

- 19:23 会话在断流前出现一次 `1261 ms` update，以及一次 `2292 ms` 后台更新（其中 `region_select=1884 ms`、`skeleton=304 ms`）。这些长尾与 RPC 断流时间接近，但日志只能证明相关性，不能证明其导致 AirSim RPC 超时；应分别增加系统负载和 renderer RPC 时延诊断。
- 修复或隔离 AirSim RGB-D RPC 断流后，重新执行不中断的 `(0,0) -> (0,140) -> (0,0)`，并确认当前左侧高风险时仍稳定选择右侧路线。
- witness 内部安全空间尚未纳入 terminal clearance loss，仍作为独立问题处理。

<a id="chg-0012"></a>
## CHG-0012 local goal 严格沿 witness 弧长前视

- 记录时间：2026-08-26
- 状态：代码、自动化测试和真实左侧走廊路径顺序复测完成；路线最优性仍待修复
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0012](TEST_REPORT_2026-08-26.md#chg-0012)

问题与根因：

- 18:44:51 和 18:43:56 两次会话均停在左侧走廊 `y≈80 m`。最新会话末段 accepted witness 要求先从约 `(-7.9,79.6)` 横移到 `(-21.95,80.35)`，再折回向前；其 10 m 弧长前视点约为 `(-17.9,80.35)`。
- 旧 `selectNextGoal()` 在存在 mission goal 时按 mission-axis 投影寻找 10 m 前进点，错误发布约 `(-8.75,90)`，跳过 witness 的整个横向绕行段。YOPO 因而尝试直达 witness 后段，在障碍前减速横移；EPIC 检测 blocked 后重规划同类 witness，又被相同投影逻辑跳过。

修改内容：

- 新增 `routeLookaheadPoint()`，先将 vehicle 投影到 accepted witness，再严格沿 forward witness 累计路径弧长并选择 lookahead 点。
- `selectNextGoal()` 删除 mission-axis 投影分支，横向绕行、短时后退和 U 形路线均按 witness 的实际顺序执行。
- mission goal 进入 `goal_connect_distance_m` 后直接发布最终 goal 的原有收敛逻辑保持不变；`local_goal_min_advance_m` 后备逻辑保持不变。

验证结果：

- `scalenav_graph_ros2` Release 增量编译通过。
- 未修改仓库测试源码；额外临时 smoke test 覆盖“先横移再前进”“先后退再绕行”“无人机从直线 witness 中段投影”，`3/3` 通过。
- 最终工作树状态下全包 CTest `6/6` 通过；`test_route_memory` 为原有 `12` 项，限定本包结果汇总为 `68 tests, 0 errors, 0 failures, 0 skipped`；`epic_online_simulation` 为 `4/4`。
- 对应现有 `MT-M5-001`、`MT-M5-004`、`TC-M6-006` 和 `IT-FLT-007` 的 local-goal/witness 顺序与闭环执行要求；未修改 `FUNCTION_TEST_CASES.md`。
- 18:57:56 真实左廊复测中，vehicle 位于 `(-3.50,77.77)` 时 accepted witness 先沿 `y≈80.35` 横移，新 local goal 为 `(-12.23,80.35)`，与从 vehicle 投影起约 `10 m` 的折线弧长点一致。无人机实际执行横移，最远到 `x=-28.88`，随后到达 `(0,140)` 并切换回程目标，确认不再跳到 witness 后段 `y≈90 m`。
- 该会话在回程到 `(-5.37,102.81)`、速度约 `4.48 m/s` 时收到 `SIGINT/SIGTERM`，因此不是卡停，但也不能计为完整往返验收。

后续：

- 本次复测仍没有选择更短的右侧路线。对应 graph 快照在 `75<=y<=110 m` 有右侧平面拓扑节点 `55` 个、左侧 `24` 个，说明不是右侧无图。
- 当前原始语义与持久化风险图发生矛盾：转弯前 `semantic_227` 的五个水平分区峰值约为 `0.631/0.506/0.247/0.202/0.188`，结合当时位姿投影后，高风险明确位于世界坐标左侧；但规划快照中该区域的 `14` 个持久化 `SEM-RISK` 全在右侧、左侧为 `0`。因此不能把持久化 marker 数量当成当前帧语义，也不能据此得出“当前语义选择左路”的结论。
- 该次搜索加载 `435` 个局部语义节点，而每帧最多只插入或更新 `15` 个点。`semanticNodes()` 只做空间半径筛选、不做观测时效筛选，`edgeSemanticRisk()` 又对候选取最大值；绝大多数未被当前帧触碰的历史虚拟点仍参与计算，一个旧的右侧高风险点即可持续抬高整条右路代价。固定 30 m 射线端点是推测位置，却按永久世界锚点进入搜索，这是当前“左侧风险更高但仍选左路”的首要根因候选。
- 当前 `goal_path_cost_weight=0.2`、`semantic_cost_weight=2.0`；risk=`0.35` 时每米语义项约为 `0.86`，是基础几何项 `0.2` 的约 `4.3` 倍。还需确认这些右侧虚拟风险是否为有效语义证据，并重新标定几何/语义权重。
- safety loss 只使用边端点 Bubble 半径，所选左路在 update 中的 clearance loss 被记为 `0.00`，但同一时刻结构化日志的实际 witness `path_min=0.144 m`，全会话最低 `0.035 m`。应把 witness 内部最小安全空间纳入边代价后再比较左右候选。
- 当前日志只记录胜出路线，不能还原同一 tick 所有候选的完整 loss；增加候选 geometry/semantic/clearance/progress/direction/FOV/smoothness 分解，以及语义候选的当前帧/历史分项后，完成一次不中断的 `(0,0) -> (0,140) -> (0,0)` 验收。

<a id="chg-0011"></a>
## CHG-0011 A* 语义边代价改为邻域索引查询

- 记录时间：2026-08-26
- 状态：代码、自动化测试和真实往返性能日志验收完成
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0011](TEST_REPORT_2026-08-26.md#chg-0011)

修改内容：

- `graphSearch()` 和 `goalDirectedSearch()` 每次搜索对当前局部语义池构建一次均匀空间哈希，网格尺寸使用语义影响半径且不小于 `0.5 m`。
- 每条拓扑边按实际 witness polyline 的各段建立扩展包围盒，只查询语义影响半径内可能相关的网格；跨折线段候选去重后继续执行原有点到 witness 精确距离和风险衰减计算。
- `semantic_query_nodes` 继续表示本次 A* 的完整局部语义池，`semantic_candidate_checks` 改为索引预筛后真正进入精确距离判断的候选数，用于直接衡量优化收益。
- 语义影响半径、风险公式、terminal loss、硬约束和公开的 `semanticRiskForEdge()` / `routeEdgeCost()` 接口均未改变。

验证结果：

- `scalenav_graph_ros2` Release 增量编译通过，安装目录中的 `epic_graph_node` 软链接已指向新构建产物。
- 全包 CTest `6/6` 通过，`0` 失败；`TC-M4-010` 对应的 witness 折线风险、语义绕行、frontier 排序及在线集成回归均通过，未修改测试源码。
- 对 18:11 会话最终 graph snapshot 按 launch 中 A* 的 `10 m` 语义影响半径做离线预估：475 个 45 m（35 m 搜索半径加 10 m 影响范围）内虚拟语义点和 988 条 35 m 内边，逐边全池扫描为 `469300` 次，空间哈希包围盒候选为 `68359` 次，保留 `14.6%`，约减少 `6.9` 倍。
- 18:40 会话完成 `(0,0) -> (0,140) -> (0,0)` 往返，36 条 update 中 `astar_semantic_checks` P50/P95/最大值为 `93040/273263/298414`，不再等于 `astar_edge_evaluations * astar_semantic_nodes`。全程实际精确检查 `3888153` 次，对应逐边全池扫描理论值 `23372933` 次，累计减少 `6.01` 倍。
- 同一会话 A* P50/P95/最大值为 `39.69/70.00/139.22 ms`，update 为 `107.99/267.35/365.61 ms`，且 `astar_timed_out=0`。与 18:11 优化前基线的语义检查 P95 `1627093`、最大 `1788796` 相比，分别下降 `83.2%` 和 `83.3%`。
- 18:44 前后的三个短会话同样显示空间索引生效，逐边全池扫描理论值相对实际检查数减少 `4.66-4.80` 倍；其中两次在左侧走廊 `y≈80 m` 失败属于 local-goal 跳过 witness 横向绕行段的执行问题，不是语义候选索引失效或 A* 超时。

后续：

- 日志中的 `local_semantic_radius=43 m` 来自独立的 `8 m` 路线风险影响参数，不是 A* 的 `10 m` 边风险影响参数；两种窗口口径继续保持分离。
- A* 最大值仍达到 `139.22 ms`，update P95 仍为 `267.35 ms`，后续性能工作应继续细分非 A* 时间和长尾，不应再恢复逐边全池扫描。

<a id="chg-0010"></a>
## CHG-0010 持久化、全局图与局部 A* 工作集诊断

- 记录时间：2026-08-26
- 状态：代码、自动化测试和单次真实往返日志验收完成
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0010](TEST_REPORT_2026-08-26.md#chg-0010)

修改内容：

- 保留原有 `[EPIC timing][update]` 字段，并新增 `persistent_semantic_records`、`global_nodes/global_edges/global_semantic_nodes`，明确区分持久化记录和全局发布图规模。
- 新增 `local_graph_nodes`、`local_semantic_nodes` 和 `local_semantic_radius`，分别记录 `35 m` 拓扑窗口与当前 `43 m`（35 m 搜索半径加 8 m 语义影响范围）语义查询池，不再用全局 marker 数量推断局部规划负载；该半径不同于障碍点云的 `40 m` 历史窗口。
- `graphSearch()` 和 `goalDirectedSearch()` 新增只读搜索统计，记录搜索次数、实际展开节点、incumbent/candidate 分项展开数、边代价计算次数、语义候选扫描次数、可行 terminal 数及超时状态。
- 局部窗口诊断遍历放在原有 update 计时截止点之后，避免新增打印改变 `total` 的统计口径；A* 内部计数不改变搜索排序和返回结果。

验证结果：

- `scalenav_graph_ros2` 增量编译通过。
- 全包 CTest `6/6` 通过，`0` 失败；未修改测试源码。
- 已确认编译产物包含完整新增日志格式。
- 18:11 会话完成 `(0,0) -> (0,140) -> (0,0)`。43 条 update 样本均输出完整诊断字段，`astar_timed_out=0`；最终持久化语义记录为 `1586`，全局图为 `2182` 节点/`4078` 边，35 m 局部图为 `507` 节点，43 m 语义查询池为 `472` 节点，A* 实际展开 `303` 节点。
- 回访历史空间时局部工作集明显增加。相距约 `4.0 m` 的去程/回程位置，持久化记录从 `338` 增至 `1203`，局部图从 `419` 增至 `659`，局部语义点从 `326` 增至 `619`，A* 语义候选检查从 `499328` 增至 `1625456`。
- 43 条样本中 `astar_semantic_checks` 均严格等于 `astar_edge_evaluations * astar_semantic_nodes`，确认当前边代价仍逐边扫描整个局部语义池；P95 为 `1627093`，最大 `1788796`。
- update 平均 `208.16 ms`、P95 `522.51 ms`、最大 `634.39 ms`，其中 A* 平均 `46.43 ms`、P95 `78.97 ms`、最大 `94.17 ms`。扣除 A*、publish 和 odom_connect 后的未分项时间平均 `127.06 ms`、P95 `424.42 ms`，说明超周期不能只归因于 A*。

日志结论与后续：

- 语义边邻域查询已由 CHG-0011 实现；仍需用修改后的在线日志比较 `astar_semantic_checks` 和 A* 时延。
- 继续细分 update 中等待 `topology_operation_mutex_`、`updateTopoSemanticMemory()`、路线 remap/metrics 和 publish 后风险复核的耗时；当前 `total` 中仍有较大的未分项区间。
- 后台 rebuild 平均 `262.50 ms`、P95 `524.58 ms`，其中 `region_select` 平均 `128.53 ms`、最大 `476.47 ms`；需要降低其与在线 update 的锁竞争。

<a id="chg-0009"></a>
## CHG-0009 frontier 统一软代价与语义证据置信度

- 记录时间：2026-08-26
- 状态：代码、自动化测试和两次真实往返完成，待第三次重复验收
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0009](TEST_REPORT_2026-08-26.md#chg-0009)

修改内容：

- `goalDirectedSearch()` 不再用 `31.5 m`、任务方向投影或相机 FOV 预先淘汰 frontier 候选。局部搜索范围内具有有效连接、碰撞检查 witness 且满足 local-goal 执行长度的节点统一参与排序。
- `31.5 m` 保留为滚动规划储备和候选进度的软目标；任务方向、水平 FOV 与路径转向平滑度也改为软代价。几何、语义、安全空间、incumbent 连续性和上述偏好共同构成 terminal loss。
- 新增 `frontier_progress_loss_weight`、`frontier_direction_loss_weight`、`frontier_fov_loss_weight` 和 `frontier_smoothness_loss_weight` 参数，默认分别为 `0.5/0.35/0.2/0.35`。
- mission goal 进入局部窗口后取消 frontier 执行长度下限，允许短路线直接收敛到任务终点。
- 语义 patch 保留固定 optical Z 投影后的原始三维世界坐标，不再在语义更新阶段清零垂向分量或压到固定飞行层。
- 单行/多行垂向一致性、FOV 边缘位置和 fixed-layer 下的疑似地面响应只调整语义置信度，不作为节点或路线的硬约束；置信度随语义分数一起进入节点 EMA 和路线语义代价。

验证结果：

- `scalenav_graph_ros2` Release 编译通过，CTest `6/6` 通过。
- `colcon test-result --verbose` 汇总为 `68 tests, 0 errors, 0 failures`；`epic_online_simulation` 为 `4/4` 检查通过。
- 未修改测试源码；现有 `RadialFrontierAllowsWideSemanticDetour`、`IncompleteFrontierFallbackStillUsesCombinedLoss`、语义路线选择和在线集成回归均通过。
- 17:40 会话完成 `(0,0) -> (0,140) -> (0,0)`；初始 `vehicle_to_frontier=30.87 m` 的候选被接受，确认旧 `31.5 m` 硬门槛已移除。41 条节流样本中找到候选 38 次、接受 33 次，说明候选不会无条件替换 incumbent。
- 18:11 会话再次完成相同往返；43 条节流样本中找到候选 39 次、接受 31 次，12 次 `RHC_DISPLAY` 保留 incumbent。两次任务均确认软 loss 与路线滞回持续生效。

待验收：

- 在 AirSim/Colosseum 长航线中记录各候选 loss 分解，确认低于 `31.5 m` 但明显更安全的 terminal 能稳定胜出，且不会选择不可执行的近点。
- 分别注入单行地面响应、三行一致目标和 FOV 边缘目标，确认地面误检只降置信度、不造成持续偏航或左右振荡。
- 复测完整局部图搜索时延，确认遍历全部物理可行 terminal 后仍满足在线规划预算。

<a id="chg-0008"></a>
## CHG-0008 persistent terminal 恢复与兼容式 frontier 延伸

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成，待真实仿真验收
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0008](TEST_REPORT_2026-08-26.md#chg-0008)

修改内容：

- accepted terminal 同时保存位置和 `persistent_id`，graph rebuild 后优先按稳定 ID 恢复，位置最近邻仅作为后备。
- incumbent 恢复不再要求剩余路线仍有完整 `31.5 m` 规划储备；储备不足只触发候选延伸，不再使 incumbent 自动失效。
- 增加独立的执行储备检查。terminal remap/A* 暂时失败时，只要 accepted witness 连续、无碰撞且仍有 local-goal 前视，就继续发布旧 witness。
- frontier 滚动候选必须比 accepted route 更长，并在受保护前缀内保持横向兼容；近车位置换到相邻横向走廊仍必须通过风险/综合代价滞回。
- 硬碰撞、accepted witness 不可用或执行储备耗尽时仍允许强制提交可达候选。
- 规划日志增加 `incumbent=NOT_ELIGIBLE|REMAP_FAILED|SEARCH_FAILED|RECOVERED` 和 `terminal_id`，用于区分重映射失败、不可达/超时和正常复用。

验证结果：

- `test_route_memory` 增至 `12` 项并全部通过；已验证同走廊前向延伸可以提交，近车横向换道不能借“延伸”绕过切换滞回。
- `scalenav_graph_ros2` 编译通过，全包 CTest `6/6` 通过。
- 18:44:51 和 18:43:56 两次会话分别有 `7/11`、`5/11` 条 update 为 `route_blocked=1`，均规划到左侧 terminal `(-5.45,110.05)`，最终停留在 `y≈79.6/79.8 m` 后由 SIGINT 结束。
- 18:44:51 的结构化 `/epic/path` 已定位直接根因：末段 witness 从约 `(-7.9,79.6)` 先沿 y≈80 横移到 `x=-21.95`，绕过障碍后才折回前进；`selectNextGoal()` 的 mission-guided 分支却按任务方向投影选择 10 m 前视点，直接发布约 `(-8.75,90)`，跳过必须执行的约 14 m 横向绕行。YOPO 因而朝 witness 后段直连目标运动，在障碍前减速横移；EPIC 再次检测 blocked 后重规划同类 witness，形成重复闭环。

待验收：

- 长航线统计 `RHC_DISPLAY/RHC_REPLAN/EXTEND`，确认 horizon ready 且无阻塞时不再全部接受候选。
- 分析 `SEARCH_FAILED` 的剩余案例，必要时继续区分 disconnected 与 timeout。
- local-goal 跳段已由 CHG-0012 修复并完成自动化回归；仍需在 `y≈80 m` 左廊场景确认发布点落在 witness 的首段横向绕行上。

<a id="chg-0007"></a>
## CHG-0007 全局语义 graph 的局部代价查询与重复观测合并

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成，待真实仿真验收
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0007](TEST_REPORT_2026-08-26.md#chg-0007)

修改内容：

- 全局 TopoGraph 和历史语义属性继续持久化，不删除无人机 `40 m` 窗口外的 graph。
- `graphSearch()`、`goalDirectedSearch()`、路线风险评估和发布统计只加载当前 `35 m` 搜索范围加语义影响半径内的语义节点。
- 语义边代价不再对每条候选边重复遍历全部全局语义 graph。
- 语义点匹配距离统一使用 `semantic_node_match_distance`，默认 `2.5 m`；同一空间附近的重复观测更新已有 persistent node，而不是按 `0.75 m` Bubble 半径持续新建节点。

验证结果：

- `test_topo_semantic` 增至 `40` 项并全部通过；新增测试确认局部语义查询排除远距离历史节点。
- `test_epic_integration` `5/5`、全包 CTest `6/6` 通过。

待验收：

- 长航线确认全局 graph 仍完整保留，同时 `semantic_memory` 增长率和 A* 时间明显下降。

<a id="chg-0006"></a>
## CHG-0006 语义点采用普通节点模型和固定 optical depth

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成，待真实仿真验收
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0006](TEST_REPORT_2026-08-26.md#chg-0006)

修改内容：

- `semantic_virtual_depth_m=30` 现在表示相机 optical Z depth，与普通深度点使用相同的针孔投影约定。
- 不再归一化每条 patch 射线；中心和边缘 patch 的相机前向深度都为 `30 m`，边缘点的三维距离允许大于 `30 m`。
- 新语义点使用普通 `TopoNodeRole::Geometric`，共享普通 graph 的中心、半径、邻接、witness、权重、persistent id 和语义属性。
- 真实深度验证前仅通过 `geometry_state=Unknown` 表示几何尚未确认；真实 Bubble 到达后提升为 `Verified` 并保留语义状态。
- 普通 `epic_skeleton_nodes` marker 现在包含未验证语义点；`epic_semantic_points` 仅作为风险颜色和文字覆盖层，不再代表另一套 graph。

验证结果：

- `test_topo_semantic` `40/40` 通过；已确认中心和边缘 patch 的 optical Z depth 均为 `30 m`、边缘点三维距离大于 `30 m`，并且投影不依赖 measured depth。
- `scalenav_graph_ros2` 编译通过，全包 CTest `6/6` 通过。

待验收：

- 仿真确认无人机前方普通 graph 中可见这些节点，并且真实深度到达后能稳定提升几何状态。

<a id="chg-0005"></a>
## CHG-0005 固定 30 m 虚拟深度语义点（已被 CHG-0006 修正投影定义）

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成，待真实仿真验收
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0005](TEST_REPORT_2026-08-26.md#chg-0005)

修改内容：

- 将远场语义链路统一为“语义风险 patch -> 固定 `30 m` 虚拟深度语义点”。
- 语义点不再依赖 measured depth，也不再使用 `pseudo/speculative` 概念。
- 语义点与普通 graph 点共用 `TopoNode`、persistent id、邻接边、witness、权重和语义属性；来源角色为 `Semantic`，真实 Bubble 验证前几何状态为 `Unknown`。
- 语义点通过普通 graph 插入和连接流程加入拓扑，连接边继续执行点云碰撞搜索，虚拟深度不作为已验证自由空间。
- 删除 `semantic_depth_clip_m=20`、`speculative_forward_m=22` 和 3 秒 speculative 过期流程。
- 新增 `semantic_virtual_depth_m=30` 和 `semantic_point_*` 参数。
- RViz 与日志命名改为 `epic_semantic_points`、`EPIC semantic graph` 和 `virtual_depth`。

验证结果：

- `scalenav_graph_ros2` 编译通过，CTest `6/6` 通过。
- `test_topo_semantic` 增至 `39` 项，新增测试确认中心和边缘 patch 均投影到固定 `30 m`，且投影不依赖 measured depth。
- 当前自动化结果为 `64` 项 GTest 和 `4/4` 在线仿真检查通过；另有 1 个依赖外部日志的 rebuild 场景按条件跳过。

待验收：

- 重新运行 AirSim/Colosseum，确认有效语义帧输出 `virtual_depth=30.00 m`。
- 确认无人机前方持续存在 `epic_semantic_points`，不再因真实 depth 小于裁剪值而消失。
- 确认语义点能连接到 `35 m` 局部 graph，并在高风险 patch 附近提高 A* 边代价。
- 确认所有被采用的语义点连接均具有 collision-checked witness。

<a id="chg-0004"></a>
## CHG-0004 障碍点云改为无人机中心 40 m 严格滑动窗口

- 记录时间：2026-08-26
- 状态：代码、自动化测试和日志复核完成
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0004](TEST_REPORT_2026-08-26.md#chg-0004)

修改内容：

- 障碍点云改为严格的无人机中心滑动窗口，默认半径 `40 m`。
- 窗口外点不再因为仍位于任务边界内而永久保留。
- 全局路线记忆只由 TopoGraph 节点、边和 witness path 负责。
- 点云达到容量上限时按距无人机由近到远保留，避免当前近障碍被历史点挤出 KD-tree。

验证结果：

- `test_lidar_map` 的严格滑窗和近点优先容量测试通过。
- 14:29 仿真末段 `/epic/clearance.vehicle_m` 约为 `2.0–2.4 m`，不再出现修改前错误的约 `29 m` 安全空间。

待验收：

- 继续运行完整长航线，确认 `/epic/clearance` 与原始点云近障碍距离持续一致。
- 确认遇墙时 route blocked 或候选切换在一个规划窗口内出现，且不发布穿墙 local goal。

<a id="chg-0003"></a>
## CHG-0003 路线重评估、切换滞回与 witness 连续性

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成，待长航线复核
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0003](TEST_REPORT_2026-08-26.md#chg-0003)

修改内容：

- 将“语义风险触发候选搜索”和“提交新路线”拆成两个阶段。
- 候选只有在明显更安全或综合代价明显更低时才替换 incumbent；硬碰撞仍允许直接切换。
- 风险基线按已评估路线累计，增加高风险锁存和释放阈值，避免稳态风险每个规划 tick 重搜。
- witness 连续性改为无人机到整条路径的投影和横向距离，不再检查无人机到 witness 首点的距离。
- 增加短时 local goal 保底，并禁止候选在通过切换门槛前覆盖 accepted witness。

验证结果：

- route memory、候选切换滞回和 witness 投影相关单元测试通过。
- 最新日志未再出现 witness 首点落后导致的不连续拒绝。

待验收：

- 继续统计 `route_mode=EXTEND candidate_accepted=1`，确认 incumbent terminal 在真实 graph rebuild 条件下能够稳定复用。
- 验证稳态高风险只触发一次候选评估，不会造成左右路线反复切换。

<a id="chg-0002"></a>
## CHG-0002 A* 加入语义、安全空间和弱连续性代价

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0002](TEST_REPORT_2026-08-26.md#chg-0002)

修改内容：

- 保留几何路径代价。
- 增加语义风险代价和安全空间代价。
- 增加上一条路线的弱连续性偏置。
- 搜索与候选路线比较复用统一边代价，避免两处代价排序不一致。

验证结果：

- 语义 A*、安全空间代价、witness polyline 风险和路线选择测试通过。

<a id="chg-0001"></a>
## CHG-0001 三层 goal 命名与 35 m 滚动规划范围

- 记录时间：2026-08-26
- 状态：代码与自动化测试完成
- 测试记录：[TEST_REPORT_2026-08-26 / CHG-0001](TEST_REPORT_2026-08-26.md#chg-0001)

修改内容：

- 将局部 graph 半径从 `50 m` 调整为 `35 m`。
- 将目标拆分为 `mission_goal`、`frontier_goal` 和 `local_goal`。
- frontier 使用 `35 m - 3.5 m = 31.5 m` 的滚动储备阈值。
- local goal 保持约 `10 m` 前视，并按规划 tick 持续发布。

验证结果：

- 三层目标命名、局部搜索半径和 frontier 滚动相关测试通过。
