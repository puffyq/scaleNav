# Test Report 2026-08-26

## 1. 测试对象

- 工作区：`/mnt/code/lab/yopo/OpenSeek/scalenav_ws`
- 最新真实仿真会话：`log_scalenav/session_20260826_192324_326`
- 最新会话 ROS 日志：`/home/puffy/.ros/log/epic_graph_node_982042_1787743404275.log`
- 最新完成往返会话：`log_scalenav/session_20260826_184033_343`
- 完整往返 ROS 日志：`/home/puffy/.ros/log/epic_graph_node_939730_1787740833415.log`
- 覆盖时间：2026-08-26 18:40:33–19:24:30

<a id="chg-0016"></a>
### CHG-0016 路线延伸稳定性与 witness 净空代价

- 修复后日志复核：最新 `/home/puffy/.ros/log/epic_graph_node_1028017_1787747454078.log` 中，CHG-0015 已使多数样本保持 `route_blocked=0` 并恢复 incumbent；但末段仍在 `route_blocked=0 / incumbent=RECOVERED / horizon_ready=0` 时接受新路线，terminal 从 `(7.75,63.85)` 改为 `(4.45,86.95)`。直接代码原因是执行储备不足仍属于 `hard_switch`。
- 路线宽松度复核：同一会话多条胜出路线的日志 `clearance=0.00`，而该字段只由边端点 Bubble 半径计算，不能表示 witness 中间净空；因此不能据此认为规划器已比较并选择更宽松路线。
- 实现：执行储备不足只触发搜索，候选仍须通过正常 loss 滞回或兼容延伸检查。仅 blocked、accepted route 缺失、incumbent 恢复失败和最终目标窗口保留硬切换语义。
- 实现：边建立/修复时缓存 edge witness 的保守最小净空，A* clearance loss 读取该缓存并按 witness 实际长度计权；没有在 A* 热循环中增加点云查询。update 日志新增 `switch_reason` 以记录实际接受原因。
- 自动化验证：Release 编译通过；`colcon test --packages-select scalenav_graph_ros2` 通过，`colcon test-result --verbose` 汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。
- 测试范围：未修改 `FUNCTION_TEST_CASES.md` 或现有测试源码。真实左右廊的切换率、clearance loss 与结构化 `path_min_m` 对齐仍待下一次仿真日志验收。

<a id="chg-0015"></a>
### CHG-0015 前向 witness 阻塞检查与局部目标顺序修复

- 代码修改：`current_route_blocked` 不再检查完整 `last_witness_path_`，而是先截取车辆当前位置之后的 forward witness 后缀；update 日志新增 `route_probe_points`。
- 代码修改：删除发布阶段对已按拓扑顺序展开 witness 的第二次全局最近线段投影，避免回弯或平行走廊使 local goal 跳回旧段。local goal 仍沿 witness 弧长前视，未改变 A* 代价和语义风险规则。
- 预期行为：已通过的旧前缀发生地图变化时，不应再使前方可执行路线整体 `route_blocked`；局部目标应保持当前 witness 的前向顺序。
- Release 编译：通过。
- 自动化回归：`colcon test --packages-select scalenav_graph_ros2` 通过；`colcon test-result --verbose` 汇总 `76 tests, 0 errors, 0 failures, 0 skipped`。
- 测试范围：未修改 `FUNCTION_TEST_CASES.md` 或现有测试源码；本次尚未执行真实仿真复测，因此左右廊切换次数、`route_probe_points` 收缩和 local goal 后向指向仍待日志验收。

### CHG-0014 路径本身无进展的 witness 拒绝

- 根因复核：`19:45:00` 会话的 `RHC_DISPLAY / incumbent=RECOVERED / horizon_ready=1` 组合只证明旧 witness 几何上可复用；`path_99` 起点附近的回环/回退没有被现有连续性检查识别，导致 local goal 在约 `y=14..16 m` 局部区域反复更新。更直接的漏洞是旧代码在 `accepted_witness_usable && route_has_execution_horizon` 时可直接 `found=1`，本帧 `path_nodes` 为空或 odom 无连通边也会继续发布 `last_witness_path_`。
- 日志口径：该会话可见的 `path_nodes=10..13` 是拓扑节点数；`witness_points=26..58` 是由每条拓扑边的 `paths_` 展开后的几何采样点；`semantic_path_nodes=0/1` 只表示语义节点数。因此“无头路”指的是 remembered witness 可脱离本帧拓扑搜索单独被接受的代码路径，不等于这些具体帧的 `path_nodes` 字段均为零。
- 实现：删除仅凭 remembered witness 置 `found=1` 的分支；必须由当前图搜索恢复至少两个拓扑节点，且 odom 节点存在连通边。A* 返回空/单节点结果会被拒绝，并在发布前再次执行同一最终检查，避免“无节点绑定的几何折线”继续驱动 RHC。本批次不新增路径形状或任务轴阈值。
- 自动化验证：Release 编译通过；`colcon test --packages-select scalenav_graph_ros2` 为 `6/6` CTest、`76 tests, 0 errors, 0 failures, 0 skipped`，`test_rebuild_crash` 的 1 项按条件跳过。未修改 `FUNCTION_TEST_CASES.md` 或测试源码。
- 待真实验证：复跑原 `19:45:00` 场景，确认当前图无法恢复节点路径时不再继续发布旧 witness local goal，并确认正常拓扑路径仍可执行。

## 2. 最新日志证据

### 19:23:24 CHG-0013 当前语义代际与深度断流复测

- 本次成功到达 `(0,140)` 并切换回程目标。去程从 `y≈30 m` 后主要沿右侧 `x≈7.75..14.35 m` 前进，说明过滤历史虚拟风险后，右侧可行拓扑能够被选中并执行。
- 回程没有直接沿去程的右侧 witness 返回，首要原因是目标翻转时 `onGoal()` 主动清空 `last_witness_path_`、`last_path_nodes_`、route terminal 和 `corridor_hint_route_`；只复用了全局 graph/edges/semantic memory，没有复用原 witness，也没有先构造其反向路线。在同一静态图和代价快照下，反向右路理论上应先作为候选比较；当前实现让它在回程第一轮 `incumbent=NOT_ELIGIBLE` 时完全缺席，重新搜索先选了左侧 terminal `(-2.15,110.05)`，随后沿左廊到 `(-4.56,94.38)`。
- 左廊末段 `route_blocked=1` 后，规划器才重新搜索并恢复出右侧 terminal `(11.05,67.15)`，发布 local goal 约为 `(5.34,93.05)`。最终 `/epic/path` 有 `22` 个 witness 点，从车辆附近先向右横切，再沿右侧向任务原点前进；没有继续锁定左侧阻塞 terminal。
- 新增的工作集日志确认代际过滤生效：末段 `persistent_semantic_records=1105`、`global_virtual_semantic_nodes=1058`，但 `local_inactive_virtual_semantic_nodes=459`、`astar_inactive_virtual_semantic_nodes=498`，实际 `astar_semantic_nodes=34`。持久化存储没有删除，A* 也没有再加载此前约 `435` 个混合历史虚拟点。
- 语义工作集随当前帧保持在约 `13..47` 个，历史虚拟点跳过量逐步增至约 `459/498`；末段 `astar_semantic_checks=2814`。这验证了“存储规模”和“实际运算窗口”已分开。
- 飞机停止的直接原因不是规划器没有右路。`1787743463.096920154` 只是每 `2 s` 节流输出的 EPIC cloud timing，并非最后一帧；结构化日志的最后一帧 depth/pointcloud stamp 为 `1787743464.870896333`。在线控制器的 depth watchdog 为 `0.5 s`，50 Hz 控制回调约在 `1787743465.371` 起停止发布；独立的 `2 s` 状态定时器在 `1787743465.917690983` 才打印 `depth stream timed out; control output stopped`。renderer 随后在 `1787743467.000404360` 报 `AirSim render RPC failed: timed out`，并继续出现 RPC waiting/pose timeout。飞机最终停在 `(-4.63,93.82)`。
- 停止时右切路径仍有效：planner 为 `incumbent=RECOVERED`、`route_blocked=0`、`horizon_ready=1`，路径 `22` 点，结构化 clearance 为 `vehicle_m≈1.795 m`、`path_min_m=0.723 m`。因此不能把停机归因于右切 witness 碰撞或 A* 搜索失败。
- 性能仍有显著长尾：`1787743461.407` 的 update 为 `1261 ms`；`1787743463.810` 的后台更新为 `2292 ms`，其中 `region_select=1884 ms`、`skeleton=304 ms`。它们与点云停止时间接近，可能造成资源竞争，但当前日志没有 AirSim 服务端耗时或系统负载证据，只能列为 RPC 断流的潜在诱因，不能当作已证实根因。
- 本次不计完整往返验收。下一次应先保证 RGB-D RPC 连续，再重复同一任务，确认右切可以实际执行并回到 `(0,0)`。

#### 回程规划与耗时分解

- `19:23:24` 目标翻转日志为 `REUSE_EXISTING_GRAPH`，但路线记忆已清空；因此“没走原本的路”发生在搜索入口之前，不是 A* 算出右路后被执行器改成左路。
- “右路按节点代价应更优”目前不能仅由胜出路线日志证实或证伪：实现比较的是有向 edge witness 几何代价、当前语义帧风险、净空和 progress/direction/FOV/smoothness terminal loss；回程时语义快照和局部拓扑也已变化。必须把去程 witness 反向生成一个经过当前地图/语义重新验证的候选，并打印它与 A* 最优候选的完整 loss，才能确认是否存在代价实现错误。
- 回程第一次有效 update（`1787743456.412`）重新搜索左侧 terminal：`odom_connect=0.000 ms`、`astar=88.396 ms`、`publish=19.227 ms`、`total=149.287 ms`；A* 展开 `273` 个节点、评估 `1713` 条边、实际语义检查 `10146` 次，`candidate_accepted=0` 是恢复旧候选失败后的新路线提交。
- 左廊继续前进时 update（`1787743458.466`）为 `odom_connect=12.781 ms`、`astar=15.219 ms`、`publish=17.855 ms`、`total=277.100 ms`；此时 `route_blocked=1`，重新提交左侧 `(-5.45,100.15)` terminal。
- 到 `y≈102.9` 的下一次 update（`1787743463.487`）为 `odom_connect=6.870 ms`、`astar=14.380 ms`、`publish=14.535 ms`、`total=306.153 ms`；仍是左侧 `(-5.45,73.75)`，随后才在下一次搜索恢复右切路线。
- 右切路线发布后的 update（`1787743467.518`）为 `odom_connect=0.000 ms`、`astar=39.181 ms`、`publish=23.451 ms`、`total=87.688 ms`；`incumbent=RECOVERED`、`route_blocked=0`、`horizon_ready=1`，说明此时已经正常沿右切 witness 运行。
- 与在线规划同时发生的后台图更新长尾为 `1787743463.810` 的 `2291.605 ms`，其中 `region_select=1884.077 ms`、`skeleton=304.272 ms`；另有 `1787743461.407` 的 update 总耗时 `1261.051 ms`。这些是计算耗时，不等同于“回程没走原路”的原因；它们可能增加调度压力，但当前日志不足以证明导致 AirSim RPC 超时。
- 传感器停机是后续独立事件：最后结构化 depth/pointcloud stamp 为 `1787743464.870896333`，控制器 `0.5 s` watchdog 约在 `3465.371` 停止控制，`2 s` 状态定时器到 `3465.918` 才打印错误；之后 renderer 在 `3467.000` 报 RPC timeout。

### 18:57:56 CHG-0012 修复后左廊复测

- 会话 odom 覆盖 `68.31 s`，x/y 范围为 `-28.88..10.27 m` / `0..140.03 m`。去程到达 `(0,140)` 并在 18:58:47 将 mission goal 切换为 `(0,0)`；结束时仍在回程，位置 `(-5.37,102.81,1.59)`、速度约 `4.48 m/s`，随后收到 `SIGINT/SIGTERM`。因此本次不是 y≈80 卡停，但只完成去程，不能计为完整往返。
- CHG-0012 的关键行为已由真实日志确认：vehicle=`(-3.50,77.77)` 时，accepted witness 先到 `(-5.45,80.35)`，再沿 `y≈80.35` 向左横移；local goal=`(-12.23,80.35)`，与 vehicle 投影后约 `10 m` 折线弧长点一致。无人机随后实际横移到左侧，未再把 local goal 跳到 witness 后段 `y≈90 m`。
- 但路线最优性未解决。同期 graph 快照在 `75<=y<=110 m` 内有右侧平面拓扑节点 `55` 个、左侧 `24` 个，右侧不是无路。
- 原始语义实际支持“左侧风险更高”：转弯前 `semantic_227` 的五个水平分区峰值约为 `0.631/0.506/0.247/0.202/0.188`，左侧两列显著高于右侧；结合该帧 vehicle=`(9.65,70.28)`、yaw≈`115.1 deg`，图像左侧固定深度射线投影到世界坐标负 x，即左侧走廊。左右投影公式与普通深度使用的 optical→FLU 约定一致，没有发现简单的列翻转。
- 与当前帧相反，同期持久化 graph marker 在该区域显示 `14` 个 `SEM-RISK` 全位于右侧、左侧为 `0`。这不是当前风险分布，而是多帧历史虚拟端点累积后的状态；此前按 marker 数量解释当前语义方向是错误的。
- 该次 A* 查询加载 `435` 个局部语义节点，一帧新语义只有 `15` 个。`semanticNodes()` 只按 45 m 空间半径取历史节点、不检查观测年龄；`edgeSemanticRisk()` 对每条边附近的全部节点取最大值，没有当前帧/历史权重或衰减。固定 30 m 射线端点只是推测位置，却作为持久世界风险锚点继续参与后续搜索。因此即使当前左侧更危险，旧的右侧风险仍可压制右路；这是现有证据最强的左右选择根因。
- 参数为 `goal_path_cost_weight=0.2`、`semantic_cost_weight=2.0`。以 risk=`0.35` 为例，语义项为 `2 * -log(0.65)≈0.86/m`，是基础几何项 `0.2/m` 的约 `4.3` 倍；一个未衰减的历史高风险点就可能抵消右路的距离优势。当前日志只保存胜出路线，缺少被拒右侧候选及当前/历史语义 loss 分解，尚不能给出两条候选的精确总分差。
- safety loss 同时存在口径缺失：A* 的 `edgeClearancePenalty()` 只看边两端 Bubble 半径，没有检查 witness 折线内部净空。所选左路在 update 中记录 clearance loss=`0.00`，同一时刻结构化 clearance 却给出 `path_min=0.144 m`，全会话最低为 `0.035 m`。这会让贴障的大幅绕行路线在候选排序中显得比实际更安全。
- 31 条 update 中 `route_blocked=11`、`astar_timed_out=0`。A* 平均/P95/最大值为 `30.4/69.3/104.7 ms`，update 为 `134.4/273.1/330.4 ms`；`astar_semantic_checks` 平均/P95/最大值为 `105061/290357/327224`。空间索引仍生效，路线选择问题不能归因于 A* 超时。
- 结论分层：local-goal 跳段根因已修复并通过真实左廊验证；未选右侧的首要问题是当前帧与持久化虚拟风险没有分层，历史右侧风险在高语义权重和 max 聚合下可覆盖当前“左侧更危险”的证据。同时 witness 内部真实净空没有进入 safety loss。下一步应先限制虚拟语义参与计算的年龄或将当前/历史证据分开，再把 witness 最小净空纳入边代价，并记录左右候选完整 loss 后调权重。

### 18:43 后最新三次会话：左侧走廊失败复现

- 最新三次会话目标均为 `(0,140,1.6)`，不是此前短会话中误发的原点目标。`18:44:51` 会话运行 `28.11 s`、路径 `92.75 m`，最终 `(-5.61,79.64,1.60)`；`18:43:56` 会话运行 `32.70 s`、路径 `93.44 m`，最终 `(-4.14,79.85,1.59)`；`18:43:11` 会话运行 `27.91 s`、路径 `35.09 m`，最终 `(-5.16,30.82,1.61)`。三次均由 `SIGINT/SIGTERM` 结束，未到达任务终点。
- 两个有效失败样本在 `40<=y<85 m` 的平均 x 分别为 `-4.44 m` 和 `-5.39 m`，均进入左侧走廊；18:40 成功往返的同一区间去程平均 x 为 `+11.86 m`，走右侧走廊并顺利通过 `y≈80 m`。
- 最新两次分别有 `7/11`、`5/11` 条 update 为 `route_blocked=1`，terminal 均 EXTEND 到左侧 `(-5.45,110.05)`，结束前 `vehicle_to_terminal=30.50/30.32 m`。但 terminal/走廊选择不是飞机钉在 y≈80 的直接原因，实际执行点与完整 witness 脱节才是根因。
- 最新会话 `path_50` 至 `path_56` 的 accepted witness 从约 `(-7.9,79.6)` 先沿 `y≈80.35` 横移到 `x=-21.95`，再经 `y≈83.65` 折回并前进到 terminal。这是一条约 `55 m` 的大幅侧向绕行路线，其前 10 m 弧长点约为 `(-17.9,80.35)`。
- `selectNextGoal()` 在 `have_goal_` 时走 mission-guided 分支，不使用已经计算出的 `nearest_progress + lookahead_m`，而寻找 mission 方向投影达到 `10 m` 的后续 witness 点。该逻辑从上述路线直接选出 `local_goal≈(-8.75,90 m)`，跳过前方必须执行的整段横移/折返。
- YOPO 实际收到的正是该跳段 local goal，控制轨迹在 `y≈80 m` 把前向速度降到接近 0 并横移；EPIC 随后把未按 witness 执行的旧路线判为 blocked，再生成类似绕行并再次被 local-goal 投影跳过，形成稳定闭环。最低 `path_min_m` 分别为 `0.036 m` 和 `0.051 m`。
- 因此直接根因是 local goal 选取违反 witness 路径顺序，不是 mission goal 丢失、30 m 语义点未生成、A* 超时，也不能仅归因于 blocked 时没有搜索右侧 terminal。corridor hint 可能影响选左/右，但不是这次卡死的必要条件。
- 三个短会话 `astar_timed_out=0`。两个 y≈80 失败会话 update 平均/P95 分别为 `119.1/185.8 ms` 和 `147.8/325.6 ms`，A* 平均/P95 分别为 `24.1/49.3 ms` 和 `31.0/69.9 ms`；性能仍有长尾，但不能解释为何稳定选择左廊。
- CHG-0012 已删除 mission-axis 跳段逻辑，local goal 改为从 vehicle 在 forward witness 上的投影开始，严格按路径弧长选择前视点。以上失败会话保留为修复前基线；18:57:56 真实左廊复测已确认跳段消失，但暴露出独立的左右路线 loss 排序问题。

### 18:40 CHG-0011 优化后完整往返

- 会话完成 `(0,0) -> (0,140) -> (0,0)`，最终 odom 约为 `(0,0,1.60)`。记录覆盖 `77.79 s`，飞行路径 `315.69 m`，最大速度 `6.00 m/s`。
- 36 条 update 的 `astar_semantic_checks` P50/P95/最大值为 `93040/273263/298414`；全程实际精确检查 `3888153` 次，而逐边扫描完整 A* 语义池的理论值为 `23372933` 次，累计工作量减少 `6.01` 倍，确认空间索引在线生效。
- 18:11 优化前基线的语义检查 P95/最大值为 `1627093/1788796`；修改后分别下降 `83.2%/83.3%`。A* P50/P95/最大值为 `39.69/70.00/139.22 ms`，update 为 `107.99/267.35/365.61 ms`，`astar_timed_out=0`。
- 日志出现 1 次 `no reachable real Bubble topology` 和 8 次终点附近的 `rejected discontinuous witness path`；任务仍完成往返，但 witness 退化检查仍未满足 `IT-FLT-004`。

### 18:11 CHG-0011 优化前基线

- 完成 `(0,0) -> (0,140) -> (0,0)` 往返任务，最终 odom 约为 `(0,0,1.60)`，没有 emergency stop。记录覆盖 `100.60 s`，飞行路径 `310.10 m`，最大速度 `5.74 m/s`。
- 初始 `vehicle_to_frontier=30.87 m`，低于旧 `31.5 m` 阈值但候选仍被接受。43 条节流样本中找到候选 39 次、接受 31 次；12 次 `RHC_DISPLAY` 保留 incumbent，其中 8 次找到但未接受候选、4 次无需候选，说明候选不会无条件替换当前路线。
- 会话结束时持久化语义记录为 `1586`；全局图为 `2182` 节点、`4078` 边，其中语义节点 `1577`，持久化记录与当前全局图相差 9 条。当前 35 m 拓扑窗口为 `507` 节点，占全局图 `23.2%`；43 m（35 m 搜索半径加 8 m 语义影响范围）语义查询池为 `472` 节点，占全局语义图 `29.9%`。该查询池不同于障碍点云的 40 m 历史窗口；持久化、全局图和各局部窗口已能在同一条日志中明确区分。
- 最后一条样本中 A* 查询语义点 `499`、展开节点 `303`、评估边 `1798`、语义候选检查 `897202`。A* 查询以搜索起点为圆心且半径为 45 m（35 m 搜索范围加 10 m 边风险影响范围），诊断窗口以实时 vehicle position 为圆心且半径为 43 m（35 m 加 8 m 路线风险影响范围），因此 `astar_semantic_nodes` 与 `local_semantic_nodes` 允许不同。
- 全程局部图节点 P95/最大值为 `661/678`，局部语义节点为 `607/639`，A* 展开节点为 `416/433`，语义候选检查为 `1627093/1788796`。43 条样本中语义检查数均严格等于“评估边数 × A* 查询语义点数”，确认修改前每条边都会扫描整个局部语义池；该会话作为 CHG-0011 的优化前基线。
- 回访会显著放大局部工作集：去程 `(10.16,58.72)` 与回程 `(11.99,62.29)` 相距约 `4.0 m`，持久化记录 `338 -> 1203`、全局节点 `558 -> 1706`、局部图 `419 -> 659`、局部语义 `326 -> 619`、A* 展开 `240 -> 397`、语义检查 `499328 -> 1625456`。持久化记录并非全部参与单次搜索，但进入当前位置窗口的历史节点会实际增加搜索开销。
- update 平均 `208.16 ms`、P95 `522.51 ms`、最大 `634.39 ms`，30/43 超过 `100 ms`、17/43 超过 `200 ms`；A* 平均 `46.43 ms`、P95 `78.97 ms`、最大 `94.17 ms`，且 `astar_timed_out=0`。扣除 A*、publish 和 odom_connect 后，未分项时间平均 `127.06 ms`、P95 `424.42 ms`、最大 `505.26 ms`，所以总体超周期不等于 A* 超时。
- 后台 rebuild 共 41 条，平均 `262.50 ms`、P95 `524.58 ms`、最大 `587.84 ms`；其中 `region_select` 平均 `128.53 ms`、最大 `476.47 ms`，`skeleton` 平均 `62.71 ms`、最大 `262.34 ms`。点云 map update 平均 `51.67 ms`、最大 `82.93 ms`。update 的未分项区间包含等待 topology mutex、语义内存更新、路线 remap/metrics 和发布后风险复核，仍需加分项计时才能精确归因。
- 共出现 17 条 `EPIC rejected discontinuous witness path`：远端终点静止 7 条、原点静止 9 条、回程途中 1 条（横向误差 `1.59 m`）；另有 3 次 odom 无法连接真实 Bubble topology、2 帧点云因 odom 时间差超限被丢弃。任务完成但仍不满足 `IT-FLT-004`。
- 177 条 clearance 记录中 `vehicle_m` 最低 `0.150 m`，176 条有效路径记录中 `path_min_m` 最低 `0.029 m`。会话没有 emergency stop，但该余量明显偏低；需要对齐对应点云、位姿和 accepted witness 逐帧确认是端点/地图表面口径还是实际近碰风险。
- 结构化日志包含 181 帧语义图；语义输出持续为 `5x3`、`15 points`、`virtual_depth=30.00 m`。最终全局语义节点中 `95.4%` 为虚拟语义节点，这也是局部语义扫描规模较大的直接来源。

### 较早会话已确认

- 无人机确实移动：odom 从约 `(0,0,1.60)` 到 `(-3.99,79.60,1.59)`。
- EPIC 持续输出前方 `frontier_goal=(-5.45,113.35)` 和 `local_goal≈(-8.75,89.8–90.1)`；`uav_sim` 没有 emergency-stop ERROR。
- 末段 `/epic/clearance.vehicle_m` 约为 `2.0–2.4 m`，不再出现上一会话约 `29 m` 的近障碍漏检，说明 `40 m` 严格滑动窗口修复有效。
- 普通 graph 最后有 `260` 个节点；无人机前方 35 m 内有 `46` 个节点，距无人机约 `27.1/30.4/33.7 m` 均存在普通节点。
- 修改前的远场语义层仍错误：日志只生成 `range=21–22 m` 的 speculative 节点，末段语义帧出现 `pseudo=0` 后该图层最终为 0。

### 较早语义根因

旧实现把“语义风险 patch”错误地绑定到真实 depth：只有 depth 达到 `20 m` 裁剪值时才生成远场节点，再经过 `distance-2` 限制后实际只能位于 `21–22 m`。真实 depth 小于裁剪值时不生成节点，已有节点还会在 3 秒后删除。

现已改为每个有效风险 patch 沿相机射线使用固定 `30 m` 虚拟深度生成语义点，完全不读取 measured depth。语义点与普通点使用同一个 `TopoNode`，连接仍执行点云碰撞检查。17:40 会话已确认该链路在完整往返任务中持续运行。

### 当前已通过自动化验证的修改

- CHG-0013：未验证虚拟语义按当前成功应用帧 stamp 组成规划代际；历史点继续持久化，`Verified` 节点继续长期参与。没有新增 `semantic_virtual_planning_max_age_ms` 滑动窗口参数。
- CHG-0012：local goal 严格按 forward witness 弧长选择，不再按 mission-axis 投影跳过横向或短时后退绕行段。
- CHG-0011：A* 搜索对局部语义池建立一次空间哈希，每条 witness 边只精确检查语义影响半径内的邻域候选；风险公式和路线排序规则不变。18:40 完整往返已确认在线实际检查量累计减少 `6.01` 倍。
- CHG-0010：update 日志明确区分持久化、全局发布图、局部规划窗口和 A* 实际搜索工作量，并记录展开、边计算、语义扫描、候选 terminal 与超时状态。
- CHG-0009：frontier 候选改为物理可行性硬约束加统一软 loss；`31.5 m`、任务方向和水平 FOV 不再是 terminal 硬门槛；语义三维位置保留，单行/多行、FOV边缘和疑似地面响应改为置信度因素。
- CHG-0006：所有 patch 使用固定 `30 m` optical depth，语义点是普通 `Geometric TopoNode`，`Unknown` 仅表示真实几何尚未验证。
- CHG-0007：全局 graph 保留不变，A*、路线风险和发布统计只查询当前局部语义节点，重复观测按 `2.5 m` 合并。
- CHG-0008：terminal 按 persistent id 恢复，规划储备不足与 incumbent 失效分离；可执行的 accepted witness 在临时恢复失败时继续发布。
- CHG-0006/0007/0008/0009/0010/0011/0012/0013 已完成编译和自动化测试。CHG-0013 已确认右侧去程和 blocked 后右切均可生成，但回程因 AirSim 深度 RPC 断流停止。`IT-FLT-001` 要求 3 次，且 witness 连续性、路线恢复和传感器连续性仍需复核，因此不能声明完整仿真验收完成。

## 3. 自动化测试

### 3.1 变更批次关联

<a id="chg-0013"></a>
- [CHG-0013](CHANGELOG.md#chg-0013)：规划入口统一传递当前成功应用语义帧 stamp，`Unknown` 历史虚拟点退出计算、`Verified` 历史点保留；未修改仓库测试源码。临时 smoke test `5/5`、Release 编译、全包 CTest `6/6` 和 `epic_online_simulation 4/4` 通过。19:23 会话确认持久化虚拟点 `1058` 与 A* 实际语义池 `34` 已分离；完整回程因 RGB-D RPC 断流未完成。

<a id="chg-0012"></a>
- [CHG-0012](CHANGELOG.md#chg-0012)：新增 `routeLookaheadPoint()` 并由 `selectNextGoal()` 统一调用；未修改仓库测试源码，额外临时 smoke test 覆盖横向绕行、短时后退/U 形和直线中段投影。Release 编译、全包 CTest `6/6` 和真实左廊顺序复测通过，对应 `MT-M5-001`、`MT-M5-004`、`TC-M6-006`、`IT-FLT-007`；完整往返和路线最优性仍待验收。

<a id="chg-0011"></a>
- [CHG-0011](CHANGELOG.md#chg-0011)：搜索期空间哈希按 witness 扩展包围盒预筛语义邻域，再沿用原有精确风险计算。增量编译和全包 CTest `6/6` 通过，未修改测试源码；18:40 完整往返中累计候选检查减少 `6.01` 倍，P95 从优化前 `1627093` 降至 `273263`。覆盖现有 `TC-M4-010` 相关回归；`MT-M4-004` 的检查规模通过，规划周期长尾仍待优化。

<a id="chg-0010"></a>
- [CHG-0010](CHANGELOG.md#chg-0010)：搜索接口增加只读统计参数，update 日志增加持久化、全局、局部窗口和 A* 工作量字段。未修改测试源码；增量编译和全包 CTest `6/6` 通过，18:11 完整往返确认字段有效。

<a id="chg-0009"></a>
- [CHG-0009](CHANGELOG.md#chg-0009)：现有 frontier、语义代价、路线记忆和在线集成回归全部通过。17:40 和 18:11 两次真实往返完成，且低于 `31.5 m` 的初始 frontier 均被接受。地面误导抑制和单行/多行置信度仍缺少日志字段与直接场景断言。

<a id="chg-0008"></a>
- [CHG-0008](CHANGELOG.md#chg-0008)：`test_route_memory` 覆盖 compatible frontier extension 与近车横向换道保护；`test_epic_integration` 覆盖在线发布契约。自动化测试通过，真实长航线待验收。

<a id="chg-0007"></a>
- [CHG-0007](CHANGELOG.md#chg-0007)：`test_topo_semantic` 覆盖局部语义查询排除远距离历史节点；`test_epic_integration` 覆盖节点发布接口。自动化测试通过，语义内存增长率和 A* 时延待真实长航线复测。

<a id="chg-0006"></a>
- [CHG-0006](CHANGELOG.md#chg-0006)：`test_topo_semantic` 覆盖固定 optical Z=`30 m`、边缘点三维距离和 measured-depth 独立性。自动化测试通过，普通 graph 中的实际节点分布待仿真验收。

<a id="chg-0005"></a>
- [CHG-0005](CHANGELOG.md#chg-0005)：`test_topo_semantic` 和 `epic_online_simulation` 覆盖初版固定虚拟深度语义点；投影定义随后由 CHG-0006 修正。保留该记录用于追踪历史行为，不作为当前设计验收依据。

<a id="chg-0004"></a>
- [CHG-0004](CHANGELOG.md#chg-0004)：`test_lidar_map` 覆盖 `40 m` 严格滑窗和容量满时近点优先；15:17 会话的 clearance 日志用于场景复核。自动化与该日志复核通过，完整长航线仍待验收。

<a id="chg-0003"></a>
- [CHG-0003](CHANGELOG.md#chg-0003)：`test_route_memory` 覆盖风险/代价切换滞回、整条 witness 投影和前向路线复用；日志用于复核不连续拒绝。自动化测试通过，稳态高风险和 graph rebuild 下的切换比例待长航线验收。

<a id="chg-0002"></a>
- [CHG-0002](CHANGELOG.md#chg-0002)：`test_topo_semantic` 与路线测试覆盖语义边代价、安全空间代价、witness polyline 风险及统一路线排序。自动化测试通过。

<a id="chg-0001"></a>
- [CHG-0001](CHANGELOG.md#chg-0001)：`test_route_memory` 和 `test_epic_integration` 覆盖三层目标、`35 m` 搜索范围、frontier 储备与 local-goal 发布约束。自动化测试通过。

### 3.2 全量回归

命令：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/code/lab/yopo/OpenSeek/scalenav_ws
colcon build --packages-select scalenav_graph_ros2
colcon test --packages-select scalenav_graph_ros2
colcon test-result --verbose
```

结果：

- `test_route_memory`：原有 12 项通过，包括 compatible frontier extension 和近车横向换道保护；未修改测试源码。CHG-0012 临时 smoke test `3/3` 通过，覆盖横向/后退 witness 的弧长 local-goal 前视和直线中段投影。
- `test_lidar_map`：9 项通过，包括严格滑动窗口和近点优先容量测试。
- `scalenav_log/test_log_storage`：1 项通过，确认日志 session 不因容量滚动且旧 session 不被自动删除。
- `test_topo_semantic`：41 项通过，包括固定 30 m optical-depth 投影、不依赖 measured depth、局部语义查询，以及低于前向储备目标的径向/不完整 frontier 综合代价选择。
- `test_epic_integration`：5 项通过。
- `test_rebuild_crash`：1 项按测试条件跳过，无失败。
- `epic_online_simulation`：4/4 检查通过。

总计：`6/6` CTest 通过；限定 `scalenav_graph_ros2/test_results` 的汇总为 `68 tests, 0 errors, 0 failures, 0 skipped`；`epic_online_simulation` 的 `4/4` 检查通过。控制台中的外部日志 rebuild 场景按条件跳过，但其 JUnit 结果和 CTest 目标均正常通过。

CHG-0013 另有位于忽略 build 目录的临时 smoke test，覆盖当前虚拟点保留、上一代虚拟点退出、`Verified` 历史点保留、语义流超龄时虚拟点退出和默认接口兼容，`5/5` 条件通过；没有修改仓库测试用例。

## 4. 待补测试

- 为 update 增加 topology mutex 等待、语义内存更新、路线 remap/metrics 和发布后风险复核分项计时，解释 P95 `424.42 ms` 的未分项时间。
- 为 frontier 候选增加 geometry/semantic/clearance/progress/direction/FOV/smoothness loss 分解，语义部分再区分当前帧和历史节点，直接比较同一 tick 的左右候选；当前日志不能精确复算被拒右路。
- 将 edge witness 内部最小净空纳入 safety loss，避免端点 Bubble 安全但中间折线贴障的路线得到 `clearance=0` 惩罚。
- 当前代虚拟语义工作集已由 CHG-0013 实现；继续校准地面/视角置信度和 `0.2:2.0` 的几何/语义权重，并用候选 loss 分解验证，而不是增加固定毫秒级历史虚拟点窗口。
- 诊断并隔离 AirSim renderer RGB-D RPC 超时；记录 AirSim 服务端 RPC 耗时、CPU/GPU/内存压力以及 depth 发布间隔，判断 `2292 ms` 后台图更新与断流是否存在因果关系。
- 完成不中断的 `(0,0) -> (0,140) -> (0,0)` 往返；19:23 会话已完成右侧去程并在回程生成右切路径，但控制因深度断流停止。
- 修复终点零长度/退化 witness 的 16 次伪不连续拒绝以及回程途中 1 次 `1.59 m` 不连续拒绝，并重新执行 `IT-FLT-004`。
- 对齐最低 `vehicle_m=0.150 m`、`path_min_m=0.029 m` 时刻的点云、odom、graph 和 witness，确认是否存在真实安全余量违规。
- 用修复后的 `map_history_radius_m=40` 重新跑完整 AirSim/Colosseum 长航线。
- 断言每个有效语义帧都有 `points>0 virtual_depth=30.00 m`，且 `EPIC semantic graph` 的距离为约 `30 m`，不再出现 `pseudo=0` 导致语义点清空。
- 断言 `epic_semantic_points` 位于无人机前方 30 m 附近，并且 connected 数量足以影响 35 m 局部 A*。
- 在墙面进入 40 m 窗口后，断言 `/epic/clearance.path_min_m` 与原始深度点云的最近距离同量级。
- 断言 route blocked 后不会继续发布同一阻塞 witness 的 local goal。
- 记录 incumbent/candidate 的 terminal、risk、objective 和切换原因，确认 `candidate_accepted=1` 不再成为每 tick 默认结果。
- 验收 local goal 连续发布、frontier 每消耗约 `3.5 m` 延伸一次，以及 300–500 ms 保底上限。
- 增加 CHG-0009 的直接自动化断言：低于 `31.5 m` 的安全候选可击败远端高风险候选；单行/多行与 FOV 只改变 loss；疑似地面响应不会成为路线硬约束。
- 记录 CHG-0009 各项 frontier loss 分解和搜索耗时，验证完整候选排序不会突破在线规划预算。

## 5. 仿真验收指标

- 无人机在墙前不得生成穿过墙面的 accepted witness。
- 新墙面被观测后，规划安全空间应在不超过 2 个点云/规划周期内下降。
- 无碰撞且无人机仍在 accepted witness 走廊内时，不能因 witness 首点落后而丢失 local goal。
- 语义风险升高可触发候选搜索，但无明显改善时不得横向来回切换。
- 任务完成前必须持续保留可执行的 local goal 前视窗口；`frontier_goal` 允许必要的侧向或短期后向选择，但无硬阻塞时必须保持长期任务进度并最终收敛到 mission goal。
- 高风险 patch 生成的 30 m 语义点应在障碍进入真实 depth 范围前提高对应走廊代价；未通过碰撞检查的语义连接不得产生 accepted witness。
