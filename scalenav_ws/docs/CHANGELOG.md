# ScaleNav / EPIC Changelog

本文件按修改批次记录，不按日期聚合。每次代码更新新增一个独立变更编号；后续补充验证结果时更新对应记录，不把不同修改合并到同一天的章节中。

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
- 未修改仓库测试源码；额外临时 smoke test 覆盖“先横移再前进”“先后退再绕行”“车辆从直线 witness 中段投影”，`3/3` 通过。
- 最终工作树状态下全包 CTest `6/6` 通过；`test_route_memory` 为原有 `12` 项，限定本包结果汇总为 `68 tests, 0 errors, 0 failures, 0 skipped`；`epic_online_simulation` 为 `4/4`。
- 对应现有 `MT-M5-001`、`MT-M5-004`、`TC-M6-006` 和 `IT-FLT-007` 的 local-goal/witness 顺序与闭环执行要求；未修改 `FUNCTION_TEST_CASES.md`。
- 18:57:56 真实左廊复测中，vehicle 位于 `(-3.50,77.77)` 时 accepted witness 先沿 `y≈80.35` 横移，新 local goal 为 `(-12.23,80.35)`，与从 vehicle 投影起约 `10 m` 的折线弧长点一致。无人机实际执行横移，最远到 `x=-28.88`，随后到达 `(0,140)` 并切换回程目标，确认不再跳到 witness 后段 `y≈90 m`。
- 该会话在回程到 `(-5.37,102.81)`、速度约 `4.48 m/s` 时收到 `SIGINT/SIGTERM`，因此不是卡停，但也不能计为完整往返验收。

后续：

- 本次复测仍没有选择更短的右侧路线。对应 graph 快照在 `75<=y<=110 m` 有右侧平面拓扑节点 `55` 个、左侧 `24` 个，说明不是右侧无图。
- 当前原始语义与持久化风险图发生矛盾：转弯前 `semantic_227` 的五个水平分区峰值约为 `0.631/0.506/0.247/0.202/0.188`，结合当时位姿投影后，高风险明确位于世界坐标左侧；但规划快照中该区域的 `14` 个持久化 `SEM-RISK` 全在右侧、左侧为 `0`。因此不能把持久化 marker 数量当成当前帧语义，也不能据此得出“当前语义选择左路”的结论。
- 该次搜索加载 `435` 个局部语义节点，而每帧最多只插入或更新 `15` 个点。`semanticNodes()` 只做空间半径筛选、不做观测时效筛选，`edgeSemanticRisk()` 又对候选取最大值；绝大多数未被当前帧触碰的历史虚拟点仍参与计算，一个旧的右侧高风险点即可持续抬高整条右路代价。固定 30 m 射线端点是推测位置，却按永久世界锚点进入搜索，这是当前“左侧风险更高但仍选左路”的首要根因候选。
- 当前 `goal_path_cost_weight=0.2`、`semantic_cost_weight=2.0`；risk=`0.35` 时每米语义项约为 `0.86`，是基础几何项 `0.2` 的约 `4.3` 倍。还需确认这些右侧虚拟风险是否为有效语义证据，并重新标定几何/语义权重。
- safety loss 只使用边端点 Bubble 半径，所选左路在 update 中的 clearance loss 被记为 `0.00`，但同一时刻结构化日志的实际 witness `path_min=0.144 m`，全会话最低 `0.035 m`。应把 witness 内部最小净空纳入边代价后再比较左右候选。
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
