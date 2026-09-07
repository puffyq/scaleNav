# TopoGuide: A Plug-and-Play Topological Route Layer for Language-Guided Aerial Navigation

*Anonymous Authors*

> **【本段目的】** 开篇 teaser 图：用一个真实飞行场景一次性展示系统的全部要素（传感器输入、持久拓扑骨架、语义远场观测、GCN 路线选择与飞行轨迹），为后文所有概念提供可视化锚点。

![Fig. 1. TopoGuide's layered route decision in Map2. (a) Synchronized RGB, semantic response, and depth at graph time $t^*$; white depth pixels have no return beyond $20$ m. (b) The light footprint is the complete UE mesh truth for reference. Dots and thin links are persistent skeleton nodes and collision-checked edges; teal is the logged A* topology path, dashed orange is its polynomial witness, and black is the flown trajectory. Red crosses are logged far-field semantic observations; the three labeled goals are published by this graph snapshot. (c) At a recorded route fork before the large block, the GCN selects an early bypass matching the 35-m A* label while the hand-designed ranking selects another column. Dark points and links are the synchronized cloud and online skeleton; the solid pale footprints are privileged context for visualization only.](pics/teaser.pdf)

**Fig. 1.** TopoGuide's layered route decision in Map2. (a) Synchronized RGB, semantic response, and depth at graph time $t^*$; white depth pixels have no return beyond $20$ m. (b) The light footprint is the complete UE mesh truth for reference. Dots and thin links are persistent skeleton nodes and collision-checked edges; teal is the logged A* topology path, dashed orange is its polynomial witness, and black is the flown trajectory. Red crosses are logged far-field semantic observations; the three labeled goals are published by this graph snapshot. (c) At a recorded route fork before the large block, the GCN selects an early bypass matching the 35-m A* label while the hand-designed ranking selects another column. Dark points and links are the synchronized cloud and online skeleton; the solid pale footprints are privileged context for visualization only.

## Abstract

> **【本段目的】** 压缩版论证链：提出问题（局部规划器无路线记忆）→ 给出系统（即插即用拓扑路线层）→ 说明接口（移动局部目标对接 YOPO/EGO/SUPER）→ 汇报关键数字（100%/70%/90% vs 0%，3-D 80%，P50/P95 负载）→ 升华为架构主张（navigator 与 driver 分离）。

Goal-conditioned local planners react quickly to observed geometry but do not retain the route memory needed to circumnavigate obstacles larger than their sensing horizon. TopoGuide is a plug-and-play topological route layer that adds this memory without retraining or changing the core algorithm of the local planner. From depth and odometry, it maintains persistent verified free-space bubbles and collision-checked witness paths while restricting each online search to a local working set. Open-vocabulary image responses add provisional far-field frontiers and continuous language-conditioned cost along verified witnesses. A topology-graph GCN learns long-range frontier-direction selection from the online skeleton, while A* and geometric checks remain authoritative. The accepted witness is exposed as a moving world-frame local goal: YOPO consumes it directly, while thin ROS adapters relay the same interface to EGO-Planner and SUPER. No route-conditioned model or planner retraining is required. In the primary 140-m mission, three TopoGuide-guided executor configurations achieved $100\%$, $70\%$, and $90\%$ success, whereas each corresponding direct-goal control achieved $0\%$. In a separate height-changing line-obstacle mission, the 3-D configuration achieved $80\%$ success and a $2.22\pm0.48$ m vertical span over successful flights. Across the persistent-map profile, the local graph and active A* workloads remain 214/249 and 185/215 nodes at P50/P95. The result is an architectural separation: topology remembers and selects the route, acting as a navigator, while an interchangeable local planner executes only the next goal.

## I. INTRODUCTION

> **【本段目的】** 提出问题：局部规划器快但无路线记忆，单帧观测无法保持对长障碍物的早期左/右绕行承诺；引用地面导航中的拓扑记忆工作证明"保留路线结构"是长程可靠行为的关键。

Flying quickly through an unknown environment forces a robot to reconcile processes that naturally operate at very different rates. A local policy must convert the current depth image and vehicle state into a dynamically feasible command within milliseconds [2], [3], [4], [1], [5], whereas building a reusable map, reasoning over long-horizon alternatives, and running open-vocabulary perception all take orders of magnitude longer [13], [18]. Learned one-stage planners [1], [5] have shown how much a fast reactive layer alone can achieve; yet a single observation cannot preserve an early left–right commitment around a long obstacle, and once the fork disappears from view a memoryless policy is free to undo it. Topological memories in ground navigation have repeatedly demonstrated that retaining route structure, rather than reconstructing it from scratch at every tick, is what converts local reactivity into reliable long-range behavior [14], [15].

> **【本段目的】** 指出第二个机会点：RGB 携带深度看不到的语义证据（电线、玻璃幕墙、远处建筑）；现有开放词汇导航只解决 where to go，未解决飞行速度下的 where not to go，语义慢意见与快速承诺之间的鸿沟正是任务失败之处。

A complementary opportunity has emerged on the perception side. RGB carries evidence that depth does not: a power line, a glass facade, or a distant building can be semantically obvious in pixels while remaining invisible to a depth channel clipped at tens of meters. Open-vocabulary models make this evidence addressable in free language [18], [19], [20], [21], and recent navigation systems already exploit it to choose *where to go* [22], [23]. What they largely do not address is *where not to go* at flight speed: their semantic layers run at a few hertz, hold no executable notion of free space, and hand a reactive executor a goal rather than a route. For an aerial vehicle, the resulting gap between a slow semantic opinion and a fast commitment is precisely where missions are lost.

> **【本段目的】** 给出系统三层总览：第一层持久验证自由空间图（无占据栅格/ESDF）；第二层语言条件路由（语义远场前沿 + GCN 排序，但 A* 与碰撞检查保持权威）；第三层刻意可替换的执行器（标准世界系目标消息，planner 核心不重训）。

TopoGuide closes this gap by making the hierarchy explicit rather than implicit in a monolithic network. Its first layer converts generic depth and odometry into a persistent graph of verified free-space bubbles connected by collision-checked witness polylines, with no occupancy grid or ESDF in the route layer [6], [7]. The retained backbone may grow as new space is explored, but rolling A* and graph updates operate on a fixed-radius local working set. Its second, slower layer performs language-conditioned routing on that graph. Depth-clipped semantic observations become provisional far-field frontiers and a continuous risk field along entire witnesses. A topology-graph GCN ranks discrete frontier directions from the online skeleton, while A*, collision checks, and route memory remain authoritative. The selected route is stored as a world-frame witness and remapped after atomic graph replacement. Its third, fast layer is deliberately replaceable: TopoGuide samples a moving local goal along the accepted witness and publishes a standard world-frame goal message. YOPO [1] consumes that goal directly; EGO-Planner and SUPER consume it through thin topic adapters. Their planner cores are neither retrained nor made graph-aware.

> **【本段目的】** 用 navigator/driver 隐喻概括功能分工：拓扑导航器读图叫路，可互换的局部规划器作为驾驶员执行下一个目标，A* 可达性与碰撞检查裁定哪些指令合法。

Functionally, TopoGuide acts as a topological navigator: it reads the course ahead from a persistent topological map and calls the next branch, while an interchangeable local planner remains the driver that executes the next goal; A* reachability and collision checks determine which calls are legal.

> **【本段目的】** 防御段（预设审稿质疑）：精确划定安全边界——语义前沿只是乐观路线假设，从不被当作未观测空间自由的证明；语义证据是"经过验证备选方案之上的塑形偏好"，而非概率安全保证。

This separation lets us state the safety boundary precisely, and we regard that boundary as a first-class design contribution. The mapping layer validates measured free space. A provisional semantic-frontier connection is an optimistic route hypothesis: it is checked against all currently known obstacles before publication, but it is never treated as proof that unobserved space is free. Semantic risk repels or attracts route choices, while current depth and the local executor remain solely responsible for newly observed obstacles. In the language of risk-aware planning [24], semantic evidence enters as a shaped preference over verified alternatives, not as a probabilistic safety guarantee.

> **【本段目的】** 四条贡献清单，即全文路线图：贡献1 即插即用持久拓扑路线层；贡献2 拓扑图 GCN 前沿策略；贡献3 语言偏好与几何权威分离；贡献4 跨执行器验证。

The main contributions are:

- **A plug-and-play persistent topological route layer.** Verified bubbles, witness edges, stable identities, and atomic replacement preserve route structure. A moving world-frame local goal exposes this memory to goal-conditioned planners without graph inputs or retraining, while fixed-radius operations control online work (Secs. III-B and III-D).
- **A topology-graph GCN frontier policy.** Weighted graph convolution and global topology pooling learn a 35-m-look-ahead frontier decision directly on the online skeleton. The policy supports five planar directions or 15 yaw–pitch directions, while downstream A* retains reachability and collision authority (Sec. III-E).
- **Language preference separated from geometric authority.** Provisional far-field frontiers and witness-level semantic risk can change route ranking before reliable depth, while A* connectivity and collision checks retain control of route acceptance (Sec. III-C).
- **Cross-executor validation.** The same topology-layer implementation is connected to YOPO, EGO-Planner, and SUPER through executor-specific goal and command adapters, and evaluated in both the primary cross-planner mission and a height-changing 3-D line-obstacle mission (Sec. IV).

> **【本段目的】** 预告实验维度：系统集成、路线选择、体积导航、图持久性四条线分离评估；声明语义观测全部使用 PEARL 但接口本身模型无关；主实验固定高度，另有单独 3-D 任务。

The evaluation separates system integration, route selection, volumetric navigation, and graph persistence. All reported semantic observations use PEARL [21]; the interface itself is model-agnostic. The primary cross-planner comparison holds altitude fixed for controlled branch selection, and a separate 3-D task enables the full volumetric graph and vertical motion.

> **【本段目的】** 系统架构图：总览数据流——深度/里程计维护持久拓扑，RGB/文本塑造路线代价，GCN 排序，A* 验收 witness，执行器只收到深度、状态与局部目标。

![Fig. 2. TopoGuide as a plug-and-play route layer. Depth and odometry maintain persistent verified topology; RGB and text shape route cost, and a topology GCN ranks frontier directions. A* and collision checks accept a witness before the moving-goal interface drives a goal-conditioned local executor. The executor receives only depth, state, and the local goal; topology, semantics, and route memory remain upstream.](pics/system_architecture_topology_v3.pdf)

**Fig. 2.** TopoGuide as a plug-and-play route layer. Depth and odometry maintain persistent verified topology; RGB and text shape route cost, and a topology GCN ranks frontier directions. A* and collision checks accept a witness before the moving-goal interface drives a goal-conditioned local executor. The executor receives only depth, state, and the local goal; topology, semantics, and route memory remain upstream.

## II. RELATED WORK

> **【本段目的】** 文献线一：反应式空中规划器（用延迟换全局一致性）；落点——TopoGuide 与它们正交，增量在不改动局部规划器核心的目标适配器上。

Reactive aerial planners trade global consistency for latency: RAPPIDS and NanoMap operate on depth and short history [2], [3], EGO-Planner removes the ESDF from optimization [4], and learned policies generate motion primitives directly [1], [5]. Fast-Planner, FASTER, and SUPER instead combine search or point-cloud corridors with trajectory refinement [8], [9], [10]. TopoGuide is orthogonal: it supplies a persistent route decision above an unchanged local planner core through a goal adapter.

> **【本段目的】** 文献线二：拓扑与前沿记忆是分层分离的天然载体；落点——本文图存储的是已验证 witness、跨原子替换保持路线身份，并把骨架拓扑同时暴露给 GCN 策略。

Topological and frontier memories provide the natural substrate for this separation. Incremental frontier structures and bubble graphs [11], [12], [7], [6] preserve reusable free-space structure, while learned topological memories [14], [15] show the value of graph state for long-horizon navigation. Our graph stores verified witnesses, preserves route identity across atomic replacement, and exposes the same skeleton topology to the GCN frontier policy.

> **【本段目的】** 文献线三：图卷积网络适合在稀疏连接上聚合关系状态；落点——TopoGuide 把该范式实例化到在线导航骨架上（显式边权、全局拓扑池化、候选打分头），但路线构建与碰撞验证仍在网络之外。

Graph convolutional networks aggregate relational state directly over sparse connectivity [16], making them a natural alternative to raster or independent-candidate encoders for route selection. Practical graph-policy designs combine node message passing, graph-level context, and an action-scoring head [17]. TopoGuide instantiates this pattern on the online navigation skeleton with explicit topology-edge weights, global topology pooling, and a candidate-scoring head; the planar policy also uses recurrent direction state. The 35-m route-direction labels and online candidate interface are specific to TopoGuide, while route construction and collision validation remain outside the network.

> **【本段目的】** 文献线四：开放词汇模型与前沿选择系统解决 where to go；落点——本文处理互补的空中问题 where to avoid：语义热图成为临时图前沿与 witness 级风险，但几何图检查保持权威。

Open-vocabulary models align image regions with text [18], [19], [20], [21] and have been lifted into semantic maps. Frontier-selection systems [22], [23] use such evidence to choose where to go. TopoGuide addresses the complementary aerial problem of where to avoid: semantic heatmaps become provisional graph frontiers and witness-level risk, but geometric graph checks remain authoritative.

> **【本段目的】** 文献线五：风险感知规划（不将置信度当作碰撞概率）；落点——本文遵循该原则，把语义证据限制在窄接口之后，可执行自由空间由拓扑图、A* 与碰撞检查定义。

Risk-aware planning shapes route preference without treating confidence as a collision probability [24]. We follow this principle and keep semantic evidence behind a narrow interface; the topology graph, A*, and collision checks define executable free space.

## III. METHOD

### III-A. Problem Formulation and System Overview

> **【本段目的】** 给出定义（问题形式化）：车辆状态、传感器输入（RGB、深度、里程计）、任务输入（世界系目标与描述需避开区域的文本查询）；声明无先验几何地图。

Let the vehicle state at time $t$ be $\boldsymbol{x}_t=(\boldsymbol{p}_t,\boldsymbol{v}_t,\boldsymbol{a}_t,\boldsymbol{R}_t)$. The sensors provide an RGB image $I_t$, a depth image $D_t$, and odometry. The mission specifies a world-frame goal $\boldsymbol{g}$ and a text query $q$ describing regions to avoid. No prior geometric map is available.

> **【本段目的】** 给出分层组合的接口契约（公式 (1)–(3)）：建图、路线选择、执行三层各自输入输出分明，随后逐层说明接口为何刻意收窄，并界定 GCN 输出只是排序提示而非轨迹或安全决策；最后给出 YOPO 的 9 维观测向量（公式 (4)），确立"plug-and-play 边界 = 单个标准目标"这一核心接口主张。

The planner must output a dynamically feasible local trajectory $\tau_t$. Following the layered organization, mapping, route selection, and execution compose as

$$
\begin{aligned}
  \mathcal{G}_t
    &= \Pi_{\mathrm{map}}(\mathcal{G}_{t-1},D_t,\boldsymbol{x}_t), \tag{1}\\
  (\mathcal{W}_t,\boldsymbol{g}^{\mathrm{front}}_t,
   \boldsymbol{g}^{\mathrm{loc}}_t,\hat c_t)
    &= \Pi_{\mathrm{route}}(\mathcal{G}_t,I_t,q,\boldsymbol{g}), \tag{2}\\
  \tau_t
    &= \Pi_{\mathrm{exec}}(D_t,\boldsymbol{o}_t). \tag{3}
\end{aligned}
$$

Each layer has a deliberately narrow interface. $\Pi_{\mathrm{map}}$ consumes depth, odometry, and its previous graph state; neither the semantic model nor the local planner appears in its update. The slower $\Pi_{\mathrm{route}}$ produces an accepted world-frame witness $\mathcal{W}_t$, an internal route anchor $\boldsymbol{g}^{\mathrm{front}}_t$, and a moving execution goal $\boldsymbol{g}^{\mathrm{loc}}_t$ sampled ahead on that witness. When enabled, the topology-graph policy additionally publishes a frontier index $\hat c_t$: $\hat c_t\in\{0,\ldots,4\}$ in planar mode and $\hat c_t\in\{0,\ldots,14\}$ in volumetric mode. This is a ranking hint, not a trajectory or safety decision. The plug-and-play boundary is the single standard goal $\boldsymbol{g}^{\mathrm{loc}}_t$. For YOPO, the released policy receives its usual $160\times96$ depth channel, body-frame motion, and this goal expressed in the body frame:

$$
 \boldsymbol{o}_t = [\boldsymbol{v}^{B}_t,\boldsymbol{a}^{B}_t,
                (\boldsymbol{g}^{\mathrm{loc}}_t-\boldsymbol{p}_t)^{B}] \in \mathbb{R}^9. \tag{4}
$$

EGO-Planner and SUPER receive the same world-frame goal through a thin ROS topic adapter. This division mirrors hierarchical topological control [15], but no graph tensor, semantic feature, witness polyline, or corridor representation crosses the executor boundary.

> **【本段目的】** 声明表示的普适性：所有要素都在 $\mathbb{R}^3$ 中，可选的单高度投影只是受控对比用的开关；关闭投影后同一 witness 接口同时指挥垂直与水平运动，为 IV-E 的 3-D 实验埋伏笔。

Bubble centers, checked witnesses, route progress, and moving goals are all represented in $\mathbb{R}^3$. The implementation can optionally project the derived topology onto one navigation height for controlled planar comparisons. With that projection disabled, graph construction and A* preserve node altitude, and the same witness interface commands vertical as well as horizontal motion. Semantic cost is evaluated on the complete 3-D witness in either mode.

### III-B. Topology Graph Construction and Maintenance

> **【本段目的】** 贡献1主体（第一部分）：定义图的顶点（公式 (5)：中心、间隙、语义状态、持久身份）与边（已检查 witness 折线与长度）；强调 witness 而非端点连线才是碰撞检查与语义融合使用的几何对象。

The mapper receives depth and odometry and publishes an atomic skeleton graph. It is independent of the semantic model, GCN, and local executor. The current depth frame is voxel-filtered into obstacle points and free-ray evidence; no occupancy grid or ESDF is retained. Collision-free bubbles are clustered into vertices

$$
 v_i=(\boldsymbol{c}_i,\rho_i,z_i,h_i), \tag{5}
$$

where $\boldsymbol{c}_i$ is center, $\rho_i$ is clearance, $z_i$ is semantic state, and $h_i$ is a persistent identity. Each edge stores a checked witness polyline $\pi_{ij}$ and length $L_{ij}$. The witness, rather than the chord between endpoints, is the geometric object used by both collision checking and later semantic fusion.

> **【本段目的】** 贡献1主体（第二部分）：异步图构建与原子替换机制（持久身份跨替换存活、过期气泡有宽限期）；给出间隙软代价公式 (6)，并立刻声明"权威分层"原则——软偏好永远不会把几何有效边变成无效边。

Graph construction runs asynchronously. A background worker builds bubbles, clusters regions, connects candidate edges, and atomically swaps a complete snapshot into the planner. The point buffer, local radius, and virtual-node cap bound transient memory; persistent identities survive graph replacement, while stale bubbles receive a grace period and are then deleted. The minimum witness clearance $\bar\rho_{ij}$ contributes a soft preference

$$
 C_{\mathrm{clr}}(e_{ij})=
 \lambda_{\mathrm{clr}}L_{ij}
 \left(\frac{\rho^\star}{\rho^\star+\bar\rho_{ij}}\right)^2, \tag{6}
$$

but never turns a geometrically valid edge into an invalid one.

### III-C. Semantic Heatmap–Topology Fusion

> **【本段目的】** 贡献3主体（第一部分）：语义投影机制（公式 (7)）——$5\times3$ 网格均值池化后按固定光轴深度投影；关键设计点：投影不查询实测深度，因此空的远场回波无法抹去语义证据。

The frozen PEARL model maps $(I_t,q)$ to a patch heatmap. We mean-pool it on a $5\times3$ grid and project each retained patch at fixed optical-axis depth $\ell_s$:

$$
 \boldsymbol{y}_{u,t}=\boldsymbol{p}_t+\mathbf{R}_t
 \left(\boldsymbol{t}_{\mathrm{cam}}+\ell_s\widetilde{\boldsymbol{d}}_u\right). \tag{7}
$$

The projection does not consult measured depth, so an empty far-field return cannot erase semantic evidence. A patch-mean score and confidence tuple $(s_i,c_i)$ is associated with a persistent graph identity; stale observations are rejected by timestamp gates and graph replacement restores the tuple.

> **【本段目的】** 贡献3主体（第二部分）：临时语义前沿的接入规则（最多挂两个已验证骨干节点、不能穿越未知空间连接两条已验证走廊、被后续几何证实后升级为验证节点）；落点重申：前沿是路线假设，绝非自由空间证书。

High-scoring, spatially separated projections become provisional semantic frontiers. An unknown frontier may attach to at most two nearby verified backbone nodes, but its witness is checked against all currently known obstacles and cannot connect two verified corridors through unknown space. When later geometry creates a bubble at the same position, the identity is promoted to a verified node. Thus a frontier is a route hypothesis, never a free-space certificate.

> **【本段目的】** 贡献3主体（第三部分）：语义风险与整条 witness 融合而非仅端点（公式 (8) 的距离与高斯衰减、公式 (9) 的偏好代价）；再次重申权威分层——语义只重塑路线偏好，永不覆盖几何有效性。

Semantic evidence is fused with complete witness edges rather than only with endpoints. For semantic node $k$ and edge witness $\pi_{ij}$,

$$
\begin{aligned}
 d_{k,ij}&=\min_{\boldsymbol{x}\in\pi_{ij}}
             \|\boldsymbol{y}_k-\boldsymbol{x}\|_2,\\
 r_{ij}&=\max\Big\{\tfrac12(s_i c_i+s_jc_j),\\
 &\qquad \max_{k:d_{k,ij}\le R_s}s_kc_k
 \exp[-d_{k,ij}^2/(2(R_s/2)^2)]\Big\}.
\end{aligned} \tag{8}
$$

The resulting preference cost is

$$
 C_{\mathrm{sem}}(e_{ij})=
 \lambda_{\mathrm{sem}}L_{ij}
 [-\log(\max(\epsilon,1-r_{ij}))]. \tag{9}
$$

It reshapes route preference but never overrides geometric validity.

> **【本段目的】** 收尾路由搜索与记忆机制：半径 $R_g$ 内以组合代价搜索、世界系持有 witness 直到进展门槛或几何检查失败、原子替换不留图节点指针、时间戳快照防止半重建拓扑泄露到控制流。

The planner searches reachable graph vertices within radius $R_g$ using $C(e)=W_{ij}+C_{\mathrm{clr}}(e)+C_{\mathrm{sem}}(e)$. It stores the selected witness in world coordinates and holds it until progress reaches a fixed fraction or its geometric check fails. A fresh search then replaces the complete route; no graph-node pointers are retained across atomic replacement. The asynchronous contract uses timestamped snapshots, so graph, semantic, and control streams cannot expose a partially rebuilt topology.

### III-D. Planner-Agnostic Goal Interface

> **【本段目的】** 兑现 plug-and-play：接受路线的载体是 witness 而非修改局部策略；给出移动局部目标的弧长参数化公式 (10)（$\ell_{\mathrm{loc}}=15$ m）及 witness 替换时的投影与短暂保持门控机制。

The accepted witness, rather than a modified local policy, carries the route decision. Let $\mathcal{W}_t(s)$ be its arc-length parameterization and $s_t$ the closest forward progress to the vehicle. TopoGuide publishes

$$
 \boldsymbol{g}^{\mathrm{loc}}_t=
 \mathcal{W}_t\!\left(\min(s_t+\ell_{\mathrm{loc}},L_{\mathcal W})\right),
 \qquad \ell_{\mathrm{loc}}=15~\mathrm{m}. \tag{10}
$$

The farther $\boldsymbol{g}^{\mathrm{front}}_t$ identifies the selected route branch inside TopoGuide and is published for state inspection; the executor follows only $\boldsymbol{g}^{\mathrm{loc}}_t$. When a witness is replaced, the vehicle is projected onto the new polyline before Eq. (10) is evaluated. A short hold gate suppresses transient goal loss during an atomic graph swap.

> **【本段目的】** 说明接口的工程实现与可迁移契约：标准 `geometry_msgs/PoseStamped`、YOPO 直接消费、EGO/SUPER 经薄桥接转发；给出精确的适配条件——局部规划器只需接受位置目标并发布可执行指令，无需理解图、语义或路线记忆。

The implementation exposes $\boldsymbol{g}^{\mathrm{loc}}_t$ as a world-frame `geometry_msgs/PoseStamped`. The original YOPO checkpoint consumes it through its ordinary relative-goal input. For EGO-Planner and SUPER, a thin bridge rate-limits and republishes the same message on their native goal topic; separate point-cloud and command bridges adapt the shared simulator I/O. These adapters do not alter either planner's search, optimization, or control law. Consequently, the transferable contract is precise: a local planner must accept a position goal and publish executable commands; it need not understand TopoGuide's graph, semantics, or route memory.

> **【本段目的】** 防御性声明：闭环实验中不向任何规划器传入走廊、气泡序列、路线掩码或图特征，也不重训权重；过期或失效 witness 在上游被拒绝并触发新 A*——预先排除"接口泄漏"质疑。

The closed-loop experiments pass no corridor, bubble sequence, route mask, or graph feature into any planner and do not retrain planner weights. TopoGuide instead preserves route intent by advancing the goal along the witness, while current depth and the local planner remain responsible for short-horizon motion. A stale or invalid witness is rejected upstream and triggers new A*.

### III-E. Topology-Graph GCN Frontier Policy

> **【本段目的】** 贡献2主体（第一部分）：GCN 作为学习的长程路线选择组件——平面 5 个偏航方向、3-D 15 个偏航-俯仰方向；限定其角色为"提议者"：既不创建路线也不发布执行指令，几何验收仍归 A*。

The GCN is TopoGuide's learned long-range route-selection component. In the planar configuration, it predicts which of five yaw directions should be considered first: left $40^\circ$, left $20^\circ$, forward, right $20^\circ$, or right $40^\circ$. The 3-D configuration crosses these yaw angles with upward, level, and downward pitch rows, yielding 15 candidate directions. In both cases, the output is only a frontier proposal: it neither creates a route nor publishes an executor command, leaving geometric acceptance to A*.

> **【本段目的】** 贡献2主体（第二部分）：在线推理细节——虚拟候选节点插入、20 维节点特征、两层边加权 GCN + 全局均值池化 + GRU + MLP 打分头；强调 GCN 列与手工排序走完全相同的 A* 可达性与碰撞检验管线。

At each inference tick, skeleton nodes and edges form the input graph. In the planar configuration, five virtual candidates are inserted $10$ m from the vehicle and connected to their nearest skeleton node with inverse-distance edge weights. Each node has a 20-dimensional feature vector containing normalized world and body-frame position; bubble radius and degree; semantic score and confidence; candidate identity; origin and goal distance; odometry and forward-candidate flags; yaw; range; bearing; and body-frame mission-goal state. The deployed `FrontierGCN(input_dim=20)` uses two lightweight edge-weighted GCN message-passing layers following classical graph convolution [16]. Global mean pooling supplies graph-level context, a GRU fuses the previous direction, and an MLP scores the five candidate nodes. Its column is published on `/scalenav/gcn_frontier_column`; A* then tests reachability and collision validity exactly as it does for a hand-designed ranking. Fig. 3 summarizes this deployed policy.

> **【本段目的】** GCN 架构图：用紧凑示意图支撑上文的部署策略描述（两层加权图卷积编码骨架与虚拟候选，池化递归上下文打分，A* 仍在下游）。

![Fig. 3. Compact architecture of the deployed FrontierGCN. Two weighted graph-convolution layers encode the online skeleton and five virtual candidates. Candidate embeddings are scored with pooled recurrent graph context to produce a five-way route proposal; A* remains downstream.](pics/gcn_model_architecture.pdf)

**Fig. 3.** Compact architecture of the deployed `FrontierGCN`. Two weighted graph-convolution layers encode the online skeleton and five virtual candidates. Candidate embeddings are scored with pooled recurrent graph context to produce a five-way route proposal; A* remains downstream.

> **【本段目的】** 说明体积版检查点的差异（15 个候选、24 维特征、三层 GCN、俯仰行 $+20^\circ/0^\circ/-20^\circ$），并声明其输出受与平面版相同的下游 A* 与 witness 检查约束。

The volumetric checkpoint inserts all 15 yaw–pitch candidates, preserves three-dimensional graph coordinates, and uses 24 node features and three weighted GCN layers. Its pitch rows are $+20^\circ$, $0^\circ$, and $-20^\circ$; the output remains subject to the same downstream A* and witness checks as the planar policy.

> **【本段目的】** 贡献2主体（第三部分）：离线标签构造——特权静态点云投影 + 膨胀 + 八邻域 A* 提供 35-m 前瞻方向标签，3-D 版用 26 连通 A*；会话级划分防泄漏；特权几何仅用于离线标签；末尾自我降级 claim：分类准确率不是安全声明。

Planar training labels use the complete static ground-truth point cloud, projected onto a $0.75$ m grid and inflated by the $1.2$ m vehicle radius. Eight-neighbor A* provides the direction of the first path point at least $35$ m ahead; out-of-view and incomplete paths are discarded. Session-level splits and class-weighted cross-entropy prevent adjacent frames from crossing splits. For the 3-D checkpoint, a 26-connected A* instead searches the inflated Unreal mesh volume and labels the yaw–pitch direction of the 35-m look-ahead point. The ground-truth geometry is used only to construct offline labels and is not available to the deployed policy. The resulting planar classifier is evaluated separately in Sec. IV-D; its accuracy is not a safety claim.
