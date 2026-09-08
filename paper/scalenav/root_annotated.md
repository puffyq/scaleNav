# TopoGuide: A Plug-and-Play Topological Route Layer for Language-Guided Aerial Navigation

*Anonymous Authors*

> **【本段目的】** 开篇 teaser 图：用一个真实飞行场景一次性展示系统的全部要素（传感器输入、持久拓扑骨架、语义远场观测、GCN 路线选择与飞行轨迹），为后文所有概念提供可视化锚点。

![Fig. 1. TopoGuide's layered route decision in Map2. (a) Synchronized RGB, semantic response, and depth at graph time $t^*$; white depth pixels have no return beyond $20$ m. (b) The light footprint is the complete UE mesh truth for reference. Dots and thin links are persistent skeleton nodes and collision-checked edges; teal is the logged A* topology path, dashed orange is its polynomial witness, and black is the flown trajectory. Red crosses are logged far-field semantic observations; the three labeled goals are published by this graph snapshot. (c) At a recorded route fork before the large block, the GCN selects an early bypass matching the 35-m A* label while the hand-designed ranking selects another column. Dark points and links are the synchronized cloud and online skeleton; the solid pale footprints are privileged context for visualization only.](pics/teaser.pdf)

**Fig. 1.** Schematic of TopoGuide's layered route decision beyond the depth-sensing horizon. (a) The RGB image, semantic response, and depth map are synchronized at graph time $t^*$; white depth pixels indicate no return beyond $20$ m. (b) FrontierGCN selects an early bypass at the route fork before the large obstacle under semantic, safety, and mission constraints, integrates it into the node graph, and publishes it as a frontier goal to guide the UAV.

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
## IV. EXPERIMENTS

> **【本段目的】** 实验总起：声明四个互补评估维度；交代仿真平台（UE 5.7 + Colosseum/AirSim 兼容）与主任务设置（1.6-m 固定高度、$(0,0,1.6)$ 到 $(0,140,1.6)$ m 单程、不随机化布局），并说明 IV-E 单独评估解除固定层约束的体积任务。

The evaluation examines four complementary aspects of the system: closed-loop compatibility of the topological goal interface with different local planners, the effects of semantic cost and the GCN policy on route selection, height-changing 3-D navigation, and the online graph-search workload under persistent geometry. Closed-loop and profiling trials are conducted in Unreal Engine 5.7 with Colosseum, an AirSim-compatible simulator [26], using synchronized RGB-D and odometry. Unless noted otherwise, the primary experiments use a 1.6-m planning layer and one-way mission from $(0,0,1.6)$ to $(0,140,1.6)$ m; neither the obstacle layout nor the nominal start and goal is randomized across methods. Sec. IV-E separately evaluates a volumetric Map4 mission with the fixed-layer constraints disabled.

### IV-A. Evaluation Protocol

> **【本段目的】** 实验协议（试次控制）：定义公平的起跑条件（位置/速度/航向门槛保持 1 s）与结果归类规则（启动失败剔除，碰撞与超时计入，完成指标只用成功试次）。

*Trial control and statistics.* The simulation is reset before every trial. A goal is issued only after the vehicle remains within 0.10 m of the common start, below 0.10 m/s, and within 0.10 rad of the mission heading for 1 s. Startup failures are excluded from navigation outcomes, while collision and timeout remain in the outcome rates; completion metrics use successful trials only.

> **【本段目的】** 实验协议（指标定义）：成功率/碰撞/超时/路径长/飞行时间/平均速度/最大速度；失败飞行单独报告 $L_{\rm obs}$ 且绝不混入成功均值；profiling 分离点处理、图构建、活跃 A* 与完整规划节拍。

*Metrics.* Closed-loop metrics are success, collision, timeout, path length $L$, flight time $T$, $\bar v=L/T$, and maximum speed. For failed flights, distance traveled before termination is reported separately as $L_{\rm obs}$ and never mixed into successful-flight completion means. Profiling separates point processing, graph construction, active A*, and the complete planning tick.

> **【本段目的】** 实验协议（基线免责，解释阀门）：预先声明 EGO/SUPER 的失败刻画的是这些部署而非规划器的普遍能力；plug-in 声明只关乎窄目标接口与不改 planner 核心；no-graph 行是直接目标对照而非竞争性全局规划基线——防止审稿人误读对比的意图。

*Baseline fairness.* EGO-Planner [4] and SUPER [10] retain their planner cores but require simulator-specific cloud, goal, and command adapters. Their executor configurations and timeouts differ from YOPO's, and recent benchmarking shows that such stacks are sensitive to latency, field of view, and scenario difficulty [25]. Their failures therefore characterize these deployments, not universal planner capability. The plug-in claim concerns the narrow goal interface and absence of planner-core changes, not strict experimental interchangeability. The no-graph rows are direct-goal executor controls rather than competitive global planning baselines; Table II therefore tests the route-interface deployment, not superiority over map-based global A* or sampling-based planners.

### IV-B. Cross-Planner Closed-Loop Evaluation

#### Methods and Conditions

> **【本段目的】** 交代主对比的对照设计：每个独立执行器配对 TopoGuide 引导变体，所有变体共享同一图实现与接口，执行器内部不消费 TopoGuide 数据结构；每行 10 次有效试次；引出共享参数表 Table I。

The primary comparison pairs each standalone executor with a TopoGuide-guided variant. All TopoGuide variants run the same graph-node implementation, semantic route formulation, A* acceptance, witness memory, and local-goal publisher. YOPO consumes the goal directly; EGO-Planner and SUPER use the adapter described in Sec. III-D. Their internal planners do not consume TopoGuide data structures. Each row in the plug-in comparison contains ten valid trials. Table I lists route-layer parameters shared by the principal YOPO ablations.

> **【本段目的】** 参数表：用一张固定参数清单支撑可复现性，同时佐证"所有变体共享同一路线层实现"的对照声明。

**Table I.** Fixed Parameters Used by the Current Implementation

| Parameter | Value | Parameter | Value |
|---|---|---|---|
| Point voxel | 0.10 m | Point budget | 100,000 |
| Frame history | current frame only | Graph update | 100 ms |
| Graph rebuild | 100 ms | Search radius | 45 m |
| Local lookahead | 15 m | Goal-connect budget | 20 ms |
| $\lambda_{\rm sem}$ | 2.0 | $R_s$ | 8 m |
| $\lambda_{\rm clr}$ | 2.0 | $\rho^\star$ | 1.2 m |
| Score update | latest match | Edge-memory discount | disabled |
| Optical depth $\ell_s$ | 35 m | Node merge distance | 2.5 m |
| Semantic candidates/frame | $\leq 10$ | Virtual-node cap | 512 |
| Semantic candidates/edge | 8 | Semantic grid | $5\times3$ |

#### Environment and Task

> **【本段目的】** 环境定义：描述有序障碍课程（前 25 m 密集小障碍、后半 60-m 级大障碍块强制早期左右承诺）、其超出 20-m 深度截断与执行器反应视野、真值网格尺寸与占据统计；声明真值仅用于环境刻画；匹配语言试次用查询 "blocks, wall."

All methods traverse the same ordered obstacle course (Fig. 1(b)). Along the $+y$ mission direction, the first approximately 25 m contain a dense field of small obstacles. The latter half is dominated by a single approximately 60-m-scale block, spanning roughly $y=68$–$122$ m and $x=-27$–$26$ m, which forces an early commitment to a left or right bypass. Its extent exceeds the 20-m depth clip and the local executor's immediate reaction horizon, making persistent branch selection the central task. The ground-truth mesh spans $140.0\times132.0$ m. In the evaluation corridor $x\in[-45,45]$ m, $y\in[0,120]$ m, $3.91\%$ of 0.5-m cells intersect ground-truth geometry in the navigation slab $z\in[0.6,2.6]$ m. Ground truth is used only for environment characterization and is not available to any method online. Matched language trials use the query "blocks, wall."

#### Results

> **【本段目的】** 结果引读：先用代表性轨迹图（Fig. 4）做定性对比（仅前沿 vs 图引导，三执行器），再引出聚合结果表 Table II。

Fig. 4 is a qualitative visualization of representative current-scene trajectories. It contrasts frontier-only runs with graph-guided TopoGuide runs for YOPO, EGO-Planner, and SUPER; the TopoGuide (YOPO) panel uses the shortest successful trajectory from its ten-trial evaluation. Aggregate results are reported in Table II.

> **【本段目的】** 轨迹对比图：直观展示无图时三执行器全部撞毁、有图时成功绕行的定性差异，用颜色编码速度强化"图引导不只是能到，还飞得顺"。

![Fig. 4. Representative trajectories with and without the topology graph. Rows compare YOPO-Simple, EGO-Planner, and SUPER; the left column uses each standalone planner with its direct mission goal and the right column uses the TopoGuide route layer. The three TopoGuide panels use the shortest successful trajectory from their respective evaluations. Color denotes executed speed; circles, stars, and crosses mark starts, completed goals, and incomplete collision endpoints.](pics/experiments/map2_0_140_1p6/trajectory_speed_comparison_current.pdf)

**Fig. 4.** Representative trajectories with and without the topology graph. Rows compare YOPO-Simple, EGO-Planner, and SUPER; the left column uses each standalone planner with its direct mission goal and the right column uses the TopoGuide route layer. The three TopoGuide panels use the shortest successful trajectory from their respective evaluations. Color denotes executed speed; circles, stars, and crosses mark starts, completed goals, and incomplete collision endpoints.

> **【本段目的】** 主结果表：用三个执行器配对的 0% vs 100%/70%/90% 成功率支撑核心主张——同一拓扑目标契约可被三个不同 planner 核心执行（支撑"接口可执行"而非"规划器优越性"）。

**Table II.** Plug-in evaluation on the fixed 140-m mission ($n=10$ per row). Every row uses the same map, start, goal, and obstacle sequence. Each pair contrasts the executor's standalone goal with TopoGuide's moving local goal. TopoGuide rows share the same topology-layer implementation; executor-specific adapters only relay goals, point clouds, and commands.

| Method | Success (%) | Collision (%) | Timeout (%) | Path (m) | Time (s) | Avg. speed (m/s) | Max. speed (m/s) |
|---|---|---|---|---|---|---|---|
| YOPO-Simple (no graph) | 0 | 100 | 0 | $129.79\pm0.58^\dagger$ | $61.46\pm0.36^\dagger$ | $2.11\pm0.01^\dagger$ | $6.46\pm0.02^\dagger$ |
| TopoGuide (YOPO) | 100 | 0 | 0 | $171.59\pm6.57$ | $33.61\pm1.25$ | $5.105\pm0.045$ | $6.254\pm0.138$ |
| EGO-Planner (no graph) | 0 | 100 | 0 | $73.24\pm0.69^\dagger$ | $54.32\pm1.05^\dagger$ | $1.35\pm0.02^\dagger$ | $2.43\pm0.25^\dagger$ |
| TopoGuide (EGO) | 70 | 30 | 0 | $181.59\pm11.33$ | $114.05\pm7.82$ | $1.593\pm0.017$ | $2.852\pm0.292$ |
| SUPER (no graph) | 0 | 100 | 0 | $47.07\pm28.27^\dagger$ | $10.25\pm4.99^\dagger$ | $4.30\pm0.68^\dagger$ | $5.60\pm0.23^\dagger$ |
| TopoGuide (SUPER) | 90 | 10 | 0 | $179.76\pm10.00$ | $39.32\pm2.62$ | $4.577\pm0.159$ | $5.829\pm0.043$ |

> **【本段目的】** 汇报结果并解释 † 脚注语义：带 † 的值是被截断失败飞行的均值而非完成指标；同时交代 TopoGuide (EGO) 用 200-s 超时而其他行用 90 s——透明化不对等条件。

All three standalone configurations terminated without a successful mission, whereas the TopoGuide-guided YOPO, EGO-Planner, and SUPER variants completed 10/10, 7/10, and 9/10 trials. This demonstrates that the same topological goal contract is executable by three different planner cores. Values marked $\dagger$ are means over truncated failed flights, not completion metrics; unmarked continuous values use successful flights only. The TopoGuide (EGO) deployment uses a 200-s mission timeout, while the other rows use 90 s.

> **【本段目的】** 汇报结果（YOPO 行细节）：10/10 无碰撞无超时、22.6% 路径开销、最长试次 trial 6、终端位置误差与终端速度——用完成质量数字堵住"成功但飞得差"的潜在质疑。

The TopoGuide (YOPO) run produced no collision or timeout events: all ten trials reached the goal and satisfied the final position and settling-speed tolerances. Relative to the 140-m straight-line mission, its mean path is 31.59 m longer, corresponding to a $22.6\%$ path overhead. The longest trial was trial 6 at $188.43$ m and $36.72$ s, which accounts for most of the path length spread. The mean terminal position error was $0.096\pm0.029$ m and the mean terminal speed was $0.294\pm0.004$ m/s.

> **【本段目的】** 汇报失败模式并自我限定：列出 EGO 三次碰撞与 SUPER 一次碰撞的发生位置；强调终端记录本身无法把失败归因于 GCN、witness 或执行器跟踪——需要带时间戳的路线与指令轨迹，为 IV-F 的日志边界铺垫。

The latest executor regressions also expose distinct failure modes. TopoGuide with EGO-Planner had three collisions after 31.30, 83.41, and 132.15 m of executed motion, spanning early, middle, and late portions of the mission. TopoGuide with SUPER had one early collision after 6.95 m of executed motion and no timeouts. All ten trials in this evaluation were valid, with no exclusions. These terminal records distinguish collision from route divergence or stall, but do not by themselves attribute a failure to the GCN, accepted witness, or executor tracking. Such attribution requires the timestamped route and command trace rather than the terminal outcome alone.

### IV-C. Route-Layer Ablations

#### Semantic Influence

> **【本段目的】** 消融动机与协议：固定任务/超时/图配置/查询不变，仅关闭 PEARL、语义节点与语义边代价，检验路线收益是否依赖深度包络之外的语义证据；引出 Table III。

To test whether the route benefit depends on semantic evidence beyond the depth envelope, we repeated the GCN stack with the same fixed mission, 90-s timeout, graph configuration, and query, while disabling PEARL, semantic nodes, and semantic edge cost. Completion metrics use successful flights only, while $L_{\rm obs}$ records the path observed before collision or timeout. The resulting conventional flight metrics are reported in Table III.

> **【本段目的】** 语义消融表：固定 GCN 启发式下对比匹配查询/关闭语义/无关查询三条件，量化语义对结局率的影响。

**Table III.** Semantic route-layer ablation on the fixed mission with the GCN heuristic fixed. Semantic-off removes PEARL heatmaps, far-field nodes, and semantic edge cost; geometry, A*, goal publication, and YOPO remain enabled.

| Condition | Success (%) | Collision (%) | Timeout (%) | $L_{\rm succ}$ (m) | $L_{\rm obs}$ (m) | Time (s) | Avg. speed (m/s) | Max. speed (m/s) |
|---|---|---|---|---|---|---|---|---|
| Matched query | 100 | 0 | 0 | $171.59\pm6.57$ | -- | $33.61\pm1.25$ | $5.105\pm0.045$ | $6.254\pm0.138$ |
| Semantic disabled | 60 | 40 | 0 | $169.47\pm3.16$ | $11.69\pm1.60$ | $32.34\pm2.70$ | $5.261\pm0.315$ | $6.404\pm0.158$ |
| Irrelevant query | 80 | 20 | 0 | $175.76\pm6.09$ | $20.32\pm2.67$ | $35.02\pm1.65$ | $5.023\pm0.127$ | $6.398\pm0.274$ |

> **【本段目的】** 解读消融：语义改变结局率但不改变几何有效性契约（开 10/10，关 4 次早期碰撞）；无关查询 8/10 证明路线选择确实响应"语言查询与场景内容的对齐"而非盲动——支撑贡献3。

Semantic influence changes the outcome rate without changing the geometric validity contract: the semantic-on condition completed all ten flights, whereas the semantic-off condition had four early collisions near $y\approx10$ m. The successful-flight path and speed differences are secondary to this outcome gap. Together with the depth-clipped projection visualized in Fig. 1, this behavior is consistent with semantic evidence affecting a route before the corresponding obstacle is measured by depth. The irrelevant-query condition (prompt "trees") completes 8/10 trials, confirming that route selection responds to the alignment between the language query and visual scene content.

#### GCN Frontier-Policy Ablation

> **【本段目的】** 消融动机：隔离学习组件——固定任务、语义查询、路线代价参数与 YOPO 执行器，只替换前沿排序策略（手工 vs GCN）。

To isolate the learned route-selection component, we compare TopoGuide's hand-designed frontier ranking with the GCN policy while holding the fixed mission, semantic query, route-cost parameters, and YOPO executor fixed.

> **【本段目的】** 前沿策略消融表：在 TopoGuide 内部对比手工排序与 GCN，量化学习组件对成功率与路径效率的增益（支撑贡献2）。

**Table IV.** Frontier-Policy Ablation Within TopoGuide

| Method | Success (%) | Collision (%) | Timeout (%) | Path (m) | Time (s) | Avg. speed (m/s) | Max. speed (m/s) |
|---|---|---|---|---|---|---|---|
| Hand-designed ($n=27$) | 88.9 | 11.1 | 0 | $199.14\pm14.12$ | $38.54\pm3.06$ | $5.172\pm0.097$ | $6.409\pm0.184$ |
| GCN ($n=10$) | 100.0 | 0.0 | 0.0 | $171.59\pm6.57$ | $33.61\pm1.25$ | $5.105\pm0.045$ | $6.254\pm0.138$ |

> **【本段目的】** 解读消融：GCN 10/10 vs 手工 24/27，平均路径 199.14→171.59 m、时间 38.54→33.61 s；强调只替换排序策略，A*、witness 验收、语义代价与局部目标接口均未变——增益可归因于学习组件本身。

The GCN policy completed 10/10 trials, compared with 24/27 for the hand-designed ranking, while reducing mean path length from 199.14 m to 171.59 m and mean flight time from 38.54 s to 33.61 s. Only the frontier-ranking policy is replaced; A*, witness acceptance, semantic route cost, and the local-goal interface remain unchanged.

### IV-D. GCN Policy Evaluation

> **【本段目的】** GCN 离线评估（刻意与在线安全解耦）：报告数据集划分（5,337 帧 = 3,844 训练 / 786 验证 / 707 测试）、测试集 89.3% exact / 89.4% macro、99.4% 相邻列内、手工策略仅 22.2% 匹配严格标签；末尾强调这些数字度量方向预测而非在线安全。

The privileged dataset contains 5,337 frames, split by session into 3,844 training, 786 validation, and 707 held-out test frames. A validation sweep selects no additional switching penalty. On the untouched test split, the deployed checkpoint obtains $89.3\%$ exact and $89.4\%$ macro accuracy; $99.4\%$ of predictions are within one adjacent direction column, with a mean absolute column error of $0.115$. The logged hand-designed frontier policy matches $22.2\%$ of the strict 35-m labels. These results measure route-direction prediction, not online safety.

> **【本段目的】** GCN 离线指标表：把方向分类的关键数字集中呈现，便于与正文引用互查。

**Table V.** GCN Direction Classification

| Metric | Test |
|---|---|
| Exact accuracy | 89.3% |
| Macro accuracy | 89.4% |
| Within one direction column | 99.4% |
| Mean absolute column error | 0.115 |

> **【本段目的】** 引出在线接口实录图：说明 Fig. 5 保留了真实记录实验中的 RGB、PEARL 热图、GCN 所选列与任务目标方向，作为离线指标之外的在线行为证据。

Fig. 5 shows synchronized frames from the online interface. The RGB image, PEARL heatmap, selected GCN column, and mission-goal direction are retained from the recorded experiment.

> **【本段目的】** GCN 在线接口图：用闭环飞行中记录的四组画面展示提议前沿列（红）与 35-m 任务目标方向（绿虚箭头），让评审直观检查 GCN 提议的合理性。

![Fig. 5. Recorded GCN-interface views from closed-loop flights. (a) Building view 1. (b) Building view 2. (c) Forest view 1. (d) Forest view 2. Each panel preserves the RGB frame and PEARL heatmap; red marks the proposed frontier column and the green dashed arrow marks the 35-m mission-goal direction.](pics/0903_graph_semantic_frontier_building1.png)

(a) Building view 1: `pics/0903_graph_semantic_frontier_building1.png` 　(b) Building view 2: `pics/0903_graph_semantic_frontier_building2.png` 　(c) Forest view 1: `pics/0903_graph_semantic_frontier_forest1.png` 　(d) Forest view 2: `pics/0903_graph_semantic_frontier_forest2.png`

**Fig. 5.** Recorded GCN-interface views from closed-loop flights. Each panel preserves the RGB frame and PEARL heatmap; red marks the proposed frontier column and the green dashed arrow marks the 35-m mission-goal direction.

### IV-E. Height-Changing 3-D Navigation

> **【本段目的】** 3-D 任务设置：单独的 Map4 输电线走廊任务（$(0,0,3.5)$ 到 $(0,140,3.5)$ m），关闭图投影与 YOPO 定高约束，使用 15 方向专用检查点（2,200 个特权 3-D 网格标注状态）；在线观测仍只有 RGB-D、里程计与查询 "line"——证明系统不是平面特例。

We further evaluate the volumetric mode in a separate Map4 line-obstacle mission. The start and goal are $(0,0,3.5)$ and $(0,140,3.5)$ m, placing the nominal flight height through the power-line corridor. Both the graph-layer projection and YOPO's fixed-altitude constraint are disabled. A dedicated 15-direction checkpoint, trained on 2,200 graph states labeled by 26-connected A* in the privileged 3-D mesh volume, ranks three pitch rows and five yaw columns; the online planner still observes only RGB-D, odometry, and the text query "line."

> **【本段目的】** 3-D 结果表：给出变高任务的成功率与连续指标（含 $z$ 跨度），量化体积模式下的飞行行为。

**Table VI.** Height-Changing Map4 Line-Obstacle Mission ($n=10$). Continuous values are computed over the successful flights.

| Method | Success (%) | Collision (%) | Timeout (%) | Path (m) | Time (s) | Avg. speed (m/s) | $z$ span (m) |
|---|---|---|---|---|---|---|---|
| TopoGuide (GCN-3D) | 80 | 20 | 0 | $167.40\pm13.83$ | $36.98\pm5.30$ | $4.564\pm0.327$ | $2.225\pm0.477$ |

> **【本段目的】** 解读 3-D 结果：80% 成功无超时，每次成功飞行都有实质高度变化（垂直跨度 1.731–2.934 m），两次失败均为早期碰撞；落点——同一拓扑-witness-目标管线支持变高路线执行而非仅平面分支选择。

TopoGuide completes $80\%$ of the trials without timeout. Every successful flight changes altitude substantially: the per-flight vertical span ranges from 1.731 to 2.934 m. The shortest successful route is 149.203 m in 30.934 s and spans 1.808 m vertically. The two failures are early collisions after 10.907 and 10.019 m. Together with the non-planar graph and executor settings, these trajectories verify that the same topology–witness–goal pipeline supports height-changing route execution rather than only planar branch selection.

### IV-F. Online Planning Workload

> **【本段目的】** 防御 scalability 质疑：量化在线负载——持久模式下局部节点 214/249（P50/P95）、活跃 A* 扩展 185/215 节点，与滑动几何消融（197/231、173/194）对比，证明保留骨干不会让全部节点进入每个规划节拍；GCN 在 RTX 3090 上仅加 0.83 ms 平均延迟。

The online search is restricted to a local working set even when the verified topology persists outside the current sensing window. The current configuration keeps at most 100,000 point samples, rebuilds the skeleton every 100 ms, searches within 45 m, and caps virtual semantic nodes at 512. Fig. 6 compares the persistent-geometry mode with a sliding-geometry ablation. The persistent mode uses 214/249 local nodes at P50/P95 and expands 185/215 nodes per active A* search, compared with 197/231 and 173/194 when persistence is disabled. Thus retaining the explored backbone does not make every retained node part of each planning tick. Over normalized session time, the observed local-node P95 remains below 279 with persistence and 248 without it. At approximately 500 nodes and 3,200 directed edges, GCN inference on an NVIDIA GeForce RTX 3090 adds 0.83 ms mean latency to the online workload, with 0.60 ms at P50 and 1.97 ms at P95.

> **【本段目的】** 资源曲线图：用局部节点随时间的曲线与模块耗时柱状图可视化持久 vs 滑动几何的在线开销对比（27 次持久 + 10 次滑动运行聚合）。

![Fig. 6. Online resources for persistent and 40-m sliding geometry. (a) Local nodes over normalized time. (b) Module wall time. Lines/bars show P50/mean and dashed lines/caps show P95. The profiles aggregate 27 persistent-geometry and 10 sliding-geometry runs of the same fixed mission; global nodes are excluded from the local search.](pics/experiments/map2_0_140_1p6/graph_resource_scaling.pdf)

**Fig. 6.** Online resources for persistent and 40-m sliding geometry. (a) Local nodes over normalized time. (b) Module wall time. Lines/bars show P50/mean and dashed lines/caps show P95. The profiles aggregate 27 persistent-geometry and 10 sliding-geometry runs of the same fixed mission; global nodes are excluded from the local search.

> **【本段目的】** 补充时序行为：路线持有 witness 直到进展门槛或几何检查失败，使路线切换可观测；语义流约 2 Hz，图与控制更新异步——交代系统的运行节奏。

A route holds its accepted witness until a progress gate is reached or a geometric check fails; this makes route switching observable and prevents a partially rebuilt graph from leaking into the controller. The semantic stream runs at roughly 2 Hz, whereas graph and control updates remain asynchronous.

> **【本段目的】** 可复现性与失败归因边界：列出每试次记录字段；解释日志边界为何重要——把安全事件归因到图-规划器-执行器接口，而不是单独的离线 GCN 分类分数（呼应 IV-B 的失败归因限定）。

For reproducibility, each trial records the graph snapshot timestamp, selected frontier column, witness clearance, A* replacement reason, local-goal distance, and terminal outcome. These fields let us distinguish a direction proposal from the route actually accepted after collision checking. They also expose the failure sequence seen in the evaluation: a stale or low-clearance witness is rejected, a fresh A* search is requested, and a moving vehicle may reach the executor's start-failure condition before the replacement goal is established. This logging boundary is important because it attributes safety events to the graph–planner–executor interface rather than to an offline GCN classification score alone.

## V. DISCUSSION AND LIMITATIONS

> **【本段目的】** 自我降级 claim 之一：精确限定 "plug-and-play" 的含义——不重训不改算法（在目标与指令接口适配之后），但不等于任意规划器零集成代码即可接入。

The contribution is the topology route layer and its narrow goal contract, not a modified local policy. The current YOPO checkpoint receives only depth, state, and the moving local goal. EGO-Planner and SUPER require simulator and topic adapters, but their planner cores do not receive graph features or semantic scores. Thus "plug-and-play" means no planner retraining or algorithm modification after the goal and command interfaces are adapted; it does not mean arbitrary planners can be connected without integration code.

> **【本段目的】** 自我降级 claim 之二：责任边界——执行器保留短视距可行运动的责任（无走廊约束、无气泡序列）；10/10、7/10、9/10 只证明固定任务下三种执行器架构间的可移植性。

The single-goal interface defines a clear responsibility boundary. TopoGuide advances the goal along the accepted witness, while each executor retains responsibility for feasible short-horizon motion rather than receiving a corridor constraint or bubble sequence. The 10/10, 7/10, and 9/10 TopoGuide outcomes demonstrate portability across three executor architectures in the fixed mission.

> **【本段目的】** 自我降级 claim 之三：离线 GCN 精度只度量方向预测，在线路线有效性仍由 A* 与 witness 碰撞检查保证；指出未来图策略可加入可达间隙、制动余量与 A* 代价并在学习排序前屏蔽无效列。

The offline GCN accuracy measures direction prediction, while online route validity remains enforced by A* reachability and witness collision checks. Future graph policies can incorporate reachable clearance, braking margin, and A* cost directly and mask invalid columns before learned ranking.

> **【本段目的】** 自我降级 claim 之四：承认评估范围有限——主对比用固定高度层，3-D 任务平均垂直跨度 $2.225\pm0.477$ m；随机布局、动态障碍、真实传感器与外观变化留待未来；固定深度投影仍是假设，其一致性与拒绝率有待量化；更长任务与 RSS 测量将进一步刻画持久骨干。

The primary Map2 comparison uses a common fixed-height layer to control the cross-executor study, while the Map4 line-obstacle evaluation exercises the volumetric graph, 15-direction GCN, and vertical executor motion. Across its successful flights, the latter produces a mean vertical span of $2.225\pm0.477$ m. Evaluation across randomized layouts, dynamic obstacles, and real sensor and appearance variation will test the same 3-D pipeline more broadly. Fixed-depth projections remain hypotheses: known-obstacle checks constrain their witnesses, and future work can quantify projection consistency and rejection. Longer missions and process-RSS measurements will further characterize the persistent backbone.

> **【本段目的】** 自我降级 claim 之五：失败归因的局限与训练/部署的信息不对称——GCN 训练用特权静态图标签、部署只见部分在线骨架；再次划清离线精度（方向预测）与闭环试次（完整系统）的度量边界。

Logged route state separates graph acceptance from execution; synchronized command and tracking traces can refine failure attribution. The GCN uses privileged static-map A* labels in training but only the partial online skeleton at deployment, where A* and witness checks validate each proposal. Thus offline accuracy measures direction prediction, while closed-loop trials measure the complete system.

## VI. CONCLUSION

> **【本段目的】** 单段总结呼应 abstract：重申 TopoGuide 是拓扑路线层而非又一个局部规划器；回收全部关键证据（三执行器闭环、3-D 变高、局部搜索仅数百节点）；落回中心系统主张——长程记忆与语言条件路线选择可叠加在可互换短视距执行器之上。

TopoGuide is a topological route layer rather than another local planner. It turns depth and odometry into persistent witness-verified structure, combines language risk with a topology-graph GCN frontier policy upstream, and exports only a moving world-frame local goal. The original YOPO policy consumes that goal directly, while goal adapters connect the same topology implementation to EGO-Planner and SUPER without planner retraining. Closed-loop results across three heterogeneous executors validate the shared route layer and moving-goal contract; a separate 3-D line-obstacle evaluation demonstrates height-changing graph construction and route execution. The observed local search remains a few hundred nodes. These results support the central systems claim: long-horizon memory and language-conditioned route choice can be added above interchangeable short-horizon executors in planar and volumetric missions.

## REFERENCES

1. J. Lu, X. Zhang, H. Shen, L. Xu, and B. Tian, "You Only Plan Once: A Learning-Based One-Stage Planner With Guidance Learning," *IEEE Robot. Autom. Lett.*, vol. 9, no. 7, pp. 6083–6090, 2024.
2. N. Bucki, J. Lee, and M. W. Mueller, "Rectangular Pyramid Partitioning Using Integrated Depth Sensors (RAPPIDS): A Fast Planner for Multicopter Navigation," *IEEE Robot. Autom. Lett.*, vol. 5, no. 3, pp. 4626–4633, 2020.
3. P. R. Florence, J. Carter, J. Ware, and R. Tedrake, "NanoMap: Fast, Uncertainty-Aware Proximity Queries With Lazy Search of Local 3-D Data," in *Proc. IEEE Int. Conf. Robot. Autom.*, 2018, pp. 7631–7638.
4. X. Zhou, Z. Wang, H. Ye, C. Xu, and F. Gao, "EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors," *IEEE Robot. Autom. Lett.*, vol. 6, no. 2, pp. 478–485, 2021.
5. A. Loquercio, E. Kaufmann, R. Ranftl, M. Müller, V. Koltun, and D. Scaramuzza, "Learning High-Speed Flight in the Wild," *Sci. Robot.*, vol. 6, no. 59, 2021.
6. Y. Ren, F. Zhu, W. Liu, Z. Wang, Y. Lin, F. Gao, and F. Zhang, "Bubble Planner: Planning High-Speed Smooth Quadrotor Trajectories Using Receding Corridors," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.*, 2022, pp. 6332–6339.
7. S. Geng, Z. Ning, F. Zhang, and B. Zhou, "EPIC: A Lightweight LiDAR-Based AAV Exploration Framework for Large-Scale Scenarios," *IEEE Robot. Autom. Lett.*, vol. 10, no. 5, pp. 5090–5097, 2025.
8. X. Zhou, J. Gao, C. Wang, Z. Wang, H. Ye, and F. Gao, "Fast-Planner: A Robust and Efficient Trajectory Planner for Autonomous Quadrotor Flight," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.*, 2019, pp. 8455–8461.
9. J. Tordesillas, B. T. Lopez, J. P. How, and L. Carlone, "FASTER: A Robust Fast-Planner for Safe Autonomous Navigation in Unknown Environments," *IEEE Trans. Robot.*, vol. 38, no. 2, pp. 1112–1131, 2022.
10. Y. Ren, F. Zhu, G. Lu, Y. Cai, L. Yin, F. Kong, J. Lin, N. Chen, and F. Zhang, "Safety-Assured High-Speed Navigation for MAVs," *Sci. Robot.*, vol. 10, no. 98, p. eado6187, 2025.
11. B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration," in *Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom.*, 1997, pp. 146–151.
12. C. Cao, H. Zhu, H. Choset, and J. Zhang, "TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments," in *Proc. Robot.: Sci. Syst.*, 2021.
13. H. Oleynikova, Z. Taylor, M. Fehr, R. Siegwart, and J. Nieto, "Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning," *IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2017.
14. N. Savinov, A. Dosovitskiy, and V. Koltun, "Semi-Parametric Topological Memory for Navigation," in *Proc. Int. Conf. Learn. Represent.*, 2018.
15. D. Shah, B. Eysenbach, G. Kahn, N. Rhinehart, and S. Levine, "ViNG: Learning Open-World Navigation With Visual Goals," in *Proc. IEEE Int. Conf. Robot. Autom.*, 2021, pp. 13215–13222.
16. T. N. Kipf and M. Welling, "Semi-Supervised Classification with Graph Convolutional Networks," in *Proc. Int. Conf. Learn. Represent.*, 2017.
17. A. Schutz and V.-A. Darvariu, "Using Graph Neural Networks in Reinforcement Learning: A Practical Guide," *ICLR Blogposts*, 2026. [Online]. Available: `https://iclr-blogposts.github.io/2026/blog/2026/rl-with-gnns/`
18. A. Radford et al., "Learning Transferable Visual Models From Natural Language Supervision," in *Proc. Int. Conf. Mach. Learn.*, 2021, pp. 8748–8763.
19. B. Li, K. Q. Weinberger, S. Belongie, V. Koltun, and R. Ranftl, "Language-Driven Semantic Segmentation," in *Proc. Int. Conf. Learn. Represent.*, 2022.
20. G. Ghiasi, X. Gu, Y. Cui, and T.-Y. Lin, "Scaling Open-Vocabulary Image Segmentation With Image-Level Labels," in *Proc. Eur. Conf. Comput. Vis.*, 2022.
21. G. Pei, X. Jiang, X. Cai, T. Chen, Y. Yao, and B. Jeon, "PEARL: Geometry Aligns Semantics for Training-Free Open-Vocabulary Semantic Segmentation," *arXiv preprint arXiv:2603.21528*, 2026.
22. N. Yokoyama, S. Ha, D. Batra, J. Wang, and B. Bucher, "VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation," in *Proc. IEEE Int. Conf. Robot. Autom.*, 2024, pp. 42–48.
23. O. Alama, A. Bhattacharya, H. He, S. Kim, Y. Qiu, W. Wang, C. Ho, N. Keetha, and S. Scherer, "RayFronts: Open-Set Semantic Ray Frontiers for Online Scene Understanding and Exploration," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.*, 2025, pp. 5930–5937.
24. L. Blackmore, M. Ono, and B. C. Williams, "Chance-Constrained Optimal Path Planning With Obstacles," *IEEE Trans. Robot.*, vol. 27, no. 6, pp. 1080–1094, 2011.
25. S.-A. Yu, C. Yu, F. Gao, Y. Wu, and Y. Wang, "FlightBench: Benchmarking Learning-Based Methods for Ego-Vision-Based Quadrotors Navigation," in *Proc. IEEE Int. Conf. Robot. Autom.*, 2025.
26. S. Shah, D. Dey, C. Lovett, and A. Kapoor, "AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles," in *Field and Service Robotics*, 2018, pp. 621–635.
