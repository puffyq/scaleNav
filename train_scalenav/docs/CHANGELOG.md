# ScaleNav Route-Conditioned YOPO Changelog

本文件按修改批次记录，不按日期聚合。每次代码更新新增一个独立变更编号；后续补充测试、
数据生成和模型评测结果时更新对应记录，不把不同修改合并到同一天的章节中。

<a id="chg-0031"></a>
## CHG-0031 收敛为 corridor barrier-only 路线条件

- 关闭 angle、centerline MSE 和 SafetyLoss route attraction；`wp` 与 `wcenterline` 仅保留
  兼容字段并默认为 0。
- RouteLoss 改为对 ordered Bubble union 计算 `relu(min_k(||p-c_k||-r_k))^2`，仅惩罚
  轨迹越出安全走廊，不把 witness 当作带时间参数的专家轨迹。
- trainer 将真实 `route_radii_world` 传入 loss，corridor barrier 进入候选 score target；
  五次多项式、15 个 primitive 和原版 YOPO cost 保持不变。
- SafetyLoss 与 corridor 使用配置中的 `safety_eval_points=100`，与离线 101 点审计保持同量级。
- 定向/完整测试：`66 passed`；`train_large_001` GPU 单 epoch smoke 和 900-route 离线评测已完成。

<a id="chg-0030"></a>
## CHG-0030 固定高度 Route 训练合同与可复现验证

- 根因是原 3x5 lattice 的上下行锚点为 `+20/-20 deg`，单 primitive 只能修正
  `+/-15 deg`，因此上下行不可能拟合水平 Route；原训练却对全部 15 条候选使用软 Route
  loss，在线又直接执行未投影的三维终态。
- Route-active 样本现在先将世界系终点投影到 Route local-subgoal 高度，并设置终端
  `vz=az=0`，再计算 ESDF、Route 和 score target；无效路线几何样本仅保留原三维 YOPO。
- 修复 safety ranking 错接总 safety/corridor cost 的问题，改用真实 collision barrier，
  并采用 `safety_ranking_target_margin` 判定 unsafe。
- validation motion 改为按 seed/scene/route 确定性生成，训练 seed 同时控制 frame-group
  split；checkpoint/run metadata 补齐 `wpath_mse` 与固定高度投影合同。
- 新增 `run_training_006.sh`，默认读取 `train_large_001`，从
  `saved_large/YOPO_0/best.pth` 微调并启用 score/safety ranking。
- 真实大数据 CPU 单 batch 冒烟训练通过；训练测试 `64 passed`。

<a id="chg-0029"></a>
## CHG-0029 扩充三场景训练数据并继续训练

- 新增独立训练批次 `dataset/train_large_001`：`yopo_forest`、`yopo_real_forest`、`blocks`
  三个场景，各 1000 帧、每帧 3 条真值 A* witness route，共 9000 条路线。
- 发布校验通过；`YOPODataset` 实际得到 8100 条 train、900 条 valid 样本，三维路径点、
  clearance、bubble 半径和拓扑字段完整。数据 viewer：
  `dataset/train_large_001/viewer/index.html`。
- 从 `saved_corrected/YOPO_5/best.pth` 以 `1e-5` 学习率、batch 32、GPU RTX 3090 继续训练
  20 epoch；产物：`saved_large/YOPO_0/best.pth`（最佳 epoch 6），训练日志和每 5 epoch
  checkpoint 同目录保存。
- 新批次全量 9000 条路线：碰撞率 `0.589%` (53/9000)，平均中心线距离 `0.334 m`，
  平均最大走廊越界 `0.105 m`，平均进度 `6.355 m`。评测 HTML：
  `dataset/train_large_001/evaluation_large_best/index.html`。
- 旧 `benchmark_004_esdf_001` 回归：碰撞率 `0.135%` (2/1479)，中心线距离 `0.319 m`，
  平均进度 `6.502 m`。三模型对比 HTML：
  `dataset/benchmark_004_esdf_001/comparison_large_best/viewer/index.html`。
- 旧基准上的三方结果为：新模型 `0.135%/6.502 m`、上一版 `0.135%/6.794 m`、
  YOPO-Simple `8.046%/6.225 m`（碰撞率/平均进度）。新大批次更难，不能只用旧基准
  指标判断泛化；后续应按场景分别分析并继续补充 hard-negative route。

<a id="chg-0016"></a>
## CHG-0016 加密安全轨迹时域采样

- 根因审计：YOPO 原安全项每条多项式只检查 30 个时刻，窄障碍可能落在采样点之间；用 101/401 点离线复查发现 30 点低估碰撞。
- `SafetyLoss` 新增 `safety_eval_points=100`，只增加安全代价的时域采样，不改变 15 个 primitive 或 witness 的 30 点 `path_mse` 目标。
- 离线轨迹评测默认改为 101 点，与训练安全审计保持同量级采样密度。
- 新增坐标/ESDF 查询回归测试仍通过；需用新采样配置重新训练后比较真实碰撞率。
- GPU 复训得到 `YOPO_45`：同一 1479 条路线、401 点密集评测下碰撞率 `0.406%`、平均进度 `5.235 m`、平均最大走廊越界 `0.333 m`；`YOPO_33` 同口径为 `0.879%`、`5.622 m`、`0.122 m`。安全收益明确，但进度和贴线性需要下一批次继续平衡。
- 评测报告：`dataset/benchmark_004_esdf_001/evaluation_yopo45/evaluation_report.json`。该次使用 `--report-only`，未生成 viewer HTML。

<a id="chg-0017"></a>
## CHG-0017 对齐 YOPO-Simple 的俯仰 anchor 角度

- 对比发现原版 YOPO-Simple 使用 `vertical_anchor_fov=30°`，ScaleNav 之前误设为 `0°`；这会完全关闭 primitive 的俯仰修正。
- 代码内部仍统一使用弧度：配置值只在 `LatticePrimitive` 中通过 `degrees / 180 * pi` 转换一次，`Rotation.from_euler(..., degrees=False)` 接收弧度。
- 旧 checkpoint 在该参数下不可直接比较，需重新训练后再生成 Route-YOPO 与 YOPO-Simple 的配对结果。
- 重新训练得到 `YOPO_46`：401 点密集评测碰撞率 `0.270%`、平均进度 `5.635 m`。
- 三方配对结果（1479 条路线）：Route-YOPO `0.270%` 碰撞、`5.635 m` 进度；原版 YOPO-Simple `8.046%`、`6.225 m`。`YOPO_45` 使用旧的 `0°` 俯仰解码，不能作为同口径基线。
- 对比 HTML：`dataset/benchmark_004_esdf_001/comparison_023/viewer/index.html`；单模型 HTML：`dataset/benchmark_004_esdf_001/evaluation_yopo46_viewer/index.html`。

<a id="chg-0018"></a>
## CHG-0018 提高引导线拟合权重

- 在 `vertical_anchor_fov=30°` 和密集安全采样基础上，将 `wpath_mse` 从 `0.05` 提升到 `0.2`，重新训练得到 `YOPO_47`。
- 1479 条路线结果：平均最大走廊偏差由 `0.202 m` 降至 `0.126 m`，平均进度由 `5.635 m` 提升至 `6.348 m`；碰撞率为 `0.473%`。
- 统一对比页面：`dataset/benchmark_004_esdf_001/comparison_025/viewer/index.html`。其中 Route-YOPO 与 YOPO-Simple 使用同一批输入和同一评测采样。

<a id="chg-0019"></a>
## CHG-0019 接入几何中心线与切向 loss

- 修复两个漏接：`path_centerline`（轨迹到 witness 最近线段距离）和 `path_tangent`（终点速度方向与 witness 切向）此前虽计算但未加入总 loss。
- 当前默认 `wcenterline=0.1`、`wtangent=0.25`，并保留 `wpath_mse=0.2`。
- `YOPO_50` 复训后平均最大走廊偏差降至 `0.090 m`，平均进度 `6.603 m`，碰撞率 `0.473%`。
- 统一对比 HTML：`dataset/benchmark_004_esdf_001/comparison_026/viewer/index.html`；单模型 HTML：`dataset/benchmark_004_esdf_001/evaluation_yopo50_viewer/index.html`。

<a id="chg-0020"></a>
## CHG-0020 消除 bubble 采样造成的人工窄腰

- `sample_route_bubbles` 之前使用每个采样区间内的最小半径，单个局部低净空点会把整颗 bubble 缩小。
- 改为使用 bubble 球心处的 ESDF 半径；每颗球仍独立满足安全净空，邻接重叠继续由路线质量门禁审计。
- 该修改只改变模型输入 bubble 的半径表达，不平移 witness 中心线；需要在新输入下重新训练和评测。

<a id="chg-0021"></a>
## CHG-0021 保持路径搜索选定的安全半径

- 修复中心线 refinement 的安全约束回退：原先 A* 选出的 clearance threshold 在后续样条/梯度 refinement 中丢失，局部安全半径可能从 `1.2 m` 缩回基础规划余量。
- `refine_witness_centerline` 现在接收 `minimum_safe_radius_m` 作为窄腰改善目标；候选点和线段始终保持基础可行净空，并在低于目标时沿 3D clearance 梯度优先抬高局部最小半径。
- 对称真实窄通道不会被强行平移；生成新数据后，bubble 的窄腰应只保留无法改善的真实瓶颈。

<a id="chg-0022"></a>
## CHG-0022 窄腰参数对照数据

- 新增小批量三场景验证集：`dataset/pilot_narrowwaist_002/viewer/index.html`，覆盖 YOPO 树林、真实树点云和大方块。
- 新增放宽绕行预算的对照集：`dataset/pilot_narrowwaist_detour_001/viewer/index.html`，`widest_detour_ratio=1.35`。
- 对照结果：平均 `safe_radius_p05` 从 `0.837 m` 提升到 `1.091 m`，平均 bubble overlap margin 从 `1.386 m` 提升到 `1.904 m`，平均路线长度从 `14.24 m` 增至 `15.70 m`。
- 结论：窄腰主要来自路线搜索的绕行预算，不是 bubble 半径区间采样；默认 `1.12` 暂不盲目修改，完整数据重建时应把该参数作为可配置实验项。
- `data.ground_truth_dataset` 新增 `--widest-detour-ratio` 与 `--widest-clearance-target`，可复现实验参数并生成完整训练集。

<a id="chg-0023"></a>
## CHG-0023 YOPO_52 窄腰修复后训练评测

- 使用 `YOPO_50/best.pth` 初始化，在 `benchmark_004_esdf_001` 的 1479 条路线继续训练 6 个 epoch，GPU 为 RTX 3090。
- 单模型 HTML：`dataset/benchmark_004_esdf_001/evaluation_yopo52_viewer/index.html`。
- 统一对比 HTML：`dataset/benchmark_004_esdf_001/comparison_027/viewer/index.html`。
- `YOPO_52`：碰撞率 `0.203%`、平均最大走廊偏差 `0.109 m`、平均进度 `6.642 m`。
- `YOPO_50`：碰撞率 `0.473%`、平均最大走廊偏差 `0.090 m`、平均进度 `6.603 m`；YOPO-Simple：碰撞率 `8.046%`、平均最大走廊偏差 `0.287 m`、平均进度 `6.225 m`。

<a id="chg-0024"></a>
## CHG-0024 中心线距离量化与 YOPO_54

- 评测和统一对比报告新增 `meanCenterlineDistanceM` 与 `meanMaximumCenterlineDistanceM`，按 3D witness 折线计算。
- `YOPO_53`（中心线权重 `2.0`）继续以 `1e-5` 学习率微调得到 `YOPO_54`。
- `YOPO_54`：平均中心线距离 `0.220 m`、平均最大中心线距离 `0.609 m`、平均最大走廊偏差 `0.057 m`、碰撞率 `0.203%`。
- 对比页面：`dataset/benchmark_004_esdf_001/comparison_029/viewer/index.html`。

<a id="chg-0025"></a>
## CHG-0025 更新 Batch 004 训练报告

- 新增 [TRAINING_REPORT_004.md](TRAINING_REPORT_004.md)，记录 `YOPO_54` 的中心线拟合训练、3D 中心线距离定义和原版 YOPO-Simple 配对结果。
- 文档中的所有 checkpoint、HTML 和 JSON 路径均对应当前实际产物；测试规模固定为 1479 条路线。

<a id="chg-0026"></a>
## CHG-0026 双模型 15-Primitive 三维对拍

- `evaluate_candidates.py` 现对同一 depth、motion、10 m goal 和 pose 同时保存
  Route-YOPO 与 YOPO-Simple 各 15 条、每条 101 个 XYZ 点，并记录 body/world 终态 `p/v/a`。
- viewer 增加同步 XY、XZ、YZ 投影、模型/安全/selected 开关和三维终态表：
  `dataset/benchmark_004_esdf_001/candidate_diagnostic_003/viewer/index.html`。
- 新增严格 3D 合同测试，覆盖 3x5 行列顺序、pitch/yaw、完整 p/v/a 解码、XYZ Poly5 和
  非零 yaw/pitch/roll 的 body-FLU/world-ENU 变换；未发现坐标系或 primitive 解码不一致。
- 发现独立根因：历史 Route-YOPO head 使用 `[depth, observation, route]`，原版契约是
  `[observation, depth]`。当前改为 `[observation, depth, route]`，旧 133-channel checkpoint
  加载时显式置换第一层权重以保持函数不变；checkpoint 新增 `feature_order` 元数据。
- 旧 Route checkpoint 的 optimizer moments 无法可靠迁移，禁止直接 resume，必须
  `--finetune` 或从原版 YOPO-Simple 权重重新初始化训练。
- 1479 条 route 上，Route-YOPO 三个输出行的平均 body z 为
  `+0.442/+0.024/-0.397 m`，上下层顺序正确；完整报告见
  `candidate_diagnostic_003/candidate_diagnostic_report.json`。
- 所有输入 goal body z 均为 `0`；YOPO-Simple selected 终点平均上抬 `0.451 m` 且未选择
  下层，Route-YOPO 为 `0.099 m`、下层占 `3.4%`。这是 checkpoint 的垂直选择偏置，不是
  z 轴符号或 world/body 变换错误。
- 使用说明：[CANDIDATE_DIAGNOSTIC.md](CANDIDATE_DIAGNOSTIC.md)。

<a id="chg-0027"></a>
## CHG-0027 修正通道顺序后的独立重训

- 从原版 YOPO-Simple `epoch50.pth` 重新初始化，使用 `[observation, depth, route]`
  的正确 head 契约；训练数据为 Batch 004 的 1479 条 route，RTX 3090。
- 第一阶段 15 epoch 得到 `saved_corrected/YOPO_0/best.pth`；第二阶段降低学习率续训
  20 epoch 得到 `saved_corrected/YOPO_1/best.pth`；随后冻结轨迹分支，仅校准 score head
  12 epoch 得到 `saved_corrected/YOPO_2/best.pth`。
- `YOPO_1` 全量评测：碰撞 `1.758%`、平均 3D 中心线距离 `0.373 m`、平均进度 `7.105 m`。
  score-only 校准后的 `YOPO_2`：碰撞 `1.623%`、中心线距离 `0.368 m`、进度 `7.020 m`。
- 新模型全部 15 候选中存在 `0%` 碰撞的安全 oracle，但 score-selected 碰撞仍为
  `1.623%`，匹配率 `5.1%`；当前瓶颈已定位为 score 目标/排序，而不是 3D 候选生成。
- 产物：[YOPO_1 对比 HTML](../dataset/benchmark_004_esdf_001/comparison_corrected_v1/viewer/index.html)、
  [YOPO_2 评测 HTML](../dataset/benchmark_004_esdf_001/evaluation_corrected_scorecal/index.html)、
  [YOPO_2 15-candidate 3D HTML](../dataset/benchmark_004_esdf_001/candidate_diagnostic_corrected_scorecal/viewer/index.html)。

<a id="chg-0028"></a>
## CHG-0028 迁移 YOPO_54 后的安全回归

- 将已训练的 `YOPO_54` 首层按 `depth_observation_route_v0 -> observation_depth_route_v1`
  迁移，在 Batch 004 以 `1e-5` 学习率微调 10 epoch，得到
  `saved_corrected/YOPO_5/best.pth`。
- 全量 1479 route：碰撞 `0.135%` (2/1479)、平均 3D 中心线距离 `0.309 m`、最大中心线
  距离 `0.786 m`、最大走廊越界 `0.099 m`、进度 `6.794 m`。相比 `YOPO_54` 的 `0.203%`
  碰撞有所改善，但中心线距离略增。
- 统一三模型对比：[comparison_corrected_migrated54/viewer/index.html](../dataset/benchmark_004_esdf_001/comparison_corrected_migrated54/viewer/index.html)。
- 全部 15 候选三维诊断：[candidate_diagnostic_corrected_migrated54/viewer/index.html](../dataset/benchmark_004_esdf_001/candidate_diagnostic_corrected_migrated54/viewer/index.html)。
- 冻结轨迹的 safety-binary ranking 和 total-cost pairwise ranking 均未降低碰撞，已作为
  负向实验保留，不纳入默认配置。当前安全 oracle 仍为 `0%` 碰撞，后续应在完整模型训练
  中改进 score target，而不是继续叠加独立 ranking 权重。

<a id="chg-0009"></a>
## CHG-0009 安全球融合 ESDF 场并只增加有序 path MSE

- 安全球半径继续使用与 `SafetyLoss` 相同的 3-D ESDF 生成。
- `path_corridor` 改为 ESDF 同形状的有符号指数场：安全球内为正 signed distance、球外为负并被推回球内，球内仍有连续梯度向安全体积中心收敛，
  不再使用“进入球内后代价恒为零”的 ReLU 越界项。
- bubble field 融合进原版 `safety` 项；训练总项恢复为 YOPO 的 smooth、safety、frontier、acceleration、score regression，
  只额外加入有序 witness `path_mse`。`path_corridor` 作为安全项的内部组成，同时继续单独记录诊断值。
- 关闭独立 safety score ranking，避免候选排序偏向短轨迹。
- 训练 CLI 新增 `--bubble-weight`，用于在不改动数据和网络的情况下标定反向 bubble field 的安全/进度权衡。
- GPU 对照：`YOPO_30`（bubble `0.05`）碰撞率 `0.135%`、平均进度 `4.103 m`；`YOPO_31`（bubble `0.01`）碰撞率
  `0.609%`、平均进度 `4.876 m`。相较 `YOPO_29`（`1.217%`、`4.493 m`），`0.01` 作为当前平衡默认值。
- 评测 HTML：`dataset/benchmark_004_esdf_001/evaluation_yopo30/index.html`、`evaluation_yopo31/index.html`、
  `comparison_019/viewer/index.html`。

<a id="chg-0010"></a>
## CHG-0010 恢复 witness 终点前进约束

- 诊断确认安全球覆盖范围远大于模型终点，问题是候选没有纵向进度要求，而不是可行空间不足。
- 将已有的有序 `path_progress` 短缺代价正式加入总 loss 和 detached score target；它约束终点沿 witness
  弧长达到不超过 `10 m` 的 local-subgoal。
- 配置 `wprogress=0.2`、`wprogress_floor=0`：保留连续前进信号，避免 progress floor 形成过强硬约束。
- bubble/ESDF 仍负责安全，`path_mse` 负责中心线，progress 负责到达更远的可行终点。
- GPU 对照：`YOPO_32`（progress `0.2`）进度 `5.859 m`、碰撞率 `1.487%`；`YOPO_33`（progress `0.1`）进度
  `5.622 m`、碰撞率 `0.811%`。综合安全和进度，默认配置固定为 `wprogress=0.1`。
- 最终对比 HTML：`dataset/benchmark_004_esdf_001/comparison_020/viewer/index.html`。

<a id="chg-0012"></a>
## CHG-0012 保持 YOPO 原始 30 点采样并复训验证

- 确认 `10 m` 表示 witness 弧长投影范围，不改变 YOPO 原有的 `30` 个轨迹采样点；当前 `path_mse`
  仍使用 30 点均匀时间采样与前 10 m witness 的有序几何目标。
- 从 `YOPO_33` 继续训练得到 `YOPO_35`，全量 1479 条路线结果为碰撞率 `0.947%`、平均进度 `5.467 m`，
  未超过 `YOPO_33`（`0.811%`、`5.622 m`），因此 `YOPO_33` 保持当前最佳模型。
- `YOPO_34` 是中断的临时十点采样实验，不作为有效 checkpoint。
- 对比 HTML：`dataset/benchmark_004_esdf_001/comparison_021/viewer/index.html`。

<a id="chg-0013"></a>
## CHG-0013 提高已审计 witness 中心线权重

- 观察到模型输出虽处于安全空间，但与 witness 中心线偏离明显；由于 witness 已通过连续 ESDF clearance 审计，
  将 `wpath_mse` 从 `0.05` 提高到 `0.2`。
- 新增训练参数 `--path-mse-weight`，便于复现实验；bubble/ESDF safety 权重不变。

<a id="chg-0014"></a>
## CHG-0014 高 path MSE 实验与根因定位

- `YOPO_36`（`wpath_mse=0.2`）和 `YOPO_38`（`wpath_mse=1.0`）验证了简单增大全 3D path MSE
  不会使轨迹贴线：碰撞率分别为 `2.164%` 和 `10.345%`，中心线平均偏差未下降。
- benchmark witness 中心线连续点云净空最小 `0.706 m`，因此路线标签本身安全；问题来自全时间点 GT
  强制拟合与单段五次多项式的动力学约束冲突，弯道处会过冲/切弯。
- 默认权重恢复为已验证的 `0.05`。后续应改用按有序 witness segment 的 cross-track loss，再重新标定权重，
  不把高权重全 3D MSE 作为发布配置。

<a id="chg-0015"></a>
## CHG-0015 30 点 witness 几何目标与候选选择诊断

- 保持 YOPO 原始 30 个轨迹采样点；`path_mse` 使用 witness 前 10 m 的有序 30 点几何目标，
  不改变模型点数和数据合同。
- `YOPO_39` 的 cross-track 实验未改善中心线偏差，已撤回该实验性定义。
- `YOPO_40` 的 full-cost score ranking 实验仍显示候选选择问题：安全候选存在率 `100%`、oracle 碰撞率
  `0%`，但模型选择碰撞率 `2.299%`。高 path 权重和 ranking 不作为默认配置。


<a id="chg-0007"></a>
## CHG-0007 有序 witness path MSE 与安全通道统一

- 新增弱 `path_mse`：按 witness 弧长将轨迹采样点与 local-subgoal（10 m）范围内的有序路径对齐，
  只提供几何引导，不替代 YOPO 的动力学目标。
- 保留 `path_corridor` bubble 惩罚和原版 3D ESDF safety；三者职责分别为路径方向、安全球边界和实际障碍安全。
- 当前配置 `wpath_mse=0.05`、`wp=0.05`，安全权威仍为 `SafetyLoss` 的 3D ESDF。
- 后续数据批次应使用与 `SafetyLoss` 相同的 3D ESDF 采样生成 bubble 半径；当前旧数据的半径仍来自 PLY 最近点，
  可能造成标签尺度差异。

<a id="chg-0008"></a>
## CHG-0008 Bubble 半径对齐 3D ESDF

- 新增 `data/rebuild_esdf_bubbles.py`，使用与 `SafetyLoss` 相同的 0.2 m 体素化、占据栅格、正负 ESDF
  和坐标原点，在已有 witness path/topology 点上重采样 clearance 与 bubble radius。
- 生成数据 `dataset/pilot_esdf_001`，共 3 个场景、1855 条路线；新的最小 clearance 为 `0.587 m`，
  无原有效路线因 ESDF 对齐被淘汰。
- 新旧 route clearance 均保持同一安全定义：`radius = esdf_clearance - 0.5 m`；训练仍使用原版 3D ESDF
  safety，加弱 bubble corridor 和弱 ordered path MSE。
- 数据合同、48 项测试和 CPU 单批次反向传播均通过；GPU 可用性曾短暂受容器设备映射影响，后已在宿主权限下恢复。
- GPU 恢复后完成 `YOPO_29`：使用 `pilot_esdf_001` 训练（关闭 score ranking）。在同样 ESDF 对齐的
  `benchmark_004_esdf_001` 上，碰撞率 `1.22%`（18/1479），平均轨迹长度 `4.69 m`，大方块场景碰撞率 `0%`。
  安全显著改善但路径仍偏短，说明 ESDF 对齐解决了安全标签冲突，却没有解决 score/目标进度偏置。

<a id="chg-0006"></a>
## CHG-0006 收敛为原版 YOPO 加单一路由安全通道

- 训练目标恢复为 YOPO-Simple 原有的 smooth、ESDF safety、frontier、acceleration 和 score regression。
- 唯一新增训练项为 `path_corridor`：使用 witness bubbles 的安全半径惩罚轨迹越出安全通道。
- `path_centerline`、progress、tangent、progress floor 和候选 ranking 保留为诊断/API 兼容项，但不再进入总 loss。
- 中止了多目标 centerline 实验训练；避免路线监督压过原版动力学/目标优化。
- 代码测试：`48 passed`（随后定向回归 `9 passed`）。
- `YOPO_22` 是关闭 peak 的中间对照；`YOPO_23` 使用最终配置 `wp=0.05`、peak=2.0。三场景
  1479 条路线评测结果为碰撞率 `5.27%`、平均轨迹长度 `6.05 m`、平均通道违例 `0.073 m`。
  结果已保存至 [comparison_014](../dataset/benchmark_004/comparison_014/viewer/index.html)。
- `YOPO_20`/`YOPO_21` 作为通道权重过强的对照保留，不作为最终模型：二者分别出现明显停滞或
  平均长度下降，说明新增安全通道必须保持弱权重。
- 新增候选级诊断工具 `tools/diagnose_candidate_selection.py`。对 `YOPO_23` 的 1479 条路线，
  每条输入均存在至少一个 clearance >= `0.5 m` 的安全 primitive；按候选真实点云 clearance
  选择的 oracle 碰撞率为 `0%`，而模型 score 选择为 `5.27%`。因此当前主要问题是 score head
  排序/标定，而不是网络无法生成安全轨迹。诊断结果保存于
  `dataset/benchmark_004/comparison_014/candidate_selection_diagnosis.json`。
- `YOPO_24` 冻结终态、训练 ESDF safety score ranking 后，碰撞率降至 `1.69%`，但平均长度降至
  `3.29 m`；`YOPO_25` 改为二元 unsafe/safe ranking，碰撞率回升至 `3.38%`、长度 `4.71 m`。
  这表明 score 排序优化有效但会产生安全偏置，后续应采用安全约束下的长度/目标联合标定，
  不再直接提高 ranking 权重。
- `YOPO_26`/`YOPO_27` 的 score-only 对照没有改善该权衡，保留作实验记录，不作为当前模型。

<a id="chg-0005"></a>
## CHG-0005 安全优先候选排序优化

- 记录时间：2026-08-27
- 新增完整轨迹总代价的候选 pairwise ranking loss，避免 score 回归数值正确但候选排序错误。
- 新增独立 safety-barrier ranking loss，优先把进入 `0.5 m` required-clearance 边界的候选排到安全候选之后。
- ranking 权重和 margin 写入 checkpoint 与 `run.json`，保证实验可复现。
- `YOPO_13`（完整代价排序）和 `YOPO_14`（安全排序权重 4.0）已训练；`YOPO_14` 在
  `benchmark_004` 的 1479 条路线中碰撞率 `4.87%`、平均轨迹长度 `6.43 m`，作为当前平衡候选。
- `YOPO_15` 使用安全排序权重 8.0，碰撞率降至 `3.79%`、平均长度 `6.21 m`，作为安全优先候选，
  但不作为满足长度目标的最终模型。
- 配对评测 HTML：[comparison_005](../dataset/benchmark_004/comparison_005/viewer/index.html)、
  [comparison_006](../dataset/benchmark_004/comparison_006/viewer/index.html)、
  [comparison_007](../dataset/benchmark_004/comparison_007/viewer/index.html)。

- 修复 viewer 初始化时 `fixed` 函数声明顺序错误导致页面空白的问题；同时修正比较 viewer
  从 `comparison_xxx/viewer` 加载 RGB 的相对路径。三个 comparison viewer 已重新生成并验证。

<a id="chg-0004"></a>
## CHG-0004 固定 10 m local subgoal 与速度恢复训练

- 记录时间：2026-08-27
- 状态：V2 数据合同、GPU 训练、独立三场景配对评测和 HTML 完成
- 训练数据：[pilot_003](../dataset/pilot_003)
- 测试数据：[benchmark_004](../dataset/benchmark_004)
- 最终 checkpoint：`saved/YOPO_10/epoch12.pth`
- 模型报告：[comparison_003/comparison_report.json](../dataset/benchmark_004/comparison_003/comparison_report.json)
- HTML：[comparison_003/viewer/index.html](../dataset/benchmark_004/comparison_003/viewer/index.html)

问题与修复：

- 旧批次把完整 witness 末端（平均约 13 m、最高约 18 m 的欧氏 goal）同时用于
  `frontier_goal_world` 和 YOPO goal loss；这与 YOPO 单 primitive 的 10 m subgoal 合同不一致。
- V2 保留 `frontier_goal_world` 为完整 witness/frontier 终点，新增 `local_subgoal_world` 和
  `local_subgoal_distance_m`。local goal 沿完整 witness 弧长插值，精确为 10 m；witness
  corridor 仍可延伸到 20--30 m。V1 route table 仍可读取并回退旧字段。
- 派生工具过滤 witness 短于 10 m 的路线；源批次只读复制，旧报告和旧 routes.npz 不被修改。
- 进度 floor 微调到 `6.8 m`、权重 `0.7`；仍保留 corridor union、progress、tangent、ESDF
  safety 和原有 `wg=0.15` goal loss，不使用行为克隆。

配对结果（1479 条，零初始速度/加速度，三模型逐样本共享 depth、pose、10 m local goal）：

| 模型 | 碰撞率 | Corridor violation | 平均轨迹长度 | 平均速度 |
|---|---:|---:|---:|---:|
| YOPO_10 | 6.63% (98) | 26.57% | 6.681 m | 4.009 m/s |
| YOPO_9 | 5.68% (84) | 24.41% | 6.438 m | 3.863 m/s |
| YOPO-Simple 3x5 | 7.98% (118) | 47.67% | 6.191 m | 3.715 m/s |

分场景平均轨迹长度（Route-YOPO - YOPO-Simple）：圆柱 YOPO 森林 `+0.055 m`，原始
`tree.ply` 真实树 `+0.640 m`，30 m Map2 大方块 `+0.779 m`。最终节点三个场景均不短于
YOPO-Simple；单次 primitive 仍受 `10 m` endpoint radius 限制，20--30 m 只属于完整
witness 或多次滚动累计路径。

<a id="chg-0003"></a>
## CHG-0003 修复后三场景重训与新旧三模型配对评测

- 记录时间：2026-08-27
- 状态：正式训练数据、两阶段训练、独立三场景全量评测、HTML 和自动化测试完成
- 训练报告：[TRAINING_REPORT_002.md](TRAINING_REPORT_002.md)
- 数据报告：[benchmark_003 generation report](../dataset/benchmark_003/generation_report.json)
- 模型报告：[benchmark_003 comparison report](../dataset/benchmark_003/comparison_002/comparison_report.json)

问题与修改：

- 使用 CHG-0002 修复后的 labeler 生成 `pilot_002`，不覆盖旧 `pilot_001`。三个场景分别为
  YOPO 原版参数的圆柱近似树林、原始 `tree.ply` 实例树林和最大边长 30 m 的 Map2 方块。
- 训练启动审计发现旧 `YOPODataset` 在场景数 `>=3` 时把最后一个场景整场作为 validation，
  导致大方块不参与训练。现统一为每个场景内部按 frame 分组留出 10%；同一 depth frame
  的多条路线不跨 split，三个场景均为 train `675` / valid `75` 条。
- `pilot_002` 共 2250 条路线；连续最小 clearance `0.630 m`、Bubble overlap 最小余量
  `0.262 m`、最大搜索绕行比 `1.11984 < 1.12`，数据合同校验通过。
- 第一阶段 `YOPO_5` 从上一正式版 `YOPO_3/best.pth` 加载权重、重置 optimizer，以
  `wprogress=0.8` 训练 30 epoch；best 为 epoch 11，validation selected cost `5.36600`。
- 第一阶段全量测试安全提升但真实树进度偏短，因此保持 corridor `wp=2.0` 不变，将
  `wprogress` 调到 `1.2`，以半学习率继续训练 15 epoch。最终 `YOPO_6/best.pth` 为
  第二阶段 epoch 10。
- `compare_yopo.py` 支持当前 Route-YOPO、上一 Route-YOPO 和 YOPO-Simple 三模型逐样本
  配对；HTML 增加绿色上一版本轨迹及单独开关/指标。三模型共享 depth、pose、零 motion
  和 frontier goal，两个 Route-YOPO 还共享相同 witness bubbles。
- 新增 `run_pilot_002.sh`、`run_training_002.sh` 和 `run_benchmark_003.sh`；训练 CLI 增加
  `--progress-weight`，checkpoint 和 run metadata 记录实际 loss 权重及训练参数。

验证结果：

- `benchmark_003` 使用独立 seed `820003`，三个场景各 200 帧、600 条路线，总计 1800 条；
  连续最小 clearance `0.680 m`、Bubble overlap 最小余量 `0.363 m`、最大绕行比
  `1.11990 < 1.12`。
- 最终 YOPO_6 为 6 次碰撞（`0.33%`）、corridor violation `8.67%`、平均最小净空
  `1.358 m`、平均进度 `4.230 m`。
- 上一正式版 YOPO_3 为 73 次碰撞（`4.06%`）、corridor violation `16.61%`、平均最小
  净空 `1.251 m`、平均进度 `4.367 m`。新模型显著降低碰撞和走廊违例，平均进度降低
  `0.137 m`，不将其表述为所有指标全面胜出。
- YOPO-Simple 3x5 为 140 次碰撞（`7.78%`）、corridor violation `54.33%`、平均进度
  `6.117 m`。真实树场景碰撞率分别为 YOPO_6 `0.33%`、YOPO_3 `11.00%`、
  YOPO-Simple `19.33%`。
- 三模型 HTML：[benchmark_003 viewer](../dataset/benchmark_003/comparison_002/viewer/index.html)。
- `train_scalenav/tests` 最终为 `45 passed`；两个正式数据集合同校验、Python 编译、三个
  可复现脚本 Bash 语法及 `git diff --check` 通过。

限制与后续工作：

- 本结果为零初始速度/加速度的单步离线测试，不是闭环飞行成功率；YOPO-Simple 进度更长。
- 本批仍只修改训练和离线评测。ScaleNav 在线 C++ M4 的等价路线搜索/中心线逻辑及
  Python/C++ 固定地图对拍仍需单独建立上游 CHG 编号。

<a id="chg-0002"></a>
## CHG-0002 Witness 伪细腰修复与受限 widest-shortest 搜索

- 记录时间：2026-08-27
- 状态：训练侧代码、新编号数据、HTML、旧 checkpoint 离线验证和自动化测试完成；正式重训与配对评测已由 CHG-0003 完成
- 设计记录：[ARCHITECTURE.md](ARCHITECTURE.md)

问题与根因：

- 现有合成路线图的边权只有欧氏步长，A*/Dijkstra 只找最短路，不最大化路线瓶颈
  clearance；存在略长但明显更宽的同拓扑路线时，witness 仍可能贴障。
- 原始 A* 使用 `planning_occupancy`，但 Chaikin/Spline 平滑使用更小的
  `smoothing_occupancy`，会把已满足规划 margin 的折线重新拉向障碍。
- `build_witness_corridor()` 直接以 witness 点作为 Bubble 圆心，没有中心线净空优化；
  固定 K 采样又会优先保留局部最小半径，因此伪细腰会稳定进入模型输入和 corridor loss。
- `RouteLoss` 按安全 Bubble 并集惩罚是正确行为。放宽 loss 只会隐藏错误标签，不属于
  本批修复方案。

修改内容：

- 增加固定方块伪细腰 fixture、单侧贴障中心线 fixture，同时保留真实窄通道反例。
- 对每个候选 frontier 先求最短安全长度，再在
  `route_length <= detour_ratio * shortest_safe_length` 约束内选择最高 clearance 门槛路线；
  同门槛下最小化 clearance risk 和长度。
- 平滑、折线短化和最终连续 segment 审计统一使用同一套
  `robot_radius + safety_margin + planning_extra_margin`。
- 固定起点、frontier 和受保护转折点，只移动 witness 中间点；沿 clearance 梯度中心化后
  重新计算 corridor 圆心、clearance 和半径。
- 质量报告增加 minimum safe radius、P05、neck length、Bubble overlap margin、搜索绕行比
  和中心线优化增益；新数据使用新批次号，不覆盖 `pilot_001/test_001/benchmark_001/002`。

当前验证结果：

- 可绕开的固定方块场景中，旧最短路 P05 安全半径 `<0.8 m`；受限 widest-shortest
  提升到 `>=1.1 m`，路径长度保持在 `1.12x` 预算内。
- 真实窄通道 fixture 在 `1.08x` 长度预算下继续穿过原通道，没有为了追求宽度跨拓扑远绕。
- 单侧贴障中心线固定首尾点，将 P05 安全半径从 `0.60 m` 提升到约 `0.80 m`，同时降低
  clearance risk；对称窄通道横向位移小于 `0.05 m`，未虚构自由空间。
- 搜索净空门槛采用单调二分，并复用候选已有 Dijkstra 最短路径；同组地面测试耗时由
  约 `26.6 s` 降至约 `19.8--21.8 s`。
- `test_route_contract.py` 与 `test_ground_truth_dataset.py` 当前合计 `18 passed`；新
  `routes.npz` 审计字段保持可选，旧 batch 可继续读取。
- 进一步发现解析树林的旧 occupancy 按“栅格中心是否落入圆柱”离散；树干半径小于
  `0.2 m` 栅格时可能完全漏掉，导致搜索 clearance 与最终点云 KD-tree clearance 不一致。
  现改为直接计算栅格中心到圆柱/旋转方块真实表面的解析距离；真实树场景查询原始树点云
  截面。新增亚栅格树干回归，`0.06 m` 半径树不会再被漏栅格化。
- 新编号 `test_002` 使用 seed `902002`，包含圆柱树林和大方块各 5 帧、10 条路线。
  大方块最差中段安全半径 `1.023 m`、P05 最低 `1.062 m`；圆柱树林最差中段安全半径
  `0.610 m`，对应最大搜索绕行比 `1.114`，仍低于 `1.12` 硬预算。
- 旧 `benchmark_001` 的大方块 P05 安全半径均值约 `0.635 m`，`test_002` 为
  `1.097 m`；圆柱树林由约 `0.384 m` 提升到 `0.913 m`。由于 seed 和样本规模不同，
  该结果用于标签分布审计，不作为模型效果的严格配对显著性结论。
- 旧 checkpoint 在 `test_002` 的 20 条路线为零碰撞，最小轨迹 clearance `0.699 m`、
  平均进度 `4.558 m`。数据 HTML：[test_002 viewer](../dataset/test_002/viewer/index.html)；
  模型输出 HTML：[test_002 model output](../dataset/test_002/model_eval_001/index.html)。
- 最终 `train_scalenav/tests` 为 `44 passed`；Python 语法检查、脚本语法、数据契约和
  `git diff --check` 通过。

后续工作：

- 正式新训练批次、分布审计和新模型训练已在 CHG-0003 完成，且未覆盖 `pilot_001`。
- 本批只修改训练和离线 labeler。ScaleNav 在线 M4 的等价 widest-shortest/中心线实现需
  单独建立上游 CHG 编号并做 C++/Python 固定地图对拍，不能用 Python 训练修复冒充在线生效。

验收条件：

- 固定伪细腰场景瓶颈半径明显增加，路径长度不超过配置的 `detour_ratio`。
- 固定真实窄通道不得报告虚假净空提升；起点和 frontier 不移动。
- 所有最终 witness segment 通过统一 planning margin，Bubble 无断连。
- 旧 checkpoint 先在新 corridor 上离线评估，再决定是否重新训练。

<a id="chg-0001"></a>
## CHG-0001 YOPO-Simple 原始树资产场景与三场景配对基准

- 记录时间：2026-08-27
- 状态：代码、数据生成、HTML、配对模型评测和自动化测试完成
- 数据报告：[benchmark_002 generation report](../dataset/benchmark_002/generation_report.json)
- 模型报告：[benchmark_002 comparison report](../dataset/benchmark_002/comparison_001/comparison_report.json)

问题与修改：

- 原 `yopo_forest` 只复用 YOPO-Simple `maze_type=5` 的 4 m 抖动布局和 `0.5--1.0`
  缩放分布，每棵树实际由解析圆柱代替，不能称为原版真实树场景。
- 新增 `yopo_real_forest`：读取 YOPO-Simple 原始二进制 `tree.ply`，体素降采样后按 4 m
  布局生成 400 个带随机 roll/pitch/yaw 和缩放的实例。
- 固定高度 occupancy、KD-tree clearance、深度 z-buffer 和导出 `tree.ply` 全部使用实例化
  后的真实树干、树枝和树冠点，不以圆柱替代训练几何。
- 路线采样单独保存树实例中心作为 blocker direction；它只用于产生绕行朝向，不替代
  点云碰撞和 clearance。
- 新增 `run_benchmark_002.sh`，固定生成圆柱近似树林、真实树树林和 Map2 大方块三个场景；
  Route-YOPO 与 YOPO-Simple 逐样本使用相同 depth、零 motion 和 frontier goal。

验证结果：

- `benchmark_002` 包含 3 个场景、600 帧、1800 条路线；每场景 200 帧、600 条路线。
- 真实树场景包含 400 棵树、`4,426,889` 个导出障碍点；原始资产 SHA-256 为
  `10a47b4d0de50fbafefbb0e26426091ce9fbd2c2bc62b166b233a37749ea66bb`。
- 当前 checkpoint 在真实树场景的碰撞率为 Route-YOPO `17.33%`、YOPO-Simple
  `20.33%`；corridor violation rate 分别为 `30.00%`、`61.50%`。该分布必须与圆柱树林
  分开报告。
- 数据 HTML：[ground-truth viewer](../dataset/benchmark_002/viewer/index.html)；模型输出 HTML：
  [paired model viewer](../dataset/benchmark_002/comparison_001/viewer/index.html)。
- `train_scalenav/tests` 为 `39 passed`；脚本语法、数据契约和 `git diff --check` 通过。

限制：

- 使用的是原始树点云资产和原版布局参数，但随机数引擎与深度渲染由 Python 实现，
  不是原 CUDA 数据生成器的逐位复现。
