# Route-YOPO 方向调研与推荐方案

日期：2026-08-31

## 结论

当前“引导线夹角 loss + 中心线 MSE”不是合适的主方向。Route-YOPO 应继续使用五次多项式和
YOPO 的 15 个候选 primitive，把 EPIC/ScaleNav 的路线解释为**有序安全走廊和任务方向先验**，
而不是一条必须按时间拟合的专家轨迹。

推荐的主目标为：

```text
原版 YOPO：smooth + ESDF safety + frontier + acceleration + score regression
Route-YOPO：在 safety 中增加 corridor barrier（轨迹越出 Bubble 才受罚）
```

暂时移除 angle、centerline MSE、SafetyLoss 中的 attraction，以及独立的 ranking 权重。等
路线标签和基础排序在固定口径下稳定后，再单独做 score calibration/ranking 实验。

## 为什么原方向不对

### 1. YOPO 的训练对象是候选集合，不是单条 GT 轨迹

原版 YOPO 的网络同时预测 15 个动态 primitive 的终态和 cost，执行时选择 cost 最小的候选。
原论文的核心是用可微的 smooth/safety/goal 代价训练候选和 score，而不是将所有候选回归到
同一条路径：

- Lu et al., *You Only Plan Once: A Learning-Based One-Stage Planner With Guidance Learning*,
  IEEE RA-L 2024, DOI: `10.1109/LRA.2024.3399589`。
- YOPO-Rally 继续保留“multiple trajectory candidates with costs”，并明确指出全局路径问题
  具有多模态解；见 <https://arxiv.org/abs/2505.18714>。

因此，同一个安全走廊内的左偏、右偏、减速和提前停车都可能是合法解。中心线 MSE 会把这些
解错误地压成一个点对点答案。

### 2. 夹角只看起点到终点的 chord，不能表达弯道

当前 `RouteLoss` 的 angle 项只比较：

```text
start -> route_points[-1]
start -> predicted_position[-1]
```

路线在中间发生转弯时，终点 chord 方向可能与局部可行方向不同。它可以让终点方向变好，
但不保证中间 101 个轨迹点在路线走廊内。

### 3. 中心线 MSE 与单段五次多项式的时间参数不一致

当前 MSE 将多项式采样点按归一化弧长同步到 witness。单段五次多项式同时受起点速度/加速度、
终点状态和固定时长约束，弯道处通常无法精确满足这个时间对应关系。增大权重只会制造过冲、
切弯或提前减速，并不等价于更安全。

### 4. 实验已经把瓶颈分开了

候选诊断显示当前 corrected Route-YOPO 在 `benchmark_004_esdf_001` 上：

```text
selected collision       0.135% (2/1479)
centerline oracle        0%
selected/oracle match    6.6%
```

这说明 15 个 primitive 中已有安全候选，剩余问题主要是 score 选择和代价定义，而不是
“模型不会生成贴路线的轨迹”。对应报告：
`dataset/benchmark_004_esdf_001/candidate_diagnostic_corrected_migrated54/candidate_diagnostic_report.json`。

相反，900-route 的独立消融中，中心线 MSE-only 碰撞率为 `14.67%`，高于 angle-only 的
`2.67%`；两者使用 zero-motion 口径，不能与 paired deterministic 结果混比，但足以说明
MSE 不是安全约束。历史变更记录也显示高 path-MSE 权重会升高碰撞，见 `docs/CHANGELOG.md`
的 CHG-0014/CHG-0015。

## 推荐架构

### A. 路线数据先保证是安全走廊

1. 搜索使用受限 widest-shortest：先满足连续 clearance，再在 detour 预算内最大化
   bottleneck clearance，不能只按栅格长度 A*。
2. witness 中心线优化固定起点、终点和拓扑分支，只沿真实 clearance 梯度小步调整；每次
   调整后重新做连续线段安全检查。
3. 用同一条最终 witness 生成 dense center、raw clearance 和
   `safe_radius = clearance - robot_radius - safety_margin`。模型输入可以裁剪半径，质量门和
   loss 不得使用裁剪值替代真值。
4. 修复平滑阶段的安全门，禁止平滑使用比搜索更小的 occupancy margin。

仓库已有这套数据层设计，但当前合成搜索仍可能产生细腰；详见 `docs/ARCHITECTURE.md`
的 S0-S3 和“当前问题定位”。在数据未通过 neck/r_min/continuous-clearance 统计前，不应
继续训练新的路线 loss。

### B. 网络输入保持路线条件，不改变 primitive 表达

保留：

```text
depth + 9-D motion/goal observation + 12 个 ordered route bubbles(center, safe_radius)
```

继续使用原版 3x5 五次多项式 primitive。不要改 spline，也不要把完整 witness 当作带时间戳
的行为克隆目标。

### C. Loss 只增加“走廊可行性”

对每个候选的 30/101 个多项式采样点，计算到 Bubble 并集的 signed distance：

```text
d_corridor(p) = min_k(||p - c_k|| - r_k)
L_corridor = mean(relu(d_corridor(p))^2) + lambda_peak * max(relu(d_corridor(p)))^2
```

该项只惩罚越出安全走廊的部分；在走廊内部不强迫轨迹贴中心线，因此保留多解和减速能力。
它应并入 safety cost，和真实 3-D ESDF safety 使用同一采样/安全半径定义。

默认总 loss：

```text
L = L_smooth + L_ESDF + L_corridor + L_frontier + L_acc + L_score
```

先将 `wp=0`、`wcenterline=0`、`safety_route_attraction_weight=0`。progress 只能在基础
安全版本稳定后作为很小的独立实验项，不能和中心线回归同时引入。

### D. Score 训练分两步

1. 先用 detached 的真实候选总代价训练原版 score regression，确认 selected/oracle/regret
   口径和原版一致。
2. 如果安全 oracle 仍为 0% 而 selected 有碰撞，再单独评估 score calibration 或带安全
   margin 的 pairwise ranking。历史冻结轨迹 ranking 实验会牺牲进度，不能直接作为默认项。

训练选择标准不能只看 validation total loss；必须同时记录 selected collision、oracle
collision、selection regret、top-1、corridor violation 和 progress。

## 最小验证矩阵

在同一 `train_large_001`、同一 valid split、同一 deterministic motion 下只跑小规模训练：

| 实验 | 路线数据 | 路线项 | 用途 |
|---|---|---|---|
| A | 当前数据 | 无（原版 YOPO + route 输入） | 确认通道输入本身是否改变原版行为 |
| B | 当前数据 | corridor barrier only | 验证路线作为安全走廊是否有效 |
| C | 修复后数据 | corridor barrier only | 区分标签细腰与模型损失问题 |
| D | 修复后数据 | B + score calibration | 只解决候选排序，不改变轨迹生成 |

每组先跑 3-5 epoch smoke 和完整 valid，再决定是否扩展到 12-20 epoch。验收门槛：

- `oracle collision` 必须为 0 或显式报告候选生成失败；
- `selected collision` 不得以显著降低 progress 为代价下降；
- 按 `safe_radius_p05` 和 `neck_length` 分桶报告碰撞率；
- 训练安全采样与离线评测至少同量级，推荐 101 点或连续线段下界检查；
- 所有模型对比使用完全相同的 depth、motion、frontier 和点云 clearance 查询。

## 当前执行顺序

1. 固定并审计路线生成：r_min、P05、neck length、连续线段 clearance。
2. 恢复/实现 corridor barrier-only 版本，关闭 angle、centerline MSE、attraction。
3. 用原版 checkpoint 做 A/B 小实验，先看 oracle 和 selected 的差距。
4. 只有在 B/C 稳定后才做 score calibration；不再盲目叠加新的几何回归项。
