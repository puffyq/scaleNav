# Route-YOPO Batch 003：10 m Local Subgoal 训练报告

| 项目 | 内容 |
|---|---|
| 训练数据 | `dataset/pilot_003`，1855 条有效路线 |
| 独立测试数据 | `dataset/benchmark_004`，1479 条有效路线 |
| 最终 checkpoint | `saved/YOPO_10/epoch12.pth` |
| 输入合同 | 相同 depth、pose、零 motion、10 m local subgoal；Route-YOPO 额外接收完整 witness bubbles |
| 单步 primitive | endpoint radius 0--10 m，`sgm_time=1.67 s` |

完整 witness 保留路线搜索得到的 10--30 m 路径。`frontier_goal_world` 是 witness 的真实终点；
`local_subgoal_world` 是从起点沿 witness 弧长插值 10 m 的 YOPO 局部目标。goal loss 只消费
local subgoal，corridor/progress/tangent loss 消费完整有序 witness。V2 数据过滤短于 10 m 的
路线，因此本报告所有样本 local subgoal distance 都是 10.0 m。

第一阶段从 `YOPO_6/best.pth` 在 pilot_003 训练 25 epoch，floor `6.4 m`、权重 `0.5`，
生成 `YOPO_9`。第二阶段从 `YOPO_9/epoch5.pth` 微调 15 epoch，floor `6.8 m`、权重 `0.7`，
生成 `YOPO_10`。最终节点依据独立测试集的三场景长度、安全和 corridor 联合门槛选择。

| 模型 | 碰撞率 | Corridor violation | 平均最小净空 | 平均进度 | 平均轨迹长度 | 平均速度 |
|---|---:|---:|---:|---:|---:|---:|
| YOPO_10 | 6.63% (98/1479) | 26.57% | 1.197 m | 6.597 m | 6.681 m | 4.009 m/s |
| YOPO_9 | 5.68% (84/1479) | 24.41% | 1.220 m | 6.361 m | 6.438 m | 3.863 m/s |
| YOPO-Simple 3x5 | 7.98% (118/1479) | 47.67% | 1.157 m | 6.225 m | 6.191 m | 3.715 m/s |

| 场景 | YOPO_10 长度 | YOPO-Simple 长度 | 差值 | YOPO_10 碰撞率 |
|---|---:|---:|---:|---:|
| 圆柱 YOPO 森林 | 6.455 m | 6.401 m | +0.055 m | 7.46% |
| 原始 `tree.ply` 真实树 | 6.281 m | 5.641 m | +0.640 m | 12.47% |
| 30 m Map2 大方块 | 7.303 m | 6.525 m | +0.779 m | 0.00% |

产物：[数据 viewer](../dataset/benchmark_004/viewer/index.html)、[模型 viewer](../dataset/benchmark_004/comparison_003/viewer/index.html)、[JSON 报告](../dataset/benchmark_004/comparison_003/comparison_report.json)。这是单步离线评测；20--30 m 是完整 witness 或滚动多段 primitive 的累计路径，不是单次 primitive 长度。
