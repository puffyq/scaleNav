# Route-YOPO Batch 002 训练与离线评测报告

| 项目 | 内容 |
|---|---|
| 训练数据 | `dataset/pilot_002` |
| 独立测试数据 | `dataset/benchmark_003` |
| 最终 checkpoint | `saved/YOPO_6/best.pth` |
| 上一正式版本 | `saved/YOPO_3/best.pth` |
| YOPO-Simple | `YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth` |
| 测试假设 | 单步离线、零初始速度和加速度 |

## 1. 数据

`pilot_002` 使用 seed `502002`，包含圆柱树林、YOPO-Simple 原始树点云树林和 Map2
大方块三个场景，各 250 帧、750 条路线，总计 2250 条。训练/验证按每个场景内部的
frame 分组为 2025/225 条；同一 depth frame 的多条路线不会跨 split。

数据合同审计结果：连续最小 clearance 为 `0.630 m`，Bubble overlap 最小余量为
`0.262 m`，最大 search detour ratio 为 `1.11984 < 1.12`。所有写入训练集的路线通过
连续 clearance、曲率、lattice 方向和 Bubble 连通质量门。

## 2. 训练

第一阶段从 `YOPO_3/best.pth` 加载模型权重并重置 optimizer，以 `wprogress=0.8` 训练
30 epoch。`YOPO_5/best.pth` 来自 epoch 11，validation selected cost 为 `5.36600`。

第一阶段显著降低碰撞，但平均进度降到 `3.921 m`。第二阶段保持 corridor 权重
`wp=2.0`，只将 `wprogress` 从 `0.8` 调到 `1.2`，从 YOPO_5 以 `7.5e-5` 学习率训练
15 epoch。最终 `YOPO_6/best.pth` 来自第二阶段 epoch 10。

## 3. 配对测试

`benchmark_003` 使用独立 seed `820003`，三个场景各 600 条，共 1800 条。三模型逐样本
共享 depth、pose、零 motion 和 frontier goal；两个 Route-YOPO 还共享同一组 witness
bubbles。指标来自同一 41 点五次多项式采样和同一障碍点云 clearance 查询。

| 模型 | 碰撞率 | Corridor violation | 平均最小净空 | 平均进度 |
|---|---:|---:|---:|---:|
| YOPO_6（最终） | 0.33% (6/1800) | 8.67% | 1.358 m | 4.230 m |
| YOPO_3（上一正式版） | 4.06% (73/1800) | 16.61% | 1.251 m | 4.367 m |
| YOPO-Simple 3x5 | 7.78% (140/1800) | 54.33% | 1.146 m | 6.117 m |

分场景碰撞率：

| 场景 | YOPO_6 | YOPO_3 | YOPO-Simple |
|---|---:|---:|---:|
| 圆柱树林 | 0.67% | 1.17% | 4.00% |
| 原始树点云树林 | 0.33% | 11.00% | 19.33% |
| 30 m 大方块 | 0.00% | 0.00% | 0.00% |

## 4. 结论与限制

YOPO_6 相比上一正式版本将碰撞从 73 次降到 6 次，将 corridor violation 降低约一半，
平均进度只下降 `0.137 m`。相比 YOPO-Simple 安全性和走廊一致性明显更高，但
YOPO-Simple 的平均进度更长；该差异是安全/路线约束的代价，不能表述为所有指标全面胜出。

这是离线、零 motion、单步评测，不是闭环飞行成功率。ScaleNav 在线 C++ M4 尚未接入
等价 widest-shortest 和中心线优化，不能用本报告替代在线集成验证。

产物：

- 数据报告：[generation_report.json](../dataset/benchmark_003/generation_report.json)
- 模型报告：[comparison_report.json](../dataset/benchmark_003/comparison_002/comparison_report.json)
- 三模型 HTML：[viewer/index.html](../dataset/benchmark_003/comparison_002/viewer/index.html)
