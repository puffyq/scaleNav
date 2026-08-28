# Route-YOPO Batch 004：中心线拟合与原版对比

| 项目 | 内容 |
|---|---|
| 训练数据 | `dataset/benchmark_004_esdf_001` |
| 测试路线 | 1479 条有效路线，三场景分组 |
| 最终 checkpoint | `saved/YOPO_54/best.pth` |
| 初始化 checkpoint | `saved/YOPO_53/best.pth` |
| YOPO-Simple | `/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth` |
| 输入 | 相同 depth、pose、零初始速度/加速度和 frontier goal；Route-YOPO 额外使用 witness bubbles |
| 训练设备 | NVIDIA RTX 3090 |

## 训练变更

`YOPO_53` 从 `YOPO_50` 微调，中心线权重设为 `2.0`。随后 `YOPO_54` 从 `YOPO_53` 以
`1e-5` 学习率继续微调 6 epoch。总 loss 保留原版 smooth、safety、frontier、acceleration
和 score regression，并包含 bubble safety field、ordered path MSE、centerline 和 tangent。

评测新增 3D witness 折线距离：每个轨迹采样点投影到最近 witness segment，统计平均距离和最大距离。

## 配对结果

| 模型 | 碰撞率 | 平均中心线距离 | 平均最大中心线距离 | 平均最大走廊偏差 | 平均进度 |
|---|---:|---:|---:|---:|---:|
| YOPO_54 | 0.203% (3/1479) | 0.220 m | 0.609 m | 0.057 m | 6.545 m |
| YOPO_50 | 0.541% (8/1479) | 0.266 m | 0.727 m | 0.095 m | 6.644 m |
| YOPO-Simple 3x5 | 8.046% (119/1479) | 0.413 m | 1.087 m | 0.287 m | 6.225 m |

YOPO_54 相比 YOPO-Simple 的平均中心线距离降低约 47%，平均最大走廊偏差降低约 80%，
碰撞率降低约 97.5%。相比 YOPO_50，中心线距离和安全性更好，但平均进度低约 `0.10 m`。

## 产物

- [YOPO_54 单模型 HTML](../dataset/benchmark_004_esdf_001/evaluation_yopo54_viewer/index.html)
- [YOPO_54/YOPO_50/YOPO-Simple 对比 HTML](../dataset/benchmark_004_esdf_001/comparison_029/viewer/index.html)
- [配对 JSON 报告](../dataset/benchmark_004_esdf_001/comparison_029/comparison_report.json)

这是单步离线评测；单次 primitive 的平均轨迹长度约 `6.61 m`，不能直接等同于完整 witness 的
10--30 m 路径长度。
