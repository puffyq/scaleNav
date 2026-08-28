# Route-YOPO Batch 005：三场景扩充数据训练

## 数据

`dataset/train_large_001` 使用 `seed=602040` 生成：

| 场景 | 帧数 | 原始 routes | Dataset 样本 |
|---|---:|---:|---:|
| yopo_forest | 1000 | 3000 | 3000 |
| yopo_real_forest | 1000 | 3000 | 3000 |
| blocks | 1000 | 3000 | 3000 |
| 合计 | 3000 | 9000 | 9000 |

`YOPODataset` 划分为 8100 train / 900 valid。每条样本保留 frontier goal、witness path、
ordered bubbles、连续 clearance 和 route quality 字段。数据 viewer 位于
`../dataset/train_large_001/viewer/index.html`。

## 训练

- 初始化：`saved_corrected/YOPO_5/best.pth`
- 配置：20 epoch，batch 32，`1e-5`，`wpath_mse=0.2`，`wbubble=0.01`，
  `wcenterline=0.1`，`wprogress=0.1`，无 safety-ranking
- 设备：NVIDIA RTX 3090
- 最佳 checkpoint：`../saved_large/YOPO_0/best.pth`，validation selected cost 最低在 epoch 6

## 结果

| 测试集 | 碰撞率 | 中心线距离 | 最大走廊越界 | 平均进度 |
|---|---:|---:|---:|---:|
| train_large_001（9000） | 0.589% (53) | 0.334 m | 0.105 m | 6.355 m |
| benchmark_004_esdf_001（1479） | 0.135% (2) | 0.319 m | 0.087 m | 6.502 m |

新批次包含真实树和大方块宽尺度场景，难度高于旧基准；旧基准回归没有退化，但新分布
碰撞率升高，说明仍需按场景做 hard-negative 采样和 score 排序校准，不能宣称已经完全安全。

## 产物

- [训练数据 viewer](../dataset/train_large_001/viewer/index.html)
- [新批次评测](../dataset/train_large_001/evaluation_large_best/index.html)
- [旧基准三模型对比](../dataset/benchmark_004_esdf_001/comparison_large_best/viewer/index.html)
- [旧基准对比 JSON](../dataset/benchmark_004_esdf_001/comparison_large_best/comparison_report.json)
