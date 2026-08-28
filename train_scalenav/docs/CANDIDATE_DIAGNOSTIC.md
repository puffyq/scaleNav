# 15-Primitive Candidate Diagnostic

该诊断保留 YOPO 网络输出的全部 15 个 primitive，而不是只保存 score 最低的一个。
每个候选保存 101 个轨迹采样点，以及 score、碰撞、最小净空、中心线距离、走廊偏差和路线进度。

运行：

```bash
cd /mnt/code/lab/yopo/OpenSeek/train_scalenav
PYTHONPATH=. /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python evaluate_candidates.py \
  --data dataset/benchmark_004_esdf_001 \
  --checkpoint saved/YOPO_54/best.pth \
  --output dataset/benchmark_004_esdf_001/candidate_diagnostic_002 \
  --batch-size 64 --workers 0 --device cuda
```

产物：

- `candidate_diagnostic_002/viewer/index.html`：交互式候选轨迹 viewer；
- `candidate_diagnostic_report.json`：selected/oracle aggregate 指标；
- `candidate_predictions.json`：每条路线全部候选的详细结果。

viewer 中：

- `score selected` 是网络 score 最低的 primitive；
- `centerline oracle` 是安全候选中平均 3D witness 中心线距离最小的 primitive；
- `selectionCenterlineGapM` 是两者的中心线距离差；
- `All 15 candidates` 可显示全部候选，`Safe candidates only` 可隐藏碰撞候选。

解释规则：若 oracle 明显优于 selected，问题主要在 score head 排序；若两者都偏离中心线，
问题主要在 end-state/trajectory 表达或 path loss 梯度。
