# Dataset Workspace

Generated ScaleNav scenes and `routes.npz` files belong here. Dataset contents
are intentionally ignored by Git.

The target scene contract is defined in
`../../scalenav_ws/docs/TODO_001.md`. The generated batch-001 pilot lives in
`pilot_001/` and contains two 500-frame scenes with three ground-truth routes
per frame. Regenerate it with `python -m data.ground_truth_dataset` as shown in
the top-level training README. `test_001/` is a separate 400-frame offline
test set generated from seed 900001. Both directories contain a static
`viewer/index.html`; test data must not be passed to `train_yopo.py`.

Batch 001 uses 80 x 80 m maps and a 2.5-30 m long-tailed rotated-block side
distribution. `pilot_001` contains 3000 routes (53.53% detours, minimum
clearance 0.544 m); `test_001` contains 1200 routes (52.83% detours, minimum
clearance 0.549 m). A trained model evaluation is written separately under
`test_001/model_eval_001/`, so ground truth and model output are not confused.
The final batch-001 model output report covers all 1200 test routes with zero
collisions, 0.623 m minimum clearance, 0.0090 m mean maximum bubble-union
violation, and 5.348 m mean progress.

`benchmark_002/` is the three-scene paired offline benchmark: 600 cylindrical
YOPO-forest routes, 600 routes using YOPO-Simple's original transformed tree
point-cloud asset, and 600 Map2-style large-block routes. The ground-truth-only
viewer is `benchmark_002/viewer/index.html`; the Route-YOPO versus YOPO-Simple
model-output viewer is `benchmark_002/comparison_001/viewer/index.html`.

`test_002/` 是 CHG-0002 witness 伪细腰修复后的编号验证批次，包含圆柱树林和大方块各
5 帧、10 条路线。`viewer/index.html` 显示新 corridor 审计字段；
`model_eval_001/index.html` 显示旧 checkpoint 在新 corridor 上的离线输出。该小批次用于
几何标签与数据链路验收，不替代正式大规模独立测试集。

`pilot_002/` 是修复后正式训练批次，包含三个场景各 250 帧、750 条路线，总计 2250 条。
`benchmark_003/` 使用独立 seed，包含三个场景各 200 帧、600 条路线。最终新模型、上一
正式版本与 YOPO-Simple 的同输入配对结果在
`benchmark_003/comparison_002/viewer/index.html`，完整 JSON 在同目录下。
