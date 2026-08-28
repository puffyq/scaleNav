# TODO-002 1080p PEARL 电线场景与语义地图融合

| 项目 | 内容 |
|---|---|
| 批次号 | `002` |
| 主题 | 1080p 细线语义检测、三维融合和语义引导换廊 |
| 状态 | 待实现 |
| 优先级 | P0 |

## 1. 目标

在仿真中构造一根低分辨率几何深度难以稳定观测、但 1080p RGB 中清晰可见的电线，
使用 PEARL 检出电线，将同步深度和多帧观测融合为持久的世界坐标语义折线，并让
EPIC A* 在碰撞前提前选择低风险 corridor。

本批次必须分别验证：

- 1080p 图像中电线具有足够像素宽度，PEARL 能稳定产生连续热响应。
- `160 x 96` YOPO/几何深度链路保持不变，不能因语义实验增加控制链路延迟。
- 电线使用真实深度或多视角几何进入语义地图，不把固定 30 m 投影当作最终位置。
- 同一根电线跨帧复用同一个语义实体，不按帧数线性增加节点。
- A* edge witness 查询电线风险场，并在存在安全替代走廊时提前换廊。

## 2. 当前限制

- AirSim RGB 和 Depth 当前均为 `160 x 96`，远处细线在传感器输入阶段已经丢失。
- PEARL 默认 `ViT-B/16`、`short_side=336`，对低分辨率源图放大不能恢复电线细节。
- PEARL 输出会插值回源图大小；输出为 1080p 不代表模型具有逐像素 1080p 特征。
- EPIC 当前将热图压缩为 `5 x 3` maxima，每帧最多保留 16 个语义点。
- 固定 `semantic_virtual_depth_m=30 m` 会使同一电线随视点移动产生漂移锚点，不能作为
  最终三维融合结果。
- 独立 virtual semantic point 不创建拓扑边；它们应作为风险证据，而不是电线的几何
  路径节点。

## 3. 场景契约

### 3.1 相机与电线

| 参数 | 第一版配置 |
|---|---:|
| 语义 RGB | `1920 x 1080` |
| 语义 DepthPerspective | `1920 x 1080`，只以约 2 Hz 请求 |
| 水平 FOV | `90 deg` |
| YOPO/几何 Depth | 保持 `160 x 96` |
| PEARL 频率 | `2 Hz` |
| 电线直径 | `0.08-0.12 m` |
| 电线长度 | `8-15 m` |
| 初次有效观测距离 | `20-25 m` |
| 材质 | 深灰、哑光、非透明 |
| 背景 | 明亮天空或浅色墙面 |
| 运动模糊 | 关闭 |

90 deg HFOV 下，近似像素宽度为：

```text
wire_pixels = (image_width / 2) * wire_diameter_m / distance_m
```

`1920` 宽、距离 `20 m` 时，`0.08-0.12 m` 电线约占 `3.8-5.8 px`；同一目标在
`160` 宽图像中仅约 `0.32-0.48 px`。该配置用于制造“高分辨率语义可见、低分辨率
几何不稳定”的受控场景。

### 3.2 Corridor 布局

- 起点和终点之间至少存在左右两条可通行 corridor。
- 两条 corridor 中心间距为 `8-12 m`，均满足几何 clearance。
- 电线只横跨默认较短的一条 corridor，不能封死整个任务空间。
- 电线中心高度与飞行层一致，第一版固定为约 `z=1.6 m`。
- 无语义时 A* 应倾向较短 corridor；开启语义后应在距离电线 `15-30 m` 时选择替代
  corridor。
- 场景保存电线 GT polyline、直径、材质、世界位姿和相机参数，供误差审计使用。

## 4. 设计方案

```text
1080p semantic RGB ----> PEARL heatmap ----> threshold/ridge/skeleton
          |                                         |
          +---- synchronized semantic depth --------+
                                                    |
                                                    v
                                         sparse 3-D wire samples
                                                    |
                                         line/polyline fitting
                                                    |
                                         persistent SemanticWire
                                                    |
                                      A* witness-to-capsule risk
```

### 4.1 双分辨率传感器

- [ ] 在 AirSim 中增加独立语义相机或独立语义 capture 配置。
- [ ] 语义相机与规划相机使用相同安装位姿和 FOV，或提供经过测试的外参。
- [ ] 以同一次 capture 返回同步的 1080p RGB 和 DepthPerspective。
- [ ] 语义 RGB-D 使用独立 ROS2 topic 和 `CameraInfo`。
- [ ] 保持原有 `160 x 96` RGB-D、点云和 YOPO 输入不变。
- [ ] 不以 20 Hz 发布 1080p Depth；只在 PEARL 处理帧附近以约 2 Hz 获取。
- [ ] 记录 capture、传输、PEARL 和语义地图更新的独立延迟。

### 4.2 PEARL 细线检测

- [ ] 第一版 prompt 使用 `power line, electrical wire, overhead cable`。
- [ ] 基线采用 `ViT-B/16, short_side=672, crop_size=224, stride=56`。
- [ ] 对 `short_side={336, 672, 1080}` 和 `stride={112, 56}` 做召回率/延迟消融。
- [ ] 保存 raw probability heatmap，不使用彩色可视化图做地图输入。
- [ ] 对热图做阈值、连通域、ridge/skeleton 提取，保留细线方向和连续性。
- [ ] 沿 skeleton 等距采样，单帧最多输出 64 个候选，不直接发布全部高分辨率像素。
- [ ] 记录电线 GT mask 上的 recall、背景 false-positive rate、线段连续率和首次检出距离。
- [ ] 若 PEARL 对 4-6 px 电线仍不能稳定响应，记录失败并评估专用电线分割器；不得用
  单纯放大热图掩盖模型分辨率不足。

### 4.3 三维定位与融合

- [ ] 对每个 skeleton sample 读取同时间戳语义深度和 `CameraInfo`。
- [ ] 拒绝无效、非有限、越界或与相邻像素深度明显不连续的样本。
- [ ] 将有效样本通过相机外参和 capture pose 转换到 `world_enu`。
- [ ] 使用 RANSAC line/分段 polyline 拟合抑制背景深度污染。
- [ ] 深度不足时保存 bearing-only track；有足够视差后使用多视角三角化升级。
- [ ] 用位置、方向、重投影误差和观测时间关联同一根电线。
- [ ] 融合中心线、端点、风险、置信度、协方差、观测次数和最后观测时间。
- [ ] 同一根电线跨 100 帧只保留一个稳定 `wire_id`；不得生成 `frames x samples` 个
  persistent entity。
- [ ] 固定 30 m 投影只保留为对照基线，不得标记为 verified semantic geometry。

建议地图实体：

```text
SemanticWire {
  wire_id
  polyline_world[]
  physical_radius_m
  influence_radius_m
  risk
  confidence
  observation_count
  last_seen_ns
  position_covariance
  geometry_state: Virtual | Verified
}
```

### 4.4 A* 风险查询

- [ ] SemanticWire 不进入 Bubble 邻接图，不增加拓扑边。
- [ ] 为语义折线/capsule 建立局部空间索引。
- [ ] A* 对每条 collision-free edge witness 查询附近 SemanticWire。
- [ ] 使用 witness 到 polyline/capsule 的最小距离计算语义风险。
- [ ] `influence_radius_m` 是软代价范围，不替代几何碰撞检查。
- [ ] 当语义风险增量或累计风险越过阈值时请求候选搜索。
- [ ] 日志区分 `semantic_request`、`search_trigger`、`switch_reason` 和候选风险下降，避免
  把 `FRONTIER_HALF` 与纯语义触发混为一谈。

## 5. 测试矩阵

| ID | 配置 | 预期结果 |
|---|---|---|
| PW-001 | 1080p RGB，电线 8/10/12 cm，距离 15/20/25/30 m | 输出像素宽度和 PEARL recall 随距离可解释变化 |
| PW-002 | 160 x 96 几何深度，语义关闭 | 电线不形成稳定远场几何障碍，记录基线 corridor |
| PW-003 | PEARL 开启，固定 30 m 投影 | 仅作为漂移和重复节点对照，不计为最终通过 |
| PW-004 | PEARL + 同步 1080p depth | 三维点落在 GT 电线附近并形成稳定 polyline |
| PW-005 | PEARL + 多视角融合 | 视角变化后保持同一 `wire_id`，误差和协方差收敛 |
| PW-006 | 左 corridor 有线、右 corridor 无线 | A* 在影响范围内选择右 corridor |
| PW-007 | 两 corridor 均无线 | 不因背景热响应产生无必要换廊 |
| PW-008 | 电线高于飞行层超过安全范围 | 保留三维语义，但不惩罚平面执行路径 |
| PW-009 | 短时漏检/遮挡 | 已验证 SemanticWire 不立即消失，置信度按策略衰减 |
| PW-010 | 往返任务 | 返程复用同一地图实体，不重复生成反向电线记录 |

## 6. 验收指标

- [ ] 20 m 处电线在 1080p 图像中可见宽度不少于 3 px。
- [ ] 20-25 m 范围内 PEARL 帧级检出率不低于 90%。
- [ ] 背景帧 false-positive rate 不高于 5%。
- [ ] verified polyline 到 GT 电线的 P95 距离误差不高于 `0.5 m`。
- [ ] 跨 100 帧 `wire_id` 数量保持为 1，允许短时 virtual track 但必须合并或淘汰。
- [ ] 语义地图实体数有明确上限，运行 10 分钟不随帧数线性增长。
- [ ] 有替代 corridor 时，至少 9/10 次任务在进入 `15 m` 范围前完成换廊。
- [ ] 无电线场景 10 次任务不发生语义误触发换廊。
- [ ] 语义链路不改变 YOPO 输入 shape、控制周期和低分辨率几何点云契约。
- [ ] PEARL 2 Hz 运行时 dropped frame 比例低于 5%，GPU 峰值内存和 P50/P95 延迟写入报告。

## 7. 交付物

- [ ] AirSim 电线双 corridor 场景和可复现场景参数。
- [ ] 1080p 低频语义 RGB-D topic、`CameraInfo` 和时间同步实现。
- [ ] PEARL 细线检测与 skeleton sample 输出。
- [ ] SemanticWire 三维融合、持久 ID、空间索引和 RViz marker。
- [ ] A* witness-to-wire 风险代价和触发原因日志。
- [ ] PW-001 至 PW-010 的自动化结果及至少一次闭环视频/轨迹图。
- [ ] PEARL 分辨率/步长消融、地图误差、换廊成功率和资源报告。

## 8. 完成定义

只有同时满足“1080p 稳定检出、三维位置经过深度或多视角验证、跨帧复用同一
SemanticWire、A* 在受控场景中提前换廊、资源指标有界”时，本 TODO 才能标记完成。
仅看到彩色热图、仅生成固定 30 m virtual point，或仅靠低分辨率几何碰撞后绕行，均不
算完成。
