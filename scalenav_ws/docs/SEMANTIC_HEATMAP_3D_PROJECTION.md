# ScaleNav 语义热力图当前代码说明

| 项目 | 内容 |
|---|---|
| 文档标识 | `SCALENAV-SEMANTIC-CODE` |
| 文档版本 | `V1.0` |
| 编写日期 | `2026-09-02` |
| 适用包 | `scalenav_graph_ros2` |
| ROS 包版本 | `0.1.0` |
| 当前代码提交 | `9de4cfb` |
| 文档性质 | 当前实现说明，不代表未接入方案 |

## 1. 代码边界

当前语义热力图链路由以下模块组成：

```text
text_heatmap_ros2.py
    -> /scalenav/text_heatmap_raw (32FC1)
    -> scalenav_graph_node.cpp::onSemanticHeatmap()
    -> SemanticFrame（当前帧缓存）
    -> updateTopoSemanticMemory()
    -> TopoGraph::insertSemanticNodes()
    -> semantic_score / semantic_confidence 参与全局图搜索
```

当前代码没有接入 VINS、Kimera-VIO、RayFronts 或 OVO，也没有实现语义特征跟踪和多帧三角化。

## 2. 热力图生产

实现文件：`src/scalenav/text_heatmap_ros2.py`。

`PEARLHeatmapEncoder` 位于 `src/scalenav/text_tracker/pearl_adapter.py`，使用 PEARL/CLIP 推理得到二维浮点热力图。运行节点同时发布：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scalenav/text_heatmap_raw` | `sensor_msgs/msg/Image` | `32FC1` 原始浮点热力图，供规划器使用 |
| `/scalenav/text_heatmap` | `sensor_msgs/msg/Image` | `bgr8` 着色图，仅用于显示 |

`start.sh` 和 `start_route_yopo.sh` 当前默认配置：

```text
PROMPT="tree, blocks, wall"
RATE=2 Hz
```

PEARL 热力图数值在当前规划代码中被当作 `[0,1]` 语义分数处理，不是障碍距离、ESDF、碰撞概率或 bubble 半径。

## 3. 热力图输入校验与同步

实现函数：`ScaleNavGraphNode::onSemanticHeatmap()`。

接收数据必须满足：

- `encoding == "32FC1"`；
- `is_bigendian == false`；
- 宽、高大于 0；
- `step >= width * sizeof(float)`；
- 数据长度不少于 `step * height`。

不满足时直接丢弃并节流告警。

### 3.1 位姿同步

热力图通过 `poseForCloud()` 匹配 odometry。默认参数：

```text
semantic_pose_tolerance_ms = 250.0
odom_topic = /sim/odom
```

超过容差的热力图不进入语义帧缓存。

### 3.2 深度同步

深度由 `onSemanticDepth()` 缓存在 `semantic_depth_history_` 中，再由 `semanticDepthForStamp()` 按时间戳匹配。默认参数：

```text
semantic_depth_topic = /camera/depth/image
semantic_depth_tolerance_ms = 50.0
semantic_depth_max_m = 20.0
```

深度必须是小端 `32FC1`。只有有限、正数且小于 `semantic_depth_max_m - 1e-4` 的采样才被视为 measured depth。无匹配深度时，当前帧仍可生成 virtual projection。

## 4. 当前 patch 处理

当前代码固定使用：

```text
patch_cols = 5
patch_rows = 3
```

热力图的全部像素按位置划分为 `5 x 3` patch。每个 patch 计算有限像素的均值；代码还计算了 lower-quantile baseline 和 calibrated score，但当前生成节点时实际使用的是 patch mean：

```cpp
const float calibrated_score = patch_mean;
```

因此 baseline 目前主要用于诊断，不改变插入节点的排序或分数。

规划只处理中间行（`patch_rows / 2`），所以每个有效水平列最多生成两类投影：

1. 一个 measured projection（该列中心深度有效时）；
2. 一个 virtual projection（每个有效列始终生成）。

上、下两行 patch 不会生成语义节点。

## 5. 当前三维投影公式

实现函数：`pointcloud_topo/graph.h::virtualSemanticPointFlu()`。

输入为归一化图像坐标、FOV、投影深度和相机 FLU 外参。函数构造：

```text
horizontal_tangent = tan(horizontal_fov_deg * pi / 360)
vertical_tangent   = tan(vertical_fov_deg * pi / 360)

depth_ray = [
    1,
    -(2*u - 1) * horizontal_tangent,
    -(2*v - 1) * vertical_tangent
]

body_FLU = camera_translation + depth_m * depth_ray
```

随后使用同步 odometry 的姿态转换到世界坐标：

```text
point_world = capture_pose.position + capture_pose.orientation * body_FLU
```

当前默认相机外参：

```text
semantic_camera_translation_flu = (0.5, 0.0, -0.1) m
```

### 5.1 measured projection

当 patch 中心从同步深度图取得有效深度时，使用该深度生成 `is_virtual = false` 的点。它表示当前深度图采样位置的语义注释。

### 5.2 virtual projection

每个中间行有效列始终使用固定 `semantic_virtual_depth_m_` 再生成一个 `is_virtual = true` 的点。参数来源：

| 来源 | 默认值 |
|---|---:|
| `scalenav_graph.launch.py` | `35.0 m` |
| C++ 参数 fallback | `30.0 m` |

正常使用 launch 时，launch 参数覆盖 C++ fallback。该点不是深度测量结果，当前代码没有保存射线的距离区间，也没有三角化。

## 6. 规划平面投影

实现函数：`projectPlanningPoint()`。

默认：

```text
graph_fixed_layer = true
graph_layer_z = 1.6 m
```

在 `updateTopoSemanticMemory()` 中，measured 和 virtual 点都会经过：

```text
if graph_fixed_layer:
    planning_point.z = graph_layer_z
```

因此当前拓扑语义节点实际使用的是 `x/y` 加固定 `z=1.6 m` 的规划点。原始热力图垂直坐标不会保留为节点高度。

## 7. 语义节点插入

实现函数：`ScaleNavGraphNode::updateTopoSemanticMemory()`。

每个语义点携带：

```text
center          planning_point
semantic_score  patch mean
confidence      FOV confidence × ground confidence
is_virtual      measured=false / virtual=true
column          0..4
stamp_ns        当前语义帧时间戳
```

当前置信度计算：

```text
fov_radius = max(abs(2u-1), abs(2v-1))
fov_confidence = 1 - 0.35 * fov_radius^2
```

fixed-layer 模式下 `ground_confidence = 1.0`。

插入前还会执行：

- 距离检查：点到当前语义帧原点的距离必须 `>= 1.0 m`；
- 地图范围检查：measured 点要求 `IsInBox()`，virtual 点要求 `IsInMap()`；
- 去重检查：节点间距离小于 `max(1.0, semantic_point_separation_m)` 时丢弃后者；
- 分数和置信度裁剪到 `[0,1]`。

默认参数：

```text
semantic_points_enabled = true
semantic_point_min_score = 0.20
semantic_point_separation_m = 1.5 m
semantic_point_radius_m = 0.75 m
semantic_point_max_nodes = 16
virtual_semantic_max_nodes = 512
```

注意：`semantic_point_min_score` 的参数在当前 `onSemanticHeatmap()`/`updateTopoSemanticMemory()` 代码路径中没有用于逐点过滤；当前逐点插入使用的是有效 patch，分数只在后续图逻辑中参与语义风险判断。

## 8. 语义对全局规划的影响

`TopoNode` 保存：

```cpp
semantic_score_
semantic_confidence_
semantic_observations_
semantic_stamp_ns_
semantic_is_virtual_
semantic_column_
```

`TopoGraph` 在边代价和 frontier 选择中使用 `semantic_score * semantic_confidence`。相关参数来自 launch：

```text
semantic_cost_weight = 2.0
frontier_semantic_score_weight = 1.0
semantic_route_influence_m = 8.0
semantic_route_switch_risk_margin = 0.08
semantic_route_switch_cost_ratio = 0.90
```

语义分数只改变全局拓扑搜索的代价/候选选择。可执行路径仍由深度点云、occupancy、witness 和碰撞检查约束。

## 9. 当前代码没有做的事情

以下能力在当前 ScaleNav 代码中不存在：

- VINS-Fusion、Kimera-VIO 或其他 VIO 接入；
- heatmap 区域的跨帧特征跟踪；
- 无深度 heatmap 的多帧三角化；
- semantic ray 数据结构；
- RayFronts 的 ray/frontier casting；
- OVO 的 3D semantic instance map；
- virtual projection 的真实距离估计；
- 用语义节点计算障碍物 clearance；
- 用语义节点生成 bubble 或替换 ESDF 碰撞约束。

当前代码能准确描述为：

> PEARL 生成 `32FC1` 热力图；ScaleNav 对 `5 x 3` patch 的中间行进行处理，在有同步深度时生成 measured projection，并始终生成固定深度 virtual projection；两者投影到 `world_enu` 后再压到固定规划层，作为 TopoGraph 的语义分数和 frontier/路线选择依据。真实几何安全仍由深度/occupancy/witness/碰撞检查负责。

## 10. 代码与版本索引

| 内容 | 路径 |
|---|---|
| 热力图 ROS2 节点 | `scalenav_ws/src/scalenav/text_heatmap_ros2.py` |
| PEARL adapter | `scalenav_ws/src/scalenav/text_tracker/pearl_adapter.py` |
| 热力图接收与投影 | `scalenav_ws/src/global_graph/scalenav_graph_ros2/src/scale_manager/src/scalenav_graph_node.cpp` |
| virtual FLU 投影函数 | `scalenav_ws/src/global_graph/scalenav_graph_ros2/src/pointcloud_topo/include/pointcloud_topo/graph.h` |
| launch 参数 | `scalenav_ws/src/global_graph/scalenav_graph_ros2/launch/scalenav_graph.launch.py` |
| ROS 包版本 | `scalenav_ws/src/global_graph/scalenav_graph_ros2/package.xml` |
| 启动脚本 | `scalenav_ws/scripts/start.sh`、`scalenav_ws/scripts/start_route_yopo.sh` |
