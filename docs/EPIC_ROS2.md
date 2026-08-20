# EPIC ROS2 接入

这个端口迁移 EPIC 的真实自由空间球生成、区域聚类、拓扑图和 Bubble A*。输入使用现有的
`DepthPlanar -> /frgraph/points`，不启动 ROS1/catkin，也不迁移 EPIC exploration
manager。

ROS2 适配器把机体系点云按 `/sim/odom` 转到世界系，并用 10 cm 增量体素哈希维护
EPIC 所需的累计障碍点图；最近障碍距离、KNN 和 box search 使用 PCL KD-tree。

实际数据流为：

```text
PointCloud2 + nearest-obstacle query
  -> getRegionsToUpdate()
  -> generateBubble()
  -> BubbleUnionSet::unionSetCluster()
  -> updateSkeleton() / TopoNode / witness edges
  -> graphSearch()
  -> selected dense witness path
```

沿 A-B 方向生成的 goal-directed 节点仍然保留，但只作为真实 EPIC skeleton 上的
附加候选，不替代 BubbleNode 图生成。

## 构建

```bash
cd /mnt/code/lab/yopo/OpenSeek
set +u; source /opt/ros/humble/setup.bash; set -u
colcon build --symlink-install --packages-select openseek_epic_ros2 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 运行

先启动现有深度转点云和 FRGraph 输入链：

```bash
ros2 launch openseek_frgraph_ros2 frgraph_gap_pipeline.launch.py
```

另一个终端启动 EPIC：

```bash
source /opt/ros/humble/setup.bash
source /mnt/code/lab/yopo/OpenSeek/install/setup.bash
ros2 launch openseek_epic_ros2 epic_graph.launch.py
```

EPIC 订阅 `/frgraph/points`、`/sim/odom`、`/goal`，发布：

```text
/epic/graph    visualization_msgs/MarkerArray
/epic/bubbles  visualization_msgs/MarkerArray
/epic/path     nav_msgs/Path
```

`/epic/graph` 包含真实骨架节点、goal-directed 节点、拓扑边、所有边的见证路径、
A* 拓扑路径和最终选中的稠密见证路径。`/epic/bubbles` 只显示 EPIC
`generateBubble()` 生成的真实 BubbleNode。

## RViz

直接启动 AirSim 帧回放、点云链、EPIC 和 RViz：

```bash
bash scripts/39_view_epic_airsim.sh
```

也可以只打开已经运行中的话题：

```bash
rviz2 -d /mnt/code/lab/yopo/OpenSeek/install/openseek_epic_ros2/share/openseek_epic_ros2/config/epic_graph.rviz
```

固定坐标系是 `odom`。点云显示已配置为 `Best Effort + Volatile`，与深度点云发布器的 QoS 一致；配置同时显示输入点云、EPIC 节点/边、自由空间球和 A* 路径。

## 回放验收

AirSim 采集帧（默认 `Scene_0002/frame0`）：

```bash
bash scripts/37_test_epic_airsim_frame.sh
```

面对墙合成场景：

```bash
bash scripts/38_test_epic_ros2_wall.sh
```

`passed: true` 现在要求：真实 BubbleNode 非零、真实 skeleton node 非零、边见证
路径非零、A* 选中路径非空，并且选中见证路径的每一段都不碰当前帧已知深度表面。
日志会打印 Bubble、节点、边、见证点数量以及首帧建图和后续更新耗时。
