# FRGraph ROS2 输入链

当前采用对原 FRGraph 算法改动最少的输入方式：

```text
/camera/depth/image (sensor_msgs/msg/Image, 32FC1, Z-depth)
        |
        v
depth_planar_to_pointcloud_node
        |
        v
/frgraph/points (sensor_msgs/msg/PointCloud2, XYZ, body FLU)
        |
        v
FRGraph ROS2 planner
```

转换节点只处理当前帧，不累计点云，也不构建占据地图。AirSim 的 optical-frame
Z-depth 按相机内参反投影，再应用 optical -> body FLU 的固定轴变换：

```text
x_body = z_optical + tx
y_body = -x_optical + ty
z_body = -y_optical + tz
```

其中 `tx/ty/tz` 是相机在机体系中的安装位置。输出点云使用原深度图的时间戳，
默认 frame 为 `base_link`。

## 编译和测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select openseek_frgraph_ros2 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select openseek_frgraph_ros2 \
  --event-handlers console_direct+
```

启动输入节点：

```bash
bash scripts/31_start_depth_pointcloud.sh
```

启动 DepthPlanar 到原版 FRGraph GapExtractor 的完整 ROS2 链：

```bash
bash scripts/32_start_frgraph_gap_ros2.sh
```

该 launch 现在启动两个节点；第二个节点内部直接实例化原版
GapExtractor 和 PlannerManager：

```text
DepthPlanar -> /frgraph/points
             -> 原版 GapExtractor -> /gap_candidates
             -> 原版 PlannerManager 凸区域/Graph
```

PlannerManager 只接收当前帧点云，不累计点云，也不建立 OctoMap。它订阅
`/sim/odom` 和 `/goal`，在收到目标后从当前帧生成一次 FRGraph，并发布：

```text
/frgraph/graph       visualization_msgs/MarkerArray
/frgraph/free_space visualization_msgs/MarkerArray
```

两个话题都使用 `odom` frame。`/frgraph/graph` 中绿色球是当前起点节点，
橙色球/线是 frontier 候选节点和边，紫色球是终点节点；黄色粗线是当前
首选的乐观 `起点 -> waypoint -> goal` 路径，尚未认证的终点连接单独放在
`frgraph_optimistic_edges` namespace。紫色细线仍表示起点到终点的直接方向，
用于对照碰撞情况；局部绕行边不要求和它共线。
`/frgraph/free_space` 是每条边对应的 3D 自由空间凸区域边界。
FRGraph 的 Bezier/PIQP 轨迹优化没有在这一层调用，
YOPO 继续负责局部轨迹执行。

## 在线可视化验收

用之前 AirSim 采集的真实 DepthPlanar 帧做离线 ROS2 回放（不需要启动 UE）：

```bash
SCENE=Scene_0002 FRAME=0 bash scripts/35_test_frgraph_airsim_frame.sh
```

该命令回放 `data/Map2GraphData` 中的 AirSim 深度图和位姿，走完整的
`DepthPlanar -> PointCloud2 -> GapExtractor -> PlannerManager` 链，并打印：

```text
DepthPlanar->PointCloud2
FRGraph first graph
PlannerManager expandNodePrimaryOnly
Background expansion
```

若要测试其他已采集帧，修改 `SCENE` 和 `FRAME` 环境变量。

启动 AirSim/Colosseum 和 ROS2 链后，在另一个终端运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
```

在 RViz 中将 Fixed Frame 设为 `odom`，添加两个 `MarkerArray` display：

```text
/frgraph/graph
/frgraph/free_space
```

也可以先用命令确认消息确实在发布：

```bash
ros2 topic echo --once /frgraph/graph visualization_msgs/msg/MarkerArray
ros2 topic echo --once /frgraph/free_space visualization_msgs/msg/MarkerArray
```

已有 UE graph bridge 需要显式指定新话题：

```bash
MARKER_TOPIC=/frgraph/graph bash scripts/24_start_online_graph_ue.sh
```

目标输入默认是 `geometry_msgs/msg/PoseStamped` 的 `/goal`，里程计默认是
`nav_msgs/msg/Odometry` 的 `/sim/odom`；两者都可以在启动时通过
`-p goal_topic:=...` 和 `-p odom_topic:=...` 改变。
