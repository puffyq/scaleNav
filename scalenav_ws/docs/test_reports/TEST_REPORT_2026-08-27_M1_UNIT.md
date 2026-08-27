# M1 感知同步与局部地图单元测试报告

## 1. 测试范围

- 测试依据：[`FUNCTION_TEST_CASES.md`](../FUNCTION_TEST_CASES.md) 的 `TC-M1-001` 至
  `TC-M1-013`。
- 测试日期：2026-08-27。
- 代码范围：`scalenav_graph_ros2` 的 `LIOInterface` 和 `EpicGraphNode` 感知回调。
- 自动化入口：`test_lidar_map`。
- 真实日志：`log_scalenav/session_20260827_101213_519/index.jsonl`。
- ROS 日志：`/home/puffy/.ros/log/epic_graph_node_284950_1787796733478.log`；容差分支补充使用
  `/home/puffy/.ros/log/epic_graph_node_3470167_1787295968710.log`。

本轮只执行 M1 单元层。M1 模块测试 `MT-M1-*`、M2 持久拓扑和闭环任务不计入本报告。

## 2. 执行结果

| 用例 | 结果 | 证据 | 判定摘要 |
|---|---|---|---|
| TC-M1-001 | 通过 | `TcM1001InitializationAllowsEmptyQueries` | 初始化后空 KNN、空 boxSearch 和空树距离查询不崩溃 |
| TC-M1-002 | 失败 | `TcM1002VectorIsInBoxHonorsBoundaryAndDeadArea` | 位于配置禁区中心的点实际返回 `true` |
| TC-M1-003 | 通过 | `TcM1003PointIsInBoxMatchesVectorOverload` | 4 组输入与 `Vector3f` 重载一致；不抵消 TC-M1-002 的语义失败 |
| TC-M1-004 | 失败 | `TcM1004VectorIsInMapUsesInsetBoundary` | 边界及距边界 `0.5e-4 m` 的点实际返回 `true` |
| TC-M1-005 | 通过 | `TcM1005PointIsInMapMatchesVectorOverload` | 6 组输入与 `Vector3f` 重载一致；不抵消 TC-M1-004 的语义失败 |
| TC-M1-006 | 通过 | `TcM1006DistanceOverloadsMatchGeometricTruth` | 30 个查询点的 3 个重载均与几何真值误差小于 `1e-5 m` |
| TC-M1-007 | 通过 | `TcM1007KnnReturnsSortedMatchingPoints` | `k=1/3/20` 各执行 10 次，数量、平方距离和非降序满足判定 |
| TC-M1-008 | 通过 | `TcM1008BoxSearchUsesClosedBounds` | 相交/空 AABB 各执行 10 次，闭区间端点被包含 |
| TC-M1-009 | 失败 | `TcM1009OneHundredFramesRespectVoxelAndWindowRules` | 预期 2 个窗内体素，实际保留 3 个，`40.01 m` 点在静止重复帧中重入 |
| TC-M1-010 | 部分通过 | 最新会话 5491 条 odom | 四元数范数范围 `0.999999999..1.000000001`，位置和时间戳连续；未注入非单位输入，也未从日志读取内部 deque 容量 |
| TC-M1-011 | 部分通过 | 两份 ROS 日志 | 最新日志覆盖精确同步 `0.000 ms`；历史日志覆盖成功的 `10.022..40.013 ms` 和被拒的 `50..90 ms`；日志没有直接输出返回 pose 的逐分量比较 |
| TC-M1-012 | 部分通过 | 最新会话 410 条 pointcloud | 410 帧均保存并进入正常点云链路，23 条节流日志 `pose_sync=0.000 ms` 且无 dropped cloud；未独立注入 NaN 和 40 m 外点 |
| TC-M1-013 | 通过 | `TcM1013OneHundredTwentyFreeRayFramesDoNotCreateObstacles`；最新会话日志 | GTest 连续 120 帧验证占据数和最近距离不变；日志另有 410 对同时间戳 pointcloud/free-ray |

汇总：13 条用例中，7 条通过、3 条失败、3 条部分通过。

## 3. 自动化执行记录

构建命令：

```bash
colcon build --packages-select scalenav_graph_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

构建结果：通过。PCL 的 pcap/png 可选功能警告不影响本次目标。

执行命令：

```bash
./test_lidar_map --gtest_color=no \
  --gtest_output=xml:/tmp/scalenav_m1_lidar_full.xml
```

`test_lidar_map` 全目标共执行 19 项，16 项通过、3 项失败；其中新增的 M1 契约测试执行
10 项，7 项通过、3 项失败。失败项分别对应 TC-M1-002、TC-M1-004 和 TC-M1-009。

## 4. 日志复核

最新结构化会话共记录：

| 数据类型 | 数量 |
|---|---:|
| odom | 5491 |
| pointcloud | 410 |
| free_ray | 410 |
| depth | 411 |
| graph/path/clearance | 各 464 |

410 条 pointcloud 与 410 条 free-ray 按 stamp 全部一一配对。pointcloud 原始点数范围为
`781..14621`，总计 `3619775` 点，资产写入前后点数一致。最新 ROS 日志含 23 条
`[EPIC timing][cloud]`，均为 `pose_sync=0.000 ms`，没有点云同步丢弃。

历史 ROS 日志提供非零时间差分支：`10.022 ms`、`30.005 ms`、`40.013 ms` 的点云被
接受；同一日志包含 5553 条超容差丢弃记录，差值覆盖 `50..90 ms`。这些日志足以证明
成功/拒绝分支曾在线执行，但由于没有输出 `poseForCloud()` 返回姿态的逐分量值，
TC-M1-011 保持“部分通过”。

## 5. 失败定位

### 5.1 TC-M1-002 禁区未参与活动实现

当前编译使用 `include/lidar_map/lidar_map.h`。其中 `IsInBox(Vector3f)` 只检查全局最小/
最大边界，没有遍历 `dead_area_*`。仓库内另一份 legacy 头文件
`src/lidar_map/include/lidar_map/lidar_map.h` 有禁区判断，但不是本目标当前包含的实现。

### 5.2 TC-M1-004 地图边界未内缩

活动头文件的 `IsInMap(Vector3f)` 直接调用 `IsInBox`，因此边界本身和距边界小于
`1e-4 m` 的点仍返回 true。legacy 头文件存在内缩判断，说明当前存在两份契约不一致的
`LIOInterface` 实现。

### 5.3 TC-M1-009 静止重复帧可重新插入窗外点

`updateCloudWorld()` 在首次更新时会裁剪 `40.01 m` 点；下一帧又先把该点插入占据索引。
由于无人机没有移动到 `prune_distance` 且点数没有超过容量，本帧不再执行裁剪，窗外点
随后持续保留。因此“只要输入点在窗口外就不进入 M1 缓存”的文档判定当前不成立。

生产 launch 当前默认 `map_history_radius_m=0.0`，表示 current-frame 模式；而
TC-M1-009 明确验证 40 m 跨帧缓存。需要先决定正式契约是 current-frame 还是 40 m
滑窗，再选择修复实现或改写该测试用例，不能用 current-frame 日志宣称 40 m 用例通过。

## 6. 未覆盖项

- TC-M1-010：非单位四元数输入和 `odom_history_ <= 512` 的内部状态断言。
- TC-M1-011：精确、容差内和容差外三种输入对应返回 pose 的逐分量断言。
- TC-M1-012：NaN、近点、40 m 外点在同一消息中的过滤和世界坐标数值断言。
- M1 写入是否产生 M2 persistent node 属于跨模块边界，应在 `MT-M1-003` 或 M1-M2 集成
  用例中验证，不作为单个 `LIOInterface` GTest 的通过依据。
