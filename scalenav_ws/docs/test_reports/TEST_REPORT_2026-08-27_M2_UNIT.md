# M2 安全球与拓扑图单元测试报告

## 1. 测试范围

- 测试依据：[`FUNCTION_TEST_CASES.md`](../FUNCTION_TEST_CASES.md) 的 `TC-M2-001` 至
  `TC-M2-023`。
- 测试日期：2026-08-27。
- 代码范围：`scalenav_graph_ros2` 的 `TopoGraph`、`BubbleUnionSet` 和图持久化函数。
- 单元测试入口：`test_topo_semantic`。
- 辅助测试入口：`test_epic_integration` 中与平面投影、安全边直接相关的 2 项检查。
- 在线日志：`/home/puffy/.ros/log/epic_graph_node_351020_1787798504126.log`。

本报告只判定 M2 函数级用例。运行日志可证明函数在真实重建链路中执行，但不能替代
私有函数的边界输入断言；未覆盖完整输入集合或规定次数时记为“部分通过”。

## 2. 执行结果

| 用例 | 结果 | 证据 | 判定摘要 |
|---|---|---|---|
| TC-M2-001 | 通过 | `TcM2001ProjectsPlanarAndThreeDimensionalPoints`；`PlanarGraphProjectsFarFreeRayOntoItsLayer` | 1000 个点验证平面模式固定 z、三维模式保持原坐标，辅助集成检查通过 |
| TC-M2-002 | 已有测试，待场景复核 | 在线 `background initialize`；现有 rebuild 测试夹具 | 合法地图、A* 和参数初始化已实际执行；尚未按 10 组配置直接断言全部依赖指针、区域参数和空图状态 |
| TC-M2-003 | 通过 | `TcM2003RegionIndexUsesFloorForNegativeCoordinates` | 1000 个跨正负边界坐标均与 floor 结果一致 |
| TC-M2-004 | 失败 | `TcM2004IndexBoundaryRejectsOutsideConfiguredGrid` | 合法索引 AABB 正确；`(-1,0,0)` 和 `(5,0,0)` 均错误返回 true |
| TC-M2-005 | 通过 | `TcM2005ConcurrentSameRegionReturnsOnePointer` | 100 轮、每轮 32 线程均取得同一指针，区域 map 只增加 1 项 |
| TC-M2-006 | 部分通过 | 最新日志 16 次 rebuild 的 `regions/occupied_regions`；在线前向 goal | 区域选择链路正常执行；日志不能证明占据/自由区域全集、前向排序和上限三个断言 |
| TC-M2-007 | 部分通过 | 最新日志每次 rebuild 均生成 `bubbles_3d/bubbles_planar`；安全边辅助检查通过 | 已证明真实地图可生成安全球；未分别注入空旷、近墙和完全占据 AABB 并检查中心与最近障碍距离 |
| TC-M2-008 | 测试设计已定义 | 无直接入口 | 部分覆盖立方体的细分结果尚未测试 |
| TC-M2-009 | 测试设计已定义 | 无直接入口 | 主安全球覆盖缺口及重复抑制尚未测试 |
| TC-M2-010 | 测试设计已定义 | 无直接入口 | 全覆盖、部分覆盖、空集合三分支尚未测试 |
| TC-M2-011 | 测试设计已定义 | 无直接入口 | `updateRegionNode` 的两簇归属尚未直接测试 |
| TC-M2-012 | 通过 | `TcM2012UnionSetClustersTransitively` | 100 轮均将相交链传递合并为一簇、保留分离簇且空输入为空 |
| TC-M2-013 | 通过 | `TcM2013RemoveNodesClearsAllReverseEdgeState` | 100 轮均删除节点及双向邻接、路径、权重和安全空间记录 |
| TC-M2-014 | 部分通过 | `OpenLongBubbleEdgeIsNotRejectedByAnArbitraryTwoMeterCap`；在线 `exist_*` 统计 | 最新 15 次增量重建检查 2685 条已有边，保留 2522、修复 122、删除 2、软重试 39；历史日志还出现碰撞拒绝和超时，但未达到每状态 100 次受控输入 |
| TC-M2-015 | 部分通过 | 最新在线 `edge_*` 统计 | 416 个候选中成功 370、无路径 42，插入 96 个节点；最新场景无超时和碰撞拒绝，未完成每状态 100 次受控输入 |
| TC-M2-016 | 通过 | `TcM2016InsertNodeCreatesSymmetricEdgesAndWitnesses` | 100 轮均建立 2 条双向邻接、正反 witness、相同权重和相同安全空间记录 |
| TC-M2-017 | 通过 | `TcM2017RemoveNodeIsIdempotent` | 100 轮首次删除完整，重复删除无异常且不恢复边 |
| TC-M2-018 | 通过 | `DuplicateVerticesAreMergedAndEdgesArePreserved`；`NearbyValidVerticesAreNotCollapsedAsDuplicates`；在线图统计 | 0.04 m 重复点合并且 incident edges 保留，0.20 m 分支保留；最新 17 份图统计均无 0.25 m 内重复点 |
| TC-M2-019 | 通过 | `HalfEdgesAreRemovedFromThePersistentGraph`；在线图统计 | 单向半边被删除，合法双向边保留；最新 17 份图统计的 asymmetric 和 dangling 均为 0 |
| TC-M2-020 | 通过 | `DetachedRebuildCarriesVerifiedNodesAndEdges`；`DetachedRebuildCarriesSemanticNodes` | detached rebuild 保留 persistent 几何节点、双向边、witness 和语义，不复制临时 odom 节点 |
| TC-M2-021 | 部分通过 | 最新日志 1 次初始化重建和 15 次增量重建 | 连续重建未崩溃，产生 656 个区域节点、插入 96、移除 8；未完成 1000 次稳定/增障碍/移窗口受控序列，不能判定稳态不漂移 |
| TC-M2-022 | 测试设计已定义 | 无并发快照测试 | 尚未执行 10000 次 rebuild 并发读写及离锁后可读检查 |
| TC-M2-023 | 测试设计已定义 | 无函数级边界测试 | 开放、墙阻断和极短预算三场景及预算单调性尚未测试 |

汇总：23 条用例中，10 条通过、1 条失败、5 条部分通过、1 条已有测试待场景
复核、6 条测试设计已定义但本轮未执行。

## 3. 自动化执行记录

构建命令：

```bash
colcon build --packages-select scalenav_graph_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

构建通过。PCL 的 pcap/png 可选功能警告不影响本次目标。

完整单元目标：

```bash
./test_topo_semantic --gtest_color=no \
  --gtest_output=xml:/tmp/scalenav_m2_topo_full.xml
```

`test_topo_semantic` 共执行 49 项，48 项通过、1 项失败。原有 41 项全部通过；本轮增加的
8 项 M2 契约测试中 7 项通过，唯一失败为 TC-M2-004。

辅助检查：

```bash
./test_epic_integration --gtest_color=no \
  --gtest_filter='EpicIntegration.PlanarGraphProjectsFarFreeRayOntoItsLayer:EpicIntegration.OpenLongBubbleEdgeIsNotRejectedByAnArbitraryTwoMeterCap'
```

2 项均通过。它们只作为 TC-M2-001 和 TC-M2-014 的补充证据，不计作其他 M2
单元用例通过。

## 4. 在线日志复核

最新日志包含 1 次初始化重建、15 次增量重建和 17 条图一致性统计。重建统计累计值如下：

| 指标 | 累计值 |
|---|---:|
| regions | 646 |
| bubbles_3d / bubbles_planar | 2921 / 2921 |
| nodes | 656 |
| remained / inserted / removed | 574 / 96 / 8 |
| edge_candidates / edge_success / edge_no_path | 416 / 370 / 42 |
| exist_checked / exist_kept / exist_repaired | 2685 / 2522 / 122 |
| exist_removed / exist_soft_retry | 2 / 39 |

17 条 `EPIC graph stats` 的 `asymmetric=0`、`dangling=0`、`duplicate<0.25m=0`。这些数据
证明区域选择、安全球生成、拓扑差分、已有边重验和一致性归一化在真实飞行链路中持续
执行。由于最新场景没有 `edge_timeout` 或 `edge_collision_reject`，对应受控分支不能据此
判定通过；历史日志虽出现过这两类计数，也不替代规定的每状态 100 次函数级输入。

## 5. 失败定位

### 5.1 TC-M2-004 越界索引未拒绝

`TopoGraph::index2boundary()` 当前直接根据索引计算低、高边界并无条件返回 true，没有检查
`region_idx` 是否位于 `[0, x_len) x [0, y_len) x [0, z_len)`。因此负索引和等于
`x_len` 的索引均被报告为合法。合法首、末索引的 AABB 数值计算正确，失败仅位于越界
返回值契约。

本轮没有修改生产实现。

## 6. 后续测试项

- TC-M2-008 至 TC-M2-011：需要为安全球私有生成函数建立可控地图夹具或测试 peer。
- TC-M2-022：需要并发启动 rebuild 与快照读取，并执行 10000 次生命周期检查。
- TC-M2-023：需要可控的开放、墙阻断和超时地图，直接核对状态码、witness 边界和预算。
- TC-M2-002、TC-M2-006、TC-M2-007、TC-M2-014、TC-M2-015、TC-M2-021：已有执行证据，仍需补齐报告表中规定的场景或次数后才能升级为“已测试通过”。
