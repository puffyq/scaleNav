# Route-Conditioned YOPO 在线控制入口测试报告

| 项目 | 结果 |
|---|---|
| 日期 | 2026-08-28 |
| checkpoint | `train_scalenav/saved_corrected/YOPO_5/best.pth` |
| 控制入口 | `scalenav_ws/scripts/start_route_yopo.sh` |
| 控制输出 | `/scalenav/trajectory_point`，50 Hz |
| 旧入口 | `scalenav_ws/scripts/start.sh` 未修改 |

## 1. 被测实现

| 文件 | 职责 |
|---|---|
| `route_yopo_control_core.py` | 三级状态、route id、RouteCondition 特征、Poly5 p/v/a 和候选选择 |
| `route_yopo_control_ros2.py` | EPIC 聚合、checkpoint 推理、连续深度安全门和 50 Hz 控制 |
| `start_route_yopo.sh` | 完整 Route-YOPO 控制启动或 `--attach` 接入已有 EPIC |
| `benchmark_route_yopo_control.py` | GPU 模型与完整规划 tick 性能基准 |
| `test_route_yopo_control.py` | 函数、安全门、控制发布和冲突回归 |

## 2. 控制数据流

```text
EPIC accepted path/frontier/safety space
    -> RouteCondition 兼容聚合
    -> YOPO_5 15 primitive
    -> score 顺序逐条检查
    -> 101 点 Poly5 连续安全门
    -> 首条 CERTIFIED primitive
    -> 50 Hz p/v/a 插值
    -> /scalenav/trajectory_point
```

无认证轨迹、模型失败、深度/里程计超时或轨迹过期时，控制器清除旧 Poly5，向无人机发布
当前位置和三轴零速度保持命令。控制话题存在第二 publisher 时停止发布，避免旧 planner 与
Route-YOPO 同时控制。

## 3. 执行结果

### 3.1 checkpoint 合同

```text
feature_order=observation_depth_route_v1
endstate=(1, 9, 3, 5)
score=(1, 3, 5)
finite_endstate=true
finite_score=true
```

12 个 route anchors 与 checkpoint 元数据一致。当前直接加载普通 PyTorch checkpoint；
TorchScript 仍是后续部署优化项，不阻断当前 Python ROS2 控制入口。

### 3.2 定向测试

```text
14 passed in 1.16s
```

| 覆盖项 | 结果 |
|---|---|
| ROUTE/FRONTIER_ONLY/SAFETY_HOLD | 通过 |
| adapter-local route id 单调性 | 通过 |
| RouteCondition body-FLU 与归一化 | 通过 |
| 四元数完整三维旋转 | 通过 |
| 101 点 XYZ Poly5 位置边界 | 通过 |
| 101 点 Poly5 速度/加速度边界 | 通过 |
| score 最优碰撞后选择下一安全项 | 通过 |
| 全部不安全/未知/非有限时无选择 | 通过 |
| 自由深度认证、2 m 障碍拒绝 | 通过 |
| 高分辨率安全深度保守降采样 | 通过 |
| 控制节点使用既有 trajectory topic | 通过 |
| 认证轨迹发布 p/v/a 控制 | 通过 |
| 无轨迹发布当前位置零速度保持 | 通过 |
| 第二控制 publisher 抑制 | 通过 |

### 3.3 RTX 3090 性能基准

模型 warm-up 50 次；纯模型统计 1000 tick，完整规划与安全门统计 100 tick。

| 指标 | P50 | P95 | 最大值 |
|---|---:|---:|---:|
| Route-YOPO 模型推理 | 1.743 ms | 2.935 ms | 4.785 ms |
| 前处理 + GPU + Poly5 + 15 候选连续安全门 | 36.139 ms | 64.460 ms | 68.883 ms |

峰值 GPU allocated memory 为 `192.03 MiB`。输入是合成 `96x160`、20 m 远深度、零 motion
和直线路线。该结果满足 5 Hz 模型更新周期；50 Hz 控制定时器只进行数组插值和消息发布，
不在定时器中重复模型推理或安全门。

安全门优化前，15 候选单 tick 的 CPU 中位/P95 约为 `187/206 ms`。保持 101 点不变，使用
相邻采样半间距扩张球并消除端点重复检查后，CPU 合成测试中位/P95 约为 `83/117 ms`。

### 3.4 ROS2 控制 publisher

执行 `start_route_yopo.sh --attach --device cuda` 后，ROS 图查询结果为：

```text
Type: trajectory_msgs/msg/MultiDOFJointTrajectoryPoint
Publisher count: 1
Node name: scalenav_route_yopo_controller
Topic: /scalenav/trajectory_point
```

启动初期 DDS 短暂保留旧 endpoint 时，节点检测到 publisher count 为 2，停止控制发布；
旧 endpoint 消失后记录冲突解除，并等待下一条新认证轨迹。该现象验证了双控制器保护，
最终 ROS 图中只有 Route-YOPO 控制 publisher。

## 4. 接口现状

EPIC 的 `/epic/path`、`/epic/graph` 和 `/epic/clearance` 仍是分离话题，且
`/epic/bubbles` 不携带真实安全球半径。控制节点要求 source stamp 差不超过 `0.20 s`，
使用 path 最小安全空间广播半径，并在诊断中标记 `epic_compat_non_atomic`。stamp 不一致时
降级为 FRONTIER_ONLY；frontier 也失效时进入 SAFETY_HOLD。

当前深度扫掠球安全门是控制发布的最终门。未知区不会因模型 score 较低而放行；score 最优
候选碰撞时检查下一候选，15 条均无法认证时立即保持。

## 5. 待执行

1. 用实际 EPIC 和仿真深度完成 10 次窄门、转弯和障碍突入闭环飞行。
2. 真实障碍分布运行至少 1000 个模型 tick，记录完整控制链 P50/P95/max。
3. 注入 depth、odom、route 超时和旧 planner publisher，核对飞行日志中的保持行为。
4. EPIC 增加 source route id、frontier、centers、真实 radii、route geometry 和 flags 的原子消息。
