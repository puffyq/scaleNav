# Colosseum 与 YOPO-Simple 坐标契约

本文只描述原版 YOPO-Simple 在线推理真正使用的数据。运行时只有两个坐标系：

| 名称 | 约定 | 用途 |
| --- | --- | --- |
| 世界系 `W` | ROS ENU：X East、Y North、Z Up | odom、目标、规划轨迹、控制命令 |
| 机体系 `B` | ROS FLU：X Forward、Y Left、Z Up | YOPO 的状态输入和轨迹输出 |

`world_enu` 和 `body_flu` 是本文中的语义名称。`drone_1`、
`odom_local_enu` 等字符串是 Colosseum bridge 的消息/TF 实现细节，不构成新的
规划坐标系，也不能用于再次旋转 odom 或目标。

## 模型输入输出

当前模型 `models/original_yopo_simple/model.pt` 是从原版
`YopoNetwork.inference()` 导出的 TorchScript，不是网络的裸 `forward()`。
因此归一化、primitive-frame 输入展开和 primitive 输出解码已经包含在模型中。

输入：

```text
depth: [1, 1, 96, 160]
state: [1, 9] = [velocity_B, acceleration_B, goal_B]
```

- 深度来自 `DepthPlanar`，`32FC1`，单位米。
- 深度处理与原版 `test_yopo_ros.py` 一致：最近邻缩放到 `160x96`，截断到
  `20m`，除以 `20`，对 `NaN` 和小于 `0.04m` 的像素执行 `cv2.inpaint`。
- 传入模型的速度、加速度和目标保持物理量，模型内部负责归一化。

输出：

```text
endstate: [1, 9, 3, 5]
         = 15 条 [position_B, velocity_B, acceleration_B]
score:    [1, 3, 5]
```

`endstate` 已经是机体系物理量，不能再调用 `pred_to_endstate()`。选择 score 最小
的一条，然后仅做一次机体系到世界系转换。

## UE 到模型的边界转换

UE 使用左手坐标，轴为 X Forward、Y Right、Z Up，长度单位厘米。
AirSim/Colosseum 的 `NedTransform` 将它变成局部 NED：

```text
p_NED = (X_UE, Y_UE, -Z_UE) / 100
```

ROS bridge 再将 NED 变成 ENU：

```text
p_W = p_ENU = (y_NED, x_NED, -z_NED)
```

两步合并后：

```text
(X_UE, Y_UE, Z_UE) -> (Y_UE, X_UE, Z_UE) / 100
```

所以 UE 红色 `+X` 前方在 ROS ENU 中是世界 `+Y`，不是世界 `+X`。
AirSim NED yaw 为 `0` 时机头朝 North；对应 ROS ENU yaw 为 `+90 deg`。
这是固定坐标定义，不是需要补偿的误差。

姿态由 bridge 一次性转换：

```text
R_WB = R_ENU_FLU
     = C_ENU_NED * R_NED_FRD * C_FRD_FLU
```

planner 直接使用 `/sim/odom.pose.orientation` 给出的 `R_WB`。不得再应用 TF
出生变换，也不得手工交换 X/Y。

## 原版运行时公式

从 odom 读取世界系位置、速度和姿态：`p_W`、`v_W`、`R_WB`。目标 `g_W`
也直接用同一世界 ENU 数值。

模型输入只做世界系到机体系旋转：

```text
v_B = R_WB^T * v_W
a_B = R_WB^T * a_W
g_B = R_WB^T * (g_W - p_W)
```

模型输出只做反向旋转：

```text
p_end_W = p_W + R_WB * p_end_B
v_end_W =       R_WB * v_end_B
a_end_W =       R_WB * a_end_B
```

这两组公式必须使用同一帧 odom 的同一个 `R_WB`，因此有闭环恒等式：

```text
R_WB * (R_WB^T * vector_W) = vector_W
```

除此之外，YOPO 推理链路没有 ENU/NED、UE、camera optical 或 TF 转换。

## ROS topic 约定

| Topic | 语义 |
| --- | --- |
| `/sim/odom` | planner 输入；数值是世界 ENU |
| `/goal_pose` | 世界 ENU 目标，frame 必须为 `world_enu` |
| `/camera/depth/image` | 原版模型深度输入，米 |
| `/openseek/planned_path` | 世界 ENU 规划轨迹 |
| `/openseek/odom` | 给 RViz 的 odom；数值与 `/sim/odom` 完全相同，只修正 frame 标签 |
| `/colosseum_node/drone_1/vel_cmd_world_frame` | 世界 ENU 速度与 yaw-rate |

Colosseum 把 `/sim/odom.header.frame_id` 写成了 `drone_1`，但 pose 数值本身是
局部世界 ENU。若 RViz 的 Fixed Frame 是 `world_enu` 且直接显示 `/sim/odom`，
RViz 会额外应用出生 TF，造成飞机和 Path/Goal 看起来旋转或平移错位。planner
因此发布 `/openseek/odom` 供 RViz 使用；它只改 frame 标签，不改任何数值。

控制命令已经是世界 ENU。bridge 在 AirSim 边界执行唯一一次逆转换：

```text
(vx, vy, vz)_ENU -> (vy, vx, -vz)_NED
yaw_rate_ENU -> -yaw_rate_NED
```

## 最小验证

初始 yaw 为 `+90 deg` 时：

```text
goal_W = (0, 10, 2)  -> 目标在机头正前方，goal_B 的 X 为正、Y 约为 0
goal_W = (10, 0, 2)  -> 目标在飞机右侧，goal_B 的 Y 为负
```

启动 planner 与 RViz 后，可先关闭控制验证转换：

```bash
cd /mnt/code/lab/yopo/OpenSeek
CONTROL=0 bash scripts/08_start_openseek_planner.sh
bash scripts/12_start_rviz.sh
GOAL_X=0 GOAL_Y=10 GOAL_Z=2 bash scripts/11_send_yopo_goal.sh
```

新日志第一条 `model` 事件应满足：`goal_delta_world` 约为 `[0,10,*]`，
`goal_body` 约为 `[10,0,*]`；对应 `trajectory.end_position_world` 应主要沿世界
`+Y` 展开。确认后再用 `CONTROL=1` 启动控制。

注意：原版 YOPO 发布位置、速度、加速度轨迹给位置控制器。当前 Colosseum 接口
只直接接收速度和 yaw-rate，因此飞机不可能像位置闭环控制器一样严格贴合 RViz
曲线。这个跟踪误差属于控制器接口，不应通过增加坐标转换来补偿。
