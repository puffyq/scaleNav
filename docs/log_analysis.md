# OpenSeek 日志分析

## 深度单位

Colosseum ROS2 bridge 发布的 `/camera/depth/image` 是 `32FC1`，每个像素的数值单位为米。RViz 直接读取这些米值，但默认颜色会按当前图像范围自动映射，因此不能只凭颜色判断绝对距离。

YOPO-Simple 的原版预处理是：

```text
32FC1: 直接读取米值
16UC1: 毫米除以 1000，得到米值
depth_tensor = inpaint(uint8(clip(depth_m, 20) / 20 * 255)) / 255
```

因此网络深度张量的数值范围是 `[0, 1]`，不再是米；物理含义仍对应 `0-20 m`。日志里的 `model_depth_mm.png` 是将这个张量乘回 `20 m` 后以 uint16 毫米保存，目的是方便逐像素检查，不是送入网络的另一份输入。

当前相机为 `160x96`、水平 FOV `90 deg`，按宽高比计算的垂直 FOV 约为 `61.9 deg`，接近模型配置的 `60 deg`。Colosseum 的 `DepthPlanar` 是相机前向轴 Z-depth；原版 YOPO ROS 节点也直接使用 Z-depth，没有转换成欧氏射线距离。

## 与原版的关键差异

深度预处理和模型状态/输出解码已经按 YOPO-Simple 原版执行。仍需在分析时关注两类域差异：

- 原版输出完整的 P/V/A/yaw 给位置控制器；当前 Colosseum 链路最终直接发布世界系 velocity 和 yaw-rate。日志页面同时显示模型 P/V/A、世界轨迹和实际执行速度，用来量化这一差异。
- 训练环境与 UE 场景的几何和材质分布不同。细树干、叶片透明材质、近距离机体遮挡都可能产生训练集中较少见的深度形态。

## 启动

```bash
cd /mnt/code/lab/yopo/OpenSeek
bash scripts/13_start_log_viewer.sh
```

浏览器打开 `http://127.0.0.1:8765`。指定其他目录或端口：

```bash
LOG_DIR=/path/to/log_event PORT=8766 bash scripts/13_start_log_viewer.sh
```

查看器只读 `log_event`，不会修改日志。服务端先建立轻量的 JSONL 偏移索引，再按当前帧读取事件和深度 PNG，避免把几十到几百 MB 的日志一次加载到浏览器。

## 逐帧内容

- 原始米制深度图与网络预处理后恢复成米的深度图。
- 网络 motion 输入、推理时间、候选 score 和机体系 P/V/A。
- 由选中 P/V/A 构造的世界系五次多项式轨迹。
- 同一时间窗内的实际 odom、实际发布的控制采样和 LiDAR。
- 可在机体系 FLU 与世界系 ENU 之间切换的 3D 深度点云和轨迹。

旧日志没有完整候选和张量统计时仍可查看，只显示当时已经记录的选中输出。重新启动 `08_start_openseek_planner.sh` 后生成的新日志会包含全部字段。
