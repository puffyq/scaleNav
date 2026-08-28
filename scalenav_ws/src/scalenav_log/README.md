# scalenav_log

独立的 ROS 2 C++ 运行日志包。节点按消息到达顺序写入一个 session，每次启动创建一个新的 session；日志不会因容量或数量限制自动滚动，也不会自动删除旧 session。

记录内容：

- 深度图：`depth/*.pgm`，16-bit PGM，单位毫米；不支持的编码保留为 `*.bin`。
- 点云和 free-ray：`pointcloud/*.pcd`，PCD ASCII，支持按 stride 和最大点数抽样。
- graph、bubbles、path：`graph/*.json`，保留 marker 类型、命名空间、颜色、姿态和点序列。完整 graph 默认以 2 Hz 记录，bubbles/path 保持源话题频率。
- odometry、goal、semantic heatmap：结构化写入 `index.jsonl`。
- trajectory control 使用 logger 接收时间，并记录 position、velocity、acceleration 和由连续命令计算的 jerk。
- `mission` 记录每个新目标的开始，以及进入 0.5 m 目标域并降到 0.3 m/s 后的完成事件。
- `/sim/collision` 的状态变化和累计碰撞标志记录为 `collision`；它是仿真碰撞标签，不由 clearance 推断。
- `/scalenav/timing` 的每周期结构化计时记录为 `timing`，用于计算真实 mean/P99，而不是从节流文本日志采样。
- 净空诊断：`/scalenav/clearance` 自动写为 `clearance` 事件。`vehicle_m` 是飞机当前位置的实际执行净空诊断；`global_witness_min_m` 和 `global_witness_mean_m` 只描述 ScaleNav 全局 witness 在当前障碍距离场中的参考净空。YOPO 会围绕该全局引导独立执行局部避障，因此 witness 净空不等于实际飞行净空，也不参与碰撞、成功或安全判定。旧日志中的同义字段为 `path_min_m` 和 `path_mean_m`。

论文评测以第一个非平凡目标为单程任务：到达目标并停止后结束 trial，不把返程计入时间、里程或成功率。`analyze_flight_logs.py --mission-mode all` 仅用于诊断旧的多段日志。

默认话题见 `config/scalenav_log.yaml`，均可通过 ROS 参数覆盖。

## 启动采集

```bash
source /opt/ros/humble/setup.bash
source scalenav_ws/install/setup.bash
ros2 run scalenav_log scalenav_log_node \
  --ros-args --params-file scalenav_ws/install/scalenav_log/share/scalenav_log/config/scalenav_log.yaml
```

主系统入口 `scalenav_ws/scripts/start.sh` 已默认自动启动日志节点。日志会持续写入
`/mnt/code/lab/yopo/OpenSeek/log_scalenav`；可通过 `SCALENAV_LOG_DIR=/path/to/logs` 覆盖目录。每次启动创建一个
session，收到消息后立即写入 depth、pointcloud、graph/path 和结构化事件。

日志目录没有自动清理参数。磁盘空间管理由部署者或外部运维策略负责。

## 启动回放网页

推荐直接运行工作区脚本：

```bash
cd /mnt/code/lab/yopo/OpenSeek/scalenav_ws
./scripts/log_viewer.sh
```

默认读取 `/mnt/code/lab/yopo/OpenSeek/log_scalenav`，监听 `http://127.0.0.1:8765`。
也可以用 `--root DIR` 或 `--port PORT` 覆盖参数。

等价的底层命令：

```bash
scalenav_ws/install/scalenav_log/lib/scalenav_log/scalenav_log_viewer \
  --root /mnt/code/lab/yopo/OpenSeek/log_scalenav \
  --web-root scalenav_ws/install/scalenav_log/share/scalenav_log/web \
  --port 8765
```

打开 <http://127.0.0.1:8765>。网页从 `/api/sessions` 读取 session，从 `index.jsonl` 按时间轴加载 depth、点云和 graph 资源。服务只提供 GET，路径会拒绝绝对路径和 `..` 穿越。

## 构建和测试

```bash
cd scalenav_ws
colcon build --packages-select scalenav_log --symlink-install
colcon test --packages-select scalenav_log --event-handlers console_direct+
```
