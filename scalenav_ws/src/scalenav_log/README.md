# scalenav_log

独立的 ROS 2 C++ 运行日志包。节点按消息到达顺序写入一个 session，并在磁盘达到限制时自动创建新 session，随后按时间从旧到新删除 session。

记录内容：

- 深度图：`depth/*.pgm`，16-bit PGM，单位毫米；不支持的编码保留为 `*.bin`。
- 点云和 free-ray：`pointcloud/*.pcd`，PCD ASCII，支持按 stride 和最大点数抽样。
- graph、bubbles、path：`graph/*.json`，保留 marker 类型、命名空间、颜色、姿态和点序列。
- odometry、trajectory control、goal、semantic heatmap：结构化写入 `index.jsonl`。

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

主要清理参数：

- `max_total_bytes`：所有 session 的总字节上限。
- `max_sessions`：最多保留的 session 数量。
- `max_session_bytes`：单 session 达到上限后滚动到新 session。

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
