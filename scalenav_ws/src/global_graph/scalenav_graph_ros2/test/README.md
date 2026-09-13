# Empty-obstacle-cloud regression

A valid depth frame can contain no obstacle returns within the 20 m range.
`/depth/points` is then empty while `/depth/free_rays` still supplies observed
ray endpoints. The graph node must process the empty frame, transform the rays
with synchronized odometry, and include their regions in background graph
construction. Rays select regions; they do not create occupied points or erase
previous obstacle observations from the sliding map.

The regression holds the vehicle stationary at z=60.6 m for five seconds and
requires a local goal before simulated odometry begins moving. It then checks
that the graph and local goal advance, that the graph covers side corridors,
and that the obstacle map remains empty. The semantic variant also checks the
five virtual semantic candidates. A completely missing sensor stream is not
equivalent to a valid frame with zero obstacle returns.

After building the workspace, run from the repository root:

```bash
source /opt/ros/humble/setup.bash
ctest --test-dir scalenav_ws/build/scalenav_graph_ros2 \
  --output-on-failure -R '^empty_cloud_online_'
```

To exercise the actual GCN checkpoint, use the Python environment from
`start_gcn_online.sh` (including PyTorch and ROS 2):

```bash
python scalenav_ws/src/global_graph/scalenav_graph_ros2/test/test_empty_cloud_online.py \
  --node-binary scalenav_ws/install/scalenav_graph_ros2/lib/scalenav_graph_ros2/scalenav_graph_node \
  --free-rays --semantic --gcn-model train_gcn/frontier_gcn_map2_35m.pt \
  --domain-id 174
```

Tests use synthetic inputs on a separate ROS domain and do not command AirSim.
Use different domain IDs for concurrent invocations. Omitting `--free-rays`
checks that empty frames alone do not invent a geometric graph or local goal.

The route-memory tests also cover the execution progress watchdog: lateral
oscillation and backtracking cannot reset its deadline, while forward route
progress does. Recovery clears the stalled route, frontier command, and
polynomial guide before a fresh odometry-rooted search.
