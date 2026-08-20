# FRGraph Reproduction and OpenSeek Integration

## Upstream boundary

The pinned upstream checkout is `third_party/FRGraph` at commit `1705b09`.
Its full planner is a ROS1 catkin package that expects `/scan` or
`/velodyne_points` plus odometry. OpenSeek runs ROS2 Humble and receives a
`160x96` forward DepthPlanar image, so the original ROS node is not linked into
the runtime.

The ROS-independent `DecompUtil` convex-decomposition core builds locally and
all four upstream tests pass. Reproduce that result together with the OpenSeek
adapter and Map2 acceptance check using:

```bash
bash scripts/30_test_frgraph_reproduction.sh
```

## Adapter contract

`graph.frgraph_adapter.FRGraphAdapter` ports the upstream range-map/gap idea to
one DepthPlanar frame. It produces directional free regions and accepts an
optional PEARL heatmap for region scoring. Unknown and far-clipped pixels are
optimistic; measured geometry inside the robot envelope blocks a region.

Only the sparse OpenSeek Graph persists. No point cloud, OctoMap, or dense
occupancy map is retained between frames.

The integration path is:

```text
DepthPlanar + optional PEARL
  -> FRGraphAdapter free regions
  -> SparseDepthGraph candidate directions
  -> CERTIFIED / UNVALIDATED / INVALID edges
  -> 3-D waypoint for YOPO
```

Run the fixed Map2 acceptance frame:

```bash
bash scripts/28_run_frgraph_map2.sh
```

The expected result is one right-side free region, a certified local edge from
the start to that region, and optimistic path `0 -> 2 -> Goal`.

Start the offline viewer on port 8768:

```bash
PORT=8768 PEARL_DEVICE=cuda bash scripts/29_start_frgraph_viewer.sh
```

## Runtime status

The ROS2 runtime now links the original FRGraph `GapExtractor` and
`PlannerManager` geometry code. A current DepthPlanar frame is converted to a
temporary XYZ cloud, fed to the original gap extractor, and then to the
original anisotropic convex-region and free-region graph code. The graph and
the convex free-space boundaries are published as ROS2 `MarkerArray` messages
on `/frgraph/graph` and `/frgraph/free_space` for RViz/UE acceptance.

Continuous-safe Bezier/PIQP trajectory generation is intentionally not called:
YOPO remains the local trajectory executor. No point cloud or occupancy map is
retained between frames.
