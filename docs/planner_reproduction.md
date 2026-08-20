# Planner Reproduction Baselines

These repositories are pinned research references. They are kept isolated from
the OpenSeek runtime until their original tests pass and their sensor/frame
contracts are documented.

## Fetch

```bash
bash scripts/25_fetch_planner_references.sh
```

The fetch script pins exact commits instead of following moving branches.

Run the reproducible build/test checks with:

```bash
bash scripts/26_test_planner_references.sh
```

This builds and benchmarks RAPPIDS, compiles the ROS-independent NanoMap core
and runs its upstream tests, then builds FAR Planner's six ROS2 packages.

For the direct Map2 comparison:

```bash
bash scripts/27_run_rappids_map2.sh
```

The default uses the RAPPIDS paper radii (`0.26m` physical, `0.46m`
planning). To test OpenSeek's current conservative body envelope:

```bash
PHYSICAL_RADIUS=0.6 PLANNING_RADIUS=0.75 \
  bash scripts/27_run_rappids_map2.sh
```

## Reproduction Order

### 1. RAPPIDS

- Source: <https://github.com/nlbucki/RAPPIDS>
- Paper: Bucki, Lee, and Mueller, IEEE RA-L 2020
- DOI: `10.1109/LRA.2020.3003277`
- Role: single-DepthPlanar free-space pyramids and trajectory collision checks
- Local acceptance: build the upstream `Benchmarker`, run a reduced Monte Carlo
  test, then feed the Map2 `160x96` DepthPlanar frame without converting it to a
  point cloud or mesh.

Upstream build:

```bash
cmake -S third_party/repro/RAPPIDS -B third_party/repro/RAPPIDS/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/repro/RAPPIDS/build -j
third_party/repro/RAPPIDS/build/test/Benchmarker --test_type 2 -n 100 \
  --w 160 --h 96 --f 80 --cx 79.5 --cy 47.5 --numCompTimesForTCTest 2
```

### 2. NanoMap

- Source: <https://github.com/peteflorence/nanomap_ros>
- Paper: Florence et al., ICRA 2018
- DOI: `10.1109/ICRA.2018.8463195`
- Role: lazy queries over a short history of depth observations and relative
  poses, without fusing a global occupancy map
- Constraint: the upstream ROS package targets Kinetic/Indigo. Its core under
  `src/` is ROS-independent; only that core should be ported first.

### 3. FAR Planner

- Source: <https://github.com/MichaelFYang/far_planner>, branch `humble-jazzy`
- Paper: Yang et al., IROS 2022
- DOI: `10.1109/IROS47612.2022.9981574`
- Role: incremental visibility Graph and attemptable routes in unknown space
- Constraint: the executable depends on the CMU Autonomous Exploration
  Development Environment. Reproduce it as a full ROS2 baseline before
  borrowing only its Graph update rules.

### 4. FIESTA

- Source: <https://github.com/HKUST-Aerial-Robotics/FIESTA>
- Paper: Han et al., IROS 2019
- DOI: `10.1109/IROS40897.2019.8968199`
- Role: map-based ESDF comparison only
- Constraint: ROS1 and a dense incremental map; it is not the proposed OpenSeek
  online representation.

## Integration Boundary

The upstream methods must remain independent baselines until verified. In
particular, offline `tree.ply` and a triangulated single depth frame are not
allowed to provide runtime collision labels.

The intended comparison is:

1. RAPPIDS validates the currently observed local motion directly in depth
   image space.
2. NanoMap supplies short-history proximity evidence outside the latest field
   of view.
3. FAR Planner supplies incremental optimistic/certified Graph behavior.
4. YOPO remains the fast local trajectory executor.

## Verified Locally

- RAPPIDS: upstream Release build and `Benchmarker` passed at `160x96`,
  `f=80`. On Map2, the paper radii classify `Current->2` as `FREE` and
  `Current->5` as `COLLISION`. Candidate-to-goal edges are intentionally
  deferred because the upstream planner only certifies trajectories starting
  at the current camera origin.
- NanoMap: all five upstream core tests passed. A temporary empty
  `pcl_conversions` shim is used because that header is included but unused by
  the core, while the upstream package itself targets ROS1.
- FAR Planner: all six packages on branch `humble-jazzy` built under the local
  ROS2 Humble installation.
- FIESTA: source pinned only; its ROS1 ESDF runtime is intentionally not mixed
  into the ROS2 OpenSeek stack.
