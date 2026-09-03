#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "lidar_map/lidar_map.h"
#include "pointcloud_topo/graph.h"

namespace {

using fast_planner::LIOInterface;
using fast_planner::PointType;

constexpr double kDepthPeriodS = 0.100;    // 10 Hz AirSim depth/point cloud
constexpr double kSemanticPeriodS = 0.500; // 2 Hz text heatmap
constexpr double kPlannerPeriodS = 0.200;  // 5 Hz graph update callback
constexpr double kSimulationDurationS = 5.0;

Eigen::Vector3f p(float x, float y, float z = 1.6F)
{
  return Eigen::Vector3f(x, y, z);
}

void connect(const TopoNode::Ptr &from, const TopoNode::Ptr &to)
{
  from->neighbors_.insert(to);
  to->neighbors_.insert(from);
  from->paths_[to] = {from->center_, to->center_};
  to->paths_[from] = {to->center_, from->center_};
  const float length = (from->center_ - to->center_).norm();
  from->weight_[to] = length;
  to->weight_[from] = length;
}

struct CadenceCounters {
  int depth = 0;
  int semantic = 0;
  int planner = 0;
};

pcl::PointCloud<PointType> singlePoint(const PointType &point)
{
  pcl::PointCloud<PointType> cloud;
  cloud.push_back(point);
  return cloud;
}

TEST(EpicIntegration, RealisticCadenceProducesExpectedInputCounts)
{
  CadenceCounters counts;
  double next_depth = 0.0;
  double next_semantic = 0.0;
  double next_planner = 0.0;
  for (int tick = 0; tick <= 500; ++tick) {
    const double now = tick * 0.01;
    if (now + 1e-9 >= next_depth) {
      ++counts.depth;
      next_depth += kDepthPeriodS;
    }
    if (now + 1e-9 >= next_semantic) {
      ++counts.semantic;
      next_semantic += kSemanticPeriodS;
    }
    if (now + 1e-9 >= next_planner) {
      ++counts.planner;
      next_planner += kPlannerPeriodS;
    }
  }

  // Explicit input/output contract for the simulated 5-second run.
  EXPECT_EQ(counts.depth, 51);     // t=0 ... 5.0 inclusive
  EXPECT_EQ(counts.semantic, 11);
  EXPECT_EQ(counts.planner, 26);
}

TEST(EpicIntegration, SemanticUpdateChangesTheNextPlannerDecision)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;

  auto start = std::make_shared<TopoNode>();
  auto direct = std::make_shared<TopoNode>();
  auto side = std::make_shared<TopoNode>();
  auto goal = std::make_shared<TopoNode>();
  auto semantic = std::make_shared<TopoNode>();
  start->center_ = p(0.0F, 0.0F);
  direct->center_ = p(5.0F, 0.0F);
  side->center_ = p(5.0F, 8.0F);
  goal->center_ = p(10.0F, 0.0F);
  semantic->center_ = p(5.0F, 0.5F);
  semantic->role_ = TopoNodeRole::Geometric;
  semantic->geometry_state_ = TopoGeometryState::Unknown;

  for (const auto &node : {start, direct, side, goal, semantic}) {
    region->topo_nodes_.insert(node);
  }
  connect(start, direct);
  connect(direct, goal);
  connect(start, side);
  connect(side, goal);

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, path, 0.2, 0.2F, 1.0F, {}, 4.0F));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path[1], direct);
  EXPECT_EQ(path.back(), goal);

  // The semantic frame arrives at t=0.5 s. It is the only state change; the
  // next planner tick must select the clear side branch.
  graph.updateNodeSemantic(semantic, 1.0F, 1.0F, 500000000);
  path.clear();
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, path, 0.2, 0.2F, 1.0F, {}, 4.0F));
  ASSERT_EQ(path.size(), 3U);
  // The frontier_goal remains the outer verified Bubble; the changed middle node
  // proves that the semantic update selected the clear branch.
  EXPECT_EQ(path[1], side);
  EXPECT_EQ(path.back(), goal);
  EXPECT_GT(graph.semanticRiskForEdge(start, direct),
            graph.semanticRiskForEdge(start, side) + 0.2F);
}

TEST(EpicIntegration, TenHertzDepthUpdatesPreserveTheOccupiedMap)
{
  LIOInterface map;
  map.configureStorage(0.25F, 20.0F, 20000, 0.5F);
  const Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  const Eigen::Vector3f pose = Eigen::Vector3f::Zero();
  const auto hit = singlePoint(PointType(20.0F, 0.0F, 1.6F));
  const auto free_ray = singlePoint(PointType(20.0F, 8.0F, 1.6F));

  int depth_updates = 0;
  int free_ray_updates = 0;
  for (int tick = 0; tick <= 50; ++tick) {
    const bool depth_changed = map.updateCloudWorld(hit, pose, orientation);
    const bool free_changed = map.updateFreeRaysWorld(free_ray, pose, orientation);
    if (depth_changed) ++depth_updates;
    if (free_changed) ++free_ray_updates;
  }

  EXPECT_GE(depth_updates, 1);
  EXPECT_EQ(free_ray_updates, 0);
  EXPECT_EQ(map.accumulatedCloudSnapshot().size(), 1U);
  EXPECT_TRUE(map.freeSpaceSnapshot().empty());
  EXPECT_NEAR(map.getDisToOcc(p(20.0F, 0.0F)), 0.0, 1e-5);
}

TEST(EpicIntegration, PlanarGraphProjectsFarFreeRayOntoItsLayer)
{
  // At 20 m, a pixel away from the optical center has a large vertical
  // displacement.  The planar graph must retain its horizontal free-space
  // evidence rather than rejecting the endpoint by z.
  const Eigen::Vector3f far_ray(20.0F, 8.0F, 8.0F);
  const auto projected = projectGraphPoint(far_ray, true, 1.6F);
  EXPECT_FLOAT_EQ(projected.x(), 20.0F);
  EXPECT_FLOAT_EQ(projected.y(), 8.0F);
  EXPECT_FLOAT_EQ(projected.z(), 1.6F);
}

TEST(EpicIntegration, OpenLongBubbleEdgeIsNotRejectedByAnArbitraryTwoMeterCap)
{
  auto map = std::make_shared<LIOInterface>();
  map->configureBounds(Eigen::Vector3f(-10.0F, -10.0F, 0.0F),
                       Eigen::Vector3f(20.0F, 20.0F, 4.0F));
  map->configureStorage(0.25F, 20.0F, 20000, 0.5F);
  // The obstacle is five metres to the side.  The six-metre edge is open,
  // but the old min(2 m, clearance) code gave 2+2 < 6 and rejected it.
  const auto obstacle = singlePoint(PointType(0.0F, 5.0F, 1.6F));
  map->updateCloudWorld(obstacle, Eigen::Vector3f::Zero(),
                        Eigen::Quaternionf::Identity());

  ParallelBubbleAstar astar;
  astar.lidar_map_interface_ = map;
  astar.safe_distance_ = 0.0;
  std::vector<Eigen::Vector3f> path = {p(0.0F, 0.0F), p(6.0F, 0.0F)};
  EXPECT_TRUE(astar.collisionCheck_shortenPath(path));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_TRUE(path.front().isApprox(p(0.0F, 0.0F)));
  EXPECT_TRUE(path.back().isApprox(p(6.0F, 0.0F)));
}

TEST(EpicIntegration, EdgeCrossingObservedObstacleIsRejectedBetweenEndpoints)
{
  auto map = std::make_shared<LIOInterface>();
  map->configureBounds(Eigen::Vector3f(-2.0F, -2.0F, 0.0F),
                       Eigen::Vector3f(8.0F, 2.0F, 4.0F));
  map->configureStorage(0.10F, 20.0F, 20000, 0.5F);
  // The endpoints are clear and their bubbles overlap, but the observed
  // obstacle lies in the middle of the chord. Endpoint-only checking used to
  // accept this edge and allowed a topology connection through a wall.
  map->updateCloudWorld(singlePoint(PointType(3.0F, 0.0F, 1.6F)),
                        Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());

  ParallelBubbleAstar astar;
  astar.lidar_map_interface_ = map;
  astar.resolution_ = 0.10;
  astar.safe_distance_ = 0.20;
  std::vector<Eigen::Vector3f> path = {p(0.0F, 0.0F), p(6.0F, 0.0F)};
  EXPECT_FALSE(astar.collisionCheck_shortenPath(path));
}

TEST(EpicIntegration, ObservedWallKeepsSemanticEdgeBlockedWhenNextFrameLooksAway)
{
  auto map = std::make_shared<LIOInterface>();
  map->configureBounds(Eigen::Vector3f(-2.0F, -2.0F, 0.0F),
                       Eigen::Vector3f(40.0F, 10.0F, 4.0F));
  map->configureStorage(0.10F, 40.0F, 20000, 0.5F);
  map->updateCloudWorld(singlePoint(PointType(12.0F, 0.0F, 1.6F)),
                        Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());

  ParallelBubbleAstar astar;
  astar.lidar_map_interface_ = map;
  astar.resolution_ = 0.10;
  astar.safe_distance_ = 0.61;
  std::vector<Eigen::Vector3f> initial_edge = {p(0.0F, 0.0F), p(30.0F, 0.0F)};
  EXPECT_FALSE(astar.collisionCheck_shortenPath(initial_edge));

  // A later camera frame observes another direction and does not contain the
  // wall. The sliding obstacle window must retain the earlier hit, otherwise
  // the same ordinary-semantic chord is immediately recreated through it.
  map->updateCloudWorld(singlePoint(PointType(5.0F, 6.0F, 1.6F)),
                        Eigen::Vector3f(1.0F, 0.0F, 1.6F),
                        Eigen::Quaternionf::Identity());
  std::vector<Eigen::Vector3f> rechecked_edge = {p(0.0F, 0.0F), p(30.0F, 0.0F)};
  ParallelBubbleAstar::CollisionCheckInfo info;
  EXPECT_FALSE(astar.collisionCheck_shortenPath(rechecked_edge, &info));
  EXPECT_LT(info.minimum_clearance, astar.safe_distance_);
  EXPECT_LT(std::abs(info.minimum_clearance_point.x() - 12.0F),
            astar.safe_distance_ + 0.05);
}

}  // namespace
