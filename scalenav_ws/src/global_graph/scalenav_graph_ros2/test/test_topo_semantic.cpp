#include <gtest/gtest.h>

#include <limits>
#include <thread>
#include <array>
#include <cstdlib>

#include "pointcloud_topo/graph.h"

namespace {

TEST(TopoGraphM2Contract, TcM2001ProjectsPlanarAndThreeDimensionalPoints)
{
  for (int index = 0; index < 1000; ++index) {
    const Eigen::Vector3f input(
      0.01F * static_cast<float>(index),
      -0.02F * static_cast<float>(index),
      -5.0F + 0.01F * static_cast<float>(index));
    const auto planar = projectGraphPoint(input, true, 1.6F);
    const auto spatial = projectGraphPoint(input, false, 1.6F);
    EXPECT_FLOAT_EQ(planar.x(), input.x());
    EXPECT_FLOAT_EQ(planar.y(), input.y());
    EXPECT_FLOAT_EQ(planar.z(), 1.6F);
    EXPECT_TRUE(spatial.isApprox(input));
  }
}

TEST(TopoGraphM2Contract, TcM2003RegionIndexUsesFloorForNegativeCoordinates)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f(-10.0F, -20.0F, -3.0F);
  graph.init_region_size_x_ = 2.0;
  graph.init_region_size_y_ = 4.0;
  graph.init_region_size_z_ = 1.5;

  for (int index = 0; index < 1000; ++index) {
    const Eigen::Vector3f point(
      -12.5F + 0.037F * static_cast<float>(index),
      -24.5F + 0.053F * static_cast<float>(index),
      -4.0F + 0.011F * static_cast<float>(index));
    Eigen::Vector3i actual;
    graph.getIndex(point, actual);
    const Eigen::Vector3i expected(
      static_cast<int>(std::floor((point.x() - graph.min_bd.x()) / 2.0F)),
      static_cast<int>(std::floor((point.y() - graph.min_bd.y()) / 4.0F)),
      static_cast<int>(std::floor((point.z() - graph.min_bd.z()) / 1.5F)));
    EXPECT_EQ(actual, expected);
  }
}

TEST(TopoGraphM2Contract, TcM2004IndexBoundaryRejectsOutsideConfiguredGrid)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f(-10.0F, -20.0F, -3.0F);
  graph.init_region_size_x_ = 2.0;
  graph.init_region_size_y_ = 4.0;
  graph.init_region_size_z_ = 1.5;
  graph.x_len = 5;
  graph.y_len = 4;
  graph.z_len = 3;

  for (int repetition = 0; repetition < 50; ++repetition) {
    Eigen::Vector3f low;
    Eigen::Vector3f high;
    EXPECT_TRUE(graph.index2boundary(Eigen::Vector3i(0, 0, 0), low, high));
    EXPECT_TRUE(low.isApprox(graph.min_bd));
    EXPECT_TRUE(high.isApprox(graph.min_bd + Eigen::Vector3f(2.0F, 4.0F, 1.5F)));
    EXPECT_TRUE(graph.index2boundary(Eigen::Vector3i(4, 3, 2), low, high));
    EXPECT_TRUE(low.isApprox(Eigen::Vector3f(-2.0F, -8.0F, 0.0F)));
  }
  for (int repetition = 0; repetition < 50; ++repetition) {
    Eigen::Vector3f low;
    Eigen::Vector3f high;
    EXPECT_FALSE(graph.index2boundary(Eigen::Vector3i(-1, 0, 0), low, high));
    EXPECT_FALSE(graph.index2boundary(Eigen::Vector3i(5, 0, 0), low, high));
  }
}

TEST(TopoGraphM2Contract, TcM2005ConcurrentSameRegionReturnsOnePointer)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    const Eigen::Vector3i key(2, -3, 1);
    std::array<RegionNode::Ptr, 32> results;
    std::vector<std::thread> workers;
    workers.reserve(results.size());
    for (std::size_t index = 0; index < results.size(); ++index) {
      workers.emplace_back([&graph, &results, &key, index]() {
        results[index] = graph.getRegionNode(key);
      });
    }
    for (auto &worker : workers) worker.join();
    ASSERT_TRUE(results.front());
    for (const auto &result : results) EXPECT_EQ(result, results.front());
    EXPECT_EQ(graph.reg_map_idx2ptr_.size(), 1U);
  }
}

TEST(TopoGraphM2Contract, TcM2012UnionSetClustersTransitively)
{
  BubbleUnionSet union_set(0.5);
  const Eigen::Vector3f region_center = Eigen::Vector3f::Zero();
  for (int repetition = 0; repetition < 100; ++repetition) {
    auto a = std::make_shared<BubbleNode>(1.0, Eigen::Vector3f(0.0F, 0.0F, 0.0F));
    auto b = std::make_shared<BubbleNode>(1.0, Eigen::Vector3f(1.0F, 0.0F, 0.0F));
    auto c = std::make_shared<BubbleNode>(1.0, Eigen::Vector3f(2.0F, 0.0F, 0.0F));
    auto isolated = std::make_shared<BubbleNode>(1.0, Eigen::Vector3f(10.0F, 0.0F, 0.0F));
    std::vector<TopoNode::Ptr> nodes;
    Eigen::Vector3f center = region_center;
    union_set.unionSetCluster({a, b, c, isolated}, nodes, center);
    EXPECT_EQ(nodes.size(), 2U);

    nodes.clear();
    center = region_center;
    union_set.unionSetCluster({}, nodes, center);
    EXPECT_TRUE(nodes.empty());
  }
}

TEST(TopoGraphM2Contract, TcM2013RemoveNodesClearsAllReverseEdgeState)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    auto left = std::make_shared<TopoNode>();
    auto right = std::make_shared<TopoNode>();
    left->center_ = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
    right->center_ = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    left->neighbors_.insert(right);
    right->neighbors_.insert(left);
    left->paths_[right] = {left->center_, right->center_};
    right->paths_[left] = {right->center_, left->center_};
    left->weight_[right] = 1.0F;
    right->weight_[left] = 1.0F;
    left->edge_clearance_[right] = 2.0F;
    right->edge_clearance_[left] = 2.0F;
    graph.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(left);
    graph.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(right);
    std::vector<TopoNode::Ptr> removed{left};
    graph.removeNodes(removed);
    EXPECT_EQ(graph.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.count(left), 0U);
    EXPECT_TRUE(left->neighbors_.empty());
    EXPECT_TRUE(left->paths_.empty());
    EXPECT_TRUE(left->weight_.empty());
    EXPECT_TRUE(left->edge_clearance_.empty());
    EXPECT_EQ(right->neighbors_.count(left), 0U);
    EXPECT_EQ(right->paths_.count(left), 0U);
    EXPECT_EQ(right->weight_.count(left), 0U);
    EXPECT_EQ(right->edge_clearance_.count(left), 0U);
  }
}

TEST(TopoGraphM2Contract, TcM2016InsertNodeCreatesSymmetricEdgesAndWitnesses)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    graph.min_bd = Eigen::Vector3f(-10.0F, -10.0F, -2.0F);
    graph.init_region_size_x_ = 5.0;
    graph.init_region_size_y_ = 5.0;
    graph.init_region_size_z_ = 2.0;
    graph.parallel_bubble_astar_ = std::make_shared<ParallelBubbleAstar>();
    graph.parallel_bubble_astar_->lidar_map_interface_ =
      std::make_shared<fast_planner::LIOInterface>();
    auto inserted = std::make_shared<TopoNode>();
    auto left = std::make_shared<TopoNode>();
    auto right = std::make_shared<TopoNode>();
    inserted->center_ = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
    left->center_ = Eigen::Vector3f(-1.0F, 0.0F, 0.0F);
    right->center_ = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    std::vector<TopoNode::Ptr> neighbors{left, right};
    std::vector<std::vector<Eigen::Vector3f>> paths{
      {inserted->center_, left->center_}, {inserted->center_, right->center_}};
    graph.insertNode(inserted, neighbors, paths);
    ASSERT_EQ(inserted->neighbors_.size(), 2U);
    for (const auto &neighbor : neighbors) {
      EXPECT_EQ(neighbor->neighbors_.count(inserted), 1U);
      ASSERT_EQ(inserted->paths_.at(neighbor).size(), 2U);
      ASSERT_EQ(neighbor->paths_.at(inserted).size(), 2U);
      EXPECT_TRUE(inserted->paths_.at(neighbor).front().isApprox(inserted->center_));
      EXPECT_TRUE(neighbor->paths_.at(inserted).front().isApprox(neighbor->center_));
      EXPECT_FLOAT_EQ(inserted->weight_.at(neighbor), neighbor->weight_.at(inserted));
      EXPECT_FLOAT_EQ(
        inserted->edge_clearance_.at(neighbor), neighbor->edge_clearance_.at(inserted));
    }
  }
}

TEST(TopoGraphM2Contract, TcM2017RemoveNodeIsIdempotent)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    auto node = std::make_shared<TopoNode>();
    auto neighbor = std::make_shared<TopoNode>();
    node->neighbors_.insert(neighbor);
    neighbor->neighbors_.insert(node);
    node->paths_[neighbor] = {Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitX()};
    neighbor->paths_[node] = {Eigen::Vector3f::UnitX(), Eigen::Vector3f::Zero()};
    node->weight_[neighbor] = 1.0F;
    neighbor->weight_[node] = 1.0F;
    graph.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(node);
    graph.removeNode(node);
    EXPECT_TRUE(node->neighbors_.empty());
    EXPECT_TRUE(neighbor->neighbors_.empty());
    graph.removeNode(node);
    EXPECT_TRUE(node->neighbors_.empty());
    EXPECT_TRUE(neighbor->neighbors_.empty());
  }
}

TEST(TopoGraphConcurrency, CreatesLazyRegionsFromMultipleWorkers)
{
  TopoGraph graph;
  constexpr int kWorkers = 8;
  constexpr int kRegionsPerWorker = 500;
  std::vector<std::thread> workers;
  workers.reserve(kWorkers);
  for (int worker = 0; worker < kWorkers; ++worker) {
    workers.emplace_back([&graph, worker]() {
      for (int index = 0; index < kRegionsPerWorker; ++index) {
        const Eigen::Vector3i key(worker, index, worker + index);
        const auto region = graph.getRegionNode(key);
        ASSERT_NE(region, nullptr);
        EXPECT_EQ(region->region_idx_, key);
      }
    });
  }
  for (auto &worker : workers) worker.join();
  EXPECT_EQ(graph.reg_map_idx2ptr_.size(),
            static_cast<std::size_t>(kWorkers * kRegionsPerWorker));
}

TEST(TopoNodeModel, UsesOneNodeStructureForSemanticPromotion)
{
  auto node = std::make_shared<TopoNode>();
  EXPECT_EQ(node->role_, TopoNodeRole::Geometric);
  EXPECT_EQ(node->geometry_state_, TopoGeometryState::Verified);

  node->role_ = TopoNodeRole::Geometric;
  node->geometry_state_ = TopoGeometryState::Unknown;
  node->center_ = Eigen::Vector3f(5.0F, 0.0F, 1.6F);
  node->semantic_score_ = 0.85F;
  node->semantic_confidence_ = 0.7F;

  // A depth-confirmed Bubble promotes the same TopoNode state; it does not
  // create a second semantic node class or discard the semantic evidence.
  node->role_ = TopoNodeRole::Geometric;
  node->geometry_state_ = TopoGeometryState::Verified;
  EXPECT_EQ(node->role_, TopoNodeRole::Geometric);
  EXPECT_EQ(node->geometry_state_, TopoGeometryState::Verified);
  EXPECT_NEAR(node->semantic_score_, 0.85F, 1e-6F);
  EXPECT_NEAR(node->semantic_confidence_, 0.7F, 1e-6F);
}

TEST(TopoNodeModel, OdomUsesTheSamePersistentNodeStructure)
{
  auto node = std::make_shared<TopoNode>();
  node->is_viewpoint_ = true;
  node->role_ = TopoNodeRole::Odom;
  node->geometry_state_ = TopoGeometryState::Verified;
  EXPECT_TRUE(node->is_viewpoint_);
  EXPECT_EQ(node->role_, TopoNodeRole::Odom);
  EXPECT_EQ(node->geometry_state_, TopoGeometryState::Verified);
}

TEST(TopoNodeModel, SemanticUpdateKeepsOdomOnPlanarLayer)
{
  TopoGraph graph;
  graph.planar_graph_ = true;
  graph.planar_z_ = 1.598123F;
  graph.odom_node_ = std::make_shared<TopoNode>();
  graph.lidar_map_interface_ = std::make_shared<fast_planner::LIOInterface>();
  graph.parallel_bubble_astar_ = std::make_shared<ParallelBubbleAstar>();

  const float nan = std::numeric_limits<float>::quiet_NaN();
  graph.insertSemanticNodes(
    {Eigen::Vector3f(nan, nan, nan)}, {0.5F}, 0.5F,
    Eigen::Vector3f(0.0F, 0.0F, 1.600000F), 1);

  EXPECT_FLOAT_EQ(graph.odom_node_->center_.z(), graph.planar_z_);
}

TEST(TopoNodeModel, SemanticSnapshotUsesOrdinaryObservedNodes)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  auto semantic = std::make_shared<TopoNode>();
  semantic->role_ = TopoNodeRole::Geometric;
  semantic->geometry_state_ = TopoGeometryState::Unknown;
  semantic->semantic_observations_ = 1;
  auto geometric = std::make_shared<TopoNode>();
  geometric->role_ = TopoNodeRole::Geometric;
  region->topo_nodes_.insert(semantic);
  region->topo_nodes_.insert(geometric);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;

  const auto candidates = graph.semanticNodes();
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), semantic);
  EXPECT_EQ(candidates.front()->geometry_state_, TopoGeometryState::Unknown);
}

TEST(TopoNodeModel, SemanticSnapshotCanBeLimitedToTheLocalPlanningWindow)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  auto local = std::make_shared<TopoNode>();
  local->center_ = Eigen::Vector3f(5.0F, 0.0F, 1.6F);
  local->semantic_observations_ = 1;
  auto history = std::make_shared<TopoNode>();
  history->center_ = Eigen::Vector3f(80.0F, 0.0F, 1.6F);
  history->semantic_observations_ = 1;
  region->topo_nodes_.insert(local);
  region->topo_nodes_.insert(history);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;

  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  const auto candidates = graph.semanticNodes(&origin, 40.0F);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), local);
  EXPECT_EQ(graph.semanticNodes().size(), 2U);
}

TEST(TopoNodeModel, SemanticPatchUsesFixedVirtualDepth)
{
  const Eigen::Vector3f camera_translation(0.5F, 0.0F, -0.1F);
  const Eigen::Vector3f center = virtualSemanticPointFlu(
    0.5F, 0.5F, 90.0F, 60.0F, 30.0F, camera_translation);
  const Eigen::Vector3f corner = virtualSemanticPointFlu(
    0.0F, 0.0F, 90.0F, 60.0F, 30.0F, camera_translation);

  EXPECT_NEAR((center - camera_translation).norm(), 30.0F, 1e-5F);
  EXPECT_GT((corner - camera_translation).norm(), 30.0F);
  EXPECT_NEAR(center.x(), 30.5F, 1e-5F);
  EXPECT_NEAR(corner.x(), 30.5F, 1e-5F);
  EXPECT_GT(corner.y(), 0.0F);
  EXPECT_GT(corner.z(), camera_translation.z());
}

TEST(TopoNodeModel, VirtualSemanticDepthDoesNotDependOnMeasuredDepth)
{
  const Eigen::Vector3f camera_translation = Eigen::Vector3f::Zero();
  const Eigen::Vector3f point = virtualSemanticPointFlu(
    0.5F, 0.5F, 90.0F, 60.0F, 30.0F, camera_translation);

  EXPECT_FLOAT_EQ(point.x(), 30.0F);
  EXPECT_GT(point.x(), 20.0F);
}

TEST(TopoSemanticCost, RiskAnchorHasAnExplicitNoiseFloor)
{
  EXPECT_FALSE(isSemanticRiskAnchor(0.02F, 1.0F, 0.35F));
  EXPECT_FALSE(isSemanticRiskAnchor(0.34F, 1.0F, 0.35F));
  EXPECT_FALSE(isSemanticRiskAnchor(0.80F, 0.49F, 0.35F));
  EXPECT_TRUE(isSemanticRiskAnchor(0.80F, 0.50F, 0.35F));
}

TEST(TopoSemanticCost, RawHeatmapIsConvertedToFrameRelativeRisk)
{
  EXPECT_NEAR(calibrateSemanticScore(0.40F, 0.20F), 0.25F, 1e-6F);
  EXPECT_NEAR(calibrateSemanticScore(0.70F, 0.20F), 0.625F, 1e-6F);
  EXPECT_FLOAT_EQ(calibrateSemanticScore(0.10F, 0.20F), 0.0F);
}

TEST(TopoSemanticCost, ForestFilledFrameKeepsHighRisk)
{
  EXPECT_GT(calibrateSemanticScore(0.50F, 0.496F), 0.30F);
  EXPECT_GT(calibrateSemanticScore(0.496F, 0.496F), 0.30F);
}

TEST(TopoSemanticCost, MaxPooledPatchBaselineUsesLowerBackgroundQuantile)
{
  const std::vector<float> patches{0.70F, 0.72F, 0.74F, 0.76F, 0.78F,
                                   0.80F, 0.82F, 0.84F, 0.86F};
  EXPECT_NEAR(semanticFrameBaseline(patches, 0.25F), 0.74F, 1e-6F);
}

TEST(TopoSemanticCost, BaselineIgnoresNonFinitePatchScores)
{
  const std::vector<float> patches{
    std::numeric_limits<float>::quiet_NaN(), 0.2F,
    std::numeric_limits<float>::infinity(), 0.8F};
  EXPECT_NEAR(semanticFrameBaseline(patches, 0.25F), 0.2F, 1e-6F);
}

TEST(TopoGraphM3Contract, TcM3001BaselineCoversQuantilesAndNonFiniteInput)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<float> values{nan, 0.8F, 0.2F, inf, 0.6F, 0.4F};
  for (int repetition = 0; repetition < 100; ++repetition) {
    EXPECT_FLOAT_EQ(semanticFrameBaseline({}, 0.25F), 0.0F);
    EXPECT_FLOAT_EQ(semanticFrameBaseline({nan, inf}, 0.25F), 0.0F);
    EXPECT_FLOAT_EQ(semanticFrameBaseline(values, 0.0F), 0.2F);
    EXPECT_FLOAT_EQ(semanticFrameBaseline(values, 0.25F), 0.2F);
    EXPECT_FLOAT_EQ(semanticFrameBaseline(values, 1.0F), 0.8F);
    EXPECT_FLOAT_EQ(semanticFrameBaseline({-1.0F, 2.0F}, 0.0F), 0.0F);
    EXPECT_FLOAT_EQ(semanticFrameBaseline({-1.0F, 2.0F}, 1.0F), 1.0F);
  }
}

TEST(TopoGraphM3Contract, TcM3002CalibrationIsFiniteClampedAndMonotonic)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_FLOAT_EQ(calibrateSemanticScore(nan, 0.2F), 0.0F);
    EXPECT_FLOAT_EQ(calibrateSemanticScore(0.8F, inf), 0.0F);
    EXPECT_FLOAT_EQ(calibrateSemanticScore(-1.0F, 0.2F), 0.0F);
    EXPECT_FLOAT_EQ(calibrateSemanticScore(2.0F, 0.2F), 1.0F);
    EXPECT_LT(calibrateSemanticScore(0.4F, 0.2F),
              calibrateSemanticScore(0.8F, 0.2F));
    EXPECT_NEAR(calibrateSemanticScore(0.7F, 0.2F), 0.625F, 1e-6F);
  }
}

TEST(TopoGraphM3Contract, TcM3003RiskAnchorUsesInclusiveThresholds)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_FALSE(isSemanticRiskAnchor(0.349F, 0.5F, 0.35F, 0.5F));
    EXPECT_FALSE(isSemanticRiskAnchor(0.35F, 0.499F, 0.35F, 0.5F));
    EXPECT_TRUE(isSemanticRiskAnchor(0.35F, 0.5F, 0.35F, 0.5F));
    EXPECT_TRUE(isSemanticRiskAnchor(0.9F, 0.9F, 0.35F, 0.5F));
    EXPECT_FALSE(isSemanticRiskAnchor(nan, 1.0F, 0.35F, 0.5F));
    EXPECT_FALSE(isSemanticRiskAnchor(1.0F, inf, 0.35F, 0.5F));
  }
}

TEST(TopoGraphM3Contract, TcM3004ProjectionPreservesThreeVerticalRows)
{
  const Eigen::Vector3f camera(0.5F, -0.25F, 0.1F);
  for (int repetition = 0; repetition < 500; ++repetition) {
    const float u = repetition % 2 == 0 ? 0.0F : 1.0F;
    const auto upper = virtualSemanticPointFlu(u, 0.0F, 90.0F, 60.0F, 30.0F, camera);
    const auto middle = virtualSemanticPointFlu(u, 0.5F, 90.0F, 60.0F, 30.0F, camera);
    const auto lower = virtualSemanticPointFlu(u, 1.0F, 90.0F, 60.0F, 30.0F, camera);
    EXPECT_FLOAT_EQ(upper.x() - camera.x(), 30.0F);
    EXPECT_FLOAT_EQ(middle.x() - camera.x(), 30.0F);
    EXPECT_FLOAT_EQ(lower.x() - camera.x(), 30.0F);
    EXPECT_GT(upper.z(), middle.z());
    EXPECT_GT(middle.z(), lower.z());
  }
  const auto center = virtualSemanticPointFlu(
    0.5F, 0.5F, 90.0F, 60.0F, 30.0F, camera);
  const auto corner = virtualSemanticPointFlu(
    0.0F, 0.0F, 90.0F, 60.0F, 30.0F, camera);
  for (int repetition = 0; repetition < 500; ++repetition) {
    EXPECT_GT((corner - camera).norm(), (center - camera).norm());
  }
}

TEST(TopoGraphM3Contract, TcM3006SemanticInsertionReusesNearbyIdentity)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    graph.min_bd = Eigen::Vector3f(-50.0F, -50.0F, -50.0F);
    graph.init_region_size_x_ = 5.0;
    graph.init_region_size_y_ = 5.0;
    graph.init_region_size_z_ = 5.0;
    graph.lidar_map_interface_ = std::make_shared<fast_planner::LIOInterface>();
    graph.lidar_map_interface_->configureBounds(
      Eigen::Vector3f(-50.0F, -50.0F, -50.0F),
      Eigen::Vector3f(50.0F, 50.0F, 50.0F));
    graph.parallel_bubble_astar_ = std::make_shared<ParallelBubbleAstar>();
    graph.parallel_bubble_astar_->lidar_map_interface_ = graph.lidar_map_interface_;

    ASSERT_EQ(graph.insertSemanticNodes(
      {Eigen::Vector3f(10.0F, 0.0F, 2.0F)}, {0.4F}, 0.5F,
      Eigen::Vector3f::Zero(), 100), 1U);
    auto nodes = graph.semanticNodes();
    ASSERT_EQ(nodes.size(), 1U);
    const auto id = nodes.front()->persistent_id_;
    EXPECT_TRUE(nodes.front()->neighbors_.empty());

    ASSERT_EQ(graph.insertSemanticNodes(
      {Eigen::Vector3f(12.4F, 0.0F, 2.0F)}, {0.9F}, 0.5F,
      Eigen::Vector3f::Zero(), 200), 1U);
    nodes = graph.semanticNodes();
    ASSERT_EQ(nodes.size(), 1U);
    EXPECT_EQ(nodes.front()->persistent_id_, id);
    EXPECT_EQ(nodes.front()->semantic_observations_, 2U);
    EXPECT_FLOAT_EQ(nodes.front()->semantic_score_, 0.9F);

    ASSERT_EQ(graph.insertSemanticNodes(
      {Eigen::Vector3f(20.0F, 0.0F, 2.0F)}, {0.7F}, 0.5F,
      Eigen::Vector3f::Zero(), 300), 1U);
    nodes = graph.semanticNodes();
    ASSERT_EQ(nodes.size(), 2U);
    const auto disconnected = std::find_if(nodes.begin(), nodes.end(),
      [](const TopoNode::Ptr &node) { return node->center_.x() > 15.0F; });
    ASSERT_NE(disconnected, nodes.end());
    EXPECT_TRUE((*disconnected)->neighbors_.empty());
  }
}

TEST(TopoGraphM3Contract, TcM3007SemanticEmaAndClamping)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    graph.init_region_size_x_ = 5.0;
    graph.init_region_size_y_ = 5.0;
    graph.init_region_size_z_ = 5.0;
    auto node = std::make_shared<TopoNode>();
    node->center_ = Eigen::Vector3f::Zero();
    graph.updateNodeSemantic(node, 0.4F, 1.0F, 100, 0.4F);
    graph.updateNodeSemantic(node, 0.8F, 0.0F, 200, 0.8F);
    EXPECT_FLOAT_EQ(node->semantic_score_, 0.4F);
    EXPECT_FLOAT_EQ(node->semantic_confidence_, 0.4F);
    graph.updateNodeSemantic(node, 0.8F, 0.5F, 300, 0.8F);
    EXPECT_NEAR(node->semantic_score_, 0.6F, 1e-6F);
    EXPECT_NEAR(node->semantic_confidence_, 0.6F, 1e-6F);
    graph.updateNodeSemantic(node, 1.2F, 1.0F, 400, -0.5F);
    EXPECT_FLOAT_EQ(node->semantic_score_, 1.0F);
    EXPECT_FLOAT_EQ(node->semantic_confidence_, 0.0F);
    EXPECT_EQ(node->semantic_observations_, 4U);
    EXPECT_EQ(node->semantic_stamp_ns_, 400);
  }
}

TEST(TopoGraphM3Contract, TcM3008SemanticQueryFiltersRangeAndEvidence)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  auto local = std::make_shared<TopoNode>();
  auto far = std::make_shared<TopoNode>();
  auto no_evidence = std::make_shared<TopoNode>();
  local->center_ = Eigen::Vector3f(5.0F, 0.0F, 1.6F);
  far->center_ = Eigen::Vector3f(50.0F, 0.0F, 1.6F);
  no_evidence->center_ = Eigen::Vector3f(2.0F, 0.0F, 1.6F);
  local->semantic_observations_ = 1;
  far->semantic_observations_ = 1;
  region->topo_nodes_.insert(local);
  region->topo_nodes_.insert(far);
  region->topo_nodes_.insert(no_evidence);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const auto nodes = graph.semanticNodes(&origin, 10.0F);
    ASSERT_EQ(nodes.size(), 1U);
    EXPECT_EQ(nodes.front(), local);
  }
}

TEST(TopoGraphM3Contract, TcM3009SnapshotIsDetachedFromMemory)
{
  TopoGraph graph;
  graph.init_region_size_x_ = 5.0;
  graph.init_region_size_y_ = 5.0;
  graph.init_region_size_z_ = 5.0;
  std::vector<TopoNode::Ptr> nodes;
  for (int index = 0; index < 3; ++index) {
    auto node = std::make_shared<TopoNode>();
    node->center_ = Eigen::Vector3f(static_cast<float>(index), 0.0F, 1.6F);
    graph.updateNodeSemantic(node, 0.2F * static_cast<float>(index + 1),
      1.0F, 100 + index);
    nodes.push_back(node);
  }
  for (int repetition = 0; repetition < 100; ++repetition) {
    auto snapshot = graph.semanticMemorySnapshot();
    ASSERT_EQ(snapshot.size(), 3U);
    snapshot.front().score = 1.0F;
    const auto next = graph.semanticMemorySnapshot();
    const auto unchanged = std::find_if(next.begin(), next.end(),
      [&](const TopoSemanticRecord &record) {
        return record.node_id == snapshot.front().node_id;
      });
    ASSERT_NE(unchanged, next.end());
    EXPECT_NE(unchanged->score, snapshot.front().score);
  }
}

TEST(TopoGraphM3Contract, TcM3010LoadKeepsNewestRecordPerIdentity)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    const TopoSemanticRecord newer{
      42, Eigen::Vector3f(2.0F, 0.0F, 1.6F), Eigen::Vector3i::Zero(),
      0.9F, 0.8F, 5, 200};
    const TopoSemanticRecord older{
      42, Eigen::Vector3f(1.0F, 0.0F, 1.6F), Eigen::Vector3i::Zero(),
      0.2F, 0.3F, 2, 100};
    graph.loadSemanticMemory({newer, older});
    const auto snapshot = graph.semanticMemorySnapshot();
    ASSERT_EQ(snapshot.size(), 1U);
    EXPECT_EQ(snapshot.front().stamp_ns, newer.stamp_ns);
    EXPECT_FLOAT_EQ(snapshot.front().score, newer.score);
    EXPECT_TRUE(snapshot.front().center.isApprox(newer.center));
  }
}

TEST(TopoGraphM3Contract, TcM3011SizeAlwaysMatchesSnapshot)
{
  TopoGraph graph;
  EXPECT_EQ(graph.semanticMemorySize(), 0U);
  graph.loadSemanticMemory({
    {1, Eigen::Vector3f(0.0F, 0.0F, 1.6F), Eigen::Vector3i::Zero(), 0.2F, 1.0F, 1, 10},
    {2, Eigen::Vector3f(1.0F, 0.0F, 1.6F), Eigen::Vector3i::Zero(), 0.4F, 1.0F, 1, 20}});
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_EQ(graph.semanticMemorySize(), graph.semanticMemorySnapshot().size());
  }
}

TEST(TopoGraphM3Contract, TcM3012RestoreHonorsDistanceAndUnavailableIds)
{
  for (int repetition = 0; repetition < 100; ++repetition) {
    TopoGraph graph;
    graph.min_bd = Eigen::Vector3f(-10.0F, -10.0F, -10.0F);
    graph.init_region_size_x_ = 5.0;
    graph.init_region_size_y_ = 5.0;
    graph.init_region_size_z_ = 5.0;
    graph.loadSemanticMemory({
      {7, Eigen::Vector3f(0.0F, 0.0F, 1.6F), Eigen::Vector3i::Zero(),
       0.8F, 0.7F, 3, 100}});
    auto near = std::make_shared<TopoNode>();
    near->center_ = Eigen::Vector3f(0.2F, 0.0F, 1.6F);
    std::vector<TopoNode::Ptr> near_nodes{near};
    ASSERT_EQ(graph.restoreNodeSemanticMemory(near_nodes), 1U);
    EXPECT_EQ(near->persistent_id_, 7U);
    EXPECT_FLOAT_EQ(near->semantic_score_, 0.8F);

    auto claimed = std::make_shared<TopoNode>();
    claimed->center_ = Eigen::Vector3f(0.1F, 0.0F, 1.6F);
    std::vector<TopoNode::Ptr> claimed_nodes{claimed};
    EXPECT_EQ(graph.restoreNodeSemanticMemory(claimed_nodes, {7}), 0U);
    EXPECT_NE(claimed->persistent_id_, 7U);

    auto far = std::make_shared<TopoNode>();
    far->center_ = Eigen::Vector3f(8.0F, 0.0F, 1.6F);
    std::vector<TopoNode::Ptr> far_nodes{far};
    EXPECT_EQ(graph.restoreNodeSemanticMemory(far_nodes), 0U);
    EXPECT_EQ(far->semantic_observations_, 0U);
  }
}

TEST(TopoGraphM3Contract, TcM3017OverlappingFramesReusePersistentIdentity)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f(-50.0F, -50.0F, -50.0F);
  graph.init_region_size_x_ = 5.0;
  graph.init_region_size_y_ = 5.0;
  graph.init_region_size_z_ = 5.0;
  graph.lidar_map_interface_ = std::make_shared<fast_planner::LIOInterface>();
  graph.lidar_map_interface_->configureBounds(
    Eigen::Vector3f(-50.0F, -50.0F, -50.0F),
    Eigen::Vector3f(50.0F, 50.0F, 50.0F));
  graph.parallel_bubble_astar_ = std::make_shared<ParallelBubbleAstar>();
  graph.parallel_bubble_astar_->lidar_map_interface_ = graph.lidar_map_interface_;
  ASSERT_EQ(graph.insertSemanticNodes(
    {Eigen::Vector3f(30.0F, 0.0F, 1.6F)}, {0.5F}, 0.5F,
    Eigen::Vector3f::Zero(), 1), 1U);
  const auto initial = graph.semanticNodes();
  ASSERT_EQ(initial.size(), 1U);
  const auto persistent_id = initial.front()->persistent_id_;

  for (int frame = 0; frame < 1000; ++frame) {
    const float offset = 0.001F * static_cast<float>(frame % 100);
    ASSERT_EQ(graph.insertSemanticNodes(
      {Eigen::Vector3f(30.0F + offset, 0.0F, 1.6F)}, {0.6F}, 0.5F,
      Eigen::Vector3f(2.65F, 0.0F, 0.0F), frame + 2), 1U);
  }
  const auto final_nodes = graph.semanticNodes();
  ASSERT_EQ(final_nodes.size(), 1U);
  EXPECT_EQ(final_nodes.front()->persistent_id_, persistent_id);
  EXPECT_EQ(final_nodes.front()->semantic_observations_, 1001U);
  EXPECT_EQ(graph.semanticMemorySize(), 1U);
}

TEST(TopoGraphSemanticPrune, RemovesPassedHistoryAndPreservesProtectedEvidence)
{
  TopoGraph graph;
  auto region = graph.getRegionNode(Eigen::Vector3i::Zero());
  auto verified = std::make_shared<TopoNode>();
  verified->center_ = Eigen::Vector3f::Zero();
  verified->geometry_state_ = TopoGeometryState::Verified;
  region->topo_nodes_.insert(verified);

  auto make_virtual = [&](float x, float risk, std::int64_t stamp) {
    auto node = std::make_shared<TopoNode>();
    node->center_ = Eigen::Vector3f(x, 0.0F, 1.6F);
    node->geometry_state_ = TopoGeometryState::Unknown;
    graph.updateNodeSemantic(node, risk, 1.0F, stamp, 1.0F);
    region->topo_nodes_.insert(node);
    return node;
  };
  auto behind = make_virtual(-5.0F, 0.8F, 100);
  auto weak = make_virtual(5.0F, 0.1F, 200);
  auto strong = make_virtual(6.0F, 0.9F, 300);
  auto active = make_virtual(7.0F, 0.2F, 500);
  auto protected_node = make_virtual(8.0F, 0.3F, 400);

  behind->neighbors_.insert(verified);
  verified->neighbors_.insert(behind);
  behind->paths_[verified] = {behind->center_, verified->center_};
  verified->paths_[behind] = {verified->center_, behind->center_};
  behind->weight_[verified] = 5.0F;
  verified->weight_[behind] = 5.0F;

  const auto result = graph.pruneVirtualSemanticNodes(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitX(), 2.0F, 3U, 500,
    {protected_node->persistent_id_});
  EXPECT_EQ(result.before, 5U);
  EXPECT_EQ(result.removed_behind, 1U);
  EXPECT_EQ(result.removed_capacity, 1U);
  EXPECT_EQ(result.after, 3U);
  EXPECT_EQ(graph.semanticMemorySize(), 3U);
  EXPECT_EQ(region->topo_nodes_.count(behind), 0U);
  EXPECT_EQ(region->topo_nodes_.count(weak), 0U);
  EXPECT_EQ(region->topo_nodes_.count(strong), 1U);
  EXPECT_EQ(region->topo_nodes_.count(active), 1U);
  EXPECT_EQ(region->topo_nodes_.count(protected_node), 1U);
  EXPECT_EQ(verified->neighbors_.count(behind), 0U);
}

TEST(TopoGraphSemanticPrune, AstarNeverUsesUnknownAsTransitTopology)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto unknown = std::make_shared<TopoNode>();
  auto detour = std::make_shared<TopoNode>();
  auto goal = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  unknown->center_ = Eigen::Vector3f(2.0F, 0.0F, 3.0F);
  detour->center_ = Eigen::Vector3f(2.0F, 2.0F, 1.6F);
  goal->center_ = Eigen::Vector3f(4.0F, 0.0F, 1.6F);
  unknown->geometry_state_ = TopoGeometryState::Unknown;
  for (const auto &node : {start, detour, goal}) {
    node->geometry_state_ = TopoGeometryState::Verified;
  }
  auto connect = [](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
    const float length = (left->center_ - right->center_).norm();
    left->neighbors_.insert(right);
    right->neighbors_.insert(left);
    left->paths_[right] = {left->center_, right->center_};
    right->paths_[left] = {right->center_, left->center_};
    left->weight_[right] = length;
    right->weight_[left] = length;
  };
  connect(start, unknown);
  connect(unknown, goal);

  std::vector<TopoNode::Ptr> path;
  EXPECT_FALSE(graph.goalDirectedSearch(
    start, goal->center_, path, 0.2, 1.0F, 1.0F, {}, 0.0F,
    20.0F, &start->center_, 0.0F, true));
  EXPECT_TRUE(path.empty());

  connect(start, detour);
  connect(detour, goal);
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, path, 0.2, 1.0F, 1.0F, {}, 0.0F,
    20.0F, &start->center_, 0.0F, true));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path[1], detour);
  EXPECT_EQ(path.back(), goal);
}

TEST(TopoGraphM3Contract, TcM3018FixedLayerDoesNotFlattenSemanticRows)
{
  const Eigen::Vector3f camera = Eigen::Vector3f::Zero();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const auto upper = virtualSemanticPointFlu(
      0.5F, 1.0F / 6.0F, 90.0F, 60.0F, 30.0F, camera);
    const auto middle = virtualSemanticPointFlu(
      0.5F, 0.5F, 90.0F, 60.0F, 30.0F, camera);
    const auto lower = virtualSemanticPointFlu(
      0.5F, 5.0F / 6.0F, 90.0F, 60.0F, 30.0F, camera);
    EXPECT_GT(upper.z(), middle.z());
    EXPECT_GT(middle.z(), lower.z());
    EXPECT_GT((upper - middle).norm(), 1.5F);
    EXPECT_GT((middle - lower).norm(), 1.5F);
  }
}

TEST(TopoGraphM3Contract, TcM3021OnlyCurrentUnknownGenerationParticipates)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i::Zero());
  auto current = std::make_shared<TopoNode>();
  auto old = std::make_shared<TopoNode>();
  auto verified = std::make_shared<TopoNode>();
  for (const auto &node : {current, old, verified}) {
    node->semantic_observations_ = 1;
    node->semantic_score_ = 0.8F;
    node->semantic_confidence_ = 1.0F;
    region->topo_nodes_.insert(node);
  }
  current->geometry_state_ = TopoGeometryState::Unknown;
  current->semantic_stamp_ns_ = 200;
  old->geometry_state_ = TopoGeometryState::Unknown;
  old->semantic_stamp_ns_ = 100;
  verified->geometry_state_ = TopoGeometryState::Verified;
  verified->semantic_stamp_ns_ = 50;
  graph.reg_map_idx2ptr_[Eigen::Vector3i::Zero()] = region;

  for (int repetition = 0; repetition < 1000; ++repetition) {
    std::size_t skipped = 0;
    const auto active = graph.semanticNodes(nullptr,
      std::numeric_limits<float>::infinity(), 200, &skipped);
    EXPECT_EQ(active.size(), 2U);
    EXPECT_NE(std::find(active.begin(), active.end(), current), active.end());
    EXPECT_NE(std::find(active.begin(), active.end(), verified), active.end());
    EXPECT_EQ(std::find(active.begin(), active.end(), old), active.end());
    EXPECT_EQ(skipped, 1U);

    const auto expired = graph.semanticNodes(nullptr,
      std::numeric_limits<float>::infinity(), -1, &skipped);
    ASSERT_EQ(expired.size(), 1U);
    EXPECT_EQ(expired.front(), verified);
    EXPECT_EQ(skipped, 2U);
  }
  EXPECT_EQ(graph.semanticNodes().size(), 3U);
}

TEST(TopoGraphPersistence, KeepsAOneFrameGeometryMiss)
{
  EXPECT_TRUE(retainGeometryAfterMiss(1U));
  EXPECT_TRUE(retainGeometryAfterMiss(2U));
  EXPECT_FALSE(retainGeometryAfterMiss(3U));
}

TEST(TopoGraphPersistence, DetachedRebuildCarriesVerifiedNodesAndEdges)
{
  TopoGraph source;
  source.min_bd = Eigen::Vector3f::Zero();
  source.init_region_size_x_ = 10.0;
  source.init_region_size_y_ = 10.0;
  source.init_region_size_z_ = 3.0;
  auto from = std::make_shared<TopoNode>();
  from->center_ = Eigen::Vector3f(1.0F, 1.0F, 1.0F);
  from->persistent_id_ = 7;
  from->is_route_anchor_ = true;
  auto to = std::make_shared<TopoNode>();
  to->center_ = Eigen::Vector3f(5.0F, 1.0F, 1.0F);
  from->neighbors_.insert(to);
  to->neighbors_.insert(from);
  from->paths_[to] = {from->center_, to->center_};
  to->paths_[from] = {to->center_, from->center_};
  from->weight_[to] = 4.0F;
  to->weight_[from] = 4.0F;
  source.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(from);
  source.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(to);

  TopoGraph rebuilt;
  rebuilt.min_bd = source.min_bd;
  rebuilt.init_region_size_x_ = source.init_region_size_x_;
  rebuilt.init_region_size_y_ = source.init_region_size_y_;
  rebuilt.init_region_size_z_ = source.init_region_size_z_;
  rebuilt.copyPersistentNodesFrom(source);

  const auto region = rebuilt.getRegionNode(Eigen::Vector3i(0, 0, 0));
  ASSERT_EQ(region->topo_nodes_.size(), 2U);
  TopoNode::Ptr copied_from;
  TopoNode::Ptr copied_to;
  for (const auto &node : region->topo_nodes_) {
    if (node->persistent_id_ == 7) copied_from = node;
    else copied_to = node;
  }
  ASSERT_TRUE(copied_from);
  ASSERT_TRUE(copied_to);
  EXPECT_TRUE(copied_from->is_route_anchor_);
  EXPECT_EQ(rebuilt.routeAnchorCount(), 1U);
  EXPECT_EQ(copied_from->neighbors_.count(copied_to), 1U);
  EXPECT_FLOAT_EQ(copied_from->weight_.at(copied_to), 4.0F);
}

TEST(TopoGraphPersistence, DetachedRebuildCarriesSemanticNodes)
{
  TopoGraph source;
  source.min_bd = Eigen::Vector3f::Zero();
  source.init_region_size_x_ = 10.0;
  source.init_region_size_y_ = 10.0;
  source.init_region_size_z_ = 3.0;
  auto semantic = std::make_shared<TopoNode>();
  semantic->center_ = Eigen::Vector3f(5.0F, 1.0F, 1.0F);
  semantic->role_ = TopoNodeRole::Geometric;
  semantic->geometry_state_ = TopoGeometryState::Unknown;
  semantic->persistent_id_ = 19;
  semantic->semantic_score_ = 0.8F;
  semantic->semantic_observations_ = 1;
  source.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(semantic);

  TopoGraph rebuilt;
  rebuilt.min_bd = source.min_bd;
  rebuilt.init_region_size_x_ = source.init_region_size_x_;
  rebuilt.init_region_size_y_ = source.init_region_size_y_;
  rebuilt.init_region_size_z_ = source.init_region_size_z_;
  rebuilt.copyPersistentNodesFrom(source);

  const auto candidates = rebuilt.semanticNodes();
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front()->persistent_id_, 19U);
  EXPECT_EQ(candidates.front()->geometry_state_, TopoGeometryState::Unknown);
  EXPECT_NEAR(candidates.front()->semantic_score_, 0.8F, 1e-6F);
}

TEST(TopoGraphConnectivity, DuplicateVerticesAreMergedAndEdgesArePreserved)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f::Zero();
  graph.init_region_size_x_ = 10.0;
  graph.init_region_size_y_ = 10.0;
  graph.init_region_size_z_ = 3.0;

  auto duplicate_a = std::make_shared<TopoNode>();
  duplicate_a->center_ = Eigen::Vector3f(2.0F, 2.0F, 1.0F);
  duplicate_a->persistent_id_ = 10;
  auto duplicate_b = std::make_shared<TopoNode>();
  duplicate_b->center_ = Eigen::Vector3f(2.02F, 2.0F, 1.0F);
  duplicate_b->persistent_id_ = 11;
  auto left = std::make_shared<TopoNode>();
  left->center_ = Eigen::Vector3f(0.0F, 2.0F, 1.0F);
  auto right = std::make_shared<TopoNode>();
  right->center_ = Eigen::Vector3f(4.0F, 2.0F, 1.0F);

  duplicate_a->neighbors_.insert(left);
  left->neighbors_.insert(duplicate_a);
  duplicate_a->paths_[left] = {duplicate_a->center_, left->center_};
  left->paths_[duplicate_a] = {left->center_, duplicate_a->center_};
  duplicate_a->weight_[left] = 2.0F;
  left->weight_[duplicate_a] = 2.0F;
  duplicate_b->neighbors_.insert(right);
  right->neighbors_.insert(duplicate_b);
  duplicate_b->paths_[right] = {duplicate_b->center_, right->center_};
  right->paths_[duplicate_b] = {right->center_, duplicate_b->center_};
  duplicate_b->weight_[right] = 2.0F;
  right->weight_[duplicate_b] = 2.0F;

  const auto region = graph.getRegionNode(Eigen::Vector3i(0, 0, 0));
  region->topo_nodes_.insert(duplicate_a);
  region->topo_nodes_.insert(duplicate_b);
  region->topo_nodes_.insert(left);
  region->topo_nodes_.insert(right);

  EXPECT_EQ(graph.deduplicateNearbyNodes(0.05F), 1U);
  ASSERT_EQ(region->topo_nodes_.size(), 3U);
  TopoNode::Ptr canonical;
  for (const auto &node : region->topo_nodes_) {
    if (node->persistent_id_ == 10 || node->persistent_id_ == 11) canonical = node;
  }
  ASSERT_TRUE(canonical);
  EXPECT_EQ(canonical->neighbors_.size(), 2U);
  EXPECT_EQ(left->neighbors_.count(canonical), 1U);
  EXPECT_EQ(right->neighbors_.count(canonical), 1U);
  EXPECT_EQ(canonical->paths_.count(left), 1U);
  EXPECT_EQ(canonical->paths_.count(right), 1U);
  const auto stale = canonical == duplicate_a ? duplicate_b : duplicate_a;
  EXPECT_EQ(left->neighbors_.count(stale), 0U);
  EXPECT_EQ(right->neighbors_.count(stale), 0U);
}

TEST(TopoGraphConnectivity, NearbyValidVerticesAreNotCollapsedAsDuplicates)
{
  TopoGraph graph;
  auto region = graph.getRegionNode(Eigen::Vector3i(0, 0, 0));
  auto left = std::make_shared<TopoNode>();
  auto right = std::make_shared<TopoNode>();
  left->center_ = Eigen::Vector3f(1.0F, 1.0F, 1.0F);
  right->center_ = Eigen::Vector3f(1.20F, 1.0F, 1.0F);
  left->neighbors_.insert(right);
  right->neighbors_.insert(left);
  left->paths_[right] = {left->center_, right->center_};
  right->paths_[left] = {right->center_, left->center_};
  region->topo_nodes_.insert(left);
  region->topo_nodes_.insert(right);

  EXPECT_EQ(graph.deduplicateNearbyNodes(), 0U);
  EXPECT_EQ(region->topo_nodes_.size(), 2U);
  EXPECT_EQ(left->neighbors_.count(right), 1U);
  EXPECT_EQ(right->neighbors_.count(left), 1U);
}

TEST(TopoGraphConnectivity, SemanticAssociationRadiusDoesNotMergeGeometryDiff)
{
  TopoGraph graph;
  auto old_a = std::make_shared<TopoNode>();
  auto old_b = std::make_shared<TopoNode>();
  auto new_a = std::make_shared<TopoNode>();
  old_a->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  old_b->center_ = Eigen::Vector3f(2.0F, 0.0F, 1.0F);
  new_a->center_ = Eigen::Vector3f(0.9F, 0.0F, 1.0F);
  std::vector<TopoNode::Ptr> old_nodes{old_a, old_b};
  std::vector<TopoNode::Ptr> new_nodes{new_a};
  std::vector<TopoNode::Ptr> remained;
  std::vector<TopoNode::Ptr> removed;
  graph.overlap(new_nodes, old_nodes, remained);
  graph.setdiff(old_nodes, new_nodes, removed);

  // 0.9 m is outside geometric jitter but inside the 2.5 m semantic
  // association radius. Only the latter must remain unmatched and removed.
  ASSERT_EQ(remained.size(), 0U);
  ASSERT_EQ(removed.size(), 2U);
}

TEST(TopoGraphCorridorMemory, ProtectsOnlyVerifiedRouteGeometry)
{
  TopoGraph graph;
  auto verified = std::make_shared<TopoNode>();
  auto unknown = std::make_shared<TopoNode>();
  auto odom = std::make_shared<TopoNode>();
  verified->geometry_state_ = TopoGeometryState::Verified;
  unknown->geometry_state_ = TopoGeometryState::Unknown;
  odom->role_ = TopoNodeRole::Odom;

  EXPECT_EQ(graph.protectRouteGeometry({verified, unknown, odom}), 1U);
  EXPECT_TRUE(verified->is_route_anchor_);
  EXPECT_FALSE(unknown->is_route_anchor_);
  EXPECT_FALSE(odom->is_route_anchor_);
  EXPECT_EQ(graph.protectRouteGeometry({verified}), 0U);
}

TEST(TopoGraphCorridorMemory, DeduplicationKeepsRouteAnchor)
{
  TopoGraph graph;
  auto ordinary = std::make_shared<TopoNode>();
  auto anchor = std::make_shared<TopoNode>();
  ordinary->center_ = Eigen::Vector3f(1.0F, 1.0F, 1.0F);
  anchor->center_ = Eigen::Vector3f(1.02F, 1.0F, 1.0F);
  anchor->is_route_anchor_ = true;
  const auto region = graph.getRegionNode(Eigen::Vector3i::Zero());
  region->topo_nodes_.insert(ordinary);
  region->topo_nodes_.insert(anchor);

  EXPECT_EQ(graph.deduplicateNearbyNodes(0.05F), 1U);
  ASSERT_EQ(region->topo_nodes_.size(), 1U);
  EXPECT_EQ(*region->topo_nodes_.begin(), anchor);
  EXPECT_TRUE((*region->topo_nodes_.begin())->is_route_anchor_);
  EXPECT_EQ(graph.routeAnchorCount(), 1U);
}

TEST(TopoGraphConnectivity, HalfEdgesAreRemovedFromThePersistentGraph)
{
  TopoGraph graph;
  auto region = graph.getRegionNode(Eigen::Vector3i(0, 0, 0));
  auto left = std::make_shared<TopoNode>();
  auto right = std::make_shared<TopoNode>();
  left->center_ = Eigen::Vector3f(1.0F, 1.0F, 1.0F);
  right->center_ = Eigen::Vector3f(2.0F, 1.0F, 1.0F);
  left->neighbors_.insert(right);
  left->paths_[right] = {left->center_, right->center_};
  left->weight_[right] = 1.0F;
  region->topo_nodes_.insert(left);
  region->topo_nodes_.insert(right);

  EXPECT_EQ(graph.normalizeConnectivity(), 1U);
  EXPECT_TRUE(left->neighbors_.empty());
  EXPECT_TRUE(left->paths_.empty());
  EXPECT_TRUE(left->weight_.empty());
  EXPECT_TRUE(right->neighbors_.empty());
}

TEST(TopoSemanticCost, SemanticNodeUsesTheSameEndpointCostAsAnyTopoNode)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  auto from = std::make_shared<TopoNode>();
  from->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  auto to = std::make_shared<TopoNode>();
  to->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  from->neighbors_.insert(to);
  to->neighbors_.insert(from);
  from->paths_[to] = {from->center_, to->center_};
  to->paths_[from] = {to->center_, from->center_};
  region->topo_nodes_.insert(from);
  region->topo_nodes_.insert(to);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto semantic = std::make_shared<TopoNode>();
  semantic->center_ = Eigen::Vector3f(5.0F, 0.5F, 1.6F);
  semantic->role_ = TopoNodeRole::Geometric;
  semantic->geometry_state_ = TopoGeometryState::Unknown;
  semantic->semantic_score_ = 0.9F;
  semantic->semantic_confidence_ = 1.0F;
  semantic->semantic_observations_ = 1;
  region->topo_nodes_.insert(semantic);

  const float risk = graph.semanticRiskForEdge(from, to);
  auto far_from = std::make_shared<TopoNode>();
  auto far_to = std::make_shared<TopoNode>();
  far_from->center_ = Eigen::Vector3f(0.0F, 10.0F, 1.6F);
  far_to->center_ = Eigen::Vector3f(10.0F, 10.0F, 1.6F);
  const float far_risk = graph.semanticRiskForEdge(far_from, far_to);
  // A semantic node contributes a continuous field to nearby edge costs,
  // even when neither edge endpoint is the semantic node itself.
  EXPECT_GT(risk, far_risk + 0.2F);
  EXPECT_EQ(semantic->role_, TopoNodeRole::Geometric);
  EXPECT_EQ(semantic->geometry_state_, TopoGeometryState::Unknown);

  from->semantic_score_ = 0.9F;
  from->semantic_confidence_ = 1.0F;
  EXPECT_GE(graph.semanticRiskForEdge(from, to), risk);
}

TEST(TopoSemanticCost, RiskUsesTheExecutedWitnessPolyline)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto from = std::make_shared<TopoNode>();
  auto to = std::make_shared<TopoNode>();
  from->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  to->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  // The actual collision-free edge bends through y=5.  Its endpoint chord is
  // far from the semantic point, so a chord-only implementation misses it.
  from->paths_[to] = {from->center_, Eigen::Vector3f(5.0F, 5.0F, 1.6F), to->center_};
  auto risk = std::make_shared<TopoNode>();
  risk->center_ = Eigen::Vector3f(5.0F, 5.5F, 1.6F);
  risk->role_ = TopoNodeRole::Geometric;
  risk->geometry_state_ = TopoGeometryState::Unknown;
  risk->semantic_score_ = 1.0F;
  risk->semantic_confidence_ = 1.0F;
  risk->semantic_observations_ = 1;
  region->topo_nodes_.insert(from);
  region->topo_nodes_.insert(to);
  region->topo_nodes_.insert(risk);

  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_GT(graph.semanticRiskForEdge(from, to), 0.5F);
  }
}

TEST(TopoGraphM4Contract, TcM4002ResetClearsSearchCaches)
{
  ParallelBubbleAstar astar;
  astar.safe_node.insert(Eigen::Vector3i(1, 2, 3));
  astar.dangerous_node.insert(Eigen::Vector3i(4, 5, 6));

  for (int repetition = 0; repetition < 1000; ++repetition) {
    astar.reset();
    EXPECT_TRUE(astar.safe_node.empty());
    EXPECT_TRUE(astar.dangerous_node.empty());
    astar.safe_node.insert(Eigen::Vector3i(repetition, 0, 0));
    astar.dangerous_node.insert(Eigen::Vector3i(0, repetition, 0));
  }
  astar.reset();
}

TEST(TopoGraphM4Contract, TcM4003And004GridIndexRoundTrip)
{
  ParallelBubbleAstar astar;
  astar.origin_ = Eigen::Vector3f(-12.5F, -7.25F, -2.0F);
  astar.resolution_ = 0.2;
  astar.inv_resolution_ = 5.0;

  for (int repetition = 0; repetition < 10000; ++repetition) {
    const Eigen::Vector3f point(
      -20.0F + 0.017F * static_cast<float>(repetition),
      8.0F - 0.013F * static_cast<float>(repetition),
      -4.0F + 0.007F * static_cast<float>(repetition));
    Eigen::Vector3i index;
    astar.posToIndex(point, index);
    const Eigen::Vector3i expected =
      ((point - astar.origin_) * astar.inv_resolution_).array().floor().cast<int>();
    EXPECT_EQ(index, expected);

    Eigen::Vector3f center;
    astar.IndexToPos(center, index);
    EXPECT_TRUE((center - point).cwiseAbs().maxCoeff() <=
      static_cast<float>(astar.resolution_) * 0.5F + 1e-5F);
  }
}

TEST(TopoGraphM4Contract, TcM4009PathCostHandlesPolylineAndEmptyInput)
{
  ParallelBubbleAstar astar;
  const std::vector<Eigen::Vector3f> polyline = {
    Eigen::Vector3f(0.0F, 0.0F, 0.0F),
    Eigen::Vector3f(3.0F, 4.0F, 0.0F),
    Eigen::Vector3f(3.0F, 4.0F, 12.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    double cost = -1.0;
    astar.calculatePathCost(polyline, cost);
    EXPECT_DOUBLE_EQ(cost, 17.0);
  }

  EXPECT_EXIT(
    {
      ParallelBubbleAstar child_astar;
      double cost = -1.0;
      child_astar.calculatePathCost({}, cost);
      std::_Exit(cost == 0.0 ? 0 : 1);
    },
    ::testing::ExitedWithCode(0), "");
}

TEST(TopoGraphM4Contract, TcM4011ClearanceCostUsesContinuousFormula)
{
  TopoGraph graph;
  auto from = std::make_shared<TopoNode>();
  auto to = std::make_shared<TopoNode>();
  from->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  to->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  from->bubble_radius_ = 2000.0F;
  to->bubble_radius_ = 2000.0F;
  from->weight_[to] = 10.0F;

  for (int repetition = 0; repetition < 1000; ++repetition) {
    from->edge_clearance_[to] = 0.0F;
    const float at_zero = graph.clearanceCostForEdge(from, to);
    from->edge_clearance_[to] = 1.2F;
    const float at_target = graph.clearanceCostForEdge(from, to);
    from->edge_clearance_[to] = 12.0F;
    const float above_target = graph.clearanceCostForEdge(from, to);
    from->edge_clearance_[to] = 1200.0F;
    const float very_large = graph.clearanceCostForEdge(from, to);

    EXPECT_NEAR(at_zero, 20.0F, 1e-5F);
    EXPECT_NEAR(at_target, 5.0F, 1e-5F);
    EXPECT_GT(at_target, above_target);
    EXPECT_GT(above_target, very_large);
    EXPECT_GT(very_large, 0.0F);
  }
}

TEST(TopoGraphM4Contract, TcM4012RouteEdgeCostCombinesAndDiscountsTerms)
{
  TopoGraph graph;
  auto from = std::make_shared<TopoNode>();
  auto to = std::make_shared<TopoNode>();
  from->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  to->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  from->bubble_radius_ = 1.2F;
  to->bubble_radius_ = 1.2F;
  from->weight_[to] = 12.0F;
  from->edge_clearance_[to] = 1.2F;

  auto risk = std::make_shared<TopoNode>();
  risk->center_ = Eigen::Vector3f(5.0F, 0.2F, 1.6F);
  risk->semantic_score_ = 0.8F;
  risk->semantic_confidence_ = 1.0F;
  risk->semantic_observations_ = 1;
  const std::vector<TopoNode::Ptr> semantic_nodes = {risk};

  for (int repetition = 0; repetition < 1000; ++repetition) {
    const float base = graph.routeEdgeCost(from, to, 1.0F, 0.0F);
    const float discounted = graph.routeEdgeCost(from, to, 1.0F, 0.0F, true, 0.5F);
    const float semantic = graph.routeEdgeCost(
      from, to, 1.0F, 2.0F, false, 1.0F, &semantic_nodes);
    EXPECT_NEAR(base, 18.0F, 1e-5F);
    EXPECT_NEAR(discounted, 12.0F, 1e-5F);
    EXPECT_GT(semantic, base);
  }
}

TEST(TopoGraphM4Contract, TcM4015EmptyAndSingleNodePathsAreZero)
{
  EXPECT_EXIT(
    {
      TopoGraph graph;
      const std::vector<TopoNode::Ptr> empty;
      const double length = graph.getPathLength(empty);
      std::_Exit(length == 0.0 ? 0 : 1);
    },
    ::testing::ExitedWithCode(0), "");
  EXPECT_EXIT(
    {
      TopoGraph graph;
      const std::vector<TopoNode::Ptr> single = {std::make_shared<TopoNode>()};
      const double length = graph.getPathLength(single);
      std::_Exit(length == 0.0 ? 0 : 1);
    },
    ::testing::ExitedWithCode(0), "");
}

TEST(TopoGraphM4Contract, TcM4021RejectsConnectedNodeWithoutWitness)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto frontier_goal = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  frontier_goal->center_ = Eigen::Vector3f(12.0F, 0.0F, 1.6F);
  start->neighbors_.insert(frontier_goal);
  frontier_goal->neighbors_.insert(start);
  start->weight_[frontier_goal] = 12.0F;
  frontier_goal->weight_[start] = 12.0F;

  std::vector<TopoNode::Ptr> path;
  EXPECT_FALSE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2));
  EXPECT_TRUE(path.empty());
}

TEST(TopoGraphM4Contract, TcM4018EquivalentRiskDensityKeepsFrontierGoalRanking)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto start = std::make_shared<TopoNode>();
  auto risky = std::make_shared<TopoNode>();
  auto safe = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  risky->center_ = Eigen::Vector3f(20.0F, 0.0F, 1.6F);
  safe->center_ = Eigen::Vector3f(19.0F, 8.0F, 1.6F);
  for (const auto &node : {start, risky, safe}) {
    node->geometry_state_ = TopoGeometryState::Verified;
    region->topo_nodes_.insert(node);
  }
  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, risky);
  connect(start, safe);

  auto add_risk = [&region, &risky]() {
    auto risk = std::make_shared<TopoNode>();
    risk->center_ = risky->center_ + Eigen::Vector3f(0.0F, 0.2F, 0.0F);
    risk->geometry_state_ = TopoGeometryState::Unknown;
    risk->semantic_score_ = 0.8F;
    risk->semantic_confidence_ = 1.0F;
    risk->semantic_observations_ = 1;
    region->topo_nodes_.insert(risk);
  };
  add_risk();

  std::vector<TopoNode::Ptr> sparse_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), sparse_path,
    0.2, 1.0F, 1.0F, {}, 4.0F));
  ASSERT_EQ(sparse_path.size(), 2U);
  EXPECT_EQ(sparse_path.back(), risky);

  for (int index = 1; index < 235; ++index) add_risk();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    std::vector<TopoNode::Ptr> dense_path;
    ASSERT_TRUE(graph.goalDirectedSearch(
      start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), dense_path,
      0.2, 1.0F, 1.0F, {}, 4.0F));
    ASSERT_EQ(dense_path.size(), sparse_path.size());
    EXPECT_EQ(dense_path.back(), sparse_path.back());
  }
}

TEST(TopoGraphM4Contract, TcM4022OnlyVerifiedBubbleCanBeFrontierGoal)
{
  for (int repetition = 0; repetition < 1000; ++repetition) {
    TopoGraph graph;
    auto start = std::make_shared<TopoNode>();
    auto virtual_semantic = std::make_shared<TopoNode>();
    auto verified = std::make_shared<TopoNode>();
    start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
    virtual_semantic->center_ = Eigen::Vector3f(15.0F, 0.0F, -4.0F);
    virtual_semantic->geometry_state_ = TopoGeometryState::Unknown;
    virtual_semantic->semantic_score_ = 0.9F;
    virtual_semantic->semantic_confidence_ = 1.0F;
    virtual_semantic->semantic_observations_ = 1;
    verified->center_ = Eigen::Vector3f(12.0F, 4.0F, 1.6F);
    verified->geometry_state_ = TopoGeometryState::Verified;

    auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
      a->neighbors_.insert(b);
      b->neighbors_.insert(a);
      a->paths_[b] = {a->center_, b->center_};
      b->paths_[a] = {b->center_, a->center_};
      const float length = (a->center_ - b->center_).norm();
      a->weight_[b] = length;
      b->weight_[a] = length;
    };
    connect(start, virtual_semantic);
    connect(start, verified);

    std::vector<TopoNode::Ptr> path;
    ASSERT_TRUE(graph.goalDirectedSearch(
      start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2));
    ASSERT_EQ(path.size(), 2U);
    EXPECT_EQ(path.back(), verified);

    start->neighbors_.erase(verified);
    verified->neighbors_.erase(start);
    path.clear();
    EXPECT_FALSE(graph.goalDirectedSearch(
      start, virtual_semantic->center_, path, 0.2));
    EXPECT_TRUE(path.empty());
  }
}

TEST(TopoSemanticCost, UnknownSemanticNodeCannotBecomeFrontierFrontierGoal)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto semantic = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  semantic->center_ = Eigen::Vector3f(4.0F, 0.0F, 1.6F);
  semantic->role_ = TopoNodeRole::Geometric;
  semantic->geometry_state_ = TopoGeometryState::Unknown;
  semantic->semantic_score_ = 0.9F;
  semantic->semantic_confidence_ = 1.0F;
  semantic->semantic_observations_ = 1;
  start->neighbors_.insert(semantic);
  semantic->neighbors_.insert(start);
  start->paths_[semantic] = {start->center_, semantic->center_};
  semantic->paths_[start] = {semantic->center_, start->center_};
  start->weight_[semantic] = 4.0F;
  semantic->weight_[start] = 4.0F;

  std::vector<TopoNode::Ptr> path;
  EXPECT_FALSE(graph.goalDirectedSearch(
    start, semantic->center_, path, 0.2, 1.0, 0.0, {}, 1.0F));
  EXPECT_TRUE(path.empty());
}

TEST(TopoSemanticCost, VerifiedLowRiskBubbleRemainsAValidSafeBranch)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto safe = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  safe->center_ = Eigen::Vector3f(12.0F, 0.0F, 1.6F);
  safe->role_ = TopoNodeRole::Geometric;
  safe->geometry_state_ = TopoGeometryState::Verified;
  safe->semantic_score_ = 0.0F;
  safe->semantic_confidence_ = 1.0F;
  safe->semantic_observations_ = 1;
  start->neighbors_.insert(safe);
  safe->neighbors_.insert(start);
  start->paths_[safe] = {start->center_, safe->center_};
  safe->paths_[start] = {safe->center_, start->center_};
  start->weight_[safe] = 12.0F;
  safe->weight_[start] = 12.0F;

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, safe->center_, path, 0.2, 0.2F, 1.0F, {}, 2.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), safe);
}

TEST(TopoSemanticCost, AstarTurnsAwayFromNearbySemanticRisk)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;

  auto start = std::make_shared<TopoNode>();
  auto near_risk = std::make_shared<TopoNode>();
  auto far_side = std::make_shared<TopoNode>();
  auto goal = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  near_risk->center_ = Eigen::Vector3f(5.0F, 0.0F, 1.6F);
  far_side->center_ = Eigen::Vector3f(5.0F, 8.0F, 1.6F);
  goal->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);

  auto risk = std::make_shared<TopoNode>();
  risk->center_ = Eigen::Vector3f(5.0F, 0.5F, 1.6F);
  risk->role_ = TopoNodeRole::Geometric;
  risk->geometry_state_ = TopoGeometryState::Unknown;
  risk->semantic_score_ = 1.0F;
  risk->semantic_confidence_ = 1.0F;
  risk->semantic_observations_ = 1;

  for (const auto &node : {start, near_risk, far_side, goal, risk}) {
    region->topo_nodes_.insert(node);
  }
  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, near_risk);
  connect(near_risk, goal);
  connect(start, far_side);
  connect(far_side, goal);

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, path, 0.2, 0.2F, 1.0F, {}, 4.0F));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path[1], far_side);
  EXPECT_EQ(path.back(), goal);
}

TEST(TopoSemanticCost, OriginalModeKeepsGeometryRouteWhenSemanticScoresChange)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto start = std::make_shared<TopoNode>();
  auto lower = std::make_shared<TopoNode>();
  auto upper = std::make_shared<TopoNode>();
  auto goal = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  lower->center_ = Eigen::Vector3f(5.0F, 0.0F, 1.6F);
  upper->center_ = Eigen::Vector3f(5.0F, 6.0F, 1.6F);
  goal->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  for (const auto &node : {start, lower, upper, goal}) region->topo_nodes_.insert(node);
  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, lower);
  connect(lower, goal);
  connect(start, upper);
  connect(upper, goal);

  std::vector<TopoNode::Ptr> first_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, first_path, 0.2, 0.2F, 1.0F, {}, 20.0F));
  ASSERT_EQ(first_path.size(), 3U);
  EXPECT_EQ(first_path[1], lower);
  EXPECT_EQ(first_path.back(), goal);

  lower->semantic_score_ = 1.0F;
  upper->semantic_score_ = 0.0F;
  std::vector<TopoNode::Ptr> second_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, second_path, 0.2, 0.2F, 1.0F, {}, 20.0F));
  ASSERT_EQ(second_path.size(), 3U);
  EXPECT_EQ(second_path[1], lower);
  EXPECT_EQ(second_path.back(), goal);
}

TEST(TopoSearchRadius, GoalDirectedSearchStopsAtLocalForwardNode)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto near = std::make_shared<TopoNode>();
  auto far = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  near->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  far->center_ = Eigen::Vector3f(30.0F, 0.0F, 1.6F);

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, near);
  connect(near, far);

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 0.0F, 15.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), near);
}

TEST(TopoSearchRadius, DirectGoalSearchPicksMaxMissionProgressAtFork)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto corridor = std::make_shared<TopoNode>();
  auto bypass = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  corridor->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  bypass->center_ = Eigen::Vector3f(10.5F, -4.0F, 1.6F);

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, corridor);
  connect(start, bypass);

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 0.0F, 15.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), bypass);
}

TEST(TopoSearchRadius, DirectGoalSearchPicksMaxProgressOverShorterDetour)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto corridor = std::make_shared<TopoNode>();
  auto detour = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  corridor->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  detour->center_ = Eigen::Vector3f(12.0F, 8.0F, 1.6F);

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, corridor);
  connect(start, detour);

  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 0.0F, 35.0F, &vehicle, 10.0F, false));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), detour);
}

TEST(TopoSearchRadius, FrontierGoalUsesTheLocalGraphFrontier)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto local_goal_lookahead = std::make_shared<TopoNode>();
  auto frontier_goal = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  local_goal_lookahead->center_ = Eigen::Vector3f(10.0F, 0.0F, 1.6F);
  frontier_goal->center_ = Eigen::Vector3f(32.0F, 0.0F, 1.6F);

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, local_goal_lookahead);
  connect(local_goal_lookahead, frontier_goal);

  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 0.0F, 35.0F, &vehicle, 31.5F, false));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path[1], local_goal_lookahead);
  EXPECT_EQ(path.back(), frontier_goal);
}

TEST(TopoSearchRadius, GeometryWeightBalancesSemanticFrontierRisk)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto start = std::make_shared<TopoNode>();
  auto direct_mid = std::make_shared<TopoNode>();
  auto direct_frontier = std::make_shared<TopoNode>();
  auto safe_mid = std::make_shared<TopoNode>();
  auto safe_frontier = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  direct_mid->center_ = Eigen::Vector3f(15.0F, 0.0F, 1.6F);
  direct_frontier->center_ = Eigen::Vector3f(32.0F, 0.0F, 1.6F);
  safe_mid->center_ = Eigen::Vector3f(15.0F, 10.0F, 1.6F);
  safe_frontier->center_ = Eigen::Vector3f(31.5F, 10.0F, 1.6F);
  direct_mid->semantic_score_ = 0.95F;
  direct_mid->semantic_confidence_ = 1.0F;
  direct_mid->semantic_observations_ = 1;
  direct_frontier->semantic_score_ = 0.1F;
  direct_frontier->semantic_confidence_ = 1.0F;
  auto direct_risk = std::make_shared<TopoNode>();
  direct_risk->center_ = direct_mid->center_ + Eigen::Vector3f(0.0F, 0.2F, 0.0F);
  direct_risk->geometry_state_ = TopoGeometryState::Unknown;
  direct_risk->semantic_score_ = 0.95F;
  direct_risk->semantic_confidence_ = 1.0F;
  direct_risk->semantic_observations_ = 1;
  for (const auto &node : {start, direct_mid, direct_frontier, safe_mid, safe_frontier, direct_risk}) {
    node->geometry_state_ = TopoGeometryState::Verified;
    region->topo_nodes_.insert(node);
  }

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, direct_mid);
  connect(direct_mid, direct_frontier);
  connect(start, safe_mid);
  connect(safe_mid, safe_frontier);

  const Eigen::Vector3f mission_goal(100.0F, 0.0F, 1.6F);
  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> balanced_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, mission_goal, balanced_path, 0.2, 0.01F, 1.0F, {}, 10.0F,
    35.0F, &vehicle, 31.0F, false));
  ASSERT_EQ(balanced_path.size(), 3U);

  std::vector<TopoNode::Ptr> distance_dominated_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, mission_goal, distance_dominated_path, 0.2, 1.0F, 1.0F, {}, 0.0F,
    35.0F, &vehicle, 31.0F, false));
  ASSERT_EQ(distance_dominated_path.size(), 3U);
  EXPECT_EQ(distance_dominated_path[1], direct_mid);
  EXPECT_EQ(distance_dominated_path.back()->center_.x(), direct_frontier->center_.x());
}

TEST(TopoSearchRadius, DirectGoalSearchUsesSemanticCostWhenBranchesAreParallel)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto risky_frontier = std::make_shared<TopoNode>();
  auto safe_frontier = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  risky_frontier->center_ = Eigen::Vector3f(25.0F, 0.0F, 1.6F);
  safe_frontier->center_ = Eigen::Vector3f(24.0F, 5.0F, 1.6F);
  risky_frontier->semantic_score_ = 0.1F;
  risky_frontier->semantic_confidence_ = 1.0F;

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, risky_frontier);
  connect(start, safe_frontier);

  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 2.0F, 35.0F, &vehicle, 31.5F, false));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), risky_frontier);
}

TEST(TopoSearchRadius, DirectGoalSearchUsesSemanticCostDuringExpansion)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto start = std::make_shared<TopoNode>();
  auto safe = std::make_shared<TopoNode>();
  auto risky = std::make_shared<TopoNode>();
  auto goal = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  safe->center_ = Eigen::Vector3f(10.0F, 1.5F, 1.6F);
  risky->center_ = Eigen::Vector3f(12.0F, 0.0F, 1.6F);
  goal->center_ = Eigen::Vector3f(20.0F, 0.0F, 1.6F);
  risky->semantic_score_ = 0.9F;
  risky->semantic_confidence_ = 1.0F;
  for (const auto &node : {start, safe, risky, goal}) {
    node->geometry_state_ = TopoGeometryState::Verified;
    region->topo_nodes_.insert(node);
  }

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, safe);
  connect(safe, goal);
  connect(start, risky);
  connect(risky, goal);

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, path, 0.2,
    0.2F, 1.0F, {}, 2.0F, 25.0F));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path[1], safe);
  EXPECT_EQ(path.back(), goal);
}

TEST(TopoSearchRadius, DirectGoalSearchPrefersTowardGoalCorridorAtYFork)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto left = std::make_shared<TopoNode>();
  auto right = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  left->center_ = Eigen::Vector3f(-5.0F, 10.0F, 1.6F);
  right->center_ = Eigen::Vector3f(12.0F, 30.0F, 1.6F);

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b, float scale) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length * scale;
    b->weight_[a] = length * scale;
  };
  connect(start, left, 1.4F);
  connect(start, right, 1.0F);

  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(0.0F, 140.0F, 1.6F), path, 0.2,
    1.0F, 1.0F, {}, 0.0F, 35.0F, &vehicle, 0.0F, false,
    std::numeric_limits<float>::infinity(), 10.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), right);
  EXPECT_GT(path.back()->center_.x(), 5.0F);
  EXPECT_GE(path.back()->center_.y(), 20.0F);
}

TEST(TopoSearchRadius, GoalInWindowSelectsNearestGoalNode)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto intermediate = std::make_shared<TopoNode>();
  auto at_goal = std::make_shared<TopoNode>();
  auto overshoot = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  intermediate->center_ = Eigen::Vector3f(5.0F, 0.0F, 1.6F);
  at_goal->center_ = Eigen::Vector3f(10.2F, 0.4F, 1.6F);
  overshoot->center_ = Eigen::Vector3f(20.0F, -4.0F, 1.6F);

  auto connect = [](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    a->neighbors_.insert(b);
    b->neighbors_.insert(a);
    a->paths_[b] = {a->center_, b->center_};
    b->paths_[a] = {b->center_, a->center_};
    const float length = (a->center_ - b->center_).norm();
    a->weight_[b] = length;
    b->weight_[a] = length;
  };
  connect(start, intermediate);
  connect(intermediate, at_goal);
  connect(start, overshoot);

  const Eigen::Vector3f goal(10.0F, 0.0F, 1.6F);
  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal, path, 0.2, 0.2F, 1.0F, {}, 0.0F, 35.0F, &vehicle, 0.0F, true));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path[1], intermediate);
  EXPECT_EQ(path.back(), at_goal);
}

TEST(TopoSearchRadius, GraphSearchRejectsEndOutsideLocalWindow)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto far = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  far->center_ = Eigen::Vector3f(30.0F, 0.0F, 1.6F);
  start->neighbors_.insert(far);
  far->neighbors_.insert(start);
  start->paths_[far] = {start->center_, far->center_};
  far->paths_[start] = {far->center_, start->center_};
  start->weight_[far] = 30.0F;
  far->weight_[start] = 30.0F;

  std::vector<TopoNode::Ptr> path;
  EXPECT_FALSE(graph.graphSearch(
    start, far, path, 0.2, false, {}, 0.0F, 15.0F));
  EXPECT_TRUE(path.empty());
}

TEST(TopoGraphConnectivity, IncludesDiagonalRegionNeighbors)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f::Zero();
  graph.init_region_size_x_ = 3.3;
  graph.init_region_size_y_ = 3.3;
  graph.init_region_size_z_ = 2.0;

  auto from = std::make_shared<TopoNode>();
  auto diagonal = std::make_shared<TopoNode>();
  from->center_ = Eigen::Vector3f(1.0F, 1.0F, 1.6F);
  diagonal->center_ = Eigen::Vector3f(4.0F, 4.0F, 1.6F);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] =
    std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(1, 1, 0)] =
    std::make_shared<RegionNode>(Eigen::Vector3i(1, 1, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)]->topo_nodes_.insert(from);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(1, 1, 0)]->topo_nodes_.insert(diagonal);

  std::vector<TopoNode::Ptr> neighbors;
  graph.getPreNbrs(from, neighbors);
  EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), diagonal), neighbors.end());
}

TEST(TopoSemanticMemory, RestoresSemanticsAfterANodeIsRecreated)
{
  TopoGraph original;
  original.min_bd = Eigen::Vector3f(-10.0F, -10.0F, -2.0F);
  original.init_region_size_x_ = 3.3;
  original.init_region_size_y_ = 3.3;
  original.init_region_size_z_ = 2.0;

  auto observed = std::make_shared<TopoNode>();
  observed->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  original.updateNodeSemantic(observed, 0.8F, 0.25F, 100);
  original.updateNodeSemantic(observed, 0.2F, 0.25F, 200);
  ASSERT_NE(observed->persistent_id_, 0U);
  EXPECT_NEAR(observed->semantic_score_, 0.65F, 1e-6F);
  EXPECT_EQ(observed->semantic_observations_, 2U);

  TopoGraph rebuilt;
  // A new goal changes EPIC's map bounds and local region origin. Semantic
  // identity remains anchored in world coordinates.
  rebuilt.min_bd = Eigen::Vector3f(-100.0F, -50.0F, -2.0F);
  rebuilt.init_region_size_x_ = original.init_region_size_x_;
  rebuilt.init_region_size_y_ = original.init_region_size_y_;
  rebuilt.init_region_size_z_ = original.init_region_size_z_;
  rebuilt.loadSemanticMemory(original.semanticMemorySnapshot());

  auto replacement = std::make_shared<TopoNode>();
  replacement->center_ = Eigen::Vector3f(2.0F, 0.1F, 1.6F);
  std::vector<TopoNode::Ptr> inserted{replacement};
  EXPECT_EQ(rebuilt.restoreNodeSemanticMemory(inserted), 1U);
  EXPECT_EQ(replacement->persistent_id_, observed->persistent_id_);
  EXPECT_NEAR(replacement->semantic_score_, observed->semantic_score_, 1e-6F);
  EXPECT_NEAR(replacement->semantic_confidence_, observed->semantic_confidence_, 1e-6F);
  EXPECT_EQ(replacement->semantic_observations_, observed->semantic_observations_);
  EXPECT_EQ(replacement->semantic_stamp_ns_, observed->semantic_stamp_ns_);
}

TEST(TopoSemanticMemory, CurrentEvidenceCanDecayAnOldHighScore)
{
  TopoGraph graph;
  graph.init_region_size_x_ = 3.3;
  graph.init_region_size_y_ = 3.3;
  graph.init_region_size_z_ = 2.0;
  auto node = std::make_shared<TopoNode>();
  graph.updateNodeSemantic(node, 1.0F, 0.5F, 100);
  graph.updateNodeSemantic(node, 0.0F, 0.5F, 200);
  EXPECT_NEAR(node->semantic_score_, 0.5F, 1e-6F);
}

TEST(TopoSemanticMemory, DoesNotAssignDistantSemanticsToANewNode)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f(-10.0F, -10.0F, -2.0F);
  graph.init_region_size_x_ = 3.3;
  graph.init_region_size_y_ = 3.3;
  graph.init_region_size_z_ = 2.0;

  auto observed = std::make_shared<TopoNode>();
  observed->center_ = Eigen::Vector3f::Zero();
  graph.updateNodeSemantic(observed, 0.9F, 0.3F, 100);

  auto distant = std::make_shared<TopoNode>();
  distant->center_ = Eigen::Vector3f(4.0F, 0.0F, 0.0F);
  std::vector<TopoNode::Ptr> inserted{distant};
  EXPECT_EQ(graph.restoreNodeSemanticMemory(inserted), 0U);
  EXPECT_NE(distant->persistent_id_, 0U);
  EXPECT_NE(distant->persistent_id_, observed->persistent_id_);
  EXPECT_EQ(distant->semantic_observations_, 0U);
}

TEST(TopoSemanticMemory, DoesNotDuplicateAnActiveNodeIdentity)
{
  TopoGraph graph;
  graph.min_bd = Eigen::Vector3f(-10.0F, -10.0F, -2.0F);
  graph.init_region_size_x_ = 3.3;
  graph.init_region_size_y_ = 3.3;
  graph.init_region_size_z_ = 2.0;

  auto active = std::make_shared<TopoNode>();
  active->center_ = Eigen::Vector3f::Zero();
  graph.updateNodeSemantic(active, 0.9F, 0.3F, 100);

  auto nearby_new = std::make_shared<TopoNode>();
  nearby_new->center_ = Eigen::Vector3f(0.2F, 0.0F, 0.0F);
  std::vector<TopoNode::Ptr> inserted{nearby_new};
  const std::unordered_set<std::uint64_t> active_ids{active->persistent_id_};
  EXPECT_EQ(graph.restoreNodeSemanticMemory(inserted, active_ids), 0U);
  EXPECT_NE(nearby_new->persistent_id_, active->persistent_id_);
  EXPECT_EQ(nearby_new->semantic_observations_, 0U);
}

}  // namespace
