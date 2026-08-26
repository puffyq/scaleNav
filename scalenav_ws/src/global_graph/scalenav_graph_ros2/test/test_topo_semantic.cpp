#include <gtest/gtest.h>

#include <limits>
#include <thread>

#include "pointcloud_topo/graph.h"

namespace {

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

  EXPECT_GT(graph.semanticRiskForEdge(from, to), 0.5F);
}

TEST(TopoSemanticCost, SemanticNodeRemainsAValidAstarCandidate)
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
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, semantic->center_, path, 0.2, 1.0, 0.0, {}, 1.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), semantic);
}

TEST(TopoSemanticCost, LowScoreSemanticNodeRemainsAValidSafeBranch)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto safe = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  safe->center_ = Eigen::Vector3f(12.0F, 0.0F, 1.6F);
  safe->role_ = TopoNodeRole::Geometric;
  safe->geometry_state_ = TopoGeometryState::Unknown;
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
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), far_side);
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
  ASSERT_EQ(first_path.size(), 2U);
  EXPECT_EQ(first_path.back(), lower);

  lower->semantic_score_ = 1.0F;
  upper->semantic_score_ = 0.0F;
  std::vector<TopoNode::Ptr> second_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, goal->center_, second_path, 0.2, 0.2F, 1.0F, {}, 20.0F));
  ASSERT_EQ(second_path.size(), 2U);
  EXPECT_EQ(second_path.back(), lower);
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

TEST(TopoSearchRadius, GoalDirectedSearchPrefersCorridorTerminal)
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
  EXPECT_EQ(path.back(), corridor);
}

TEST(TopoSearchRadius, AstarDoesNotPreferAFartherDetourFrontier)
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
  EXPECT_EQ(path.back(), corridor);
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

TEST(TopoSearchRadius, RadialFrontierAllowsWideSemanticDetour)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;
  auto start = std::make_shared<TopoNode>();
  auto risky_forward = std::make_shared<TopoNode>();
  auto safe_detour = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  risky_forward->center_ = Eigen::Vector3f(32.0F, 0.0F, 1.6F);
  safe_detour->center_ = Eigen::Vector3f(24.0F, 18.0F, 1.6F);
  risky_forward->semantic_score_ = 0.9F;
  risky_forward->semantic_confidence_ = 1.0F;
  risky_forward->semantic_observations_ = 1;
  safe_detour->semantic_score_ = 0.0F;
  safe_detour->semantic_confidence_ = 1.0F;
  safe_detour->semantic_observations_ = 1;
  for (const auto &node : {start, risky_forward, safe_detour}) {
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
  connect(start, risky_forward);
  connect(start, safe_detour);

  const Eigen::Vector3f vehicle = start->center_;
  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 4.0F, 35.0F, &vehicle, 31.5F, false, 30.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), safe_detour);
  EXPECT_NEAR((safe_detour->center_ - vehicle).norm(), 30.0F, 1e-5F);
  EXPECT_LT(safe_detour->center_.x(), 31.5F);
}

TEST(TopoSearchRadius, GeometryWeightBalancesSemanticFrontierRisk)
{
  TopoGraph graph;
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
  direct_frontier->semantic_score_ = 0.1F;
  direct_frontier->semantic_confidence_ = 1.0F;

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
    start, mission_goal, balanced_path, 0.2, 0.2F, 1.0F, {}, 2.0F,
    35.0F, &vehicle, 31.0F, false));
  ASSERT_EQ(balanced_path.size(), 3U);
  EXPECT_EQ(balanced_path.back(), safe_frontier);

  std::vector<TopoNode::Ptr> distance_dominated_path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, mission_goal, distance_dominated_path, 0.2, 1.0F, 1.0F, {}, 2.0F,
    35.0F, &vehicle, 31.0F, false));
  ASSERT_EQ(distance_dominated_path.size(), 3U);
  EXPECT_EQ(distance_dominated_path.back(), direct_frontier);
}

TEST(TopoSearchRadius, IncompleteFrontierFallbackStillUsesCombinedLoss)
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
  EXPECT_EQ(path.back(), safe_frontier);
}

TEST(TopoSearchRadius, GoalDirectedSearchPrefersSaferTerminalOverFartherRisk)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto safe = std::make_shared<TopoNode>();
  auto risky = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  safe->center_ = Eigen::Vector3f(10.0F, 1.5F, 1.6F);
  risky->center_ = Eigen::Vector3f(12.0F, 0.0F, 1.6F);
  risky->semantic_score_ = 0.9F;
  risky->semantic_confidence_ = 1.0F;

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
  connect(start, risky);

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, Eigen::Vector3f(100.0F, 0.0F, 1.6F), path, 0.2,
    0.2F, 1.0F, {}, 2.0F, 20.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.back(), safe);
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
