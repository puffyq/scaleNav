#include <gtest/gtest.h>

#include "pointcloud_topo/graph.h"

namespace {

TEST(TopoNodeModel, UsesOneNodeStructureForSpeculativePromotion)
{
  auto node = std::make_shared<TopoNode>();
  EXPECT_EQ(node->role_, TopoNodeRole::Geometric);
  EXPECT_EQ(node->geometry_state_, TopoGeometryState::Verified);

  node->role_ = TopoNodeRole::Speculative;
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

TEST(TopoNodeModel, SpeculativeSnapshotUsesTheUnifiedTopoNodeRole)
{
  TopoGraph graph;
  auto region = std::make_shared<RegionNode>(Eigen::Vector3i(0, 0, 0));
  auto speculative = std::make_shared<TopoNode>();
  speculative->role_ = TopoNodeRole::Speculative;
  speculative->geometry_state_ = TopoGeometryState::Unknown;
  auto geometric = std::make_shared<TopoNode>();
  geometric->role_ = TopoNodeRole::Geometric;
  region->topo_nodes_.insert(speculative);
  region->topo_nodes_.insert(geometric);
  graph.reg_map_idx2ptr_[Eigen::Vector3i(0, 0, 0)] = region;

  const auto candidates = graph.speculativeNodes();
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), speculative);
  EXPECT_EQ(candidates.front()->geometry_state_, TopoGeometryState::Unknown);
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

TEST(TopoGraphPersistence, DetachedRebuildCarriesSpeculativeNodes)
{
  TopoGraph source;
  source.min_bd = Eigen::Vector3f::Zero();
  source.init_region_size_x_ = 10.0;
  source.init_region_size_y_ = 10.0;
  source.init_region_size_z_ = 3.0;
  auto speculative = std::make_shared<TopoNode>();
  speculative->center_ = Eigen::Vector3f(5.0F, 1.0F, 1.0F);
  speculative->role_ = TopoNodeRole::Speculative;
  speculative->geometry_state_ = TopoGeometryState::Unknown;
  speculative->persistent_id_ = 19;
  speculative->semantic_score_ = 0.8F;
  source.getRegionNode(Eigen::Vector3i(0, 0, 0))->topo_nodes_.insert(speculative);

  TopoGraph rebuilt;
  rebuilt.min_bd = source.min_bd;
  rebuilt.init_region_size_x_ = source.init_region_size_x_;
  rebuilt.init_region_size_y_ = source.init_region_size_y_;
  rebuilt.init_region_size_z_ = source.init_region_size_z_;
  rebuilt.copyPersistentNodesFrom(source);

  const auto candidates = rebuilt.speculativeNodes();
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front()->persistent_id_, 19U);
  EXPECT_EQ(candidates.front()->geometry_state_, TopoGeometryState::Unknown);
  EXPECT_NEAR(candidates.front()->semantic_score_, 0.8F, 1e-6F);
}

TEST(TopoSemanticCost, SpeculativeNodeUsesTheSameEndpointCostAsAnyTopoNode)
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
  auto speculative = std::make_shared<TopoNode>();
  speculative->center_ = Eigen::Vector3f(5.0F, 0.5F, 1.6F);
  speculative->role_ = TopoNodeRole::Speculative;
  speculative->geometry_state_ = TopoGeometryState::Unknown;
  speculative->semantic_score_ = 0.9F;
  speculative->semantic_confidence_ = 1.0F;
  region->topo_nodes_.insert(speculative);

  const float risk = graph.semanticRiskForEdge(from, to);
  auto far_from = std::make_shared<TopoNode>();
  auto far_to = std::make_shared<TopoNode>();
  far_from->center_ = Eigen::Vector3f(0.0F, 10.0F, 1.6F);
  far_to->center_ = Eigen::Vector3f(10.0F, 10.0F, 1.6F);
  const float far_risk = graph.semanticRiskForEdge(far_from, far_to);
  // A speculative node contributes a continuous field to nearby edge costs,
  // even when neither edge endpoint is the speculative node itself.
  EXPECT_GT(risk, far_risk + 0.2F);
  EXPECT_EQ(speculative->role_, TopoNodeRole::Speculative);
  EXPECT_EQ(speculative->geometry_state_, TopoGeometryState::Unknown);

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
  // far from the speculative point, so a chord-only implementation misses it.
  from->paths_[to] = {from->center_, Eigen::Vector3f(5.0F, 5.0F, 1.6F), to->center_};
  auto risk = std::make_shared<TopoNode>();
  risk->center_ = Eigen::Vector3f(5.0F, 5.5F, 1.6F);
  risk->role_ = TopoNodeRole::Speculative;
  risk->geometry_state_ = TopoGeometryState::Unknown;
  risk->semantic_score_ = 1.0F;
  risk->semantic_confidence_ = 1.0F;
  region->topo_nodes_.insert(from);
  region->topo_nodes_.insert(to);
  region->topo_nodes_.insert(risk);

  EXPECT_GT(graph.semanticRiskForEdge(from, to), 0.5F);
}

TEST(TopoSemanticCost, SpeculativeNodeRemainsAValidAstarCandidate)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto speculative = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  speculative->center_ = Eigen::Vector3f(4.0F, 0.0F, 1.6F);
  speculative->role_ = TopoNodeRole::Speculative;
  speculative->geometry_state_ = TopoGeometryState::Unknown;
  speculative->semantic_score_ = 0.9F;
  speculative->semantic_confidence_ = 1.0F;
  start->neighbors_.insert(speculative);
  speculative->neighbors_.insert(start);
  start->paths_[speculative] = {start->center_, speculative->center_};
  speculative->paths_[start] = {speculative->center_, start->center_};
  start->weight_[speculative] = 4.0F;
  speculative->weight_[start] = 4.0F;

  std::vector<TopoNode::Ptr> path;
  ASSERT_TRUE(graph.goalDirectedSearch(
    start, speculative->center_, path, 0.2, 1.0, 0.0, {}, 1.0F));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), speculative);
}

TEST(TopoSemanticCost, LowScoreSpeculativeNodeRemainsAValidSafeBranch)
{
  TopoGraph graph;
  auto start = std::make_shared<TopoNode>();
  auto safe = std::make_shared<TopoNode>();
  start->center_ = Eigen::Vector3f(0.0F, 0.0F, 1.6F);
  safe->center_ = Eigen::Vector3f(12.0F, 0.0F, 1.6F);
  safe->role_ = TopoNodeRole::Speculative;
  safe->geometry_state_ = TopoGeometryState::Unknown;
  safe->semantic_score_ = 0.0F;
  safe->semantic_confidence_ = 1.0F;
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

TEST(TopoSemanticCost, AstarTurnsAwayFromNearbySpeculativeRisk)
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
  risk->role_ = TopoNodeRole::Speculative;
  risk->geometry_state_ = TopoGeometryState::Unknown;
  risk->semantic_score_ = 1.0F;
  risk->semantic_confidence_ = 1.0F;

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
