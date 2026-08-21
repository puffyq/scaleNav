#include <gtest/gtest.h>

#include "pointcloud_topo/graph.h"

namespace {

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
  rebuilt.min_bd = original.min_bd;
  rebuilt.init_region_size_x_ = original.init_region_size_x_;
  rebuilt.init_region_size_y_ = original.init_region_size_y_;
  rebuilt.init_region_size_z_ = original.init_region_size_z_;
  rebuilt.loadSemanticMemory(original.semanticMemorySnapshot());

  auto replacement = std::make_shared<TopoNode>();
  replacement->center_ = Eigen::Vector3f(0.4F, 0.1F, 1.6F);
  std::vector<TopoNode::Ptr> inserted{replacement};
  EXPECT_EQ(rebuilt.restoreNodeSemanticMemory(inserted), 1U);
  EXPECT_EQ(replacement->persistent_id_, observed->persistent_id_);
  EXPECT_NEAR(replacement->semantic_score_, observed->semantic_score_, 1e-6F);
  EXPECT_NEAR(replacement->semantic_confidence_, observed->semantic_confidence_, 1e-6F);
  EXPECT_EQ(replacement->semantic_observations_, observed->semantic_observations_);
  EXPECT_EQ(replacement->semantic_stamp_ns_, observed->semantic_stamp_ns_);
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
