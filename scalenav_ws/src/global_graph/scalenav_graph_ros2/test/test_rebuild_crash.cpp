#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>

#include <pcl/io/pcd_io.h>

#include "lidar_map/lidar_map.h"
#include "pointcloud_topo/graph.h"

namespace {

TEST(RebuildCrash, FirstSkeletonFromLoggedCloud)
{
  const char *pcd_path =
    "/mnt/code/lab/yopo/OpenSeek/log_scalenav/session_20260824_160915_978/"
    "pointcloud/pointcloud_29.pcd";
  if (!std::filesystem::exists(pcd_path)) {
    GTEST_SKIP() << "recorded crash fixture was rotated: " << pcd_path;
  }

  pcl::PointCloud<fast_planner::PointType> body;
  ASSERT_GT(pcl::io::loadPCDFile(pcd_path, body), -1) << pcd_path;
  ASSERT_FALSE(body.empty());

  pcl::PointCloud<fast_planner::PointType> world;
  world.reserve(body.size());
  for (const auto &p : body.points) {
    world.push_back(fast_planner::PointType(p.x + 0.5F, p.y, p.z + 1.5F));
  }

  auto map = std::make_shared<fast_planner::LIOInterface>();
  const Eigen::Vector3f pose(0.0F, 0.0F, 1.6F);
  const Eigen::Vector3f goal(80.0F, 0.0F, 1.6F);
  map->configureBounds(
    pose.cwiseMin(goal) - Eigen::Vector3f::Constant(50.0F),
    pose.cwiseMax(goal) + Eigen::Vector3f::Constant(50.0F));
  map->configureStorage(0.1F, 50.0F, 100000, 0.5F);
  map->setGraphObstacleMinZ(0.6F);
  map->loadSnapshot(world, world, pose, Eigen::Quaternionf::Identity());
  ASSERT_GT(map->pointCount(), 0U);

  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::InitOptions options;
    options.auto_initialize_logging(false);
    rclcpp::init(argc, nullptr, options);
  }
  auto node = std::make_shared<rclcpp::Node>("rebuild_crash_test");
  node->declare_parameter<double>("bubble_topo/min_x", 1.11);
  node->declare_parameter<double>("bubble_topo/min_y", 1.11);
  node->declare_parameter<double>("bubble_topo/min_z", 0.51);
  node->declare_parameter<double>("bubble_topo/init_region_size_x", 3.3);
  node->declare_parameter<double>("bubble_topo/init_region_size_y", 3.3);
  node->declare_parameter<double>("bubble_topo/init_region_size_z", 2.0);
  node->declare_parameter<double>("bubble_topo/bubble_min_radius", 0.65);
  node->declare_parameter<double>("bubble_topo/frontier_bubble_min_radius", 0.65);
  node->declare_parameter<double>("bubble_topo/cube_discrete_size", 0.40);
  node->declare_parameter<bool>("bubble_topo/planar_graph", true);
  node->declare_parameter<double>("bubble_topo/planar_z", 1.6);
  node->declare_parameter<int>("max_update_region_num", 0);
  node->declare_parameter<double>("parallel_astar/update_connection_timeout", 0.003);
  node->declare_parameter<double>("parallel_astar/insert_node_timeout", 0.02);
  node->declare_parameter<double>("bubble_astar/resolution_astar", 0.30);
  node->declare_parameter<double>("bubble_astar/lambda_heu", 1.0);
  node->declare_parameter<double>("bubble_astar/safe_distance", 0.61);
  node->declare_parameter<bool>("bubble_astar/planar_search", true);
  node->declare_parameter<double>("bubble_astar/planar_z", 1.6);
  node->declare_parameter<int>("bubble_astar/allocate_num", 100000);
  node->declare_parameter<bool>("bubble_astar/debug", false);

  ros::NodeHandle nh(node);
  auto astar = std::make_shared<ParallelBubbleAstar>();
  auto topo = std::make_shared<TopoGraph>();
  astar->init(nh, map);
  topo->init(nh, map, astar);
  astar->planar_search_ = true;
  astar->planar_z_ = 1.6F;
  topo->planar_graph_ = true;
  topo->planar_z_ = 1.6F;
  topo->setUpdateGoal(goal);
  topo->getRegionsToUpdate();
  std::cerr << "regions=" << topo->toponodes_update_region_arr_.size()
            << " map_points=" << map->pointCount()
            << " about to updateSkeleton\n";
  topo->updateSkeleton();
  const auto timing = topo->getLastUpdateTiming();
  std::cerr << "done bubbles=" << timing.bubbles
            << " nodes=" << timing.new_nodes
            << " edge_candidates=" << timing.insert_candidate_edges
            << " edge_success=" << timing.insert_success_edges
            << "\n";
  EXPECT_GT(timing.bubbles + timing.new_nodes + timing.insert_candidate_edges, 0U);
}

}  // namespace
