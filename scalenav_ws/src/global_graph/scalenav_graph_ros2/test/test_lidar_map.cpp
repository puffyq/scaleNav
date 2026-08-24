#include <gtest/gtest.h>

#include "lidar_map/lidar_map.h"

namespace {

using fast_planner::LIOInterface;
using fast_planner::PointType;

pcl::PointCloud<PointType> cloud(std::initializer_list<PointType> points)
{
  pcl::PointCloud<PointType> result;
  result.reserve(points.size());
  for (const auto &point : points) result.push_back(point);
  return result;
}

std::shared_ptr<LIOInterface> snapshotOf(
    const LIOInterface &source,
    const pcl::PointCloud<PointType> &latest,
    const Eigen::Vector3f &pose)
{
  auto snapshot = std::make_shared<LIOInterface>();
  snapshot->configureStorage(0.25F, 100.0F, 1000, 100.0F);
  snapshot->loadSnapshot(
    source.accumulatedCloudSnapshot(), latest, pose, Eigen::Quaternionf::Identity());
  return snapshot;
}

TEST(LidarMapOccupied, AccumulatesDistinctOccupiedVoxels)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const auto first = cloud({PointType(1.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    first, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  const auto second = cloud({PointType(3.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    second, Eigen::Vector3f(1.0F, 0.0F, 0.0F), Eigen::Quaternionf::Identity()));

  const auto accumulated = map.accumulatedCloudSnapshot();
  ASSERT_EQ(accumulated.size(), 2U);
  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(1.0F, 0.0F, 0.0F)), 0.0, 1e-6);
  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(3.0F, 0.0F, 0.0F)), 0.0, 1e-6);
}

TEST(LidarMapOccupied, PreservesEveryCurrentFrameEndpoint)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const Eigen::Vector3f pose = Eigen::Vector3f::Zero();
  const auto frame = cloud({
    PointType(1.0F, 0.0F, 0.0F),
    PointType(2.0F, 0.0F, 0.0F),
  });

  ASSERT_TRUE(map.updateCloudWorld(frame, pose, Eigen::Quaternionf::Identity()));
  const auto snapshot = snapshotOf(map, frame, pose);
  EXPECT_NEAR(snapshot->getDisToOcc(Eigen::Vector3f(1.0F, 0.0F, 0.0F)), 0.0, 1e-6);
  EXPECT_NEAR(snapshot->getDisToOcc(Eigen::Vector3f(2.0F, 0.0F, 0.0F)), 0.0, 1e-6);
}

TEST(LidarMapOccupied, KeepsThePointClosestToTheVoxelCenter)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  ASSERT_TRUE(map.updateCloudWorld(
    cloud({PointType(0.24F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  ASSERT_TRUE(map.updateCloudWorld(
    cloud({PointType(0.13F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  const auto accumulated = map.accumulatedCloudSnapshot();
  ASSERT_EQ(accumulated.size(), 1U);
  EXPECT_NEAR(accumulated.front().x, 0.13F, 1e-5F);
}

TEST(LidarMapOccupied, RepeatedFrameIsAHotPathNoOp)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const auto frame = cloud({PointType(2.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    frame, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  EXPECT_FALSE(map.updateCloudWorld(
    frame, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  EXPECT_EQ(map.accumulatedCloudSnapshot().size(), 1U);
}

TEST(LidarMapOccupied, FreeRaysAreIgnored)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const auto hit = cloud({PointType(2.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    hit, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  EXPECT_FALSE(map.updateFreeRaysWorld(
    cloud({PointType(4.0F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  ASSERT_EQ(map.accumulatedCloudSnapshot().size(), 1U);
  EXPECT_NEAR(map.accumulatedCloudSnapshot().front().x, 2.0F, 1e-6F);
  EXPECT_TRUE(map.freeSpaceSnapshot().empty());
}

TEST(LidarMapOccupied, PlanarClearanceIgnoresGroundBelowLayer)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  pcl::PointCloud<PointType> frame;
  for (int i = -4; i <= 4; ++i) {
    frame.push_back(PointType(static_cast<float>(i) * 0.5F, 0.0F, 0.0F));
  }
  frame.push_back(PointType(8.0F, 0.0F, 1.6F));
  ASSERT_TRUE(map.updateCloudWorld(
    frame, Eigen::Vector3f(0.0F, 0.0F, 1.6F), Eigen::Quaternionf::Identity()));

  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(0.0F, 0.0F, 1.6F)), 1.6, 0.05);

  map.setGraphObstacleMinZ(0.6F);
  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(0.0F, 0.0F, 1.6F)), 8.0, 0.05);
}

TEST(LidarMapBounds, ExpansionPreservesMapMemory)
{
  LIOInterface map;
  map.configureBounds(
    Eigen::Vector3f(-20.0F, -20.0F, -5.0F),
    Eigen::Vector3f(100.0F, 20.0F, 5.0F));
  map.configureStorage(0.25F, 200.0F, 1000U, 0.5F);
  const auto occupied = cloud({PointType(10.0F, 0.0F, 1.6F)});
  map.loadSnapshot(
    occupied, occupied, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());

  EXPECT_FALSE(map.IsInBox(Eigen::Vector3f(-80.0F, 0.0F, 1.6F)));
  EXPECT_TRUE(map.expandBounds(
    Eigen::Vector3f(-100.0F, -20.0F, -5.0F),
    Eigen::Vector3f(100.0F, 20.0F, 5.0F)));
  EXPECT_TRUE(map.IsInBox(Eigen::Vector3f(-80.0F, 0.0F, 1.6F)));
  ASSERT_EQ(map.accumulatedCloudSnapshot().size(), 1U);
  EXPECT_NEAR(map.accumulatedCloudSnapshot().front().x, 10.0F, 1e-6F);
  EXPECT_FALSE(map.expandBounds(
    Eigen::Vector3f(-50.0F, -10.0F, -2.0F),
    Eigen::Vector3f(50.0F, 10.0F, 2.0F)));
}

}  // namespace
