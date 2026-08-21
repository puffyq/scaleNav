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

TEST(LidarMapRayCarving, RemovesAnOldHitObservedFreeByANewDepthRay)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const Eigen::Vector3f first_pose = Eigen::Vector3f::Zero();
  const auto old_frame = cloud({PointType(1.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    old_frame, first_pose, Eigen::Quaternionf::Identity()));

  const Eigen::Vector3f second_pose(1.0F, 0.0F, 0.0F);
  const auto new_frame = cloud({PointType(3.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    new_frame, second_pose, Eigen::Quaternionf::Identity()));

  const auto accumulated = map.accumulatedCloudSnapshot();
  ASSERT_EQ(accumulated.size(), 1U);
  EXPECT_NEAR(accumulated.front().x, 3.0F, 1e-6F);

  std::size_t hit_voxels = 0;
  std::size_t free_voxels = 0;
  std::size_t carved_voxels = 0;
  map.lastRayCarvingStats(hit_voxels, free_voxels, carved_voxels);
  EXPECT_EQ(hit_voxels, 1U);
  EXPECT_GT(free_voxels, 0U);
  EXPECT_EQ(carved_voxels, 1U);

  const auto snapshot = snapshotOf(map, new_frame, second_pose);
  EXPECT_NEAR(snapshot->getDisToOcc(second_pose), 2.0, 1e-6);
}

TEST(LidarMapRayCarving, PreservesEveryCurrentFrameEndpoint)
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

TEST(LidarMapRayCarving, KeepsFreeRayEvidenceAcrossMapSnapshots)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const auto frame = cloud({PointType(2.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    frame, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));

  const auto free = map.freeSpaceSnapshot();
  ASSERT_GT(free.size(), 0U);

  LIOInterface restored;
  restored.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  restored.loadSnapshot(
    map.accumulatedCloudSnapshot(), frame, free,
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
  EXPECT_EQ(restored.freeSpaceSnapshot().size(), free.size());
}

TEST(LidarMapRayCarving, NewHitInvalidatesRememberedFreeVoxel)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  ASSERT_TRUE(map.updateCloudWorld(
    cloud({PointType(2.0F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  const auto before = map.freeSpaceSnapshot();
  ASSERT_GT(before.size(), 0U);

  ASSERT_TRUE(map.updateCloudWorld(
    cloud({PointType(1.0F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  const auto after = map.freeSpaceSnapshot();
  EXPECT_LT(after.size(), before.size());
}

TEST(LidarMapRayCarving, FarPlaneRayAddsFreeSpaceWithoutAnOccupiedEndpoint)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  ASSERT_TRUE(map.updateFreeRaysWorld(
    cloud({PointType(4.0F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  EXPECT_TRUE(map.accumulatedCloudSnapshot().empty());
  EXPECT_GT(map.freeSpaceSnapshot().size(), 10U);
}

TEST(LidarMapRayCarving, FarPlaneRayDoesNotEraseACurrentFrameHit)
{
  LIOInterface map;
  map.configureStorage(0.25F, 100.0F, 1000, 100.0F);
  const auto hit = cloud({PointType(2.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    hit, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  map.updateFreeRaysWorld(
    cloud({PointType(4.0F, 0.0F, 0.0F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
  ASSERT_EQ(map.accumulatedCloudSnapshot().size(), 1U);
  EXPECT_NEAR(map.accumulatedCloudSnapshot().front().x, 2.0F, 1e-6F);
}

}  // namespace
