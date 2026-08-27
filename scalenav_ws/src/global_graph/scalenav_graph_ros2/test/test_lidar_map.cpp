#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "lidar_map/lidar_map.h"

namespace {

using fast_planner::LIOInterface;
using fast_planner::PointType;
using fast_planner::PointVector;

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

TEST(LidarMapContract, TcM1001InitializationAllowsEmptyQueries)
{
  LIOInterface map;
  ros::NodeHandle node;
  map.init(node);

  PointVector nearest;
  std::vector<float> distances;
  map.KNN(PointType(0.0F, 0.0F, 0.0F), 3, nearest, distances);
  EXPECT_TRUE(nearest.empty());
  EXPECT_TRUE(distances.empty());

  PointVector boxed;
  map.boxSearch(Eigen::Vector3f(-1.0F, -1.0F, -1.0F),
                Eigen::Vector3f(1.0F, 1.0F, 1.0F), boxed);
  EXPECT_TRUE(boxed.empty());
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  EXPECT_TRUE(std::isfinite(map.getDisToOcc(origin)));
}

TEST(LidarMapContract, TcM1002VectorIsInBoxHonorsBoundaryAndDeadArea)
{
  LIOInterface map;
  map.configureBounds(Eigen::Vector3f(-2.0F, -2.0F, -2.0F),
                      Eigen::Vector3f(2.0F, 2.0F, 2.0F));
  map.lp_->dead_area_num_ = 1;
  map.lp_->dead_area_min_boundary_vec_ = {Eigen::Vector3f(-0.5F, -0.5F, -0.5F)};
  map.lp_->dead_area_max_boundary_vec_ = {Eigen::Vector3f(0.5F, 0.5F, 0.5F)};

  EXPECT_TRUE(map.IsInBox(Eigen::Vector3f(1.0F, 0.0F, 0.0F)));
  EXPECT_TRUE(map.IsInBox(Eigen::Vector3f(-2.0F, 0.0F, 0.0F)));
  EXPECT_FALSE(map.IsInBox(Eigen::Vector3f(2.01F, 0.0F, 0.0F)));
  EXPECT_FALSE(map.IsInBox(Eigen::Vector3f(0.0F, 0.0F, 0.0F)));
}

TEST(LidarMapContract, TcM1003PointIsInBoxMatchesVectorOverload)
{
  LIOInterface map;
  map.configureBounds(Eigen::Vector3f(-2.0F, -2.0F, -2.0F),
                      Eigen::Vector3f(2.0F, 2.0F, 2.0F));
  map.lp_->dead_area_num_ = 1;
  map.lp_->dead_area_min_boundary_vec_ = {Eigen::Vector3f(-0.5F, -0.5F, -0.5F)};
  map.lp_->dead_area_max_boundary_vec_ = {Eigen::Vector3f(0.5F, 0.5F, 0.5F)};
  const std::array<Eigen::Vector3f, 4> inputs = {
    Eigen::Vector3f(1.0F, 0.0F, 0.0F), Eigen::Vector3f(-2.0F, 0.0F, 0.0F),
    Eigen::Vector3f(2.01F, 0.0F, 0.0F), Eigen::Vector3f(0.0F, 0.0F, 0.0F)};

  for (const auto &input : inputs) {
    EXPECT_EQ(map.IsInBox(input), map.IsInBox(PointType(input.x(), input.y(), input.z())));
  }
}

TEST(LidarMapContract, TcM1004VectorIsInMapUsesInsetBoundary)
{
  LIOInterface map;
  map.configureBounds(Eigen::Vector3f(-1.0F, -1.0F, -1.0F),
                      Eigen::Vector3f(1.0F, 1.0F, 1.0F));
  constexpr float inset = 1.0e-4F;

  EXPECT_FALSE(map.IsInMap(Eigen::Vector3f(-1.0F, 0.0F, 0.0F)));
  EXPECT_FALSE(map.IsInMap(Eigen::Vector3f(-1.0F + 0.5F * inset, 0.0F, 0.0F)));
  EXPECT_TRUE(map.IsInMap(Eigen::Vector3f(-1.0F + 2.0F * inset, 0.0F, 0.0F)));
  EXPECT_TRUE(map.IsInMap(Eigen::Vector3f(1.0F - 2.0F * inset, 0.0F, 0.0F)));
  EXPECT_FALSE(map.IsInMap(Eigen::Vector3f(1.0F - 0.5F * inset, 0.0F, 0.0F)));
  EXPECT_FALSE(map.IsInMap(Eigen::Vector3f(1.01F, 0.0F, 0.0F)));
}

TEST(LidarMapContract, TcM1005PointIsInMapMatchesVectorOverload)
{
  LIOInterface map;
  map.configureBounds(Eigen::Vector3f(-1.0F, -1.0F, -1.0F),
                      Eigen::Vector3f(1.0F, 1.0F, 1.0F));
  constexpr float inset = 1.0e-4F;
  const std::array<Eigen::Vector3f, 6> inputs = {
    Eigen::Vector3f(-1.0F, 0.0F, 0.0F),
    Eigen::Vector3f(-1.0F + 0.5F * inset, 0.0F, 0.0F),
    Eigen::Vector3f(-1.0F + 2.0F * inset, 0.0F, 0.0F),
    Eigen::Vector3f(1.0F - 2.0F * inset, 0.0F, 0.0F),
    Eigen::Vector3f(1.0F - 0.5F * inset, 0.0F, 0.0F),
    Eigen::Vector3f(1.01F, 0.0F, 0.0F)};

  for (const auto &input : inputs) {
    EXPECT_EQ(map.IsInMap(input), map.IsInMap(PointType(input.x(), input.y(), input.z())));
  }
}

TEST(LidarMapContract, TcM1006DistanceOverloadsMatchGeometricTruth)
{
  LIOInterface map;
  map.configureStorage(0.05F, 100.0F, 1000, 100.0F);
  pcl::PointCloud<PointType> obstacles;
  for (int index = 0; index < 10; ++index) {
    obstacles.push_back(PointType(static_cast<float>(index), 0.0F, 0.0F));
  }
  ASSERT_TRUE(map.updateCloudWorld(
    obstacles, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));

  for (int index = 0; index < 30; ++index) {
    const float x = static_cast<float>(index) * 0.31F;
    const Eigen::Vector3f query(x, 0.4F, 0.0F);
    double expected = std::numeric_limits<double>::infinity();
    for (const auto &obstacle : obstacles) {
      expected = std::min(expected, static_cast<double>(
        (query - Eigen::Vector3f(obstacle.x, obstacle.y, obstacle.z)).norm()));
    }
    const double vector_f = map.getDisToOcc(query);
    const Eigen::Vector3d query_d = query.cast<double>();
    const double vector_d = map.getDisToOcc(query_d);
    const double point = map.getDisToOcc(PointType(query.x(), query.y(), query.z()));
    EXPECT_NEAR(vector_f, expected, 1.0e-5);
    EXPECT_NEAR(vector_d, expected, 1.0e-5);
    EXPECT_NEAR(point, expected, 1.0e-5);
    EXPECT_NEAR(vector_f, vector_d, 1.0e-5);
    EXPECT_NEAR(vector_f, point, 1.0e-5);
  }
}

TEST(LidarMapContract, TcM1007KnnReturnsSortedMatchingPoints)
{
  LIOInterface map;
  map.configureStorage(0.05F, 100.0F, 1000, 100.0F);
  pcl::PointCloud<PointType> known;
  for (int index = 0; index < 10; ++index) {
    known.push_back(PointType(static_cast<float>(index), 0.0F, 0.0F));
  }
  ASSERT_TRUE(map.updateCloudWorld(
    known, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));

  for (const int k : {1, 3, 20}) {
    for (int repetition = 0; repetition < 10; ++repetition) {
      PointVector nearest;
      std::vector<float> distances;
      const float query_x = 4.25F + 0.01F * static_cast<float>(repetition);
      map.KNN(PointType(query_x, 0.0F, 0.0F), k, nearest, distances);
      ASSERT_EQ(nearest.size(), static_cast<std::size_t>(std::min(k, 10)));
      ASSERT_EQ(distances.size(), nearest.size());
      EXPECT_TRUE(std::is_sorted(distances.begin(), distances.end()));
      for (std::size_t index = 0; index < nearest.size(); ++index) {
        EXPECT_NEAR(distances[index],
                    (nearest[index].x - query_x) * (nearest[index].x - query_x),
                    1.0e-5F);
      }
    }
  }
}

TEST(LidarMapContract, TcM1008BoxSearchUsesClosedBounds)
{
  LIOInterface map;
  map.configureStorage(0.05F, 100.0F, 1000, 100.0F);
  const auto known = cloud({
    PointType(-2.0F, 0.0F, 0.0F), PointType(-1.0F, 0.0F, 0.0F),
    PointType(0.0F, 0.0F, 0.0F), PointType(1.0F, 0.0F, 0.0F),
    PointType(2.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    known, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));

  for (int repetition = 0; repetition < 10; ++repetition) {
    PointVector selected;
    map.boxSearch(Eigen::Vector3f(-1.0F, -0.1F, -0.1F),
                  Eigen::Vector3f(1.0F, 0.1F, 0.1F), selected);
    ASSERT_EQ(selected.size(), 3U);
    for (const auto &point : selected) {
      EXPECT_GE(point.x, -1.0F);
      EXPECT_LE(point.x, 1.0F);
    }
  }
  for (int repetition = 0; repetition < 10; ++repetition) {
    PointVector selected;
    map.boxSearch(Eigen::Vector3f(3.0F, -0.1F, -0.1F),
                  Eigen::Vector3f(4.0F, 0.1F, 0.1F), selected);
    EXPECT_TRUE(selected.empty());
  }
}

TEST(LidarMapContract, TcM1009OneHundredFramesRespectVoxelAndWindowRules)
{
  LIOInterface map;
  map.configureStorage(0.25F, 40.0F, 1000, 0.1F);
  const Eigen::Vector3f pose = Eigen::Vector3f::Zero();
  const auto repeated = cloud({
    PointType(0.24F, 0.0F, 0.0F), PointType(0.13F, 0.0F, 0.0F),
    PointType(39.0F, 0.0F, 0.0F), PointType(40.01F, 0.0F, 0.0F)});

  for (int frame = 0; frame < 100; ++frame) {
    map.updateCloudWorld(repeated, pose, Eigen::Quaternionf::Identity());
  }
  const auto accumulated = map.accumulatedCloudSnapshot();
  ASSERT_EQ(accumulated.size(), 2U);
  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(0.13F, 0.0F, 0.0F)), 0.0, 1.0e-5);
  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(39.0F, 0.0F, 0.0F)), 0.0, 1.0e-5);
  EXPECT_GT(map.getDisToOcc(Eigen::Vector3f(40.01F, 0.0F, 0.0F)), 1.0);
}

TEST(LidarMapContract, TcM1013OneHundredTwentyFreeRayFramesDoNotCreateObstacles)
{
  LIOInterface map;
  map.configureStorage(0.25F, 40.0F, 1000, 0.1F);
  const Eigen::Vector3f pose = Eigen::Vector3f::Zero();
  const auto occupied = cloud({PointType(2.0F, 0.0F, 0.0F)});
  const auto free_rays = cloud({
    PointType(2.0F, 0.0F, 0.0F), PointType(4.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(map.updateCloudWorld(
    occupied, pose, Eigen::Quaternionf::Identity()));
  const std::size_t initial_count = map.pointCount();
  const double initial_distance = map.getDisToOcc(Eigen::Vector3f(4.0F, 0.0F, 0.0F));

  for (int frame = 0; frame < 120; ++frame) {
    EXPECT_FALSE(map.updateFreeRaysWorld(
      free_rays, pose, Eigen::Quaternionf::Identity()));
  }

  EXPECT_EQ(map.pointCount(), initial_count);
  EXPECT_TRUE(map.freeSpaceSnapshot().empty());
  EXPECT_NEAR(map.getDisToOcc(Eigen::Vector3f(4.0F, 0.0F, 0.0F)),
              initial_distance, 1.0e-5);
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

TEST(LidarMapBounds, SlidingWindowDropsPointsOutsideRadiusEvenInsideMissionBox)
{
  LIOInterface map;
  map.configureBounds(
    Eigen::Vector3f(-20.0F, -20.0F, -5.0F),
    Eigen::Vector3f(100.0F, 20.0F, 5.0F));
  map.configureStorage(0.25F, 20.0F, 1000U, 0.5F);
  ASSERT_TRUE(map.updateCloudWorld(
    cloud({PointType(10.0F, 0.0F, 1.6F)}),
    Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()));
  ASSERT_TRUE(map.updateCloudWorld(
    cloud({PointType(80.0F, 0.0F, 1.6F)}),
    Eigen::Vector3f(80.0F, 0.0F, 1.6F), Eigen::Quaternionf::Identity()));
  const auto accumulated = map.accumulatedCloudSnapshot();
  ASSERT_EQ(accumulated.size(), 1U);
  EXPECT_NEAR(accumulated.front().x, 80.0F, 1e-6F);
}

TEST(LidarMapOccupied, CapacityKeepsPointsNearestToVehicle)
{
  LIOInterface map;
  map.configureStorage(0.05F, 40.0F, 1000U, 0.1F);
  pcl::PointCloud<PointType> frame;
  frame.reserve(1200);
  for (int i = 0; i < 1200; ++i) {
    frame.push_back(PointType(0.05F * static_cast<float>(i), 0.0F, 1.6F));
  }

  const Eigen::Vector3f pose(20.0F, 0.0F, 1.6F);
  ASSERT_TRUE(map.updateCloudWorld(frame, pose, Eigen::Quaternionf::Identity()));
  const auto accumulated = map.accumulatedCloudSnapshot();
  ASSERT_EQ(accumulated.size(), 1000U);
  EXPECT_NEAR(map.getDisToOcc(pose), 0.0, 1e-5);
  EXPECT_LT(map.getDisToOcc(Eigen::Vector3f(0.0F, 0.0F, 1.6F)), 10.1);
  EXPECT_GT(map.getDisToOcc(Eigen::Vector3f(59.95F, 0.0F, 1.6F)), 9.9);
}

}  // namespace
