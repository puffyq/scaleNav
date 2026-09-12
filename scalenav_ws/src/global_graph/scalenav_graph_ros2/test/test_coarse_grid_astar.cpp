#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "coarse_grid_astar/coarse_grid_astar.h"

namespace {

scalenav::CoarseGridAstarConfig makeConfig(float resolution, float origin_x,
                                           float origin_y, float origin_z,
                                           int width, int height, int depth) {
  scalenav::CoarseGridAstarConfig config;
  config.resolution_m = resolution;
  config.inflate_radius_m = 0.61F;
  config.origin_x_m = origin_x;
  config.origin_y_m = origin_y;
  config.origin_z_m = origin_z;
  config.width = width;
  config.height = height;
  config.depth = depth;
  return config;
}

void markColumn(scalenav::CoarseGridAstar &grid, float x, float y) {
  const auto &config = grid.config();
  for (int iz = 0; iz < config.depth; ++iz) {
    const float z = config.origin_z_m + (static_cast<float>(iz) + 0.5F) * config.resolution_m;
    grid.markWorld(x, y, z);
  }
}

float maxAbsX(const std::vector<Eigen::Vector3f> &path) {
  float value = 0.0F;
  for (const auto &point : path) value = std::max(value, std::abs(point.x()));
  return value;
}

TEST(CoarseGridAstar, EmptyMapGoesStraightToGoal) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 20.0F, 1.6F});
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_NEAR(result.path_length_m, 20.0F, 1.0F);
  EXPECT_LT(maxAbsX(result.path), 0.8F);
  EXPECT_NEAR(result.path.back().z(), 1.6F, 0.6F);
}

TEST(CoarseGridAstar, TallBlockForcesLocalDetour) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -20.0F, -5.0F, 0.0F, 80, 100, 16));
  for (float x = -8.0F; x <= 8.0F; x += 0.1F) {
    markColumn(grid, x, 10.0F);
  }
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 140.0F, 1.6F});
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(result.clipped);
  EXPECT_NEAR(result.clipped_goal.y(), 20.0F, 0.8F);
  EXPECT_LT(result.path.back().y(), 22.0F);
  EXPECT_GT(maxAbsX(result.path), 8.0F);
}

TEST(CoarseGridAstar, ThinLayerIsNotAVerticalWall) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  for (float x = -8.0F; x <= 8.0F; x += 0.1F) {
    grid.markWorld(x, 10.0F, 1.6F);
  }
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 20.0F, 1.6F});
  ASSERT_TRUE(result.success) << result.reason;
  float min_z = 10.0F;
  float max_z = 0.0F;
  for (const auto &point : result.path) {
    min_z = std::min(min_z, point.z());
    max_z = std::max(max_z, point.z());
  }
  EXPECT_TRUE(max_z > 2.2F || min_z < 1.0F) << "min_z=" << min_z << " max_z=" << max_z;
  EXPECT_LT(maxAbsX(result.path), 8.0F);
  EXPECT_GT(result.path.back().y(), 15.0F);
}

TEST(CoarseGridAstar, RecedingHorizonDoesNotPlanTheFullMission) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -50.0F, -10.0F, 0.0F, 200, 340, 20));
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 140.0F, 1.6F});
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(result.clipped);
  EXPECT_NEAR(result.path_length_m, 20.0F, 1.5F);
  EXPECT_NEAR(result.path.back().y(), 20.0F, 0.8F);
}

TEST(CoarseGridAstar, RollingWindowDropsFarHits) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -50.0F, -10.0F, 0.0F, 200, 340, 20));
  EXPECT_TRUE(grid.markWorld(0.0F, 50.0F, 1.6F));
  EXPECT_EQ(grid.occupiedCount(), 1U);
  grid.retainAround(0.0F, 0.0F, 1.6F);
  EXPECT_EQ(grid.occupiedCount(), 0U);
}

TEST(CoarseGridAstar, WideGapStaysOpenAtCoarseResolution) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -20.0F, -5.0F, 0.0F, 80, 60, 16));
  for (float x = -10.0F; x <= -2.5F; x += 0.1F) markColumn(grid, x, 10.0F);
  for (float x = 2.5F; x <= 10.0F; x += 0.1F) markColumn(grid, x, 10.0F);
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 20.0F, 1.6F});
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_LT(maxAbsX(result.path), 2.5F);
}

void markWallWithGap(scalenav::CoarseGridAstar &grid, float y, float gap_half_width) {
  for (float x = -9.5F; x <= 9.5F; x += 0.05F) {
    if (std::abs(x) >= gap_half_width) markColumn(grid, x, y);
  }
}

TEST(CoarseGridAstar, NarrowPassageClosesAtCoarseResolution) {
  const Eigen::Vector3f start(0.0F, 0.0F, 1.6F);
  const Eigen::Vector3f goal(0.0F, 20.0F, 1.6F);
  auto fine = scalenav::CoarseGridAstar(makeConfig(0.1F, -10.0F, -5.0F, 0.0F, 200, 300, 40));
  auto coarse = scalenav::CoarseGridAstar(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  markWallWithGap(fine, 10.0F, 0.75F);
  markWallWithGap(coarse, 10.0F, 0.75F);

  const auto fine_result = fine.search(start, goal);
  const auto coarse_result = coarse.search(start, goal);
  ASSERT_TRUE(fine_result.success) << fine_result.reason;
  EXPECT_LT(fine_result.path_length_m, 24.0F);
  EXPECT_LT(coarse_result.path.back().y(), 10.0F);
  EXPECT_NE(coarse_result.reason, "ok");
}

TEST(CoarseGridAstar, BlockedForwardDoesNotReverse) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  markWallWithGap(grid, 6.0F, 0.75F);
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 140.0F, 1.6F});
  for (const auto &point : result.path) {
    EXPECT_GE(point.y(), -0.3F);
    EXPECT_LT(std::abs(point.x()), 8.0F);
  }
}

TEST(CoarseGridAstar, DenseSmallObstaclesBlockCoarseGrid) {
  const Eigen::Vector3f start(0.0F, 0.0F, 1.6F);
  const Eigen::Vector3f goal(0.0F, 20.0F, 1.6F);
  auto fine = scalenav::CoarseGridAstar(makeConfig(0.1F, -10.0F, -5.0F, 0.0F, 200, 300, 40));
  auto coarse = scalenav::CoarseGridAstar(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  for (int row = 0; row < 4; ++row) {
    const float y = 8.0F + 1.4F * static_cast<float>(row);
    for (int col = -7; col <= 7; ++col) {
      const float x = 1.4F * static_cast<float>(col);
      markColumn(fine, x, y);
      markColumn(coarse, x, y);
    }
  }

  const auto fine_result = fine.search(start, goal);
  const auto coarse_result = coarse.search(start, goal);
  ASSERT_TRUE(fine_result.success) << fine_result.reason;
  EXPECT_LT(fine_result.path_length_m, 30.0F);
  EXPECT_LT(coarse_result.path.back().y(), 8.5F);
  EXPECT_NE(coarse_result.reason, "ok");
}

TEST(CoarseGridAstar, LookaheadWalksAlongFoundPath) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 20.0F, 1.6F});
  ASSERT_TRUE(result.success);
  const Eigen::Vector3f local =
    grid.lookahead(result.path, {0.0F, 0.0F, 1.6F}, 8.0F);
  EXPECT_NEAR(local.x(), 0.0F, 0.6F);
  EXPECT_NEAR(local.y(), 8.0F, 0.8F);
}

TEST(CoarseGridAstar, ForwardGoalRejectsCurrentPoseAndReverse) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  const Eigen::Vector3f start(0.0F, 0.0F, 1.6F);
  const Eigen::Vector3f goal(0.0F, 140.0F, 1.6F);
  EXPECT_FALSE(grid.isForwardGoal(start, goal, start));
  EXPECT_FALSE(grid.isForwardGoal(start, goal, {0.0F, -2.0F, 1.6F}));
  EXPECT_FALSE(grid.isForwardGoal(start, goal, {15.0F, 0.2F, 1.6F}));
  EXPECT_TRUE(grid.isForwardGoal(start, goal, {0.0F, 5.0F, 1.6F}));
  EXPECT_TRUE(grid.isForwardGoal(start, goal, {3.0F, 8.0F, 2.4F}));
}

TEST(CoarseGridAstar, ProgressLookaheadStaysOnMissionSide) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  const Eigen::Vector3f start(0.0F, 0.0F, 1.6F);
  const Eigen::Vector3f goal(0.0F, 140.0F, 1.6F);
  const std::vector<Eigen::Vector3f> path = {
    start,
    {8.0F, 0.2F, 1.6F},
    {8.0F, 6.0F, 2.2F},
    {2.0F, 12.0F, 1.8F},
  };
  const Eigen::Vector3f local = grid.progressLookahead(path, start, goal, 15.0F);
  EXPECT_GT(local.y(), 5.0F);
  EXPECT_TRUE(grid.isForwardGoal(start, goal, local));
}

TEST(CoarseGridAstar, GroundVoxelsDoNotBlockFlightAltitude) {
  scalenav::CoarseGridAstar grid(makeConfig(0.5F, -10.0F, -5.0F, 0.0F, 40, 60, 16));
  EXPECT_TRUE(grid.markWorld(0.0F, 10.0F, 0.1F));
  EXPECT_TRUE(grid.markWorld(0.0F, 10.0F, 1.6F));
  EXPECT_EQ(grid.occupiedCount(), 2U);
  EXPECT_TRUE(grid.isBlocked(0.0F, 10.0F, 1.6F));
  EXPECT_FALSE(grid.isBlocked(0.0F, 10.0F, 4.0F));
  const auto result = grid.search({0.0F, 0.0F, 1.6F}, {0.0F, 20.0F, 1.6F});
  ASSERT_TRUE(result.success) << result.reason;
}

}  // namespace
