#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "openseek_frgraph_ros2/depth_projection.hpp"

namespace {

using openseek_frgraph_ros2::CameraModel;
using openseek_frgraph_ros2::depth_planar_to_flu;
using openseek_frgraph_ros2::depth_planar_far_plane_to_flu;

TEST(DepthProjection, ConvertsOpticalDepthToBodyFlu) {
  const std::vector<float> depth = {
      2.0F, 2.0F, 2.0F,
      2.0F, 2.0F, 2.0F,
      2.0F, 2.0F, 2.0F,
  };
  CameraModel camera;
  camera.fx = 2.0;
  camera.fy = 2.0;
  camera.cx = 1.0;
  camera.cy = 1.0;

  const auto points = depth_planar_to_flu(
      depth.data(), 3, 3, 3 * sizeof(float), camera, 20.0);
  ASSERT_EQ(points.size(), 9U);

  const auto &top_left = points.front();
  EXPECT_FLOAT_EQ(top_left.x, 2.0F);
  EXPECT_FLOAT_EQ(top_left.y, 1.0F);
  EXPECT_FLOAT_EQ(top_left.z, 1.0F);

  const auto &center = points[4];
  EXPECT_FLOAT_EQ(center.x, 2.0F);
  EXPECT_FLOAT_EQ(center.y, 0.0F);
  EXPECT_FLOAT_EQ(center.z, 0.0F);

  const auto &bottom_right = points.back();
  EXPECT_FLOAT_EQ(bottom_right.x, 2.0F);
  EXPECT_FLOAT_EQ(bottom_right.y, -1.0F);
  EXPECT_FLOAT_EQ(bottom_right.z, -1.0F);
}

TEST(DepthProjection, FiltersInvalidAndOutOfRangeSamples) {
  const std::vector<float> depth = {
      std::numeric_limits<float>::quiet_NaN(),
      0.0F,
      -1.0F,
      20.0F,
      21.0F,
      5.0F,
  };
  CameraModel camera;
  camera.fx = 1.0;
  camera.fy = 1.0;
  camera.cx = 0.0;
  camera.cy = 0.0;

  const auto points = depth_planar_to_flu(
      depth.data(), 6, 1, 6 * sizeof(float), camera, 20.0);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points[0].x, 5.0F);
  EXPECT_FLOAT_EQ(points[0].y, -25.0F);
  EXPECT_FLOAT_EQ(points[0].z, 0.0F);
}

TEST(DepthProjection, AppliesCameraTranslationInBodyFrame) {
  const float depth = 3.0F;
  CameraModel camera;
  camera.fx = 1.0;
  camera.fy = 1.0;
  camera.cx = 0.0;
  camera.cy = 0.0;
  camera.body_from_camera_tx = 0.1;
  camera.body_from_camera_ty = -0.2;
  camera.body_from_camera_tz = 0.3;

  const auto points = depth_planar_to_flu(
      &depth, 1, 1, sizeof(float), camera, 20.0);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points[0].x, 3.1F);
  EXPECT_FLOAT_EQ(points[0].y, -0.2F);
  EXPECT_FLOAT_EQ(points[0].z, 0.3F);
}

TEST(DepthProjection, SeparatesFarPlaneFreeRaysFromObstacleReturns) {
  const std::vector<float> depth = {20.0F, 5.0F, 19.9F, 0.0F};
  CameraModel camera;
  camera.fx = 1.0;
  camera.fy = 1.0;
  camera.cx = 1.5;
  camera.cy = 0.0;

  const auto points = depth_planar_to_flu(
      depth.data(), 4, 1, 4 * sizeof(float), camera, 20.0);
  const auto rays = depth_planar_far_plane_to_flu(
      depth.data(), 4, 1, 4 * sizeof(float), camera, 20.0);
  ASSERT_EQ(points.size(), 2U);
  ASSERT_EQ(rays.size(), 1U);
  EXPECT_FLOAT_EQ(rays.front().x, 20.0F);
  EXPECT_FLOAT_EQ(rays.front().y, 30.0F);
}

TEST(DepthProjection, SubsamplesFreeRaysAcrossBothHorizontalSides) {
  const std::vector<float> depth(8, 20.0F);
  CameraModel camera;
  camera.fx = 2.0;
  camera.fy = 2.0;
  camera.cx = 3.5;
  camera.cy = 0.0;

  const auto rays = depth_planar_far_plane_to_flu(
      depth.data(), 8, 1, 8 * sizeof(float), camera, 20.0, 2);
  ASSERT_EQ(rays.size(), 4U);
  EXPECT_GT(rays.front().y, 0.0F);
  EXPECT_LT(rays.back().y, 0.0F);
}

}  // namespace
