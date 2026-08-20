#include <gtest/gtest.h>

#include "openseek_epic_ros2/route_memory.hpp"

namespace {

Eigen::Vector3f point(float x, float y = 0.0F, float z = 0.0F)
{
  return Eigen::Vector3f(x, y, z);
}

TEST(RouteMemory, KeepsOnlyTheRouteAheadOfTheVehicle)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(2.0F), point(5.0F), point(8.0F), point(12.0F)};
  const auto window = openseek_epic::forwardRouteWindow(path, point(4.0F, 1.0F), 5.0F);

  ASSERT_GE(window.size(), 3U);
  EXPECT_NEAR(window.front().x(), 4.0F, 1e-5F);
  EXPECT_NEAR(window.front().y(), 0.0F, 1e-5F);
  EXPECT_NEAR(window.back().x(), 9.0F, 1e-5F);
  EXPECT_NEAR(window.back().y(), 0.0F, 1e-5F);
}

TEST(RouteMemory, DoesNotRememberEdgesAlreadyBehindTheVehicle)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(2.0F), point(5.0F), point(8.0F), point(12.0F)};
  const auto window = openseek_epic::forwardRouteWindow(path, point(4.0F), 5.0F);

  EXPECT_FALSE(openseek_epic::edgeFollowsRoute(point(1.0F), point(2.0F), window, 0.25F));
  EXPECT_TRUE(openseek_epic::edgeFollowsRoute(point(5.0F), point(7.0F), window, 0.25F));
}

TEST(RouteMemory, RejectsANearbyParallelCorridorOutsideTheRemapTolerance)
{
  const std::vector<Eigen::Vector3f> route = {point(4.0F), point(10.0F)};

  EXPECT_TRUE(openseek_epic::edgeFollowsRoute(
    point(5.0F, 0.15F), point(7.0F, 0.15F), route, 0.25F));
  EXPECT_FALSE(openseek_epic::edgeFollowsRoute(
    point(5.0F, 0.35F), point(7.0F, 0.35F), route, 0.25F));
}

TEST(RouteMemory, HoldsTheRollingTerminalUntilItIsActuallyReached)
{
  EXPECT_TRUE(openseek_epic::shouldReuseTerminal(point(0.0F), point(2.0F), 1.0F));
  EXPECT_FALSE(openseek_epic::shouldReuseTerminal(point(1.2F), point(2.0F), 1.0F));
}

TEST(RouteMemory, ReleasesAStaleTerminalAfterLeavingTheDirectedRoute)
{
  const std::vector<Eigen::Vector3f> route = {point(0.0F), point(0.0F, 2.0F)};
  EXPECT_TRUE(openseek_epic::canReuseForwardRoute(
    point(0.0F, 0.5F), route, 0.5F, 1.0F));
  EXPECT_FALSE(openseek_epic::canReuseForwardRoute(
    point(2.0F, 1.0F), route, 0.5F, 1.0F));
  EXPECT_FALSE(openseek_epic::canReuseForwardRoute(
    point(0.0F, 2.0F), route, 0.5F, 1.0F));
}

}  // namespace
