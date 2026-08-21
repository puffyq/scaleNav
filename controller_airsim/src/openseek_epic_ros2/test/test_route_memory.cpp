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

TEST(RouteLookahead, KeepsTheConfiguredMinimumAtLowSpeed)
{
  EXPECT_FLOAT_EQ(
    openseek_epic::velocityCompensatedLookahead(10.0F, 1.0F, 2.0F, 5.0F), 10.0F);
}

TEST(RouteLookahead, PreservesReserveUntilTheNextPlanningCycle)
{
  EXPECT_FLOAT_EQ(
    openseek_epic::velocityCompensatedLookahead(10.0F, 4.0F, 2.0F, 5.0F), 13.0F);
  EXPECT_FLOAT_EQ(
    openseek_epic::velocityCompensatedLookahead(10.0F, 5.0F, 2.0F, 5.0F), 15.0F);
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

TEST(RouteMemory, ExtendsBeforeTheLookaheadReachesTheRememberedTerminal)
{
  const std::vector<Eigen::Vector3f> route = {point(0.0F), point(20.0F)};
  const float lookahead =
    openseek_epic::velocityCompensatedLookahead(10.0F, 4.0F, 2.0F, 5.0F);

  EXPECT_TRUE(openseek_epic::canReuseForwardRoute(
    point(0.0F), route, lookahead, 1.0F));
  EXPECT_FALSE(openseek_epic::canReuseForwardRoute(
    point(8.0F), route, lookahead, 1.0F));
  EXPECT_TRUE(openseek_epic::canReuseForwardRoute(
    point(8.0F), route, 0.0F, 1.0F));
}

TEST(RaycastShortcut, CollapsesAnOpenPolylineToItsEndpoints)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(1.0F, 1.0F), point(2.0F, -1.0F), point(4.0F)};
  auto open_space = [](const Eigen::Vector3f &) { return 100.0F; };
  openseek_epic::RaycastShortcutStats stats;

  const auto shortened = openseek_epic::farthestVisibleShortcut(
    path, 0.25F, 0.65F, open_space, &stats);

  ASSERT_EQ(shortened.size(), 2U);
  EXPECT_TRUE(shortened.front().isApprox(path.front()));
  EXPECT_TRUE(shortened.back().isApprox(path.back()));
  EXPECT_GT(stats.clearance_queries, 2U);
  EXPECT_EQ(stats.accepted_segments, 1U);
}

TEST(RaycastShortcut, RetainsAWaypointWhenTheDirectSegmentIsBlocked)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(1.0F, 1.0F), point(3.0F, 1.0F), point(4.0F)};
  const Eigen::Vector3f obstacle = point(2.0F);
  auto clearance = [&obstacle](const Eigen::Vector3f &query) {
    return (query - obstacle).norm();
  };

  const auto shortened = openseek_epic::farthestVisibleShortcut(
    path, 0.10F, 0.55F, clearance);

  ASSERT_GT(shortened.size(), 2U);
  EXPECT_TRUE(shortened.front().isApprox(path.front()));
  EXPECT_TRUE(shortened.back().isApprox(path.back()));
  for (std::size_t i = 1; i < shortened.size(); ++i) {
    EXPECT_TRUE(openseek_epic::segmentHasClearance(
      shortened[i - 1], shortened[i], 0.10F, 0.55F, clearance));
  }
}

TEST(RaycastShortcut, RejectsClearanceBelowTheSafetyThreshold)
{
  auto insufficient = [](const Eigen::Vector3f &) { return 0.649F; };
  auto exact = [](const Eigen::Vector3f &) { return 0.65F; };

  EXPECT_FALSE(openseek_epic::segmentHasClearance(
    point(0.0F), point(1.0F), 0.25F, 0.65F, insufficient));
  EXPECT_TRUE(openseek_epic::segmentHasClearance(
    point(0.0F), point(1.0F), 0.25F, 0.65F, exact));
}

TEST(RaycastShortcut, RejectsUnknownInteriorButAllowsUnknownWitnessEndpoints)
{
  auto unknown_interior = [](const Eigen::Vector3f &query) {
    return query.x() > 0.24F && query.x() < 0.76F ?
      std::numeric_limits<float>::quiet_NaN() : 10.0F;
  };
  auto unknown_endpoints = [](const Eigen::Vector3f &query) {
    return (query.x() < 1e-4F || query.x() > 0.9999F) ?
      std::numeric_limits<float>::quiet_NaN() : 10.0F;
  };

  EXPECT_FALSE(openseek_epic::segmentHasClearance(
    point(0.0F), point(1.0F), 0.25F, 0.65F, unknown_interior));
  EXPECT_TRUE(openseek_epic::segmentHasClearance(
    point(0.0F), point(1.0F), 0.25F, 0.65F, unknown_endpoints));
}

TEST(RaycastShortcut, LeavesATwoPointPathUnchanged)
{
  const std::vector<Eigen::Vector3f> path = {point(0.0F), point(2.0F)};
  auto blocked = [](const Eigen::Vector3f &) { return 0.0F; };

  const auto shortened = openseek_epic::farthestVisibleShortcut(
    path, 0.25F, 0.65F, blocked);

  ASSERT_EQ(shortened.size(), 2U);
  EXPECT_TRUE(shortened.front().isApprox(path.front()));
  EXPECT_TRUE(shortened.back().isApprox(path.back()));
}

TEST(RaycastShortcut, RemovesRepeatedPointsWithoutStalling)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(0.0F), point(1.0F), point(1.0F), point(2.0F)};
  auto open_space = [](const Eigen::Vector3f &) { return 100.0F; };

  const auto shortened = openseek_epic::farthestVisibleShortcut(
    path, 0.25F, 0.65F, open_space);

  ASSERT_EQ(shortened.size(), 2U);
  EXPECT_TRUE(shortened.front().isApprox(point(0.0F)));
  EXPECT_TRUE(shortened.back().isApprox(point(2.0F)));
}

TEST(RaycastShortcut, SamplesBothEndpointsAndTheInterior)
{
  auto endpoint_obstacle = [](const Eigen::Vector3f &query) {
    return query.x() > 0.99F ? 0.1F : 100.0F;
  };
  auto narrow_obstacle = [](const Eigen::Vector3f &query) {
    return std::abs(query.x() - 0.5F) < 1e-4F ? 0.1F : 100.0F;
  };

  EXPECT_FALSE(openseek_epic::segmentHasClearance(
    point(0.0F), point(1.0F), 0.25F, 0.65F, endpoint_obstacle));
  EXPECT_FALSE(openseek_epic::segmentHasClearance(
    point(0.0F), point(1.0F), 0.25F, 0.65F, narrow_obstacle));
}

}  // namespace
