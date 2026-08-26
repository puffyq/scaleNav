#include <gtest/gtest.h>

#include "scalenav_graph_ros2/route_memory.hpp"

namespace {

Eigen::Vector3f point(float x, float y = 0.0F, float z = 0.0F)
{
  return Eigen::Vector3f(x, y, z);
}

TEST(RouteMemory, KeepsOnlyTheRouteAheadOfTheVehicle)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(2.0F), point(5.0F), point(8.0F), point(12.0F)};
  const auto window = scalenav_graph::forwardRouteWindow(path, point(4.0F, 1.0F), 5.0F);

  ASSERT_GE(window.size(), 3U);
  EXPECT_NEAR(window.front().x(), 4.0F, 1e-5F);
  EXPECT_NEAR(window.front().y(), 0.0F, 1e-5F);
  EXPECT_NEAR(window.back().x(), 9.0F, 1e-5F);
  EXPECT_NEAR(window.back().y(), 0.0F, 1e-5F);
}

TEST(RouteMemory, PersistentForwardRouteKeepsTheSameTerminal)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(3.0F, 1.0F), point(8.0F, 1.0F), point(14.0F, 2.0F)};
  const auto forward = scalenav_graph::forwardRouteFromPosition(
    path, point(2.0F, 0.8F));

  ASSERT_EQ(forward.size(), 4U);
  EXPECT_NEAR(forward.front().x(), 2.04F, 1e-2F);
  EXPECT_NEAR(forward.front().y(), 0.69F, 1e-2F);
  EXPECT_TRUE(forward.back().isApprox(path.back()));
  EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
    point(2.0F, 0.8F), path, 0.0F, 0.5F));
}

TEST(RouteMemory, DoesNotRememberEdgesAlreadyBehindTheVehicle)
{
  const std::vector<Eigen::Vector3f> path = {
    point(0.0F), point(2.0F), point(5.0F), point(8.0F), point(12.0F)};
  const auto window = scalenav_graph::forwardRouteWindow(path, point(4.0F), 5.0F);

  EXPECT_FALSE(scalenav_graph::edgeFollowsRoute(point(1.0F), point(2.0F), window, 0.25F));
  EXPECT_TRUE(scalenav_graph::edgeFollowsRoute(point(5.0F), point(7.0F), window, 0.25F));
}

TEST(RouteMemory, RejectsANearbyParallelCorridorOutsideTheRemapTolerance)
{
  const std::vector<Eigen::Vector3f> route = {point(4.0F), point(10.0F)};

  EXPECT_TRUE(scalenav_graph::edgeFollowsRoute(
    point(5.0F, 0.15F), point(7.0F, 0.15F), route, 0.25F));
  EXPECT_FALSE(scalenav_graph::edgeFollowsRoute(
    point(5.0F, 0.35F), point(7.0F, 0.35F), route, 0.25F));
}

TEST(RouteMemory, HoldsTheRollingTerminalUntilItIsActuallyReached)
{
  EXPECT_TRUE(scalenav_graph::shouldReuseTerminal(point(0.0F), point(2.0F), 1.0F));
  EXPECT_FALSE(scalenav_graph::shouldReuseTerminal(point(1.2F), point(2.0F), 1.0F));
}

TEST(RouteMemory, ReleasesAStaleTerminalAfterLeavingTheDirectedRoute)
{
  const std::vector<Eigen::Vector3f> route = {point(0.0F), point(0.0F, 2.0F)};
  EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
    point(0.0F, 0.5F), route, 0.5F, 1.0F));
  EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
    point(2.0F, 1.0F), route, 0.5F, 1.0F));
  EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
    point(0.0F, 2.0F), route, 0.5F, 1.0F));
}

TEST(RouteMemory, ExtendsBeforeTheLookaheadReachesTheRememberedTerminal)
{
  const std::vector<Eigen::Vector3f> route = {point(0.0F), point(20.0F)};
  const float lookahead = 5.0F;

  EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
    point(0.0F), route, lookahead, 1.0F));
  EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
    point(8.0F), route, lookahead, 1.0F));
  EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
    point(16.0F), route, lookahead, 1.0F));
  EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
    point(8.0F), route, 0.0F, 1.0F));
}

TEST(RouteMemory, IgnoresSmallSemanticEmaChanges)
{
  EXPECT_FALSE(scalenav_graph::semanticRiskChangeRequiresReplan(0.20F, 0.30F, 0.15F));
  EXPECT_TRUE(scalenav_graph::semanticRiskChangeRequiresReplan(0.20F, 0.36F, 0.15F));
  EXPECT_FALSE(scalenav_graph::semanticRiskChangeRequiresReplan(0.40F, 0.20F, 0.15F));
  EXPECT_FALSE(scalenav_graph::semanticRiskChangeRequiresReplan(
    std::numeric_limits<float>::quiet_NaN(), 0.2F, 0.15F));
}

TEST(RouteMemory, StaticGeometryDoesNotResetOnSemanticFrameNoiseByDefault)
{
  EXPECT_FALSE(scalenav_graph::semanticRouteResetRequested(false, 0.10F, 0.90F, 0.15F));
  EXPECT_TRUE(scalenav_graph::semanticRouteResetRequested(true, 0.10F, 0.90F, 0.15F));
}

TEST(RouteMemory, ContinuityUsesTheNearestPathSegmentInsteadOfTheFirstPoint)
{
  const std::vector<Eigen::Vector3f> route = {
    point(0.0F), point(3.0F), point(8.0F), point(14.0F)};
  EXPECT_TRUE(scalenav_graph::isContinuousForwardRoute(
    point(6.0F, 0.2F), route, 0.5F));
  EXPECT_FALSE(scalenav_graph::isContinuousForwardRoute(
    point(6.0F, 1.0F), route, 0.5F));
}

TEST(RouteMemory, CandidateNeedsHysteresisToReplaceAnIncumbent)
{
  EXPECT_FALSE(scalenav_graph::shouldSwitchRoute(
    false, 0.25F, 0.22F, 0.08F, 10.0F, 9.5F, 0.0F, 3.5F, 0.90F));
  EXPECT_TRUE(scalenav_graph::shouldSwitchRoute(
    false, 0.30F, 0.20F, 0.08F, 10.0F, 9.5F, 0.0F, 3.5F, 0.90F));
  EXPECT_TRUE(scalenav_graph::shouldSwitchRoute(
    true, 0.25F, 0.24F, 0.08F, 10.0F, 10.0F, 20.0F, 0.1F, 0.99F));
}

TEST(RouteMemory, CompatibleCandidateMayExtendTheRollingFrontier)
{
  const std::vector<Eigen::Vector3f> accepted = {
    point(0.0F), point(10.0F), point(20.0F), point(30.0F)};
  const std::vector<Eigen::Vector3f> extension = {
    point(0.0F, 0.1F), point(10.0F, 0.1F), point(20.0F, 0.2F),
    point(30.0F, 0.4F), point(35.0F, 1.0F)};
  const std::vector<Eigen::Vector3f> lane_switch = {
    point(0.0F), point(8.0F, 3.3F), point(20.0F, 3.3F), point(35.0F, 3.3F)};

  EXPECT_TRUE(scalenav_graph::candidateExtendsAcceptedRoute(
    accepted, extension, 1.0F, 1.5F));
  EXPECT_FALSE(scalenav_graph::candidateExtendsAcceptedRoute(
    accepted, lane_switch, 1.0F, 1.5F));
}

}  // namespace
