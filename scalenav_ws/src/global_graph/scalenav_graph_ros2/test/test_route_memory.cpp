#include <gtest/gtest.h>

#include "scalenav_graph_ros2/route_memory.hpp"

namespace {

Eigen::Vector3f point(float x, float y = 0.0F, float z = 0.0F)
{
  return Eigen::Vector3f(x, y, z);
}

TEST(RouteMemoryM5Contract, TcM5001PointSegmentDistanceCoversProjectionAndDegenerateSegment)
{
  for (int repetition = 0; repetition < 10000; ++repetition) {
    EXPECT_FLOAT_EQ(scalenav_graph::pointSegmentDistance(
      point(2.0F, 3.0F), point(0.0F), point(4.0F)), 3.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::pointSegmentDistance(
      point(-3.0F, 4.0F), point(0.0F), point(4.0F)), 5.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::pointSegmentDistance(
      point(7.0F, 4.0F), point(0.0F), point(4.0F)), 5.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::pointSegmentDistance(
      point(4.0F, 6.0F, 3.0F), point(1.0F, 2.0F, 3.0F),
      point(1.0F, 2.0F, 3.0F)), 5.0F);
  }
}

TEST(RouteMemoryM5Contract, TcM5002PointPathDistanceCoversEmptySingleAndPolyline)
{
  const std::vector<Eigen::Vector3f> single{point(0.0F)};
  const std::vector<Eigen::Vector3f> polyline{
    point(0.0F), point(4.0F), point(4.0F, 4.0F)};
  for (int repetition = 0; repetition < 10000; ++repetition) {
    EXPECT_TRUE(std::isinf(scalenav_graph::pointPathDistance(point(1.0F), {})));
    EXPECT_FLOAT_EQ(scalenav_graph::pointPathDistance(
      point(3.0F, 4.0F), single), 5.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::pointPathDistance(
      point(2.0F, 2.0F), polyline), 2.0F);
  }
}

TEST(RouteMemoryM5Contract, TcM5003ForwardRouteWindowStartsAtProjectionAndHonorsHorizon)
{
  const std::vector<Eigen::Vector3f> route{
    point(0.0F), point(4.0F), point(8.0F), point(12.0F), point(20.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const auto window = scalenav_graph::forwardRouteWindow(
      route, point(5.0F, 1.0F), 10.0F);
    ASSERT_EQ(window.size(), 4U);
    EXPECT_TRUE(window.front().isApprox(point(5.0F)));
    EXPECT_TRUE(window.back().isApprox(point(15.0F)));
    EXPECT_NEAR(scalenav_graph::routeLength(window), 10.0F, 1e-5F);
  }
}

TEST(RouteMemoryM5Contract, TcM5004ForwardRouteFromPositionKeepsCompleteSuffix)
{
  const std::vector<Eigen::Vector3f> route{
    point(0.0F), point(4.0F), point(8.0F, 2.0F), point(12.0F, 2.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const auto forward = scalenav_graph::forwardRouteFromPosition(
      route, point(6.0F, 2.0F));
    ASSERT_EQ(forward.size(), 3U);
    EXPECT_TRUE(forward.front().isApprox(point(6.4F, 1.2F), 1e-5F));
    EXPECT_TRUE(forward.back().isApprox(route.back()));
  }
}

TEST(RouteMemoryM5Contract, TcM5005ContinuousRouteCoversBoundaryOffsetAndPassedFrontierGoal)
{
  const std::vector<Eigen::Vector3f> route{point(0.0F), point(10.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_TRUE(scalenav_graph::isContinuousForwardRoute(
      point(5.0F), route, 0.5F));
    EXPECT_TRUE(scalenav_graph::isContinuousForwardRoute(
      point(5.0F, 0.5F), route, 0.5F));
    EXPECT_FALSE(scalenav_graph::isContinuousForwardRoute(
      point(5.0F, 0.51F), route, 0.5F));
    EXPECT_FALSE(scalenav_graph::isContinuousForwardRoute(
      point(10.2F), route, 0.5F));
  }
}

TEST(RouteMemoryM5Contract, TcM5006SwitchUsesAggregateLossHysteresisAndHardFailure)
{
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_TRUE(scalenav_graph::shouldSwitchRoute(
      true, 0.2F, 0.9F, 0.08F, 10.0F, 50.0F, 20.0F, 3.5F, 0.9F));
    EXPECT_FALSE(scalenav_graph::shouldSwitchRoute(
      false, 0.30F, 0.22F, 0.08F, 10.0F, 9.1F, 0.0F, 3.5F, 0.9F));
    EXPECT_TRUE(scalenav_graph::shouldSwitchRoute(
      false, 0.31F, 0.22F, 0.08F, 10.0F, 50.0F, 20.0F, 3.5F, 0.9F));
    EXPECT_TRUE(scalenav_graph::shouldSwitchRoute(
      false, 0.20F, 0.20F, 0.08F, 10.0F, 8.99F, 0.0F, 3.5F, 0.9F));
    EXPECT_FALSE(scalenav_graph::shouldSwitchRoute(
      false, 0.20F, 0.20F, 0.08F, 10.0F, 8.0F, 3.51F, 3.5F, 0.9F));
  }
}

TEST(RouteMemoryM5Contract, TcM5007EdgeFollowsRouteChecksEndpointsAndMidpoint)
{
  const std::vector<Eigen::Vector3f> straight{point(0.0F), point(10.0F)};
  const std::vector<Eigen::Vector3f> u_shape{
    point(0.0F), point(0.0F, 10.0F), point(10.0F, 10.0F), point(10.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_TRUE(scalenav_graph::edgeFollowsRoute(
      point(2.0F, 0.1F), point(8.0F, 0.1F), straight, 0.25F));
    EXPECT_FALSE(scalenav_graph::edgeFollowsRoute(
      point(0.0F), point(10.0F), u_shape, 0.25F));
    EXPECT_FALSE(scalenav_graph::edgeFollowsRoute(
      point(2.0F, 1.0F), point(8.0F, 1.0F), straight, 0.25F));
  }
}

TEST(RouteMemoryM5Contract, TcM5008RouteLengthSkipsNonFiniteSegments)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<Eigen::Vector3f> non_finite{
    point(0.0F), point(3.0F, 4.0F), point(nan), point(6.0F, 8.0F),
    point(9.0F, 8.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_FLOAT_EQ(scalenav_graph::routeLength({}), 0.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::routeLength(
      {point(0.0F), point(3.0F, 4.0F)}), 5.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::routeLength(
      {point(0.0F), point(3.0F), point(3.0F, 4.0F)}), 7.0F);
    EXPECT_FLOAT_EQ(scalenav_graph::routeLength(non_finite), 8.0F);
  }
}

TEST(RouteMemoryM5Contract, TcM5009OnlyCompatibleLongerCandidateIsAnExtension)
{
  const std::vector<Eigen::Vector3f> accepted{
    point(0.0F), point(10.0F), point(20.0F), point(30.0F)};
  const std::vector<Eigen::Vector3f> extension{
    point(0.0F, 0.1F), point(10.0F, 0.1F), point(20.0F, 0.2F),
    point(30.0F, 0.4F), point(35.0F, 0.5F)};
  const std::vector<Eigen::Vector3f> lane_switch{
    point(0.0F), point(8.0F, 3.0F), point(20.0F, 3.0F), point(35.0F, 3.0F)};
  const std::vector<Eigen::Vector3f> insufficient{
    point(0.0F), point(10.0F), point(20.0F), point(30.5F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_TRUE(scalenav_graph::candidateExtendsAcceptedRoute(
      accepted, extension, 1.0F, 1.5F));
    EXPECT_FALSE(scalenav_graph::candidateExtendsAcceptedRoute(
      accepted, lane_switch, 1.0F, 1.5F));
    EXPECT_FALSE(scalenav_graph::candidateExtendsAcceptedRoute(
      accepted, insufficient, 1.0F, 1.5F));
  }
}

TEST(RouteMemoryM5Contract, TcM5010FrontierGoalReuseUsesStrictReleaseBoundary)
{
  const Eigen::Vector3f frontier_goal = point(2.0F);
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_TRUE(scalenav_graph::shouldReuseFrontierGoal(
      point(0.0F), frontier_goal, 1.0F));
    EXPECT_FALSE(scalenav_graph::shouldReuseFrontierGoal(
      point(1.0F), frontier_goal, 1.0F));
    EXPECT_FALSE(scalenav_graph::shouldReuseFrontierGoal(
      point(1.1F), frontier_goal, 1.0F));
  }
}

TEST(RouteMemoryM5Contract, TcM5011ForwardRouteReuseNeedsAlignmentAndRemainingDistance)
{
  const std::vector<Eigen::Vector3f> route{point(0.0F), point(10.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
      point(5.0F), route, 2.0F, 0.5F));
    EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
      point(5.0F, 1.0F), route, 2.0F, 0.5F));
    EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
      point(11.0F), route, 2.0F, 0.5F));
    EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
      point(9.0F), route, 2.0F, 0.5F));
  }
}

TEST(RouteMemoryM5Contract, TcM5012SemanticRiskIncreaseUsesInclusiveFiniteThreshold)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_FALSE(scalenav_graph::semanticRiskIncreaseRequiresReplan(
      0.25F, 0.49F, 0.25F));
    EXPECT_TRUE(scalenav_graph::semanticRiskIncreaseRequiresReplan(
      0.25F, 0.50F, 0.25F));
    EXPECT_TRUE(scalenav_graph::semanticRiskIncreaseRequiresReplan(
      0.25F, 0.51F, 0.25F));
    EXPECT_FALSE(scalenav_graph::semanticRiskIncreaseRequiresReplan(
      0.50F, 0.25F, 0.25F));
    EXPECT_FALSE(scalenav_graph::semanticRiskIncreaseRequiresReplan(
      nan, 0.50F, 0.25F));
    EXPECT_FALSE(scalenav_graph::semanticRiskIncreaseRequiresReplan(
      0.25F, nan, 0.25F));
  }
}

TEST(RouteMemoryM5Contract, TcM5013SemanticRiskChangeMatchesIncreaseContract)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<std::vector<float>> cases{
    {0.25F, 0.49F, 0.25F}, {0.25F, 0.50F, 0.25F},
    {0.25F, 0.51F, 0.25F}, {0.50F, 0.25F, 0.25F},
    {nan, 0.50F, 0.25F}, {0.25F, nan, 0.25F}};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    for (const auto &input : cases) {
      EXPECT_EQ(scalenav_graph::semanticRiskChangeRequiresReplan(
        input[0], input[1], input[2]),
        scalenav_graph::semanticRiskIncreaseRequiresReplan(
        input[0], input[1], input[2]));
    }
  }
}

TEST(RouteMemoryM5Contract, TcM5014SemanticResetRequiresEnableAndRiskTrigger)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int repetition = 0; repetition < 1000; ++repetition) {
    EXPECT_FALSE(scalenav_graph::semanticRouteResetRequested(
      false, 0.1F, 0.9F, 0.25F));
    EXPECT_FALSE(scalenav_graph::semanticRouteResetRequested(
      true, 0.1F, 0.34F, 0.25F));
    EXPECT_TRUE(scalenav_graph::semanticRouteResetRequested(
      true, 0.25F, 0.50F, 0.25F));
    EXPECT_TRUE(scalenav_graph::semanticRouteResetRequested(
      true, 0.25F, 0.51F, 0.25F));
    EXPECT_FALSE(scalenav_graph::semanticRouteResetRequested(
      true, nan, 0.9F, 0.25F));
  }
}

TEST(RouteMemoryM5Contract, TcM5018RouteLookaheadFollowsPolylineOrder)
{
  const std::vector<Eigen::Vector3f> route{
    point(0.0F), point(0.0F, 5.0F), point(-10.0F, 5.0F),
    point(-10.0F, 15.0F)};
  for (int repetition = 0; repetition < 1000; ++repetition) {
    Eigen::Vector3f next_goal = point(99.0F);
    ASSERT_TRUE(scalenav_graph::routeLookaheadPoint(
      route, point(0.0F, 2.0F), 10.0F, next_goal));
    EXPECT_TRUE(next_goal.isApprox(point(-7.0F, 5.0F), 1e-5F));
  }
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

TEST(RouteMemory, PersistentForwardRouteKeepsTheSameFrontierGoal)
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

TEST(RouteMemory, HoldsTheRollingFrontierGoalUntilItIsActuallyReached)
{
  EXPECT_TRUE(scalenav_graph::shouldReuseFrontierGoal(point(0.0F), point(2.0F), 1.0F));
  EXPECT_FALSE(scalenav_graph::shouldReuseFrontierGoal(point(1.2F), point(2.0F), 1.0F));
}

TEST(RouteMemory, ReleasesAStaleFrontierGoalAfterLeavingTheDirectedRoute)
{
  const std::vector<Eigen::Vector3f> route = {point(0.0F), point(0.0F, 2.0F)};
  EXPECT_TRUE(scalenav_graph::canReuseForwardRoute(
    point(0.0F, 0.5F), route, 0.5F, 1.0F));
  EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
    point(2.0F, 1.0F), route, 0.5F, 1.0F));
  EXPECT_FALSE(scalenav_graph::canReuseForwardRoute(
    point(0.0F, 2.0F), route, 0.5F, 1.0F));
}

TEST(RouteMemory, ExtendsBeforeTheLookaheadReachesTheRememberedFrontierGoal)
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

TEST(RouteMemory, PolynomialGuideSnapsVelocityBoundaryToLayer)
{
  const std::vector<Eigen::Vector3f> witness{
    point(0.0F), point(0.0F, 10.0F), point(0.0F, 20.0F)};
  const Eigen::Vector3f velocity(5.0F, 0.0F, 0.05F);
  const auto guide = scalenav_graph::buildPolynomialGuidePath(
    witness, point(0.0F, 2.0F), velocity, 1.6F);

  ASSERT_GE(guide.size(), 2U);
  for (const auto &point : guide) {
    EXPECT_NEAR(point.z(), 1.6F, 1e-5F);
  }
  EXPECT_NEAR(guide.front().y(), 2.0F, 1e-3F);
}

}  // namespace
