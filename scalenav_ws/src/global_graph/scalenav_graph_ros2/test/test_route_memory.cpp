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

TEST(RouteMemoryM5Contract, TcM5019SemanticHeightUsesTheFullInfluenceRadius)
{
  EXPECT_TRUE(scalenav_graph::semanticPointCanInfluenceFixedLayer(6.6F, 1.6F, 5.0F));
  EXPECT_TRUE(scalenav_graph::semanticPointCanInfluenceFixedLayer(-3.4F, 1.6F, 5.0F));
  EXPECT_FALSE(scalenav_graph::semanticPointCanInfluenceFixedLayer(6.61F, 1.6F, 5.0F));
  EXPECT_FALSE(scalenav_graph::semanticPointCanInfluenceFixedLayer(
    std::numeric_limits<float>::quiet_NaN(), 1.6F, 5.0F));
}

TEST(RouteMemoryM5Contract, TcM5020FrontierRefreshUsesConfiguredProgressFraction)
{
  EXPECT_FALSE(scalenav_graph::routeProgressReachedFraction(7.99F, 20.0F, 0.40F));
  EXPECT_TRUE(scalenav_graph::routeProgressReachedFraction(8.0F, 20.0F, 0.40F));
  EXPECT_TRUE(scalenav_graph::routeProgressReachedFraction(8.01F, 20.0F, 0.40F));
  EXPECT_FALSE(scalenav_graph::routeProgressReachedFraction(8.0F, 0.0F, 0.40F));
}

TEST(RouteMemoryM5Contract, TcM5021MissionGoalUsesTheLargerDirectHorizon)
{
  EXPECT_TRUE(scalenav_graph::missionGoalWithinDirectHorizon(15.0F, 6.0F, 15.0F));
  EXPECT_FALSE(scalenav_graph::missionGoalWithinDirectHorizon(15.01F, 6.0F, 15.0F));
  EXPECT_TRUE(scalenav_graph::missionGoalWithinDirectHorizon(8.0F, 8.0F, 5.0F));
  EXPECT_FALSE(scalenav_graph::missionGoalWithinDirectHorizon(
    std::numeric_limits<float>::quiet_NaN(), 6.0F, 15.0F));
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

TEST(RouteMemory, ConstrainedPolynomialUsesExactReplanState)
{
  const Eigen::Vector3f start(2.0F, 3.0F, 1.6F);
  const Eigen::Vector3f velocity(4.0F, 1.0F, 0.0F);
  const std::vector<Eigen::Vector3f> guide{
    start,
    start + 0.2F * velocity,
    Eigen::Vector3f(5.0F, 8.0F, 1.6F),
    Eigen::Vector3f(4.0F, 16.0F, 1.6F)};

  const auto curve =
    scalenav_graph::WitnessParametricCurve::fitWithInitialVelocity(guide, velocity);

  ASSERT_TRUE(curve.valid);
  EXPECT_TRUE(curve.evaluate(0.0F).isApprox(start, 1e-5F));
  EXPECT_TRUE(curve.evaluate(1.0F).isApprox(guide.back(), 1e-4F));
  const Eigen::Vector3f physical_initial_velocity =
    curve.derivative(0.0F) * (velocity.norm() / curve.total_length);
  EXPECT_TRUE(physical_initial_velocity.isApprox(velocity, 1e-4F));
}

TEST(SemanticOpportunity, RequiresTwoConsistentWorldDirectionFrames)
{
  const std::vector<Eigen::Vector3f> candidates{
    point(10.0F, -10.0F), point(10.0F, -5.0F), point(10.0F),
    point(10.0F, 5.0F), point(10.0F, 10.0F)};
  const std::vector<float> scores{0.10F, 0.30F, 0.90F, 0.50F, 0.40F};
  const std::vector<std::uint8_t> virtual_flags(5, 1U);
  const std::vector<std::int8_t> columns{0, 1, 2, 3, 4};
  const auto observation = scalenav_graph::evaluateSemanticOpportunity(
    candidates, scores, virtual_flags, columns, point(0.0F), point(20.0F),
    45.0F, 0.08F);

  ASSERT_TRUE(observation.valid);
  EXPECT_EQ(observation.best_column, 0);
  EXPECT_EQ(observation.route_column, 2);
  EXPECT_GT(observation.improvement_m, 3.0F);
  Eigen::Vector3f pending = Eigen::Vector3f::Zero();
  int frames = 0;
  EXPECT_FALSE(scalenav_graph::updateSemanticOpportunityPersistence(
    observation, 3.0F, 0.85F, 2, pending, frames));
  EXPECT_EQ(frames, 1);
  EXPECT_TRUE(scalenav_graph::updateSemanticOpportunityPersistence(
    observation, 3.0F, 0.85F, 2, pending, frames));
  EXPECT_EQ(frames, 2);
}

TEST(SemanticOpportunity, CameraColumnChangesStillAccumulateByWorldDirection)
{
  scalenav_graph::SemanticOpportunity first;
  first.valid = true;
  first.best_column = 0;
  first.route_column = 2;
  first.improvement_m = 10.0F;
  first.best_world_direction = point(1.0F, 0.1F).normalized();
  auto second = first;
  second.best_column = 1;
  second.best_world_direction = point(1.0F, 0.12F).normalized();

  Eigen::Vector3f pending = Eigen::Vector3f::Zero();
  int frames = 0;
  EXPECT_FALSE(scalenav_graph::updateSemanticOpportunityPersistence(
    first, 3.0F, 0.85F, 2, pending, frames));
  EXPECT_TRUE(scalenav_graph::updateSemanticOpportunityPersistence(
    second, 3.0F, 0.85F, 2, pending, frames));
}

TEST(SemanticOpportunity, AlternatingDirectionsAndSmallBenefitsDoNotTrigger)
{
  scalenav_graph::SemanticOpportunity left;
  left.valid = true;
  left.best_column = 0;
  left.route_column = 2;
  left.improvement_m = 10.0F;
  left.best_world_direction = point(1.0F, -1.0F).normalized();
  auto right = left;
  right.best_column = 4;
  right.best_world_direction = point(1.0F, 1.0F).normalized();

  Eigen::Vector3f pending = Eigen::Vector3f::Zero();
  int frames = 0;
  EXPECT_FALSE(scalenav_graph::updateSemanticOpportunityPersistence(
    left, 3.0F, 0.85F, 2, pending, frames));
  EXPECT_FALSE(scalenav_graph::updateSemanticOpportunityPersistence(
    right, 3.0F, 0.85F, 2, pending, frames));
  EXPECT_EQ(frames, 1);
  right.improvement_m = 2.9F;
  EXPECT_FALSE(scalenav_graph::updateSemanticOpportunityPersistence(
    right, 3.0F, 0.85F, 2, pending, frames));
  EXPECT_EQ(frames, 0);
}

}  // namespace
