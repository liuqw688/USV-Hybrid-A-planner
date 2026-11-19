/**
 * @file test_waypoint_route.cpp
 * @brief 验证航点密度无关的前视、单调投影、闭环及原生导航航向。
 */
#include <usv_planner/waypoint_route.hpp>
#include <gtest/gtest.h>
#include <limits>

using usv_planner::WaypointRoute;

TEST(WaypointRoute, DenseAndSparseRoutesHaveSameFrontier)
{
  WaypointRoute dense, sparse;
  std::vector<Eigen::Vector2d> points;
  for (int y = 0; y <= 200; y += 5) points.emplace_back(0, y);
  dense.reset(points);
  sparse.reset({{0, 0}, {0, 200}});
  for (auto* route : {&dense, &sparse}) {
    route->update({0, 0});
    EXPECT_TRUE(route->target({0, 0}, 60).pose.isApprox(Eigen::Vector3d(0, 60, 0)));
    route->update({0, 12});
    EXPECT_NEAR(route->progress(), 12, 1e-9);
    EXPECT_NEAR(route->target({0, 12}, 60).pose.y(), 72, 1e-9);
    EXPECT_FALSE(route->target({0, 12}, 60).terminal);
  }
}

TEST(WaypointRoute, FrontierUsesConfiguredRange)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 200}});
  route.update({0, 0});
  EXPECT_NEAR(route.target({0, 0}, 35).pose.y(), 35, 1e-9);
  EXPECT_NEAR(route.target({0, 0}, 80).pose.y(), 80, 1e-9);
}

TEST(WaypointRoute, ClosedLoopDoesNotFinishAtItsStart)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 100}, {100, 100}, {100, 0}, {0, 0}});
  route.update({0, 0});
  EXPECT_FALSE(route.target({0, 0}, 60).terminal);
  for (const Eigen::Vector2d& position : std::vector<Eigen::Vector2d>{
    {0, 50}, {0, 100}, {50, 100}, {100, 100}, {100, 50}, {100, 0}, {50, 0}}) {
    route.update(position);
  }
  EXPECT_NEAR(route.progress(), 350, 1e-9);
  EXPECT_TRUE(route.target({50, 0}, 60).terminal);
  EXPECT_TRUE(route.target({50, 0}, 60).pose.head<2>().isZero());
}

TEST(WaypointRoute, CrossingAndReverseMotionDoNotJumpProgress)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 100}, {100, 100}, {100, 0}, {0, 0}, {0, 200}});
  route.update({0, 0});
  route.update({0, 10});
  EXPECT_NEAR(route.progress(), 10, 1e-9);
  route.update({0, 5});
  EXPECT_NEAR(route.progress(), 10, 1e-9);
  route.update({0, 5});
  EXPECT_NEAR(route.progress(), 10, 1e-9);
}

TEST(WaypointRoute, EntryLegStartsFromVessel)
{
  WaypointRoute route;
  route.reset({{0, 30}, {0, 200}});
  route.update({0, 0});
  EXPECT_NEAR(route.target({0, 0}, 60).pose.y(), 60, 1e-9);
  EXPECT_NEAR(route.progress(), 0, 1e-9);
}

TEST(WaypointRoute, OffsetAndCornersKeepTargetInPerceptionCircle)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 40}, {100, 40}});
  route.update({0, 0});
  route.update({10, 20});
  const Eigen::Vector2d position(10, 20);
  const auto target = route.target(position, 60);
  EXPECT_LE((target.pose.head<2>() - position).norm(), 60 + 1e-9);
  EXPECT_NEAR(target.pose.x(), 30, 1e-9);
  EXPECT_FALSE(target.terminal);
  const Eigen::Vector2d distant(200, -200);
  EXPECT_NEAR((route.target(distant, 60).pose.head<2>() - distant).norm(), 60, 1e-9);
}

TEST(WaypointRoute, HeadingIsNorthZeroEastNegativeWestPositive)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 40}, {40, 40}, {0, 40}});
  route.update({0, 0});
  EXPECT_NEAR(route.target({0, 0}, 10).pose.z(), 0, 1e-9);
  EXPECT_NEAR(route.target({0, 0}, 60).pose.z(), -std::acos(-1.0) / 2, 1e-9);
  route.update({0, 40});
  route.update({40, 40});
  EXPECT_NEAR(route.target({40, 40}, 60).pose.z(), std::acos(-1.0) / 2, 1e-9);
}

TEST(WaypointRoute, OnlyWholeRouteEndIsTerminal)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 10}, {0, 100}});
  route.update({0, 0});
  EXPECT_FALSE(route.target({0, 0}, 60).terminal);
  route.update({0, 45});
  const auto target = route.target({0, 45}, 60);
  EXPECT_TRUE(target.terminal);
  EXPECT_NEAR(target.pose.y(), 100, 1e-9);
}

TEST(WaypointRoute, RejectsInvalidInputAndPreservesNonAdjacentRepeats)
{
  WaypointRoute route;
  EXPECT_THROW(route.target({0, 0}, 60), std::invalid_argument);
  EXPECT_THROW(route.reset({{0, 0}, {0, 0}}), std::invalid_argument);
  EXPECT_THROW(route.reset({{0, 0}, {0, std::numeric_limits<double>::quiet_NaN()}}),
    std::invalid_argument);
  route.reset({{0, 0}, {0, 0}, {0, 100}, {0, 0}});
  route.update({0, 0});
  EXPECT_FALSE(route.target({0, 0}, 60).terminal);
  EXPECT_THROW(route.target({0, 0}, 0), std::invalid_argument);
  route.reset({{0, 0}, {100, 0}});
  EXPECT_THROW(route.target({0, 0}, 60), std::invalid_argument);
}

TEST(WaypointRoute, BlockedFrontierSearchesFromTenMetresBeforeObstacle)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 60}, {0, 150}});
  route.update({0, 0});
  const auto safe = [](const Eigen::Vector3d& pose) {return pose.y() < 55 || pose.y() > 65;};
  const auto target = route.safeTarget({0, 0}, 60, safe);
  ASSERT_TRUE(target);
  EXPECT_NEAR(target->pose.y(), 44.5, 1e-9);
  EXPECT_TRUE(safe(target->pose));
  EXPECT_FALSE(target->terminal);
  EXPECT_DOUBLE_EQ(route.progress(), 0.0);

  route.update({0, 15});
  const auto after_obstacle = route.safeTarget({0, 15}, 60, safe);
  ASSERT_TRUE(after_obstacle);
  EXPECT_NEAR(after_obstacle->pose.y(), 75, 1e-9);
  EXPECT_FALSE(after_obstacle->terminal);
}

TEST(WaypointRoute, BlockedIntermediateWaypointsDoNotPreventBypass)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 30}, {0, 100}});
  route.update({0, 0});
  const auto target = route.safeTarget({0, 0}, 60,
    [](const Eigen::Vector3d& pose) {return pose.y() < 25 || pose.y() > 35;});
  ASSERT_TRUE(target);
  EXPECT_NEAR(target->pose.y(), 60, 1e-9);
  EXPECT_FALSE(target->terminal);
}

TEST(WaypointRoute, ForwardSearchSkipsBlockedSearchStart)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 150}});
  route.update({0, 0});
  const auto safe = [](const Eigen::Vector3d& pose) {
    return (pose.y() < 40 || pose.y() > 48) && pose.y() < 55;
  };
  const auto target = route.safeTarget({0, 0}, 60, safe);
  ASSERT_TRUE(target);
  EXPECT_NEAR(target->pose.y(), 48.5, 1e-9);
  EXPECT_TRUE(safe(target->pose));
}

TEST(WaypointRoute, BlockedMissionEndDoesNotBecomeAnotherTerminalGoal)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 40}});
  route.update({0, 0});
  const auto target = route.safeTarget({0, 0}, 60,
    [](const Eigen::Vector3d& pose) {return pose.y() < 35;});
  ASSERT_TRUE(target);
  EXPECT_FALSE(target->terminal);
  EXPECT_NEAR(target->pose.y(), 24.5, 1e-9);
  const auto cleared = route.safeTarget({0, 0}, 60, [](const Eigen::Vector3d&) {return true;});
  ASSERT_TRUE(cleared);
  EXPECT_TRUE(cleared->terminal);
  EXPECT_NEAR(cleared->pose.y(), 40, 1e-9);
}

TEST(WaypointRoute, NoSafeForwardTargetReturnsEmpty)
{
  WaypointRoute route;
  route.reset({{0, 0}, {0, 100}});
  route.update({0, 0});
  EXPECT_FALSE(route.safeTarget({0, 0}, 60,
    [](const Eigen::Vector3d& pose) {return pose.y() <= 0;}));
  EXPECT_FALSE(route.safeTarget({0, 0}, 60, [](const Eigen::Vector3d&) {return false;}));
  EXPECT_DOUBLE_EQ(route.progress(), 0.0);
  EXPECT_THROW(route.safeTarget({0, 0}, 60, [](const Eigen::Vector3d&) {return true;}, 0),
    std::invalid_argument);
}

TEST(WaypointRoute, SafeSearchPreservesRangeAndNativeHeading)
{
  WaypointRoute route;
  route.reset({{0, 0}, {100, 0}});
  route.update({0, 0});
  const Eigen::Vector2d position(0, 10);
  const auto target = route.safeTarget(position, 60,
    [](const Eigen::Vector3d& pose) {return pose.x() < 45;});
  ASSERT_TRUE(target);
  EXPECT_LE((target->pose.head<2>() - position).norm(), 60);
  EXPECT_NEAR(target->pose.z(), -std::acos(-1.0) / 2, 1e-9);
  EXPECT_FALSE(target->terminal);
}
