/**
 * @file test_route_clearance.cpp
 * @brief 验证路线及环形航道的水域覆盖和最小净距。
 */

#include "clockwise_route.hpp"
#include "route_water.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using namespace usv_map_tools;
constexpr double kPi = 3.14159265358979323846;

void add(usv_map::Chart & chart, const char * group, const char * wkt)
{
  usv_map::ChartFeature feature;
  feature.group = group;
  feature.geometry.push_back(usv_map::parse_wkt(wkt));
  chart.features.push_back(feature);
}

std::vector<RoutePoint> reference()
{
  std::vector<RoutePoint> points;
  for (int i = 0; i < 100; ++i) {
    const double angle = 2 * kPi * i / 100;
    points.push_back({100 * std::cos(angle), 200 * std::sin(angle)});
  }
  return points;
}

TEST(RouteClearance, FitsBetweenBanksAndKeepsIslandInsideClockwiseLoop)
{
  usv_map::Chart chart;
  add(chart, "DEPARE", "POLYGON ((-85 -250, 85 -250, 85 250, -85 250, -85 -250))");
  add(chart, "LNDARE", "POLYGON ((-15 -20, 15 -20, 15 20, -15 20, -15 -20))");
  const auto route = make_safe_clockwise_route(reference(), 2.0, chart, 10.0);
  ASSERT_GT(route.poses.size(), 100u);
  EXPECT_GE(route.minimum_clearance_m, 10.0);
  double turning = 0.0;
  double area = 0.0;
  for (std::size_t i = 1; i < route.poses.size(); ++i) {
    const auto a = route.poses[i - 1];
    const auto b = route.poses[i];
    // 用独立解析边界校验岸线净距；连接采样点的直线段也位于该条带内。
    EXPECT_LE(std::abs(a.position.x), 75.0);
    EXPECT_LE(std::abs(a.position.y), 240.0);
    // 此处检查点到矩形的距离；连续线段碰撞由后续用例覆盖。
    const double dx = std::max(0.0, std::abs(a.position.x) - 15.0);
    const double dy = std::max(0.0, std::abs(a.position.y) - 20.0);
    EXPECT_GE(std::hypot(dx, dy), 10.0);
    const double turn = std::remainder(b.yaw - a.yaw, 2 * kPi);
    EXPECT_LE(turn, 1e-9);
    turning += turn;
    area += a.position.x * b.position.y - b.position.x * a.position.y;
  }
  EXPECT_LT(area, 0.0);
  EXPECT_NEAR(turning, -2 * kPi, 1e-8);
  EXPECT_DOUBLE_EQ(route.poses.front().position.x, route.poses.back().position.x);
  EXPECT_DOUBLE_EQ(route.poses.front().yaw, route.poses.back().yaw);
  // 正仿射缩放后，路线仍包围中心岛屿。
  double winding = 0.0;
  for (std::size_t i = 1; i < route.poses.size(); ++i) {
    const auto a = route.poses[i - 1].position;
    const auto b = route.poses[i].position;
    winding += std::remainder(std::atan2(b.y, b.x) - std::atan2(a.y, a.x), 2 * kPi);
  }
  EXPECT_NEAR(winding, -2 * kPi, 1e-8);
}

TEST(RouteClearance, DetectsThinObstacleBetweenSamplesAndOnClosingSegment)
{
  usv_map::Chart chart;
  add(chart, "DEPARE", "POLYGON ((-200 -200, 200 -200, 200 200, -200 200, -200 -200))");
  add(chart, "LNDARE", "POLYGON ((-0.05 -1, 0.05 -1, 0.05 1, -0.05 1, -0.05 -1))");
  const RouteWater water(chart, 10.0);
  EXPECT_GE(water.clearance({{-50, 0}, {-50, 50}}), 10.0);
  EXPECT_GE(water.clearance({{50, 0}, {50, 50}}), 10.0);
  // 两端点均满足净距，但连接线段穿过陆地。
  EXPECT_LT(water.clearance({{-50, 0}, {50, 0}}), 0.0);
  EXPECT_GT(water.unsafe_length({{-50, 0}, {50, 0}}), 0.0);
  // 仅闭合边与障碍碰撞，用于覆盖首尾连接的检查。
  EXPECT_LT(water.clearance({{-50, 0}, {-50, 50}, {50, 50}, {50, 0}, {-50, 0}}), 0.0);
}

TEST(RouteClearance, UnionsWaterWithoutArtificialSeamsAndHonorsLandPriority)
{
  usv_map::Chart chart;
  add(chart, "DEPARE", "POLYGON ((-100 -100, 10 -100, 10 100, -100 100, -100 -100))");
  add(chart, "DEPARE", "POLYGON ((-10 -100, 100 -100, 100 100, -10 100, -10 -100))");
  EXPECT_NEAR(RouteWater(chart, 10).clearance({{-50, 0}, {50, 0}}), 50.0, 1e-9);
  add(chart, "lndare", "POLYGON ((-2 -2, 2 -2, 2 2, -2 2, -2 -2))");
  EXPECT_LT(RouteWater(chart, 10).clearance({{-50, 0}, {50, 0}}), 0.0);
}

TEST(RouteClearance, PreservesWaterHolesAndSeparateMultiPolygonExteriors)
{
  usv_map::Chart chart;
  add(chart, "DEPARE", "MULTIPOLYGON ("
    "((-100 -100, 100 -100, 100 100, -100 100, -100 -100), "
    "(-10 -10, 10 -10, 10 10, -10 10, -10 -10)), "
    "((200 -100, 400 -100, 400 100, 200 100, 200 -100)))");
  const RouteWater water(chart, 10);
  EXPECT_LT(water.clearance({{-5, 0}, {5, 0}}), 0.0);
  EXPECT_GE(water.clearance({{250, 0}, {350, 0}}), 10.0);
  EXPECT_LT(water.clearance({{150, -5}, {150, 5}}), 0.0);
  EXPECT_NEAR(water.clearance({{25, -5}, {25, 5}}), 15.0, 1e-9);
}

TEST(RouteClearance, RepairsSelfTouchingContoursAndKeepsCollapsedLandEdgesBlocked)
{
  usv_map::Chart chart;
  add(chart, "DEPARE", "POLYGON ((-100 -100, 100 -100, 100 100, -100 100, -100 -100))");
  // 零面积轮廓经 MakeValid 修复后仍应作为线障碍参与检查。
  add(chart, "LNDARE", "POLYGON ((0 -50, 0 50, 0 0, 0 -50))");
  const RouteWater water(chart, 10);
  EXPECT_LT(water.clearance({{-50, 0}, {50, 0}}), 0.0);
  EXPECT_NEAR(water.clearance({{20, -10}, {20, 10}}), 20.0, 1e-9);
}

TEST(RouteClearance, RefusesMissingInvalidOrInfeasibleMapInsteadOfPublishingUnsafeLoop)
{
  const auto points = reference();
  EXPECT_THROW(make_safe_clockwise_route(points, 2.0, {}, 10.0), std::runtime_error);
  usv_map::Chart narrow;
  add(narrow, "DEPARE", "POLYGON ((-5 -200, 5 -200, 5 200, -5 200, -5 -200))");
  EXPECT_THROW(make_safe_clockwise_route(points, 2.0, narrow, 10.0), std::runtime_error);
  usv_map::Chart small;
  add(small, "DEPARE", "POLYGON ((-40 -40, 40 -40, 40 40, -40 40, -40 -40))");
  EXPECT_THROW(make_safe_clockwise_route(points, 2.0, small, 10.0), std::runtime_error);
  small.has_unreadable_navigation_geometry = true;
  EXPECT_THROW(RouteWater(small, 10.0), std::runtime_error);
  for (double margin : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(make_safe_clockwise_route(points, 2.0, small, margin), std::invalid_argument);
  }
}
}  // namespace
