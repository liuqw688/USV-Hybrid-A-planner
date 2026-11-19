/**
 * @file test_clockwise_route.cpp
 * @brief 验证顺时针闭合航线及双向靠右航道生成。
 */

#include "clockwise_route.hpp"

#include <gtest/gtest.h>
#include <usv_map/parser.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using usv_map_tools::make_clockwise_route;
using usv_map_tools::make_safe_clockwise_route;
using usv_map_tools::RoutePoint;
using usv_map_tools::RoutePose;
constexpr double kPi = 3.14159265358979323846;

void expect_clockwise_loop(const std::vector<RoutePose> & route, double spacing)
{
  ASSERT_GE(route.size(), 9u);
  EXPECT_DOUBLE_EQ(route.front().position.x, route.back().position.x);
  EXPECT_DOUBLE_EQ(route.front().position.y, route.back().position.y);
  EXPECT_DOUBLE_EQ(route.front().yaw, route.back().yaw);
  const std::size_t count = route.size() - 1;
  double area = 0.0;
  double heading_change = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto & a = route[i];
    const auto & b = route[(i + 1) % count];
    const auto & c = route[(i + 2) % count];
    const double dx = b.position.x - a.position.x;
    const double dy = b.position.y - a.position.y;
    const double length = std::hypot(dx, dy);
    EXPECT_TRUE(std::isfinite(a.yaw));
    EXPECT_GT(length, 0.0);
    EXPECT_LE(length, spacing * 1.01);
    EXPECT_GE(length, spacing * 0.95);
    // 位姿沿路线切向；弦转角和切向变化均应指向右转。
    EXPECT_GT((dx * std::cos(a.yaw) + dy * std::sin(a.yaw)) / length, 0.98);
    EXPECT_LE(dx * (c.position.y - b.position.y) - dy * (c.position.x - b.position.x), 1e-8);
    const double yaw_delta = std::remainder(b.yaw - a.yaw, 2.0 * kPi);
    EXPECT_LE(yaw_delta, 1e-9);
    heading_change += yaw_delta;
    area += a.position.x * b.position.y - b.position.x * a.position.y;
  }
  EXPECT_LT(area, 0.0);
  EXPECT_NEAR(heading_change, -2.0 * kPi, 1e-8);
}

TEST(ClockwiseRoute, SmoothClosedLoopWithTangentOrientationAndUniformSpacing)
{
  const std::vector<RoutePoint> points{{-100, -100}, {100, -100}, {100, 100}, {-100, 100}};
  const auto route = make_clockwise_route(points, 2.0);
  expect_clockwise_loop(route, 2.0);
  for (const auto & pose : route) {
    EXPECT_LT(std::abs(pose.position.x), 100.0);
    EXPECT_LT(std::abs(pose.position.y), 100.0);
  }
}

TEST(ClockwiseRoute, RemovesConcaveWigglesAndDoesNotDependOnStopsOrRecordingDirection)
{
  std::vector<RoutePoint> points{{20, 10}};
  for (int i = 0; i < 160; ++i) {
    const double angle = static_cast<double>(i) * 2.0 * kPi / 160.0;
    points.push_back({25 + 100 * std::cos(angle), -40 + 200 * std::sin(angle)});
    // 停留点和凹形绕行不应改变凸包或引入反向曲率。
    points.push_back({25 + 40 * std::cos(3 * angle), -40 + 80 * std::sin(2 * angle)});
  }
  const auto route = make_clockwise_route(points, 2.0);
  expect_clockwise_loop(route, 2.0);
  points.insert(points.end(), 500, RoutePoint{25, -40});
  std::reverse(points.begin() + 1, points.end());
  const auto reversed = make_clockwise_route(points, 2.0);
  ASSERT_EQ(route.size(), reversed.size());
  for (std::size_t i = 0; i < route.size(); ++i) {
    EXPECT_DOUBLE_EQ(route[i].position.x, reversed[i].position.x);
    EXPECT_DOUBLE_EQ(route[i].position.y, reversed[i].position.y);
    EXPECT_DOUBLE_EQ(route[i].yaw, reversed[i].yaw);
  }
  for (const auto & pose : route) {
    const double x = (pose.position.x - 25) / 100;
    const double y = (pose.position.y + 40) / 200;
    EXPECT_LE(x * x + y * y, 1.0 + 1e-9);
  }
  const auto distance_to_start = [](const auto & pose) {
      return std::hypot(pose.position.x - 20, pose.position.y - 10);
    };
  for (const auto & pose : route) {
    EXPECT_GE(distance_to_start(pose), distance_to_start(route.front()));
  }
}

TEST(ClockwiseRoute, ShapingMovesEastSideInwardAndRoundsTheBottom)
{
  usv_map::Chart chart;
  usv_map::ChartFeature water;
  water.group = "DEPARE";
  water.geometry.push_back(usv_map::parse_wkt(
    "POLYGON ((-1000 -1000,1000 -1000,1000 1000,-1000 1000,-1000 -1000))"));
  chart.features.push_back(water);
  std::vector<RoutePoint> points;
  for (int i = 0; i < 120; ++i) {
    const double angle = 2.0 * kPi * i / 120.0;
    points.push_back({100.0 * std::cos(angle), 200.0 * std::sin(angle)});
  }

  const auto base = make_safe_clockwise_route(points, 2.0, chart, 15.0);
  const auto shaped = make_safe_clockwise_route(points, 2.0, chart, 15.0, 8.0, 20.0);
  ASSERT_EQ(base.controls.size(), shaped.controls.size());
  const auto east = std::max_element(base.controls.begin(), base.controls.end(),
    [](auto a, auto b) {return a.x < b.x; });
  const auto south = std::min_element(base.controls.begin(), base.controls.end(),
    [](auto a, auto b) {return a.y < b.y; });
  const auto east_index = static_cast<std::size_t>(east - base.controls.begin());
  const auto south_index = static_cast<std::size_t>(south - base.controls.begin());
  EXPECT_LT(shaped.controls[east_index].x, base.controls[east_index].x);
  EXPECT_GT(shaped.controls[south_index].y, base.controls[south_index].y);
  EXPECT_LT(shaped.controls[east_index].x, 100.0);
  EXPECT_GT(shaped.controls[south_index].y, -200.0);
  expect_clockwise_loop(shaped.poses, 2.0);
}

TEST(ClockwiseRoute, RejectsDegenerateReferencesAndInvalidOrExcessiveSampling)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<RoutePoint> triangle{{0, 0}, {100, 0}, {0, 100}};
  for (const double spacing : {0.0, -1.0, nan, 1e-9}) {
    EXPECT_THROW(make_clockwise_route(triangle, spacing), std::invalid_argument);
  }
  EXPECT_THROW(make_clockwise_route({}, 2.0), std::invalid_argument);
  EXPECT_THROW(make_clockwise_route({{0, 0}, {1, 0}}, 2.0), std::invalid_argument);
  EXPECT_THROW(make_clockwise_route({{0, 0}, {0, 0}, {0, 0}}, 2.0), std::invalid_argument);
  EXPECT_THROW(make_clockwise_route({{0, 0}, {1, 1}, {2, 2}}, 2.0), std::invalid_argument);
  EXPECT_THROW(make_clockwise_route({{0, 0}, {1, 0}, {nan, 1}}, 2.0), std::invalid_argument);
  expect_clockwise_loop(make_clockwise_route(triangle, 2.0), 2.0);
}
}  // namespace
