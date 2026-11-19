/**
 * @file bspline_reference_test.cpp
 * @brief USV B 样条到 NMPC 参考点的接口测试。
 */

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "usv_nmpc_controller/bspline_reference.hpp"

namespace
{
usv_interfaces::msg::UsvBspline makeStraightBspline()
{
  constexpr int degree = 3;
  constexpr int point_count = 4;
  usv_interfaces::msg::UsvBspline message;
  message.order = degree;
  message.traj_id = 42;
  const std::vector<double> control_x{0.0, 0.0, 5.0, 5.0};
  for (int i = 0; i < point_count; ++i) {
    usv_interfaces::msg::UsvBsplineControlPoint point;
    point.x = control_x[static_cast<std::size_t>(i)];
    point.phi = -3.14159265358979323846 / 2.0;
    message.control_points.push_back(point);
  }
  for (int i = 0; i < point_count + degree + 1; ++i) {
    message.knots.push_back(i < degree + 1 ? 0.0 : 4.0);
  }
  return message;
}

usv_interfaces::msg::UsvBspline makeTwoSegmentBspline()
{
  constexpr double pi = 3.14159265358979323846;
  const std::vector<std::pair<double, double>> xy{
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
    {3.0, 1.0}, {3.0, 2.0}, {3.0, 3.0}};
  const std::vector<double> heading_deg{179.0, 179.5, 180.0, -179.5, -179.0, -178.5, -178.0};
  usv_interfaces::msg::UsvBspline message;
  message.order = 3;
  for (std::size_t i = 0; i < xy.size(); ++i) {
    usv_interfaces::msg::UsvBsplineControlPoint point;
    point.x = xy[i].first;
    point.y = xy[i].second;
    point.phi = heading_deg[i] * pi / 180.0;
    message.control_points.push_back(point);
  }
  message.knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  return message;
}
}  // namespace

TEST(BsplineReferenceTest, TimeSplinePreservesItsBuiltInBrakingProfile)
{
  const auto message = makeStraightBspline();
  usv_nmpc::ReferenceTrajectory trajectory;
  std::string error;

  ASSERT_TRUE(usv_nmpc::buildReferenceFromBspline(
      message, 0.25, trajectory, error)) << error;
  ASSERT_GT(trajectory.points.size(), 10U);
  EXPECT_NEAR(trajectory.points.front().x, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.front().y, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(trajectory.points.front().speed, 0.0);
  EXPECT_NEAR(trajectory.points[4].t, 1.0, 1e-9);
  EXPECT_NEAR(trajectory.points[4].x, 0.78125, 1e-9);
  EXPECT_NEAR(trajectory.points[4].speed, 1.40625, 1e-9);
  EXPECT_NEAR(trajectory.points.back().x, 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(trajectory.points.back().speed, 0.0);
}

TEST(BsplineReferenceTest, ConstantYawControlPointsSetReferenceHeading)
{
  auto message = makeStraightBspline();
  for (auto & point : message.control_points) {
    point.phi = -3.14159265358979323846 / 2.0;
  }
  usv_nmpc::ReferenceTrajectory trajectory;
  std::string error;

  ASSERT_TRUE(usv_nmpc::buildReferenceFromBspline(
      message, 0.5, trajectory, error)) << error;
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.psi, -3.14159265358979323846 / 2.0, 1e-9);
  }
}

TEST(BsplineReferenceTest, PositionAndHeadingDerivativesProduceSurgeAndSway)
{
  auto message = makeStraightBspline();
  for (auto & point : message.control_points) {
    point.phi = -3.14159265358979323846 / 4.0;
  }
  usv_nmpc::ReferenceTrajectory trajectory;
  std::string error;

  ASSERT_TRUE(usv_nmpc::buildReferenceFromBspline(
      message, 0.5, trajectory, error)) << error;
  ASSERT_GT(trajectory.points.size(), 2U);
  const auto & moving = trajectory.points[2];
  EXPECT_NEAR(moving.surge_ratio, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(moving.sway_ratio, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(moving.course, -3.14159265358979323846 / 2.0, 1e-9);
}

TEST(BsplineReferenceTest, RejectsMalformedKnotVector)
{
  auto message = makeStraightBspline();
  message.knots[4] = message.knots[3] - 1.0;
  usv_nmpc::ReferenceTrajectory trajectory;
  std::string error;

  EXPECT_FALSE(usv_nmpc::buildReferenceFromBspline(
      message, 0.1, trajectory, error));
  EXPECT_FALSE(error.empty());
}

TEST(BsplineReferenceTest, EvaluatesRollingBezierSliceAcrossRepeatedKnotsAndYawWrap)
{
  usv_nmpc::BsplineReference spline;
  std::string error;
  ASSERT_TRUE(spline.configure(makeTwoSegmentBspline(), error)) << error;

  usv_nmpc::RefPoint before, after;
  ASSERT_TRUE(spline.sample(0.99, before));
  ASSERT_TRUE(spline.sample(1.01, after));
  EXPECT_TRUE(std::isfinite(before.x));
  EXPECT_TRUE(std::isfinite(before.y));
  EXPECT_TRUE(std::isfinite(before.u));
  EXPECT_TRUE(std::isfinite(before.r));
  EXPECT_LT(std::abs(std::remainder(after.psi - before.psi, 2.0 * std::acos(-1.0))),
    2.0 * std::acos(-1.0) / 180.0);
  EXPECT_NEAR(before.x, 3.0, 0.04);
  EXPECT_NEAR(after.y, 0.03, 0.04);
}

TEST(BsplineReferenceTest, RejectsNonFinitePublishedControlPoint)
{
  auto message = makeStraightBspline();
  message.control_points[2].phi = std::numeric_limits<double>::quiet_NaN();
  usv_nmpc::BsplineReference spline;
  std::string error;

  EXPECT_FALSE(spline.configure(message, error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(spline.valid());
}

TEST(BsplineReferenceTest, HorizonUsesAbsoluteSplineTimeAndDerivativeVelocity)
{
  const auto message = makeStraightBspline();
  usv_nmpc::BsplineReference spline;
  std::string error;
  ASSERT_TRUE(spline.configure(message, error)) << error;

  std::vector<usv_nmpc::RefPoint> references;
  ASSERT_TRUE(spline.buildHorizon(2.0, 4, 0.5, references));
  ASSERT_EQ(references.size(), 5U);
  EXPECT_NEAR(references.front().x, 2.5, 1e-9);
  EXPECT_NEAR(references.front().u, 1.875, 1e-9);
  EXPECT_NEAR(references[2].x, 4.21875, 1e-9);
  EXPECT_NEAR(references[2].u, 1.40625, 1e-9);
  EXPECT_NEAR(references.back().x, 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(references.back().u, 0.0);
}

TEST(BsplineReferenceTest, AcceptsNonzeroEndpointSpeedWithoutExtraMessageFlag)
{
  auto message = makeStraightBspline();
  message.control_points.back().x += 1.0;
  usv_nmpc::BsplineReference spline;
  std::string error;

  ASSERT_TRUE(spline.configure(message, error)) << error;
  usv_nmpc::RefPoint endpoint;
  ASSERT_TRUE(spline.sample(spline.duration(), endpoint));
  EXPECT_NEAR(endpoint.u, 0.75, 1e-9);
}

TEST(BsplineReferenceTest, LocalSegmentMayEndAtNonzeroSpeedAndExtrapolates)
{
  auto message = makeStraightBspline();
  message.control_points.back().x += 1.0;
  usv_nmpc::BsplineReference spline;
  std::string error;
  ASSERT_TRUE(spline.configure(message, error)) << error;

  std::vector<usv_nmpc::RefPoint> references;
  ASSERT_TRUE(spline.buildHorizon(4.0, 2, 1.0, references));
  ASSERT_EQ(references.size(), 3U);
  EXPECT_NEAR(references[0].x, 6.0, 1e-9);
  EXPECT_NEAR(references[0].u, 0.75, 1e-9);
  EXPECT_NEAR(references[1].x, 6.75, 1e-9);
  EXPECT_NEAR(references[2].x, 7.5, 1e-9);
  EXPECT_NEAR(references[2].u, 0.75, 1e-9);

  usv_nmpc::ReferenceTrajectory trajectory;
  ASSERT_TRUE(spline.buildTrajectory(0.3, trajectory, error)) << error;
  EXPECT_NEAR(trajectory.points.back().speed, 0.75, 1e-9);
  EXPECT_FALSE(trajectory.terminal);
}
