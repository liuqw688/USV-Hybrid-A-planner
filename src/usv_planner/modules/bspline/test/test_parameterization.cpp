#include <bspline/non_uniform_bspline.h>
#include <gtest/gtest.h>
#include <cmath>

// 比较三个右端项同时求解与原逐列求解，包括跨越 ±π 的航向。
TEST(BsplineParameterization, MatchesIndependentSolves)
{
  const double dt = 0.7;
  const std::vector<Eigen::Vector3d> points = {
      {0.0, 0.0, 3.0}, {0.4, 1.0, 3.12}, {1.0, 2.0, -3.10},
      {1.8, 3.0, -2.9}, {2.5, 4.0, -2.8}};
  const std::vector<Eigen::Vector3d> derivatives = {
      {0.2, 1.0, 0.1}, {0.6, 0.8, 0.1}, {0.1, 0.0, 0.0}, {0.0, -0.1, 0.0}};
  const int k = static_cast<int>(points.size());
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(k + 4, k + 2);
  const Eigen::RowVector3d pos(1, 4, 1), vel(-1, 0, 1), acc(1, -2, 1);
  for (int i = 0; i < k; ++i) a.block<1, 3>(i, i) = pos / 6.0;
  a.block<1, 3>(k, 0) = vel / (2 * dt);
  a.block<1, 3>(k + 1, k - 1) = vel / (2 * dt);
  a.block<1, 3>(k + 2, 0) = acc / (dt * dt);
  a.block<1, 3>(k + 3, k - 1) = acc / (dt * dt);
  Eigen::MatrixXd b(k + 4, 3);
  for (int i = 0; i < k; ++i) {
    b.row(i) = points[i].transpose();
    if (i) {
      double d = b(i, 2) - b(i - 1, 2);
      b(i, 2) = b(i - 1, 2) + std::atan2(std::sin(d), std::cos(d));
    }
  }
  for (int i = 0; i < 4; ++i) b.row(k + i) = derivatives[i].transpose();
  Eigen::MatrixXd expected(k + 2, 3);
  for (int axis = 0; axis < 3; ++axis)
    expected.col(axis) = a.colPivHouseholderQr().solve(b.col(axis));
  Eigen::MatrixXd actual;
  fast_planner::NonUniformBspline::parameterizeToBspline(dt, points, derivatives, actual);
  ASSERT_EQ(actual.rows(), k + 2);
  ASSERT_EQ(actual.cols(), 3);
  EXPECT_TRUE(actual.isApprox(expected, 1e-12));
}

TEST(BsplineParameterization, RejectsMissingDerivatives)
{
  Eigen::MatrixXd output;
  const std::vector<Eigen::Vector3d> points(2, Eigen::Vector3d::Zero());
  fast_planner::NonUniformBspline::parameterizeToBspline(1.0, points, {}, output);
  EXPECT_EQ(output.size(), 0);
}

TEST(BsplineParameterization, EnforcesZeroTerminalVelocityAndPreservesEndpoint)
{
  const double dt = 0.4;
  const std::vector<Eigen::Vector3d> points = {
      {0.0, 0.0, 0.0}, {0.5, 0.8, 0.1}, {1.2, 1.6, 0.2},
      {2.0, 2.2, 0.3}, {2.8, 2.5, 0.4}, {3.5, 2.6, 0.5}};
  const std::vector<Eigen::Vector3d> derivatives = {
      {0.4, 0.8, 0.1}, {0.7, 0.1, 0.05}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  std::vector<Eigen::Vector3d> zero_end_velocity_derivatives = derivatives;
  zero_end_velocity_derivatives[1].setZero();
  Eigen::MatrixXd unconstrained, stopped;
  fast_planner::NonUniformBspline::parameterizeToBspline(
      dt, points, zero_end_velocity_derivatives, unconstrained);
  fast_planner::NonUniformBspline::parameterizeToBspline(
      dt, points, zero_end_velocity_derivatives, stopped, true);

  fast_planner::NonUniformBspline before(unconstrained, 3, dt);
  fast_planner::NonUniformBspline after(stopped, 3, dt);
  const Eigen::VectorXd expected_end = before.evaluateDeBoorT(before.getTimeSum());
  const Eigen::VectorXd actual_end = after.evaluateDeBoorT(after.getTimeSum());
  const Eigen::VectorXd end_velocity =
      after.getDerivative().evaluateDeBoorT(after.getTimeSum());

  ASSERT_EQ(stopped.rows(), static_cast<Eigen::Index>(points.size() + 2));
  EXPECT_LT((actual_end - expected_end).norm(), 1e-10);
  EXPECT_LT(end_velocity.norm(), 1e-10);
}

TEST(BsplineParameterization, RestoresZeroTerminalVelocityWithNonUniformKnots)
{
  const double dt = 0.4;
  Eigen::MatrixXd points(8, 3);
  points << 0.0, 0.0, 0.0,
      0.4, 0.8, 0.1,
      0.9, 1.5, 0.2,
      1.5, 2.0, 0.3,
      2.2, 2.4, 0.4,
      3.0, 2.6, 0.5,
      3.8, 2.7, 0.6,
      4.5, 2.8, 0.7;
  ASSERT_TRUE(fast_planner::NonUniformBspline::enforceEndVelocity(
      points, dt, Eigen::Vector3d::Zero()));
  fast_planner::NonUniformBspline spline(points, 3, dt);
  ASSERT_LT(spline.getDerivative().evaluateDeBoorT(spline.getTimeSum()).norm(), 1e-10);
  Eigen::VectorXd knots = spline.getKnot();
  // 改变末端支撑区内的节点间隔，复现重分配时间后原零速约束失效。
  knots.tail(knots.size() - 8).array() += 0.13;
  spline.setKnot(knots);

  const double end_time = spline.getTimeSum();
  ASSERT_GT(spline.getDerivative().evaluateDeBoorT(end_time).norm(), 1e-5);
  const Eigen::VectorXd expected_end = spline.evaluateDeBoorT(end_time);
  ASSERT_TRUE(spline.enforceEndVelocity(Eigen::Vector3d::Zero()));

  const Eigen::VectorXd actual_end = spline.evaluateDeBoorT(end_time);
  const Eigen::VectorXd actual_velocity =
      spline.getDerivative().evaluateDeBoorT(end_time);
  EXPECT_LT((actual_end - expected_end).norm(), 1e-10);
  EXPECT_LT(actual_velocity.norm(), 1e-10);
}

TEST(BsplineParameterization, TimeAdjustmentPreservesMeasuredStartAndStoppedEnd)
{
  Eigen::MatrixXd points(12, 3);
  for (int i = 0; i < points.rows(); ++i)
    points.row(i) << 0.1 * i * i, 0.7 * i, 0.02 * i;
  fast_planner::NonUniformBspline spline(points, 3, 0.4);
  const Eigen::Vector3d position(3.0, -8.0, -0.5);
  const Eigen::Vector3d velocity(0.4, 1.6, 0.12);
  const Eigen::Vector3d acceleration(-0.1, -0.3, -0.02);
  ASSERT_TRUE(spline.enforceStartState(position, velocity, acceleration));
  for (int iteration = 0; iteration < 4; ++iteration) {
    Eigen::VectorXd knots = spline.getKnot();
    for (int i = 2; i < knots.size(); ++i) knots(i) += 0.04 * (i - 1);
    spline.setKnot(knots);
    ASSERT_GT((spline.getDerivative().evaluateDeBoorT(0.0) - velocity).norm(), 0.01);
    ASSERT_TRUE(spline.enforceStartState(position, velocity, acceleration));
    ASSERT_TRUE(spline.enforceEndVelocity(Eigen::Vector3d::Zero()));
    EXPECT_LT((spline.evaluateDeBoorT(0.0) - position).norm(), 1e-10);
    EXPECT_LT((spline.getDerivative().evaluateDeBoorT(0.0) - velocity).norm(), 1e-10);
    EXPECT_LT((spline.getDerivative().getDerivative().evaluateDeBoorT(0.0) - acceleration).norm(), 1e-10);
    EXPECT_LT(spline.getDerivative().evaluateDeBoorT(spline.getTimeSum()).norm(), 1e-10);
  }
  EXPECT_FALSE(spline.enforceStartState(position, velocity, Eigen::Vector3d::Constant(NAN)));
}
