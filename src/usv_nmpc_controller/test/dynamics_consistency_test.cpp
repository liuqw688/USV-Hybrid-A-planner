/**
 * @file dynamics_consistency_test.cpp
 * @brief 共享动力学一致性、航向坐标与控制变化率约束的回归测试。
 */

#include <array>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "usv_nmpc_controller/coordinate_conventions.hpp"
#include "usv_nmpc_controller/nmpc_controller.hpp"
#include "usv_nmpc_controller/usv_model.hpp"

namespace usv_nmpc
{

TEST(DynamicsConsistency, NumericAndSymbolicModelsMatch)
{
  const auto xs = casadi::SX::sym("x", 6);
  const auto us = casadi::SX::sym("u", 2);
  const casadi::Function dynamics("usv_dynamics", {xs, us},
    {USVModel::dynamicsSX(xs, us)});
  for (double speed : {-0.4, 0.0, 1.0}) {
    for (double yaw_rate : {-0.12, 0.0, 0.12}) {
      const State predicted{4.0, -2.0, 0.7, 0.6, 0.05, -0.03};
      const auto derivative = USVModel::derivative(predicted, {speed, yaw_rate});
      const auto symbolic = dynamics(std::vector<casadi::DM>{
          casadi::DM(std::vector<double>{predicted.x, predicted.y, predicted.psi,
            predicted.u, predicted.v, predicted.r}),
          casadi::DM(std::vector<double>{speed, yaw_rate})}).front().get_elements();
      const std::vector<double> expected{derivative.x, derivative.y,
        derivative.psi, derivative.u, derivative.v, derivative.r};
      for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR(symbolic[i], expected[i], 1e-12);
      }
    }
  }
}

TEST(DynamicsConsistency, Rk4UsesTheIdentifiedSurgeAndYawDynamics)
{
  const State state{0.0, 0.0, 0.0, 0.8, 0.1, -0.02};
  const Control command{0.6, 0.08};
  const double dt = 0.001;
  const auto derivative = USVModel::derivative(state, command);
  const auto next = USVModel::stepRK4(state, command, dt);

  EXPECT_NEAR(next.u, state.u + derivative.u * dt, 1e-7);
  EXPECT_NEAR(next.v, state.v + derivative.v * dt, 1e-7);
  EXPECT_NEAR(next.r, state.r + derivative.r * dt, 1e-7);
  EXPECT_NEAR(next.psi, state.psi + state.r * dt, 1e-7);
}

TEST(DynamicsConsistency, UsesSpecifiedNorthReferencedModelEquations)
{
  const State state{2.0, 3.0, 0.4, 0.8, 0.1, -0.02};
  const Control command{0.6, 0.08};
  const auto derivative = USVModel::derivative(state, command);
  EXPECT_NEAR(derivative.x, -0.8 * std::sin(0.4) + 0.1 * std::cos(0.4), 1e-12);
  EXPECT_NEAR(derivative.y, 0.8 * std::cos(0.4) + 0.1 * std::sin(0.4), 1e-12);
  EXPECT_DOUBLE_EQ(derivative.psi, -0.02);
  EXPECT_NEAR(derivative.u, -0.431930955 * 0.8 + 0.437133189 * 0.6, 1e-12);
  EXPECT_NEAR(derivative.v,
    -0.313324916 * 0.1 + 0.006741061 * -0.02 +
    0.407958500 * 0.08 + 0.000993619, 1e-12);
  EXPECT_NEAR(derivative.r,
    -0.166118168 * 0.1 - 0.544509453 * -0.02 +
    0.837536662 * 0.08 + 0.000626529, 1e-12);
}

TEST(DynamicsConsistency, CardinalRfuHeadingsAndLateralVelocityKeepTheirSigns)
{
  // 北、东、南、西：/odom 四元数与状态角直接一致，v 正向指向右舷。
  for (double ins_heading : {0.0, -PI / 2.0, PI, PI / 2.0}) {
    const double heading = coordinates::rfuOdometryHeading(
      0.0, 0.0, std::sin(ins_heading / 2.0), std::cos(ins_heading / 2.0));
    EXPECT_NEAR(wrapAngle(heading - ins_heading), 0.0, 1e-12);
    const State state{0.0, 0.0, heading, 1.0,
      coordinates::usvStateLateralVelocityToStarboard(0.2), 0.03};
    const auto derivative = USVModel::derivative(state, {});
    EXPECT_NEAR(derivative.x, -std::sin(heading) + 0.2 * std::cos(heading), 1e-12);
    EXPECT_NEAR(derivative.y, std::cos(heading) + 0.2 * std::sin(heading), 1e-12);
    EXPECT_DOUBLE_EQ(derivative.psi, 0.03);
  }
}

TEST(DynamicsConsistency, FirstCommandSlewUsesControlInterval)
{
  NMPCConfig config;
  config.horizon_steps = 20;
  config.dt = 0.2;
  NMPCController controller(config);
  const State state{0.0, 0.0, 0.0, 0.6, 0.0, 0.0};
  for (double direction : {-1.0, 1.0}) {
    std::vector<RefPoint> references;
    for (int i = 0; i <= config.horizon_steps; ++i) {
      references.push_back({0.2 * i, direction * 0.5, direction * 0.1,
          1.0, 0.0, direction * 0.03, direction * 0.03});
    }
    const Control previous{0.6, -direction * 0.04};
    controller.setLastCommand(previous);
    for (double control_dt : {0.025, 0.05, 0.075}) {
      const auto command = controller.solve(state, references, control_dt);
      ASSERT_FALSE(controller.lastSolveFailed());
      const double speed_delta = command.speed - previous.speed;
      EXPECT_LE(speed_delta,
        config.max_speed_command_accel_mps2 * control_dt + 1e-6);
      EXPECT_GE(speed_delta,
        -config.max_speed_command_decel_mps2 * control_dt - 1e-6);
      EXPECT_LE(std::abs(command.yaw_rate - previous.yaw_rate),
        config.max_yaw_rate_command_accel_radps2 * control_dt + 1e-6);
      const auto prediction = controller.predictedFirstCommand();
      EXPECT_NEAR(command.speed, prediction.speed, 1e-6);
      EXPECT_NEAR(command.yaw_rate, prediction.yaw_rate, 1e-6);
    }
  }
}

TEST(DynamicsConsistency, CommandLimiterSeparatesAccelerationAndBraking)
{
  NMPCConfig config;
  config.max_speed_command_accel_mps2 = 0.12;
  config.max_speed_command_decel_mps2 = 1.25;

  const auto accelerating = limitCommandChange({5.0, 0.0}, {1.0, 0.0}, 0.1, config);
  const auto braking = limitCommandChange({0.0, 0.0}, {5.0, 0.0}, 0.1, config);
  EXPECT_NEAR(accelerating.speed, 1.012, 1e-12);
  EXPECT_NEAR(braking.speed, 4.875, 1e-12);
}

TEST(NmpcSimplification, StageCostsMatchPredictedStateErrors)
{
  NMPCConfig config;
  config.horizon_steps = 6;
  NMPCController controller(config);
  const std::array<RefPoint, 3> cases{{
    {0.0, 0.0, 0.4, 1.0, 0.2, 0.0, 0.0, -1.0},
    {0.0, 0.0, -PI + 0.02, 1.0, -0.2, 0.0, 0.0, 0.7},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}};
  for (const auto & reference : cases) {
    SCOPED_TRACE(reference.psi);
    controller.resetWarmStart();
    const std::vector<RefPoint> references(config.horizon_steps + 1, reference);
    const State initial{0.1, -0.15,
      reference.psi < -3.0 ? PI - 0.02 : 0.01, reference.u, reference.v * 2.0, 0.0};
    controller.solve(initial, references);
    ASSERT_FALSE(controller.lastSolveFailed());
    const auto & prediction = controller.predictedTrajectory();
    ASSERT_EQ(prediction.size(), references.size());
    auto expected_costs = [&config, &reference](const State & state) {
        const double dx = state.x - reference.x;
        const double dy = state.y - reference.y;
        const double chi_ref = reference.psi -
          courseSideslipAngle(reference.u, reference.v);
        const double lateral = std::cos(chi_ref) * dx +
          std::sin(chi_ref) * dy;
        const double raw_heading = reference.psi - state.psi +
          courseSideslipAngle(state.u, state.v) -
          courseSideslipAngle(reference.u, reference.v);
        const double heading = std::atan2(
        std::sin(raw_heading), std::cos(raw_heading));
        return std::array<double, 5>{
        config.weight_lateral * lateral * lateral,
        config.weight_course * heading * heading,
        config.weight_u * (state.u - reference.u) * (state.u - reference.u),
        config.weight_v * (state.v - reference.v) * (state.v - reference.v),
        config.weight_r * (state.r - reference.r) * (state.r - reference.r)};
      };
    std::array<double, 5> stage{};
    for (int k = 0; k < config.horizon_steps; ++k) {
      const auto costs = expected_costs(prediction[k]);
      for (std::size_t i = 0; i < stage.size(); ++i) {
        stage[i] += costs[i];
      }
    }
    const auto & actual = controller.lastCostBreakdown();
    const std::array<double, 5> actual_stage{
      actual.stage_lateral, actual.stage_course, actual.stage_u,
      actual.stage_v, actual.stage_r};
    double total = actual.stage_delta_control;
    for (std::size_t i = 0; i < stage.size(); ++i) {
      EXPECT_NEAR(actual_stage[i], stage[i], 1e-9);
      total += stage[i];
    }
    EXPECT_NEAR(actual.total, total, 1e-9);
  }
}

TEST(NmpcSimplification, CourseMetadataDoesNotAffectOptimization)
{
  NMPCConfig config;
  config.horizon_steps = 6;
  NMPCController controller(config);
  std::vector<RefPoint> references(config.horizon_steps + 1,
    RefPoint{0.0, 0.0, 0.4, 1.0, 0.2, 0.0, 0.0, -1.0});
  const State initial{0.1, -0.15, 0.01, 1.0, 0.4, 0.0};
  const auto baseline = controller.solve(initial, references);
  ASSERT_FALSE(controller.lastSolveFailed());
  const double baseline_cost = controller.lastCostBreakdown().total;
  for (auto & reference : references) {
    reference.course = 2.0;
    reference.curvature = 0.5;
  }
  controller.resetWarmStart();
  const auto command = controller.solve(initial, references);
  ASSERT_FALSE(controller.lastSolveFailed());
  EXPECT_NEAR(command.speed, baseline.speed, 1e-9);
  EXPECT_NEAR(command.yaw_rate, baseline.yaw_rate, 1e-9);
  EXPECT_NEAR(controller.lastCostBreakdown().total, baseline_cost, 1e-9);
}

TEST(NmpcSimplification, ReferenceGroundCourseDefinesFrenetFrame)
{
  RefPoint ref{};
  ref.course = 0.7;
  const State east{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const State north{0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  ref.psi = 0.0;
  EXPECT_NEAR(pathFrameError(east, ref).lateral, 1.0, 1e-12);
  ref.psi = -PI / 2.0;
  EXPECT_NEAR(pathFrameError(north, ref).lateral, -1.0, 1e-12);
  ref.psi = PI / 2.0;
  EXPECT_NEAR(pathFrameError(north, ref).lateral, 1.0, 1e-12);
  ref.psi = PI;
  EXPECT_NEAR(pathFrameError(east, ref).lateral, -1.0, 1e-12);

  // 非零参考侧滑时，Frenet 横向轴随地速切线旋转，而不是继续使用船首轴。
  ref = {};
  ref.psi = 0.0;
  ref.u = 1.0;
  ref.v = 1.0;
  const double chi = -courseSideslipAngle(ref.u, ref.v);
  const State tangent{-std::sin(chi), std::cos(chi), 0.0, 0.0, 0.0, 0.0};
  EXPECT_NEAR(pathFrameError(tangent, ref).along, 1.0, 1e-12);
  EXPECT_NEAR(pathFrameError(tangent, ref).lateral, 0.0, 1e-12);
}

TEST(NmpcSimplification, InfeasibleStateClearsPredictionAndNextSolveRecovers)
{
  NMPCConfig config;
  config.horizon_steps = 6;
  NMPCController controller(config);
  const std::vector<RefPoint> references(config.horizon_steps + 1,
    RefPoint{0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0});
  const State valid{0.0, 0.0, 0.0, 0.6, 0.0, 0.0};
  controller.solve(valid, references);
  ASSERT_FALSE(controller.lastSolveFailed());
  ASSERT_FALSE(controller.predictedTrajectory().empty());

  for (int failure = 1; failure <= 2; ++failure) {
    // 现有指令及变化率范围不可能在一步内把 -4 m/s 恢复至 -0.6 m/s 以上。
    const auto command = controller.solve({0.0, 0.0, 0.0, -4.0, 0.0, 0.0}, references);
    EXPECT_TRUE(controller.lastSolveFailed());
    EXPECT_EQ(controller.consecutiveFailures(), failure);
    EXPECT_EQ(controller.lastCostBreakdown().consecutive_failures, failure);
    EXPECT_DOUBLE_EQ(command.speed, 0.0);
    EXPECT_DOUBLE_EQ(command.yaw_rate, 0.0);
    EXPECT_TRUE(controller.predictedTrajectory().empty());
    EXPECT_DOUBLE_EQ(controller.predictedFirstCommand().speed, 0.0);
    EXPECT_DOUBLE_EQ(controller.lastCostBreakdown().total, 0.0);
    EXPECT_FALSE(controller.lastCostBreakdown().solver_succeeded);
    controller.setLastCommand({});
  }
  controller.solve(valid, references);
  EXPECT_FALSE(controller.lastSolveFailed());
  EXPECT_EQ(controller.consecutiveFailures(), 0);
  EXPECT_TRUE(controller.lastCostBreakdown().solver_succeeded);
  EXPECT_EQ(controller.predictedTrajectory().size(), references.size());
}

}  // namespace usv_nmpc
