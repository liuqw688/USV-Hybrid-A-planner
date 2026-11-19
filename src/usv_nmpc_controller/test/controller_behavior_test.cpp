/**
 * @file controller_behavior_test.cpp
 * @brief 坐标约定和 NMPC 指令行为测试。
 */

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "usv_nmpc_controller/coordinate_conventions.hpp"
#include "usv_nmpc_controller/nmpc_controller.hpp"
#include "usv_nmpc_controller/usv_model.hpp"

namespace usv_nmpc
{
namespace
{
std::vector<RefPoint> straightReferences(const NMPCConfig & config, double speed)
{
  std::vector<RefPoint> references;
  references.reserve(static_cast<std::size_t>(config.horizon_steps + 1));
  for (int i = 0; i <= config.horizon_steps; ++i) {
    const double distance = speed * config.dt * i;
    references.push_back({0.0, distance, 0.0, speed, 0.0, 0.0});
  }
  return references;
}
}  // namespace

TEST(CoordinateConventions, HeadingsAndBodyVelocitiesUseNorthReferencedFrame)
{
  using coordinates::kPi;
  EXPECT_NEAR(coordinates::usvStateHeadingToMap(0.0), 0.0, 1e-12);
  EXPECT_NEAR(coordinates::usvStateHeadingToMap(-kPi / 2.0), -kPi / 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(coordinates::usvStateForwardVelocityToBody(1.25), 1.25);
  EXPECT_DOUBLE_EQ(coordinates::usvStateLateralVelocityToStarboard(0.4), 0.4);
  EXPECT_DOUBLE_EQ(coordinates::usvStateYawRateToMap(-0.3), -0.3);

  for (double heading : {0.0, -kPi / 2.0, kPi, kPi / 2.0}) {
    const State state{0.0, 0.0, heading, 1.0, 0.2, 0.03};
    const State derivative = USVModel::derivative(state, {});
    EXPECT_NEAR(derivative.x, -std::sin(heading) + 0.2 * std::cos(heading), 1e-12);
    EXPECT_NEAR(derivative.y, std::cos(heading) + 0.2 * std::sin(heading), 1e-12);
    EXPECT_DOUBLE_EQ(derivative.psi, 0.03);
  }
}

TEST(CoordinateConventions, HeadingInterpolationTakesShortestBranch)
{
  constexpr double to_rad = PI / 180.0;
  const double before = 179.0 * to_rad;
  const double after = -179.0 * to_rad;
  EXPECT_NEAR(coordinates::unwrapNear(after, before), 181.0 * to_rad, 1e-12);
  EXPECT_NEAR(coordinates::shortestAngularDistance(before, after), 2.0 * to_rad, 1e-12);
  EXPECT_NEAR(coordinates::interpolateHeading(before, after, 0.5), PI, 1e-12);
}

TEST(CoordinateConventions, LowSpeedSideslipCorrectionIsContinuous)
{
  EXPECT_LT(std::abs(courseSideslipAngle(-0.12, -0.08)), 0.30);
  EXPECT_NEAR(courseSideslipAngle(0.0, 0.0), 0.0, 1e-12);
}

TEST(NmpcController, WarmStartTracksStraightReferenceWithinCommandLimits)
{
  NMPCConfig config;
  config.horizon_steps = 12;
  config.solver_max_iterations = 80;
  NMPCController controller(config);
  const auto references = straightReferences(config, 3.0);
  State state{0.0, 1.0, 0.0, 0.5, 0.0, 0.0};
  Control previous{};

  for (int i = 0; i < 4; ++i) {
    const Control command = controller.solve(state, references);
    ASSERT_FALSE(controller.lastSolveFailed());
    EXPECT_GE(command.speed, config.min_speed_command_mps - 1e-6);
    EXPECT_LE(command.speed, config.max_speed_command_mps + 1e-6);
    EXPECT_GE(command.yaw_rate, -config.max_yaw_rate_command_radps - 1e-6);
    EXPECT_LE(command.yaw_rate, config.max_yaw_rate_command_radps + 1e-6);
    EXPECT_TRUE(std::isfinite(controller.lastCostBreakdown().total));
    if (i > 0) {
      EXPECT_LT(std::abs(command.speed - previous.speed), 2.0);
      EXPECT_LT(std::abs(command.yaw_rate - previous.yaw_rate), 0.5);
    }
    controller.setLastCommand(command);
    state = USVModel::stepRK4(state, command, config.dt);
    previous = command;
  }
}

TEST(NmpcController, PredictedStatesRespectConfiguredVelocityBounds)
{
  NMPCConfig config;
  config.horizon_steps = 20;
  config.max_speed_command_mps = 5.0;
  config.max_yaw_rate_command_radps = 0.3;
  config.max_surge_state_mps = 5.0;
  config.max_sway_state_mps = 2.0;
  config.max_yaw_rate_state_radps = 0.3;
  NMPCController controller(config);
  const auto references = straightReferences(config, 8.0);
  const Control command = controller.solve(State{}, references);

  ASSERT_FALSE(controller.lastSolveFailed());
  EXPECT_LE(command.speed, 5.0 + 1e-6);
  EXPECT_LE(std::abs(command.yaw_rate), 0.3 + 1e-6);
  for (const State & state : controller.predictedTrajectory()) {
    EXPECT_LE(state.u, config.max_surge_state_mps + 1e-5);
    EXPECT_GE(state.u, -config.max_reverse_speed_mps - 1e-5);
    EXPECT_LE(std::abs(state.v), config.max_sway_state_mps + 1e-5);
    EXPECT_LE(std::abs(state.r), config.max_yaw_rate_state_radps + 1e-5);
  }
}

TEST(NmpcController, CourseErrorSelectsTheCorrectTurnDirection)
{
  NMPCConfig config;
  config.horizon_steps = 20;
  config.weight_lateral = 30.0;
  config.weight_course = 60.0;
  config.weight_u = 3.0;
  config.weight_v = 4.0;
  config.weight_r = 3.0;
  for (const double direction : {-1.0, 1.0}) {
    NMPCController controller(config);
    const std::vector<RefPoint> references(config.horizon_steps + 1,
      RefPoint{0.0, 0.0, direction * 0.25, 0.0, 0.0, 0.0});
    const Control command = controller.solve(State{}, references);
    ASSERT_FALSE(controller.lastSolveFailed());
    EXPECT_GT(direction * command.yaw_rate, 0.0);
  }
}

TEST(NmpcController, SurgeAndYawRateSmoothnessAreIndependent)
{
  NMPCConfig config;
  config.horizon_steps = 20;
  config.max_yaw_rate_command_accel_radps2 = 0.12;
  auto references = straightReferences(config, 2.0);

  NMPCController responsive_surge(config);
  const double baseline_surge = responsive_surge.solve(State{}, references).speed;
  ASSERT_FALSE(responsive_surge.lastSolveFailed());
  config.weight_surge_delta = 1e6;
  NMPCController smooth_surge(config);
  const double smoothed_surge = smooth_surge.solve(State{}, references).speed;
  ASSERT_FALSE(smooth_surge.lastSolveFailed());
  EXPECT_LT(smoothed_surge, 0.25 * baseline_surge);

  config.weight_surge_delta = 3.0;
  config.weight_yaw_rate_delta = 0.0;
  const std::vector<RefPoint> turning(config.horizon_steps + 1,
    RefPoint{0.0, 0.0, 0.5, 0.0, 0.0, 0.0});
  NMPCController responsive_yaw(config);
  const double baseline_yaw = responsive_yaw.solve(State{}, turning).yaw_rate;
  ASSERT_FALSE(responsive_yaw.lastSolveFailed());
  config.weight_yaw_rate_delta = 1e6;
  NMPCController smooth_yaw(config);
  const double smoothed_yaw = smooth_yaw.solve(State{}, turning).yaw_rate;
  ASSERT_FALSE(smooth_yaw.lastSolveFailed());
  EXPECT_LT(std::abs(smoothed_yaw), 0.25 * std::abs(baseline_yaw));
}

TEST(NmpcController, ReverseMotionRequiresExplicitConfiguration)
{
  NMPCConfig config;
  config.horizon_steps = 20;
  config.min_speed_command_mps = -1.0;
  config.max_reverse_speed_mps = 1.0;
  std::vector<RefPoint> references;
  for (int i = 0; i <= config.horizon_steps; ++i) {
    const double speed = -0.5;
    references.push_back({speed * config.dt * i, 0.0, 0.0, speed, 0.0, 0.0});
  }
  NMPCController controller(config);
  const State state{0.0, 0.0, 0.0, -0.5, 0.0, 0.0};
  const Control command = controller.solve(state, references);
  ASSERT_FALSE(controller.lastSolveFailed());
  const State next = USVModel::stepRK4(state, command, config.dt);
  EXPECT_LT(next.x, 0.0);
  EXPECT_NEAR(next.u, -0.5, 0.02);
}

TEST(NmpcController, SteadyFeedforwardBalancesIdentifiedCoupling)
{
  NMPCController controller(NMPCConfig{});
  for (const double yaw_rate : {-0.1, 0.0, 0.1}) {
    const double sway = steadySwayVelocity(yaw_rate);
    const RefPoint reference{0.0, 0.0, 0.0, 1.5, sway, yaw_rate};
    const Control command = controller.feedforwardForReference(reference);
    const State derivative = USVModel::derivative(
      {0.0, 0.0, 0.0, 1.5, sway, yaw_rate}, command);
    EXPECT_NEAR(derivative.u, 0.0, 1e-12);
    EXPECT_NEAR(derivative.v, 0.0, 1e-12);
    EXPECT_NEAR(derivative.r, 0.0, 1e-12);
  }
}

}  // namespace usv_nmpc
