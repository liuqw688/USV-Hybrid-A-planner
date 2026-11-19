/**
 * @file test_dynamics.cpp
 * @brief 验证六状态船体模型、命令限幅及数值积分。
 */

#include "usv_model_sim/dynamics.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace usv_model_sim
{

TEST(Dynamics, ForwardAndRightwardVelocityMatchNorthReferencedModelAxes)
{
  Dynamics dynamics;
  State state;
  state.u = 1.0;
  auto derivative = dynamics.derivative(state, {});
  EXPECT_NEAR(derivative.east, 0.0, 1e-12);
  EXPECT_NEAR(derivative.north, 1.0, 1e-12);
  state.u = 0.0;
  state.v = 1.0;
  derivative = dynamics.derivative(state, {});
  EXPECT_NEAR(derivative.east, 1.0, 1e-12);
  EXPECT_NEAR(derivative.north, 0.0, 1e-12);
  state.yaw = -kPi * 0.5;
  derivative = dynamics.derivative(state, {});
  EXPECT_NEAR(derivative.east, 0.0, 1e-12);
  EXPECT_NEAR(derivative.north, -1.0, 1e-12);
  state.yaw = kPi;
  derivative = dynamics.derivative(state, {});
  EXPECT_NEAR(derivative.east, -1.0, 1e-12);
  EXPECT_NEAR(derivative.north, 0.0, 1e-12);
  state.yaw = kPi * 0.5;
  derivative = dynamics.derivative(state, {});
  EXPECT_NEAR(derivative.east, 0.0, 1e-12);
  EXPECT_NEAR(derivative.north, 1.0, 1e-12);
}

TEST(Dynamics, UsesSpecifiedIdentifiedDynamics)
{
  const State state{0.0, 0.0, 0.4, 0.8, 0.1, -0.02};
  const Command command{0.6, 0.08};
  const auto derivative = Dynamics::evaluate(state, command);
  EXPECT_NEAR(derivative.east, -0.8 * std::sin(0.4) + 0.1 * std::cos(0.4), 1e-12);
  EXPECT_NEAR(derivative.north, 0.8 * std::cos(0.4) + 0.1 * std::sin(0.4), 1e-12);
  EXPECT_NEAR(derivative.u, -0.431930955 * 0.8 + 0.437133189 * 0.6, 1e-12);
  EXPECT_NEAR(derivative.v,
    -0.313324916 * 0.1 + 0.006741061 * -0.02 +
    0.407958500 * 0.08 + 0.000993619, 1e-12);
  EXPECT_NEAR(derivative.yaw_rate,
    -0.166118168 * 0.1 - 0.544509453 * -0.02 +
    0.837536662 * 0.08 + 0.000626529, 1e-12);
}

TEST(Dynamics, LeftTurnRetainsIdentifiedSideslip)
{
  Dynamics dynamics;
  State state;
  for (int i = 0; i < 200; ++i) {
    dynamics.step(state, Command{1.0, 5.0 * kPi / 180.0}, 0.05);
  }
  EXPECT_GT(state.v, 0.0);  // 辨识模型中侧向速度 v 以右舷方向为正。
  EXPECT_GT(state.yaw_rate, 0.0);
  EXPECT_GT(state.yaw, 0.0);
}

TEST(Dynamics, LimitsCommandToConfiguredBounds)
{
  Dynamics dynamics(Parameters{2.0, 15.0 * kPi / 180.0});
  const auto command = dynamics.limit_command(Command{8.0, -2.0});
  EXPECT_DOUBLE_EQ(command.forward_speed, 2.0);
  EXPECT_DOUBLE_EQ(command.yaw_rate, -15.0 * kPi / 180.0);
}

TEST(Dynamics, ForwardCommandMovesNorthAtZeroHeading)
{
  Dynamics dynamics;
  State state;
  dynamics.step(state, Command{2.0, 0.0}, 0.1);
  EXPECT_NEAR(state.east, 0.0, 1e-3);
  EXPECT_GT(state.north, 0.0);
  EXPECT_GT(state.u, 0.0);
}

TEST(Dynamics, EastwardMotionUsesMinusNinetyDegreeHeading)
{
  Dynamics dynamics;
  State state;
  state.yaw = -kPi / 2.0;
  dynamics.step(state, Command{2.0, 0.0}, 0.1);
  EXPECT_GT(state.east, 0.0);
  EXPECT_NEAR(state.north, 0.0, 1e-3);
}

TEST(Dynamics, YawRemainsBounded)
{
  Dynamics dynamics;
  State state;
  state.yaw = 4.0 * kPi;
  dynamics.step(state, Command{}, 0.1);
  EXPECT_LE(state.yaw, kPi);
  EXPECT_GE(state.yaw, -kPi);
}

}  // namespace usv_model_sim
