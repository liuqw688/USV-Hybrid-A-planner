/**
 * @file test_usv_dynamics.cpp
 * @brief 验证无人船动力学、坐标约定和数值积分。
 */

#include "path_searching/usv_dynamics.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace fast_planner
{
namespace usv_dynamics
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(USVDynamics, CardinalHeadingsUseNorthZeroAndRfuAxes)
{
  struct Case
  {
    double phi;
    Eigen::Vector2d forward;
    Eigen::Vector2d starboard;
  };
  const std::vector<Case> cases{
    {0.0, {0.0, 1.0}, {1.0, 0.0}},
    {-kPi / 2.0, {1.0, 0.0}, {0.0, -1.0}},
    {kPi / 2.0, {-1.0, 0.0}, {0.0, 1.0}},
    {kPi, {0.0, -1.0}, {-1.0, 0.0}}};

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.phi);
    State state = State::Zero();
    state(2) = test_case.phi;
    state(3) = 1.0;
    auto rate = derivative(state, Command::Zero());
    EXPECT_NEAR(rate(0), test_case.forward.x(), 1e-12);
    EXPECT_NEAR(rate(1), test_case.forward.y(), 1e-12);

    state(3) = 0.0;
    state(4) = 1.0;
    rate = derivative(state, Command::Zero());
    EXPECT_NEAR(rate(0), test_case.starboard.x(), 1e-12);
    EXPECT_NEAR(rate(1), test_case.starboard.y(), 1e-12);
  }
}

TEST(USVDynamics, UsesTheUSVSystemsIdentifiedEquations)
{
  State state;
  state << 2.0, 3.0, 0.4, 0.8, 0.1, -0.02;
  Command command;
  command << 0.6, 0.08, 91.0;
  const State rate = derivative(state, command);

  EXPECT_NEAR(rate(0), -0.8 * std::sin(0.4) + 0.1 * std::cos(0.4), 1e-12);
  EXPECT_NEAR(rate(1), 0.8 * std::cos(0.4) + 0.1 * std::sin(0.4), 1e-12);
  EXPECT_DOUBLE_EQ(rate(2), -0.02);
  EXPECT_NEAR(rate(3), -0.431930955 * 0.8 + 0.437133189 * 0.6, 1e-12);
  EXPECT_NEAR(rate(4), -0.313324916 * 0.1 + 0.006741061 * -0.02 +
    0.407958500 * 0.08 + 0.000993619, 1e-12);
  EXPECT_NEAR(rate(5), -0.166118168 * 0.1 - 0.544509453 * -0.02 +
    0.837536662 * 0.08 + 0.000626529, 1e-12);

  command(2) = -400.0;
  EXPECT_TRUE(derivative(state, command).isApprox(rate, 0.0));
}

TEST(USVDynamics, BodyAndWorldVelocityConversionsAreInverse)
{
  for (const double phi : {-kPi, -kPi / 2.0, -0.37, 0.0, 0.82, kPi / 2.0}) {
    const Eigen::Vector3d pose(4.0, -3.0, phi);
    const Eigen::Vector3d body_velocity(1.7, -0.28, 0.12);
    const auto world_velocity = toWorldVelocity(pose, body_velocity);
    const auto recovered = toBodyVelocity(pose, world_velocity);
    EXPECT_TRUE(recovered.isApprox(body_velocity, 1e-12));
  }
}

TEST(USVDynamics, CardinalVectorConversionsMatchNavigationHeading)
{
  EXPECT_NEAR(headingFromForwardVector(0.0, 1.0), 0.0, 1e-12);
  EXPECT_NEAR(headingFromForwardVector(1.0, 0.0), -kPi / 2.0, 1e-12);
  EXPECT_NEAR(headingFromForwardVector(-1.0, 0.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(std::abs(headingFromForwardVector(0.0, -1.0)), kPi, 1e-12);
  EXPECT_NEAR(headingFromRightVector(1.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(headingFromRightVector(0.0, -1.0), -kPi / 2.0, 1e-12);
}

TEST(USVDynamics, PoseDerivativesRecoverStateAndControl)
{
  State state;
  state << 8.0, -2.0, -0.71, 1.2, 0.31, -0.09;
  Command expected_command;
  expected_command << 0.84, 0.11, 0.0;
  const Eigen::Vector3d pose = state.head<3>();
  const Eigen::Vector3d world_velocity = toWorldVelocity(pose, state.tail<3>());
  const Eigen::Vector3d world_acceleration = poseAcceleration(state, expected_command);

  Eigen::Vector3d body_velocity;
  Eigen::Vector3d command;
  stateAndCommandFromPoseDerivatives(
    pose, world_velocity, world_acceleration, body_velocity, command);

  EXPECT_TRUE(body_velocity.isApprox(state.tail<3>(), 1e-12));
  EXPECT_TRUE(command.isApprox(expected_command, 1e-12));
}

TEST(USVDynamics, Rk4KeepsHeadingContinuousAndAcceptsZeroDuration)
{
  State state;
  state << 0.0, 0.0, kPi - 0.02, 0.7, 0.1, 0.2;
  Command command = commandFromBodyAccelerations(0.7, 0.1, 0.2, 0.0, 0.0);
  EXPECT_TRUE(integrateRK4(state, command, 0.0, 10).isApprox(state, 0.0));
  const State advanced = integrateRK4(state, command, 0.5, 10);
  EXPECT_GT(advanced(2), kPi);
  EXPECT_THROW(integrateRK4(state, command, -0.1, 10), std::invalid_argument);
  EXPECT_THROW(integrateRK4(state, command, 0.1, 0), std::invalid_argument);
}

TEST(USVDynamics, SharedRk4CollisionSamplesMatchIndependentIntegration)
{
  // 覆盖不同速度、侧滑、航向和控制量，验证碰撞检查点数值偏差。
  for (double phi : {-3.0, -1.2, 0.0, 1.5, 3.0}) {
    for (double u : {0.0, 2.0, 5.0}) {
      for (double w : {-0.3, 0.0, 0.3}) {
        State start;
        start << 4.0, -2.0, phi, u, 0.3, -0.2;
        Command command(4.0, w, 0.0);
        std::vector<State> samples;
        integrateRK4Samples(start, command, 0.6, 10, 5, samples);
        ASSERT_EQ(samples.size(), 5u);
        for (int i = 0; i < 5; ++i) {
          const State independent = integrateRK4(start, command, 0.6 * (i + 1) / 5.0, 10);
          EXPECT_LT((samples[i] - independent).cwiseAbs().maxCoeff(), 1e-6);
        }
        EXPECT_LT((samples.back() - integrateRK4(start, command, 0.6, 10)).norm(), 1e-12);
      }
    }
  }
}

TEST(USVDynamics, ShotPerturbationReusesExactUnchangedPrefix)
{
  using Controls = Eigen::Matrix<double, 8, 1>;
  for (double phi : {-3.0, 0.0, 1.8}) {
    for (double duration : {24.0, 48.0, 100.0}) {
      State start;
      start << 2.0, -3.0, phi, 2.0, 0.2, -0.1;
      Controls controls;
      controls << 3.0, 0.1, 4.0, -0.2, 2.0, 0.05, 1.0, 0.0;
      const double piece_duration = duration / 4.0;
      auto advance = [&](State state, const Controls & values, int first, int last,
                         std::array<State, 5> * prefix) {
        if (prefix) (*prefix)[first] = state;
        for (int piece = first; piece < last; ++piece) {
          const Command command(values(2 * piece), values(2 * piece + 1), 0.0);
          double elapsed = 0.0;
          while (elapsed < piece_duration - 1e-12) {
            const double step = std::min(0.1, piece_duration - elapsed);
            state = integrateRK4(state, command, step, 10);
            elapsed += step;
          }
          if (prefix) (*prefix)[piece + 1] = state;
        }
        return state;
      };
      std::array<State, 5> prefix;
      const State baseline = advance(start, controls, 0, 4, &prefix);
      for (int j = 0; j < 8; ++j) {
        Controls perturbed = controls;
        perturbed(j) += 1e-3;
        const State full = advance(start, perturbed, 0, 4, nullptr);
        const State reused = advance(prefix[j / 2], perturbed, j / 2, 4, nullptr);
        EXPECT_EQ((full - reused).cwiseAbs().maxCoeff(), 0.0);
        EXPECT_TRUE(baseline.allFinite());
      }
    }
  }
}

TEST(USVDynamics, SurgeCommandInterpolationMatchesDirectIntegration)
{
  for (double phi : {-3.0, 0.0, 1.8}) {
    for (double steering : {-0.3, -0.075, 0.0, 0.075, 0.3}) {
      State start;
      start << 75.0, -120.0, phi, 1.4, -0.2, 0.15;
      std::vector<State> low, high, direct;
      integrateRK4Samples(start, Command(0.0, steering, 0.0), 0.5, 10, 5, low);
      integrateRK4Samples(start, Command(2.0, steering, 0.0), 0.5, 10, 5, high);
      for (int i = 0; i < 5; ++i) {
        const double fraction = i / 4.0;
        integrateRK4Samples(start, Command(2.0 * fraction, steering, 0.0), 0.5, 10, 5, direct);
        for (int k = 0; k < 5; ++k) {
          const State interpolated = low[k] + fraction * (high[k] - low[k]);
          EXPECT_LT((interpolated - direct[k]).cwiseAbs().maxCoeff(), 1e-11);
        }
      }
    }
  }
}

TEST(USVDynamics, ShotStepPreservesFineIntegrationAccuracy)
{
  for (double phi : {-3.0, 0.0, 1.8}) {
    for (double duration : {6.0, 12.0, 25.0}) {
      for (double steering : {-0.255, 0.0, 0.255}) {
        State start;
        start << 2.0, -3.0, phi, 2.0, 0.2, -0.1;
        const Command command(1.85, steering, 0.0);
        const State fine = integrateRK4(start, command, duration,
          static_cast<int>(std::ceil(duration / 0.01)));
        const State shot = integrateRK4(start, command, duration,
          static_cast<int>(std::ceil(duration / 0.1)));
        EXPECT_LT((shot.head<2>() - fine.head<2>()).norm(), 1e-5);
        EXPECT_LT((shot.tail<4>() - fine.tail<4>()).cwiseAbs().maxCoeff(), 1e-6);
      }
    }
  }
}

}  // namespace
}  // namespace usv_dynamics
}  // namespace fast_planner
