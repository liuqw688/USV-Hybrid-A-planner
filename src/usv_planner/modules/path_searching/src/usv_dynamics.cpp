/**
 * @file usv_dynamics.cpp
 * @brief 无人船模型、坐标转换及 RK4 数值积分实现。
 */

#include "path_searching/usv_dynamics.hpp"

#include <cmath>
#include <stdexcept>

namespace fast_planner
{
namespace usv_dynamics
{
namespace
{

State addScaled(const State & state, const State & derivative_value, double scale)
{
  return state + scale * derivative_value;
}

}  // namespace

State derivative(const State & state, const Command & command)
{
  const usv_model_sim::State model_state{
    state(0), state(1), state(2), state(3), state(4), state(5)};
  const usv_model_sim::Command model_command{command(0), command(1)};
  const auto model_rate = usv_model_sim::Dynamics::evaluate(model_state, model_command);
  State result;
  result << model_rate.east, model_rate.north, model_rate.yaw,
    model_rate.u, model_rate.v, model_rate.yaw_rate;
  return result;
}

SurgeBrakingEnvelope estimatePassiveSurgeBraking(double initial_u, double target_abs_u)
{
  if (!std::isfinite(initial_u) || !std::isfinite(target_abs_u) || target_abs_u <= 0.0)
    throw std::invalid_argument("USV braking speeds must be finite and the target tolerance positive.");

  const double initial_speed = std::abs(initial_u);
  if (initial_speed <= target_abs_u)
    return {0.0, 0.0};

  const double damping = -kModelParameters.au;
  if (!std::isfinite(damping) || damping <= 0.0 || kModelParameters.bu0 != 0.0)
    throw std::logic_error("Passive surge braking requires positive damping and zero model bias.");

  return {
    std::log(initial_speed / target_abs_u) / damping,
    (initial_speed - target_abs_u) / damping};
}

Eigen::Vector3d toWorldVelocity(
  const Eigen::Vector3d & pose, const Eigen::Vector3d & body_velocity)
{
  const double phi = pose(2);
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  const double u = body_velocity(0);
  const double v = body_velocity(1);
  return Eigen::Vector3d(c * v - s * u, s * v + c * u, body_velocity(2));
}

Eigen::Vector3d toBodyVelocity(
  const Eigen::Vector3d & pose, const Eigen::Vector3d & world_velocity)
{
  const double phi = pose(2);
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  const double x_dot = world_velocity(0);
  const double y_dot = world_velocity(1);
  return Eigen::Vector3d(-s * x_dot + c * y_dot,
    c * x_dot + s * y_dot, world_velocity(2));
}

double headingFromRightVector(double east_right_axis, double north_right_axis)
{
  return std::atan2(north_right_axis, east_right_axis);
}

double headingFromForwardVector(double east_forward_axis, double north_forward_axis)
{
  return std::atan2(-east_forward_axis, north_forward_axis);
}

Command commandFromBodyAccelerations(
  double forward_velocity, double starboard_velocity, double yaw_rate,
  double forward_acceleration, double yaw_acceleration)
{
  const auto & p = kModelParameters;
  Command command;
  command(0) = (forward_acceleration - p.au * forward_velocity - p.bu0) / p.bu;
  command(1) = (yaw_acceleration - p.ar1 * starboard_velocity -
    p.ar2 * yaw_rate - p.br0) / p.br;
  command(2) = 0.0;
  return command;
}

Eigen::Vector3d poseAcceleration(const State & state, const Command & command)
{
  const State state_rate = derivative(state, command);
  const double phi = state(2);
  const double u = state(3);
  const double v = state(4);
  const double r = state(5);
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  return Eigen::Vector3d(
    -s * state_rate(3) + c * state_rate(4) - r * (c * u + s * v),
    c * state_rate(3) + s * state_rate(4) + r * (c * v - s * u),
    state_rate(5));
}

void stateAndCommandFromPoseDerivatives(
  const Eigen::Vector3d & pose,
  const Eigen::Vector3d & pose_velocity,
  const Eigen::Vector3d & pose_acceleration,
  Eigen::Vector3d & body_velocity,
  Eigen::Vector3d & command)
{
  body_velocity = toBodyVelocity(pose, pose_velocity);
  const double phi = pose(2);
  const double r = pose_velocity(2);
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  const double u_dot = -s * pose_acceleration(0) + c * pose_acceleration(1) -
    r * body_velocity(1);
  command = commandFromBodyAccelerations(
    body_velocity(0), body_velocity(1), r, u_dot, pose_acceleration(2));
}

State integrateRK4(
  const State & state, const Command & command, double duration_s, int steps)
{
  if (!std::isfinite(duration_s) || duration_s < 0.0)
    throw std::invalid_argument("USV integration duration must be finite and nonnegative.");
  if (steps <= 0)
    throw std::invalid_argument("USV integration step count must be positive.");

  State result = state;
  const double h = duration_s / static_cast<double>(steps);
  for (int i = 0; i < steps; ++i)
  {
    const State k1 = derivative(result, command);
    const State k2 = derivative(addScaled(result, k1, 0.5 * h), command);
    const State k3 = derivative(addScaled(result, k2, 0.5 * h), command);
    const State k4 = derivative(addScaled(result, k3, h), command);
    result += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
  return result;
}

void integrateRK4Samples(const State & state, const Command & command, double duration_s,
  int steps, int count, std::vector<State> & samples)
{
  if (count <= 0 || steps <= 0 || steps % count != 0)
    throw std::invalid_argument("RK4 sample count must divide the positive step count.");
  samples.clear();
  samples.reserve(count);
  State current = state;
  for (int i = 0; i < count; ++i) {
    current = integrateRK4(current, command, duration_s / count, steps / count);
    samples.push_back(current);
  }
}

}  // namespace usv_dynamics
}  // namespace fast_planner
