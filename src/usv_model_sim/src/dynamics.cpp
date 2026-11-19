/**
 * @file dynamics.cpp
 * @brief 三自由度动力学的导数、RK4 积分和速度限幅。
 */

#include "usv_model_sim/dynamics.hpp"

#include <algorithm>
#include <stdexcept>

namespace usv_model_sim
{
namespace
{

State add_scaled(const State & state, const State & derivative, double scale)
{
  State result = state;
  result.east += derivative.east * scale;
  result.north += derivative.north * scale;
  result.yaw += derivative.yaw * scale;
  result.u += derivative.u * scale;
  result.v += derivative.v * scale;
  result.yaw_rate += derivative.yaw_rate * scale;
  return result;
}

}  // namespace

Dynamics::Dynamics(Parameters parameters)
: parameters_(parameters)
{
}

Command Dynamics::limit_command(Command command) const
{
  command.forward_speed = std::clamp(
    command.forward_speed, -parameters_.max_speed_mps, parameters_.max_speed_mps);
  command.yaw_rate = std::clamp(
    command.yaw_rate, -parameters_.max_yaw_rate_rad_s, parameters_.max_yaw_rate_rad_s);
  return command;
}

State Dynamics::derivative(const State & state, const Command & raw_command) const
{
  return evaluate(state, limit_command(raw_command));
}

State Dynamics::evaluate(const State & state, const Command & command)
{
  const auto & p = kModelParameters;
  State derivative;
  derivative.east = -state.u * std::sin(state.yaw) + state.v * std::cos(state.yaw);
  derivative.north = state.u * std::cos(state.yaw) + state.v * std::sin(state.yaw);
  derivative.yaw = state.yaw_rate;

  // 动力学系数来自《07_完整船体模型与全部参数.pdf》，仅在公共头文件维护。
  derivative.u = p.au * state.u + p.bu * command.forward_speed + p.bu0;
  derivative.v = p.av1 * state.v + p.av2 * state.yaw_rate + p.bv * command.yaw_rate + p.bv0;
  derivative.yaw_rate = p.ar1 * state.v + p.ar2 * state.yaw_rate + p.br * command.yaw_rate + p.br0;
  return derivative;
}

State Dynamics::integrate(const State & state, const Command & command, double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("积分步长必须为有限正数");
  }

  const State k1 = evaluate(state, command);
  const State k2 = evaluate(add_scaled(state, k1, dt * 0.5), command);
  const State k3 = evaluate(add_scaled(state, k2, dt * 0.5), command);
  const State k4 = evaluate(add_scaled(state, k3, dt), command);

  State result = add_scaled(state, k1, dt / 6.0);
  result = add_scaled(result, k2, dt / 3.0);
  result = add_scaled(result, k3, dt / 3.0);
  result = add_scaled(result, k4, dt / 6.0);
  return result;
}

void Dynamics::step(State & state, const Command & command, double dt) const
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return;
  }

  state = integrate(state, limit_command(command), dt);
  state.yaw = std::remainder(state.yaw, 2.0 * kPi);
  state.u = std::clamp(state.u, -parameters_.max_speed_mps, parameters_.max_speed_mps);
  state.yaw_rate = std::clamp(
    state.yaw_rate, -parameters_.max_yaw_rate_rad_s, parameters_.max_yaw_rate_rad_s);
}

}  // namespace usv_model_sim
