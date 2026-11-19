/**
 * @file usv_model.cpp
 * @brief 无人船公共动力学的符号表达与数值积分适配。
 */

#include "usv_nmpc_controller/usv_model.hpp"

#include <cmath>
#include <stdexcept>

namespace usv_nmpc
{
namespace
{

State addScaled(const State & state, const State & derivative, double scale)
{
  return {
    state.x + derivative.x * scale,
    state.y + derivative.y * scale,
    state.psi + derivative.psi * scale,
    state.u + derivative.u * scale,
    state.v + derivative.v * scale,
    state.r + derivative.r * scale};
}

State evaluate(const State & state, const Control & control)
{
  const auto & p = kModelParameters;
  return {
    -state.u * std::sin(state.psi) + state.v * std::cos(state.psi),
    state.u * std::cos(state.psi) + state.v * std::sin(state.psi),
    state.r,
    p.au * state.u + p.bu * control.speed + p.bu0,
    p.av1 * state.v + p.av2 * state.r + p.bv * control.yaw_rate + p.bv0,
    p.ar1 * state.v + p.ar2 * state.r + p.br * control.yaw_rate + p.br0};
}

}  // namespace

double steadySwayVelocity(double yaw_rate)
{
  const auto & p = kModelParameters;
  return ((p.bv * p.ar2 - p.br * p.av2) * yaw_rate +
         p.bv * p.br0 - p.br * p.bv0) / (p.av1 * p.br - p.bv * p.ar1);
}

casadi::SX USVModel::dynamicsSX(
  const casadi::SX & state,
  const casadi::SX & control)
{
  const ModelParameters & p = kModelParameters;

  const casadi::SX psi = state(2);
  const casadi::SX u = state(3);
  const casadi::SX v = state(4);
  const casadi::SX r = state(5);
  const casadi::SX v_cmd = control(0);
  const casadi::SX w_cmd = control(1);

  const casadi::SX x_dot = -u * sin(psi) + v * cos(psi);
  const casadi::SX y_dot = u * cos(psi) + v * sin(psi);
  const casadi::SX u_dot =
    p.au * u + p.bu * v_cmd + p.bu0;
  const casadi::SX v_dot =
    p.av1 * v + p.av2 * r + p.bv * w_cmd + p.bv0;
  const casadi::SX r_dot =
    p.ar1 * v + p.ar2 * r + p.br * w_cmd + p.br0;

  return casadi::SX::vertcat({x_dot, y_dot, r, u_dot, v_dot, r_dot});
}

State USVModel::derivative(
  const State & state,
  const Control & control)
{
  return evaluate(state, control);
}

State USVModel::stepRK4(
  const State & state,
  const Control & control,
  double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("积分步长必须为有限正数");
  }

  const State k1 = evaluate(state, control);
  const State k2 = evaluate(addScaled(state, k1, dt * 0.5), control);
  const State k3 = evaluate(addScaled(state, k2, dt * 0.5), control);
  const State k4 = evaluate(addScaled(state, k3, dt), control);
  State result = addScaled(state, k1, dt / 6.0);
  result = addScaled(result, k2, dt / 3.0);
  result = addScaled(result, k3, dt / 3.0);
  return addScaled(result, k4, dt / 6.0);
}

}  // namespace usv_nmpc
