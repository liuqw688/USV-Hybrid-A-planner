/**
 * @file nmpc_problem.cpp
 * @brief CasADi NMPC 优化问题构建。
 */

#include "usv_nmpc_controller/nmpc_controller.hpp"

#include "usv_nmpc_controller/usv_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace usv_nmpc
{

namespace
{
constexpr int kStateSize = 6;
constexpr int kControlSize = 2;
constexpr int kParameterOffset = kStateSize + kControlSize;
constexpr int kReferenceSize = kStateSize;
casadi::SX angleError(const casadi::SX & value)
{
  return atan2(sin(value), cos(value));
}

casadi::SX courseSideslip(const casadi::SX & state);

casadi::SX lateralError(const casadi::SX & state, const casadi::SX & reference)
{
  const casadi::SX dx = state(0) - reference(0);
  const casadi::SX dy = state(1) - reference(1);
  // 位置误差位于 ENU 平面；沿参考地速方向投影，横向正方向为参考航向右侧。
  const casadi::SX chi_ref = reference(2) - courseSideslip(reference);
  return cos(chi_ref) * dx + sin(chi_ref) * dy;
}

casadi::SX courseSideslip(const casadi::SX & state)
{
  // 以前进侧滑角为基础，并在低速时平滑淡出，避免静止附近的方向角不稳定。
  const casadi::SX speed_sq = state(3) * state(3) + state(4) * state(4);
  const double floor_sq = kCourseSpeedFloorMps * kCourseSpeedFloorMps;
  return speed_sq / (speed_sq + floor_sq) *
         atan2(state(4), sqrt(state(3) * state(3) + floor_sq));
}

// 按横向位置、侧滑补偿航向、u、v、r 的顺序返回五项状态代价。
casadi::SX stateTrackingCosts(
  const casadi::SX & state, const casadi::SX & reference, const NMPCConfig & config)
{
  using casadi::SX;
  const SX lateral = lateralError(state, reference);
  const SX heading = angleError(reference(2) - state(2) +
    courseSideslip(state) - courseSideslip(reference));
  const SX u_error = state(3) - reference(3);
  const SX v_error = state(4) - reference(4);
  const SX r_error = state(5) - reference(5);
  return SX::vertcat({
        config.weight_lateral * lateral * lateral,
        config.weight_course * heading * heading,
        config.weight_u * u_error * u_error,
        config.weight_v * v_error * v_error,
        config.weight_r * r_error * r_error});
}
}  // namespace

void NMPCController::validateConfig() const
{
  if (config_.horizon_steps < 2 || config_.dt <= 0.0 ||
    config_.min_speed_command_mps > config_.max_speed_command_mps ||
    config_.max_yaw_rate_command_radps <= 0.0 ||
    config_.yaw_rate_envelope_radius_m <= 0.0 ||
    config_.yaw_rate_envelope_allowance_radps < 0.0 ||
    config_.max_reverse_speed_mps < 0.0 ||
    !std::isfinite(config_.max_reverse_speed_mps) ||
    !std::isfinite(config_.max_surge_state_mps) || config_.max_surge_state_mps <= 0.0 ||
    !std::isfinite(config_.max_sway_state_mps) || config_.max_sway_state_mps <= 0.0 ||
    !std::isfinite(config_.max_yaw_rate_state_radps) ||
    config_.max_yaw_rate_state_radps <= 0.0 ||
    config_.weight_surge_delta < 0.0 ||
    config_.weight_yaw_rate_delta < 0.0 ||
    config_.solver_max_iterations <= 0 || config_.solver_tolerance <= 0.0 ||
    config_.max_speed_command_accel_mps2 <= 0.0 ||
    config_.max_speed_command_decel_mps2 <= 0.0 ||
    config_.max_yaw_rate_command_accel_radps2 <= 0.0)
  {
    throw std::invalid_argument("NMPC 配置无效");
  }
}

void NMPCController::buildSolver()
{
  const int horizon = config_.horizon_steps;
  const casadi::SX states = casadi::SX::sym("X", kStateSize, horizon + 1);
  // 决策变量 U 为速度、角速度指令，与 /cmd 保持相同单位。
  const casadi::SX controls = casadi::SX::sym("U", kControlSize, horizon);
  const casadi::SX parameters = casadi::SX::sym(
    "P", kParameterOffset + kReferenceSize * (horizon + 1));
  // 前五项为状态代价，末项为两通道指令增量代价；与诊断字段顺序一致。
  std::vector<casadi::SX> cost_terms(6, casadi::SX(0.0));
  std::vector<casadi::SX> constraints;
  std::vector<casadi::SX> command_constraints;
  std::vector<casadi::SX> rate_constraints;
  // 动力学等式、指令幅值、转艏率包络、指令变化量分别收集，保证上下界顺序一致。
  std::vector<casadi::SX> motion_constraints;
  constraints.reserve(kStateSize * (horizon + 1) + 2 * horizon);
  constraints.push_back(states(casadi::Slice(), 0) - parameters(casadi::Slice(0, 6)));

  for (int k = 0; k < horizon; ++k) {
    const casadi::SX state = states(casadi::Slice(), k);
    const int ref_start = kParameterOffset + kReferenceSize * k;
    const casadi::SX reference = parameters(
      casadi::Slice(ref_start, ref_start + kReferenceSize));
    const casadi::SX tracking = stateTrackingCosts(state, reference, config_);
    for (int i = 0; i < 5; ++i) {
      cost_terms[i] += tracking(i);
    }
    const casadi::SX command = controls(casadi::Slice(), k);
    const casadi::SX delta = command - (k == 0 ?
      parameters(casadi::Slice(6, 8)) : controls(casadi::Slice(), k - 1));
    // 两通道指令增量分别按各自幅值上限归一化并加权，消除量纲差异。
    // 增量代价用于平滑指令，变化率硬约束限定允许范围。
    const casadi::SX surge_delta = delta(0) / config_.max_speed_command_mps;
    const casadi::SX yaw_rate_delta =
      delta(1) / config_.max_yaw_rate_command_radps;
    cost_terms[5] += (
      config_.weight_surge_delta * surge_delta * surge_delta +
      config_.weight_yaw_rate_delta * yaw_rate_delta * yaw_rate_delta) /
      config_.dt;

    const casadi::SX k1 = USVModel::dynamicsSX(state, command);
    const casadi::SX k2 = USVModel::dynamicsSX(
      state + 0.5 * config_.dt * k1, command);
    const casadi::SX k3 = USVModel::dynamicsSX(
      state + 0.5 * config_.dt * k2, command);
    const casadi::SX k4 = USVModel::dynamicsSX(
      state + config_.dt * k3, command);
    constraints.push_back(states(casadi::Slice(), k + 1) - state -
      config_.dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0);
    command_constraints.push_back(command(0));
    command_constraints.push_back(command(1));
    // 将发布端的斜率限制纳入预测，避免优化器依赖实际无法执行的瞬时反向换舵。
    rate_constraints.push_back(delta(0));
    rate_constraints.push_back(delta(1));
    // 实际转艏率包络包含低速余量，允许动力学惯性；不等同于严格最小半径。
    const casadi::SX next_state = states(casadi::Slice(), k + 1);
    const casadi::SX yaw_limit = config_.yaw_rate_envelope_allowance_radps +
      fabs(next_state(3)) /
      config_.yaw_rate_envelope_radius_m;
    motion_constraints.push_back(next_state(5) - yaw_limit);
    motion_constraints.push_back(-next_state(5) - yaw_limit);
  }

  constraints.insert(
    constraints.end(), command_constraints.begin(), command_constraints.end());
  constraints.insert(
    constraints.end(), motion_constraints.begin(), motion_constraints.end());
  constraints.insert(constraints.end(), rate_constraints.begin(), rate_constraints.end());

  const casadi::SX decision = casadi::SX::vertcat({
      casadi::SX::reshape(states, states.numel(), 1),
      casadi::SX::reshape(controls, controls.numel(), 1)});
  const casadi::SX constraint = casadi::SX::vertcat(constraints);
  const casadi::SX costs = casadi::SX::vertcat(cost_terms);
  const casadi::SX objective = casadi::SX::sum1(costs);
  cost_function_ = casadi::Function("nmpc_costs", {decision, parameters}, {costs});
  casadi::SXDict nlp{{"x", decision}, {"p", parameters},
    {"f", objective}, {"g", constraint}};
  casadi::Dict options;
  options["ipopt.print_level"] = 0;
  options["ipopt.sb"] = "yes";  // 关闭求解器启动横幅，避免覆盖终端状态表格。
  options["print_time"] = false;
  options["ipopt.max_iter"] = config_.solver_max_iterations;
  options["ipopt.tol"] = config_.solver_tolerance;
  solver_ = casadi::nlpsol("nmpc_solver", "ipopt", nlp, options);
  constraint_count_ = static_cast<int>(constraint.size1());
}

}  // namespace usv_nmpc
