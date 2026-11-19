/**
 * @file nmpc_controller.cpp
 * @brief NMPC 求解、热启动和诊断代价计算实现。
 */

#include "usv_nmpc_controller/nmpc_controller.hpp"

#include "usv_nmpc_controller/usv_model.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
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
constexpr double kInf = 1e20;

std::vector<double> stateValues(const State & state)
{
  return {state.x, state.y, state.psi, state.u, state.v, state.r};
}

}  // namespace

NMPCController::NMPCController(const NMPCConfig & config)
: config_(config)
{
  validateConfig();
  buildSolver();
}

bool NMPCController::lastSolveFailed() const
{
  return last_solve_failed_;
}

int NMPCController::consecutiveFailures() const
{
  return consecutive_failures_;
}

const std::vector<State> & NMPCController::predictedTrajectory() const
{
  return predicted_trajectory_;
}

State NMPCController::predictedNextState() const
{
  if (predicted_trajectory_.size() > 1U) {
    return predicted_trajectory_[1];
  }
  if (!predicted_trajectory_.empty()) {
    return predicted_trajectory_.front();
  }
  return {};
}

Control NMPCController::predictedFirstCommand() const
{
  if (predicted_controls_.empty()) {
    return {};
  }
  return predicted_controls_.front();
}

const NmpcCostBreakdown & NMPCController::lastCostBreakdown() const
{
  return last_cost_breakdown_;
}

void NMPCController::resetWarmStart()
{
  last_solution_.clear();
  predicted_trajectory_.clear();
  predicted_controls_.clear();
  last_cost_breakdown_ = {};
  // 新任务只清除旧预测序列，保留上周期实际指令以保证控制量连续。
  last_solve_failed_ = false;
  consecutive_failures_ = 0;
}

void NMPCController::setLastCommand(const Control & command)
{
  const Control projected = clampCommand(command, config_);
  last_speed_command_ = projected.speed;
  last_yaw_rate_command_ = projected.yaw_rate;
  has_last_command_ = true;
}

Control NMPCController::feedforwardForReference(const RefPoint & reference) const
{
  return {steadySurgeCommand(reference.u), steadyYawCommand(reference.v, reference.r)};
}

Control clampCommand(const Control & desired, const NMPCConfig & config)
{
  return {std::clamp(desired.speed, config.min_speed_command_mps, config.max_speed_command_mps),
    std::clamp(desired.yaw_rate, -config.max_yaw_rate_command_radps,
    config.max_yaw_rate_command_radps)};
}

Control limitCommandChange(
  const Control & desired, const Control & previous, double dt, const NMPCConfig & config)
{
  const Control bounded = clampCommand(desired, config);
  const double speed_accel_step = config.max_speed_command_accel_mps2 * dt;
  const double speed_decel_step = config.max_speed_command_decel_mps2 * dt;
  const double yaw_step = config.max_yaw_rate_command_accel_radps2 * dt;
  return {std::clamp(bounded.speed, previous.speed - speed_decel_step,
      previous.speed + speed_accel_step),
    std::clamp(bounded.yaw_rate, previous.yaw_rate - yaw_step, previous.yaw_rate + yaw_step)};
}

std::vector<double> NMPCController::buildInitialGuess(
  const State & state,
  const std::vector<RefPoint> & references,
  const Control & previous_command, double command_dt_s) const
{
  const int horizon = config_.horizon_steps;
  const int state_variables = kStateSize * (horizon + 1);
  const int variables = state_variables + kControlSize * horizon;
  std::vector<double> guess(static_cast<std::size_t>(variables), 0.0);
  State predicted = state;
  const bool can_shift_previous_solution =
    last_solution_.size() == guess.size();
  Control previous = previous_command;

  for (int k = 0; k <= horizon; ++k) {
    const auto values = stateValues(predicted);
    std::copy(values.begin(), values.end(), guess.begin() + k * kStateSize);
    if (k == horizon) {
      break;
    }

    Control desired;
    if (can_shift_previous_solution) {
      const int prior_index = state_variables +
        2 * std::min(k + 1, horizon - 1);
      desired = Control{last_solution_[prior_index], last_solution_[prior_index + 1]};
    } else {
      desired = feedforwardForReference(references[static_cast<std::size_t>(k)]);
    }
    const Control command = limitCommandChange(
      desired, previous, k == 0 ? command_dt_s : config_.dt, config_);
    previous = command;
    const int index = state_variables + k * kControlSize;
    guess[index] = command.speed;
    guess[index + 1] = command.yaw_rate;
    predicted = USVModel::stepRK4(predicted, command, config_.dt);
  }
  return guess;
}

void NMPCController::updateCostBreakdown(
  const casadi::DM & solution, const std::vector<double> & parameters)
{
  const auto costs = cost_function_(casadi::DMVector{solution,
        casadi::DM(parameters)})[0].get_elements();
  last_cost_breakdown_ = {};
  last_cost_breakdown_.solver_succeeded = true;
  last_cost_breakdown_.stage_lateral = costs[0];
  last_cost_breakdown_.stage_course = costs[1];
  last_cost_breakdown_.stage_u = costs[2];
  last_cost_breakdown_.stage_v = costs[3];
  last_cost_breakdown_.stage_r = costs[4];
  last_cost_breakdown_.stage_delta_control = costs[5];
  last_cost_breakdown_.total = std::accumulate(costs.begin(), costs.end(), 0.0);
}

Control NMPCController::solve(
  const State & initial_state,
  const std::vector<RefPoint> & references,
  double command_dt_s)
{
  if (references.empty()) {
    throw std::invalid_argument("参考轨迹为空");
  }
  const int horizon = config_.horizon_steps;
  const double first_command_dt = command_dt_s > 0.0 ? command_dt_s : config_.dt;
  std::vector<RefPoint> refs = references;
  refs.resize(static_cast<std::size_t>(horizon + 1), refs.back());
  const int state_variables = kStateSize * (horizon + 1);
  const int variables = state_variables + kControlSize * horizon;
  const Control steady_command = clampCommand({
      steadySurgeCommand(initial_state.u), steadyYawCommand(initial_state.v, initial_state.r)},
    config_);
  const double previous_speed_command = has_last_command_ ? last_speed_command_ :
    steady_command.speed;
  const double previous_yaw_rate_command = has_last_command_ ? last_yaw_rate_command_ :
    steady_command.yaw_rate;

  std::vector<double> parameters = stateValues(initial_state);
  parameters.reserve(kParameterOffset + kReferenceSize * (horizon + 1));
  parameters.push_back(previous_speed_command);
  parameters.push_back(previous_yaw_rate_command);
  for (int k = 0; k <= horizon; ++k) {
    const auto & ref = refs[static_cast<std::size_t>(k)];
    parameters.insert(parameters.end(),
      {ref.x, ref.y, ref.psi, ref.u, ref.v, ref.r});
  }
  std::vector<double> lower(variables, -kInf);
  std::vector<double> upper(variables, kInf);
  for (int k = 1; k <= horizon; ++k) {
    // 状态边界与指令幅值分开：动力学响应可暂时超过指令对应的稳态速度。
    lower[k * kStateSize + 3] = -config_.max_reverse_speed_mps;
    upper[k * kStateSize + 3] = config_.max_surge_state_mps;
    lower[k * kStateSize + 4] = -config_.max_sway_state_mps;
    upper[k * kStateSize + 4] = config_.max_sway_state_mps;
    lower[k * kStateSize + 5] = -config_.max_yaw_rate_state_radps;
    upper[k * kStateSize + 5] = config_.max_yaw_rate_state_radps;
  }
  std::vector<double> lower_g(constraint_count_, 0.0);
  std::vector<double> upper_g(constraint_count_, 0.0);
  const int equality_count = kStateSize * (horizon + 1);
  for (int k = 0; k < horizon; ++k) {
    const int index = equality_count + 2 * k;
    lower_g[index] = config_.min_speed_command_mps;
    upper_g[index] = config_.max_speed_command_mps;
    lower_g[index + 1] = -config_.max_yaw_rate_command_radps;
    upper_g[index + 1] = config_.max_yaw_rate_command_radps;
  }
  // 转艏率包络约束（每步 2 条，排在指令幅值约束之后）：g = ±r − limit ≤ 0，
  //   即上界 0、下界 −∞。顺序必须与 buildSolver 的追加顺序一致：
  //   [动力学等式 6*(h+1)] [指令幅值 2*h] [转艏率包络 2*h]。
  const int envelope_offset = equality_count + 2 * horizon;
  for (int k = 0; k < 2 * horizon; ++k) {
    const int index = envelope_offset + k;
    lower_g[index] = -kInf;
    upper_g[index] = 0.0;
  }
  const int rate_offset = envelope_offset + 2 * horizon;
  for (int k = 0; k < horizon; ++k) {
    const double interval = k == 0 ? first_command_dt : config_.dt;
    const double speed_accel_step = config_.max_speed_command_accel_mps2 * interval;
    const double speed_decel_step = config_.max_speed_command_decel_mps2 * interval;
    const double yaw_step = config_.max_yaw_rate_command_accel_radps2 * interval;
    lower_g[rate_offset + 2 * k] = -speed_decel_step;
    upper_g[rate_offset + 2 * k] = speed_accel_step;
    lower_g[rate_offset + 2 * k + 1] = -yaw_step;
    upper_g[rate_offset + 2 * k + 1] = yaw_step;
  }

  const auto guess = buildInitialGuess(initial_state, refs,
      {previous_speed_command, previous_yaw_rate_command}, first_command_dt);

  casadi::DMDict args;
  args["x0"] = guess;
  args["lbx"] = lower;
  args["ubx"] = upper;
  args["lbg"] = lower_g;
  args["ubg"] = upper_g;
  args["p"] = parameters;
  casadi::DMDict result;
  bool success = false;
  try {
    result = solver_(args);
    success = static_cast<bool>(solver_.stats().at("success"));
  } catch (const std::exception &) {
    // 求解异常与未收敛统一交给下方失败分支，节点会直接发布零指令。
  }
  if (!success) {
    ++consecutive_failures_;
    last_solve_failed_ = true;
    last_solution_.clear();
    predicted_trajectory_.clear();
    predicted_controls_.clear();
    last_cost_breakdown_ = {};
    last_cost_breakdown_.consecutive_failures = consecutive_failures_;
    return {};
  }

  last_solve_failed_ = false;
  consecutive_failures_ = 0;
  last_solution_ = static_cast<std::vector<double>>(result.at("x"));
  predicted_trajectory_.clear();
  predicted_trajectory_.reserve(static_cast<std::size_t>(horizon + 1));
  predicted_controls_.clear();
  predicted_controls_.reserve(static_cast<std::size_t>(horizon));
  for (int k = 0; k <= horizon; ++k) {
    const int index = k * kStateSize;
    predicted_trajectory_.push_back(State{
        last_solution_[index], last_solution_[index + 1], last_solution_[index + 2],
        last_solution_[index + 3], last_solution_[index + 4], last_solution_[index + 5]});
  }
  for (int k = 0; k < horizon; ++k) {
    const int index = state_variables + k * kControlSize;
    predicted_controls_.push_back({last_solution_[index], last_solution_[index + 1]});
  }
  updateCostBreakdown(result.at("x"), parameters);
  return {last_solution_[state_variables], last_solution_[state_variables + 1]};
}

}  // namespace usv_nmpc
