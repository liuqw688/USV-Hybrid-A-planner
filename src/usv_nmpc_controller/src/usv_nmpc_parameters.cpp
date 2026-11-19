/**
 * @file usv_nmpc_parameters.cpp
 * @brief 无人船 NMPC ROS 参数读取实现。
 */

#include "usv_nmpc_controller/usv_nmpc_node.hpp"

#include <cmath>
#include <stdexcept>

void UsvNmpcNode::loadParameters()
{
  odom_topic_ = declare_parameter("odom_topic", odom_topic_);
  bspline_topic_ = declare_parameter("bspline_topic", bspline_topic_);
  cmd_topic_ = declare_parameter("cmd_topic", cmd_topic_);
  tracking_finished_topic_ = declare_parameter(
    "tracking_finished_topic", tracking_finished_topic_);
  nmpc_diagnostics_topic_ = declare_parameter(
    "nmpc_diagnostics_topic", nmpc_diagnostics_topic_);
  control_frequency_ = declare_parameter("control_frequency", control_frequency_);
  finish_distance_m_ = declare_parameter("finish_distance_m", finish_distance_m_);
  finish_speed_mps_ = declare_parameter("finish_speed_mps", finish_speed_mps_);
  // 终点距离与合速度需连续满足确认时间。
  finish_stable_duration_s_ = declare_parameter(
    "finish_stable_duration_s", finish_stable_duration_s_);
  bspline_sample_period_s_ = declare_parameter(
    "reference.bspline_sample_period_s", bspline_sample_period_s_);
  terminal_table_enabled_ = declare_parameter(
    "terminal_table.enabled", terminal_table_enabled_);
  terminal_table_refresh_period_s_ = declare_parameter(
    "terminal_table.refresh_period_s", terminal_table_refresh_period_s_);

  if (control_frequency_ <= 0.0 ||
    finish_distance_m_ <= 0.0 ||
    finish_speed_mps_ <= 0.0 ||
    finish_stable_duration_s_ <= 0.0 ||
    state_timeout_s_ <= 0.0 ||
    reference_stale_timeout_s_ <= 0.0 ||
    bspline_sample_period_s_ <= 0.0 ||
    !std::isfinite(terminal_table_refresh_period_s_) ||
    terminal_table_refresh_period_s_ <= 0.0)
  {
    throw std::invalid_argument("NMPC 节点参数无效");
  }
}

usv_nmpc::NMPCConfig UsvNmpcNode::loadNmpcConfig()
{
  usv_nmpc::NMPCConfig config;
  config.max_speed_command_accel_mps2 = declare_parameter(
    "nmpc.max_speed_command_accel_mps2", config.max_speed_command_accel_mps2);
  config.max_speed_command_decel_mps2 = declare_parameter(
    "nmpc.max_speed_command_decel_mps2", config.max_speed_command_decel_mps2);
  config.max_yaw_rate_command_accel_radps2 = declare_parameter(
    "nmpc.max_yaw_rate_command_accel_radps2", config.max_yaw_rate_command_accel_radps2);
  config.horizon_steps = declare_parameter("nmpc.horizon_steps", config.horizon_steps);
  config.dt = declare_parameter("nmpc.dt", config.dt);
  config.weight_lateral = declare_parameter("nmpc.weight_lateral", config.weight_lateral);
  config.weight_course = declare_parameter("nmpc.weight_course", config.weight_course);
  config.weight_u = declare_parameter("nmpc.weight_u", config.weight_u);
  config.max_reverse_speed_mps = declare_parameter(
    "nmpc.max_reverse_speed_mps", config.max_reverse_speed_mps);
  config.max_surge_state_mps = declare_parameter(
    "nmpc.max_surge_state_mps", config.max_surge_state_mps);
  config.max_sway_state_mps = declare_parameter(
    "nmpc.max_sway_state_mps", config.max_sway_state_mps);
  config.max_yaw_rate_state_radps = declare_parameter(
    "nmpc.max_yaw_rate_state_radps", config.max_yaw_rate_state_radps);
  config.weight_v = declare_parameter("nmpc.weight_v", config.weight_v);
  config.weight_r = declare_parameter("nmpc.weight_r", config.weight_r);
  config.weight_surge_delta = declare_parameter(
    "nmpc.weight_surge_delta", config.weight_surge_delta);
  config.weight_yaw_rate_delta = declare_parameter(
    "nmpc.weight_yaw_rate_delta", config.weight_yaw_rate_delta);
  // 包络保留低速角速度余量，不将其误称为严格的最小转弯半径。
  config.yaw_rate_envelope_radius_m = declare_parameter(
    "nmpc.yaw_rate_envelope_radius_m", config.yaw_rate_envelope_radius_m);
  config.yaw_rate_envelope_allowance_radps = declare_parameter(
    "nmpc.yaw_rate_envelope_allowance_radps", config.yaw_rate_envelope_allowance_radps);
  config.min_speed_command_mps = declare_parameter("nmpc.min_speed_command_mps",
    config.min_speed_command_mps);
  // 优化器和发布端统一使用 config 的边界，避免额外固定限幅覆盖配置。
  config.max_speed_command_mps = declare_parameter("nmpc.max_speed_command_mps",
    config.max_speed_command_mps);
  config.max_yaw_rate_command_radps = declare_parameter("nmpc.max_yaw_rate_command_radps",
    config.max_yaw_rate_command_radps);
  config.solver_max_iterations = declare_parameter(
    "nmpc.solver_max_iterations", config.solver_max_iterations);
  config.solver_tolerance = declare_parameter(
    "nmpc.solver_tolerance", config.solver_tolerance);
  return config;
}
