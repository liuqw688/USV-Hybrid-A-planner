/**
 * @file kinodynamic_astar_ros.cpp
 * @brief 动力学 A* 的 ROS 2 参数适配层。
 */

#include "path_searching/kinodynamic_astar_ros.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace fast_planner
{

void configureKinodynamicAstar(rclcpp::Node& node, KinodynamicAstar& planner)
{
  const auto read_double = [&node](const std::string& name, double fallback) {
    return node.has_parameter(name) ? node.get_parameter(name).as_double() :
        node.declare_parameter<double>(name, fallback);
  };
  KinodynamicAstarConfig config;
  config.max_tau = node.declare_parameter<double>("search/max_tau", config.max_tau);
  config.init_max_tau = node.declare_parameter<double>("search/init_max_tau", config.init_max_tau);
  config.rk4_steps = node.declare_parameter<int>("search/rk4_steps", config.rk4_steps);
  config.search_time_limit = node.declare_parameter<double>("search/time_limit", config.search_time_limit);
  config.max_velocity = node.declare_parameter<double>("search/max_vel", config.max_velocity);
  config.max_acceleration = node.declare_parameter<double>("search/max_acc", config.max_acceleration);
  config.max_u = read_double("search/max_u", config.max_u);
  config.max_v = read_double("search/max_v", config.max_v);
  config.max_r = read_double("search/max_r", config.max_r);
  config.min_v_cmd = node.declare_parameter<double>("search/min_v_cmd", config.min_v_cmd);
  config.max_v_cmd = node.declare_parameter<double>("search/max_v_cmd", config.max_v_cmd);
  config.max_w_cmd = node.declare_parameter<double>("search/max_w_cmd", config.max_w_cmd);
  config.search_clearance = read_double("manager/safety_replan_distance", config.search_clearance);
  config.obstacle_slowdown_distance = read_double("manager/obstacle_slowdown_distance", 5.0);
  config.obstacle_max_speed = read_double("manager/obstacle_max_speed", 1.0);
  config.min_turn_radius = read_double("manager/min_turn_radius", 8.0);
  config.max_ground_speed = node.declare_parameter<double>("search/max_ground_speed", config.max_ground_speed);
  config.max_yaw_rate = node.declare_parameter<double>("search/max_yaw_rate", config.max_yaw_rate);
  config.time_weight = node.declare_parameter<double>("search/w_time", config.time_weight);
  config.horizon = node.declare_parameter<double>("search/horizon", config.horizon);
  config.shot_trigger_distance = node.declare_parameter<double>(
      "search/shot_trigger_distance", config.shot_trigger_distance);
  config.position_resolution = node.declare_parameter<double>(
      "search/resolution_astar", config.position_resolution);
  config.yaw_bins = node.declare_parameter<int>("search/yaw_bins", config.yaw_bins);
  config.velocity_resolution <<
      node.declare_parameter<double>("search/u_resolution", config.velocity_resolution(0)),
      node.declare_parameter<double>("search/v_resolution", config.velocity_resolution(1)),
      node.declare_parameter<double>("search/r_resolution", config.velocity_resolution(2));
  config.time_resolution = node.declare_parameter<double>("search/time_resolution", config.time_resolution);
  config.heuristic_weight = node.declare_parameter<double>("search/lambda_heu", config.heuristic_weight);
  config.node_capacity = node.declare_parameter<int>("search/allocate_num", config.node_capacity);
  config.collision_check_count = node.declare_parameter<int>("search/check_num", config.collision_check_count);
  config.optimistic = node.declare_parameter<bool>("search/optimistic", config.optimistic);
  config.shot_durations = node.declare_parameter<std::vector<double>>(
      "search/shot_durations", config.shot_durations);
  config.shot_cruise_speed = node.declare_parameter<double>(
      "search/shot_cruise_speed", config.shot_cruise_speed);
  config.shot_integration_step = node.declare_parameter<double>(
      "search/shot_integration_step", config.shot_integration_step);
  config.shot_collision_step = node.declare_parameter<double>(
      "search/shot_collision_step", config.shot_collision_step);
  config.shot_position_tolerance = node.declare_parameter<double>(
      "search/shot_pos_tolerance", config.shot_position_tolerance);
  config.shot_yaw_tolerance = node.declare_parameter<double>(
      "search/shot_yaw_tolerance", config.shot_yaw_tolerance);
  config.shot_u_tolerance = node.declare_parameter<double>("search/shot_u_tolerance", config.shot_u_tolerance);
  config.shot_v_tolerance = node.declare_parameter<double>("search/shot_v_tolerance", config.shot_v_tolerance);
  config.shot_r_tolerance = node.declare_parameter<double>("search/shot_r_tolerance", config.shot_r_tolerance);
  config.shot_min_v_cmd = node.declare_parameter<double>("search/shot_min_v_cmd", config.shot_min_v_cmd);
  config.shot_max_v_cmd = node.declare_parameter<double>("search/shot_max_v_cmd", config.shot_max_v_cmd);
  config.shot_max_w_cmd = node.declare_parameter<double>("search/shot_max_w_cmd", config.shot_max_w_cmd);
  config.shot_command_scale = node.declare_parameter<double>(
      "search/shot_command_scale", config.shot_command_scale);
  config.shot_max_u = node.declare_parameter<double>("search/shot_max_u", config.shot_max_u);
  config.shot_max_v = node.declare_parameter<double>("search/shot_max_v", config.shot_max_v);
  config.shot_max_r = node.declare_parameter<double>("search/shot_max_r", config.shot_max_r);
  config.shot_map_height = node.declare_parameter<double>("search/shot_map_height", config.shot_map_height);
  config.shot_fd_epsilon = node.declare_parameter<double>("search/shot_fd_epsilon", config.shot_fd_epsilon);
  config.shot_damping = node.declare_parameter<double>("search/shot_damping", config.shot_damping);
  config.shot_time_limit = node.declare_parameter<double>("search/shot_time_limit", config.shot_time_limit);
  config.shot_max_iterations = node.declare_parameter<int>(
      "search/shot_max_iterations", config.shot_max_iterations);
  config.velocity_margin = node.declare_parameter<double>("search/vel_margin", config.velocity_margin);

  if (!std::isfinite(config.max_ground_speed) || config.max_ground_speed <= 0.0 ||
      !std::isfinite(config.max_yaw_rate) || config.max_yaw_rate <= 0.0 ||
      !std::isfinite(config.time_weight) || config.time_weight < 0.0)
  {
    RCLCPP_WARN(node.get_logger(),
        "USV heuristic disabled: configure positive speed/yaw-rate bounds and a nonnegative time weight.");
  }
  planner.setConfig(config);
}

}  // namespace fast_planner
