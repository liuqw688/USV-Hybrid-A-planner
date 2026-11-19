/**
 * @file bspline_optimizer_ros.cpp
 * @brief ROS 2 参数适配层。
 */

#include "bspline_opt/bspline_optimizer_ros.hpp"

#include <array>
#include <string>

namespace fast_planner
{
namespace
{

double declareDouble(rclcpp::Node& node, const std::string& name)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<double>(name, -1.0);
  }
  return node.get_parameter(name).as_double();
}

int declareInteger(rclcpp::Node& node, const std::string& name)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<int>(name, -1);
  }
  return node.get_parameter(name).as_int();
}

}  // namespace

void configureBsplineOptimizer(rclcpp::Node& node, BsplineOptimizer& optimizer)
{
  BsplineOptimizerConfig config;
  const std::array<std::string, 8> lambda_names{
    "optimization/lambda1", "optimization/lambda2", "optimization/lambda3",
    "optimization/lambda4", "optimization/lambda5", "optimization/lambda6",
    "optimization/lambda7", "optimization/lambda8"};
  std::array<double*, 8> lambdas{
    &config.lambda1, &config.lambda2, &config.lambda3, &config.lambda4,
    &config.lambda5, &config.lambda6, &config.lambda7, &config.lambda8};
  for (std::size_t i = 0; i < lambda_names.size(); ++i) {
    *lambdas[i] = declareDouble(node, lambda_names[i]);
  }

  config.distance_margin = declareDouble(node, "optimization/dist0");
  config.max_velocity = declareDouble(node, "optimization/max_vel");
  config.max_acceleration = declareDouble(node, "optimization/max_acc");
  config.max_global_velocity = declareDouble(node, "manager/max_global_vel");
  config.max_yaw_rate = declareDouble(node, "manager/max_yaw_rate");
  config.min_turn_radius = declareDouble(node, "manager/min_turn_radius");
  config.channel_weight = node.has_parameter("optimization/channel_weight") ?
      node.get_parameter("optimization/channel_weight").as_double() :
      node.declare_parameter<double>("optimization/channel_weight", 2.0);
  config.max_global_acceleration = declareDouble(node, "manager/max_global_acc");
  config.max_yaw_acceleration = declareDouble(node, "manager/max_yaw_acc");
  config.yaw_rate_weight = declareDouble(node, "optimization/yaw_rate_weight");
  config.yaw_acceleration_weight = declareDouble(node, "optimization/yaw_acc_weight");
  config.yaw_smoothness_weight = declareDouble(node, "optimization/yaw_smoothness_weight");
  config.yaw_endpoint_weight = declareDouble(node, "optimization/yaw_endpoint_weight");
  config.yaw_guide_weight = declareDouble(node, "optimization/yaw_guide_weight");
  config.yaw_waypoint_weight = declareDouble(node, "optimization/yaw_waypoint_weight");
  config.yaw_bound = declareDouble(node, "optimization/yaw_bound");
  config.visibility_min = declareDouble(node, "optimization/visib_min");
  config.visibility_spacing = declareDouble(node, "optimization/dlmin");
  config.visibility_weight = declareDouble(node, "optimization/wnl");

  const std::array<std::string, 4> iteration_names{
    "optimization/max_iteration_num1", "optimization/max_iteration_num2",
    "optimization/max_iteration_num3", "optimization/max_iteration_num4"};
  const std::array<std::string, 4> time_names{
    "optimization/max_iteration_time1", "optimization/max_iteration_time2",
    "optimization/max_iteration_time3", "optimization/max_iteration_time4"};
  for (std::size_t i = 0; i < config.max_iterations.size(); ++i) {
    config.max_iterations[i] = declareInteger(node, iteration_names[i]);
    config.max_iteration_time[i] = declareDouble(node, time_names[i]);
  }

  config.quadratic_algorithm = declareInteger(node, "optimization/algorithm1");
  config.general_algorithm = declareInteger(node, "optimization/algorithm2");
  config.spline_order = declareInteger(node, "optimization/order");
  optimizer.setConfig(config);
}

}  // namespace fast_planner
