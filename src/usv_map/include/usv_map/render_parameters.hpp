/**
 * @file render_parameters.hpp
 * @brief 统一声明主地图和地图编辑工具的渲染参数。
 */

#pragma once

#include <cstdint>
#include <rclcpp/node.hpp>

#include "usv_map/renderer.hpp"

namespace usv_map
{

/**
 * @brief 声明地图主节点和暗礁编辑器共用的渲染参数。
 * @param[in,out] node 用于声明并读取参数的 ROS 节点。
 * @param[in] config 参数默认值；未设置的参数沿用此值。
 * @return 应用节点参数后的渲染配置。
 */
inline RenderConfig declare_render_config(rclcpp::Node & node, RenderConfig config = {})
{
  config.line_width = node.declare_parameter("line_width", config.line_width);
  config.point_size = node.declare_parameter("point_size", config.point_size);
  config.focus_navigation_features = node.declare_parameter(
    "focus_navigation_features", config.focus_navigation_features);
  config.planning_resolution = node.declare_parameter("planning_resolution", config.planning_resolution);
  config.planning_max_cells = node.declare_parameter<int64_t>("planning_max_cells", config.planning_max_cells);
  config.planning_crop_width_fraction = node.declare_parameter(
    "planning_crop_width_fraction", config.planning_crop_width_fraction);
  config.planning_crop_height_fraction = node.declare_parameter(
    "planning_crop_height_fraction", config.planning_crop_height_fraction);
  config.obstacle_inflation = node.declare_parameter("obstacle_inflation", config.obstacle_inflation);
  config.channel_corridor_width = node.declare_parameter("channel_corridor_width", config.channel_corridor_width);
  config.channel_outside_cost = static_cast<int>(node.declare_parameter<int64_t>(
      "channel_outside_cost", config.channel_outside_cost));
  return config;
}

}  // namespace usv_map
