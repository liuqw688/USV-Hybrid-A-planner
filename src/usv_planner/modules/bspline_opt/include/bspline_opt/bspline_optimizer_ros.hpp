/**
 * @file bspline_optimizer_ros.hpp
 * @brief ROS 2 参数适配接口。
 */

#ifndef BSPLINE_OPT__BSPLINE_OPTIMIZER_ROS_HPP_
#define BSPLINE_OPT__BSPLINE_OPTIMIZER_ROS_HPP_

#include <rclcpp/rclcpp.hpp>
#include "bspline_opt/bspline_optimizer.h"

namespace fast_planner
{

/**
 * @brief 从 ROS 2 参数配置 B 样条优化器。
 * @param[in] node 提供优化权重、动力学边界和求解器参数的 ROS 节点。
 * @param[out] optimizer 接收配置的优化器实例。
 */
void configureBsplineOptimizer(rclcpp::Node& node, BsplineOptimizer& optimizer);

}  // namespace fast_planner

#endif  // BSPLINE_OPT__BSPLINE_OPTIMIZER_ROS_HPP_
