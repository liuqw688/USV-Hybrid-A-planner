/**
 * @file kinodynamic_astar_ros.hpp
 * @brief 将 ROS 2 参数加载到纯 C++ 动力学 A* 配置。
 */

#ifndef PATH_SEARCHING__KINODYNAMIC_ASTAR_ROS_HPP_
#define PATH_SEARCHING__KINODYNAMIC_ASTAR_ROS_HPP_

#include <rclcpp/rclcpp.hpp>
#include "path_searching/kinodynamic_astar.h"

namespace fast_planner
{

/** @brief 从 ROS 2 参数初始化动力学搜索器。 */
void configureKinodynamicAstar(rclcpp::Node& node, KinodynamicAstar& planner);

}  // namespace fast_planner

#endif  // PATH_SEARCHING__KINODYNAMIC_ASTAR_ROS_HPP_
