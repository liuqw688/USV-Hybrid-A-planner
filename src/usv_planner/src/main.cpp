/**
 * @file main.cpp
 * @brief 无人船路径规划节点入口。
 */

#include "plan_manage/kino_replan_fsm.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("usv_planner_node");
  fast_planner::KinoReplanFSM planner;
  planner.init(*node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
