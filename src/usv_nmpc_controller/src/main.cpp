/**
 * @file main.cpp
 * @brief 无人船 NMPC 节点入口。
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "usv_nmpc_controller/usv_nmpc_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UsvNmpcNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
