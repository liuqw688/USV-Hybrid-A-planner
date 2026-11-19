/**
 * @file model_node.hpp
 * @brief 接收控制命令和地图原点，发布无人艇仿真状态。
 */

#ifndef USV_MODEL__MODEL_NODE_HPP_
#define USV_MODEL__MODEL_NODE_HPP_

#include <chrono>
#include <memory>
#include <random>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "usv_model_sim/dynamics.hpp"

namespace usv_model_sim
{

/** 发布模型状态；地图包负责里程计、轨迹和船体姿态 TF。 */
class ModelNode : public rclcpp::Node
{
public:
  /**
   * @brief 读取模型参数并创建命令订阅、原点订阅与状态发布定时器。
   * @param[in] options ROS 节点选项，可包含参数覆盖。
   * @throws std::invalid_argument 模型或仿真参数无效。
   */
  explicit ModelNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /** @brief 缓存并限幅速度命令，记录接收时刻。 */
  void on_command(const geometry_msgs::msg::Twist::SharedPtr message);
  /** @brief 接收有效地理原点，启用模型状态发布。 */
  void on_origin(const sensor_msgs::msg::NavSatFix::SharedPtr message);
  /** @brief 按经过时间积分模型，并在原点有效时发布状态。 */
  void update();
  /** @brief 发布六状态及带配置噪声的地理位置、航向观测。 */
  void publish_state();
  /** @brief 返回当前有效命令；命令缺失、超时或时间回退时返回零值。 */
  Command active_command(const rclcpp::Time & now) const;

  double max_speed_mps_{5.0};
  double max_yaw_rate_rad_s_{15.0 * kPi / 180.0};
  double command_timeout_sec_{0.5};
  double update_rate_hz_{20.0};
  double position_noise_stddev_m_{0.25};
  double heading_noise_stddev_rad_{kPi / 180.0};
  std::mt19937 noise_generator_{std::random_device{}()};
  std::normal_distribution<double> standard_normal_{0.0, 1.0};

  Dynamics dynamics_;
  State state_;
  Command command_;
  bool has_command_{false};
  rclcpp::Time last_command_time_;
  rclcpp::Time last_update_time_;
  double simulation_time_sec_{0.0};

  bool has_origin_{false};
  double origin_latitude_deg_{0.0};
  double origin_longitude_deg_{0.0};
  double origin_altitude_m_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_model_sim

#endif  // USV_MODEL__MODEL_NODE_HPP_
