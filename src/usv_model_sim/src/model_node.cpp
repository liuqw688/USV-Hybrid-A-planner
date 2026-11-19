/**
 * @file model_node.cpp
 * @brief 控制超时处理、仿真积分及 19 字段状态组帧。
 */

#include "usv_model_sim/model_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace usv_model_sim
{
namespace
{

constexpr double kEarthRadiusM = 6378137.0;
constexpr std::size_t kStateFieldCount = 19;

double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

}  // namespace

ModelNode::ModelNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("usv_model_node", options), last_command_time_(0, 0, RCL_ROS_TIME),
  last_update_time_(0, 0, RCL_ROS_TIME)
{
  max_speed_mps_ = std::max(0.0, declare_parameter("max_speed_mps", 5.0));
  max_yaw_rate_rad_s_ = std::max(0.0, declare_parameter("max_yaw_rate_deg_s", 15.0)) * kPi / 180.0;
  state_.east = declare_parameter("initial_east_m", 0.0);
  state_.north = declare_parameter("initial_north_m", 0.0);
  if (!std::isfinite(state_.east) || !std::isfinite(state_.north)) {
    throw std::invalid_argument("initial_east_m 和 initial_north_m 必须是有限数值");
  }
  const double initial_heading_deg = declare_parameter("initial_heading_deg", 90.0);
  state_.yaw = std::remainder(-initial_heading_deg * kPi / 180.0, 2.0 * kPi);
  command_timeout_sec_ = std::max(0.0, declare_parameter("command_timeout_sec", 0.5));
  update_rate_hz_ = std::max(1.0, declare_parameter("update_rate_hz", 20.0));
  position_noise_stddev_m_ = declare_parameter("position_noise_stddev_m", 0.25);
  heading_noise_stddev_rad_ = declare_parameter("heading_noise_stddev_deg", 1.0) * kPi / 180.0;
  const std::int64_t noise_seed = declare_parameter("noise_seed", std::int64_t{-1});
  if (noise_seed < -1 || noise_seed > 4294967295LL) {
    throw std::invalid_argument("noise_seed must be -1 or an unsigned 32-bit integer");
  }
  if (noise_seed >= 0) {
    noise_generator_.seed(static_cast<std::mt19937::result_type>(noise_seed));
  }
  if (!std::isfinite(position_noise_stddev_m_) || position_noise_stddev_m_ < 0.0 ||
    !std::isfinite(heading_noise_stddev_rad_) || heading_noise_stddev_rad_ < 0.0)
  {
    throw std::invalid_argument(
            "position and heading noise standard deviations must be finite and nonnegative");
  }
  const auto origin_topic = declare_parameter("origin_topic", std::string("/fix"));
  const auto cmd_topic = declare_parameter("cmd_topic", std::string("/cmd"));
  const auto state_topic = declare_parameter("state_topic", std::string("/USV_state"));
  dynamics_ = Dynamics(Parameters{max_speed_mps_, max_yaw_rate_rad_s_});

  command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {on_command(message);});
  origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    origin_topic, rclcpp::QoS(1).reliable(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr message) {on_origin(message);});
  state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(state_topic, rclcpp::QoS(10));

  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / update_rate_hz_));
  timer_ = create_wall_timer(std::max(period, std::chrono::milliseconds(1)), [this]() {update();});

  RCLCPP_INFO(
    get_logger(), "USV model ready: cmd=%s, state=%s, max speed=%.2f m/s, max yaw rate=%.2f deg/s",
    cmd_topic.c_str(), state_topic.c_str(), max_speed_mps_,
    max_yaw_rate_rad_s_ * 180.0 / kPi);
  RCLCPP_INFO(get_logger(), "Waiting for map origin on %s before publishing state", origin_topic.c_str());
}

void ModelNode::on_command(const geometry_msgs::msg::Twist::SharedPtr message)
{
  command_.forward_speed = finite_or_zero(message->linear.x);
  command_.yaw_rate = finite_or_zero(message->angular.z);
  command_ = dynamics_.limit_command(command_);
  last_command_time_ = now();
  has_command_ = true;
}

void ModelNode::on_origin(const sensor_msgs::msg::NavSatFix::SharedPtr message)
{
  if (!std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
    std::abs(message->latitude) > 90.0 || std::abs(message->longitude) > 180.0)
  {
    return;
  }
  origin_latitude_deg_ = message->latitude;
  origin_longitude_deg_ = message->longitude;
  origin_altitude_m_ = std::isfinite(message->altitude) ? message->altitude : 0.0;
  has_origin_ = true;
}

Command ModelNode::active_command(const rclcpp::Time & current_time) const
{
  if (!has_command_ || command_timeout_sec_ <= 0.0) {
    return has_command_ ? command_ : Command{};
  }
  const double age = (current_time - last_command_time_).seconds();
  return age >= 0.0 && age <= command_timeout_sec_ ? command_ : Command{};
}

void ModelNode::update()
{
  const auto stamp = now();
  double dt = last_update_time_.nanoseconds() > 0 ? (stamp - last_update_time_).seconds() :
    1.0 / update_rate_hz_;
  last_update_time_ = stamp;
  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = 1.0 / update_rate_hz_;
  }
  dt = std::min(dt, 0.25);
  dynamics_.step(state_, active_command(stamp), dt);
  simulation_time_sec_ += dt;
  publish_state();
}

void ModelNode::publish_state()
{
  if (!has_origin_) {
    return;
  }
  const double east_velocity =
    -state_.u * std::sin(state_.yaw) + state_.v * std::cos(state_.yaw);
  const double north_velocity =
    state_.u * std::cos(state_.yaw) + state_.v * std::sin(state_.yaw);

  // 东、北、天各轴先加入米制独立白噪声，再转换为经纬度；真值不加噪声。
  const double origin_latitude_rad = origin_latitude_deg_ * kPi / 180.0;
  const double longitude_scale = kEarthRadiusM * std::max(1e-9, std::cos(origin_latitude_rad));
  const std::array<double, 3> truth_position{
    origin_latitude_rad + state_.north / kEarthRadiusM,
    origin_longitude_deg_ * kPi / 180.0 + state_.east / longitude_scale,
    origin_altitude_m_};
  const auto noise_m = [this]() {
      return position_noise_stddev_m_ * standard_normal_(noise_generator_);
    };
  const double east_error = noise_m();
  const double north_error = noise_m();
  const double up_error = noise_m();
  const std::array<double, 3> position{
    truth_position[0] + north_error / kEarthRadiusM,
    truth_position[1] + east_error / longitude_scale,
    truth_position[2] + up_error};

  std_msgs::msg::Float64MultiArray message;
  message.data.reserve(kStateFieldCount);
  // 动力学、姿态和航向真值统一使用北为零、向西为正的 [-pi, pi] 角。
  const double ins_yaw = std::remainder(state_.yaw, 2.0 * kPi);
  // 每帧航向测量加入独立零均值高斯噪声；动力学和字段 [16] 保留真值。
  const double measured_yaw = std::remainder(
    ins_yaw + heading_noise_stddev_rad_ * standard_normal_(noise_generator_), 2.0 * kPi);
  const std::array<double, 3> attitude{0.0, 0.0, measured_yaw};
  const std::array<double, 3> velocity{east_velocity, north_velocity, 0.0};
  // 模型定义 v 向右；输出右前上速度为 [v, u, 0]。
  const std::array<double, 3> body_velocity{state_.v, state_.u, 0.0};
  message.data.insert(message.data.end(), attitude.begin(), attitude.end());
  message.data.insert(message.data.end(), velocity.begin(), velocity.end());
  message.data.insert(message.data.end(), position.begin(), position.end());
  message.data.insert(message.data.end(), body_velocity.begin(), body_velocity.end());
  message.data.push_back(finite_or_zero(state_.yaw_rate));
  message.data.insert(message.data.end(), truth_position.begin(), truth_position.end());
  message.data.push_back(ins_yaw);  // 航向真值，rad。
  message.data.push_back(4.0);  // 仿真状态固定为 4。
  message.data.push_back(finite_or_zero(simulation_time_sec_));
  for (auto & value : message.data) {
    value = finite_or_zero(value);
  }
  state_pub_->publish(message);
}

}  // namespace usv_model_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_model_sim::ModelNode>());
  rclcpp::shutdown();
  return 0;
}
