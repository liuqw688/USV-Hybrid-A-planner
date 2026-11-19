/**
 * @file odometry_node.cpp
 * @brief 将船舶状态周期性发布为里程计、轨迹和船体 TF。
 */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "usv_map/vessel_track.hpp"

namespace usv_map
{

/** @brief 将船舶融合状态转换为 ENU 里程计，并维护 RFU 船体变换和历史轨迹。 */
class OdometryNode : public rclcpp::Node
{
public:
  /**
   * @brief 配置状态缓存、里程计与历史轨迹的独立发布周期。
   * @param[in] options ROS 节点选项，可覆盖轨迹点数、位置滤波及发布周期参数。
   * @throws std::invalid_argument 轨迹容量、滤波参数或发布周期无效。
   */
  explicit OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("usv_odometry_node", options), origin_tf_(*this), vessel_tf_(*this)
  {
    const auto min_distance = declare_parameter("path_min_distance_m", 0.5);
    const auto max_points = declare_parameter("path_max_points", int64_t{10000});
    const auto position_measurement_alpha = declare_parameter(
      "position_measurement_alpha", 1.0);
    const auto odom_publish_period = declare_parameter("odom_publish_period_sec", 0.05);
    const auto path_publish_period = declare_parameter("publish_period_sec", 1.0);
    if (max_points < 2) {
      throw std::invalid_argument("path_max_points must be >= 2");
    }
    if (!std::isfinite(odom_publish_period) || odom_publish_period <= 0.0 ||
      !std::isfinite(path_publish_period) || path_publish_period <= 0.0)
    {
      throw std::invalid_argument("odometry and path publish periods must be finite and positive");
    }
    track_ = VesselTrack(
      min_distance, static_cast<std::size_t>(max_points), position_measurement_alpha);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/USV_path", rclcpp::QoS(1).reliable().transient_local());
    origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", rclcpp::QoS(1).reliable(),
      [this](const sensor_msgs::msg::NavSatFix & origin) {on_origin(origin);});
    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/USV_state", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64MultiArray & state) {
        if (has_origin_) {
          latest_state_ = state;
        }
      });
    // 高频发布里程计供控制器使用，低频发布历史轨迹供可视化使用。
    odom_timer_ = create_wall_timer(
      std::chrono::duration<double>(odom_publish_period), [this]() {publish_odometry();});
    path_timer_ = create_wall_timer(
      std::chrono::duration<double>(path_publish_period), [this]() {
        path_pub_->publish(track_.path());
      });
    // map 与 odom 使用相同的 ENU 原点和轴向。
    geometry_msgs::msg::TransformStamped origin;
    origin.header.stamp = now();
    origin.header.frame_id = "map";
    origin.child_frame_id = "odom";
    origin.transform.rotation.w = 1.0;
    origin_tf_.sendTransform(origin);
    RCLCPP_INFO(
      get_logger(), "Waiting for /fix; /odom and vessel TF update every %.0f Hz; /USV_path every %.2f s",
      1.0 / odom_publish_period, path_publish_period);
  }

private:
  /** @brief 接收经纬度原点；原点变化时清空待处理状态，非法原点只记录告警。 */
  void on_origin(const sensor_msgs::msg::NavSatFix & origin)
  {
    constexpr double kDegToRad = 0.017453292519943295;
    try {
      if (track_.set_origin(
          origin.latitude * kDegToRad, origin.longitude * kDegToRad, origin.altitude))
      {
        latest_state_.reset();
      }
      has_origin_ = true;
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid /fix: %s", error.what());
    }
  }

  /** @brief 消费最新状态，校验成功后发布同一时刻的里程计和船体 TF。 */
  void publish_odometry()
  {
    if (!latest_state_) {
      return;
    }
    const auto state = std::move(*latest_state_);
    latest_state_.reset();
    // 输入状态使用累计仿真时间；输出消息统一使用当前 ROS 时间戳。
    if (!track_.update(state, now())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring /USV_state: expected 19 finite fields, radian LLA and a valid state code");
      return;
    }
    const auto & pose = track_.pose();
    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.header.frame_id = "map";
    odom.child_frame_id = "usv_body_rfu";
    odom.pose.pose = pose.pose;
    // 位姿在 map 的 ENU 坐标系中表示；twist 在 RFU 子坐标系中表示。
    // RFU 轴向为 x 向右、y 向前、z 向上。
    odom.twist.twist.linear.x = state.data[9];
    odom.twist.twist.linear.y = state.data[10];
    odom.twist.twist.linear.z = state.data[11];
    odom.twist.twist.angular.z = state.data[12];
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    vessel_tf_.sendTransform(transform);
  }

  bool has_origin_{false};
  std::optional<std_msgs::msg::Float64MultiArray> latest_state_;
  VesselTrack track_;
  tf2_ros::StaticTransformBroadcaster origin_tf_;
  tf2_ros::TransformBroadcaster vessel_tf_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr path_timer_;
};

}  // namespace usv_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_map::OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
