/**
 * @file berth_designer.cpp
 * @brief 接收 RViz2 泊位位置与方向，保存并显示泊位。
 */

#include "berth_chart.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace usv_map_tools
{
/** @brief 根据 RViz 拖动位姿创建矩形泊位，并保存与显示泊位海图。 */
class BerthDesigner : public rclcpp::Node
{
public:
  /** @brief 加载泊位底图与尺寸配置，创建交互位姿、原点和泊位显示接口。 */
  BerthDesigner()
  : Node("berth_designer")
  {
    const auto input = declare_parameter<std::string>("input_chart", "");
    const auto base = declare_parameter<std::string>("base_chart", "");
    output_ = declare_parameter<std::string>("output_chart", "");
    declare_parameter("length_m", 10.0);
    declare_parameter("width_m", 3.0);
    if (input.empty() || output_.empty()) {
      throw std::invalid_argument("input_chart and output_chart must be set in config or launch");
    }
    Berth{0, {0.0, 0.0}, 0.0,
      get_parameter("length_m").as_double(), get_parameter("width_m").as_double()}.corners();
    chart_ = std::make_unique<BerthChart>(input, base);
    RCLCPP_INFO(
      get_logger(), "Chart base=%s; saved berths=%s",
      (base.empty() ? input : base).c_str(), input.c_str());
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/test_tools/berths", rclcpp::QoS(1).reliable().transient_local());
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/fix", rclcpp::QoS(1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/test_tools/berth_pose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped & message) {add_berth(message);});
    fix_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_fix();});
    publish_fix();
    publish_berths();
    RCLCPP_INFO(
      get_logger(), "Ready: drag 2D Nav Goal from berth centre toward bow; %zu berths loaded; output=%s",
      chart_->berths().size(), output_.c_str());
  }

private:
  /** @brief 校验 map 位姿并保存泊位，成功后刷新标记；失败时记录告警。 */
  void add_berth(const geometry_msgs::msg::PoseStamped & message)
  {
    try {
      if (message.header.frame_id != "map") {
        throw std::invalid_argument("berth pose frame_id must be map");
      }
      const auto & p = message.pose.position;
      const auto & q = message.pose.orientation;
      const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < 1e-12)
      {
        throw std::invalid_argument("pose must be finite and quaternion must be nonzero");
      }
      const double x = q.x / norm;
      const double y = q.y / norm;
      const double z = q.z / norm;
      const double w = q.w / norm;
      const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
      const auto berth = chart_->add(
        {0, {p.x, p.y}, yaw,
          get_parameter("length_m").as_double(), get_parameter("width_m").as_double()}, output_);
      publish_berths();
      RCLCPP_INFO(
        get_logger(), "Saved berth #%d: centre=(%.2f, %.2f) m, size=%.2f x %.2f m -> %s",
        berth.id, p.x, p.y, berth.length, berth.width, output_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Berth was not added: %s", error.what());
    }
  }

  void publish_fix()
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.frame_id = "gps_sensor";
    fix.header.stamp = now();
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.longitude = chart_->longitude();
    fix.latitude = chart_->latitude();
    fix_pub_->publish(fix);
  }

  void publish_berths()
  {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray markers;
    Marker clear;
    clear.header.frame_id = "map";
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (const auto & berth : chart_->berths()) {
      Marker fill;
      fill.header.frame_id = "map";
      fill.header.stamp = now();
      fill.ns = "berth_fill";
      fill.id = berth.id;
      fill.type = Marker::CUBE;
      fill.action = Marker::ADD;
      fill.pose.position.x = berth.center.x;
      fill.pose.position.y = berth.center.y;
      fill.pose.position.z = 1.0;
      fill.pose.orientation.z = std::sin(berth.yaw / 2.0);
      fill.pose.orientation.w = std::cos(berth.yaw / 2.0);
      fill.scale.x = berth.length;
      fill.scale.y = berth.width;
      fill.scale.z = 0.2;
      fill.color.r = 1.0;
      fill.color.g = 0.1;
      fill.color.b = 0.75;
      fill.color.a = 0.3;
      markers.markers.push_back(fill);

      auto outline = fill;
      outline.ns = "berth_outline";
      outline.type = Marker::LINE_STRIP;
      outline.pose = geometry_msgs::msg::Pose();
      outline.pose.orientation.w = 1.0;
      outline.scale.x = 0.3;
      outline.color.a = 1.0;
      for (const auto & corner : berth.corners()) {
        geometry_msgs::msg::Point point;
        point.x = corner.x;
        point.y = corner.y;
        point.z = 1.2;
        outline.points.push_back(point);
      }
      markers.markers.push_back(outline);

      auto arrow = fill;
      arrow.ns = "berth_heading";
      arrow.type = Marker::ARROW;
      arrow.pose.position.z = 1.5;
      arrow.scale.x = berth.length * 0.45;
      arrow.scale.y = 0.4;
      arrow.scale.z = 0.4;
      arrow.color.a = 1.0;
      markers.markers.push_back(arrow);

      auto label = fill;
      label.ns = "berth_label";
      label.type = Marker::TEXT_VIEW_FACING;
      label.pose.position.y += berth.width / 2.0 + 1.5;
      label.pose.position.z = 2.0;
      label.pose.orientation = geometry_msgs::msg::Quaternion();
      label.pose.orientation.w = 1.0;
      label.scale.z = 1.5;
      label.color.a = 1.0;
      label.text = "B" + std::to_string(berth.id);
      markers.markers.push_back(label);
    }
    markers_pub_->publish(markers);
  }

  std::unique_ptr<BerthChart> chart_;
  std::string output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr fix_timer_;
};
}  // namespace usv_map_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<usv_map_tools::BerthDesigner>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "berth_designer: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
