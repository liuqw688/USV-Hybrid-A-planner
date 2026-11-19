/**
 * @file obstacle_perception_sim_node.cpp
 * @brief 读取仿真暗礁真值，发布 RFU 探测快照和独立的感知范围 Marker。
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <usv_interfaces/msg/obstacle_detection_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "channel_navigation_manager/chart_data.hpp"
#include "channel_navigation_manager/perception_geometry.hpp"

namespace cnm = channel_navigation_manager;
using SteadyClock = std::chrono::steady_clock;

/** @brief 从海图暗礁真值和船体位姿生成 RFU 相对探测快照。 */
class ObstaclePerceptionSimNode final : public rclcpp::Node
{
public:
  /** @brief 校验探测范围与周期，创建真值、里程计、TF 和探测发布接口。 */
  ObstaclePerceptionSimNode()
  : Node("obstacle_perception_sim_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto chart_topic = declare_parameter("chart_lane_topic", "/chart_lane_data");
    const auto odom_topic = declare_parameter("odom_topic", "/odom");
    const auto detection_topic = declare_parameter("detections_topic", "/obstacle_detections");
    const auto marker_topic = declare_parameter("marker_topic", "/perception_markers");
    radius_m_ = declare_parameter("perception_radius_m", 60.0);
    odom_timeout_s_ = declare_parameter("odom_timeout_s", 3.0);
    const double rate = declare_parameter("publish_rate_hz", 5.0);
    if (!std::isfinite(radius_m_) || radius_m_ <= 0.0 ||
      !std::isfinite(odom_timeout_s_) || odom_timeout_s_ <= 0.0 ||
      !std::isfinite(rate) || rate < 0.5 || rate > 100.0)
    {
      throw std::invalid_argument("invalid perception radius, timeout or publish rate");
    }
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    // 动态探测采用 volatile：重启的消费者等待新观测，不接收旧的保留目标。
    detection_pub_ = create_publisher<usv_interfaces::msg::ObstacleDetectionArray>(
      detection_topic, rclcpp::QoS(1).reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, latched);
    chart_sub_ = create_subscription<std_msgs::msg::String>(
      chart_topic, latched, [this](std_msgs::msg::String::ConstSharedPtr msg) {readTruth(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(20), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = msg;
        odom_received_ = SteadyClock::now();
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {publishScan();});
    RCLCPP_INFO(get_logger(), "Simulated perception: %.1f m, output %s", radius_m_, detection_topic.c_str());
  }

private:
  /** @brief 读取并替换有效暗礁轮廓缓存，解析失败时记录错误并保留已有真值。 */
  void readTruth(const std_msgs::msg::String & msg)
  {
    try {
      boost::property_tree::ptree payload;
      std::istringstream stream(msg.data);
      boost::property_tree::read_json(stream, payload);
      std::vector<cnm::Polygon> obstacles;
      if (const auto items = payload.get_child_optional("obstacles")) {
        for (const auto & entry : *items) {
          const auto & item = entry.second;
          if (item.get("kind", std::string{}) != "REEF") {continue;}
          auto polygon = cnm::readPolygon(item);
          (void)cnm::polygonCentroid(polygon);
          obstacles.push_back(std::move(polygon));
        }
      }
      chart_frame_ = payload.get("frame_id", std::string("map"));
      obstacles_ = std::move(obstacles);
      RCLCPP_INFO(get_logger(), "Simulation truth updated: %zu reefs", obstacles_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid simulation chart: %s", e.what());
    }
  }

  /**
   * @brief 根据已缓存的里程计求传感器到海图坐标系的变换。
   * @return 里程计观测时刻的传感器位姿。
   * @throws std::invalid_argument 里程计位姿或坐标系无效。
   * @throws tf2::TransformException 对应时刻的坐标变换不可用。
   */
  tf2::Transform sensorPose() const
  {
    const auto & pose = odom_->pose.pose;
    const auto & q = pose.orientation;
    const double length2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(length2) || length2 < 1.0e-12 ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || odom_->header.frame_id.empty() ||
      odom_->child_frame_id.empty())
    {
      throw std::invalid_argument("invalid odometry pose or frame");
    }
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    tf2::Transform pose_in_odom(rotation.normalized(),
      tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    if (odom_->header.frame_id == chart_frame_) {return pose_in_odom;}
    const auto transform = tf_buffer_.lookupTransform(
      chart_frame_, odom_->header.frame_id, rclcpp::Time(odom_->header.stamp));
    const auto & t = transform.transform;
    const tf2::Transform map_from_odom(
      tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
      tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
    return map_from_odom * pose_in_odom;
  }

  /** @brief 在里程计和 TF 有效时发布范围内目标；缺少有效观测时发布空快照。 */
  void publishScan()
  {
    usv_interfaces::msg::ObstacleDetectionArray scan;
    scan.header.frame_id = odom_ ? odom_->child_frame_id : "usv_body_rfu";
    scan.header.stamp = now();
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    try {
      // 无有效定位时发布空快照；不沿用旧船位继续报告障碍。
      if (odom_ && std::chrono::duration<double>(SteadyClock::now() - odom_received_).count() <=
        odom_timeout_s_ && (now() - rclcpp::Time(odom_->header.stamp)).seconds() <= odom_timeout_s_)
      {
        const auto pose = sensorPose();
        scan.header.stamp = odom_->header.stamp;  // 与实际使用的船位/TF 同时刻。
        const auto & position = pose.getOrigin();
        std_msgs::msg::Header header;
        header.frame_id = chart_frame_;
        header.stamp = scan.header.stamp;
        cnm::Polygon circle;
        for (int i = 0; i <= 72; ++i) {
          const double angle = 2.0 * M_PI * i / 72;
          circle.push_back({position.x() + radius_m_ * std::cos(angle),
            position.y() + radius_m_ * std::sin(angle)});
        }
        markers.markers.push_back(outline(header, "perception_radius", 0, circle, false));
        for (const auto & obstacle : obstacles_) {
          // 最近边缘进入圆即探测；真值轮廓只供模拟和 RViz 调试，探测消息不含点。
          if (cnm::pointPolygonDistance(position.x(), position.y(), obstacle) > radius_m_) {
            continue;
          }
          scan.obstacles.push_back(cnm::detectObstacle("REEF", obstacle, pose));
          markers.markers.push_back(outline(header, "detected_obstacle",
            static_cast<int>(scan.obstacles.size()), obstacle, true));
        }
      }
    } catch (const std::exception & e) {
      scan.obstacles.clear();
      markers.markers.resize(1);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Perception unavailable: %s", e.what());
    }
    detection_pub_->publish(scan);  // 没有目标也要发，供消费者清除上一帧覆盖。
    marker_pub_->publish(markers);
  }

  visualization_msgs::msg::Marker outline(
    const std_msgs::msg::Header & header, const std::string & ns, int id,
    const cnm::Polygon & polygon, bool obstacle) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obstacle ? 0.8 : 0.25;
    marker.color.r = obstacle ? 1.0F : 0.0F;
    marker.color.g = obstacle ? 0.05F : 0.8F;
    marker.color.b = obstacle ? 0.05F : 1.0F;
    marker.color.a = 0.9F;
    marker.lifetime = rclcpp::Duration::from_seconds(odom_timeout_s_);
    for (const auto & p : polygon) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0.5;
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {marker.points.push_back(marker.points.front());}
    return marker;
  }

  std::string chart_frame_{"map"};
  double radius_m_{60.0}, odom_timeout_s_{3.0};
  std::vector<cnm::Polygon> obstacles_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  SteadyClock::time_point odom_received_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<usv_interfaces::msg::ObstacleDetectionArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chart_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePerceptionSimNode>());
  rclcpp::shutdown();
  return 0;
}
