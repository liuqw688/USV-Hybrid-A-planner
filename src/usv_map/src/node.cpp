/**
 * @file node.cpp
 * @brief S-57 海图发布节点实现。
 */

#include "usv_map/node.hpp"
#include "usv_map/render_parameters.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>

namespace usv_map
{

UsvMapNode::UsvMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("usv_map_node", options)
{
  declare_parameter("input_path", std::string{});
  declare_parameter("origin_longitude", 106.716974974);
  declare_parameter("origin_latitude", 29.506912001);
  render_config_ = declare_render_config(*this);

  // 发布器使用 Reliable + Transient Local，RViz2 后启动也能收到地图。
  const rclcpp::QoS chart_qos = rclcpp::QoS(1).reliable().transient_local();
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "chart_markers", chart_qos);
  metadata_pub_ = create_publisher<std_msgs::msg::String>(
    "chart_metadata", chart_qos);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("chart_costmap", chart_qos);
  fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(1).reliable());
  // 规划节点不应从 RViz Marker 反推 S-57 属性，这里只发布一次轻量 TSSLPT 数据。
  lane_data_pub_ = create_publisher<std_msgs::msg::String>("chart_lane_data", chart_qos);
  fix_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_fix();});

  publish_chart();
  publish_fix();
}

/**
 * @brief 默认海图路径：始终以功能包自身路径为基准，不依赖绝对路径。
 *
 * 安装环境使用包内 data 文件，源码构建环境回退到 resource 目录。
 */
std::string UsvMapNode::default_input_path()
{
  try {
    const std::string installed_path =
      ament_index_cpp::get_package_share_directory("usv_map") + "/data/suiku.xml";
    if (std::ifstream(installed_path, std::ios::binary).good()) {
      return installed_path;
    }
  } catch (const std::exception &) {
    // 未 source 安装环境时使用源码内资源文件。
  }
  // 未找到安装资源时，从当前目录逐级向上查找源码工作空间。
  for (auto directory = std::filesystem::current_path();;
    directory = directory.parent_path())
  {
    const std::filesystem::path candidates[] = {
      directory / "src/usv_map/resource/suiku.xml",
      // 支持直接在功能包目录运行。
      directory / "resource/suiku.xml",
    };
    for (const auto & candidate : candidates) {
      if (std::ifstream(candidate, std::ios::binary).good()) {
        return candidate.string();
      }
    }
    if (directory == directory.root_path()) {
      break;
    }
  }
  return "resource/suiku.xml";
}

Chart UsvMapNode::load_from_parameters() const
{
  std::string input_path = get_parameter("input_path").as_string();
  if (input_path.empty()) {
    input_path = default_input_path();
  }
  return load_chart(
    input_path,
    get_parameter("origin_longitude").as_double(),
    get_parameter("origin_latitude").as_double());
}

void UsvMapNode::publish_chart()
{
  try {
    chart_ = load_from_parameters();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Unable to load S-57 chart: %s", error.what());
    return;
  }
  const Chart & chart = *chart_;
  const builtin_interfaces::msg::Time stamp = now();
  const Renderer renderer(render_config_, [this](const std::string & warning) {
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    });

  markers_pub_->publish(renderer.build_markers(chart, stamp));

  std_msgs::msg::String lane_data;
  lane_data.data = renderer.lane_data_json(chart);
  lane_data_pub_->publish(lane_data);

  costmap_pub_->publish(renderer.build_costmap(chart, stamp));

  std_msgs::msg::String metadata;
  metadata.data = chart.metadata_json();
  metadata_pub_->publish(metadata);

  RCLCPP_INFO(
    get_logger(), "Published %zu features from %d map(s); origin=(%.6f, %.6f); chart=%s",
    chart.feature_count(), chart.map_count, chart.origin_longitude, chart.origin_latitude,
    chart.source_path.c_str());
}

void UsvMapNode::publish_fix()
{
  if (!chart_) {
    return;
  }
  sensor_msgs::msg::NavSatFix message;
  message.header.frame_id = "map";
  message.header.stamp = now();
  message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  message.latitude = chart_->origin_latitude;
  message.longitude = chart_->origin_longitude;
  message.altitude = 0.0;
  fix_pub_->publish(message);
}

}  // namespace usv_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usv_map::UsvMapNode>());
  rclcpp::shutdown();
  return 0;
}
