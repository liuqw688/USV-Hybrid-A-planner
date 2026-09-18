/// @file node.cpp
/// @brief S-57 海图发布节点实现。
///
/// @author susheng
/// @date 2026-08-28

#include "river_chart/node.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>

// 未安装时的源码内备用海图路径，由 CMake 以编译定义注入。
#ifndef RIVER_CHART_SOURCE_DATA
#define RIVER_CHART_SOURCE_DATA "resource/ecdis_data.xml"
#endif

namespace river_chart
{

RiverChartNode::RiverChartNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("river_chart_node", options)
{
  declare_parameter("input_path", default_input_path());
  declare_parameter("frame_id", "map");
  declare_parameter("origin_longitude", std::numeric_limits<double>::quiet_NaN());
  declare_parameter("origin_latitude", std::numeric_limits<double>::quiet_NaN());
  declare_parameter("scale", 15.0);
  declare_parameter("line_width", 0.2);
  declare_parameter("point_size", 0.15);
  declare_parameter("fill_polygons", false);
  declare_parameter("show_land_boundaries", false);
  declare_parameter("focus_navigation_features", true);
  declare_parameter("publish_labels", false);
  declare_parameter("publish_occupancy_grid", false);
  declare_parameter("grid_resolution", 0.2);
  declare_parameter("max_grid_cells", static_cast<int64_t>(20000000));
  declare_parameter("reload_period", 0.0);
  declare_parameter("marker_topic", "chart_markers");
  declare_parameter("metadata_topic", "chart_metadata");
  declare_parameter("occupancy_topic", "chart_occupancy");
  declare_parameter("publish_planning_layers", true);
  declare_parameter("planning_resolution", 0.2);
  declare_parameter("planning_max_cells", static_cast<int64_t>(20000000));
  declare_parameter("obstacle_inflation", 1.0);
  declare_parameter("channel_corridor_width", 20.0);
  declare_parameter("publish_channel_direction", false);
  declare_parameter("channel_outside_cost", static_cast<int64_t>(60));
  declare_parameter("future_obstacle_topic", "future_obstacles");
  declare_parameter("dynamic_obstacle_radius", 1.0);

  // 发布器使用 Reliable + Transient Local，RViz2 后启动也能收到地图。
  const rclcpp::QoS chart_qos = rclcpp::QoS(1).reliable().transient_local();
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    get_parameter("marker_topic").as_string(), chart_qos);
  metadata_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("metadata_topic").as_string(), chart_qos);
  occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    get_parameter("occupancy_topic").as_string(), chart_qos);
  static_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "chart_static_obstacles", chart_qos);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("chart_costmap", chart_qos);
  direction_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "chart_channel_direction", chart_qos);
  // 规划节点不应从 RViz Marker 反推 S-57 属性，这里只发布一次轻量 TSSLPT 数据。
  lane_data_pub_ = create_publisher<std_msgs::msg::String>("chart_lane_data", chart_qos);
  future_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    get_parameter("future_obstacle_topic").as_string(), chart_qos,
    std::bind(&RiverChartNode::future_obstacles_callback, this, std::placeholders::_1));

  publish_chart();
  const double reload_period = get_parameter("reload_period").as_double();
  if (std::isfinite(reload_period) && reload_period > 0.0) {
    reload_timer_ = create_wall_timer(
      std::chrono::duration<double>(reload_period), [this]() {publish_chart();});
  }
}

/// @brief 默认海图路径：始终以功能包自身路径为基准，不依赖绝对路径。
///
/// 查找顺序（返回第一个真实存在的文件）：
///   1. 已安装功能包：<prefix>/share/river_chart/data/ecdis_data.xml，
///      由 ament_index 解析，正常 source install/setup.bash 后走这里；
///   2. 直接运行 install 内可执行文件：由自身路径反推 <prefix>/share/river_chart；
///   3. 未安装、直接跑源码构建产物：使用 CMake 配置阶段记录的源码 resource 路径。
std::string RiverChartNode::default_input_path()
{
  constexpr const char * kDataName = "ecdis_data.xml";
  std::vector<std::string> candidates;

  try {
    candidates.push_back(
      ament_index_cpp::get_package_share_directory("river_chart") + "/data/" + kDataName);
  } catch (const std::exception &) {
    // 未 source 安装环境时忽略，继续尝试其它功能包路径。
  }

  std::error_code error;
  const std::filesystem::path executable = std::filesystem::read_symlink("/proc/self/exe", error);
  if (!error) {
    // <prefix>/lib/river_chart/river_chart_node -> <prefix>/share/river_chart。
    candidates.push_back(
      (executable.parent_path() / ".." / ".." / "share" / "river_chart" / "data" / kDataName)
      .lexically_normal().string());
  }

  candidates.push_back(RIVER_CHART_SOURCE_DATA);

  for (const std::string & candidate : candidates) {
    if (std::ifstream(candidate, std::ios::binary).good()) {
      return candidate;
    }
  }
  return candidates.front();
}

Chart RiverChartNode::load_from_parameters() const
{
  return load_chart(
    get_parameter("input_path").as_string(),
    get_parameter("origin_longitude").as_double(),
    get_parameter("origin_latitude").as_double(),
    get_parameter("scale").as_double());
}

RenderConfig RiverChartNode::render_config() const
{
  RenderConfig config;
  config.frame_id = get_parameter("frame_id").as_string();
  config.line_width = get_parameter("line_width").as_double();
  config.point_size = get_parameter("point_size").as_double();
  config.fill_polygons = get_parameter("fill_polygons").as_bool();
  config.show_land_boundaries = get_parameter("show_land_boundaries").as_bool();
  config.focus_navigation_features = get_parameter("focus_navigation_features").as_bool();
  config.publish_labels = get_parameter("publish_labels").as_bool();
  config.grid_resolution = get_parameter("grid_resolution").as_double();
  config.max_grid_cells = get_parameter("max_grid_cells").as_int();
  config.planning_resolution = get_parameter("planning_resolution").as_double();
  config.planning_max_cells = get_parameter("planning_max_cells").as_int();
  config.obstacle_inflation = get_parameter("obstacle_inflation").as_double();
  config.channel_corridor_width = get_parameter("channel_corridor_width").as_double();
  config.channel_outside_cost = static_cast<int>(get_parameter("channel_outside_cost").as_int());
  config.dynamic_obstacle_radius = get_parameter("dynamic_obstacle_radius").as_double();
  return config;
}

Renderer RiverChartNode::make_renderer() const
{
  return Renderer(render_config(), [this](const std::string & message) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    });
}

void RiverChartNode::publish_chart()
{
  try {
    chart_ = load_from_parameters();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Unable to load S-57 chart: %s", error.what());
    return;
  }
  const Chart & chart = *chart_;
  const builtin_interfaces::msg::Time stamp = now();
  const Renderer renderer = make_renderer();

  markers_pub_->publish(renderer.build_markers(chart, stamp));

  std_msgs::msg::String lane_data;
  lane_data.data = renderer.lane_data_json(chart);
  lane_data_pub_->publish(lane_data);

  if (get_parameter("publish_planning_layers").as_bool()) {
    publish_planning_layers(chart);
  }

  std_msgs::msg::String metadata;
  metadata.data = chart.metadata_json();
  metadata_pub_->publish(metadata);

  if (get_parameter("publish_occupancy_grid").as_bool()) {
    occupancy_pub_->publish(renderer.build_occupancy_grid(chart, stamp));
  }

  RCLCPP_INFO(
    get_logger(), "Published %zu features from %d map(s); origin=(%.6f, %.6f); chart=%s",
    chart.feature_count(), chart.map_count, chart.origin_longitude, chart.origin_latitude,
    chart.source_path.c_str());
}

void RiverChartNode::publish_planning_layers(const Chart & chart)
{
  const Renderer renderer = make_renderer();
  nav_msgs::msg::OccupancyGrid static_grid;
  nav_msgs::msg::OccupancyGrid cost_grid;
  renderer.build_planning_grids(chart, future_obstacles_, now(), static_grid, cost_grid);
  static_pub_->publish(static_grid);
  costmap_pub_->publish(cost_grid);
  if (get_parameter("publish_channel_direction").as_bool()) {
    direction_pub_->publish(renderer.build_direction_markers(chart, now()));
  }
}

void RiverChartNode::future_obstacles_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr message)
{
  future_obstacles_.clear();
  future_obstacles_.reserve(message->poses.size());
  for (const auto & pose : message->poses) {
    future_obstacles_.push_back(Point{pose.position.x, pose.position.y});
  }
  if (chart_ && get_parameter("publish_planning_layers").as_bool()) {
    publish_planning_layers(*chart_);
  }
}

}  // namespace river_chart

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<river_chart::RiverChartNode>());
  rclcpp::shutdown();
  return 0;
}
