/// @file node.hpp
/// @brief S-57 海图发布节点（C++ 版）。
///
/// @author susheng
/// @date 2026-08-28

#ifndef RIVER_CHART__NODE_HPP_
#define RIVER_CHART__NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "river_chart/parser.hpp"
#include "river_chart/renderer.hpp"

namespace river_chart
{

/// @brief 解码 S-57 XML 海图，并发布 MarkerArray、规划栅格与元数据。
///
/// 发布话题与参数和原 Python 版本完全一致：
///   /chart_markers, /chart_metadata, /chart_occupancy,
///   /chart_static_obstacles, /chart_costmap, /chart_channel_direction,
///   /chart_lane_data；订阅 /future_obstacles。
class RiverChartNode : public rclcpp::Node
{
public:
  explicit RiverChartNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// @brief 重新加载并发布全部海图数据。
  void publish_chart();
  /// @brief 按当前参数读取海图。
  Chart load_from_parameters() const;
  /// @brief 把 ROS 参数整理成渲染配置。
  RenderConfig render_config() const;
  /// @brief 构造带日志回调的渲染器。
  Renderer make_renderer() const;
  /// @brief 发布静态障碍栅格、代价栅格和可选的方向箭头。
  void publish_planning_layers(const Chart & chart);
  /// @brief 接收动态障碍物并刷新代价栅格。
  void future_obstacles_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
  /// @brief 默认海图路径：安装后为 share/river_chart/data/ecdis_data.xml。
  static std::string default_input_path();

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_data_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr future_sub_;
  rclcpp::TimerBase::SharedPtr reload_timer_;

  std::vector<Point> future_obstacles_;
  std::optional<Chart> chart_;
};

}  // namespace river_chart

#endif  // RIVER_CHART__NODE_HPP_
