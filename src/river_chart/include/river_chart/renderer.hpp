/// @file renderer.hpp
/// @brief 把解码后的海图渲染为 ROS 话题消息（C++ 版，行为等价于原 Python node.py）。
///
/// 渲染逻辑与 ROS 节点解耦，全部为纯函数式调用，便于单元测试。

/// @date 2026-08-28

#ifndef RIVER_CHART__RENDERER_HPP_
#define RIVER_CHART__RENDERER_HPP_

#include <functional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "river_chart/parser.hpp"

namespace river_chart
{

/// @brief 可视化与规划参数，对应原 Python 节点的同名 ROS 参数。
struct RenderConfig
{
  std::string frame_id{"map"};
  double line_width{0.2};
  double point_size{0.15};
  bool fill_polygons{false};
  bool show_land_boundaries{false};
  bool focus_navigation_features{true};
  bool publish_labels{false};
  double grid_resolution{0.2};
  long long max_grid_cells{20000000};
  double planning_resolution{0.2};
  long long planning_max_cells{20000000};
  double obstacle_inflation{1.0};
  double channel_corridor_width{20.0};
  int channel_outside_cost{60};
  double dynamic_obstacle_radius{1.0};
};

/// @brief 海图渲染器；无状态依赖，可反复调用。
class Renderer
{
public:
  /// @brief 告警回调，对应 Python 的 logger.warn。
  using WarnFn = std::function<void(const std::string &)>;

  explicit Renderer(RenderConfig config, WarnFn warn = WarnFn());

  /// @brief 生成 /chart_markers 矢量海图。
  visualization_msgs::msg::MarkerArray build_markers(
    const Chart & chart, const builtin_interfaces::msg::Time & stamp) const;

  /// @brief 生成 /chart_lane_data 的轻量 JSON（TSSLPT 多边形 + ORIENT）。
  std::string lane_data_json(const Chart & chart) const;

  /// @brief 生成 /chart_channel_direction 方向箭头。
  visualization_msgs::msg::MarkerArray build_direction_markers(
    const Chart & chart, const builtin_interfaces::msg::Time & stamp) const;

  /// @brief 生成 /chart_occupancy 几何诊断栅格。
  nav_msgs::msg::OccupancyGrid build_occupancy_grid(
    const Chart & chart, const builtin_interfaces::msg::Time & stamp) const;

  /// @brief 生成 /chart_static_obstacles 与 /chart_costmap 规划栅格。
  void build_planning_grids(
    const Chart & chart, const std::vector<Point> & future_obstacles,
    const builtin_interfaces::msg::Time & stamp,
    nav_msgs::msg::OccupancyGrid & static_grid,
    nav_msgs::msg::OccupancyGrid & cost_grid) const;

private:
  /// @brief 构造一个基础 Marker（ADD、固定坐标系、线宽和透明度）。
  visualization_msgs::msg::Marker base_marker(
    const std::string & ns, int id, int type, const builtin_interfaces::msg::Time & stamp) const;

  /// @brief 按 S-57 分组设置颜色；alpha 为负时保留默认透明度。
  void set_color(
    visualization_msgs::msg::Marker & marker, const std::string & group, double alpha = -1.0) const;

  RenderConfig config_;
  WarnFn warn_;
};

}  // namespace river_chart

#endif  // RIVER_CHART__RENDERER_HPP_
