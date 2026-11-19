/**
 * @file renderer.hpp
 * @brief 将海图渲染为矢量标记、航道数据与静态代价地图。
 */

#ifndef USV_MAP__RENDERER_HPP_
#define USV_MAP__RENDERER_HPP_

#include <functional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "usv_map/parser.hpp"

namespace usv_map
{

/** @brief 可视化与规划参数。 */
struct RenderConfig
{
  double line_width{0.5};
  double point_size{0.5};
  bool focus_navigation_features{true};
  double planning_resolution{2.0};
  long long planning_max_cells{650000};
  // 以完整海图左下角为固定原点；1.0 表示不裁切。
  double planning_crop_width_fraction{1.0};
  double planning_crop_height_fraction{1.0};
  double obstacle_inflation{4.0};
  double channel_corridor_width{300.0};
  int channel_outside_cost{60};
};

/** @brief 海图渲染器；无状态依赖，可反复调用。 */
class Renderer
{
public:
  /** @brief 渲染告警回调。 */
  using WarnFn = std::function<void(const std::string &)>;

  /**
   * @brief 设置海图显示和规划栅格参数。
   * @param[in] config 渲染、裁切、障碍膨胀和航道代价参数。
   * @param[in] warn 告警回调；为空时不报告渲染告警。
   */
  explicit Renderer(RenderConfig config, WarnFn warn = WarnFn());

  /**
   * @brief 生成 /chart_markers 矢量海图。
   * @param[in] chart 已转换为局部米制坐标的海图。
   * @param[in] stamp 输出标记的 ROS 时间戳。
   * @return map 坐标系中的要素标记数组。
   */
  visualization_msgs::msg::MarkerArray build_markers(
    const Chart & chart, const builtin_interfaces::msg::Time & stamp) const;

  /**
   * @brief 生成 /chart_lane_data 的轻量 JSON（TSSLPT 多边形、ORIENT 和 DIRECTION）。
   * @param[in] chart 含航道分段要素及属性的海图。
   * @return 航道几何与通行方向的 JSON 文本。
   */
  std::string lane_data_json(const Chart & chart) const;

  /**
   * @brief 生成包含航道代价和陆地障碍的静态 /chart_costmap。
   * @param[in] chart 已转换为局部米制坐标的海图。
   * @param[in] stamp 输出栅格的 ROS 时间戳。
   * @return map 坐标系中的静态代价栅格，尺寸受 planning_max_cells 限制。
   */
  nav_msgs::msg::OccupancyGrid build_costmap(
    const Chart & chart,
    const builtin_interfaces::msg::Time & stamp) const;

private:
  /** @brief 构造 map 坐标系下的基础 Marker。 */
  visualization_msgs::msg::Marker base_marker(
    const std::string & ns, int id, int type, const builtin_interfaces::msg::Time & stamp) const;

  /** @brief 按 S-57 分组设置颜色；alpha 为负时保留默认透明度。 */
  void set_color(
    visualization_msgs::msg::Marker & marker, const std::string & group, double alpha = -1.0) const;

  RenderConfig config_;
  WarnFn warn_;
};

}  // namespace usv_map

#endif  // USV_MAP__RENDERER_HPP_
