/**
 * @file node.hpp
 * @brief 静态海图发布节点，管理地图、原点与航道数据接口。
 */

#ifndef USV_MAP__NODE_HPP_
#define USV_MAP__NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "usv_map/parser.hpp"
#include "usv_map/renderer.hpp"

namespace usv_map
{

/** @brief 解码 S-57 XML 海图，并发布可视化、规划栅格与元数据。 */
class UsvMapNode : public rclcpp::Node
{
public:
  /**
   * @brief 加载海图并创建地图图层、元数据与地理原点发布接口。
   * @param[in] options ROS 节点选项，可包含海图路径及渲染参数覆盖。
   */
  explicit UsvMapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /** @brief 重新加载并发布全部海图数据。 */
  void publish_chart();
  /** @brief 按当前参数读取海图。 */
  Chart load_from_parameters() const;
  /** @brief 发布海图原点，供 RViz2 天地图图层定位。 */
  void publish_fix();
  /** @brief 默认海图路径：安装后为 share/usv_map/data/suiku.xml。 */
  static std::string default_input_path();

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_data_pub_;
  rclcpp::TimerBase::SharedPtr fix_timer_;

  RenderConfig render_config_;
  std::optional<Chart> chart_;
};

}  // namespace usv_map

#endif  // USV_MAP__NODE_HPP_
