/**
 * @file reef_designer.cpp
 * @brief 接收 RViz2 放置点，生成暗礁并刷新测试海图。
 */

#include "reef_chart.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <usv_map/render_parameters.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace usv_map_tools
{
/** @brief 根据 RViz 点击位置生成暗礁轮廓，并保存和刷新海图。 */
class ReefDesigner : public rclcpp::Node
{
public:
  /** @brief 加载暗礁底图与尺寸配置，创建点击输入及地图、原点发布接口。 */
  ReefDesigner()
  : Node("reef_designer")
  {
    const auto input = declare_parameter<std::string>("input_chart", "");
    output_ = declare_parameter<std::string>("output_chart", "");
    declare_parameter("min_radius_m", 3.0);
    declare_parameter("max_radius_m", 5.0);
    if (input.empty() || output_.empty()) {
      throw std::invalid_argument("input_chart and output_chart must be set in config or launch");
    }
    make_reef({0, 0}, 0, get_parameter("min_radius_m").as_double(),
      get_parameter("max_radius_m").as_double());
    // 渲染参数复用主地图配置，编辑场景仅由本节点发布基础地图。
    config_.focus_navigation_features = false;
    config_ = usv_map::declare_render_config(*this, config_);
    chart_ = std::make_unique<ReefChart>(input);
    // 先保存工作地图，再发布原点，确保其他节点能读取同一份资源。
    chart_->save(output_);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    reefs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/test_tools/reefs", qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/chart_markers", qos);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/chart_costmap", qos);
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/fix", rclcpp::QoS(1));
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/test_tools/reef_point", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PointStamped & message) {add_reef(message);});
    publish_map();
    fix_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_fix();});
    publish_fix();
    RCLCPP_INFO(get_logger(),
      "Ready: use RViz Publish Point to place a reef at its area centroid; %zu loaded; output=%s",
      chart_->reefs().size(), output_.c_str());
  }

private:
  /** @brief 将点击位置作为暗礁面积重心，保存成功后刷新地图和原点。 */
  void add_reef(const geometry_msgs::msg::PointStamped & message)
  {
    try {
      if (message.header.frame_id != "map" || !std::isfinite(message.point.z)) {
        throw std::invalid_argument("point must be finite and frame_id must be map");
      }
      const auto reef = chart_->add({message.point.x, message.point.y}, random_(),
        get_parameter("min_radius_m").as_double(), get_parameter("max_radius_m").as_double(), output_);
      RCLCPP_INFO(get_logger(), "Saved reef #%d: centroid=(%.2f, %.2f) m -> %s",
        reef.id, reef.center.x, reef.center.y, output_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Reef was not added: %s", error.what());
      return;
    }
    try {
      publish_map();
      publish_fix();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "XML saved, but map refresh failed: %s", error.what());
    }
  }

  void publish_fix()
  {
    sensor_msgs::msg::NavSatFix message;
    message.header.frame_id = "gps_sensor";
    message.header.stamp = now();
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.latitude = chart_->latitude();
    message.longitude = chart_->longitude();
    message.altitude = 0;
    fix_pub_->publish(message);
  }

  void publish_map()
  {
    const auto chart = usv_map::load_chart(output_);
    const usv_map::Renderer renderer(config_, [this](const std::string & warning) {
        RCLCPP_WARN(get_logger(), "%s", warning.c_str());
      });
    const auto stamp = now();
    markers_pub_->publish(renderer.build_markers(chart, stamp));
    costmap_pub_->publish(renderer.build_costmap(chart, stamp));
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;
    clear.action = Marker::DELETEALL;
    clear.header.frame_id = "map";
    array.markers.push_back(clear);
    const auto point = [](usv_map::Point p, double z) {
        geometry_msgs::msg::Point result;
        result.x = p.x; result.y = p.y; result.z = z;
        return result;
      };
    for (const auto & reef : chart_->reefs()) {
      Marker fill;
      fill.header.frame_id = "map";
      fill.header.stamp = stamp;
      fill.ns = "reef_fill";
      fill.id = reef.id;
      fill.type = Marker::TRIANGLE_LIST;
      fill.pose.orientation.w = 1;
      fill.scale.x = fill.scale.y = fill.scale.z = 1;
      fill.color.r = 1;
      fill.color.g = 0.35;
      fill.color.a = 0.65;
      for (std::size_t i = 1; i < reef.boundary.size(); ++i) {
        fill.points.insert(fill.points.end(), {point(reef.center, 1),
          point(reef.boundary[i - 1], 1), point(reef.boundary[i], 1)});
      }
      array.markers.push_back(fill);
      auto outline = fill;
      outline.ns = "reef_outline";
      outline.type = Marker::LINE_STRIP;
      outline.scale.x = 0.25;
      outline.color.a = 1;
      outline.points.clear();
      for (const auto p : reef.boundary) {outline.points.push_back(point(p, 1.2));}
      array.markers.push_back(std::move(outline));
      auto label = fill;
      label.ns = "reef_label";
      label.type = Marker::TEXT_VIEW_FACING;
      label.points.clear();
      label.pose.position = point(reef.center, 2);
      label.scale.z = 2;
      label.color.a = 1;
      label.text = "R" + std::to_string(reef.id);
      array.markers.push_back(std::move(label));
    }
    reefs_pub_->publish(array);
  }

  std::unique_ptr<ReefChart> chart_;
  std::string output_;
  usv_map::RenderConfig config_;
  std::mt19937_64 random_{std::random_device{}()};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr reefs_pub_, markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::TimerBase::SharedPtr fix_timer_;
};
}  // namespace usv_map_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<usv_map_tools::ReefDesigner>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("reef_designer"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
