/**
 * @file no_go_area_editor.cpp
 * @brief 以 RViz2 估计位置和目标位置为对角点保存矩形禁行区。
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "xml_utils.hpp"

namespace usv_map_tools
{
namespace
{

constexpr double kEarthRadius = 6378137.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

struct Point
{
  double x;
  double y;
};

std::string element_text(const tinyxml2::XMLElement * parent, const char * name)
{
  const auto * element = parent ? parent->FirstChildElement(name) : nullptr;
  const char * text = element ? element->GetText() : nullptr;
  if (!text) {
    throw std::runtime_error(std::string("chart is missing ") + name);
  }
  return text;
}

std::string number(double value) {return xml::number(value, 3);}

}  // namespace

/** @brief 将 RViz 选定的两个对角点写为海图矩形禁行区域。 */
class NoGoAreaEditor : public rclcpp::Node
{
public:
  /** @brief 加载海图，创建对角点选择订阅及禁行区、原点发布接口。 */
  NoGoAreaEditor()
  : Node("no_go_area_editor")
  {
    declare_parameter("input_chart", "");
    declare_parameter(
      "output_chart", "src/usv_map/resource/generated/reservoir_test_blocked.xml");
    load_chart();

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    region_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/test_tools/no_go_regions", transient_qos);
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/fix", rclcpp::QoS(1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      std::bind(&NoGoAreaEditor::initial_pose_callback, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      std::bind(&NoGoAreaEditor::goal_pose_callback, this, std::placeholders::_1));

    publish_fix();
    fix_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&NoGoAreaEditor::publish_fix, this));
    RCLCPP_INFO(
      get_logger(), "Ready: initial pose and goal pose mark rectangle corners; output=%s",
      output_path_.c_str());
  }

private:
  void load_chart()
  {
    input_path_ = get_parameter("input_chart").as_string();
    if (input_path_.empty()) {
      throw std::runtime_error("input_chart must point to the segmented vector chart XML");
    }
    input_path_ = std::filesystem::absolute(input_path_).string();
    output_path_ = std::filesystem::absolute(get_parameter("output_chart").as_string()).string();
    if (document_.LoadFile(input_path_.c_str()) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("cannot load chart XML: " + input_path_);
    }
    auto * root = document_.RootElement();
    map_ = root ? root->FirstChildElement("Map") : nullptr;
    if (!map_) {
      throw std::runtime_error("chart XML has no Map element");
    }
    auto * bounds = map_->FirstChildElement("Bounds");
    const double min_lon = std::stod(element_text(bounds, "MinX"));
    const double min_lat = std::stod(element_text(bounds, "MinY"));
    const double max_lon = std::stod(element_text(bounds, "MaxX"));
    const double max_lat = std::stod(element_text(bounds, "MaxY"));
    origin_lon_ = (min_lon + max_lon) * 0.5;
    origin_lat_ = (min_lat + max_lat) * 0.5;
    const double latitude_scale = kEarthRadius * std::cos(origin_lat_ * kDegToRad);
    chart_bounds_ = {
      (min_lon - origin_lon_) * kDegToRad * latitude_scale,
      (min_lat - origin_lat_) * kDegToRad * kEarthRadius,
      (max_lon - origin_lon_) * kDegToRad * latitude_scale,
      (max_lat - origin_lat_) * kDegToRad * kEarthRadius};

    features_ = map_->FirstChildElement("Features");
    if (!features_) {
      features_ = document_.NewElement("Features");
      map_->InsertEndChild(features_);
    }
    for (auto * group = features_->FirstChildElement("FeatureGroup"); group;
      group = group->NextSiblingElement("FeatureGroup"))
    {
      for (auto * feature = group->FirstChildElement("Feature"); feature;
        feature = feature->NextSiblingElement("Feature"))
      {
        for (const char * tag : {"RCID", "ID"}) {
          const auto * id_element = feature->FirstChildElement(tag);
          if (id_element && id_element->GetText()) {
            next_feature_id_ = std::max(
              next_feature_id_, std::atoi(id_element->GetText()) + 1);
          }
        }
      }
    }
  }

  void publish_fix()
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = now();
    fix.header.frame_id = "gps_sensor";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = origin_lat_;
    fix.longitude = origin_lon_;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix_pub_->publish(fix);
  }

  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!valid_frame(message->header.frame_id) || !valid_point(message->pose.pose.position)) {
      return;
    }
    initial_corner_ = Point{message->pose.pose.position.x, message->pose.pose.position.y};
    if (goal_corner_) {
      add_region(*initial_corner_, *goal_corner_);
    }
  }

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!valid_frame(message->header.frame_id) || !valid_point(message->pose.position)) {
      return;
    }
    goal_corner_ = Point{message->pose.position.x, message->pose.position.y};
    if (initial_corner_) {
      add_region(*initial_corner_, *goal_corner_);
    }
  }

  bool valid_frame(const std::string & frame_id) const
  {
    if (frame_id.empty() || frame_id == "map") {
      return true;
    }
    RCLCPP_WARN(get_logger(), "Ignoring pose in frame '%s'; RViz fixed frame must be map", frame_id.c_str());
    return false;
  }

  bool valid_point(const geometry_msgs::msg::Point & point) const
  {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      return true;
    }
    RCLCPP_WARN(get_logger(), "Ignoring non-finite selection point");
    return false;
  }

  /** @brief 消费一对选点，校验区域尺寸及海图边界，保存成功后发布矩形标记。 */
  void add_region(const Point & first, const Point & second)
  {
    initial_corner_.reset();
    goal_corner_.reset();
    const double min_x = std::min(first.x, second.x);
    const double min_y = std::min(first.y, second.y);
    const double max_x = std::max(first.x, second.x);
    const double max_y = std::max(first.y, second.y);
    if (max_x - min_x < 1.0 || max_y - min_y < 1.0) {
      RCLCPP_WARN(get_logger(), "Selection is too small; both sides must be at least 1 m");
      return;
    }
    if (min_x < chart_bounds_[0] || min_y < chart_bounds_[1] ||
      max_x > chart_bounds_[2] || max_y > chart_bounds_[3])
    {
      RCLCPP_WARN(get_logger(), "Selection lies outside chart bounds; no region was added");
      return;
    }

    append_feature(min_x, min_y, max_x, max_y);
    if (!save_chart()) {
      return;
    }
    publish_region(min_x, min_y, max_x, max_y);
    RCLCPP_INFO(
      get_logger(), "Saved non-navigable rectangle #%zu to %s",
      region_count_, output_path_.c_str());
  }

  void append_feature(double min_x, double min_y, double max_x, double max_y)
  {
    tinyxml2::XMLElement * group = nullptr;
    for (auto * item = features_->FirstChildElement("FeatureGroup"); item;
      item = item->NextSiblingElement("FeatureGroup"))
    {
      const char * acronym = item->Attribute("acronym");
      if (acronym && std::string(acronym) == "LNDARE") {
        group = item;
        break;
      }
    }
    if (!group) {
      group = document_.NewElement("FeatureGroup");
      group->SetAttribute("acronym", "LNDARE");
      group->SetAttribute("priority", "2");
      group->SetAttribute("count", "0");
      features_->InsertEndChild(group);
    }
    const int count = group->IntAttribute("count", 0) + 1;
    group->SetAttribute("count", count);

    const double origin_lat_rad = origin_lat_ * kDegToRad;
    const double origin_lon_rad = origin_lon_ * kDegToRad;
    const double longitude_scale = kEarthRadius * std::cos(origin_lat_rad);
    const std::array<Point, 5> corners{{
      {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}}};
    std::array<Point, 5> projected{};
    for (std::size_t i = 0; i < corners.size(); ++i) {
      const double lon = origin_lon_rad + corners[i].x / longitude_scale;
      const double lat = origin_lat_rad + corners[i].y / kEarthRadius;
      projected[i] = {
        kEarthRadius * lon,
        -kEarthRadius * std::log(std::tan(3.14159265358979323846 / 4.0 + lat / 2.0))};
    }
    double min_projected_x = projected[0].x;
    double min_projected_y = projected[0].y;
    double max_projected_x = projected[0].x;
    double max_projected_y = projected[0].y;
    std::string wkt = "POLYGON ((";
    for (std::size_t i = 0; i < projected.size(); ++i) {
      if (i != 0) {
        wkt += ", ";
      }
      wkt += number(projected[i].x) + " " + number(projected[i].y);
      min_projected_x = std::min(min_projected_x, projected[i].x);
      min_projected_y = std::min(min_projected_y, projected[i].y);
      max_projected_x = std::max(max_projected_x, projected[i].x);
      max_projected_y = std::max(max_projected_y, projected[i].y);
    }
    wkt += "))";

    auto * feature = document_.NewElement("Feature");
    auto add_text = [&](const char * name, const std::string & value) {
      auto * element = document_.NewElement(name);
      element->SetText(value.c_str());
      feature->InsertEndChild(element);
    };
    add_text("RCID", std::to_string(next_feature_id_));
    add_text("ID", std::to_string(next_feature_id_++));
    add_text("Name", "RVIZ_NO_GO");
    add_text("Type", "A");
    add_text("TableName", "O");
    auto * attributes = document_.NewElement("Attributes");
    auto * info = document_.NewElement("INFORM");
    info->SetText("Selected in RViz as non-navigable area");
    attributes->InsertEndChild(info);
    feature->InsertEndChild(attributes);

    auto * geometry = document_.NewElement("Geometry");
    const std::string bbox = number(min_projected_x) + "," + number(min_projected_y) + "," +
      number(max_projected_x) + "," + number(max_projected_y);
    auto * bbox_element = document_.NewElement("BoundingBox");
    bbox_element->SetText(bbox.c_str());
    geometry->InsertEndChild(bbox_element);
    auto * geometries = document_.NewElement("Geometries");
    geometries->SetAttribute("level", "0");
    auto * polygon = document_.NewElement("Polygon");
    polygon->SetText(wkt.c_str());
    geometries->InsertEndChild(polygon);
    geometry->InsertEndChild(geometries);
    feature->InsertEndChild(geometry);
    group->InsertEndChild(feature);
  }

  bool save_chart()
  {
    try {
      xml::save_atomic(document_, output_path_);
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to save XML: %s", error.what());
      return false;
    }
  }

  void publish_region(double min_x, double min_y, double max_x, double max_y)
  {
    ++region_count_;
    const int id = static_cast<int>(region_count_ * 2);
    visualization_msgs::msg::Marker fill;
    fill.header.frame_id = "map";
    fill.header.stamp = now();
    fill.ns = "no_go_regions";
    fill.id = id;
    fill.type = visualization_msgs::msg::Marker::CUBE;
    fill.action = visualization_msgs::msg::Marker::ADD;
    fill.pose.position.x = (min_x + max_x) * 0.5;
    fill.pose.position.y = (min_y + max_y) * 0.5;
    fill.pose.position.z = 0.15;
    fill.pose.orientation.w = 1.0;
    fill.scale.x = max_x - min_x;
    fill.scale.y = max_y - min_y;
    fill.scale.z = 0.3;
    fill.color.r = 1.0;
    fill.color.a = 0.22;
    fill.frame_locked = false;
    region_markers_.markers.push_back(fill);

    auto outline = fill;
    outline.id = id + 1;
    outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outline.pose.position.x = 0.0;
    outline.pose.position.y = 0.0;
    outline.pose.position.z = 1.0;
    outline.scale.x = 2.0;
    outline.color.a = 0.95;
    outline.points.clear();
    for (const Point & point : std::array<Point, 5>{{
      {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}}})
    {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.x;
      marker_point.y = point.y;
      marker_point.z = 1.0;
      outline.points.push_back(marker_point);
    }
    region_markers_.markers.push_back(std::move(outline));
    region_pub_->publish(region_markers_);
  }

  tinyxml2::XMLDocument document_;
  tinyxml2::XMLElement * map_{nullptr};
  tinyxml2::XMLElement * features_{nullptr};
  std::string input_path_;
  std::string output_path_;
  double origin_lon_{0.0};
  double origin_lat_{0.0};
  std::array<double, 4> chart_bounds_{};
  int next_feature_id_{1};
  std::size_t region_count_{0};
  std::optional<Point> initial_corner_;
  std::optional<Point> goal_corner_;
  visualization_msgs::msg::MarkerArray region_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::TimerBase::SharedPtr fix_timer_;
};

}  // namespace usv_map_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<usv_map_tools::NoGoAreaEditor>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "no_go_area_editor: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
