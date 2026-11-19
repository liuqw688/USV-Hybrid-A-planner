/**
 * @file gnss_path_publisher.cpp
 * @brief 发布 GNSS 参考轨迹、平滑航线与双向航道。
 */

#include "clockwise_route.hpp"
#include "fairway_chart.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <usv_map/parser.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace usv_map_tools
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadius = 6378137.0;

double parse_number(const std::string & text)
{
  std::size_t end = 0;
  const double value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("invalid coordinate");
  }
  return value;
}

nav_msgs::msg::Path path_message(
  const std::vector<RoutePose> & poses, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(poses.size());
  for (const auto & point : poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.position.x;
    pose.pose.position.y = point.position.y;
    pose.pose.orientation.z = std::sin(point.yaw / 2);
    pose.pose.orientation.w = std::cos(point.yaw / 2);
    path.poses.push_back(pose);
  }
  return path;
}

visualization_msgs::msg::MarkerArray fairway_markers(
  const Fairway & fairway, const std_msgs::msg::Header & header)
{
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray array;
  Marker clear;
  clear.header = header;
  clear.action = Marker::DELETEALL;
  array.markers.push_back(clear);
  if (fairway.center.empty()) {return array;}
  const auto point = [](RoutePoint p, double z) {
      geometry_msgs::msg::Point result;
      result.x = p.x;
      result.y = p.y;
      result.z = z;
      return result;
    };
  const auto marker = [&](const std::string & ns, int type, bool clockwise) {
      Marker m;
      m.header = header;
      m.ns = ns;
      m.type = type;
      m.pose.orientation.w = 1;
      m.scale.x = m.scale.y = m.scale.z = 1;
      m.color.r = clockwise ? 0.05 : 0.15;
      m.color.g = clockwise ? 0.9 : 0.45;
      m.color.b = clockwise ? 0.55 : 1.0;
      m.color.a = 1;
      return m;
    };
  for (bool clockwise : {true, false}) {
    const auto & edge = clockwise ? fairway.inner : fairway.outer;
    auto fill = marker(clockwise ? "cw_area" : "ccw_area", Marker::TRIANGLE_LIST, clockwise);
    fill.color.a = 0.3;
    for (std::size_t i = 1; i < edge.size(); ++i) {
      const auto a = point(fairway.center[i - 1].position, 0.7);
      const auto b = point(fairway.center[i].position, 0.7);
      const auto c = point(edge[i], 0.7), d = point(edge[i - 1], 0.7);
      // 两侧三角形均朝上，避免 RViz 背面剔除导致填充不可见。
      if (clockwise) {
        fill.points.insert(fill.points.end(), {a, c, b, a, d, c});
      } else {
        fill.points.insert(fill.points.end(), {a, b, c, a, c, d});
      }
    }
    array.markers.push_back(std::move(fill));
    auto border = marker(clockwise ? "inner_boundary" : "outer_boundary",
      Marker::LINE_STRIP, clockwise);
    border.scale.x = 0.5;
    for (const auto p : edge) {border.points.push_back(point(p, 1.2));}
    array.markers.push_back(std::move(border));
    const auto & poses = clockwise ? fairway.clockwise : fairway.counterclockwise;
    double arc = 30;
    int id = 0;
    for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
      if (i > 0) {
        arc += std::hypot(poses[i].position.x - poses[i - 1].position.x,
          poses[i].position.y - poses[i - 1].position.y);
      }
      if (arc < 30) {continue;}
      arc = 0;
      auto arrow = marker(clockwise ? "cw_direction" : "ccw_direction", Marker::ARROW, clockwise);
      arrow.id = id++;
      arrow.pose.position = point(poses[i].position, 1.5);
      arrow.pose.orientation.z = std::sin(poses[i].yaw / 2);
      arrow.pose.orientation.w = std::cos(poses[i].yaw / 2);
      arrow.scale.x = 6;
      arrow.scale.y = 1.5;
      arrow.scale.z = 1;
      array.markers.push_back(std::move(arrow));
    }
  }
  return array;
}
}  // namespace

/** @brief 根据 GNSS 记录及海图原点发布安全闭合航线和双向靠右航道。 */
class GnssPathPublisher : public rclcpp::Node
{
public:
  /** @brief 加载 GNSS 记录及航道配置，等待原点后生成或恢复航道。 */
  GnssPathPublisher()
  : Node("gnss_path_publisher")
  {
    std::filesystem::path file = declare_parameter(
      "trajectory_file", "matlab_nv_lse_pe_gnss.txt");
    if (file.is_relative()) {
      file = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("usv_map_tools")) / file;
    }
    load_trajectory(file);
    chart_file_ = std::filesystem::absolute(declare_parameter(
      "chart_file", "src/usv_map/resource/generated/reservoir_test_blocked.xml"));
    route_spacing_ = declare_parameter("route_spacing_m", 2.0);
    lane_width_ = declare_parameter("lane_width_m", 15.0);
    clearance_ = declare_parameter("clearance_m", 5.0);
    centerline_inset_ = declare_parameter("centerline_inset_m", 5.0);
    inward_bow_ = declare_parameter("inward_bow_m", 8.0);
    bottom_rounding_ = declare_parameter("bottom_rounding_m", 20.0);
    output_chart_ = std::filesystem::absolute(declare_parameter(
      "output_chart", "src/usv_map/resource/generated/reservoir_test_fairway.xml"));
    // 暗礁编辑模式固定航道，便于在航道内放置测试障碍物。
    preserve_fairway_ = declare_parameter("preserve_fairway", false);
    if (!std::isfinite(route_spacing_) || route_spacing_ <= 0.0) {
      throw std::invalid_argument("route_spacing_m must be finite and positive");
    }
    if (!std::isfinite(clearance_) || clearance_ < 0.0) {
      throw std::invalid_argument("clearance_m must be finite and nonnegative");
    }
    if (!std::isfinite(lane_width_) || lane_width_ <= 0.0 ||
      !std::isfinite(lane_width_ + clearance_ + centerline_inset_))
    {
      throw std::invalid_argument("lane_width_m must be finite and positive");
    }
    if (!std::isfinite(centerline_inset_) || centerline_inset_ < 0.0) {
      throw std::invalid_argument("centerline_inset_m must be finite and nonnegative");
    }
    if (!std::isfinite(inward_bow_) || inward_bow_ < 0.0 ||
      !std::isfinite(bottom_rounding_) || bottom_rounding_ < 0.0)
    {
      throw std::invalid_argument("route shaping distances must be finite and nonnegative");
    }
    if (std::filesystem::weakly_canonical(chart_file_) ==
      std::filesystem::weakly_canonical(output_chart_))
    {
      throw std::invalid_argument("output_chart must differ from chart_file");
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/test_tools/gnss_path", qos);
    route_pub_ = create_publisher<nav_msgs::msg::Path>("/test_tools/clockwise_route", qos);
    cw_pub_ = create_publisher<nav_msgs::msg::Path>("/test_tools/fairway_clockwise", qos);
    ccw_pub_ = create_publisher<nav_msgs::msg::Path>("/test_tools/fairway_counterclockwise", qos);
    fairway_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/test_tools/fairway", qos);
    origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", rclcpp::QoS(1),
      [this](const sensor_msgs::msg::NavSatFix & message) {publish_path(message);});
    RCLCPP_INFO(
      get_logger(), "Loaded %zu GNSS points; waiting for test chart origin on /fix",
      coordinates_.size());
  }

private:
  void load_trajectory(const std::filesystem::path & file)
  {
    std::ifstream input(file);
    if (!input) {
      throw std::runtime_error("cannot open GNSS trajectory: " + file.string());
    }
    std::string line;
    std::size_t skipped = 0;
    while (std::getline(input, line)) {
      if (line.find_first_not_of(" \t\r") == std::string::npos) {
        continue;
      }
      std::istringstream row(line);
      std::array<std::string, 5> columns;
      if (!(row >> columns[0] >> columns[1] >> columns[2] >> columns[3] >> columns[4])) {
        ++skipped;
        continue;
      }
      try {
        // MATLAB 日志第 4、5 列分别为纬度、经度，单位 rad；其余字段不参与航线生成。
        const double latitude = parse_number(columns[3]);
        const double longitude = parse_number(columns[4]);
        if (std::abs(latitude) > kPi / 2.0 || std::abs(longitude) > kPi) {
          throw std::invalid_argument("coordinate outside geographic range");
        }
        coordinates_.push_back({latitude, longitude});
      } catch (const std::exception &) {
        ++skipped;
      }
    }
    if (coordinates_.empty()) {
      throw std::runtime_error("GNSS trajectory contains no valid coordinates: " + file.string());
    }
    if (skipped != 0) {
      RCLCPP_WARN(get_logger(), "Skipped %zu invalid GNSS rows", skipped);
    }
  }

  void publish_path(const sensor_msgs::msg::NavSatFix & fix)
  {
    // /fix 同时供测试编辑器和卫星图层定位地图原点。
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
      std::abs(fix.latitude) > 90.0 || std::abs(fix.longitude) > 180.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid test chart origin on /fix");
      return;
    }
    const std::array<double, 2> origin{fix.latitude, fix.longitude};
    std::error_code file_error;
    const bool saved_fairway = preserve_fairway_ && std::filesystem::is_regular_file(output_chart_);
    const auto & watched_file = saved_fairway ? output_chart_ : chart_file_;
    const auto chart_write = std::filesystem::last_write_time(watched_file, file_error);
    if (last_origin_ && *last_origin_ == origin && last_chart_write_ == chart_write) {
      return;
    }
    const double latitude = fix.latitude * kPi / 180.0;
    const double longitude = fix.longitude * kPi / 180.0;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    path.poses.reserve(coordinates_.size());
    std::vector<RoutePoint> reference;
    reference.reserve(coordinates_.size());
    const double east_scale = kEarthRadius * std::cos(latitude);
    for (const auto & coordinate : coordinates_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = east_scale * (coordinate[1] - longitude);
      pose.pose.position.y = kEarthRadius * (coordinate[0] - latitude);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
      reference.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    path_pub_->publish(path);
    // 地图、原点变化或搜索失败时清除保留航线，避免 RViz 显示已失效的校验结果。
    nav_msgs::msg::Path route_path;
    route_path.header = path.header;
    route_pub_->publish(route_path);
    cw_pub_->publish(route_path);
    ccw_pub_->publish(route_path);
    fairway_pub_->publish(fairway_markers({}, path.header));
    try {
      if (file_error) {
        throw std::runtime_error("cannot read route chart: " + watched_file.string());
      }
      if (saved_fairway) {
        const auto fairway = load_saved_fairway(usv_map::load_chart(
            output_chart_.string(), fix.longitude, fix.latitude));
        publish_fairway(fairway, path.header);
        RCLCPP_INFO(get_logger(),
          "Showing saved test fairway unchanged; newly placed reefs may obstruct its lanes");
        last_origin_ = origin;
        last_chart_write_ = chart_write;
        return;
      }
      const auto chart = usv_map::load_chart(
        chart_file_.string(), fix.longitude, fix.latitude);
      RCLCPP_INFO(get_logger(),
        "Fitting 2 x %.1f m fairway; centreline inset %.1f m; required shoreline clearance %.1f m",
        lane_width_, centerline_inset_, lane_width_ + clearance_);
      // 初始路线允许向外岸预留内缩量；最终中心线和完整航道需分别校验陆地净距。
      const auto route = make_safe_clockwise_route(
        reference, route_spacing_, chart, std::max(0.0, lane_width_ + clearance_ - centerline_inset_),
        inward_bow_, bottom_rounding_);
      const auto fairway = make_fairway(
        route, lane_width_, route_spacing_, chart, clearance_, centerline_inset_);
      if (std::filesystem::last_write_time(chart_file_) != chart_write) {
        throw std::runtime_error("chart changed during route generation; retrying on next /fix");
      }
      save_fairway_chart(chart_file_, output_chart_, chart, fairway, clearance_);
      if (std::filesystem::last_write_time(chart_file_) != chart_write) {
        throw std::runtime_error("chart changed during XML export; retrying on next /fix");
      }
      publish_fairway(fairway, path.header);
      RCLCPP_INFO(
        get_logger(), "Published bidirectional fairway; centreline clearance >= %.3f m; "
        "edge clearance >= %.3f m; saved %s",
        fairway.centerline_clearance_m, fairway.minimum_clearance_m, output_chart_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot generate fairway (XML not refreshed): %s", error.what());
    }
    last_origin_ = origin;
    last_chart_write_ = chart_write;
    RCLCPP_INFO(get_logger(), "Published %zu poses on /test_tools/gnss_path", path.poses.size());
  }

  void publish_fairway(const Fairway & fairway, const std_msgs::msg::Header & header)
  {
    route_pub_->publish(path_message(fairway.center, header));
    cw_pub_->publish(path_message(fairway.clockwise, header));
    ccw_pub_->publish(path_message(fairway.counterclockwise, header));
    fairway_pub_->publish(fairway_markers(fairway, header));
  }

  std::vector<std::array<double, 2>> coordinates_;
  std::optional<std::array<double, 2>> last_origin_;
  std::optional<std::filesystem::file_time_type> last_chart_write_;
  std::filesystem::path chart_file_;
  std::filesystem::path output_chart_;
  double route_spacing_ = 2.0;
  double lane_width_ = 15.0;
  double clearance_ = 5.0;
  double centerline_inset_ = 5.0;
  double inward_bow_ = 8.0;
  double bottom_rounding_ = 20.0;
  bool preserve_fairway_ = false;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr cw_pub_, ccw_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fairway_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
};
}  // namespace usv_map_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<usv_map_tools::GnssPathPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("gnss_path_publisher"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
