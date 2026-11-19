/**
 * @file channel_navigation_node.cpp
 * @brief 订阅感知快照，按目标方向与探测障碍更新航道代价地图。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <usv_interfaces/msg/obstacle_detection_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "channel_navigation_manager/geometry.hpp"
#include "channel_navigation_manager/chart_data.hpp"
#include "channel_navigation_manager/perception_geometry.hpp"

namespace cnm = channel_navigation_manager;
using std::placeholders::_1;
using SteadyClock = std::chrono::steady_clock;

/** @brief 在静态代价地图上合并通行方向代价与临时感知障碍。 */
class ChannelNavigationNode final : public rclcpp::Node
{
public:
  /** @brief 加载航道与感知参数，创建地图、里程计、目标和探测快照接口。 */
  ChannelNavigationNode()
  : Node("channel_navigation_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    chart_costmap_topic_ = declare_parameter("chart_costmap_topic", "/chart_costmap");
    chart_lane_topic_ = declare_parameter("chart_lane_topic", "/chart_lane_data");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
    goal_path_topic_ = declare_parameter("goal_path_topic", "/path");
    channel_costmap_topic_ = declare_parameter("channel_costmap_topic", "/channel_costmap");
    marker_topic_ = declare_parameter("marker_topic", "/channel_navigation_markers");
    const auto detections_topic = declare_parameter("detections_topic", "/obstacle_detections");
    detection_timeout_s_ = declare_parameter("detection_timeout_s", 3.0);
    if (!std::isfinite(detection_timeout_s_) || detection_timeout_s_ <= 0.0) {
      throw std::invalid_argument("detection_timeout_s must be positive");
    }
    detection_inflation_m_ = declare_parameter("detection_inflation_m", 2.0);
    if (!std::isfinite(detection_inflation_m_) || detection_inflation_m_ < 0.0) {
      throw std::invalid_argument("detection_inflation_m must be finite and nonnegative");
    }
    same_direction_cost_ = declare_parameter("same_direction_cost", cnm::kSameDirectionCost);
    opposite_direction_cost_ = declare_parameter(
      "opposite_direction_cost", cnm::kOppositeDirectionCost);
    open_water_cost_ = declare_parameter("open_water_cost", cnm::kOpenWaterCost);
    obstacle_cost_ = declare_parameter("obstacle_cost", cnm::kObstacleCost);
    lane_center_cost_ = declare_parameter("lane_center_cost", 8);
    if (lane_center_cost_ < 0 || same_direction_cost_ + lane_center_cost_ >= open_water_cost_ ||
      opposite_direction_cost_ + lane_center_cost_ >= 75)
    {
      throw std::invalid_argument("lane_center_cost must preserve lane/water/obstacle cost ordering");
    }
    direction_tolerance_deg_ = declare_parameter("direction_tolerance_deg", 90.0);
    direction_arrow_length_m_ = declare_parameter("direction_arrow_length_m", 12.0);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 5.0);

    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(channel_costmap_topic_, latched);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, latched);
    lane_sub_ = create_subscription<std_msgs::msg::String>(
      chart_lane_topic_, latched, std::bind(&ChannelNavigationNode::laneCallback, this, _1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      chart_costmap_topic_, latched, std::bind(&ChannelNavigationNode::costmapCallback, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20), std::bind(&ChannelNavigationNode::odomCallback, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10), std::bind(&ChannelNavigationNode::goalCallback, this, _1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      goal_path_topic_, rclcpp::QoS(10), std::bind(&ChannelNavigationNode::pathCallback, this, _1));

    detections_sub_ = create_subscription<usv_interfaces::msg::ObstacleDetectionArray>(
      detections_topic, rclcpp::QoS(1).reliable(),
      [this](usv_interfaces::msg::ObstacleDetectionArray::ConstSharedPtr msg) {
        const double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
        if ((msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ||
          age > detection_timeout_s_ || age < -0.5) {return;}
        pending_detections_ = msg;
        detection_received_ = SteadyClock::now();
        // 空快照无需 TF 即可清除；绝不把上一帧目标追加到新快照。
        if (msg->obstacles.empty()) {clearDetections();}
        dirty_ = true;
      });
    const double rate = std::max(0.5, publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      std::bind(&ChannelNavigationNode::publishIfDirty, this));
    RCLCPP_INFO(
      get_logger(), "Channel costmap manager ready: %s, perception input %s",
      channel_costmap_topic_.c_str(), detections_topic.c_str());
  }

private:
  /** @brief 解析并替换航道缓存，标记栅格缓存需要重建；解析失败时保留已有航道。 */
  void laneCallback(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      boost::property_tree::ptree payload;
      std::istringstream stream(message->data);
      boost::property_tree::read_json(stream, payload);
      std::vector<cnm::Lane> lanes;
      const auto lanes_node = payload.get_child_optional("lanes");
      if (lanes_node) for (const auto & entry : *lanes_node) {
        const auto & item = entry.second;
        cnm::Lane lane;
        lane.lane_id = item.get("lane_id", -1);
        lane.orient_deg = item.get("orient_deg", 0.0);
        lane.direction = item.get("direction", std::string{});
        if (!cnm::isTrafficDirection(lane.direction)) {
          lane.direction.clear();
        }
        lane.points = cnm::readPolygon(item);
        if (lane.points.size() >= 3U) {
          lanes.push_back(std::move(lane));
        }
      }

      lanes_ = std::move(lanes);
      route_direction_.reset();
      raster_cache_dirty_ = true;
      frame_id_ = payload.get("frame_id", std::string("map"));
      dirty_ = true;
      RCLCPP_INFO(get_logger(), "Cached %zu lanes (obstacle truth ignored)", lanes_.size());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Invalid %s: %s", chart_lane_topic_.c_str(), error.what());
    }
  }

  /** @brief 更新静态底图；坐标系变化时清除感知覆盖，并使栅格缓存失效。 */
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (chart_costmap_ && chart_costmap_->header.frame_id != message->header.frame_id) {
      clearDetections();
    }
    chart_costmap_ = message;
    detection_cells_dirty_ = true;
    raster_cache_dirty_ = true;
    frame_id_ = message->header.frame_id.empty() ? frame_id_ : message->header.frame_id;
    dirty_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    odom_ = message;
    dirty_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    goal_ = message;
    route_yaw_.reset();
    route_direction_.reset();
    dirty_ = true;
  }

  /** @brief 用路径首个有效航段确定任务方向，避免旧的单点目标影响航道代价。 */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (message->header.frame_id != frame_id_ || message->poses.size() < 2) return;
    const auto & start = message->poses.front().pose.position;
    for (const auto & pose : message->poses) {
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != frame_id_) ||
          !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) return;
    }
    for (std::size_t i = 1; i < message->poses.size(); ++i) {
      const auto & end = message->poses[i].pose.position;
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      if (std::hypot(dx, dy) > 1e-3) {
        goal_.reset();
        route_yaw_ = std::atan2(dy, dx);
        route_direction_.reset();
        dirty_ = true;
        return;
      }
    }
  }

  /**
   * @brief 清除感知障碍及其栅格覆盖。
   * @param[in] clear_pending 为 true 时同时丢弃等待 TF 的快照。
   */
  void clearDetections(bool clear_pending = true)
  {
    dirty_ = dirty_ || !detected_circles_.empty();
    detected_circles_.clear();
    detection_cells_.clear();
    if (clear_pending) {pending_detections_.reset();}
    active_detection_stamp_.reset();
  }

  /** @brief 清除过期感知，并用观测时刻的 TF 尝试处理待接收快照；缺失 TF 时保留重试。 */
  void updateDetections()
  {
    const auto expired = [this](const builtin_interfaces::msg::Time & stamp) {
        const double age = (now() - rclcpp::Time(stamp)).seconds();
        return age > detection_timeout_s_ || age < -0.5;
      };
    if (active_detection_stamp_ &&
      (expired(*active_detection_stamp_) ||
      std::chrono::duration<double>(SteadyClock::now() - detection_received_).count() >
      detection_timeout_s_))
    {
      clearDetections(false);
      RCLCPP_WARN(get_logger(), "Perception expired; temporary obstacle overlay cleared");
    }
    if (!pending_detections_ || !chart_costmap_) {return;}
    if (expired(pending_detections_->header.stamp) ||
      std::chrono::duration<double>(SteadyClock::now() - detection_received_).count() >
      detection_timeout_s_)
    {
      pending_detections_.reset();
      return;
    }
    try {
      if (pending_detections_->header.frame_id.empty()) {
        throw std::invalid_argument("detected obstacles need a sensor frame");
      }
      const auto transform = tf_buffer_.lookupTransform(
        chart_costmap_->header.frame_id, pending_detections_->header.frame_id,
        rclcpp::Time(pending_detections_->header.stamp));
      const auto & t = transform.transform;
      const tf2::Transform map_from_sensor(
        tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
        tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
      std::vector<cnm::Circle> circles;
      for (const auto & obstacle : pending_detections_->obstacles) {
        circles.push_back(cnm::detectionCircle(obstacle, map_from_sensor, detection_inflation_m_));
      }
      detected_circles_ = std::move(circles);
      active_detection_stamp_ = pending_detections_->header.stamp;
      pending_detections_.reset();
      detection_cells_dirty_ = true;
      dirty_ = true;
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for valid perception/TF: %s", error.what());
    }
  }

  /** @brief 周期检查感知有效性，在输入变化时重建并发布航道代价与可视化标记。 */
  void publishIfDirty()
  {
    updateDetections();
    if (!dirty_ || !chart_costmap_) {
      return;
    }
    dirty_ = false;

    const auto & source = *chart_costmap_;
    nav_msgs::msg::OccupancyGrid output;
    output.header.frame_id = source.header.frame_id.empty() ? frame_id_ : source.header.frame_id;
    output.header.stamp = now();
    output.info = source.info;
    const std::size_t cell_count = static_cast<std::size_t>(output.info.width) * output.info.height;
    // 每帧从正常水域代价开始，随后覆盖航道、感知暗礁和静态障碍。
    output.data.assign(cell_count, static_cast<int8_t>(open_water_cost_));

    if (raster_cache_dirty_) {
      const auto rasterize = [&output](const cnm::Polygon & polygon) {
          return cnm::polygonCells(
            static_cast<int>(output.info.width), static_cast<int>(output.info.height),
            output.info.resolution, output.info.origin.position.x, output.info.origin.position.y,
            polygon);
        };
      lane_cells_.clear();
      lane_center_offsets_.clear();
      for (const auto & lane : lanes_) {
        lane_cells_.push_back(rasterize(lane.points));
        std::vector<int> offsets;
        offsets.reserve(lane_cells_.back().size());
        for (const auto index : lane_cells_.back()) {
          const double x = output.info.origin.position.x +
            (static_cast<double>(index % output.info.width) + 0.5) * output.info.resolution;
          const double y = output.info.origin.position.y +
            (static_cast<double>(index / output.info.width) + 0.5) * output.info.resolution;
          offsets.push_back(static_cast<int>(std::lround(
            lane_center_cost_ * cnm::laneCenterOffset({x, y}, lane))));
        }
        lane_center_offsets_.push_back(std::move(offsets));
      }
      static_obstacle_cells_.clear();
      for (std::size_t i = 0; i < std::min(cell_count, source.data.size()); ++i) {
        if (source.data[i] >= obstacle_cost_) {static_obstacle_cells_.push_back(i);}
      }
      raster_cache_dirty_ = false;
    }
    if (detection_cells_dirty_) {
      detection_cells_.clear();
      for (const auto & circle : detected_circles_) {
        detection_cells_.push_back(cnm::circleCells(
          static_cast<int>(output.info.width), static_cast<int>(output.info.height),
          output.info.resolution, output.info.origin.position.x, output.info.origin.position.y,
          circle));
      }
      detection_cells_dirty_ = false;
    }
    const auto paint = [&output](const std::vector<std::size_t> & cells, int cost) {
        for (const auto index : cells) {output.data[index] = static_cast<int8_t>(cost);}
      };
    const double tolerance = std::clamp(direction_tolerance_deg_, 0.0, 180.0) * M_PI / 180.0;
    const auto travel_yaw = travelDirection();
    if (route_yaw_ && !route_direction_ && odom_) {
      route_direction_ = preferredTrafficDirection(*route_yaw_);
    }
    const auto preferred_direction = route_direction_ ? route_direction_ :
      (travel_yaw ? preferredTrafficDirection(*travel_yaw) : std::nullopt);
    std::vector<bool> same_direction(lanes_.size(), false);
    // 重叠航段取较低通行代价，避免栅格值由海图要素顺序决定。
    std::vector<bool> lane_painted(cell_count, false);
    for (std::size_t i = 0; travel_yaw && i < lanes_.size(); ++i) {
      const auto & lane = lanes_[i];
      same_direction[i] = preferred_direction && !lane.direction.empty() ?
        cnm::normalizedTrafficDirection(lane.direction) == *preferred_direction :
        cnm::isSameNavigationDirection(*travel_yaw, lane.orient_deg, tolerance);
      const int base = same_direction[i] ? same_direction_cost_ : opposite_direction_cost_;
      for (std::size_t j = 0; j < lane_cells_[i].size(); ++j) {
        const auto index = lane_cells_[i][j];
        const int value = base + lane_center_offsets_[i][j];
        if (!lane_painted[index] || value < output.data[index])
          output.data[index] = static_cast<int8_t>(value);
        lane_painted[index] = true;
      }
    }

    // 感知只提供当前快照；每帧从基础层重建，目标消失后自然恢复航道/水域代价。
    for (const auto & cells : detection_cells_) {paint(cells, obstacle_cost_);}
    // 陆地与静态障碍最后覆盖，优先级始终高于航道通行代价。
    paint(static_obstacle_cells_, obstacle_cost_);
    costmap_pub_->publish(output);
    publishMarkers(output.header, same_direction);
  }

  std::optional<double> travelDirection() const
  {
    if (!odom_) {
      return std::nullopt;
    }
    if (route_yaw_) return route_yaw_;
    const auto & own = odom_->pose.pose.position;
    if (goal_) {
      const double dx = goal_->pose.position.x - own.x;
      const double dy = goal_->pose.position.y - own.y;
      if (std::hypot(dx, dy) > 1.0e-6) {
        return std::atan2(dy, dx);
      }
    }
    const auto & q = odom_->pose.pose.orientation;
    return cnm::bowYaw(tf2::Quaternion(q.x, q.y, q.z, q.w));
  }

  /**
   * 先按本船附近航段选定 CW/CCW，再用于整条弯曲航道。
   * 避免用同一目标方向逐段比较，造成回形航道中途反转代价。
   */
  std::optional<std::string> preferredTrafficDirection(const double travel_yaw) const
  {
    if (lanes_.empty()) {
      return std::nullopt;
    }

    const cnm::Lane * nearest = nullptr;
    double nearest_distance = std::numeric_limits<double>::infinity();
    if (odom_) {
      const cnm::Point2D position{
        odom_->pose.pose.position.x, odom_->pose.pose.position.y};
      for (const auto & lane : lanes_) {
        const double distance = cnm::pointPolygonDistance(position.x, position.y, lane.points);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = &lane;
        }
      }
    }

    const cnm::Lane * best = nullptr;
    double best_error = std::numeric_limits<double>::infinity();
    const double local_limit = nearest == nullptr ?
      std::numeric_limits<double>::infinity() : nearest_distance + 30.0;
    for (const auto & lane : lanes_) {
      if (lane.direction.empty()) {
        continue;
      }
      if (odom_) {
        const cnm::Point2D position{
          odom_->pose.pose.position.x, odom_->pose.pose.position.y};
        if (cnm::pointPolygonDistance(position.x, position.y, lane.points) > local_limit) {
          continue;
        }
      }
      const double error = std::abs(cnm::normalizeAngle(
        cnm::s57OrientationToYaw(lane.orient_deg) - travel_yaw));
      if (error < best_error) {
        best_error = error;
        best = &lane;
      }
    }
    if (best != nullptr) {
      return cnm::normalizedTrafficDirection(best->direction);
    }
    if (nearest != nullptr && !nearest->direction.empty()) {
      return cnm::normalizedTrafficDirection(nearest->direction);
    }
    return std::nullopt;
  }

  void publishMarkers(const std_msgs::msg::Header & header, const std::vector<bool> & same_direction)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    const auto stamp = now();
    for (std::size_t i = 0; i < lanes_.size(); ++i) {
      const auto & lane = lanes_[i];
      const bool same = same_direction[i];
      visualization_msgs::msg::Marker outline;
      outline.header = header;
      outline.header.stamp = stamp;
      outline.ns = same ? "same_direction_lane" : "opposite_direction_lane";
      outline.id = static_cast<int>(i);
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.pose.orientation.w = 1.0;
      outline.scale.x = 0.5;
      outline.color.r = same ? 0.1F : 1.0F;
      outline.color.g = same ? 1.0F : 0.25F;
      outline.color.b = same ? 0.2F : 0.1F;
      outline.color.a = 0.9F;
      for (const auto & point : lane.points) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x;
        marker_point.y = point.y;
        marker_point.z = 0.4;
        outline.points.push_back(marker_point);
      }
      markers.markers.push_back(outline);

      // 箭头与代价层共用方向结果：黄色表示低代价，红色表示高代价。
      if (lane.points.size() >= 3U) {
        double center_x = 0.0;
        double center_y = 0.0;
        for (const auto & point : lane.points) {
          center_x += point.x;
          center_y += point.y;
        }
        center_x /= static_cast<double>(lane.points.size());
        center_y /= static_cast<double>(lane.points.size());
        const double direction = cnm::s57OrientationToYaw(lane.orient_deg);
        visualization_msgs::msg::Marker arrow;
        arrow.header = header;
        arrow.header.stamp = stamp;
        arrow.ns = "channel_direction_arrow";
        arrow.id = static_cast<int>(i);
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = 0.8;
        arrow.scale.y = 1.8;
        arrow.scale.z = 1.8;
        arrow.color.r = 1.0F;
        arrow.color.g = same ? 0.85F : 0.05F;
        arrow.color.b = 0.05F;
        arrow.color.a = 1.0F;
        geometry_msgs::msg::Point start;
        start.x = center_x;
        start.y = center_y;
        start.z = 1.0;
        geometry_msgs::msg::Point end;
        end.x = center_x + direction_arrow_length_m_ * std::cos(direction);
        end.y = center_y + direction_arrow_length_m_ * std::sin(direction);
        end.z = 1.0;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        markers.markers.push_back(std::move(arrow));
      }
    }

    marker_pub_->publish(markers);
  }

  std::string chart_costmap_topic_, chart_lane_topic_, odom_topic_, goal_topic_, goal_path_topic_;
  std::string channel_costmap_topic_, marker_topic_, frame_id_{"map"};
  double detection_timeout_s_{3.0};
  double detection_inflation_m_{2.0};
  int same_direction_cost_{cnm::kSameDirectionCost};
  int opposite_direction_cost_{cnm::kOppositeDirectionCost};
  int open_water_cost_{cnm::kOpenWaterCost};
  int obstacle_cost_{cnm::kObstacleCost};
  int lane_center_cost_{8};
  double direction_tolerance_deg_{90.0};
  double direction_arrow_length_m_{12.0};
  double publish_rate_hz_{5.0};
  bool dirty_{true};
  std::vector<cnm::Lane> lanes_;
  std::vector<cnm::Circle> detected_circles_;
  bool raster_cache_dirty_{true}, detection_cells_dirty_{true};
  std::vector<std::vector<std::size_t>> lane_cells_, detection_cells_;
  std::vector<std::vector<int>> lane_center_offsets_;
  usv_interfaces::msg::ObstacleDetectionArray::ConstSharedPtr pending_detections_;
  std::optional<builtin_interfaces::msg::Time> active_detection_stamp_;
  SteadyClock::time_point detection_received_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<usv_interfaces::msg::ObstacleDetectionArray>::SharedPtr detections_sub_;
  std::vector<std::size_t> static_obstacle_cells_;
  nav_msgs::msg::OccupancyGrid::SharedPtr chart_costmap_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  std::optional<double> route_yaw_;
  std::optional<std::string> route_direction_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChannelNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
