#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "channel_navigation_manager/geometry.hpp"

namespace cnm = channel_navigation_manager;
using std::placeholders::_1;

class ChannelNavigationNode : public rclcpp::Node
{
public:
  // [功能与联系] 创建海图几何、地图、船位和目标订阅；加载分类阈值，等待输入后生成全图航道语义供两级规划使用。
  ChannelNavigationNode()
  : Node("channel_navigation_node")
  {
    lane_data_topic_ = declare_parameter("lane_data_topic", "/chart_lane_data");
    costmap_topic_ = declare_parameter("costmap_topic", "/local_costmap");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
    lane_costmap_topic_ = declare_parameter("lane_costmap_topic", "/channel/lane_costmap");
    marker_topic_ = declare_parameter("marker_topic", "/channel/navigation_markers");
    status_topic_ = declare_parameter("status_topic", "/channel/status");
    outside_value_ = declare_parameter("outside_value", cnm::kOutsideChannel);
    opposite_value_ = declare_parameter("opposite_value", cnm::kOppositeLane);
    main_lane_edge_value_ = declare_parameter("main_lane_edge_value", 20);
    center_clearance_ = declare_parameter("center_clearance", 16.0);
    minimum_course_speed_ = declare_parameter("minimum_course_speed", 0.10);
    reclassify_angle_ = declare_parameter("reclassify_angle", 12.0 * M_PI / 180.0);
    lane_adjacency_distance_ = declare_parameter("lane_adjacency_distance", 160.0);
    lane_continuity_angle_ = declare_parameter("lane_continuity_angle", 80.0 * M_PI / 180.0);
    reverse_enter_angle_ = declare_parameter("reverse_enter_angle", 110.0 * M_PI / 180.0);
    reverse_complete_angle_ = declare_parameter("reverse_complete_angle", 35.0 * M_PI / 180.0);
    minimum_reverse_goal_distance_ = declare_parameter("minimum_reverse_goal_distance", 10.0);

    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    lane_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(lane_costmap_topic_, latched);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, latched);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latched);
    lane_sub_ = create_subscription<std_msgs::msg::String>(
      lane_data_topic_, latched, std::bind(&ChannelNavigationNode::laneCallback, this, _1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, latched, std::bind(&ChannelNavigationNode::costmapCallback, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&ChannelNavigationNode::odomCallback, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&ChannelNavigationNode::goalCallback, this, _1));
    RCLCPP_INFO(get_logger(), "C++ channel manager ready; lane roles follow COG and task intent");
  }

private:
  // [功能与联系] 解析river_chart的航道JSON，保存多边形、lane_id和ORIENT；通过publishIfReady强制重建语义图，不搜索或清空全局路线。
  void laneCallback(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const auto payload = nlohmann::json::parse(message->data);
      std::vector<cnm::Lane> parsed;
      for (const auto & item : payload.value("lanes", nlohmann::json::array())) {
        cnm::Lane lane;
        lane.lane_id = item.at("lane_id").get<int>();
        lane.orient_deg = item.at("orient_deg").get<double>();
        for (const auto & point : item.at("points")) {
          lane.points.push_back({point.at(0).get<double>(), point.at(1).get<double>()});
        }
        if (lane.points.size() >= 4U) {parsed.push_back(std::move(lane));}
      }
      lanes_ = std::move(parsed);
      frame_id_ = payload.value("frame_id", std::string("map"));
      published_policy_yaw_.reset();
      RCLCPP_INFO(get_logger(), "Cached %zu TSSLPT lane polygons", lanes_.size());
      publishIfReady(true);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Invalid /chart_lane_data: %s", exception.what());
    }
  }

  // [功能与联系] 缓存物理地图及其网格元数据；航道节点用于对齐语义图，规划/控制节点用于碰撞和数据就绪检查。
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    const bool changed = !costmap_ || costmap_->info.width != message->info.width ||
      costmap_->info.height != message->info.height ||
      costmap_->info.resolution != message->info.resolution ||
      costmap_->info.origin.position.x != message->info.origin.position.x ||
      costmap_->info.origin.position.y != message->info.origin.position.y;
    costmap_ = message;
    publishIfReady(changed);
  }

  // [功能与联系] 缓存里程计并计算COG；低速保持方向，经updatePolicyCourse与publishIfReady按角度阈值重建语义图。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    odom_ = message;
    const double yaw = tf2::getYaw(message->pose.pose.orientation);
    const double body_vx = message->twist.twist.linear.x;
    const double body_vy = message->twist.twist.linear.y;
    if (std::hypot(body_vx, body_vy) >= minimum_course_speed_) {
      course_yaw_ = cnm::normalizeAngle(yaw + std::atan2(body_vy, body_vx));
    } else if (!course_yaw_) {
      course_yaw_ = yaw;
    }
    updatePolicyCourse(false);
    // 普通COG微小变化由reclassify_angle节流；不能使用force，否则会在每个
    // odom回调(约20Hz)重建整幅航道图。
    publishIfReady(false);
  }

  // [功能与联系] 缓存最终任务目标，更新后方任务的反向意图，再按需重建航道分类；不管理全局航点。
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    goal_ = message;
    const bool changed = updatePolicyCourse(true);
    publishIfReady(changed);
  }

  // [功能与联系] 通常采用实际COG；目标明确在后方时提前采用反向任务方位，转向完成后回到COG，供种子航段选择。
  bool updatePolicyCourse(const bool goal_changed)
  {
    if (!course_yaw_) {return false;}
    const auto previous = policy_yaw_;
    if (goal_changed) {
      reverse_intent_yaw_.reset();
      if (goal_ && odom_) {
        const auto & own = odom_->pose.pose.position;
        const auto & goal = goal_->pose.position;
        if (cnm::isReverseTask(
            *course_yaw_, own.x, own.y, goal.x, goal.y, reverse_enter_angle_,
            minimum_reverse_goal_distance_))
        {
          reverse_intent_yaw_ = cnm::bearingAndDistance(own.x, own.y, goal.x, goal.y).first;
        }
      }
    }
    if (reverse_intent_yaw_ &&
      std::abs(cnm::normalizeAngle(*course_yaw_ - *reverse_intent_yaw_)) <= reverse_complete_angle_)
    {
      reverse_intent_yaw_.reset();
    }
    policy_yaw_ = reverse_intent_yaw_ ? reverse_intent_yaw_ : course_yaw_;
    return !previous || std::abs(cnm::normalizeAngle(*policy_yaw_ - *previous)) > 1.0e-6;
  }

  // [功能与联系] 检查静态几何、地图及策略方向，按角度阈值节流分类；传播主航道、绘制语义中心梯度并发布图/可视化/状态。
  void publishIfReady(const bool force)
  {
    if (!policy_yaw_ && course_yaw_) {updatePolicyCourse(false);}
    if (lanes_.empty() || !costmap_ || !policy_yaw_) {return;}
    if (!force && published_policy_yaw_ &&
      std::abs(cnm::normalizeAngle(*policy_yaw_ - *published_policy_yaw_)) < reclassify_angle_)
    {
      return;
    }
    published_policy_yaw_ = policy_yaw_;
    const auto & own = odom_->pose.pose.position;
    main_lane_mask_ = cnm::continuousMainLaneMask(
      lanes_, *policy_yaw_, own.x, own.y, lane_adjacency_distance_, lane_continuity_angle_);
    nav_msgs::msg::OccupancyGrid lane_map;
    lane_map.header.frame_id = costmap_->header.frame_id;
    lane_map.header.stamp = now();
    lane_map.info = costmap_->info;
    lane_map.data.assign(
      static_cast<std::size_t>(lane_map.info.width * lane_map.info.height),
      static_cast<int8_t>(outside_value_));

    // 对向先画、同向后画，使多边形重叠边界始终由主航道覆盖。
    std::vector<std::size_t> ordered;
    for (std::size_t index = 0; index < lanes_.size(); ++index) {ordered.push_back(index);}
    std::stable_sort(ordered.begin(), ordered.end(), [this](const auto a, const auto b) {
      return static_cast<int>(main_lane_mask_[a]) < static_cast<int>(main_lane_mask_[b]);
    });
    for (const auto index : ordered) {
      const auto & lane = lanes_[index];
      const int value = main_lane_mask_[index] ? cnm::kMainLane : opposite_value_;
      cnm::fillPolygon(
        lane_map.data, static_cast<int>(lane_map.info.width),
        static_cast<int>(lane_map.info.height), lane_map.info.resolution,
        lane_map.info.origin.position.x, lane_map.info.origin.position.y, lane.points, value);
    }
    cnm::applyMainLaneCenterGradient(
      lane_map.data, static_cast<int>(lane_map.info.width),
      static_cast<int>(lane_map.info.height), lane_map.info.resolution,
      main_lane_edge_value_, center_clearance_);

    // 联合区域梯度能消除相邻航段接缝，但宽航段会抬高相连窄航段中心值。
    // 再计算每个同向TSSLPT自身的归一化梯度并取较小值，兼顾无假接缝与
    // “每个局部航段中心最低”。这仍只修改语义软代价，不覆盖障碍物层。
    for (std::size_t index = 0; index < lanes_.size(); ++index) {
      if (!main_lane_mask_[index]) {continue;}
      const auto & lane = lanes_[index];
      std::vector<int8_t> local(lane_map.data.size(), static_cast<int8_t>(outside_value_));
      cnm::fillPolygon(
        local, static_cast<int>(lane_map.info.width), static_cast<int>(lane_map.info.height),
        lane_map.info.resolution, lane_map.info.origin.position.x,
        lane_map.info.origin.position.y, lane.points, cnm::kMainLane);
      cnm::applyMainLaneCenterGradient(
        local, static_cast<int>(lane_map.info.width), static_cast<int>(lane_map.info.height),
        lane_map.info.resolution, main_lane_edge_value_, center_clearance_);
      for (std::size_t i = 0; i < lane_map.data.size(); ++i) {
        if (lane_map.data[i] <= main_lane_edge_value_ && local[i] <= main_lane_edge_value_) {
          lane_map.data[i] = std::min(lane_map.data[i], local[i]);
        }
      }
    }
    lane_pub_->publish(lane_map);
    publishMarkers();
    publishStatus();
  }

  // [功能与联系] 绘制主/对向航段和策略方向供RViz检查分类；不修改物理地图或控制命令。
  void publishMarkers()
  {
    if (!policy_yaw_) {return;}
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);
    const auto stamp = now();
    for (std::size_t index = 0; index < lanes_.size(); ++index) {
      const auto & lane = lanes_[index];
      const bool main = index < main_lane_mask_.size() && main_lane_mask_[index];
      visualization_msgs::msg::Marker outline;
      outline.header.frame_id = frame_id_;
      outline.header.stamp = stamp;
      outline.ns = main ? "main_lane" : "opposite_lane";
      outline.id = static_cast<int>(index * 2U);
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.pose.orientation.w = 1.0;
      outline.scale.x = main ? 1.3 : 0.8;
      outline.color.r = main ? 0.1F : 1.0F;
      outline.color.g = main ? 1.0F : 0.25F;
      outline.color.b = main ? 0.2F : 0.1F;
      outline.color.a = 0.95F;
      for (const auto & point : lane.points) {
        geometry_msgs::msg::Point p;
        p.x = point.x; p.y = point.y; p.z = 0.45;
        outline.points.push_back(p);
      }
      output.markers.push_back(outline);

      visualization_msgs::msg::Marker arrow;
      arrow.header = outline.header;
      arrow.ns = "lane_orient";
      arrow.id = static_cast<int>(index * 2U + 1U);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation.w = std::cos(cnm::s57OrientationToYaw(lane.orient_deg) / 2.0);
      arrow.pose.orientation.z = std::sin(cnm::s57OrientationToYaw(lane.orient_deg) / 2.0);
      const std::size_t count = lane.points.size() > 1U ? lane.points.size() - 1U : lane.points.size();
      for (std::size_t i = 0; i < count; ++i) {
        arrow.pose.position.x += lane.points[i].x / static_cast<double>(count);
        arrow.pose.position.y += lane.points[i].y / static_cast<double>(count);
      }
      arrow.scale.x = 10.0; arrow.scale.y = 2.0; arrow.scale.z = 2.0;
      arrow.color = outline.color;
      output.markers.push_back(arrow);
    }
    marker_pub_->publish(output);
  }

  // [功能与联系] 发布COG、策略方向、反向状态、所在区域和lane_id的JSON诊断，帮助检查主航道选择。
  void publishStatus()
  {
    if (!course_yaw_ || !policy_yaw_) {return;}
    std::string region = "OUTSIDE";
    int lane_id = -1;
    if (odom_) {
      const double x = odom_->pose.pose.position.x;
      const double y = odom_->pose.pose.position.y;
      for (std::size_t index = 0; index < lanes_.size(); ++index) {
        const auto & lane = lanes_[index];
        if (cnm::pointInPolygon(x, y, lane.points)) {
          lane_id = lane.lane_id;
          region = index < main_lane_mask_.size() && main_lane_mask_[index] ?
            "MAIN" : "OPPOSITE";
          break;
        }
      }
    }
    nlohmann::json status = {
      {"course_deg_ros", std::round(*course_yaw_ * 18000.0 / M_PI) / 100.0},
      {"policy_course_deg_ros", std::round(*policy_yaw_ * 18000.0 / M_PI) / 100.0},
      {"reverse_intent", reverse_intent_yaw_.has_value()},
      {"current_region", region}, {"lane_id", lane_id},
      {"priority", {"SAFETY", "COLREGS", "RIGHT_HAND", "EFFICIENCY"}}
    };
    std_msgs::msg::String message;
    message.data = status.dump();
    status_pub_->publish(message);
  }

  std::string lane_data_topic_, costmap_topic_, odom_topic_, goal_topic_;
  std::string lane_costmap_topic_, marker_topic_, status_topic_, frame_id_{"map"};
  int outside_value_{45}, opposite_value_{75}, main_lane_edge_value_{20};
  double center_clearance_{16.0}, minimum_course_speed_{0.1}, reclassify_angle_{0.2};
  double lane_adjacency_distance_{160.0}, lane_continuity_angle_{1.3962634};
  double reverse_enter_angle_{1.92}, reverse_complete_angle_{0.61};
  double minimum_reverse_goal_distance_{10.0};
  std::vector<cnm::Lane> lanes_;
  std::vector<bool> main_lane_mask_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  std::optional<double> course_yaw_, policy_yaw_, reverse_intent_yaw_, published_policy_yaw_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr lane_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChannelNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
