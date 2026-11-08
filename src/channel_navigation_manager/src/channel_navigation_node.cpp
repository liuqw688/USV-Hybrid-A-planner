#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    lane_topology_gap_=declare_parameter("lane_topology_gap",5.0);
    declare_parameter("compact_mission_range",30.0); // legacy setting: never overrides traffic roles
    minimum_course_speed_ = declare_parameter("minimum_course_speed", 0.10);
    reclassify_angle_ = declare_parameter("reclassify_angle", 12.0 * M_PI / 180.0);
    lane_adjacency_distance_ = declare_parameter("lane_adjacency_distance", 160.0);
    lane_continuity_angle_ = declare_parameter("lane_continuity_angle", 80.0 * M_PI / 180.0);
    reverse_enter_angle_ = declare_parameter("reverse_enter_angle", 110.0 * M_PI / 180.0);
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
    RCLCPP_INFO(get_logger(),
      "C++ channel manager ready; lane roles follow mission topology and prescribed chart traffic flow");
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
      main_lane_mask_.clear();
      // Re-evaluate geometry revisions from the captured click origin, never
      // from the vessel's moved position; shuffled IDs remain deterministic.
      if(goal_) {task_course_yaw_.reset();task_lane_roles_.clear();policy_yaw_.reset();}
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
      costmap_->info.origin.position.y != message->info.origin.position.y ||
      costmap_->info.origin.orientation != message->info.origin.orientation ||
      costmap_->header.frame_id != message->header.frame_id;
    costmap_ = message;
    publishIfReady(changed);
  }

  // [功能与联系] 缓存里程计并计算COG；COG只作无任务时的备用方向/返航诊断，
  // 有用户目标后不会因船体转艏、侧滑或短时航向抖动而改变主航道。
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
    const bool changed = updatePolicyCourse(false);
    // 有活动任务时策略方向已锁定；只有初次里程计或目标任务改变才会重建，
    // 不让20Hz的COG噪声反复翻转整张航道语义图。
    publishIfReady(changed);
    if(task_course_yaw_) {publishStatus();} // 只更新诊断，不重新分类。
  }

  // [功能与联系] 缓存最终任务目标并锁存“当前船位→目标”的任务航向；新目标到来才重新定义左右航道。
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if(goal_ && goal_->header==message->header && goal_->pose==message->pose) {return;}
    goal_ = message;
    task_origin_.reset();
    if(odom_) {captureTaskOrigin();}
    updatePolicyCourse(true);
    publishIfReady(true); // 同方向的新任务也按新的起点选一次种子。
  }

  void captureTaskOrigin()
  {
    geometry_msgs::msg::PoseStamped pose;pose.header=odom_->header;pose.pose=odom_->pose.pose;
    task_origin_=pose;task_origin_course_=course_yaw_.value_or(tf2::getYaw(pose.pose.orientation));
  }

  // [功能与联系] 将任务航向锁存为主航道分类基准。COG只用于识别该任务是否要求掉头；
  // 船转向完成后仍保持同一任务下的左右航道定义，下一次用户点击目标时才重新锁存。
  bool updatePolicyCourse(const bool goal_changed)
  {
    const auto previous = policy_yaw_;
    const auto policy_changed = [&]() {
      return policy_yaw_ && (!previous ||
        std::abs(cnm::normalizeAngle(*policy_yaw_ - *previous)) > 1.0e-6);
    };
    if (goal_changed) {
      task_course_yaw_.reset();
      reverse_intent_yaw_.reset();
      main_lane_mask_.clear();
      task_lane_roles_.clear();
      task_seed_lane_id_.reset();
      policy_yaw_.reset(); // TF未就绪先等待，不按COG给新任务误分类。
    }

    // A goal can arrive before odometry during startup. Latch it on the first
    // odometry callback rather than classifying the mission from a stale COG.
    if (goal_ && odom_ && !task_course_yaw_ && !lanes_.empty()) {
      if(!task_origin_) {captureTaskOrigin();}
      geometry_msgs::msg::PoseStamped goal_in_chart = *goal_,origin_in_chart=*task_origin_;
      if (goal_->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Ignoring mission goal with an empty frame_id");
        return policy_changed();
      }
      try {
        goal_in_chart.header.stamp=rclcpp::Time(0);origin_in_chart.header.stamp=rclcpp::Time(0);
        goal_in_chart=tf_buffer_->transform(goal_in_chart,frame_id_,std::chrono::milliseconds(100));
        origin_in_chart=tf_buffer_->transform(origin_in_chart,frame_id_,std::chrono::milliseconds(100));
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting to latch channel direction: %s", exception.what());
        return policy_changed();
      }
      const auto & own=origin_in_chart.pose.position;
      const auto & goal = goal_in_chart.pose.position;
      auto [bearing, distance] = cnm::bearingAndDistance(
        own.x, own.y, goal.x, goal.y);
      if(distance<1e-6) {bearing=tf2::getYaw(goal_in_chart.pose.orientation);}
      const auto route=cnm::missionRouteDirection(lanes_,own.x,own.y,goal.x,goal.y,
        bearing,lane_topology_gap_);
      task_course_yaw_=route.yaw;
      mission_topology_connected_=route.connected;
      mission_direction_source_=route.connected ?
        (route.topology_progress ? "LANE_TOPOLOGY":"DESTINATION_FLOW_FALLBACK") : "UNRESOLVED_TOPOLOGY";
      mission_corridor_lane_ids_=route.corridor_lane_ids;
      task_anchor_={own.x,own.y};
      const double initial_course=task_origin_course_+tf2::getYaw(origin_in_chart.pose.orientation)-
        tf2::getYaw(task_origin_->pose.orientation);
      if (cnm::isReverseTask(
          initial_course, own.x, own.y, goal.x, goal.y, reverse_enter_angle_,
          minimum_reverse_goal_distance_))
      {
        reverse_intent_yaw_ = bearing;
      }
      RCLCPP_INFO(get_logger(),
        "Latched mission lane direction %.1f deg (goal range %.1f m)%s",
        *task_course_yaw_ * 180.0 / M_PI, distance,
        reverse_intent_yaw_ ? "; reverse-direction task" : "");
    }

    policy_yaw_ = goal_ ? task_course_yaw_ : course_yaw_;
    return policy_changed();
  }

  // [功能与联系] 检查静态几何、地图及策略方向，按角度阈值节流分类；传播主航道、绘制语义中心梯度并发布图/可视化/状态。
  void publishIfReady(const bool force)
  {
    if (!policy_yaw_ && course_yaw_) {updatePolicyCourse(false);}
    if (lanes_.empty() || !costmap_ || !policy_yaw_ || !odom_) {return;}
    if (!force && published_policy_yaw_ &&
      std::abs(cnm::normalizeAngle(*policy_yaw_ - *published_policy_yaw_)) < reclassify_angle_)
    {
      return;
    }
    if(!task_course_yaw_ || main_lane_mask_.size()!=lanes_.size()) {
      geometry_msgs::msg::PoseStamped own;own.header=odom_->header;own.header.stamp=rclcpp::Time(0);
      own.pose=odom_->pose.pose;
      try {own=tf_buffer_->transform(own,frame_id_,std::chrono::milliseconds(100));}
      catch(const tf2::TransformException &) {return;}
      const auto anchor=task_course_yaw_ ? task_anchor_ :
        cnm::Point2D{own.pose.position.x,own.pose.position.y};
      std::size_t mission_seed=std::numeric_limits<std::size_t>::max();
      if(task_seed_lane_id_) {
        for(std::size_t i=0;i<lanes_.size();++i) {
          if(lanes_[i].lane_id==*task_seed_lane_id_) {mission_seed=i;break;}
        }
      }
      main_lane_mask_ = cnm::continuousMainLaneMask(
        lanes_, *policy_yaw_, anchor.x, anchor.y, std::min(lane_adjacency_distance_,lane_topology_gap_), lane_continuity_angle_,mission_seed);
      if(task_course_yaw_) {
        for(std::size_t i=0;i<lanes_.size();++i) {
          const auto saved=task_lane_roles_.find(lanes_[i].lane_id);
          if(saved!=task_lane_roles_.end()) {main_lane_mask_[i]=saved->second;}
          else {task_lane_roles_[lanes_[i].lane_id]=main_lane_mask_[i];}
        }
      }
    }
    const bool unresolved=task_course_yaw_ && (!mission_topology_connected_ ||
      std::none_of(main_lane_mask_.begin(),main_lane_mask_.end(),[](bool v){return v;}));
    if(unresolved) {
      main_lane_mask_.assign(lanes_.size(),false);
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "No connected mission topology / matching traffic flow: lane cells marked UNKNOWN");
    }
    std::vector<std::vector<cnm::Point2D>> grid_polygons;
    for(const auto &lane:lanes_) {
      std::vector<cnm::Point2D> polygon;
      for(const auto &point:lane.points) {
        geometry_msgs::msg::PoseStamped p;p.header.frame_id=frame_id_;
        p.pose.position.x=point.x;p.pose.position.y=point.y;
        try {p=tf_buffer_->transform(p,costmap_->header.frame_id,std::chrono::milliseconds(100));}
        catch(const tf2::TransformException &) {return;}
        const double angle=tf2::getYaw(costmap_->info.origin.orientation);
        const double dx=p.pose.position.x-costmap_->info.origin.position.x;
        const double dy=p.pose.position.y-costmap_->info.origin.position.y;
        polygon.push_back({std::cos(angle)*dx+std::sin(angle)*dy,-std::sin(angle)*dx+std::cos(angle)*dy});
      }
      grid_polygons.push_back(std::move(polygon));
    }
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
      const int value = unresolved ? -1:(main_lane_mask_[index] ? cnm::kMainLane : opposite_value_);
      cnm::fillPolygon(
        lane_map.data, static_cast<int>(lane_map.info.width),
        static_cast<int>(lane_map.info.height), lane_map.info.resolution,
        0.0, 0.0, grid_polygons[index], value);
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
      std::vector<int8_t> local(lane_map.data.size(), static_cast<int8_t>(outside_value_));
      cnm::fillPolygon(
        local, static_cast<int>(lane_map.info.width), static_cast<int>(lane_map.info.height),
        lane_map.info.resolution, 0.0, 0.0, grid_polygons[index], cnm::kMainLane);
      cnm::applyMainLaneCenterGradient(
        local, static_cast<int>(lane_map.info.width), static_cast<int>(lane_map.info.height),
        lane_map.info.resolution, main_lane_edge_value_, center_clearance_);
      for (std::size_t i = 0; i < lane_map.data.size(); ++i) {
        if (lane_map.data[i] <= main_lane_edge_value_ && local[i] <= main_lane_edge_value_) {
          lane_map.data[i] = std::min(lane_map.data[i], local[i]);
        }
      }
    }
    published_policy_yaw_ = policy_yaw_;
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
      geometry_msgs::msg::PoseStamped pose;pose.header=odom_->header;pose.header.stamp=rclcpp::Time(0);
      pose.pose=odom_->pose.pose;
      try {pose=tf_buffer_->transform(pose,frame_id_,std::chrono::milliseconds(20));}
      catch(const tf2::TransformException &) {return;}
      const double x = pose.pose.position.x;
      const double y = pose.pose.position.y;
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
    const bool unresolved=task_course_yaw_ && (!mission_topology_connected_ ||
      std::none_of(main_lane_mask_.begin(),main_lane_mask_.end(),[](bool v){return v;}));
    if(unresolved) region="UNRESOLVED";
    nlohmann::json status = {
      {"course_deg_ros", std::round(*course_yaw_ * 18000.0 / M_PI) / 100.0},
      {"course_frame", odom_ ? odom_->header.frame_id : ""},
      {"policy_course_deg_ros", std::round(*policy_yaw_ * 18000.0 / M_PI) / 100.0},
      {"direction_source", task_course_yaw_ ? mission_direction_source_ : "COG_FALLBACK"},
      {"task_course_deg_ros", task_course_yaw_ ?
        nlohmann::json(std::round(*task_course_yaw_ * 18000.0 / M_PI) / 100.0) :
        nlohmann::json(nullptr)},
      {"neutral_corridor_lane_ids",mission_corridor_lane_ids_},
      {"classification_valid",!unresolved},
      {"reverse_intent", reverse_intent_yaw_.has_value()},
      {"policy_frame", frame_id_},
      {"task_origin", task_course_yaw_ ? nlohmann::json::array({task_anchor_.x,task_anchor_.y}) : nlohmann::json(nullptr)},
      {"current_region", region}, {"lane_id", lane_id},
      {"priority", {"SAFETY", "COLREGS", "RIGHT_HAND", "EFFICIENCY"}}
    };
    std_msgs::msg::String message;
    message.data = status.dump();
    status_pub_->publish(message);
  }

  std::string lane_data_topic_, costmap_topic_, odom_topic_, goal_topic_;
  std::string lane_costmap_topic_, marker_topic_, status_topic_, frame_id_{"map"};
  std::optional<int> task_seed_lane_id_;
  double lane_topology_gap_{5.0};
  bool mission_topology_connected_{false};
  std::string mission_direction_source_;
  std::vector<int> mission_corridor_lane_ids_;
  int outside_value_{45}, opposite_value_{75}, main_lane_edge_value_{20};
  double center_clearance_{16.0}, minimum_course_speed_{0.1}, reclassify_angle_{0.2};
  double lane_adjacency_distance_{160.0}, lane_continuity_angle_{1.3962634};
  double reverse_enter_angle_{1.92};
  double minimum_reverse_goal_distance_{10.0};
  std::vector<cnm::Lane> lanes_;
  std::vector<bool> main_lane_mask_;
  std::unordered_map<int,bool> task_lane_roles_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  std::optional<double> course_yaw_, task_course_yaw_, policy_yaw_;
  std::optional<double> reverse_intent_yaw_, published_policy_yaw_;
  std::optional<geometry_msgs::msg::PoseStamped> task_origin_;
  double task_origin_course_{0.0};
  cnm::Point2D task_anchor_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr lane_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChannelNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
