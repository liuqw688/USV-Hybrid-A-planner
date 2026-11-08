#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <unordered_map>
#include <type_traits>
#include <vector>

#include "usv_mpc_control/tracking_model.hpp"
#include "usv_mpc_control/path_tracking.hpp"
#include "usv_mpc_control/input_frames.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hybrid_a_star_planner/msg/encounter_array.hpp"
#include "hybrid_a_star_planner/policy.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "visualization_msgs/msg/marker.hpp"

namespace
{
using VesselControl = usv_mpc_control::TrackingControl;
using VesselModelParameters = usv_mpc_control::TrackingModelParameters;
using usv_mpc_control::VesselState;
using Point = geometry_msgs::msg::Point;

struct PathSegment
{
  double ax, ay, bx, by, yaw, s, length;
};

struct Projection
{
  std::size_t segment{0};
  double x{0.0}, y{0.0}, yaw{0.0}, s{0.0}, distance{1.0e9};
};

class MpcController : public rclcpp::Node
{
public:
  MpcController()
  : Node("mpc_controller")
  {
    const auto path_topic = declare_parameter<std::string>("path_topic", "/hybrid_a_star_trajectory");
    const auto map_topic = declare_parameter<std::string>("costmap_topic", "/local_costmap");
    const auto lane_map_topic = declare_parameter<std::string>("lane_costmap_topic", "/channel/lane_costmap");
    const auto land_map_topic = declare_parameter<std::string>("static_land_topic", "/chart_static_obstacles");
    const auto channel_obstacle_topic = declare_parameter<std::string>(
      "channel_obstacle_topic", "/channel/obstacle_costmap");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto effort_topic = declare_parameter<std::string>("effort_topic", "/cmd_wrench");
    const auto model_name=declare_parameter<std::string>("dynamics_model","force");
    if(model_name!="force" && model_name!="identified_velocity_commands")
      throw std::invalid_argument("unknown dynamics_model");
    model_.identified=model_name=="identified_velocity_commands";
    const auto command_topic=declare_parameter<std::string>("command_topic","/cmd_vel");
    const auto policy_topic = declare_parameter<std::string>("policy_topic", "/colregs/policies");
    const auto route_topic = declare_parameter<std::string>("global_route_topic", "/astar_channel_waypoints");
    const auto goal_topic = declare_parameter<std::string>("active_goal_topic", "/goal_pose_from_astar");
    control_frame_ = declare_parameter<std::string>("control_frame", "odom");
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    frequency_ = declare_parameter<double>("control_frequency", 10.0);
    dt_ = declare_parameter<double>("prediction_step", 0.1);
    horizon_ = declare_parameter<double>("prediction_horizon", 5.0);
    knot_count_ = std::clamp(static_cast<int>(declare_parameter("control_knots", 5)), 3, 10);
    optimization_sweeps_ = std::clamp(static_cast<int>(declare_parameter("optimization_sweeps", 2)), 1, 5);
    max_speed_ = declare_parameter<double>("max_speed", 4.0);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.45);
    max_surge_input_ = declare_parameter<double>("max_tau_y", 10.0);
    max_yaw_input_ = declare_parameter<double>("max_tau_n", 5.0);
    path_timeout_ = declare_parameter<double>("path_timeout", 3.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 1.0);
    map_timeout_ = declare_parameter<double>("map_timeout", 0.0);
    cost_threshold_ = declare_parameter<int>("lethal_cost_threshold", 88);
    channel_enabled_ = declare_parameter<bool>("channel_enabled", true);
    shoreline_threshold_ = declare_parameter<int>("shoreline_cost_threshold", 88);
    channel_obstacle_threshold_ = declare_parameter<int>("channel_obstacle_threshold", 85);
    main_lane_max_ = declare_parameter<int>("main_lane_value_max", 20);
    opposite_lane_min_ = declare_parameter<int>("opposite_lane_value_min", 65);
    own_radius_ = declare_parameter<double>("own_ship_radius", 1.5);
    safety_buffer_ = declare_parameter<double>("safety_buffer", 4.0);
    hard_dynamic_distance_ = declare_parameter<double>("dynamic_hard_distance", 2.0);
    stop_distance_ = declare_parameter<double>("emergency_stop_distance", 2.0);
    soft_dynamic_distance_ = declare_parameter<double>("dynamic_soft_distance", 15.0);
    goal_deceleration_ = declare_parameter<double>("goal_deceleration", 0.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.5);
    cross_track_recovery_distance_ = std::max(0.25, declare_parameter<double>("cross_track_recovery_distance", 8.0));
    course_preview_time_=std::max(.1,declare_parameter("course_preview_time",2.0));
    projection_forward_distance_=std::max(2.0,declare_parameter<double>("projection_forward_distance",6.0));
    intermediate_speed_ = declare_parameter<double>("intermediate_goal_speed", 2.0);
    minimum_speed_ = declare_parameter<double>("minimum_tracking_speed", 0.25);
    terminal_heading_fade_speed_ = std::max(0.1, declare_parameter<double>("terminal_heading_fade_speed", 0.35));
    max_yaw_input_rate_ = std::max(0.1, declare_parameter<double>("max_torque_rate", 2.0));
    max_lateral_acceleration_ = std::max(0.05, declare_parameter<double>("max_lateral_acceleration", 0.6));
    curvature_preview_distance_ = std::max(2.0, declare_parameter<double>("curvature_preview_distance", 12.0));
    terminal_hold_radius_ = std::max(goal_tolerance_, declare_parameter<double>("terminal_hold_radius", 0.5));
    surge_response_time_ = declare_parameter<double>("surge_response_time", 0.8);
    yaw_response_time_ = declare_parameter<double>("yaw_response_time", 0.8);
    dt_ = std::clamp(dt_, 0.04, 0.25);
    horizon_ = std::clamp(horizon_, 2.0, 10.0);
    frequency_ = std::clamp(frequency_, 2.0, 25.0);
    steps_ = std::max(10, static_cast<int>(std::ceil(horizon_ / dt_)));
    knot_steps_ = std::max(1, static_cast<int>(std::ceil(
      static_cast<double>(steps_) / knot_count_)));

    model_.k_u = declare_parameter<double>("model.k_u", 0.290681);
    model_.k_t = declare_parameter<double>("model.k_t", 0.295742);
    model_.k_turn = declare_parameter<double>("model.k_turn", 2.126739);
    model_.k_v = declare_parameter<double>("model.k_v", 0.425383);
    model_.k_vr = declare_parameter<double>("model.k_vr", -1.149012);
    model_.k_vn = declare_parameter<double>("model.k_vn", 0.0);
    model_.k_r = declare_parameter<double>("model.k_r", 1.342561);
    model_.k_r3 = declare_parameter<double>("model.k_r3", 0.0);
    model_.k_n = declare_parameter<double>("model.k_n", 0.781688);
    min_surge_input_=-max_surge_input_;
    if(model_.identified) {
      min_surge_input_=declare_parameter<double>("identified.min_v_cmd",-.4);
      max_surge_input_=declare_parameter<double>("identified.max_v_cmd",1.5);
      max_yaw_input_=declare_parameter<double>("identified.max_w_cmd",.7);
      max_yaw_input_rate_=declare_parameter<double>("identified.max_w_cmd_rate",.5);
      if(!std::isfinite(min_surge_input_) || !std::isfinite(max_surge_input_) ||
        !std::isfinite(max_yaw_input_) || !std::isfinite(max_yaw_input_rate_) ||
        min_surge_input_>=0 || max_surge_input_<=0 || max_yaw_input_<=0 || max_yaw_input_rate_<=0)
        throw std::invalid_argument("invalid identified command limits");
    }

    cross_track_weight_ = declare_parameter<double>("weight.cross_track", 12.0);
    course_weight_ = declare_parameter<double>("weight.course", 12.0);
    progress_weight_ = declare_parameter<double>("weight.progress", 1.4);
    speed_weight_ = declare_parameter<double>("weight.speed", 2.0);
    sideslip_weight_ = declare_parameter<double>("weight.sideslip", 3.0);
    yaw_rate_weight_ = declare_parameter<double>("weight.yaw_rate", 2.0);
    effort_weight_ = declare_parameter<double>("weight.effort", 0.01);
    torque_effort_weight_ = declare_parameter<double>("weight.torque_effort", 0.008);
    slew_weight_ = declare_parameter<double>("weight.torque_slew", 0.6);
    reverse_turn_weight_ = declare_parameter<double>("weight.reverse_turn", 2.5);
    terminal_weight_ = declare_parameter<double>("weight.terminal", 4.0);
    terminal_heading_weight_=std::max(0.,declare_parameter("weight.terminal_heading",50.0));
    dynamic_clearance_weight_ = declare_parameter<double>("weight.dynamic_clearance", 12.0);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(path_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::Path::SharedPtr m) {raw_path_=std::move(m);path_receipt_=now();});
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {raw_map_ = std::move(m); map_receipt_ = now();});
    lane_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(lane_map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {raw_lane_map_ = std::move(m);});
    land_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(land_map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {raw_land_map_ = std::move(m);});
    channel_obstacle_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(channel_obstacle_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {raw_channel_obstacle_map_ = std::move(m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 20,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {raw_odom_ = std::move(m);});
    policy_sub_ = create_subscription<hybrid_a_star_planner::msg::EncounterArray>(policy_topic, 10,
      [this](hybrid_a_star_planner::msg::EncounterArray::SharedPtr m) {
        policies_ = std::move(m); policy_receipt_ = now();
      });
    // Empty route topic selects a single final goal, used by open-water demos.
    // River navigation supplies the global route to distinguish waypoint handoff.
    if (!route_topic.empty()) {
      route_sub_ = create_subscription<nav_msgs::msg::Path>(route_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        [this](nav_msgs::msg::Path::SharedPtr m) {raw_route_ = std::move(m);});
    }
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {raw_active_goal_ = std::move(m);});
    planning_status_sub_=create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("planning_status_topic","/hybrid_a_star/planning_status"),
      rclcpp::QoS(1).transient_local().reliable(),[this](std_msgs::msg::String::SharedPtr m) {
        planning_status_=m->data;planning_status_receipt_=now();
      });
    require_speed_profile_=declare_parameter("require_speed_profile",false);
    speed_limit_sub_=create_subscription<geometry_msgs::msg::TwistStamped>(
      declare_parameter<std::string>("speed_limit_topic","/hybrid_a_star/speed_limit"),
      rclcpp::QoS(1).transient_local().reliable(),[this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
        speed_profile_=std::move(m);
      });
    if(model_.identified) command_pub_=create_publisher<geometry_msgs::msg::Twist>(command_topic,10);
    else effort_pub_ = create_publisher<geometry_msgs::msg::Wrench>(effort_topic, 10);
    predicted_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/mpc/predicted_trajectory", 1);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
      std::bind(&MpcController::controlStep, this));
    warm_.assign(static_cast<std::size_t>(knot_count_), VesselControl{});
    RCLCPP_INFO(get_logger(), "MPC tracker ready: %d knots, %.1f s horizon, %.0f Hz, model=%s",
      knot_count_, horizon_, frequency_,model_name.c_str());
  }

private:
  // 将世界坐标输入转换到控制坐标系；静态地图按原消息和TF缓存，避免每周期复制栅格。
  struct FrameCache {
    std::shared_ptr<const void> source;
    geometry_msgs::msg::Transform transform;
  };
  template<class T>
  void normalizeInput(const std::shared_ptr<T> &raw, std::shared_ptr<T> &out,
    const std::string &role)
  {
    if (!raw) {out.reset(); return;}
    try {
      if (raw->header.frame_id.empty()) {throw std::runtime_error("empty frame_id");}
      if constexpr (std::is_same<T,nav_msgs::msg::Path>::value) {
        for (const auto &pose:raw->poses) {
          if (!pose.header.frame_id.empty() && pose.header.frame_id!=raw->header.frame_id) {
            throw std::runtime_error("mixed-frame Path");
          }
        }
      }
      if (raw->header.frame_id == control_frame_) {out=raw; return;}
      const auto tf=tf_buffer_->lookupTransform(control_frame_,raw->header.frame_id,tf2::TimePointZero);
      auto &cache=frame_cache_[role];
      if (out && cache.source.get()==raw.get() && cache.transform==tf.transform) {return;}
      out=std::make_shared<T>(usv_mpc_control::transformWorldMessage(*raw,tf));
      cache.source=raw;cache.transform=tf.transform;
    } catch (const std::exception &e) {
      out.reset();
      input_frame_error_ += role+" ("+raw->header.frame_id+" -> "+control_frame_+"): "+e.what()+"; ";
    }
  }
  void normalizeInputs()
  {
    input_frame_error_.clear();
    normalizeInput(raw_map_,map_,"costmap");
    normalizeInput(raw_lane_map_,lane_map_,"lane map");
    normalizeInput(raw_land_map_,land_map_,"land map");
    normalizeInput(raw_channel_obstacle_map_,channel_obstacle_map_,"channel obstacles");
    normalizeInput(raw_odom_,odom_,"odometry");
    normalizeInput(raw_route_,route_,"global route");
    normalizeInput(raw_active_goal_,active_goal_,"active goal");
    normalizeInput(raw_path_,normalized_path_,"path");
    if (normalized_path_ != path_) {
      if (normalized_path_) {onPath(normalized_path_);}
      else {path_.reset();segments_.clear();warm_.clear();}
    }
    updateFinalGoal();
  }
  static double clamp(double value, double low, double high)
  {
    return std::max(low, std::min(high, value));
  }

  void onPath(nav_msgs::msg::Path::SharedPtr message)
  {
    // 时间戳匹配限速与轨迹，防止跨话题乱序把上一条4m/s限速用于新的低速掉头。
    // 仅刷新时间戳的同形路径保留已匹配限速，不因心跳到达顺序短暂停船。
    std::size_t identity=std::hash<std::string>{}(message->header.frame_id);
    auto combine=[&identity](double value) {identity^=std::hash<double>{}(value)+
      0x9e3779b9+(identity<<6)+(identity>>2);};
    for(const auto &p:message->poses) {combine(p.pose.position.x);combine(p.pose.position.y);
      combine(p.pose.orientation.z);combine(p.pose.orientation.w);}
    if(identity!=path_identity_) {speed_profile_ready_=false;path_identity_=identity;}
    const bool changed = !path_ || path_->poses.size() != message->poses.size() ||
      (path_ && !path_->poses.empty() && !message->poses.empty() &&
       (std::hypot(path_->poses.back().pose.position.x - message->poses.back().pose.position.x,
                   path_->poses.back().pose.position.y - message->poses.back().pose.position.y) > 0.5 ||
        std::hypot(path_->poses.front().pose.position.x - message->poses.front().pose.position.x,
                   path_->poses.front().pose.position.y - message->poses.front().pose.position.y) > 0.5));
    path_ = std::move(message);
    if (changed) {
      segments_.clear();
      warm_.clear();
      terminal_hold_ = false;
      warm_elapsed_ = 0.0;
      projection_hint_ = 0;
    }
    if (path_ && path_->poses.size() >= 2) {buildSegments();}
    updateFinalGoal();
  }

  void buildSegments()
  {
    segments_.clear();segment_body_curvature_.clear();
    segments_.reserve(path_->poses.size() - 1U);
    double s = 0.0;
    for (std::size_t i = 0; i + 1U < path_->poses.size(); ++i) {
      const auto &a = path_->poses[i].pose.position;
      const auto &b = path_->poses[i + 1U].pose.position;
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      if (length < 1.0e-5) {continue;}
      segments_.push_back({a.x, a.y, b.x, b.y, std::atan2(b.y - a.y, b.x - a.x), s, length});
      segment_body_curvature_.push_back(hybrid_a_star_planner::normalizeAngle(
        tf2::getYaw(path_->poses[i+1].pose.orientation)-tf2::getYaw(path_->poses[i].pose.orientation))/length);
      s += length;
    }
    total_length_ = s;
    bend_speed_limits_.assign(segments_.size(), max_speed_);
    for (std::size_t first=0; first<segments_.size(); ++first) {
      double curvature=0.0;
      for (std::size_t i=std::max<std::size_t>(1,first); i<segments_.size() &&
        segments_[i].s <= segments_[first].s + curvature_preview_distance_; ++i) {
        const double angle=std::abs(hybrid_a_star_planner::normalizeAngle(segments_[i].yaw-segments_[i-1].yaw));
        curvature=std::max(curvature,angle/std::max(0.1,0.5*(segments_[i].length+segments_[i-1].length)));
      }
      bend_speed_limits_[first]=std::min(max_speed_,std::max(intermediate_speed_,
        std::sqrt(max_lateral_acceleration_/std::max(1.0e-6,curvature))));
    }
  }

  void updateFinalGoal()
  {
    final_goal_ = false;
    if (!active_goal_ || !path_ || path_->poses.size() < 2U ||
      path_->header.frame_id != active_goal_->header.frame_id) {return;}
    const auto &active = active_goal_->pose.position;
    const auto &local_end = path_->poses.back().pose.position;
    if (!route_sub_) {
      // Direct-goal mode has no intermediate waypoint; its active goal is final.
      final_goal_ = usv_mpc_control::isTerminalMissionPath(
        active.x, active.y, active.x, active.y, local_end.x, local_end.y, 0.5, 1.05);
      return;
    }
    if (!route_ || route_->poses.empty() ||
      route_->header.frame_id != path_->header.frame_id) {return;}
    const auto &mission_end = route_->poses.back().pose.position;
    final_goal_ = usv_mpc_control::isTerminalMissionPath(
      active.x, active.y, mission_end.x, mission_end.y, local_end.x, local_end.y, 0.5, 1.05);
  }

  Projection project(double x, double y, std::size_t hint, int back, int forward) const
  {
    Projection best;
    if (segments_.empty()) {return best;}
    const std::size_t begin = hint > static_cast<std::size_t>(back) ? hint - back : 0U;
    const std::size_t end = std::min(segments_.size(), hint + static_cast<std::size_t>(forward) + 1U);
    for (std::size_t i = begin; i < end; ++i) {
      const auto &p = segments_[i];
      const double dx = p.bx - p.ax, dy = p.by - p.ay;
      const double raw_t = ((x - p.ax) * dx + (y - p.ay) * dy) /
        std::max(1.0e-9, p.length * p.length);
      // 中间段在预测窗口内沿末端切线连续延续，不能把“尚未发布的下一段”
      // 当成停车墙。地图/动态硬检查继续覆盖延续段；最终任务点不延续。
      const double t = !final_goal_ && i + 1U == segments_.size() ?
        std::max(0.0,raw_t) : clamp(raw_t,0.0,1.0);
      const double qx = p.ax + t * dx, qy = p.ay + t * dy;
      const double d = std::hypot(x - qx, y - qy);
      if (d < best.distance) {
        best = {i, qx, qy, p.yaw, p.s + t * p.length, d};
      }
    }
    return best;
  }

  bool worldToCell(const nav_msgs::msg::OccupancyGrid &grid,
    double x, double y, int &gx, int &gy) const
  {
    if (grid.info.resolution <= 0.0) {return false;}
    const double yaw = tf2::getYaw(grid.info.origin.orientation);
    const double dx = x - grid.info.origin.position.x;
    const double dy = y - grid.info.origin.position.y;
    const double lx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ly = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    gx = static_cast<int>(std::floor(lx / grid.info.resolution));
    gy = static_cast<int>(std::floor(ly / grid.info.resolution));
    return gx >= 0 && gy >= 0 && gx < static_cast<int>(grid.info.width) &&
      gy < static_cast<int>(grid.info.height);
  }

  bool cellValue(const nav_msgs::msg::OccupancyGrid &grid,
    double x, double y, int &value) const
  {
    int gx = 0, gy = 0;
    if (!worldToCell(grid, x, y, gx, gy)) {return false;}
    const std::size_t index = static_cast<std::size_t>(gy) * grid.info.width + gx;
    if (index >= grid.data.size()) {return false;}
    value = grid.data[index];
    return true;
  }

  bool mapSafe(const VesselState &s) const
  {
    if (!map_ || map_->header.frame_id != control_frame_) {return false;}
    const double radius = std::max(0.0, own_radius_);
    constexpr double c = 0.7071067811865476;
    const double offsets[9][2] = {{0,0},{1,0},{-1,0},{0,1},{0,-1},
      {c,c},{c,-c},{-c,c},{-c,-c}};
    for (const auto &o : offsets) {
      const double x = s.x + radius * o[0], y = s.y + radius * o[1];
      int map_value = -1;
      if (!cellValue(*map_, x, y, map_value)) {return false;}
      // Lane semantics must never clear UNKNOWN or the hard clearance band.
      if (map_value < 0 || map_value >= 99) {return false;}
      int land_value = -1, obstacle_value = -1, lane_value = -1;
      const bool has_land = land_map_ && land_map_->header.frame_id == control_frame_ &&
        cellValue(*land_map_, x, y, land_value);
      const bool has_obstacle = channel_obstacle_map_ &&
        channel_obstacle_map_->header.frame_id == control_frame_ &&
        cellValue(*channel_obstacle_map_, x, y, obstacle_value);
      const bool has_lane = lane_map_ && lane_map_->header.frame_id == control_frame_ &&
        cellValue(*lane_map_, x, y, lane_value);
      // Match Hybrid A*: original land and independent obstacle inflation remain
      // lethal everywhere; only shoreline inflation is ignored inside a lane.
      // The obstacle grid is optional until a sensor/source node publishes it.
      if ((has_land && land_value >= 100) ||
        (has_obstacle && obstacle_value >= channel_obstacle_threshold_)) {return false;}
      const bool in_channel = channel_enabled_ && has_lane && lane_value >= 0 &&
        (lane_value <= main_lane_max_ || lane_value >= opposite_lane_min_);
      if (in_channel) {continue;}
      if (map_value < 0 || map_value >= (channel_enabled_ ? shoreline_threshold_ : cost_threshold_)) {
        return false;
      }
    }
    return true;
  }

  bool dynamicSafe(const VesselState &s, double future, double &soft_cost) const
  {
    soft_cost = 0.0;
    if (!policies_ || !policies_->valid ||
      (now() - policy_receipt_).seconds() > 1.0) {return true;}
    for (const auto &e : policies_->encounters) {
      const double age = std::max(0.0, (now() - rclcpp::Time(policies_->header.stamp)).seconds());
      const double tx = e.x + e.vx * (age + future);
      const double ty = e.y + e.vy * (age + future);
      const double distance = std::hypot(s.x - tx, s.y - ty);
      const double hard = std::max(hard_dynamic_distance_, own_radius_ + safety_buffer_ + e.radius);
      if (distance < hard) {return false;}
      hybrid_a_star_planner::Policy policy;
      policy.type = static_cast<hybrid_a_star_planner::EncounterType>(e.type);
      policy.level = static_cast<hybrid_a_star_planner::Risk>(e.risk);
      policy.locked = e.locked;
      policy.takeover = e.takeover;
      policy.recovering = e.recovering;
      policy.reference = e.reference;
      policy.speed = e.speed;
      if (!hybrid_a_star_planner::headingAllowedDynamic(policy, s.yaw, future,
          tf2::getYaw(odom_->pose.pose.orientation), max_yaw_rate_,
          1.2 * yaw_response_time_)) {return false;}
      if (distance < soft_dynamic_distance_ && soft_dynamic_distance_ > hard) {
        const double ratio = (soft_dynamic_distance_ - distance) /
          (soft_dynamic_distance_ - hard);
        soft_cost += dynamic_clearance_weight_ * ratio * ratio;
      }
    }
    return true;
  }

  // 中间经过点不停车，但弯道按v²*k限制横向加速度；最终点再叠加停车包络。
  double referenceSpeed(double arc) const
  {
    const auto upper=std::upper_bound(segments_.begin(),segments_.end(),arc,
      [](double value,const PathSegment &segment){return value<segment.s;});
    const std::size_t index=upper==segments_.begin() ? 0U :
      static_cast<std::size_t>(upper-segments_.begin()-1);
    const double cruise=std::min(path_speed_limit_,bend_speed_limits_.empty() ? max_speed_ : bend_speed_limits_[index]);
    return final_goal_ ? std::min(cruise, usv_mpc_control::terminalReferenceSpeed(
      total_length_ - arc, goal_tolerance_, goal_deceleration_, max_speed_)) : cruise;
  }

  void buildProgressReference(double start)
  {
    double arc = start;
    progress_reference_.clear();
    for (int i=0; i<steps_; ++i) {
      arc += referenceSpeed(arc)*dt_;
      if (final_goal_) {arc = std::min(arc,std::max(0.0,total_length_-goal_tolerance_));}
      progress_reference_.push_back(arc);
    }
  }

  std::vector<VesselControl> seedControls(const VesselState &initial, const Projection &projection)
  {
    std::vector<VesselControl> controls(static_cast<std::size_t>(knot_count_));
    const double desired_speed = referenceSpeed(projection.s);
    const double beta = usv_mpc_control::regularizedSideslip(initial.u, initial.v, 0.05);
    std::size_t target_index = projection.segment;
    while (target_index + 1U < segments_.size() &&
      segments_[target_index].s + segments_[target_index].length < projection.s + 8.0) {
      ++target_index;
    }
    const double lateral_error = usv_mpc_control::signedCrossTrackError(
      projection.x, projection.y, projection.yaw, initial.x, initial.y);
    const double target_course = usv_mpc_control::correctedTrackCourse(
      segments_[target_index].yaw, lateral_error, cross_track_recovery_distance_);
    const double heading_error = hybrid_a_star_planner::normalizeAngle(target_course - beta - initial.yaw);
    const double fade = final_goal_ ? desired_speed * desired_speed /
      (desired_speed * desired_speed + terminal_heading_fade_speed_ * terminal_heading_fade_speed_) : 1.0;
    const double target_r = clamp(0.9 * fade * heading_error, -max_yaw_rate_, max_yaw_rate_);
    const double target_u = final_goal_ ? desired_speed :
      std::min(path_speed_limit_,std::max(intermediate_speed_, desired_speed));
    VesselState s = initial;
    const auto seed = usv_mpc_control::feedbackControl(s, target_u, target_r,
      model_, surge_response_time_, yaw_response_time_, max_surge_input_, max_yaw_input_, min_surge_input_);
    double previous = last_yaw_input_;
    for (std::size_t k = 0; k < controls.size(); ++k) {
      auto &c = controls[k];
      c.surge = seed.surge;
      const double change = max_yaw_input_rate_ * (k == 0 ? 1.0 / frequency_ : knot_steps_ * dt_);
      c.yaw = clamp(seed.yaw, previous - change, previous + change);
      previous = c.yaw;
    }
    return controls;
  }

  // Reference body-yaw feedforward is an additional MPC seed, not a direct
  // command. It preserves the old seed/warm-start and passes the same rollout
  // map, dynamic traffic, force and torque-slew acceptance tests.
  std::vector<VesselControl> referenceControls(const VesselState &initial,const Projection &projection)
  {
    std::vector<VesselControl> controls(static_cast<std::size_t>(knot_count_));
    auto state=initial;auto point=projection;double previous=last_yaw_input_;
    for(std::size_t k=0;k<controls.size();++k) {
      const double speed=referenceSpeed(point.s);
      const auto index=std::min(point.segment,segments_.size()-1);
      const double beta=usv_mpc_control::regularizedSideslip(state.u,state.v,.05);
      const double error=usv_mpc_control::signedCrossTrackError(point.x,point.y,point.yaw,state.x,state.y);
      std::size_t ahead=index;
      const double preview=usv_mpc_control::speedAdaptivePreview(speed,course_preview_time_);
      while(ahead+1<segments_.size() && segments_[ahead].s+segments_[ahead].length<point.s+preview)++ahead;
      const double course=usv_mpc_control::correctedTrackCourse(point.yaw,error,cross_track_recovery_distance_);
      const double fade=final_goal_?speed*speed/(speed*speed+terminal_heading_fade_speed_*terminal_heading_fade_speed_):1.;
      const double feedforward=index<segment_body_curvature_.size()?.5*(segment_body_curvature_[index]+segment_body_curvature_[ahead])*std::hypot(state.u,state.v):0;
      const double rate=clamp(feedforward+.5*fade*hybrid_a_star_planner::normalizeAngle(course-state.yaw-beta),-max_yaw_rate_,max_yaw_rate_);
      auto c=usv_mpc_control::feedbackControl(state,speed,rate,model_,surge_response_time_,yaw_response_time_,max_surge_input_,max_yaw_input_,min_surge_input_);
      const double change=max_yaw_input_rate_*(k==0?1./frequency_:knot_steps_*dt_);
      c.yaw=clamp(c.yaw,previous-change,previous+change);controls[k]=c;previous=c.yaw;
      for(int i=0;i<knot_steps_;++i) {
        state=usv_mpc_control::predict(state,c,model_,dt_);
        if(!model_.identified){state.u=clamp(state.u,0.,max_speed_);state.r=clamp(state.r,-max_yaw_rate_,max_yaw_rate_);}
        point=project(state.x,state.y,point.segment,4,16);
      }
    }
    return controls;
  }

  double rollout(
    const VesselState &initial, const Projection &start_projection,
    const std::vector<VesselControl> &controls, std::vector<VesselState> *trace) const
  {
    VesselState s = initial;
    std::size_t hint = start_projection.segment;
    double cost = 0.0;
    double previous_tau_n = last_yaw_input_;
    if (trace) {trace->clear(); trace->push_back(s);}
    for (int i = 0; i < steps_; ++i) {
      const std::size_t knot = std::min(controls.size() - 1U,
        static_cast<std::size_t>(i / knot_steps_));
      const auto control = controls[knot];
      const double allowed_change = max_yaw_input_rate_ * (i == 0 ? 1.0 / frequency_ : knot_steps_ * dt_);
      if ((i == 0 || i % knot_steps_ == 0) &&
        std::abs(control.yaw - previous_tau_n) > allowed_change + 1.0e-8) {
        return std::numeric_limits<double>::infinity();
      }
      s = usv_mpc_control::predict(s, control, model_, dt_);
      if(model_.identified) {
        if(std::abs(s.u)>max_speed_ || std::abs(s.r)>max_yaw_rate_ || std::abs(s.v)>.8)
          return std::numeric_limits<double>::infinity();
      } else {
        s.u = clamp(s.u, 0.0, max_speed_);
        s.r = clamp(s.r, -max_yaw_rate_, max_yaw_rate_);
      }
      const double elapsed = (i + 1) * dt_;
      const auto p = project(s.x, s.y, hint, 4, 16);
      hint = p.segment;
      if (p.distance > 1.0e8 || !mapSafe(s)) {return std::numeric_limits<double>::infinity();}
      double dynamic_cost = 0.0;
      if (!dynamicSafe(s, elapsed, dynamic_cost)) {return std::numeric_limits<double>::infinity();}
      const double beta = usv_mpc_control::regularizedSideslip(s.u, s.v, 0.05);
      const double lateral_error = usv_mpc_control::signedCrossTrackError(
        p.x, p.y, p.yaw, s.x, s.y);
      const double desired_course = usv_mpc_control::correctedTrackCourse(
        p.yaw, lateral_error, cross_track_recovery_distance_);
      const double course_error = hybrid_a_star_planner::normalizeAngle(
        desired_course - (s.yaw + beta));
      const double desired_progress = progress_reference_[i];
      const double lag = std::max(0.0, desired_progress - p.s);
      const double desired_speed = referenceSpeed(p.s);
      const double speed_error = (final_goal_ ? desired_speed : std::max(minimum_speed_, desired_speed)) - s.u;
      const double heading_fade = final_goal_ ? desired_speed * desired_speed /
        (desired_speed * desired_speed + terminal_heading_fade_speed_ * terminal_heading_fade_speed_) : 1.0;
      const double yaw_switch = control.yaw * previous_tau_n < -0.04 ? reverse_turn_weight_ : 0.0;
      const double torque_delta = control.yaw - previous_tau_n;
      cost += dt_ * (cross_track_weight_ * p.distance * p.distance +
        course_weight_ * heading_fade * course_error * course_error + progress_weight_ * usv_mpc_control::slowProgressScale(path_speed_limit_) * lag * lag +
        speed_weight_ * speed_error * speed_error + sideslip_weight_ * s.v * s.v +
        yaw_rate_weight_ * s.r * s.r + dynamic_cost + yaw_switch) +
        dt_ * (effort_weight_ * control.surge * control.surge +
          torque_effort_weight_ * control.yaw * control.yaw +
          slew_weight_ * torque_delta * torque_delta);
      // The path tangent is a ground course; the requested final HULL yaw
      // is a separate objective and is not implied by course tracking at stop.
      if(final_goal_ && active_goal_ && total_length_-p.s<4.0) {
        const double yaw_error=hybrid_a_star_planner::normalizeAngle(s.yaw-tf2::getYaw(active_goal_->pose.orientation));
        const double excess=std::max(0.,std::abs(yaw_error)-24.0*M_PI/180.0);
        cost+=dt_*terminal_heading_weight_*excess*excess;
      }
      previous_tau_n = control.yaw;
      if (trace) {trace->push_back(s);}
    }
    const auto terminal = project(s.x, s.y, hint, 6, 20);
    const double terminal_error = std::max(0.0, progress_reference_.back() - terminal.s);
    cost += terminal_weight_ * (cross_track_weight_ * terminal.distance * terminal.distance +
      progress_weight_ * usv_mpc_control::slowProgressScale(path_speed_limit_) * terminal_error * terminal_error);
    return cost;
  }

  void controlStep()
  {
    normalizeInputs();
    if(speed_profile_ && raw_path_ && speed_profile_->header.stamp==raw_path_->header.stamp &&
      speed_profile_->header.frame_id==raw_path_->header.frame_id &&
      std::isfinite(speed_profile_->twist.linear.x) && speed_profile_->twist.linear.x>=0.5) {
      path_speed_limit_=std::min(max_speed_,speed_profile_->twist.linear.x);speed_profile_ready_=true;
    }
    if (!input_frame_error_.empty() || !path_ || path_->poses.size() < 2U || !odom_ || !map_ || segments_.empty() ||
      path_->header.frame_id != control_frame_ || map_->header.frame_id != control_frame_ ||
      odom_->header.frame_id != control_frame_ ||
      (channel_enabled_ && (!lane_map_ || !land_map_))) {
      std::ostringstream reason;
      if (!input_frame_error_.empty()) {reason << "TF unavailable/invalid: " << input_frame_error_;}
      if (!path_) {reason << "waiting for path; ";}
      else if (segments_.empty()) {reason << "planner withdrew path (" << planning_status_ << "); ";}
      if (!odom_) {reason << "missing odometry; ";}
      if (!map_) {reason << "missing costmap; ";}
      if (channel_enabled_ && !lane_map_) {reason << "missing lane map; ";}
      if (channel_enabled_ && !land_map_) {reason << "missing land map; ";}
      const bool expected = input_frame_error_.empty() &&
        odom_ && map_ && (!channel_enabled_ || (lane_map_ && land_map_)) &&
        (now()-planning_status_receipt_).seconds()<3.0 &&
        (planning_status_=="planning" || planning_status_=="cancelled" || planning_status_=="idle");
      publishBrake(reason.str(), !expected);
      return;
    }
    if(require_speed_profile_ && !speed_profile_ready_) {
      publishBrake("waiting for speed profile matching new path",false);return;
    }
    const double age = (now() - path_receipt_).seconds();
    const double odom_age = (now() - rclcpp::Time(odom_->header.stamp)).seconds();
    const double map_age = (now() - map_receipt_).seconds();
    if (age > path_timeout_ || odom_age > odom_timeout_ || odom_age < -0.1 ||
      (map_timeout_ > 0.0 && map_age > map_timeout_)) {
      // 保留失联制动，明确打印过期来源，不把一次发布的静态地图误认为故障。
      std::ostringstream reason;
      reason << "stale input:";
      if (age > path_timeout_) {reason << " path age=" << age << "s limit=" << path_timeout_ << "s;";}
      if (odom_age > odom_timeout_ || odom_age < -0.1) {
        reason << " odometry age=" << odom_age << "s limit=" << odom_timeout_ << "s;";
      }
      if (map_timeout_ > 0.0 && map_age > map_timeout_) {
        reason << " map age=" << map_age << "s limit=" << map_timeout_ << "s;";
      }
      publishBrake(reason.str());
      return;
    }
    VesselState initial;
    initial.x = odom_->pose.pose.position.x;
    initial.y = odom_->pose.pose.position.y;
    initial.yaw = tf2::getYaw(odom_->pose.pose.orientation);
    initial.u = odom_->twist.twist.linear.x;
    initial.v = odom_->twist.twist.linear.y;
    initial.r = odom_->twist.twist.angular.z;
    // 掉头路径可能在起点/终点附近交叠；不能一次搜索1000个点跳到未走的末段。
    // 以米而非点数限定沿程窗口，兼容0.05s密集船模轨迹与稀疏普通Path。
    if(segments_.empty()) {publishBrake("path has no traversable segment");return;}
    const auto hint=std::min(projection_hint_,segments_.size()-1U);
    auto begin=hint,end_index=hint;
    while(begin>0 && segments_[hint].s-segments_[begin-1U].s<=2.0) {--begin;}
    const double forward_arc=std::max(projection_forward_distance_,
      2.0*std::hypot(initial.u,initial.v));
    while(end_index+1U<segments_.size() &&
      segments_[end_index+1U].s-segments_[hint].s<=forward_arc) {++end_index;}
    const auto start_projection = project(initial.x, initial.y,hint,
      static_cast<int>(hint-begin),static_cast<int>(end_index-hint));
    projection_hint_ = start_projection.segment;
    if (start_projection.distance > 1.0e8) {publishBrake("path projection failed"); return;}

    if (policies_ && policies_->valid &&
      (now() - policy_receipt_).seconds() < 1.0) {
      for (const auto &e : policies_->encounters) {
        if (hybrid_a_star_planner::insideStopDistance(
            initial.x, initial.y, e.x, e.y, stop_distance_)) {
          publishBrake("emergency dynamic separation");
          return;
        }
      }
    }

    const auto &end = path_->poses.back().pose.position;
    const double end_distance=std::hypot(initial.x-end.x,initial.y-end.y);
    if (!final_goal_ || end_distance > terminal_hold_radius_ + 0.4) {terminal_hold_=false;}
    const double sway_damping=model_.identified ? -usv_mpc_control::identified::av : model_.k_v;
    const double drift_x=initial.x-initial.v*std::sin(initial.yaw)/std::max(0.01,sway_damping);
    const double drift_y=initial.y+initial.v*std::cos(initial.yaw)/std::max(0.01,sway_damping);
    if (final_goal_ && total_length_-start_projection.s<=std::max(1.0,2.0*goal_tolerance_) &&
      end_distance <= terminal_hold_radius_ && std::hypot(initial.u,initial.v)<0.10 &&
      std::hypot(drift_x-end.x,drift_y-end.y)<=terminal_hold_radius_ &&
      (!active_goal_ || std::abs(hybrid_a_star_planner::normalizeAngle(initial.yaw-tf2::getYaw(active_goal_->pose.orientation)))<=M_PI/6)) {
      terminal_hold_=true;
    }
    if (terminal_hold_) {
      // 到达终点后只耗散残余纵速/艏摇，不用偏航力矩“横移”修正厘米级误差。
      // 这是终点停车状态；中间航点及避碰恢复阶段绝不会进入此分支。
      auto control=usv_mpc_control::feedbackControl(initial,0.0,0.0,model_,surge_response_time_,
        yaw_response_time_,max_surge_input_,max_yaw_input_,min_surge_input_);
      control.yaw=clamp(control.yaw,last_yaw_input_-max_yaw_input_rate_/frequency_,
        last_yaw_input_+max_yaw_input_rate_/frequency_);
      last_yaw_input_=control.yaw;
      publishControl(control);
      warm_.clear();
      std::vector<VesselState> trace{initial};
      auto state=initial;
      for(int i=0;i<steps_;++i) {
        state=usv_mpc_control::predict(state,control,model_,dt_);
        if(!model_.identified)state.u=clamp(state.u,0.0,max_speed_);
        trace.push_back(state);
      }
      publishPrediction(trace);
      return;
    }
    buildProgressReference(start_projection.s);
    auto controls = seedControls(initial, start_projection);
    double best = rollout(initial, start_projection, controls, nullptr);
    auto reference_seed=referenceControls(initial,start_projection);
    const double reference_score=rollout(initial,start_projection,reference_seed,nullptr);
    if(reference_score<best){best=reference_score;controls=std::move(reference_seed);}
    // 热启动按实际控制周期移动，而不是每100ms跳过整整1秒的控制结点。
    if (!warm_.empty()) {
      auto shifted = warm_;
      warm_elapsed_ += 1.0 / frequency_;
      if (warm_elapsed_ >= knot_steps_ * dt_) {
        for (std::size_t k = 0; k + 1 < shifted.size(); ++k) {shifted[k] = shifted[k + 1];}
        warm_elapsed_ = 0.0;
      }
      double previous = last_yaw_input_;
      for (std::size_t k = 0; k < shifted.size(); ++k) {
        const double change = max_yaw_input_rate_ * (k == 0 ? 1.0 / frequency_ : knot_steps_ * dt_);
        shifted[k].yaw = clamp(shifted[k].yaw, previous - change, previous + change);
        previous = shifted[k].yaw;
      }
      const double score = rollout(initial, start_projection, shifted, nullptr);
      if (score < best) {best = score; controls = std::move(shifted);}
    }
    double surge_delta = max_surge_input_ * 0.35;
    double yaw_delta = max_yaw_input_ * 0.5;
    for (int sweep = 0; sweep < optimization_sweeps_; ++sweep) {
      for (std::size_t k = 0; k < controls.size(); ++k) {
        for (int axis = 0; axis < 2; ++axis) {
          const double delta = axis == 0 ? surge_delta :
            (k == 0 ? std::min(yaw_delta, max_yaw_input_rate_ / frequency_) : yaw_delta);
          const double original = axis == 0 ? controls[k].surge : controls[k].yaw;
          double local_best = best;
          double selected = original;
          for (const double factor : {-1.0, -0.5, 0.5, 1.0}) {
            const double candidate = clamp(original + factor * delta,
              axis == 0 ? min_surge_input_ : -max_yaw_input_, axis == 0 ? max_surge_input_ : max_yaw_input_);
            if (axis == 0) {controls[k].surge = candidate;} else {controls[k].yaw = candidate;}
            const double score = rollout(initial, start_projection, controls, nullptr);
            if (score + 1.0e-7 < local_best) {local_best = score; selected = candidate;}
          }
          if (axis == 0) {controls[k].surge = selected;} else {controls[k].yaw = selected;}
          best = local_best;
        }
      }
      surge_delta *= 0.65;
      yaw_delta *= 0.65;
    }

    std::vector<VesselState> trace;
    best = rollout(initial, start_projection, controls, &trace);
    if (!std::isfinite(best) || controls.empty()) {
      publishBrake("no collision-free MPC sequence");
      return;
    }
    warm_ = controls;
    const auto command=controls.front();
    last_yaw_input_ = command.yaw;
    publishControl(command);
    publishPrediction(trace);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "MPC tracking: cross-track %.2fm, speed %.2fm/s, surge input %.3f, yaw input %.3f",
      start_projection.distance, initial.u, command.surge, command.yaw);
  }

  void publishBrake(const std::string &reason, bool warning=true)
  {
    VesselControl command;
    if(model_.identified) {
      VesselState state;
      if(odom_) {state.u=odom_->twist.twist.linear.x;state.v=odom_->twist.twist.linear.y;state.r=odom_->twist.twist.angular.z;}
      command=usv_mpc_control::feedbackControl(state,0,0,model_,surge_response_time_,yaw_response_time_,
        max_surge_input_,max_yaw_input_,min_surge_input_);
    } else {
      command.surge = -max_surge_input_;
      if (odom_) command.yaw = clamp(-model_.k_r * odom_->twist.twist.angular.z / std::max(1.0e-6, model_.k_n),
        -max_yaw_input_, max_yaw_input_);
    }
    publishControl(command);
    clearPrediction();
    last_yaw_input_ = command.yaw;
    warm_.clear();
    if (warning) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "MPC braking: %s", reason.c_str());}
    else {RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "MPC waiting safely: %s", reason.c_str());}
  }

  void publishControl(const VesselControl& c) {
    if(model_.identified) {
      geometry_msgs::msg::Twist message;message.linear.x=c.surge;message.angular.z=c.yaw;
      command_pub_->publish(message);
    } else {
      geometry_msgs::msg::Wrench message;message.force.x=c.surge;message.torque.z=c.yaw;
      effort_pub_->publish(message);
    }
  }

  void clearPrediction()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = control_frame_;
    marker.header.stamp = now();
    marker.ns = "mpc_predicted_trajectory";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    predicted_pub_->publish(marker);
  }

  void publishPrediction(const std::vector<VesselState> &trace)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = control_frame_;
    marker.header.stamp = now();
    marker.ns = "mpc_predicted_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.45;
    marker.color.a = 0.95F;
    marker.color.r = 0.1F;
    marker.color.g = 0.75F;
    marker.color.b = 1.0F;
    marker.points.reserve(trace.size());
    for (const auto &s : trace) {
      Point p;
      p.x = s.x; p.y = s.y; p.z = 0.22;
      marker.points.push_back(p);
    }
    predicted_pub_->publish(marker);
  }

  std::string control_frame_{"odom"};
  double frequency_{10.0}, dt_{0.1}, horizon_{5.0};
  int knot_count_{5}, optimization_sweeps_{2}, steps_{50}, knot_steps_{10};
  double max_speed_{4.0}, max_yaw_rate_{0.45}, max_surge_input_{10.0}, max_yaw_input_{5.0};
  double min_surge_input_{-10.0};
  double path_timeout_{3.0}, odom_timeout_{1.0}, map_timeout_{0.0};
  int cost_threshold_{88};
  bool channel_enabled_{true};
  int shoreline_threshold_{88}, channel_obstacle_threshold_{85};
  int main_lane_max_{20}, opposite_lane_min_{65};
  double own_radius_{1.5}, safety_buffer_{4.0}, hard_dynamic_distance_{2.0};
  double stop_distance_{2.0}, soft_dynamic_distance_{15.0};
  double goal_deceleration_{0.5}, goal_tolerance_{0.5}, intermediate_speed_{2.0};
  std::vector<double> segment_body_curvature_;
  double cross_track_recovery_distance_{8.0},course_preview_time_{2.0};
  double terminal_heading_fade_speed_{0.35}, max_yaw_input_rate_{2.0};
  double max_lateral_acceleration_{0.6}, curvature_preview_distance_{12.0};
  double terminal_hold_radius_{0.5};
  double projection_forward_distance_{6.0};
  bool terminal_hold_{false};
  double last_yaw_input_{0.0}, warm_elapsed_{0.0};
  double minimum_speed_{0.25}, surge_response_time_{0.8}, yaw_response_time_{0.8};
  double cross_track_weight_{12.0}, course_weight_{12.0}, progress_weight_{1.4};
  double speed_weight_{2.0}, sideslip_weight_{3.0}, yaw_rate_weight_{2.0};
  double effort_weight_{0.01}, torque_effort_weight_{0.008}, slew_weight_{0.6};
  double terminal_heading_weight_{50.0};
  double reverse_turn_weight_{2.5}, terminal_weight_{4.0}, dynamic_clearance_weight_{12.0};
  double total_length_{0.0};
  std::size_t projection_hint_{0};
  bool final_goal_{false};
  VesselModelParameters model_;
  std::vector<PathSegment> segments_;
  std::vector<double> bend_speed_limits_, progress_reference_;
  std::vector<VesselControl> warm_;
  nav_msgs::msg::Path::SharedPtr path_, route_;
  nav_msgs::msg::Path::SharedPtr raw_path_,raw_route_,normalized_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr raw_map_,raw_lane_map_,raw_land_map_,raw_channel_obstacle_map_;
  nav_msgs::msg::Odometry::SharedPtr raw_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr raw_active_goal_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string,FrameCache> frame_cache_;
  std::string input_frame_error_,planning_status_{"unknown"};
  double path_speed_limit_{4.0};
  bool require_speed_profile_{false},speed_profile_ready_{false};
  std::size_t path_identity_{0};
  geometry_msgs::msg::TwistStamped::SharedPtr speed_profile_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_limit_sub_;
  rclcpp::Time planning_status_receipt_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_status_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr active_goal_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr lane_map_, land_map_, channel_obstacle_map_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  hybrid_a_star_planner::msg::EncounterArray::SharedPtr policies_;
  rclcpp::Time path_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time map_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time policy_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_, route_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lane_map_sub_, land_map_sub_, channel_obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<hybrid_a_star_planner::msg::EncounterArray>::SharedPtr policy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr effort_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predicted_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcController>());
  rclcpp::shutdown();
  return 0;
}
