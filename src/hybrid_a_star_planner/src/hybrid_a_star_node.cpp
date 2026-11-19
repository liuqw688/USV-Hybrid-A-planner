// ROS inputs, event scheduling and publication. Algorithm and path helpers
// live in planner.hpp and path_utils.hpp so they can be tested independently.
#include <functional>
#include <future>
#include "hybrid_a_star_planner/planner.hpp"
#include "hybrid_a_star_planner/msg/encounter_array.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using hybrid_a_star_planner::EncounterAssessment;
using hybrid_a_star_planner::EncounterType;
using hybrid_a_star_planner::VesselState;
using hybrid_a_star_planner::planning::DynamicObstacle;
using hybrid_a_star_planner::planning::HybridAStarPlanner;
using hybrid_a_star_planner::planning::kNoGlobalRouteGoalIndex;
using hybrid_a_star_planner::planning::matchGlobalRouteGoalIndex;
using hybrid_a_star_planner::planning::isForwardRouteWaypointAdvance;
using hybrid_a_star_planner::planning::trimPathToMeasuredState;
using hybrid_a_star_planner::planning::pathHasAvoidableGoalLoop;
using hybrid_a_star_planner::planning::kDefaultGoalHeadingTolerance;

class HybridAStarNode : public rclcpp::Node
{
public:
  static constexpr std::size_t no_route_goal_index_ =
    std::numeric_limits<std::size_t>::max();

  // [功能与联系] 加载搜索/航道/动态/平滑参数，建立TF、各安全输入和风险输出。
  // timer持续做安全监测，但事件模式只在路径风险变化时运行Hybrid A*搜索。
  HybridAStarNode()
  : Node("hybrid_a_star_node")
  {
    const double frequency = declare_parameter("planner_frequency", 2.0);
    event_driven_enabled_ = declare_parameter("event_driven.enabled", true);
    event_collision_trigger_distance_ = declare_parameter(
      "event_driven.collision_trigger_distance", 2.0);
    event_cross_track_limit_ = declare_parameter(
      "event_driven.cross_track_limit", 8.0);
    event_target_course_change_ = declare_parameter(
      "event_driven.target_course_change", 5.0 * M_PI / 180.0);
    event_target_speed_change_ = declare_parameter(
      "event_driven.target_speed_change", 0.3);
    event_search_attempts_ = std::clamp(
      static_cast<int>(declare_parameter("event_driven.search_attempts", 2L)), 1, 3);
    max_path_length_ratio_ = declare_parameter("event_driven.max_path_length_ratio", 3.0);
    max_path_extra_distance_ = declare_parameter("event_driven.max_path_extra_distance", 40.0);
    goal_loop_radius_ = declare_parameter("event_driven.goal_loop_radius", 3.0);
    goal_loop_min_arc_ = declare_parameter("event_driven.goal_loop_min_arc", 8.0);
    const std::string trajectory_topic = declare_parameter<std::string>(
      "trajectory_topic", "/hybrid_a_star/trajectory");
    const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const std::string costmap_topic = declare_parameter<std::string>(
      "costmap_topic", "/local_costmap/costmap");
    const std::string lane_costmap_topic = declare_parameter<std::string>(
      "channel.lane_costmap_topic", "/channel/lane_costmap");
    const std::string static_land_topic = declare_parameter<std::string>(
      "channel.static_land_topic", "/chart_static_obstacles");
    const std::string channel_obstacle_topic = declare_parameter<std::string>(
      "channel.obstacle_costmap_topic", "/channel/obstacle_costmap");
    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter("step_size", 0.2);
    declare_parameter("min_turning_radius", 0.5);
    declare_parameter("wheelbase", 1.5);
    declare_parameter("max_steering_angle", 0.8);
    const std::string motion_model_name = declare_parameter<std::string>(
      "motion_model", "USV_DYNAMIC");
    const double usv_max_yaw_rate = declare_parameter("usv.max_yaw_rate", 0.45);
    hybrid_a_star_planner::dynamics::Parameters usv_dynamics;
    usv_dynamics.k_u = declare_parameter("usv.model.k_u", 0.290681);
    usv_dynamics.k_t = declare_parameter("usv.model.k_t", 0.295742);
    usv_dynamics.k_turn = declare_parameter("usv.model.k_turn", 2.126739);
    usv_dynamics.k_v = declare_parameter("usv.model.k_v", 0.425383);
    usv_dynamics.k_vr = declare_parameter("usv.model.k_vr", -1.149012);
    usv_dynamics.k_vn = declare_parameter("usv.model.k_vn", 0.0);
    usv_dynamics.k_r = declare_parameter("usv.model.k_r", 1.342561);
    usv_dynamics.k_r3 = declare_parameter("usv.model.k_r3", 0.0);
    usv_dynamics.k_n = declare_parameter("usv.model.k_n", 0.781688);
    usv_dynamics.surge_response_time = declare_parameter("usv.surge_response_time", 0.8);
    usv_dynamics.yaw_response_time = declare_parameter("usv.yaw_response_time", 0.8);
    usv_dynamics.max_tau_y = declare_parameter("usv.max_tau_y", 10.0);
    usv_dynamics.max_tau_n = declare_parameter("usv.max_tau_n", 5.0);
    usv_dynamics.max_yaw_rate = usv_max_yaw_rate;
    usv_dynamics.integration_step = declare_parameter("usv.integration_step", 0.05);
    declare_parameter("d_theta", 0.15);
    declare_parameter("max_iterations", 20000);
    declare_parameter("lethal_cost_threshold", 60);

    channel_enabled_ = declare_parameter("channel.enabled", true);
    const int shoreline_threshold = declare_parameter("channel.shoreline_cost_threshold", 88);
    const int channel_obstacle_threshold = declare_parameter(
      "channel.obstacle_cost_threshold", 85);
    const int main_lane_max = declare_parameter("channel.main_lane_value_max", 20);
    const int opposite_lane_min = declare_parameter("channel.opposite_lane_value_min", 65);
    const double outside_lane_weight = declare_parameter("channel.outside_lane_weight", 12.0);
    const double opposite_lane_weight = declare_parameter("channel.opposite_lane_weight", 18.0);
    const double main_center_weight = declare_parameter("channel.main_center_weight", 2.0);
    const double channel_curve_radius = declare_parameter("channel.broad_curve_radius", 60.0);
    const double channel_curve_weight = declare_parameter("channel.broad_curve_weight", 1.0);
    const double channel_curve_center_fraction = declare_parameter("channel.broad_curve_center_fraction", 0.6);
    const double outside_map_weight = declare_parameter("channel.outside_map_weight", 2.0);
    const double channel_obstacle_weight = declare_parameter(
      "channel.obstacle_inflation_weight", 5.0);
    const double analytic_expansion_distance = declare_parameter(
      "channel.analytic_expansion_distance", 18.0);
    const double reference_path_weight = declare_parameter(
      "stability.reference_path_weight", 0.5);
    const double reference_path_max_distance = declare_parameter(
      "stability.reference_path_max_distance", 12.0);
    const double steering_change_weight = declare_parameter(
      "stability.steering_change_weight", 1.0);
    const double steering_magnitude_weight = declare_parameter(
      "stability.steering_magnitude_weight", 12.0);
    const double turn_reversal_weight = declare_parameter(
      "stability.turn_reversal_weight", 24.0);
    const double turnaround_heading_threshold = declare_parameter(
      "stability.turnaround_heading_threshold", 2.0943951024);
    const double turnaround_steering_scale = declare_parameter(
      "stability.turnaround_steering_scale", 0.25);
    const bool gentle_turn_sampling = declare_parameter("usv.gentle_turn_sampling", true);
    const double sideslip_weight = declare_parameter(
      "stability.sideslip_weight", 6.0);
    const double near_field_reference_weight = declare_parameter(
      "stability.near_field_reference_weight", 12.0);
    const double near_field_reference_horizon = declare_parameter(
      "stability.near_field_reference_horizon", 45.0);
    const double near_field_deviation_scale = declare_parameter(
      "stability.near_field_deviation_scale", 3.0);
    smoothing_enabled_ = declare_parameter("smoothing.enabled", true);
    smoothing_spacing_ = declare_parameter("smoothing.resample_spacing", 1.0);
    smoothing_iterations_ = declare_parameter("smoothing.iterations", 120);
    smoothing_weight_ = declare_parameter("smoothing.smooth_weight", 0.26);
    smoothing_data_weight_ = declare_parameter("smoothing.data_weight", 0.015);
    smoothing_max_deviation_ = declare_parameter("smoothing.max_deviation", 6.0);
    smoothing_long_range_points_ = declare_parameter("smoothing.long_range_points", 6);
    smoothing_long_range_weight_ = declare_parameter("smoothing.long_range_weight", 0.12);
    geometric_reference_enabled_ = declare_parameter("geometric_reference.enabled", true);
    geometric_reference_radius_ = std::max(3.0,declare_parameter("geometric_reference.preferred_radius",8.0));
    geometric_reference_deviation_ = std::max(0.0,declare_parameter("geometric_reference.max_deviation",12.0));
    geometric_reference_time_ = std::max(.01,declare_parameter("geometric_reference.time_limit",.15));

    cruise_speed_ = declare_parameter("cruise_speed", 4.0);
    usv_dynamics.cruise_speed = cruise_speed_;
    own_ship_radius_ = declare_parameter("own_ship_radius", 0.5);
    safety_buffer_ = declare_parameter("safety_buffer", 1.0);
    const double overtaking_distance = declare_parameter("overtaking_safe_distance", 3.0);
    prediction_horizon_ = declare_parameter("dynamic_prediction_horizon", 120.0);
    const double soft_factor = declare_parameter("dynamic_soft_distance_factor", 3.5);
    avoidance_radius_ = declare_parameter("avoidance_radius", 7.0);
    dynamic_hard_distance_ = declare_parameter("dynamic_hard_distance", 3.0);
    const double dynamic_weight = declare_parameter("dynamic_cost_weight", 16.0);
    const double colregs_weight = declare_parameter("colregs_weight", 12.0);
    const double colregs_deadband = declare_parameter("colregs_heading_deadband", 0.05);
    target_timeout_ = declare_parameter("target_state_timeout", 2.0);
    own_odom_timeout_ = declare_parameter("own_odom_timeout", 5.0);
    require_target_states_ = declare_parameter("require_target_states", false);
    colregs_risk_distance_ = declare_parameter("colregs_risk_distance", 2.0);
    colregs_time_horizon_ = declare_parameter("colregs_time_horizon", 120.0);
    head_on_bearing_ = declare_parameter("head_on_bearing", 20.0 * M_PI / 180.0);
    head_on_course_tolerance_ = declare_parameter(
      "head_on_course_tolerance", 20.0 * M_PI / 180.0);
    crossing_bearing_limit_ = declare_parameter(
      "crossing_bearing_limit", 112.5 * M_PI / 180.0);
    overtaking_speed_margin_ = declare_parameter("overtaking_speed_margin", 0.2);
    risk_thresholds_.monitor_dcpa=declare_parameter("risk.monitor_dcpa",10.0);
    risk_thresholds_.action_dcpa=declare_parameter("risk.action_dcpa",5.0);
    risk_thresholds_.emergency_dcpa=declare_parameter("risk.emergency_dcpa",2.5);
    risk_thresholds_.monitor_range=declare_parameter("risk.monitor_range",80.0);
    risk_thresholds_.action_range=declare_parameter("risk.action_range",24.0);
    risk_thresholds_.emergency_range=declare_parameter("risk.emergency_range",8.0);
    risk_thresholds_.monitor_tcpa=declare_parameter("risk.monitor_tcpa",40.0);
    risk_thresholds_.action_tcpa=declare_parameter("risk.action_tcpa",16.0);
    risk_thresholds_.emergency_tcpa=declare_parameter("risk.emergency_tcpa",6.0);
    release_tcpa_=declare_parameter("risk.release_tcpa",-2.0);
    release_range_=declare_parameter("risk.release_range",6.0);
    release_observations_=declare_parameter("risk.release_observations",2);
    manoeuvre_grace_=declare_parameter("risk.manoeuvre_grace",6.0);
    visualization_enabled_=declare_parameter("visualization.enabled",true);
    target_topics_ = declare_parameter<std::vector<std::string>>(
      "target_odom_topics", {"/target_boat/odom"});
    target_radii_ = declare_parameter<std::vector<double>>(
      "target_ship_radii", {0.5});

    planner_ = std::make_unique<HybridAStarPlanner>();
    planner_->setSearchTimeLimit(declare_parameter("search_time_limit", 3.0));
    planner_->setFastGuidedConnection(declare_parameter("fast_guided_connection", true));
    result_join_distance_ = std::max(0.1, declare_parameter("result_join_distance", 1.5));
    result_join_heading_ = std::max(0.01, declare_parameter("result_join_heading", 0.35));
    planner_->setParams(
      get_parameter("step_size").as_double(), get_parameter("min_turning_radius").as_double(),
      get_parameter("wheelbase").as_double(), get_parameter("max_steering_angle").as_double(),
      get_parameter("d_theta").as_double(), static_cast<int>(get_parameter("max_iterations").as_int()),
      static_cast<int>(get_parameter("lethal_cost_threshold").as_int()));
    if (!planner_->setMotionModel(motion_model_name, usv_max_yaw_rate)) {
      RCLCPP_WARN(get_logger(),
        "Unknown motion_model '%s'; using USV_DYNAMIC", motion_model_name.c_str());
      planner_->setMotionModel("USV_DYNAMIC", usv_max_yaw_rate);
    }
    planner_->setUsvDynamics(usv_dynamics);
    planner_->setDynamicParams(
      cruise_speed_, own_ship_radius_, safety_buffer_, prediction_horizon_, soft_factor,
      avoidance_radius_,
      dynamic_weight, colregs_weight, colregs_deadband, overtaking_distance);
    planner_->setDynamicHardDistance(dynamic_hard_distance_);
    planner_->setLargeTurnParams(
      declare_parameter("large_turn.enabled",true),
      declare_parameter("large_turn.speed",2.0),
      declare_parameter("large_turn.min_radius",5.0),
      declare_parameter("large_turn.heading_threshold",1.57));
    planner_->setCloseTurnParams(declare_parameter("close_turn.range",15.0),
      declare_parameter("close_turn.speed",1.2),declare_parameter("close_turn.min_radius",3.0),
      declare_parameter("close_turn.heading_threshold",0.7853981634));
    planner_->setSpeedSampling(declare_parameter("speed_sampling.samples",10));
    planner_->setSpeedStageTimeLimit(declare_parameter("speed_sampling.stage_time_limit",0.12));
    planner_->setGoalHeadingTolerance(declare_parameter("goal_heading_tolerance",kDefaultGoalHeadingTolerance));
    planner_->setCompactManeuverParams(declare_parameter("maneuver.max_initial_straight",6.0),
      declare_parameter("maneuver.max_detour_ratio",1.7),
      declare_parameter("maneuver.compact_guidance_distance",30.0));
    speed_recovery_interval_=declare_parameter("speed_sampling.recovery_interval",8.0);
    speed_recovery_yaw_rate_=declare_parameter("speed_sampling.settled_yaw_rate",0.04);
    planner_->setAdaptiveTurnParams(declare_parameter("turning.prefer_c_curve",true));
    planner_->setPathQualityLimits(max_path_length_ratio_,max_path_extra_distance_,
      goal_loop_radius_,goal_loop_min_arc_);
    planner_->setChannelParams(
      channel_enabled_, shoreline_threshold, channel_obstacle_threshold,
      main_lane_max, opposite_lane_min, outside_lane_weight, opposite_lane_weight,
      main_center_weight, outside_map_weight, channel_obstacle_weight,
      analytic_expansion_distance);
    planner_->setLaneManeuverParams(
      declare_parameter("channel.turn_toward_goal_weight",30.0),
      declare_parameter("channel.compact_opposite_weight_scale",0.1));
    planner_->setStabilityParams(
      reference_path_weight, reference_path_max_distance, steering_change_weight,
      steering_magnitude_weight, turn_reversal_weight, sideslip_weight);
    planner_->setTurnaroundParams(turnaround_heading_threshold, turnaround_steering_scale);
    planner_->setGentleTurnSampling(gentle_turn_sampling);
    planner_->setChannelCurveParams(channel_curve_radius, channel_curve_weight, channel_curve_center_fraction);
    planner_->setNearFieldStabilityParams(
      near_field_reference_weight, near_field_reference_horizon,
      near_field_deviation_scale);
    planner_->setRecoveryParams(
      declare_parameter("stability.recovery_weight",4.0),
      declare_parameter("stability.recovery_horizon",6.0),
      declare_parameter("prediction.goal_braking",true),
      declare_parameter("prediction.goal_deceleration",0.5),
      declare_parameter("prediction.minimum_speed",0.3),
      declare_parameter("prediction.goal_tolerance",0.4));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&HybridAStarNode::costmapCallback, this, std::placeholders::_1));
    lane_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      lane_costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        lane_costmap_ = message;
        planner_->setLaneCostmap(message);
      });
    static_land_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_land_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        static_land_map_ = message;
        planner_->setStaticLandMap(message);
      });
    channel_obstacle_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      channel_obstacle_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        channel_obstacle_map_ = message;
        planner_->setChannelObstacleMap(message);
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&HybridAStarNode::goalCallback, this, std::placeholders::_1));
    const auto global_route_topic=declare_parameter<std::string>("global_route_topic", "");
    if(!global_route_topic.empty()) {
      global_route_sub_=create_subscription<nav_msgs::msg::Path>(global_route_topic,
        rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::Path::SharedPtr route) {
          global_route_=route;
          // 不同话题无到达顺序保证：活动航点先于整条route到达时补认其索引，
          // 否则下一航点（尤其末航点）会被误当新任务清空有效路径。
          if (has_goal_ && active_route_goal_index_ == no_route_goal_index_ && !route->poses.empty()) {
            active_route_goal_index_ = matchGlobalRouteGoalIndex(route.get(), goal_, no_route_goal_index_);
          }
          if(route->poses.empty()) {
            ++goal_epoch_;
            if (search_cancel_) {search_cancel_->store(true);}
            has_goal_=false;
            has_valid_path_for_goal_=false;
            last_planning_path_.clear();
            last_output_path_.poses.clear();
            planner_->setReferencePath({});
            active_route_goal_index_=no_route_goal_index_;
            route_waypoint_replan_pending_=false;
            publishEmpty("cancelled");
          }
        });
    }

    target_tracks_.resize(target_topics_.size());
    for (std::size_t i = 0; i < target_topics_.size(); ++i) {
      target_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        target_topics_[i], rclcpp::QoS(20),
        [this, i](const nav_msgs::msg::Odometry::SharedPtr message) {
          target_tracks_[i].message = message;
          target_tracks_[i].receipt_time = now();
        }));
    }
    // Transient Local 让 RViz 晚启动或短暂掉帧后仍能拿到最后一条有效轨迹。
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      trajectory_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    planning_status_pub_=create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("planning_status_topic","/hybrid_a_star/planning_status"),
      rclcpp::QoS(1).transient_local().reliable());
    publishPlanningStatus("idle");
    speed_limit_pub_=create_publisher<geometry_msgs::msg::TwistStamped>(
      declare_parameter<std::string>("speed_limit_topic","/hybrid_a_star/speed_limit"),
      rclcpp::QoS(1).transient_local().reliable());
    encounter_pub_ = create_publisher<std_msgs::msg::String>("/colregs/encounter", 10);
    risk_marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/colregs/risk_markers",10);
    avoidance_envelope_pub_=create_publisher<visualization_msgs::msg::Marker>("/colregs/avoidance_envelope",10);
    policy_pub_=create_publisher<hybrid_a_star_planner::msg::EncounterArray>("/colregs/policies",10);
    own_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",20,
      [this](nav_msgs::msg::Odometry::SharedPtr m){ own_odom_=m; });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, frequency)),
      std::bind(&HybridAStarNode::planningTimerCallback, this));
    // 搜索完成后及时消费结果，不再等1Hz监测timer，避免额外等待一整秒。
    result_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (search_future_.valid() &&
        search_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        planningTimerCallback();
      }
    });
    RCLCPP_INFO(
      get_logger(), "Hybrid A* ready: model=%s, max USV yaw rate=%.3f rad/s, "
      "%zu target topic(s), horizon %.1f s",
      planner_->motionModelName().c_str(), usv_max_yaw_rate,
      target_topics_.size(), prediction_horizon_);
  }

private:
  struct TargetTrack
  {
    nav_msgs::msg::Odometry::SharedPtr message;
    rclcpp::Time receipt_time;
    hybrid_a_star_planner::Policy policy;
    double planned_onset{-std::numeric_limits<double>::infinity()};
    double planned_target_heading{0.0};
    double planned_target_speed{0.0};
    bool has_planned_snapshot{false};
  };

  // [功能与联系] 同一次已成功处理的会遇不重复触发规则搜索；只有新锁定会遇
  // 或目标船相对规划快照发生明显机动，才生成新的规则规划事件。
  bool encounterNeedsNewPlan(const std::vector<DynamicObstacle> &obstacles) const {
    for(const auto &obstacle:obstacles) {
      if(!obstacle.policy.locked || obstacle.policy.recovering) continue;
      for(std::size_t i=0;i<target_topics_.size();++i) {
        if(obstacle.name!=target_topics_[i]) continue;
        const auto &track=target_tracks_[i];
        if(!track.has_planned_snapshot ||
          obstacle.policy.onset>track.planned_onset+1.0e-6) return true;
        const double heading=std::atan2(obstacle.state.vy,obstacle.state.vx);
        const double speed=std::hypot(obstacle.state.vx,obstacle.state.vy);
        if(std::abs(hybrid_a_star_planner::normalizeAngle(
            heading-track.planned_target_heading))>event_target_course_change_ ||
          std::abs(speed-track.planned_target_speed)>event_target_speed_change_)
        {
          return true;
        }
      }
    }
    return false;
  }

  // [功能与联系] 仅在路径成功发布后消费本次会遇事件；搜索失败不会记录，
  // 后续监测周期仍会重试，而匀速匀向目标不会反复触发。
  void recordPlannedEncounters(const std::vector<DynamicObstacle> &obstacles) {
    for(const auto &obstacle:obstacles) {
      if(!obstacle.policy.locked) continue;
      for(std::size_t i=0;i<target_topics_.size();++i) {
        if(obstacle.name!=target_topics_[i]) continue;
        auto &track=target_tracks_[i];
        track.planned_onset=obstacle.policy.onset;
        track.planned_target_heading=std::atan2(obstacle.state.vy,obstacle.state.vx);
        track.planned_target_speed=std::hypot(obstacle.state.vx,obstacle.state.vy);
        track.has_planned_snapshot=true;
      }
    }
  }

  // [功能与联系] 创建风险显示圆的Marker并设置namespace/颜色/半径；被publishRiskMarkers调用，不产生传感器感知能力。
  void addCircle(visualization_msgs::msg::MarkerArray &out,const std::string &frame,
      const std::string &ns,int id,double x,double y,double radius,double z,
      float red,float green,float blue,float alpha,double width) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id=frame;m.header.stamp=now();m.ns=ns;m.id=id;
    m.type=visualization_msgs::msg::Marker::LINE_STRIP;
    m.action=visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w=1.0;m.scale.x=width;
    m.color.r=red;m.color.g=green;m.color.b=blue;m.color.a=alpha;
    constexpr int segments=96;
    for(int i=0;i<=segments;++i) {
      const double angle=2.0*M_PI*i/segments;
      geometry_msgs::msg::Point p;p.x=x+radius*std::cos(angle);
      p.y=y+radius*std::sin(angle);p.z=z;m.points.push_back(p);
    }
    out.markers.push_back(std::move(m));
  }

  // [功能与联系] 创建风险说明文本Marker，标注CPA和风险层；供RViz诊断。
  void addText(visualization_msgs::msg::MarkerArray &out,const std::string &frame,
      const std::string &ns,int id,double x,double y,double z,const std::string &text,
      float red=1,float green=1,float blue=1) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id=frame;m.header.stamp=now();m.ns=ns;m.id=id;
    m.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action=visualization_msgs::msg::Marker::ADD;m.pose.position.x=x;
    m.pose.position.y=y;m.pose.position.z=z;m.pose.orientation.w=1;
    m.scale.z=1.6;m.color.r=red;m.color.g=green;m.color.b=blue;m.color.a=1;
    m.text=text;out.markers.push_back(std::move(m));
  }

  // RViz overlay shared with the actual policy thresholds.  Range rings are
  // centred on the target now; DCPA rings are centred on its predicted CPA
  // position; labelled points show where the target will be at each TCPA gate.
  // [功能与联系] 发布DCPA/距离三层圈、预测线和TCPA文字等诊断；风险判定来自Policy，不是看颜色圈独立触发。
  void publishRiskMarkers(const std::string &frame,double own_x,double own_y,
      const std::vector<DynamicObstacle> &obstacles) {
    if(!visualization_enabled_)return;
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker clear;clear.action=visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear);
    int id=0;
    for(const auto &o:obstacles) {
      const auto &r=risk_thresholds_;
      // Current-distance layers: monitor/yellow, action/orange, emergency/red.
      addCircle(out,frame,"range_monitor",id++,o.state.x,o.state.y,r.monitor_range,0.05,1,1,0,0.35,0.16);
      addCircle(out,frame,"range_action",id++,o.state.x,o.state.y,r.action_range,0.10,1,0.5,0,0.45,0.20);
      addCircle(out,frame,"range_emergency",id++,o.state.x,o.state.y,r.emergency_range,0.15,1,0,0,0.60,0.25);
      // DCPA/TCPA remain in the label and prediction line; extra CPA rings are
      // intentionally omitted because they obscured the three actionable layers.
      // 参考圈画在当前目标位置；真正的动态硬域随目标预测到达位置移动。
      const double hard_distance = std::max(
        std::max(own_ship_radius_ + o.state.radius + safety_buffer_, dynamic_hard_distance_),
        o.policy.type == EncounterType::OVERTAKING ?
          get_parameter("overtaking_safe_distance").as_double() : 0.0);
      const double soft_distance = std::max({hard_distance, avoidance_radius_,
        (own_ship_radius_+o.state.radius+safety_buffer_)*
        get_parameter("dynamic_soft_distance_factor").as_double()});
      addCircle(out,frame,"avoidance_range",id++,o.state.x,o.state.y,
        soft_distance,0.52,0,0.9,1,0.95,0.34);
      addCircle(out,frame,"hard_separation",id++,o.state.x,o.state.y,
        hard_distance,0.56,1,0,1,0.95,0.30);

      visualization_msgs::msg::Marker line;
      line.header.frame_id=frame;line.header.stamp=now();line.ns="target_prediction";line.id=id++;
      line.type=visualization_msgs::msg::Marker::LINE_STRIP;line.action=visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w=1;line.scale.x=0.18;line.color.r=0.2;line.color.g=0.8;line.color.b=1;line.color.a=0.9;
      for(double t:{0.0,r.emergency_tcpa,r.action_tcpa,r.monitor_tcpa}) {
        geometry_msgs::msg::Point p;p.x=o.state.x+o.state.vx*t;p.y=o.state.y+o.state.vy*t;p.z=0.65;
        line.points.push_back(p);
        if(t>0) addText(out,frame,"tcpa_labels",id++,p.x,p.y,1.2,
          "T+"+std::to_string(static_cast<int>(std::round(t)))+"s",0.3,0.9,1);
      }
      out.markers.push_back(std::move(line));
      const double current_range=std::hypot(o.state.x-own_x,o.state.y-own_y);
      std::ostringstream label;
      label<<hybrid_a_star_planner::riskName(o.policy.level)
           <<"  Range="<<std::round(current_range*10)/10<<"m"
           <<"  TCPA="<<std::round(o.policy.metric.tcpa*10)/10<<"s"
           <<"  DCPA="<<std::round(o.policy.metric.dcpa*10)/10<<"m"
           <<"\ncyan soft="<<soft_distance
           <<"m  magenta forbidden<"<<hard_distance<<"m";
      addText(out,frame,"risk_values",id++,o.state.x,o.state.y,3.2,label.str());
    }
    risk_marker_pub_->publish(out);
  }

  // [功能与联系] 缓存物理地图及其网格元数据；航道节点用于对齐语义图，规划/控制节点用于碰撞和数据就绪检查。
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    costmap_ = message;
    planner_->setCostmap(message);
  }

  // 接收全局A*活动航点；各段均从实测状态搜索，安全交接期间保留旧轨迹。
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    // A repeated copy of the same active waypoint must not erase a valid path.
    // Previously A* published at 2 Hz while Hybrid A* planned at 1 Hz, so the
    // trajectory could be cleared before it was ever visible or consumed.
    if (has_goal_ && goal_.header.frame_id == message->header.frame_id &&
      std::hypot(
        goal_.pose.position.x - message->pose.position.x,
        goal_.pose.position.y - message->pose.position.y) < 1.0e-3 &&
      std::abs(hybrid_a_star_planner::normalizeAngle(
        tf2::getYaw(goal_.pose.orientation) - tf2::getYaw(message->pose.orientation))) < 1.0e-3)
    {
      // 同位置也可能属于新任务；保留新epoch，避免global_route_topic误判旧航点。
      goal_.header=message->header;
      if (active_route_goal_index_ == no_route_goal_index_) {
        active_route_goal_index_ = matchGlobalRouteGoalIndex(global_route_.get(), goal_, no_route_goal_index_);
      }
      return;
    }

    const std::size_t previous_route_goal_index = active_route_goal_index_;
    const std::size_t route_goal_index = findGlobalRouteGoalIndex(*message);
    const bool forward_waypoint = isForwardRouteWaypointAdvance(
      has_valid_path_for_goal_, previous_route_goal_index, route_goal_index);
    // Search each leg from live odometry. During a valid forward handoff, keep
    // the old path active until the new leg is ready; do not command a stop.
    ++goal_epoch_;
    if (search_cancel_) {search_cancel_->store(true);}
    goal_ = *message;
    has_goal_ = true;
    active_route_goal_index_ = route_goal_index;
    if (forward_waypoint) {
      route_waypoint_replan_pending_ = true;
      RCLCPP_INFO(get_logger(),
        "Retaining valid path while planning forward global waypoint %zu", route_goal_index);
      planningTimerCallback(); // 立即投递后台搜索，不能再等待1Hz检查。
      return;
    }

    route_waypoint_replan_pending_ = false;
    has_valid_path_for_goal_ = false;
    last_planning_path_.clear();
    last_output_path_.poses.clear();
    planner_->setReferencePath({});
    publishEmpty("planning");
    planningTimerCallback();
  }

  std::size_t findGlobalRouteGoalIndex(const geometry_msgs::msg::PoseStamped & goal) const
  {
    return matchGlobalRouteGoalIndex(
      global_route_.get(), goal, active_route_goal_index_);
  }

  // [功能与联系] 变换目标姿态、将体速度转世界速度并补偿消息时延，更新各船Policy；输出给搜索并发布会遇摘要。
  std::vector<DynamicObstacle> collectObstacles(
    const std::string & planning_frame, double own_x, double own_y, double reference_heading)
  {
    std::vector<DynamicObstacle> obstacles;
    std::ostringstream summary;
    const auto timeout = std::chrono::milliseconds(100);
    for (std::size_t i = 0; i < target_tracks_.size(); ++i) {
      auto & track = target_tracks_[i];
      if (!track.message || (now() - track.receipt_time).seconds() > target_timeout_) {
        // 目标失联后下次重新出现必须作为新会遇重新规划，不能沿用旧事件快照。
        track.policy = hybrid_a_star_planner::Policy{};
        track.has_planned_snapshot = false;
        if (require_target_states_) {
          inputs_valid_=false;
        }
        continue;
      }
      try {
        geometry_msgs::msg::PoseStamped source;
        source.header = track.message->header;
        source.header.stamp = rclcpp::Time(0);
        source.pose = track.message->pose.pose;
        geometry_msgs::msg::PoseStamped transformed;
        tf_buffer_->transform(source, transformed, planning_frame, timeout);

        const double yaw = tf2::getYaw(transformed.pose.orientation);
        const double body_vx = track.message->twist.twist.linear.x;
        const double body_vy = track.message->twist.twist.linear.y;
        DynamicObstacle obstacle;
        obstacle.name = target_topics_[i];
        obstacle.state.x = transformed.pose.position.x;
        obstacle.state.y = transformed.pose.position.y;
        obstacle.state.vx = std::cos(yaw) * body_vx - std::sin(yaw) * body_vy;
        obstacle.state.vy = std::sin(yaw) * body_vx + std::cos(yaw) * body_vy;
        obstacle.state.radius = target_radii_.empty() ? 0.0 :
          target_radii_[std::min(i, target_radii_.size() - 1)];
        const double age=(now()-rclcpp::Time(track.message->header.stamp)).seconds();
        if(age< -0.1 || age>target_timeout_) {
          track.policy = hybrid_a_star_planner::Policy{};
          track.has_planned_snapshot = false;
          inputs_valid_=false;continue;
        }
        obstacle.state.x+=obstacle.state.vx*std::max(0.0,age);
        obstacle.state.y+=obstacle.state.vy*std::max(0.0,age);
        // Attach YAML policy values before every update so runtime parameters
        // control both classification and the markers shown in RViz.
        track.policy.thresholds=risk_thresholds_;
        track.policy.release_tcpa=release_tcpa_;
        track.policy.release_range=release_range_;
        track.policy.release_observations=release_observations_;
        track.policy.manoeuvre_grace=manoeuvre_grace_;
        track.policy.update(now().seconds(),own_x,own_y,reference_heading,
          own_odom_->twist.twist.linear.x,obstacle.state);
        obstacle.policy=track.policy;
        const double risk_distance = std::max(
          colregs_risk_distance_,
          std::max(own_ship_radius_ + obstacle.state.radius + safety_buffer_, dynamic_hard_distance_));
        obstacle.encounter = hybrid_a_star_planner::assessEncounter(
          own_x, own_y, reference_heading, cruise_speed_, obstacle.state,
          colregs_time_horizon_, risk_distance, head_on_bearing_,
          head_on_course_tolerance_, crossing_bearing_limit_, overtaking_speed_margin_);
        obstacle.encounter.type=track.policy.type;
        obstacle.encounter.active=track.policy.locked;
        obstacle.encounter.preference=track.policy.right() ?
          hybrid_a_star_planner::TurnPreference::RIGHT : track.policy.standOn() ?
          hybrid_a_star_planner::TurnPreference::KEEP_COURSE : hybrid_a_star_planner::TurnPreference::NONE;
        obstacle.encounter.tcpa = track.policy.metric.tcpa;
        obstacle.encounter.dcpa = track.policy.metric.dcpa;
        obstacle.encounter.urgency = track.policy.level == hybrid_a_star_planner::Risk::EMERGENCY ? 1.0 :
          track.policy.level == hybrid_a_star_planner::Risk::ACTION ? 0.7 : 0.4;
        if (summary.tellp() > 0) {
          summary << "; ";
        }
        summary << obstacle.name << '=' << hybrid_a_star_planner::toString(obstacle.encounter.type)
                << " risk=" << hybrid_a_star_planner::riskName(track.policy.level)
                << " takeover=" << track.policy.takeover << " effective=" << track.policy.effective
                << " tcpa=" << std::round(track.policy.metric.tcpa * 10.0) / 10.0
                << " dcpa=" << std::round(track.policy.metric.dcpa * 10.0) / 10.0;
        obstacles.push_back(obstacle);
      } catch (const tf2::TransformException & exception) {
        inputs_valid_=false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Target transform failed: %s", exception.what());
      }
    }
    std_msgs::msg::String message;
    message.data = summary.str().empty() ? "NO_ACTIVE_TARGET" : summary.str();
    encounter_pub_->publish(message);
    return obstacles;
  }

  // [功能与联系] 高频更新感知/CPA/规则状态，但仅在新任务或当前剩余路径重新
  // 变得不安全时执行搜索；安全缓存路径只刷新时间戳供MPC持续跟踪。
  double pathLength(const std::vector<geometry_msgs::msg::Pose> & path) const
  {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      length += std::hypot(path[i].position.x - path[i-1].position.x,
        path[i].position.y - path[i-1].position.y);
    }
    return length;
  }

  bool pathHasGoalLoop(const std::vector<geometry_msgs::msg::Pose> & path) const
  {
    return pathHasAvoidableGoalLoop(path,goal_loop_radius_,goal_loop_min_arc_);
  }

  bool pathQualityAcceptable(const std::vector<geometry_msgs::msg::Pose> & path,
    double sx, double sy, double gx, double gy,double turning_radius) const
  {
    if (path.size() < 2 || pathHasGoalLoop(path)) return false;
    const double direct = std::max({1.0, std::hypot(gx-sx, gy-sy),turning_radius*std::abs(
      hybrid_a_star_planner::normalizeAngle(tf2::getYaw(path.back().orientation)-
        tf2::getYaw(path.front().orientation)))});
    const double length = pathLength(path);
    return length <= direct * max_path_length_ratio_ + max_path_extra_distance_;
  }

  std::vector<geometry_msgs::msg::Pose> pathSuffixFromNearest(
    const std::vector<geometry_msgs::msg::Pose> & path, double x, double y) const
  {
    if (path.empty()) {return {};}
    std::size_t nearest = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double distance = std::hypot(path[i].position.x - x, path[i].position.y - y);
      if (distance < nearest_distance) {nearest_distance = distance; nearest = i;}
    }
    return {path.begin() + static_cast<std::ptrdiff_t>(nearest), path.end()};
  }

  double timeFromCurrentPoseToPathTail(
    const std::vector<geometry_msgs::msg::Pose> & path, double x, double y) const
  {
    if (path.empty()) {return 0.0;}
    std::size_t nearest = 0U;
    double distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double candidate = std::hypot(path[i].position.x - x, path[i].position.y - y);
      if (candidate < distance) {distance = candidate; nearest = i;}
    }
    for (std::size_t i = nearest + 1U; i < path.size(); ++i) {
      distance += std::hypot(
        path[i].position.x - path[i - 1U].position.x,
        path[i].position.y - path[i - 1U].position.y);
    }
    return distance / std::max(0.05, cruise_speed_);
  }

  void planningTimerCallback()
  {
    if(global_route_sub_ && (!global_route_ || global_route_->poses.empty() ||
      rclcpp::Time(goal_.header.stamp)<rclcpp::Time(global_route_->header.stamp))) return;
    if (!costmap_ || !has_goal_ || !own_odom_ || (channel_enabled_ && !lane_costmap_)) {
      return;
    }
    try {
      const std::string planning_frame = costmap_->header.frame_id;
      const auto timeout = std::chrono::milliseconds(100);
      const rclcpp::Time latest(0);
      geometry_msgs::msg::PoseStamped base_pose;
      base_pose.header.frame_id = own_odom_->header.frame_id;
      base_pose.header.stamp = latest;
      base_pose.pose = own_odom_->pose.pose;
      geometry_msgs::msg::PoseStamped start;
      tf_buffer_->transform(base_pose, start, planning_frame, timeout);

      geometry_msgs::msg::PoseStamped goal = goal_;
      goal.header.stamp = latest;
      geometry_msgs::msg::PoseStamped transformed_goal;
      tf_buffer_->transform(goal, transformed_goal, planning_frame, timeout);
      const double reference_heading = tf2::getYaw(start.pose.orientation);
      // 后台搜索不再阻塞odom回调；这里只检查真实传感器数据时效。
      inputs_valid_ = (now() - rclcpp::Time(own_odom_->header.stamp)).seconds() < own_odom_timeout_;
      auto obstacles=collectObstacles(
        planning_frame, start.pose.position.x, start.pose.position.y, reference_heading);
      publishRiskMarkers(planning_frame,start.pose.position.x,start.pose.position.y,obstacles);
      planner_->setDynamicObstacles(obstacles);
      if(!inputs_valid_) {
        publishPolicies(planning_frame, obstacles);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Odometry is stale; waiting without clearing RViz path");
        return;
      }

      const bool new_encounter_event=encounterNeedsNewPlan(obstacles);
      const bool remaining_path_unsafe = planner_->remainingPathNeedsReplan(
        last_planning_path_, start.pose.position.x, start.pose.position.y,
        reference_heading, event_collision_trigger_distance_, event_cross_track_limit_, false);
      const double since_recovery=(now()-last_speed_recovery_attempt_).seconds();
      const bool speed_recovery=has_valid_path_for_goal_ && last_path_speed_limit_<cruise_speed_-.05 &&
        since_recovery>=speed_recovery_interval_ &&
        ((std::abs(own_odom_->twist.twist.angular.z)<speed_recovery_yaw_rate_ &&
         std::abs(own_odom_->twist.twist.linear.y)<0.15) ||
         since_recovery>=2*speed_recovery_interval_) &&
        std::hypot(transformed_goal.pose.position.x-start.pose.position.x,
          transformed_goal.pose.position.y-start.pose.position.y)>close_turn_range_for_recovery_;
      const bool path_requires_replan = speed_recovery || !has_valid_path_for_goal_ ||
        route_waypoint_replan_pending_ ||
        new_encounter_event || remaining_path_unsafe;
      if (event_driven_enabled_ && !path_requires_replan && !search_future_.valid()) {
        // 当前轨迹仍安全时不运行搜索。历史会遇锁仍可保留诊断，但目标已安全
        // 通过时立即标记恢复，MPC不必等待release计时才继续跟踪原避让轨迹。
        for (auto & obstacle : obstacles) {
          const double safe = std::max({
            own_ship_radius_ + obstacle.state.radius + safety_buffer_, dynamic_hard_distance_,
            obstacle.policy.type == EncounterType::OVERTAKING ?
              get_parameter("overtaking_safe_distance").as_double() : 0.0});
          if (obstacle.policy.safeToRecover(
              start.pose.position.x, start.pose.position.y, obstacle.state, safe, false))
          {
            obstacle.policy.recovering = true;
            for (std::size_t i = 0; i < target_tracks_.size(); ++i) {
              if (obstacle.name == target_topics_[i]) {
                target_tracks_[i].policy.recovering = true;
                break;
              }
            }
          }
        }
        planner_->setDynamicObstacles(obstacles);
        publishPolicies(planning_frame, obstacles);
        publishAvoidanceEnvelope(planning_frame, last_planning_path_, std::any_of(
          obstacles.begin(), obstacles.end(), [](const auto & obstacle) {
            return obstacle.policy.locked && !obstacle.policy.recovering;
          }));
        refreshCachedPath();
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Event mode: cached Hybrid A* path remains safe; search skipped");
        return;
      }

      // Every waypoint and safety replan starts from the measured vessel state.
      // Dynamic expansion uses body-frame u/v/r as well as position and yaw.
        double sx = start.pose.position.x;
      double sy = start.pose.position.y;
      double stheta = tf2::getYaw(start.pose.orientation);
      double initial_time = 0.0;
      const double start_u = own_odom_->twist.twist.linear.x;
      const double start_v = own_odom_->twist.twist.linear.y;
      const double start_r = own_odom_->twist.twist.angular.z;
      const auto search_reference_path = pathSuffixFromNearest(
        last_planning_path_, start.pose.position.x, start.pose.position.y);
      planner_->setReferencePath(speed_recovery ? std::vector<geometry_msgs::msg::Pose>{} : search_reference_path);

      // 独立搜索副本只使用启动时的输入快照；ROS回调持续接收最新船位/海图。
      // pending期间每个安全监测周期核验旧路径；不靠放宽MPC超时强行续航。
      if (!search_future_.valid()) {
        last_speed_recovery_attempt_=now();
        speed_recovery_search_=speed_recovery;
        search_epoch_ = goal_epoch_;
        auto worker = std::make_shared<HybridAStarPlanner>(*planner_);
        search_cancel_ = std::make_shared<std::atomic<bool>>(false);
        worker->setSearchCancellation(search_cancel_);
        worker->startOverallSearchBudget();
        const auto search_goal = transformed_goal;
        search_future_ = std::async(std::launch::async,
          [this, worker, obstacles, search_goal, sx, sy, stheta, initial_time,
           start_u, start_v, start_r, search_reference_path]() mutable {
          const auto & transformed_goal = search_goal;
            auto poses = worker->search(
              sx, sy, stheta,
              transformed_goal.pose.position.x, transformed_goal.pose.position.y,
              tf2::getYaw(transformed_goal.pose.orientation), initial_time,
              start_u, start_v, start_r);
            // 对遇首先强制右舷通过。只有地形或障碍令右侧走廊完全无解时，
            // 才按“安全优先于规则”临时放开侧向限制执行一次回退搜索。
            if(poses.empty() && std::any_of(obstacles.begin(),obstacles.end(),[](const auto &o){
                return o.policy.locked && !o.policy.recovering &&
                  o.policy.type==hybrid_a_star_planner::EncounterType::HEAD_ON;})) {
              worker->setHeadOnStarboardEnforced(false);
              poses=worker->search(
                sx,sy,stheta,transformed_goal.pose.position.x,transformed_goal.pose.position.y,
                tf2::getYaw(transformed_goal.pose.orientation),initial_time,start_u,start_v,start_r);
              worker->setHeadOnStarboardEnforced(true);
              if(!poses.empty()) RCLCPP_WARN(
                get_logger(),"Starboard head-on corridor infeasible; safety fallback used");
            }
            // 第一条候选通过长度/绕圈质量门槛后立即发布，不再无条件重复完整搜索。
            // 只有首条候选异常时才清除旧路径软参考并计算备用候选；硬安全与COLREG
            // 检查始终不变。这样正常场景通常只搜索一次，困难场景仍保留自救重试。
            const bool first_candidate_needs_retry = !poses.empty() && !pathQualityAcceptable(
              poses, sx, sy,
              transformed_goal.pose.position.x, transformed_goal.pose.position.y,worker->planningRadius());
            if (first_candidate_needs_retry && event_search_attempts_ > 1) {
              std::vector<geometry_msgs::msg::Pose> best = poses;
              double best_length = pathLength(best);
              for (int attempt = 1; attempt < event_search_attempts_; ++attempt) {
                worker->setDynamicObstacles(obstacles);
                worker->setReferencePath(search_reference_path);
                auto candidate = worker->search(
                  sx,sy,stheta,transformed_goal.pose.position.x,transformed_goal.pose.position.y,
                  tf2::getYaw(transformed_goal.pose.orientation),initial_time,start_u,start_v,start_r);
                if (!candidate.empty() && pathQualityAcceptable(candidate,
                    sx, sy,
                    transformed_goal.pose.position.x, transformed_goal.pose.position.y,worker->planningRadius())) {
                  const double length = pathLength(candidate);
                  if (length < best_length) { best = std::move(candidate); best_length = length; }
                }
              }
              poses = std::move(best);
              worker->setReferencePath(search_reference_path);
            }
            // If the stand-on corridor itself has no collision-free continuation,
            // waiting or stopping would be the unsafe choice.  Treat infeasibility
            // as Rule 17 intervention and immediately re-plan as the give-way vessel.
            if(poses.empty()) {
              bool retry=false;
              for(auto &o:obstacles) {
                const bool observation_complete=now().seconds()-o.policy.onset>=3.0;
                if(o.policy.standOn() && !o.policy.effective &&
                  (o.policy.level==hybrid_a_star_planner::Risk::EMERGENCY || observation_complete)) {
                  o.policy.takeover=true;o.policy.level=hybrid_a_star_planner::Risk::EMERGENCY;
                  o.encounter.preference=hybrid_a_star_planner::TurnPreference::RIGHT;
                  retry=true;
                }
              }
              if(retry) {
                worker->setDynamicObstacles(obstacles);
                poses=worker->search(
                  sx,sy,stheta,transformed_goal.pose.position.x,transformed_goal.pose.position.y,
                  tf2::getYaw(transformed_goal.pose.orientation),initial_time,start_u,start_v,start_r);
                RCLCPP_WARN(get_logger(),"Stand-on corridor infeasible; Rule 17 takeover activated");
              }
            }
          std::vector<geometry_msgs::msg::Pose> integrated_path;
          if (geometric_reference_enabled_ && worker->usesUsvDynamicModel() && !poses.empty()) {
            integrated_path = poses;
            poses = worker->optimizeTrackingPath(poses,tf2::getYaw(transformed_goal.pose.orientation),
              geometric_reference_radius_,geometric_reference_deviation_,geometric_reference_time_);
          } else if (smoothing_enabled_ && !worker->usesUsvDynamicModel() && !poses.empty()) {
            poses = worker->postProcessPath(poses, smoothing_spacing_, smoothing_iterations_,
              smoothing_weight_, smoothing_data_weight_, smoothing_max_deviation_,
              smoothing_long_range_points_, smoothing_long_range_weight_);
          }
          return SearchResult{std::move(poses), worker, std::move(integrated_path)};
        });
      }
      if (search_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        publishPolicies(planning_frame, obstacles);
        if (has_valid_path_for_goal_ && !remaining_path_unsafe) {refreshCachedPath();}
        else {publishEmpty((has_valid_path_for_goal_ && remaining_path_unsafe) ||
          planning_status_=="blocked" ? "blocked" : "planning");}
        return;
      }
      auto result = search_future_.get();
      if (search_epoch_ != goal_epoch_) {
        // 取消的任务已释放worker，立即从最新实测状态启动新航点搜索。
        RCLCPP_INFO(get_logger(), "Discarding completed search for an obsolete goal");
        planningTimerCallback();
        return;
      }
      planner_->adoptSearchCache(*result.worker);
      auto poses = std::move(result.poses);
      if(speed_recovery_search_ && has_valid_path_for_goal_ && !remaining_path_unsafe &&
        (poses.empty() || result.worker->planningSpeed()<=last_path_speed_limit_+.05)) {
        refreshCachedPath();return;
      }
      const bool cached_maneuver=planner_->maneuverActive();
      const bool cached_tight_maneuver=planner_->tightManeuver();
      const double cached_speed=planner_->planningSpeed(),cached_radius=planner_->planningRadius();
      // 只合并仍属于同一次活动会遇的Rule17接管标志，不覆盖最新船位/CPA。
      for (auto & obstacle : obstacles) {
        for (const auto & searched : result.worker->dynamicObstacles()) {
          if (obstacle.name == searched.name && obstacle.policy.locked &&
            obstacle.policy.type == searched.policy.type && searched.policy.takeover) {
            obstacle.policy.takeover = true;
            for (std::size_t i = 0; i < target_tracks_.size(); ++i) {
              auto & track = target_tracks_[i];
              if (track.policy.locked && track.policy.type == searched.policy.type &&
                track.message && obstacle.name == target_topics_[i]) {
                track.policy.takeover = true;
              }
            }
          }
        }
      }
      planner_->setDynamicObstacles(obstacles);
      if(!poses.empty()) {planner_->setActiveManeuver(result.worker->maneuverActive(),result.worker->tightManeuver(),
        result.worker->planningSpeed(),result.worker->planningRadius());}
      auto joins_current_state_safely = [&](std::vector<geometry_msgs::msg::Pose> & candidate) {
        return !candidate.empty() && trimPathToMeasuredState(candidate,
          start.pose.position.x,start.pose.position.y,reference_heading,
          result_join_distance_,result_join_heading_) &&
          !planner_->remainingPathNeedsReplan(candidate,
            start.pose.position.x,start.pose.position.y,reference_heading,
            event_collision_trigger_distance_,event_cross_track_limit_,false) &&
          pathQualityAcceptable(candidate,sx,sy,transformed_goal.pose.position.x,
            transformed_goal.pose.position.y,planner_->planningRadius());
      };
      // 几何参考若无法衔接实时船位，尝试同次搜索的原船模轨迹；两者
      // 都复查最新地图和交通，不能因后优化产生新的无路径停车。
      if (!poses.empty() && !joins_current_state_safely(poses)) {
        auto original = std::move(result.integrated_path);
        if (joins_current_state_safely(original)) {
          poses = std::move(original);
          RCLCPP_INFO(get_logger(),"Geometric reference cannot join safely; retaining verified dynamic rollout");
        } else {
          poses.clear();
          RCLCPP_WARN(get_logger(),"Completed path cannot join safely; retrying from measured state");
        }
      }
      obstacles=planner_->dynamicObstacles();
      syncRecoveryFlags(obstacles);
      publishPolicies(planning_frame,obstacles);
      if (poses.empty()) {
        planner_->setActiveManeuver(cached_maneuver,cached_tight_maneuver,cached_speed,cached_radius);
        const bool traffic_relevant=std::any_of(obstacles.begin(),obstacles.end(),[this](const auto &o){
          return o.policy.locked || o.policy.metric.range<=risk_thresholds_.monitor_range;});
        handlePlanningFailure(traffic_relevant);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Hybrid A* found no safe path");
        return;
      }
      if (!pathQualityAcceptable(poses, sx, sy,
          transformed_goal.pose.position.x, transformed_goal.pose.position.y,planner_->planningRadius())) {
        planner_->setActiveManeuver(cached_maneuver,cached_tight_maneuver,cached_speed,cached_radius);
        const bool traffic_relevant = std::any_of(obstacles.begin(), obstacles.end(),
          [](const auto & o) { return o.policy.locked || o.encounter.active; });
        handlePlanningFailure(traffic_relevant);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Rejected looping/excessively long Hybrid A* path near goal; waiting for a safer replan");
        return;
      }
      // New trajectories start at measured vessel state; never prepend an
      // already-sailed prefix or create an artificial endpoint join.
      auto published_poses = poses;

      publishAvoidanceEnvelope(planning_frame,published_poses,std::any_of(
        obstacles.begin(),obstacles.end(),[](const auto &o){return o.policy.locked;}));
      nav_msgs::msg::Path path;
      path.header.frame_id = global_frame_;
      path.header.stamp = now();
      path.poses.reserve(published_poses.size());
      for (const auto & pose : published_poses) {
        geometry_msgs::msg::PoseStamped source;
        source.header.frame_id = planning_frame;
        source.header.stamp = latest;
        source.pose = pose;
        geometry_msgs::msg::PoseStamped output;
        tf_buffer_->transform(source, output, global_frame_, timeout);
        output.header = path.header;
        path.poses.push_back(output);
      }
      last_path_speed_limit_=result.worker->planningSpeed();
      last_output_path_ = path;
      publishSpeedLimit();
      path_pub_->publish(path);
      publishPlanningStatus("tracking");
      last_planning_path_ = std::move(published_poses);
      last_output_path_ = path;
      has_valid_path_for_goal_ = true;
      route_waypoint_replan_pending_ = false;
      last_path_success_time_ = now();
      recordPlannedEncounters(obstacles);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Published V4 constrained path with %zu poses; C_turn=%s reference_speed=%.2f radius_scale=%.2f",
        poses.size(),result.worker->cTurnSelected() ? "yes" : "fallback",last_path_speed_limit_,result.worker->planningRadius());
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Planning transform failed; retaining last RViz path: %s", exception.what());
    } catch (const std::exception & exception) {
      has_valid_path_for_goal_ = false;
      publishEmpty();
      RCLCPP_ERROR(get_logger(), "Planning worker failed; retrying next cycle: %s", exception.what());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lane_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_land_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr channel_obstacle_sub_;
  // A wide translucent line makes the planned detour footprint immediately
  // visible.  It disappears as soon as no encounter policy is locked.
  // [功能与联系] 绘制活动避让时的宽轨迹包络给RViz；是可视化，不是独立感知或碰撞触发器。
  void publishAvoidanceEnvelope(const std::string &frame,
      const std::vector<geometry_msgs::msg::Pose> &poses,bool active) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id=frame;marker.header.stamp=now();marker.ns="planned_avoidance_envelope";
    marker.id=0;marker.action=active ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    marker.type=visualization_msgs::msg::Marker::LINE_STRIP;marker.pose.orientation.w=1;
    marker.scale.x=2.0*avoidance_radius_;marker.color.r=0;marker.color.g=0.85;
    marker.color.b=1;marker.color.a=0.12;
    if(active) for(const auto &pose:poses) {
      geometry_msgs::msg::Point p;p.x=pose.position.x;p.y=pose.position.y;p.z=0.02;
      marker.points.push_back(p);
    }
    avoidance_envelope_pub_->publish(marker);
  }
  // [功能与联系] 将每艘目标的会遇类型、风险、恢复状态及运动状态发布给MPC，统一两级动态约束。
  void publishPolicies(const std::string &frame,const std::vector<DynamicObstacle> &obstacles) {
    hybrid_a_star_planner::msg::EncounterArray policies;
    policies.header.frame_id=frame;policies.header.stamp=now();policies.valid=inputs_valid_;
    for(const auto &o:obstacles) {
      hybrid_a_star_planner::msg::Encounter e;
      e.type=static_cast<int>(o.policy.type);e.risk=static_cast<int>(o.policy.level);
      e.locked=o.policy.locked;e.takeover=o.policy.takeover;
      e.recovering=o.policy.recovering;
      e.reference=o.policy.reference;e.speed=o.policy.speed;
      e.x=o.state.x;e.y=o.state.y;e.vx=o.state.vx;e.vy=o.state.vy;e.radius=o.state.radius;
      policies.encounters.push_back(e);
    }
    policy_pub_->publish(policies);
  }
  // [功能与联系] 显式发布空局部Path以撤销任务并使MPC停车；普通搜索失败采用短过期保护，不周期性擦除RViz结果。
  void publishPlanningStatus(const std::string &status) {
    planning_status_=status;
    if(planning_status_pub_) {
      std_msgs::msg::String message;message.data=status;planning_status_pub_->publish(message);
    }
  }
  void publishSpeedLimit() {
    if(speed_limit_pub_) {geometry_msgs::msg::TwistStamped message;message.header=last_output_path_.header;
      message.twist.linear.x=last_path_speed_limit_;
      speed_limit_pub_->publish(message);}
  }
  void publishEmpty(const std::string &status="blocked") {
    publishPlanningStatus(status);
    nav_msgs::msg::Path p;p.header.frame_id=global_frame_;p.header.stamp=now();
    if(path_pub_)path_pub_->publish(p);
  }
  // [功能与联系] 不重新搜索，只给同一条全局坐标路径刷新消息时间戳；
  // 防止MPC把事件模式下仍有效的路径误判为过期，不改变任何轨迹点。
  void refreshCachedPath() {
    if(!has_valid_path_for_goal_ || last_output_path_.poses.size()<2U) return;
    last_output_path_.header.stamp=now();
    for(auto &pose:last_output_path_.poses) pose.header=last_output_path_.header;
    path_pub_->publish(last_output_path_);
    publishSpeedLimit();
    publishPlanningStatus("tracking");
  }
  // [功能与联系] search内部算出的即时恢复状态写回目标轨迹状态机，确保下一次
  // 只做轻量监测时，Hybrid和MPC仍使用同一会遇阶段。
  void syncRecoveryFlags(const std::vector<DynamicObstacle> &obstacles) {
    for(const auto &obstacle:obstacles) {
      for(std::size_t i=0;i<target_topics_.size();++i) {
        if(obstacle.name==target_topics_[i]) {
          target_tracks_[i].policy.recovering=obstacle.policy.recovering;
          break;
        }
      }
    }
  }
  // [功能与联系] 搜索失败时复验旧路径；安全则持续跟踪，不安全则通知控制器停车。
  void handlePlanningFailure(bool traffic_present) {
    // 只在本周期已用最新输入验证安全时继续发布旧路径；附近有船不等于
    // 旧路径必然危险。无有效安全路径则显式停车，并保留后续重试条件。
    if (has_valid_path_for_goal_ && own_odom_ && !planner_->remainingPathNeedsReplan(
        last_planning_path_, own_odom_->pose.pose.position.x, own_odom_->pose.pose.position.y,
        tf2::getYaw(own_odom_->pose.pose.orientation), event_collision_trigger_distance_,
        event_cross_track_limit_, false)) {
      refreshCachedPath();
      return;
    }
    (void)traffic_present;
    has_valid_path_for_goal_ = false;
    publishEmpty();
    return;
  }
  bool inputs_valid_{false};
  nav_msgs::msg::Odometry::SharedPtr own_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_sub_;
  rclcpp::Publisher<hybrid_a_star_planner::msg::EncounterArray>::SharedPtr policy_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> target_subs_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_pub_;
  std::string planning_status_{"idle"};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_limit_pub_;
  double last_path_speed_limit_{4.0};
  double speed_recovery_interval_{8.0},speed_recovery_yaw_rate_{0.04};
  double close_turn_range_for_recovery_{15.0};
  rclcpp::Time last_speed_recovery_attempt_{0,0,RCL_ROS_TIME};
  bool speed_recovery_search_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr encounter_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr avoidance_envelope_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr result_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr lane_costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_land_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr channel_obstacle_map_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_goal_{false};
  nav_msgs::msg::Path::SharedPtr global_route_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_route_sub_;
  std::size_t active_route_goal_index_{no_route_goal_index_};
  bool route_waypoint_replan_pending_{false};
  bool channel_enabled_{true};
  bool require_target_states_{false};
  std::string global_frame_;
  std::string base_frame_;
  std::vector<std::string> target_topics_;
  std::vector<double> target_radii_;
  std::vector<TargetTrack> target_tracks_;
  double cruise_speed_{0.8};
  double own_ship_radius_{0.5};
  double safety_buffer_{1.0};
  double dynamic_hard_distance_{3.0};
  double prediction_horizon_{120.0};
  double avoidance_radius_{7.0};
  double target_timeout_{2.0};
  double own_odom_timeout_{5.0};
  bool event_driven_enabled_{true};
  double event_collision_trigger_distance_{2.0};
  double event_cross_track_limit_{8.0};
  double event_target_course_change_{5.0*M_PI/180.0};
  double event_target_speed_change_{0.3};
  int event_search_attempts_{2};
  double max_path_length_ratio_{3.0};
  double max_path_extra_distance_{40.0};
  double goal_loop_radius_{3.0};
  double goal_loop_min_arc_{8.0};
  bool has_valid_path_for_goal_{false};
  rclcpp::Time last_path_success_time_{0, 0, RCL_ROS_TIME};
  double colregs_risk_distance_{2.0};
  double colregs_time_horizon_{120.0};
  double head_on_bearing_{15.0 * M_PI / 180.0};
  double head_on_course_tolerance_{30.0 * M_PI / 180.0};
  double crossing_bearing_limit_{112.5 * M_PI / 180.0};
  double overtaking_speed_margin_{0.2};
  hybrid_a_star_planner::RiskThresholds risk_thresholds_{};
  double release_tcpa_{-2.0};
  double release_range_{18.0};
  double manoeuvre_grace_{6.0};
  int release_observations_{2};
  bool visualization_enabled_{true};
  bool smoothing_enabled_{true};
  double smoothing_spacing_{1.0};
  int smoothing_iterations_{120};
  double smoothing_weight_{0.26};
  double smoothing_data_weight_{0.015};
  double smoothing_max_deviation_{6.0};
  int smoothing_long_range_points_{6};
  double smoothing_long_range_weight_{0.12};
  bool geometric_reference_enabled_{true};
  double geometric_reference_radius_{8.0},geometric_reference_deviation_{12.0},geometric_reference_time_{.15};
  std::vector<geometry_msgs::msg::Pose> last_planning_path_;
  nav_msgs::msg::Path last_output_path_;
  std::unique_ptr<HybridAStarPlanner> planner_;
  struct SearchResult {
    std::vector<geometry_msgs::msg::Pose> poses;
    std::shared_ptr<HybridAStarPlanner> worker;
    std::vector<geometry_msgs::msg::Pose> integrated_path;
  };
  std::size_t goal_epoch_{0U}, search_epoch_{0U};
  double result_join_distance_{1.5}, result_join_heading_{0.35};
  std::shared_ptr<std::atomic<bool>> search_cancel_;
  // 最后声明使析构首先等待worker，避免worker读取已析构的参数成员。
  std::future<SearchResult> search_future_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridAStarNode>());
  rclcpp::shutdown();
  return 0;
}
