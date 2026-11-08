#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include "hybrid_a_star_planner/policy.hpp"
#include "hybrid_a_star_planner/usv_dynamics.hpp"
#include "hybrid_a_star_planner/path_utils.hpp"
#include "hybrid_a_star_planner/path_geometry.hpp"
#include "hybrid_a_star_planner/lane_guidance.hpp"
#include "hybrid_a_star_planner/maneuver_shooting.hpp"

namespace hybrid_a_star_planner::planning
{
struct DynamicObstacle
{
  std::string name;
  VesselState state;
  EncounterAssessment encounter;
  hybrid_a_star_planner::Policy policy;
};

enum class MotionModel
{
  USV_KINEMATIC,
  USV_DYNAMIC,
  CAR_LIKE
};

struct Node3D
{
  float x{0.0F};
  float y{0.0F};
  float theta{0.0F};
  float g{0.0F};
  float h{0.0F};
  float f{0.0F};
  double t{0.0};
  // Normalized steering/yaw-rate command in [-1, 1], interpreted by motion_model_.
  float turn_control{0};
  // Last non-zero turn direction. Preserve it across straight primitives so a
  // delayed reversal is still recognized as an S-turn rather than a fresh turn.
  int turn_direction{0};
  double u{0.0};
  double v{0.0};
  double r{0.0};
  int idx{-1};
  std::shared_ptr<Node3D> parent;
};

struct CompareNode
{
  bool operator()(const std::shared_ptr<Node3D> & a, const std::shared_ptr<Node3D> & b) const
  {
    return a->f > b->f;
  }
};

class HybridAStarPlanner
{
public:
  // 墙钟预算限制失败搜索；0用于离线算法测试、不设预算。
  void setSearchTimeLimit(double seconds) {search_time_limit_ = std::max(0.0, seconds);}
  void setFastGuidedConnection(bool enabled) {fast_guided_connection_ = enabled;}
  void setLargeTurnParams(bool enabled,double speed,double radius,double angle) {
    large_turn_enabled_=enabled;large_turn_speed_=std::clamp(speed,0.5,cruise_speed_);
    large_turn_radius_=std::max(3.0,radius);large_turn_angle_=std::clamp(angle,0.5,3.14);
  }
  void setCloseTurnParams(double range,double speed,double radius,double angle=0.7853981634) {
    close_turn_range_=std::max(2.0,range);close_turn_speed_=std::clamp(speed,0.5,cruise_speed_);
    close_turn_radius_=std::max(3.0,radius);
    close_turn_angle_=std::clamp(angle,0.5,3.14);
  }
  void setAdaptiveTurnParams(bool enabled) {prefer_single_turn_=enabled;}
  void setSpeedSampling(int samples) {speed_samples_=std::clamp(samples,1,30);}
  void setSpeedStageTimeLimit(double seconds) {speed_stage_time_limit_=std::max(0.01,seconds);}
  void setGoalHeadingTolerance(double radians) {goal_heading_tolerance_=std::clamp(radians,.05,1.0);}
  double goalHeadingTolerance() const {return goal_heading_tolerance_;}
  void setCompactManeuverParams(double straight,double ratio,double guidance_distance=30.0) {
    maneuver_straight_limit_=std::max(1.0,straight);maneuver_detour_ratio_=std::max(1.1,ratio);
    compact_guidance_distance_=std::max(5.0,guidance_distance);
  }
  std::vector<double> speedSamples() const {
    std::vector<double> speeds;
    for(int i=0;i<speed_samples_;++i) {
      speeds.push_back(cruise_speed_-(cruise_speed_-close_turn_speed_)*i/std::max(1,speed_samples_-1));
    }
    return speeds;
  }
  const std::vector<double> & attemptedSpeeds() const {return attempted_speeds_;}
  void setShootingRescue(bool enabled,double seconds=1.2,double minimum_speed=.6,double minimum_radius=1.5) {
    rescue_enabled_=enabled;rescue_time_=std::clamp(seconds,.02,2.0);
    rescue_min_speed_=std::clamp(minimum_speed,.5,1.2);rescue_min_radius_=std::max(1.5,minimum_radius);
  }
  bool usedShootingRescue() const {return rescue_selected_;}
  const ShootingControl& shootingControl()const{return last_shooting_control_;}
  const std::vector<dynamics::State>& shootingTrace()const{return last_shooting_trace_;}
  void setLaneManeuverParams(double turn_weight,double opposite_scale) {
    lane_turn_weight_=std::max(0.0,turn_weight);
    compact_opposite_scale_=std::clamp(opposite_scale,0.0,1.0);
  }
  // A finite preference: safe alternatives remain available when entry is blocked.
  void setMainLaneEntryParams(double weight,double direct_distance,double horizon) {
    const double next_weight=std::max(0.0,weight);
    const double next_distance=std::max(0.0,direct_distance);
    const double next_horizon=std::max(5.0,horizon);
    if(next_weight!=main_entry_weight_ || next_distance!=main_entry_direct_distance_ ||
      next_horizon!=main_entry_horizon_) ++static_map_revision_;
    main_entry_weight_=next_weight;
    main_entry_direct_distance_=next_distance;
    main_entry_horizon_=next_horizon;
  }
  // Compare geometry and integrated candidates with the same spatial metric.
  double mainLaneEntryCost(const std::vector<geometry_msgs::msg::Pose> &path) const {
    if(path.empty() || !mainLaneEntryApplicable(path.front().position.x,path.front().position.y,
      path.back().position.x,path.back().position.y)) return 0;
    const double initial=main_lane_distance_.distance(path.front().position.x,path.front().position.y);
    if(!std::isfinite(initial)) return 0;
    double arc=0,cost=0,away=0;
    const auto sampled=resampleGroundPath(path,.5);
    for(std::size_t i=1;i<sampled.size() && arc<main_entry_horizon_;++i) {
      const auto &a=sampled[i-1].position,&b=sampled[i].position;
      const double ds=std::min(main_entry_horizon_-arc,std::hypot(b.x-a.x,b.y-a.y));
      const double d=main_lane_distance_.distance(b.x,b.y);
      // Unknown/absent lane evidence must not create a new turn obligation.
      if(!std::isfinite(d)) return 0;
      cost+=ds*std::min(2.0,d/std::max(5.0,initial));
      away=std::max(away,d-initial);arc+=ds;
    }
    return main_entry_weight_*(cost/main_entry_horizon_+
      std::min(1.0,away/main_entry_horizon_));
  }
  double planningSpeed() const {if(active_turn_speed_>0) {return active_turn_speed_;}return maneuver_active_ ?
    (tight_maneuver_ ? close_turn_speed_ : large_turn_speed_) : cruise_speed_;}
  bool maneuverActive() const {return maneuver_active_;}
  bool tightManeuver() const {return tight_maneuver_;}
  bool cTurnSelected() const {return c_turn_selected_;}
  void setActiveManeuver(bool active,bool tight=false,double speed=0,double radius=0) {
    maneuver_active_=active;tight_maneuver_=tight;active_turn_speed_=speed;active_turn_radius_=radius;
  }
  double planningRadius() const {if(active_turn_radius_>0) {return active_turn_radius_;}return maneuver_active_ ? std::min<double>(min_turning_radius_,
    tight_maneuver_ ? close_turn_radius_ : large_turn_radius_) : min_turning_radius_;}
  void setPathQualityLimits(double ratio,double extra,double radius,double arc) {
    maximum_path_ratio_=std::max(1.0,ratio);maximum_path_extra_=std::max(0.0,extra);
    loop_check_radius_=radius;loop_check_arc_=arc;
  }
  void setSearchCancellation(std::shared_ptr<std::atomic<bool>> token) {search_cancel_ = std::move(token);}
  void startOverallSearchBudget() {
    overall_deadline_=search_time_limit_>0 ? std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(search_time_limit_)) :
      std::chrono::steady_clock::time_point::max();
  }
  bool searchTimeExpired() const {
    return std::chrono::steady_clock::now()>=overall_deadline_ || (search_cancel_ && search_cancel_->load()) ||
      (search_time_limit_ > 0.0 && std::chrono::steady_clock::now() >= search_deadline_);
  }
  // [功能与联系] 配置运动基元和搜索分辨率/迭代限制；由ROS节点构造时赋值，影响search状态扩展。
  void setParams(
    float step, float min_rad, float wheelbase, float max_steer,
    float d_theta, int max_iter, int lethal_cost)
  {
    step_size_ = step;
    min_turning_radius_ = min_rad;
    wheelbase_ = wheelbase;
    max_steering_angle_ = max_steer;
    d_theta_ = d_theta;
    theta_size_ = std::max(1, static_cast<int>(std::round(2.0 * M_PI / d_theta_)));
    max_iterations_ = max_iter;
    lethal_cost_threshold_ = lethal_cost;
  }

  // [功能与联系] 选择车辆自行车模型或无人船定速-限艏摇角速度模型；USV模型不需要轴距/方向舵角。
  bool setMotionModel(const std::string & name, double usv_max_yaw_rate)
  {
    std::string normalized = name;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char character) {return static_cast<char>(std::toupper(character));});
    if (normalized == "CAR" || normalized == "CAR_LIKE") {
      motion_model_ = MotionModel::CAR_LIKE;
      motion_model_name_ = "CAR_LIKE";
    } else if (normalized == "USV" || normalized == "USV_KINEMATIC") {
      motion_model_ = MotionModel::USV_KINEMATIC;
      motion_model_name_ = "USV_KINEMATIC";
    } else if (normalized == "USV_DYNAMIC" || normalized == "USV_6DOF") {
      motion_model_ = MotionModel::USV_DYNAMIC;
      motion_model_name_ = "USV_DYNAMIC";
    } else {
      return false;
    }
    usv_max_yaw_rate_ = std::max(0.01, std::abs(usv_max_yaw_rate));
    return true;
  }

  bool usesUsvKinematicModel() const
  {
    return motion_model_ == MotionModel::USV_KINEMATIC;
  }

  bool usesUsvDynamicModel() const
  {
    return motion_model_ == MotionModel::USV_DYNAMIC;
  }

  void setUsvDynamics(const hybrid_a_star_planner::dynamics::Parameters & parameters)
  {
    usv_dynamics_ = parameters;
    usv_max_yaw_rate_ = std::max(0.01, std::abs(parameters.max_yaw_rate));
    cruise_speed_ = std::max(0.05, parameters.cruise_speed);
  }

  const std::string & motionModelName() const {return motion_model_name_;}
  double maximumAllowedCurvature() const {return maximumCurvature();}

  // [功能与联系] 配置航速、半径、安全冗余及动态软代价；primitiveIsSafe和尾段检查按相同时间模型使用。
  void setDynamicParams(
    double cruise_speed, double own_radius, double safety_buffer,
    double prediction_horizon, double soft_distance_factor,
    double avoidance_radius, double dynamic_weight, double colregs_weight,
    double colregs_deadband, double overtaking_distance = 15.0)
  {
    cruise_speed_ = std::max(0.05, cruise_speed);
    own_ship_radius_ = std::max(0.0, own_radius);
    safety_buffer_ = std::max(0.0, safety_buffer);
    prediction_horizon_ = std::max(0.0, prediction_horizon);
    soft_distance_factor_ = std::max(1.0, soft_distance_factor);
    avoidance_radius_ = std::max(0.0, avoidance_radius);
    dynamic_weight_ = std::max(0.0, dynamic_weight);
    colregs_weight_ = std::max(0.0, colregs_weight);
    colregs_deadband_ = std::max(0.0, colregs_deadband);
    overtaking_safe_distance_ = std::max(0.0, overtaking_distance);
  }

  void setDynamicHardDistance(double distance)
  {
    dynamic_hard_distance_ = std::max(0.0, distance);
  }

  // [功能与联系] 缓存物理栅格尺寸/原点/分辨率并初始化查询状态；搜索必须已有地图。
  void setCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & costmap)
  {
    costmap_ = costmap;
    ++static_map_revision_;
    if (!costmap) {
      return;
    }
    width_ = static_cast<int>(costmap->info.width);
    height_ = static_cast<int>(costmap->info.height);
    resolution_ = costmap->info.resolution;
    origin_x_ = costmap->info.origin.position.x;
    origin_y_ = costmap->info.origin.position.y;
    origin_yaw_ = tf2::getYaw(costmap->info.origin.orientation);
  }

  // [功能与联系] 注入本周期目标船预测状态与策略；search、平滑检查及COLREG约束共同读取。
  void setDynamicObstacles(std::vector<DynamicObstacle> obstacles)
  {
    dynamic_obstacles_ = std::move(obstacles);
  }

  // [功能与联系] 配置分源静态安全与航道区域软权重；isLethal决定硬安全，navigationCostAt决定区域偏好。
  void setChannelParams(
    bool enabled, int shoreline_threshold, int channel_obstacle_threshold,
    int main_max, int opposite_min, double outside_weight,
    double opposite_weight, double main_center_weight, double outside_map_weight,
    double channel_obstacle_weight,
    double analytic_expansion_distance)
  {
    channel_enabled_ = enabled;
    shoreline_threshold_ = shoreline_threshold;
    channel_obstacle_threshold_ = channel_obstacle_threshold;
    if(main_lane_max_!=main_max) main_lane_distance_.reset(lane_costmap_,main_max);
    main_lane_max_ = main_max;
    opposite_lane_min_ = opposite_min;
    outside_lane_weight_ = std::max(0.0, outside_weight);
    opposite_lane_weight_ = std::max(0.0, opposite_weight);
    main_center_weight_ = std::max(0.0, main_center_weight);
    outside_map_weight_ = std::max(0.0, outside_map_weight);
    channel_obstacle_weight_ = std::max(0.0, channel_obstacle_weight);
    analytic_expansion_distance_ = std::max(0.0, analytic_expansion_distance);
  }

  // [功能与联系] 设置旧路径参考和舵角连续性软权重；只偏好稳定，不直接复用旧搜索路径。
  void setStabilityParams(
    double reference_path_weight, double reference_path_max_distance,
    double steering_change_weight, double steering_magnitude_weight = 0.0,
    double turn_reversal_weight = 24.0, double sideslip_weight = 6.0)
  {
    reference_path_weight_ = std::max(0.0, reference_path_weight);
    reference_path_max_distance_ = std::max(0.1, reference_path_max_distance);
    steering_change_weight_ = std::max(0.0, steering_change_weight);
    steering_magnitude_weight_ = std::max(0.0, steering_magnitude_weight);
    turn_reversal_weight_ = std::max(0.0, turn_reversal_weight);
    sideslip_weight_ = std::max(0.0, sideslip_weight);
  }

  // [功能与联系] 对任务航向变化较大的掉头机动降低“持续转舵”软代价，
  // 让搜索偏向船模允许的较紧转向；最大艏摇率、积分动力学和避碰硬约束不变。
  void setTurnaroundParams(double heading_threshold, double steering_scale)
  {
    turnaround_heading_threshold_ = std::clamp(heading_threshold, 0.0, M_PI);
    turnaround_steering_scale_ = std::clamp(steering_scale, 0.0, 1.0);
  }

  double steeringMagnitudeWeightForGoal(double start_yaw, double goal_yaw) const
  {
    const double difference = std::abs(hybrid_a_star_planner::normalizeAngle(goal_yaw - start_yaw));
    return usesUsvDynamicModel() && difference >= turnaround_heading_threshold_ ?
      steering_magnitude_weight_ * turnaround_steering_scale_ : steering_magnitude_weight_;
  }

  // [功能与联系] 为欠驱动积分增加低艏摇率候选，把缓弯分散到较长航段。
  // 同时保留原有大转向动作，primitiveIsSafe仍逐步检查碰撞和COLREG。
  void setGentleTurnSampling(bool enabled) {gentle_turn_sampling_ = enabled;}

  // [功能与联系] 在航道缓弯中累计实际地面轨迹曲率平方，而不只惩罚转向指令。
  // 含侧滑的course=艏向+atan2(v,u)，可识别撤舵后的曲率尖峰。仅是软代价：
  // 障碍/窄弯仍能使用原最小半径，掉头任务不施加该缓弯偏好。
  void setChannelCurveParams(double radius, double weight, double center_fraction = 0.6)
  {
    channel_curve_radius_ = std::max(0.0, radius);
    channel_curve_weight_ = std::max(0.0, weight);
    channel_curve_center_fraction_ = std::clamp(center_fraction, 0.0, 1.0);
  }
  // [功能与联系] 设置跨周期近场稳定代价：越靠近本船、偏离旧路径越大，
  // 非线性惩罚越强；它始终是软代价，primitiveIsSafe的障碍/COLREG硬约束优先。
  void setNearFieldStabilityParams(
    double weight, double horizon_distance, double deviation_scale)
  {
    near_field_reference_weight_ = std::max(0.0, weight);
    near_field_reference_horizon_ = std::max(0.1, horizon_distance);
    near_field_deviation_scale_ = std::max(0.1, deviation_scale);
  }

  // [功能与联系] 正常对遇搜索强制从右舷通过；仅当该约束导致完全无解时，
  // 节点可暂时关闭并按“安全优先于规则”执行一次受控回退搜索。
  void setHeadOnStarboardEnforced(bool enabled) {enforce_head_on_starboard_=enabled;}

  // [功能与联系] 提供上周期有效路径，建立近端参考距离；新目标时应清除，避免旧任务妨碍新任务。
  void setReferencePath(const std::vector<geometry_msgs::msg::Pose> & path)
  {
    bool changed = path.size() != reference_path_.size();
    if (!changed) {
      for (std::size_t i = 0; i < path.size(); ++i) {
        if (std::hypot(
            path[i].position.x - reference_path_[i].position.x,
            path[i].position.y - reference_path_[i].position.y) > 1.0e-4)
        {
          changed = true;
          break;
        }
      }
    }
    reference_path_ = path;
    reference_distance_dirty_ = reference_distance_dirty_ || changed;
    // 二维启发图包含navigationCostAt，其中含旧路径连续性代价；参考路径
    // 改变时必须同步失效缓存，不能用旧偏好引导新搜索。
    if (changed) {++static_map_revision_;}
  }

  // [功能与联系] 设置风险恢复软连续性和终点减速预测模型；动态安全恢复不依赖固定直航等待时间。
  void setRecoveryParams(double weight, double horizon, bool braking,
    double deceleration, double minimum_speed, double tolerance) {
    recovery_weight_=std::max(0.0,weight);recovery_horizon_=std::max(0.0,horizon);
    arrival_braking_=braking;arrival_deceleration_=std::max(0.01,deceleration);
    arrival_min_speed_=std::max(0.05,minimum_speed);
    arrival_goal_tolerance_=std::max(0.0,tolerance);
  }
  // [功能与联系] 返回恢复方向仍有动态风险的内部标记；用于测试验证软连续性门控。
  bool recoveryContinuityActive() const {return recovery_risk_;}
  // [功能与联系] 供回归测试确认旧路径进入目标软域时已释放稳定吸引；
  // 避免为保持连续性而拖延首次必要避让。
  bool referenceSoftConflictActive() const {return reference_soft_conflict_;}
  // [功能与联系] 返回search更新后的策略/目标快照；节点将恢复标志同步发布给MPC，避免两级约束不一致。
  const std::vector<DynamicObstacle> & dynamicObstacles() const {return dynamic_obstacles_;}
  std::size_t holonomicHeuristicBuildCount() const {return holonomic_build_count_;}
  // 搜索副本完成后仅回收与当前地图版本一致的启发缓存，不覆盖实时输入。
  void adoptSearchCache(HybridAStarPlanner & worker) {
    if (static_map_revision_ == worker.holonomic_cache_revision_) {
      holonomic_cost_ = std::move(worker.holonomic_cost_);
      holonomic_cache_revision_ = worker.holonomic_cache_revision_;
      holonomic_cache_goal_index_ = worker.holonomic_cache_goal_index_;
      holonomic_cache_compact_opposite_ = worker.holonomic_cache_compact_opposite_;
      holonomic_cache_main_entry_=worker.holonomic_cache_main_entry_;
      holonomic_cache_start_x_=worker.holonomic_cache_start_x_;
      holonomic_cache_start_y_=worker.holonomic_cache_start_y_;
    }
  }

  // [功能与联系] 轻量检查正在执行的剩余路径，而不是重新运行Hybrid A*。
  // 只有路径被静态障碍阻断、预测进入动态碰撞触发域、违反当前COLREG动作，
  // 或本船已经明显偏离路径时才请求事件重规划。
  bool remainingPathNeedsReplan(
    const std::vector<geometry_msgs::msg::Pose> & path,
    double own_x, double own_y, double own_heading,
    double collision_trigger_distance, double cross_track_limit,
    bool check_colregs = true) const
  {
    if (path.size() < 2U) {return true;}
    std::size_t nearest = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double distance = std::hypot(
        path[i].position.x - own_x, path[i].position.y - own_y);
      if (distance < nearest_distance) {nearest_distance = distance; nearest = i;}
    }
    if (nearest_distance > std::max(0.1, cross_track_limit)) {return true;}

    double elapsed = 0.0;
    double previous_x = own_x;
    double previous_y = own_y;
    const std::size_t first = std::min(nearest + 1U, path.size() - 1U);
    for (std::size_t i = first; i < path.size(); ++i) {
      const double end_x = path[i].position.x;
      const double end_y = path[i].position.y;
      const double length = std::hypot(end_x - previous_x, end_y - previous_y);
      if (length < 1.0e-8) {continue;}
      const double heading = std::atan2(end_y - previous_y, end_x - previous_x);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(length / std::max(0.25F, 0.5F * resolution_))));
      double segment_x = previous_x;
      double segment_y = previous_y;
      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        const double x = previous_x + ratio * (end_x - previous_x);
        const double y = previous_y + ratio * (end_y - previous_y);
        if (isLethal(static_cast<float>(x), static_cast<float>(y))) {return true;}
        elapsed += estimateSegmentTime(
          segment_x, segment_y, x, y, length / samples);
        segment_x = x;
        segment_y = y;
        for (const auto & obstacle : dynamic_obstacles_) {
          if (check_colregs && !hybrid_a_star_planner::headingAllowed(
              obstacle.policy, heading, elapsed, 0.0, own_heading))
          {
            return true;
          }
          const double target_x = obstacle.state.x + obstacle.state.vx * elapsed;
          const double target_y = obstacle.state.y + obstacle.state.vy * elapsed;
          const double hard = std::max({
            own_ship_radius_ + obstacle.state.radius + safety_buffer_, dynamic_hard_distance_,
            obstacle.policy.type == EncounterType::OVERTAKING ?
              overtaking_safe_distance_ : 0.0});
          if (std::hypot(x - target_x, y - target_y) <
            std::max(hard, collision_trigger_distance))
          {
            return true;
          }
          if (check_colregs && obstacle.policy.locked && !obstacle.policy.recovering &&
            obstacle.policy.type == EncounterType::HEAD_ON)
          {
            const double dx = target_x - x;
            const double dy = target_y - y;
            const double forward = dx * std::cos(obstacle.policy.reference) +
              dy * std::sin(obstacle.policy.reference);
            const double lateral = -std::sin(obstacle.policy.reference) * (x - target_x) +
              std::cos(obstacle.policy.reference) * (y - target_y);
            if (forward >= -hard &&
              (lateral > 0.25 || hybrid_a_star_planner::normalizeAngle(
                heading - obstacle.policy.reference) > 2.0 * hybrid_a_star_planner::deg))
            {
              return true;
            }
          }
          if (check_colregs && obstacle.policy.locked &&
            obstacle.policy.type == EncounterType::CROSSING_STARBOARD)
          {
            const double speed = std::hypot(obstacle.state.vx, obstacle.state.vy);
            if (speed > 0.05) {
              const double along = ((x - target_x) * obstacle.state.vx +
                (y - target_y) * obstacle.state.vy) / speed;
              const double across = ((x - target_x) * obstacle.state.vy -
                (y - target_y) * obstacle.state.vx) / speed;
              if (std::abs(across) < hard && along > -hard) {return true;}
            }
          }
        }
      }
      previous_x = end_x;
      previous_y = end_y;
    }
    return false;
  }
  // 与MPC终点速度上限一致的名义到达时间；最低预测速度避免终点除零。
  // 这是预测模型，不是强制本船速度，也不构成实船动力学保证。
  // [功能与联系] 由段长及巡航/终点制动模型估计耗时；运动基元、尾段和平滑复查使用相同时间基准。
  double estimateSegmentTime(double ax,double ay,double bx,double by,double length) const {
    double speed=planningSpeed();
    if(arrival_braking_) {
      const double remaining=std::max(0.0,
        std::hypot((ax+bx)*0.5-search_goal_x_,(ay+by)*0.5-search_goal_y_)-arrival_goal_tolerance_);
      speed=std::min(speed,std::max(arrival_min_speed_,
        std::min(0.5*remaining,std::sqrt(2*arrival_deceleration_*remaining))));
    }
    return length/std::max(0.05,speed);
  }

  // [功能与联系] 注入动态航道语义；区域偏好不应与物理硬障碍数据混合。
  void setLaneCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    lane_costmap_ = map;
    main_lane_distance_.reset(map,main_lane_max_);
    ++static_map_revision_;
  }

  // [功能与联系] 注入原始陆地核心层；航道内清除岸边膨胀也不能覆盖真实陆地。
  void setStaticLandMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    static_land_map_ = map;
    ++static_map_revision_;
  }

  // [功能与联系] 注入独立障碍膨胀层；航道自由水域规则不能消除这一来源的障碍。
  void setChannelObstacleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    channel_obstacle_map_ = map;
    ++static_map_revision_;
  }

  // [功能与联系] 优先检查陆地和独立障碍；已识别航道只忽略岸边膨胀，航道外按未知/岸边阈值拒绝。
  bool isLethal(float x, float y) const
  {
    if (!costmap_) {
      return true;
    }
    int gx = 0;
    int gy = 0;
    if (!worldToMap(x, y, gx, gy)) {
      return true;
    }
    const int cost = costmap_->data[static_cast<std::size_t>(gy * width_ + gx)];
    // Preserve UNKNOWN, raw obstacles and the minimum hard clearance even
    // inside a chart lane; only SOFT shoreline inflation can be ignored.
    if (cost < 0 || cost >= 99) {return true;}
    // 原始陆地与独立的航道内障碍物永远不能被航道规则覆盖。
    const int land = costAt(static_land_map_, x, y);
    if (land >= 100) {
      return true;
    }
    const int channel_obstacle = costAt(channel_obstacle_map_, x, y);
    if (channel_obstacle >= channel_obstacle_threshold_) {
      return true;
    }
    const int region = laneRegionAt(x, y);
    if (channel_enabled_ && region != Region::UNKNOWN && region != Region::OUTSIDE) {
      // TSSLPT 内只忽略岸边膨胀；航道障碍物已在上面单独检查。
      return false;
    }
    return cost < 0 || cost >= shoreline_threshold_;
  }

  // [功能与联系] 核验船位在地图内且不是硬危险格；search启动前调用。
  bool isStartValid(float x, float y) const
  {
    if (!costmap_) {
      return false;
    }
    int gx = 0;
    int gy = 0;
    if (!worldToMap(x, y, gx, gy)) {
      return false;
    }
    return !isLethal(x, y);
  }

  // [功能与联系] 执行一次时空Hybrid A*搜索：探测恢复方向、构建启发与参考场、扩展运动基元/近端Dubins连接并回溯；由事件调度器按需调用。
  // One total budget, descending speed stages. A feasible faster stage wins;
  // retain measured u/v/r in every retry, including deceleration into a turn.
  std::vector<geometry_msgs::msg::Pose> search(
    float sx,float sy,float stheta,float gx,float gy,float gtheta,
    double initial_time=0,double start_u=0,double start_v=0,double start_r=0)
  {
    const auto begin=std::chrono::steady_clock::now();
    const double configured_budget=search_time_limit_;
    const double remaining=std::chrono::duration<double>(overall_deadline_-begin).count();
    if(remaining<=0) return {};
    const double budget=configured_budget>0 ? std::min(configured_budget,remaining) : 0;
    const auto speeds=usesUsvDynamicModel() ? speedSamples() : std::vector<double>{cruise_speed_};
    std::vector<geometry_msgs::msg::Pose> result;
    attempted_speeds_.clear();
    rescue_selected_=false;last_shooting_trace_.clear();bool rescue_attempted=false;
    heuristic_attempted_=false;heuristic_ready_=false;
    // Compact maneuver costs belong to this mission, not to a global lane role.
    // Do not reuse a heuristic built with another mission's soft penalties.
    const double chord=std::hypot(gx-sx,gy-sy);
    const bool compact_opposite=channel_enabled_ && chord<=compact_guidance_distance_ &&
      laneRegionAt(sx,sy)==Region::OPPOSITE && laneRegionAt(gx,gy)==Region::OPPOSITE;
    compact_opposite_maneuver_=compact_opposite;
    const double lateral=-(gx-sx)*std::sin(stheta)+(gy-sy)*std::cos(stheta);
    const double forward=(gx-sx)*std::cos(stheta)+(gy-sy)*std::sin(stheta);
    const Region start_lane=laneRegionAt(sx,sy),goal_lane=laneRegionAt(gx,gy);
    lane_turn_direction_=lane_turn_weight_>0 && forward<0 && std::abs(lateral)>1 &&
      ((start_lane==Region::MAIN && goal_lane==Region::OPPOSITE) ||
       (start_lane==Region::OPPOSITE && goal_lane==Region::MAIN)) &&
      std::abs(hybrid_a_star_planner::normalizeAngle(gtheta-stheta))>2.0 ?
      (lateral>0 ? 1:-1):0;
    // The main lane is a task role, not a fixed left/right side. Resolve its
    // nearest local direction even for forward/sideways goals and small yaw gaps.
    main_entry_active_=mainLaneEntryApplicable(sx,sy,gx,gy);
    if(main_entry_active_ && start_lane!=Region::MAIN) {
      const auto direction=main_lane_distance_.direction(sx,sy);
      const double toward=-direction.first*std::sin(stheta)+direction.second*std::cos(stheta);
      if(std::abs(toward)>1.0) lane_turn_direction_=toward>0 ? 1:-1;
    }
    const double requested_turn=hybrid_a_star_planner::normalizeAngle(gtheta-stheta);
    const double bearing_turn=hybrid_a_star_planner::normalizeAngle(std::atan2(gy-sy,gx-sx)-stheta);
    special_maneuver_=usesUsvDynamicModel() && (std::abs(requested_turn)>.7 || std::abs(bearing_turn)>.8);
    // Retain a safe opposite-turn candidate as a fallback while trying lower
    // speeds toward the adjacent target lane. This preference never removes
    // collision-free alternatives from the maneuver search.
    std::vector<geometry_msgs::msg::Pose> alternate_turn;
    double alternate_speed=0,alternate_radius=0,alternate_score=std::numeric_limits<double>::infinity();
    bool alternate_c=false;
    const bool lane_first=usesUsvDynamicModel() && laneRegionAt(sx,sy)==Region::MAIN &&
      laneRegionAt(gx,gy)==Region::MAIN && !special_maneuver_;
    // Ordinary navigation first searches inside the main lane. Borrowing remains
    // available when that corridor has no feasible continuation, or for a U-turn.
    for(int phase=0;phase<(lane_first ? 2:1) && result.empty();++phase) {
      strict_main_lane_=lane_first && phase==0;
      const double phase_end=budget*(strict_main_lane_ ? .65:1.0);
      for(std::size_t i=0;i<speeds.size();++i) {
        if(search_cancel_ && search_cancel_->load()) break;
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        if(budget>0 && elapsed>=phase_end) break;
        search_time_limit_=budget>0 ? (phase_end-elapsed)/(speeds.size()-i) : 0;
        // 非最低速快速顺序尝试；最低速保留剩余预算完成困难搜索。
        if(budget>0 && i+1<speeds.size()) {
          search_time_limit_=std::min(search_time_limit_,speed_stage_time_limit_);
        }
        requested_speed_=speeds[i];
        if(rescue_enabled_ && !rescue_attempted && usesUsvDynamicModel() &&
          i+1==speeds.size() && chord<=40.0 && chord>=1.0) {
          rescue_attempted=true;
          // This call is in search()'s dispatcher, NOT in searchAtSpeed().
          // The single-speed routine never calls this dispatcher or rescue.
          // Preserve its fast legal C-turn before the multi-phase fallback.
          attempted_speeds_.push_back(requested_speed_);
          const double original_limit=search_time_limit_;
          search_time_limit_=std::min(.12,original_limit>0?original_limit:.12);
          result=searchAtSpeed(sx,sy,stheta,gx,gy,gtheta,initial_time,start_u,start_v,start_r);
          if(!result.empty())break;
          const double after_preflight=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
          const double available=budget>0?phase_end-after_preflight:rescue_time_;
          if(available>.04) {
            search_time_offset_=initial_time;search_heading_=stheta;
            search_start_x_=sx;search_start_y_=sy;search_goal_x_=gx;search_goal_y_=gy;
            search_deadline_=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(std::min(rescue_time_,available*.8)));
            result=rescueShooting(sx,sy,stheta,gx,gy,gtheta,initial_time,start_u,start_v,start_r);
            if(!result.empty())break;
          }
          // Restore the remaining stage budget, not the pre-fallback budget.
          const double used=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
          search_time_limit_=budget>0?std::max(.001,phase_end-used):0;
        }
        attempted_speeds_.push_back(requested_speed_);
        const bool bounded_attempt=search_time_limit_>0;
        const auto attempt_deadline=std::chrono::steady_clock::now()+
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(search_time_limit_));
        // 终点艏向是容差区间，不只是严格朝向的一条Dubins尾段。
        // 各偏置仍属于用户原方向的容差，并逐一执行同一船模和安全检查。
        const double preferred_offset=main_entry_active_ ?
          mainLaneHeadingOffsetPreference(requested_turn,lane_turn_direction_) :
          (lane_turn_direction_==0 ? -1.0 : (requested_turn*lane_turn_direction_>0 ? -1.0:1.0));
        for(double offset:{0.0,preferred_offset*.5,-preferred_offset*.5,preferred_offset*.95,-preferred_offset*.95}) {
          if(offset!=0 && (!special_maneuver_ || goal_heading_tolerance_<=.23)) {continue;}
          if(bounded_attempt) {
            const double available=std::chrono::duration<double>(attempt_deadline-std::chrono::steady_clock::now()).count();
            if(available<=0) {break;}
            search_time_limit_=available;
          }
          result=searchAtSpeed(sx,sy,stheta,gx,gy,
            hybrid_a_star_planner::normalizeAngle(gtheta+offset*goal_heading_tolerance_),
            initial_time,start_u,start_v,start_r);
          if(!result.empty()) {
            if(std::abs(hybrid_a_star_planner::normalizeAngle(tf2::getYaw(result.back().orientation)-gtheta))>
              goal_heading_tolerance_) {result.clear();continue;}
            break;
          }
          if(searchTimeExpired()) {break;}
        }
        if(!result.empty() && special_maneuver_) {
          double length=0,straight=0;bool turning=false;
          for(std::size_t j=1;j<result.size();++j) {
            const double ds=std::hypot(result[j].position.x-result[j-1].position.x,
              result[j].position.y-result[j-1].position.y);length+=ds;
            turning=turning || std::abs(hybrid_a_star_planner::normalizeAngle(
              tf2::getYaw(result[j].orientation)-stheta))>.04;
            if(!turning) {straight+=ds;}
          }
          // 首个高速解若是长直段/长绕圈，不立即接受：继续降速缩转向尺度。
          const double chord=std::hypot(gx-sx,gy-sy);
          if(i+1<speeds.size() && (straight>std::max(maneuver_straight_limit_,.2*chord) ||
              length>std::max(chord,planningRadius()*std::abs(requested_turn))*maneuver_detour_ratio_+10)) {
            result.clear();
          }
        }
        if(!result.empty() && lane_turn_direction_!=0) {
          double first_turn=0;
          for(const auto &pose:result) {
            const double turn=hybrid_a_star_planner::normalizeAngle(tf2::getYaw(pose.orientation)-stheta);
            if(std::abs(turn)>.08) {first_turn=turn;break;}
          }
          if(first_turn*lane_turn_direction_<0) {
            double score=lane_turn_weight_;
            for(std::size_t j=1;j<result.size();++j) {
              score+=std::hypot(result[j].position.x-result[j-1].position.x,
                result[j].position.y-result[j-1].position.y);
            }
            if(score<alternate_score) {
              alternate_score=score;alternate_turn=std::move(result);
              alternate_speed=planningSpeed();alternate_radius=planningRadius();alternate_c=c_turn_selected_;
            }
            result.clear();
          }
        }
        if(!result.empty()) break;
      }
    }
    double preferred_length=0;
    for(std::size_t j=1;j<result.size();++j) {
      preferred_length+=std::hypot(result[j].position.x-result[j-1].position.x,
        result[j].position.y-result[j-1].position.y);
    }
    if(!alternate_turn.empty() && (result.empty() || alternate_score<preferred_length)) {
      result=std::move(alternate_turn);active_turn_speed_=alternate_speed;active_turn_radius_=alternate_radius;
      maneuver_active_=alternate_speed<cruise_speed_-1e-6;tight_maneuver_=alternate_speed<=close_turn_speed_+1e-6;
      c_turn_selected_=alternate_c;rescue_selected_=false;last_shooting_trace_.clear();
    }
    // A new entry guide can miss a previously feasible word under the finite
    // stage deadline. Re-run the original search as a bounded competitor when
    // the chosen route spends too much of its first leg away from the main lane.
    // Both candidates use the same measured state, maps, safety and total budget.
    if(main_entry_active_ && (result.empty() || mainLaneEntryCost(result)>main_entry_weight_*.45) &&
      !(search_cancel_ && search_cancel_->load())) {
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
      const double available=budget>0 ? budget-elapsed:0.3;
      if(available>.02) {
        auto competitor=*this;
        competitor.setMainLaneEntryParams(0,main_entry_direct_distance_,main_entry_horizon_);
        competitor.setSearchTimeLimit(available);
        auto original=competitor.search(sx,sy,stheta,gx,gy,gtheta,initial_time,start_u,start_v,start_r);
        auto score=[&](const auto &path) {
          return mainLaneEntryCost(path)+.25*groundPathQuality(path).length;
        };
        attempted_speeds_.insert(attempted_speeds_.end(),competitor.attempted_speeds_.begin(),competitor.attempted_speeds_.end());
        if(!original.empty() && (result.empty() || score(original)<score(result))) {
          result=std::move(original);active_turn_speed_=competitor.planningSpeed();
          active_turn_radius_=competitor.planningRadius();maneuver_active_=competitor.maneuver_active_;
          tight_maneuver_=competitor.tight_maneuver_;c_turn_selected_=competitor.c_turn_selected_;
          rescue_selected_=competitor.rescue_selected_;last_shooting_trace_=competitor.last_shooting_trace_;
          last_shooting_control_=competitor.last_shooting_control_;
        }
      }
    }
    strict_main_lane_=false;
    search_time_limit_=configured_budget;
    std::ostringstream stage_summary;
    for(double speed:attempted_speeds_) {if(stage_summary.tellp()>0) stage_summary<<",";stage_summary<<speed;}
    if(result.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("HybridAStar"),
        "Speed stages [%s]: no accepted path, budget %.2f s, measured u/v/r %.3f/%.3f/%.3f",
        stage_summary.str().c_str(), configured_budget, start_u, start_v, start_r);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("HybridAStar"),"Speed stages [%s]: success, selected %.2f m/s",
        stage_summary.str().c_str(),planningSpeed());
    }
    return result;
  }

  std::vector<geometry_msgs::msg::Pose> searchAtSpeed(
    float sx, float sy, float stheta, float gx, float gy, float gtheta,
    double initial_time = 0.0, double start_u = 0.0, double start_v = 0.0,
    double start_r = 0.0)
  {
    initial_time = std::max(0.0, initial_time);
    maneuver_active_=false;
    tight_maneuver_=false;
    active_turn_speed_=requested_speed_;
    const double fraction=std::clamp((requested_speed_-close_turn_speed_)/
      std::max(0.01,cruise_speed_-close_turn_speed_),0.0,1.0);
    active_turn_radius_=close_turn_radius_+fraction*(min_turning_radius_-close_turn_radius_);
    maneuver_active_=requested_speed_<cruise_speed_-1e-6;
    tight_maneuver_=requested_speed_<=close_turn_speed_+1e-6;
    c_turn_selected_=false;
    search_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(search_time_limit_));
    search_time_offset_ = initial_time;
    search_heading_=stheta;
    search_start_x_=sx;search_start_y_=sy;search_goal_x_=gx;search_goal_y_=gy;
    prefer_broad_channel_curve_ = channel_enabled_ && lane_costmap_ && !special_maneuver_ &&
      std::abs(hybrid_a_star_planner::normalizeAngle(gtheta - stheta)) <
      turnaround_heading_threshold_;
    // 当前CPA变安全不等于可以安全恢复。按恢复方向的时间同步净空
    // 决定是否加连续性软代价；每次事件搜索重算，不靠固定等待锁航向。
    recovery_risk_=false;
    const double recovery_length=std::hypot(gx-sx,gy-sy);
    const int recovery_samples=std::max(1,static_cast<int>(std::ceil(recovery_length)));
    double recovery_time=initial_time,px=sx,py=sy;
    for(int i=1;i<=recovery_samples && !recovery_risk_;++i) {
      const double f=static_cast<double>(i)/recovery_samples;
      const double x=sx+(gx-sx)*f,y=sy+(gy-sy)*f;
      recovery_time+=estimateSegmentTime(px,py,x,y,recovery_length/recovery_samples);
      for(const auto &o:dynamic_obstacles_) {
        // 不依赖会遇是否已解锁：恢复方向仍有实际动态风险时就保持软连续性。
        const double safe=std::max({own_ship_radius_+o.state.radius+safety_buffer_, dynamic_hard_distance_,
          o.policy.type==EncounterType::OVERTAKING ? overtaking_safe_distance_:0.0});
        if(std::hypot(x-o.state.x-o.state.vx*recovery_time,
          y-o.state.y-o.state.vy*recovery_time)<safe) {recovery_risk_=true;break;}
      }
      px=x;py=y;
    }
    recovery_complete_=false;
    for(auto &o:dynamic_obstacles_) {
      const double safe=std::max({own_ship_radius_+o.state.radius+safety_buffer_, dynamic_hard_distance_,
        o.policy.type==EncounterType::OVERTAKING ? overtaking_safe_distance_:0.0});
      auto future_target=o.state;
      future_target.x+=future_target.vx*initial_time;
      future_target.y+=future_target.vy*initial_time;
      auto future_policy=o.policy;
      future_policy.metric=hybrid_a_star_planner::cpa(
        sx,sy,future_policy.speed*std::cos(future_policy.reference),
        future_policy.speed*std::sin(future_policy.reference),future_target);
      o.policy.recovering=future_policy.safeToRecover(
        sx,sy,future_target,safe,recovery_risk_);
      recovery_complete_=recovery_complete_ || o.policy.recovering;
    }
    // 上一周期路径若已被当前目标预测软域占据，就不能继续用强稳定代价
    // 把新搜索吸回旧路径。先释放吸引完成必要改道；新的安全路径发布后，
    // 下一周期又会成为参考，从而稳定已经选定的避让侧。
    reference_soft_conflict_=false;
    if(!reference_path_.empty() && !dynamic_obstacles_.empty()) {
      std::size_t nearest=0;
      double nearest_distance=std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<reference_path_.size();++i) {
        const double d=std::hypot(reference_path_[i].position.x-sx,
          reference_path_[i].position.y-sy);
        if(d<nearest_distance) {nearest_distance=d;nearest=i;}
      }
      double reference_time=initial_time,reference_length=0.0,rx=sx,ry=sy;
      for(std::size_t i=nearest;i<reference_path_.size() && !reference_soft_conflict_;++i) {
        const double x=reference_path_[i].position.x,y=reference_path_[i].position.y;
        const double length=std::hypot(x-rx,y-ry);
        reference_length+=length;
        if(reference_length>near_field_reference_horizon_) break;
        reference_time+=estimateSegmentTime(rx,ry,x,y,length);rx=x;ry=y;
        for(const auto &o:dynamic_obstacles_) {
          if(!o.policy.locked && !o.encounter.active) continue;
          const double hard=std::max({own_ship_radius_+o.state.radius+safety_buffer_, dynamic_hard_distance_,
            o.policy.type==EncounterType::OVERTAKING ? overtaking_safe_distance_:0.0});
          const double soft=std::max({hard,avoidance_radius_,
            (own_ship_radius_+o.state.radius+safety_buffer_)*soft_distance_factor_});
          const double tx=o.state.x+o.state.vx*reference_time;
          const double ty=o.state.y+o.state.vy*reference_time;
          if(std::hypot(x-tx,y-ty)<soft) {reference_soft_conflict_=true;break;}
        }
      }
    }
    buildReferenceDistance();
    if (width_ == 0 || height_ == 0 || !isStartValid(sx, sy)) {
      RCLCPP_WARN(rclcpp::get_logger("HybridAStar"), "Start is outside the navigable map");
      return {};
    }
    if (isLethal(gx, gy)) {return {};}
    const double heading_difference=std::abs(hybrid_a_star_planner::normalizeAngle(gtheta-stheta));
    close_turn_search_=large_turn_enabled_ && std::hypot(gx-sx,gy-sy)<=
      std::max(close_turn_range_,compact_guidance_distance_) &&
      heading_difference>=close_turn_angle_;
    large_turn_search_=large_turn_enabled_ && (heading_difference>=large_turn_angle_ || close_turn_search_);
    // An aligned outbound leg should not expand a large lattice merely to
    // accelerate from low speed. Integrate the same six-state model and safety
    // sampler; do not snap points to a line or reset measured velocity.
    const double chord=std::hypot(gx-sx,gy-sy);
    const double bearing=std::atan2(gy-sy,gx-sx);
    if(usesUsvDynamicModel() && dynamic_obstacles_.empty() &&
      (!channel_enabled_ || strict_main_lane_ ||
       (laneRegionAt(sx,sy)==Region::OPPOSITE && laneRegionAt(gx,gy)==Region::OPPOSITE)) && heading_difference<.04 &&
      std::abs(hybrid_a_star_planner::normalizeAngle(bearing-stheta))<.04 &&
      std::abs((gx-sx)*std::sin(stheta)-(gy-sy)*std::cos(stheta))<.4) {
      auto current=std::make_shared<Node3D>();
      current->x=sx;current->y=sy;current->theta=stheta;current->t=initial_time;
      current->u=start_u;current->v=start_v;current->r=start_r;
      const int steps=std::clamp(static_cast<int>(chord/std::max(.05,planningSpeed()*.1))+100,100,20000);
      for(int i=0;i<steps && !searchTimeExpired();++i) {
        const double distance=std::hypot(current->x-gx,current->y-gy);
        if(distance<.4 && std::abs(hybrid_a_star_planner::normalizeAngle(current->theta-gtheta))<goal_heading_tolerance_)
          return extractPath(current);
        if(std::abs(hybrid_a_star_planner::normalizeAngle(
            std::atan2(gy-current->y,gx-current->x)-current->theta))>.2) break;
        Node3D propagated;double cost=0,arrival=current->t,length=0;
        const double dt=std::min(.1,std::max(.01,(distance-.2)/std::max(.1,std::hypot(current->u,current->v))));
        if(!primitiveIsSafe(*current,0,cost,arrival,propagated,length,false,-1,dt)) break;
        auto next=std::make_shared<Node3D>(propagated);next->parent=current;current=next;
      }
    }
    if(usesUsvDynamicModel() && main_entry_active_ && lane_turn_direction_ &&
      laneRegionAt(sx,sy)!=Region::MAIN) {
      Node3D seed;seed.x=sx;seed.y=sy;seed.theta=stheta;seed.t=initial_time;
      seed.u=start_u;seed.v=start_v;seed.r=start_r;
      const auto saved_deadline=search_deadline_;
      search_deadline_=std::min(saved_deadline,
        std::chrono::steady_clock::now()+std::chrono::milliseconds(30));
      auto entry=refineBroadChannelCurve(seed,gx,gy,gtheta,
        std::numeric_limits<double>::infinity(),true,false,true);
      search_deadline_=saved_deadline;
      if(!entry.empty()) return entry;
    }
    // 大航向差先用Dubins作为空间引导，输出仍逐步积分六状态船模，而非直接发布几何曲线。
    // 常速失败才降速缩半径；不改变实测初始u/v/r，不放宽碰撞或COLREG硬检查。
    if (large_turn_enabled_ && usesUsvDynamicModel() && large_turn_search_) {
      Node3D seed;seed.x=sx;seed.y=sy;seed.theta=stheta;seed.t=initial_time;
      seed.u=std::clamp(start_u,0.0,usv_dynamics_.cruise_speed);seed.v=start_v;
      seed.r=std::clamp(start_r,-usv_max_yaw_rate_,usv_max_yaw_rate_);
      if(prefer_single_turn_) {
        double fraction=requested_speed_>=large_turn_speed_ ?
          (requested_speed_-large_turn_speed_)/std::max(.01,cruise_speed_-large_turn_speed_) :
          (requested_speed_-close_turn_speed_)/std::max(.01,large_turn_speed_-close_turn_speed_);
        active_turn_radius_=requested_speed_>=large_turn_speed_ ?
          large_turn_radius_+fraction*(min_turning_radius_-large_turn_radius_) :
          close_turn_radius_+fraction*(large_turn_radius_-close_turn_radius_);
        active_turn_radius_=std::clamp(active_turn_radius_,3.0,static_cast<double>(min_turning_radius_));
        auto guided=connectMonotoneTurn(seed,gx,gy,gtheta);
        if(!guided.empty()) {c_turn_selected_=true;return guided;}
        // A shooting failure does not prove this speed infeasible: a Dubins-
        // guided rollout may still produce a single turn (especially near 90deg).
        // Bound this secondary connector so ten failed stages cannot stall MPC.
        {
          const auto saved_deadline=search_deadline_;
          if(search_time_limit_>0) search_deadline_=std::min(saved_deadline,
            std::chrono::steady_clock::now()+std::chrono::milliseconds(20));
          auto alternative=refineBroadChannelCurve(seed,gx,gy,gtheta,
            std::numeric_limits<double>::infinity(),true);
          search_deadline_=saved_deadline;
          double left=0,right=0,previous_course=0,yaw_left=0,yaw_right=0;
          for(std::size_t i=1;i<alternative.size();++i) {
            const double course=std::atan2(alternative[i].position.y-alternative[i-1].position.y,
              alternative[i].position.x-alternative[i-1].position.x);
            if(i>1) {const double d=hybrid_a_star_planner::normalizeAngle(course-previous_course);
              left+=std::max(0.0,d);right+=std::max(0.0,-d);}
            previous_course=course;
            const double yaw_change=hybrid_a_star_planner::normalizeAngle(
              tf2::getYaw(alternative[i].orientation)-tf2::getYaw(alternative[i-1].orientation));
            yaw_left+=std::max(0.0,yaw_change);yaw_right+=std::max(0.0,-yaw_change);
          }
          const double lateral=-(gx-sx)*std::sin(stheta)+(gy-sy)*std::cos(stheta);
          const double desired_turn=hybrid_a_star_planner::normalizeAngle(gtheta-stheta);
          const double forward=(gx-sx)*std::cos(stheta)+(gy-sy)*std::sin(stheta);
          const bool necessary_reverse=forward>0 && desired_turn*lateral<=0 &&
            std::abs(desired_turn)>.7 && std::abs(desired_turn)<2.5;
          const bool single_turn=std::min(left,right)<.04 && std::min(yaw_left,yaw_right)<.02;
          if(!alternative.empty() && (single_turn || necessary_reverse)) {
            c_turn_selected_=single_turn;return alternative;
          }
        }
        if(requested_speed_>close_turn_speed_+1e-6) return {};
      }
      auto guided=refineBroadChannelCurve(seed,gx,gy,gtheta,
        std::numeric_limits<double>::infinity(),true);
      if(!guided.empty()) return guided;
    }
    // 航道普通缓弯首先试16组连续引导，再用同一六状态船模积分与逐步安全
    // 检查。成功就不扩展海量离散节点；失败才进入原Hybrid A*，避障能力保留。
    if (fast_guided_connection_ && usesUsvDynamicModel() && prefer_broad_channel_curve_ &&
      gentle_turn_sampling_ && channel_curve_weight_ > 0.0 &&
      laneRegionAt(sx,sy)==Region::MAIN && laneRegionAt(gx,gy)==Region::MAIN) {
      Node3D seed;
      seed.x=sx; seed.y=sy; seed.theta=stheta; seed.t=initial_time;
      seed.u=std::clamp(start_u, 0.0, cruise_speed_); seed.v=start_v;
      seed.r=std::clamp(start_r, -usv_max_yaw_rate_, usv_max_yaw_rate_);
      seed.turn_direction=seed.r>0.02 ? 1 : (seed.r<-0.02 ? -1 : 0);
      auto guided=refineBroadChannelCurve(seed,gx,gy,gtheta,std::numeric_limits<double>::infinity());
      if (!guided.empty()) {return guided;}
      // 中央窄带是质量偏好，不是障碍：窄带连接失败后，先在完整主航道
      // 中尝试同一船模的宽弧，而不是直接转入海量格点。区域/碰撞硬检查保留。
      guided=refineBroadChannelCurve(seed,gx,gy,gtheta,
        std::numeric_limits<double>::infinity(),false,true);
      if (!guided.empty()) {return guided;}
    }
    // 先保留全程宽弧优化；失败才试快速C弯，避免把平缓航道连接变成局部急弯。
    // 主航道内的航点交接也允许该候选，strict_main_lane_仍逐步验安全。
    if(prefer_single_turn_ && usesUsvDynamicModel() && !large_turn_search_ && heading_difference>.04 &&
      (laneRegionAt(sx,sy)!=Region::MAIN || fast_guided_connection_)) {
      Node3D seed;seed.x=sx;seed.y=sy;seed.theta=stheta;seed.t=initial_time;
      seed.u=start_u;seed.v=start_v;seed.r=start_r;
      auto guided=connectMonotoneTurn(seed,gx,gy,gtheta);
      if(!guided.empty()) {c_turn_selected_=true;return guided;}
    }
    // 短距离同向姿态调整也可能需要一个S弯，不能只凭起终艏向差
    // 判断是否需要连接。主航道已有宽弧分支；开阔水域和对向航道
    // 使用同样的船模连接，保留实测侧滑/艏摇率及所有硬安全检查。
    if (fast_guided_connection_ && usesUsvDynamicModel() && !large_turn_search_ &&
      !special_maneuver_ && chord >= 10.0 && chord <= compact_guidance_distance_ &&
      laneRegionAt(sx, sy) != Region::MAIN)
    {
      Node3D seed;
      seed.x = sx; seed.y = sy; seed.theta = stheta; seed.t = initial_time;
      seed.u = start_u; seed.v = start_v; seed.r = start_r;
      auto guided = refineBroadChannelCurve(
        seed, gx, gy, gtheta, std::numeric_limits<double>::infinity());
      if (!guided.empty()) {return guided;}
    }
    // 全图二维场在同次任务内与参考速度无关。构建受当前阶段预算限制，
    // 不能越过阶段截止时间耗掉所有低速尝试；单次search最多构建一次。
    // 未完成时丢弃半成品并使用几何引导；完整缓存仍跨速度/周期复用。
    // 这里只调整启发式，所有实际扩展仍逐子步执行硬安全检查。
    if(!heuristic_attempted_) {
      heuristic_attempted_=true;
      heuristic_ready_=buildHolonomicHeuristic(gx,gy);
      if(!heuristic_ready_) {holonomic_cost_.clear();}
    }
    if(searchTimeExpired()) {return {};}
    if(heuristic_ready_) {
      int start_grid_x=0,start_grid_y=0;
      worldToMap(sx,sy,start_grid_x,start_grid_y);
      if(!std::isfinite(holonomic_cost_[start_grid_y*width_+start_grid_x])) {
        RCLCPP_WARN(rclcpp::get_logger("HybridAStar"),"No static-water connection to the goal");
        return {};
      }
    }

    const std::size_t total_states = static_cast<std::size_t>(width_) *
      static_cast<std::size_t>(height_) * static_cast<std::size_t>(theta_size_);
    std::unordered_map<std::uint64_t,float> g_values;
    // 没有动态船时，时间不会改变碰撞结果。若仍把时间放入状态键，同一
    // (x,y,yaw) 会以不同到达时刻反复进入 open set，远目标很容易耗尽迭代数。
    // 只有动态避碰真正需要时间同步时才扩展时间维度。
    const bool use_time_dimension = std::any_of(
      dynamic_obstacles_.begin(), dynamic_obstacles_.end(), [](const auto & obstacle) {
        return obstacle.policy.locked || obstacle.encounter.active;
      });
    // With moving traffic, the time-expanded state space is already large;
    // retain the soft reversal cost in nodes but omit its extra key dimension.
    // In open water, keep the turn history in the key so the C-path preference
    // is not lost when otherwise identical search states are merged.
    const bool use_turn_history_dimension =
      usesUsvDynamicModel() && dynamic_obstacles_.empty();
    const bool fine_turn_states = usesUsvDynamicModel() && gentle_turn_sampling_ &&
      prefer_broad_channel_curve_;
    const double turn_effort_weight = steeringMagnitudeWeightForGoal(stheta, gtheta);
    std::priority_queue<
      std::shared_ptr<Node3D>, std::vector<std::shared_ptr<Node3D>>, CompareNode> open_set;

    auto start = std::make_shared<Node3D>();
    start->x = sx;
    start->y = sy;
    start->theta = stheta;
    start->t = initial_time;
    start->u = usesUsvDynamicModel() ? std::clamp(start_u, 0.0, cruise_speed_) : cruise_speed_;
    start->v = usesUsvDynamicModel() ? start_v : 0.0;
    start->r = usesUsvDynamicModel() ? std::clamp(start_r, -usv_max_yaw_rate_, usv_max_yaw_rate_) : 0.0;
    start->turn_direction = start->r > 0.02 ? 1 : (start->r < -0.02 ? -1 : 0);
    start->h = heuristic(sx, sy, stheta, gx, gy, gtheta);
    start->f = start->h;
    start->idx = getIdx(sx, sy, stheta);
    if (start->idx < 0) {
      return {};
    }
    auto stateKey = [this, total_states, use_time_dimension, use_turn_history_dimension, fine_turn_states](
      int index, double time, double u, double v, double r, int turn_direction) {
        std::uint64_t dynamic_code = 0U;
        std::uint64_t dynamic_multiplier = 1U;
        std::uint64_t turn_history_code = 0U;
        std::uint64_t turn_history_multiplier = 1U;
        if (usesUsvDynamicModel()) {
          // Surge/sway use three coarse bins; gentle channel turns subdivide
          // yaw rate near zero so low-rate motion is not merged with straight
          // motion. RK4 states remain continuous: only duplicate detection is
          // quantized. Traffic searches omit the turn-history factor below.
          const int u_bin = std::clamp(static_cast<int>(std::lround(u / 2.0)), 0, 2);
          const int v_bin = std::clamp(static_cast<int>(std::lround(v / 2.0 + 1.0)), 0, 2);
          int r_bin = std::clamp(static_cast<int>(std::lround(
            (r + usv_max_yaw_rate_) / usv_max_yaw_rate_)), 0, 2);
          const int yaw_rate_bins = fine_turn_states ? 11 : 3;
          if (fine_turn_states) {
            // 原三档把小艏摇率和直航合并，小转向在角度跨格前就被低代价
            // 直航剪掉。仅在航道缓弯细分零附近艏摇率，保留小转向积累过程。
            // 船模状态仍是连续值；此处只影响重复节点判定。
            const double fraction = std::abs(r) / std::max(1.0e-6, usv_max_yaw_rate_);
            const double boundaries[] = {0.01, 0.045, 0.10, 0.20, 0.40};
            const int magnitude = static_cast<int>(
              std::upper_bound(std::begin(boundaries), std::end(boundaries), fraction) -
              std::begin(boundaries));
            r_bin = 5 + (r < 0.0 ? -magnitude : magnitude);
          }
          dynamic_code = static_cast<std::uint64_t>(u_bin + 3 * (v_bin + 3 * r_bin));
          dynamic_multiplier = 3U * 3U * static_cast<std::uint64_t>(yaw_rate_bins);
          // The cost of a future turn depends on the last non-zero direction;
          // preserve that small history dimension so A* does not merge a C
          // prefix and an S-turn prefix at an otherwise identical boat state.
          if (use_turn_history_dimension) {
            turn_history_code = static_cast<std::uint64_t>(turn_direction + 1);
            turn_history_multiplier = 3U;
          }
        }
        const std::uint64_t time_bucket = use_time_dimension ?
          static_cast<std::uint64_t>(std::floor(time / 2.0)) : 0U;
        const std::uint64_t state_code = dynamic_code + dynamic_multiplier * turn_history_code;
        return static_cast<std::uint64_t>(index) + total_states * state_code +
          total_states * dynamic_multiplier * turn_history_multiplier * time_bucket;
      };
    g_values[stateKey(start->idx,start->t,start->u,start->v,start->r,
      start->turn_direction)] = 0.0F;
    open_set.push(start);

    // CAR_LIKE maps control to steering angle; the USV modes use yaw-rate reference.
    const std::vector<float> turn_controls = usesUsvDynamicModel() ?
      (fine_turn_states ?
        std::vector<float>{-1.0F, -0.5F, -0.25F, -0.125F, -0.0625F,
          0.0F, 0.0625F, 0.125F, 0.25F, 0.5F, 1.0F} :
        std::vector<float>{-1.0F, -0.5F, -0.25F, 0.0F, 0.25F, 0.5F, 1.0F}) :
      std::vector<float>{-1.0F, -0.6F, -0.3F, 0.0F, 0.3F, 0.6F, 1.0F};

    int iterations = 0;
    while (!open_set.empty() && iterations++ < max_iterations_) {
      if (searchTimeExpired()) {
        RCLCPP_WARN(rclcpp::get_logger("HybridAStar"), "Search time budget exhausted; retry on next monitoring cycle");
        return {};
      }
      const auto current = open_set.top();
      open_set.pop();
      if (current->g > g_values[stateKey(current->idx,current->t,
          current->u,current->v,current->r,current->turn_direction)] + 1.0e-5F) {
        continue;
      }
      const float distance_to_goal = std::hypot(current->x - gx, current->y - gy);
      const float angle_difference = std::abs(hybrid_a_star_planner::normalizeAngle(
        gtheta - current->theta));
      // 解析连接会立即返回，若距离很远会绕过航道软代价，因此只在终点附近使用。
      if(!usesUsvDynamicModel() && distance_to_goal < analytic_expansion_distance_ && iterations%10==1) {
        auto tail=connect(*current,gx,gy,gtheta);
        if(!tail.empty()) {
          auto path=extractPath(current);
          path.insert(path.end(),tail.begin(),tail.end());
          return path;
        }
      }
      if (distance_to_goal < (usesUsvDynamicModel() ? 1.0F : 0.6F) &&
        angle_difference < goal_heading_tolerance_) {
        if (fine_turn_states && channel_curve_weight_ > 0.0) {
          auto broad = refineBroadChannelCurve(*start, gx, gy, gtheta, current->g);
          if (!broad.empty()) {return broad;}
        }
        return extractPath(current);
      }

      for (const float turn_control : turn_controls) {
        double proximity_cost = 0.0;
        double arrival_time=current->t;
        double propagated_distance = 0.0;
        Node3D propagated;
        if (!primitiveIsSafe(*current, turn_control, proximity_cost,arrival_time,
            propagated,propagated_distance)) {
          continue;
        }
        const float nx = propagated.x;
        const float ny = propagated.y;
        const float ntheta = propagated.theta;
        const int next_index = getIdx(nx, ny, ntheta);
        if (next_index < 0) {
          continue;
        }
        // Large-inertia vessels should not alternate the turn command at every
        // primitive. These soft costs apply equally to rudder angle and USV yaw
        // rate; collision and COLREG constraints can still demand a sharp turn.
        const double turn_change = steering_change_weight_ *
          std::abs(turn_control - current->turn_control);
        const double turn_effort = turn_effort_weight * propagated_distance *
          turn_control * turn_control;
        // Penalize a change in the *overall* turn direction, not only two
        // adjacent controls. A few straight primitives must not erase the
        // remembered direction and make a delayed S-turn effectively free.
        const int requested_turn_direction = turn_control > 1.0e-4F ? 1 :
          (turn_control < -1.0e-4F ? -1 : 0);
        const int next_turn_direction = requested_turn_direction != 0 ?
          requested_turn_direction : current->turn_direction;
        const bool reverses_turn = requested_turn_direction != 0 &&
          current->turn_direction != 0 &&
          requested_turn_direction != current->turn_direction;
        const double turn_reversal = reverses_turn ? turn_reversal_weight_ : 0.0;
        const float next_g = current->g + static_cast<float>(propagated_distance) + static_cast<float>(
          proximity_cost + colregsCost(ntheta, arrival_time) + turn_change + turn_effort + turn_reversal);
        const auto state_key=stateKey(next_index,arrival_time,
          propagated.u,propagated.v,propagated.r,next_turn_direction);
        auto old=g_values.find(state_key);
        if (old!=g_values.end() && next_g >= old->second) {
          continue;
        }
        auto next = std::make_shared<Node3D>();
        next->x = nx;
        next->y = ny;
        next->theta = ntheta;
        next->g = next_g;
        next->h = heuristic(nx, ny, ntheta, gx, gy, gtheta);
        // 缓弯时减少贪心前推：只追求快速接近终点会把转向拖到航道折角。
        // 其他任务保留原权重；启发场缓存、碰撞检查和最小半径均不改变。
        next->f = next->g + (fine_turn_states ? 1.15F : 1.5F) * next->h;
        next->t = arrival_time;
        next->turn_control = turn_control;
        next->turn_direction = next_turn_direction;
        next->u = propagated.u;
        next->v = propagated.v;
        next->r = propagated.r;
        next->idx = next_index;
        next->parent = current;
        g_values[state_key] = next_g;
        open_set.push(next);
      }
    }
    RCLCPP_WARN(
      rclcpp::get_logger("HybridAStar"), "Search failed after %d iterations", iterations);
    return {};
  }

  // Geometric tracking reference, explicitly separate from the six-state rollout.
  // Only replace a path with fewer substantial reversals and lower curvature.
  // Endpoint tolerance, map/traffic/COLREG checks and measured-state joining remain.
  std::vector<geometry_msgs::msg::Pose> optimizeTrackingPath(
    const std::vector<geometry_msgs::msg::Pose> & raw, double goal_heading,
    double preferred_radius=8.0, double maximum_deviation=12.0,
    double time_limit=.15) const
  {
    // A rescued rollout has already been solved through the full vessel
    // dynamics. Preserve it rather than replacing it with a geometric curve.
    if(rescue_selected_ || raw.size()<3) {return raw;}
    const auto before=groundPathQuality(raw);
    preferred_radius=std::max(3.0,preferred_radius);
    if(!std::isfinite(before.peak_curvature) || before.peak_curvature<1e-5 ||
      (before.reversals<=1 && before.peak_curvature<=1.0/preferred_radius)) {return raw;}
    const auto started=std::chrono::steady_clock::now();
    auto expired=[&]() {return (search_cancel_ && search_cancel_->load()) ||
      std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>time_limit;};
    const auto & first=raw.front();const auto & last=raw.back();
    const double chord=std::hypot(last.position.x-first.position.x,last.position.y-first.position.y);
    const auto sampled=resampleGroundPath(raw,.25);
    if(sampled.size()<3) {return raw;}
    const double start_course=std::atan2(sampled[1].position.y-sampled[0].position.y,
      sampled[1].position.x-sampled[0].position.x);
    const double required=std::max({1.0,chord,planningRadius()*std::abs(
      normalizeAngle(goal_heading-tf2::getYaw(first.orientation)))});
    const double minimum_radius=std::max(3.0,1.05/before.peak_curvature);
    std::vector<double> radii{minimum_radius,minimum_radius*1.1,minimum_radius*1.25,minimum_radius*1.4,
      std::max(minimum_radius,preferred_radius),
      std::max(minimum_radius,preferred_radius*1.25),std::max(minimum_radius,preferred_radius*1.5),
      std::max(minimum_radius,preferred_radius*2.0)};
    std::sort(radii.begin(),radii.end());
    radii.erase(std::unique(radii.begin(),radii.end()),radii.end());
    auto tracking_score=[&](const auto &path,const auto &quality) {
      return mainLaneEntryCost(path)+10.0*quality.reversals+
        10.0*preferred_radius*quality.peak_curvature+.1*quality.length;
    };
    auto best=raw;auto best_quality=before;
    double best_tracking_score=tracking_score(raw,before);
    bool raw_in_main=channel_enabled_;
    for(const auto &p:raw) {raw_in_main=raw_in_main && laneRegionAt(p.position.x,p.position.y)==Region::MAIN;}
    auto stays_in_original_lane=[&](const auto & candidate) {
      return !raw_in_main || std::none_of(candidate.begin(),candidate.end(),[&](const auto &p) {
        return laneRegionAt(p.position.x,p.position.y)!=Region::MAIN;
      });
    };
    for(double radius:radii) {
      if(expired()) {break;}
      auto space=std::make_shared<ompl::base::DubinsStateSpace>(radius);
      ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space),s(space);
      a->setXY(first.position.x,first.position.y);a->setYaw(start_course);
      b->setXY(last.position.x,last.position.y);
      for(double offset:{0.0,-.25,.25,-.5,.5,-.75,.75,-.95,.95}) {
        if(expired()) {break;}
        const double terminal_heading=normalizeAngle(goal_heading+offset*goal_heading_tolerance_);
        b->setYaw(terminal_heading);
        const double length=space->distance(a.get(),b.get());
        if(!std::isfinite(length) || length>required*maximum_path_ratio_+maximum_path_extra_) {continue;}
        const int count=std::max(2,static_cast<int>(std::ceil(length/.2)));
        std::vector<geometry_msgs::msg::Pose> candidate;candidate.reserve(count+1);
        bool interpolate_first=true;ompl::base::DubinsStateSpace::DubinsPath curve;
        for(int i=0;i<=count;++i) {
          space->interpolate(a.get(),b.get(),static_cast<double>(i)/count,interpolate_first,curve,s.get());
          auto p=first;p.position.x=s->getX();p.position.y=s->getY();
          tf2::Quaternion q;q.setRPY(0,0,s->getYaw());p.orientation=tf2::toMsg(q);
          candidate.push_back(p);
        }
        candidate.front()=first;candidate.back().position=last.position;
        const auto quality=groundPathQuality(candidate);
        // The same spatial metric is used on both paths; never accept a sharper
        // replacement, even if it looks like a nicer C in the visualization.
        if(quality.reversals>std::min(1,before.reversals) ||
          quality.peak_curvature>before.peak_curvature*.98 ||
          quality.rms_curvature>before.rms_curvature ||
          (!main_entry_active_ && lane_turn_direction_ && quality.dominant_turn!=lane_turn_direction_) ||
          pathHasAvoidableGoalLoop(candidate,loop_check_radius_,loop_check_arc_)) {continue;}
        const double score=tracking_score(candidate,quality);
        const bool better=main_entry_weight_>0 && mainLaneEntryApplicable(
          first.position.x,first.position.y,last.position.x,last.position.y) ?
          score<best_tracking_score-1e-5 :
          (quality.reversals<best_quality.reversals ||
          (quality.reversals==best_quality.reversals &&
          (quality.peak_curvature<best_quality.peak_curvature-1e-5 ||
          (std::abs(quality.peak_curvature-best_quality.peak_curvature)<1e-5 &&
            quality.length<best_quality.length))));
        if(!better || !groundPathsWithinDeviation(candidate,raw,maximum_deviation) ||
          !pathIsSafe(candidate)) {continue;}
        if(!stays_in_original_lane(candidate)) {continue;}
        best=std::move(candidate);best_quality=quality;best_tracking_score=tracking_score(best,quality);
      }
    }
    // A wider complete turn may be blocked by the channel. Preserve the main
    // integrated arc and smooth only its tail; peak curvature must not rise,
    // and the number of substantial reversals / RMS curvature must improve.
    if(before.reversals>1 && best_quality.reversals>1 && !expired()) {
      const auto prefix_end=dominantTurnEndIndex(raw);
      std::vector<geometry_msgs::msg::Pose> original_tail(raw.begin()+prefix_end,raw.end());
      auto tail=resampleGroundPath(original_tail,.25);
      const auto initial_tail=tail;
      const auto original_tail_quality=groundPathQuality(original_tail);
      if(tail.size()>3) {
        const auto &begin=tail.front().position;const auto &end=tail.back().position;
        const double distance=std::hypot(end.x-begin.x,end.y-begin.y);
        const double course=std::atan2(tail[1].position.y-begin.y,tail[1].position.x-begin.x);
        // A monotone tail tangent can replace several small pursuit corrections
        // without widening the already safe main turn into a neighboring bank.
        for(double entry:{.2,.35,.5}) for(double exit:{.2,.4,.6}) {
          for(double offset:{0.0,-.5,.5,-.95,.95}) {
            if(expired() || best_quality.reversals<=1) {break;}
            const double terminal=normalizeAngle(goal_heading+offset*goal_heading_tolerance_);
            const double p1x=begin.x+entry*distance*std::cos(course),p1y=begin.y+entry*distance*std::sin(course);
            const double p2x=end.x-exit*distance*std::cos(terminal),p2y=end.y-exit*distance*std::sin(terminal);
            const int count=std::max(3,static_cast<int>(std::ceil(distance/.15)));
            std::vector<geometry_msgs::msg::Pose> curve;
            for(int i=0;i<=count;++i) {
              const double t=static_cast<double>(i)/count,q=1-t;
              auto p=tail.front();
              p.position.x=q*q*q*begin.x+3*q*q*t*p1x+3*q*t*t*p2x+t*t*t*end.x;
              p.position.y=q*q*q*begin.y+3*q*q*t*p1y+3*q*t*t*p2y+t*t*t*end.y;
              const double dx=3*q*q*(p1x-begin.x)+6*q*t*(p2x-p1x)+3*t*t*(end.x-p2x);
              const double dy=3*q*q*(p1y-begin.y)+6*q*t*(p2y-p1y)+3*t*t*(end.y-p2y);
              double yaw=std::atan2(dy,dx);
              const double blend=std::min(1.0,t*distance/3.0);
              yaw=tf2::getYaw(tail.front().orientation)+blend*normalizeAngle(yaw-tf2::getYaw(tail.front().orientation));
              tf2::Quaternion orientation;orientation.setRPY(0,0,yaw);p.orientation=tf2::toMsg(orientation);
              curve.push_back(p);
            }
            curve.front()=tail.front();curve.back().position=raw.back().position;
            tf2::Quaternion orientation;orientation.setRPY(0,0,terminal);curve.back().orientation=tf2::toMsg(orientation);
            std::vector<geometry_msgs::msg::Pose> candidate(raw.begin(),raw.begin()+prefix_end);
            candidate.insert(candidate.end(),curve.begin(),curve.end());
            const auto quality=groundPathQuality(candidate);
            if(quality.reversals<best_quality.reversals && quality.rms_curvature<before.rms_curvature &&
              quality.peak_curvature<=before.peak_curvature+1e-6 &&
              groundPathQuality(curve).peak_curvature<=original_tail_quality.peak_curvature+1e-6 &&
              quality.length<=required*maximum_path_ratio_+maximum_path_extra_ &&
              (!main_entry_active_ || tracking_score(candidate,quality)<=best_tracking_score+1e-5) &&
              stays_in_original_lane(candidate) && pathIsSafe(candidate) &&
              !pathHasAvoidableGoalLoop(candidate,loop_check_radius_,loop_check_arc_) &&
              groundPathsWithinDeviation(candidate,raw,maximum_deviation)) {
              best=std::move(candidate);best_quality=quality;best_tracking_score=tracking_score(best,quality);
            }
          }
        }
      }
      for(int iteration=0;iteration<120 && !expired() && tail.size()>3 && best_quality.reversals>1;++iteration) {
        auto next=tail;
        for(std::size_t i=1;i+1<tail.size();++i) {
          const auto span=std::min<std::size_t>({8U,i,tail.size()-1-i});
          auto &p=next[i].position;const auto &old=tail[i].position;
          p.x=old.x+.26*(tail[i-1].position.x+tail[i+1].position.x-2*old.x)+
            .12*(.5*(tail[i-span].position.x+tail[i+span].position.x)-old.x);
          p.y=old.y+.26*(tail[i-1].position.y+tail[i+1].position.y-2*old.y)+
            .12*(.5*(tail[i-span].position.y+tail[i+span].position.y)-old.y);
        }
        updatePathHeadings(next,initial_tail.front().orientation,raw.back().orientation);
        // Blend body heading over the first 3m of the changed tail, preserving
        // the measured/integrated prefix instead of introducing a yaw step.
        for(std::size_t i=1;i<std::min<std::size_t>(13,next.size()-1);++i) {
          const double f=static_cast<double>(i)/12,weight=f*f*(3-2*f);
          const double yaw=tf2::getYaw(initial_tail[i].orientation)+weight*normalizeAngle(
            tf2::getYaw(next[i].orientation)-tf2::getYaw(initial_tail[i].orientation));
          tf2::Quaternion q;q.setRPY(0,0,yaw);next[i].orientation=tf2::toMsg(q);
        }
        std::vector<geometry_msgs::msg::Pose> candidate(raw.begin(),raw.begin()+prefix_end);
        candidate.insert(candidate.end(),next.begin(),next.end());
        const auto quality=groundPathQuality(candidate);
        if(quality.peak_curvature>before.peak_curvature+1e-6 ||
          groundPathQuality(next).peak_curvature>original_tail_quality.peak_curvature+1e-6 ||
          !stays_in_original_lane(candidate) || !pathIsSafe(candidate) ||
          (main_entry_active_ && tracking_score(candidate,quality)>best_tracking_score+1e-5)) {break;}
        tail=std::move(next);
        const bool better=quality.reversals<best_quality.reversals ||
          (quality.reversals==best_quality.reversals &&
            quality.rms_curvature<best_quality.rms_curvature*.98);
        if(better && quality.rms_curvature<before.rms_curvature &&
          !pathHasAvoidableGoalLoop(candidate,loop_check_radius_,loop_check_arc_) &&
          groundPathsWithinDeviation(candidate,raw,maximum_deviation)) {
          best=std::move(candidate);best_quality=quality;best_tracking_score=tracking_score(best,quality);
        }
      }
    }
    if(best_quality.reversals<before.reversals || best_quality.peak_curvature<before.peak_curvature*.98) {
      RCLCPP_INFO(rclcpp::get_logger("HybridAStar"),
        "Geometric reference: reversals %d -> %d, peak curvature %.4f -> %.4f 1/m (not a dynamic rollout)",
        before.reversals,best_quality.reversals,before.peak_curvature,best_quality.peak_curvature);
    }
    return best;
  }

  // [功能与联系] 统一弧长采样并迭代短/长尺度平滑；每次候选由pathIsSafe复查，不安全缩步或退回原安全结果。
  std::vector<geometry_msgs::msg::Pose> postProcessPath(
    const std::vector<geometry_msgs::msg::Pose> & raw, double requested_spacing,
    int iterations, double smooth_weight, double data_weight, double max_deviation,
    int long_range_points = 0, double long_range_weight = 0.0) const
  {
    if(rescue_selected_) {return raw;}
    if (raw.size() < 3U) {return raw;}
    const double spacing = std::max(0.25, requested_spacing);
    std::vector<double> arc(raw.size(), 0.0);
    for (std::size_t i = 1; i < raw.size(); ++i) {
      arc[i] = arc[i - 1] + std::hypot(
        raw[i].position.x - raw[i - 1].position.x,
        raw[i].position.y - raw[i - 1].position.y);
    }
    if (arc.back() < spacing) {return raw;}

    // 统一采样间距，避免搜索基元(2m)和Dubins尾段(0.15m)密度差异使局部
    // 圆弧在全局上形成锯齿或波浪。
    std::vector<geometry_msgs::msg::Pose> resampled;
    std::size_t segment = 1U;
    for (double distance = 0.0; distance < arc.back(); distance += spacing) {
      while (segment + 1U < arc.size() && arc[segment] < distance) {++segment;}
      const double length = std::max(1.0e-6, arc[segment] - arc[segment - 1U]);
      const double ratio = std::clamp((distance - arc[segment - 1U]) / length, 0.0, 1.0);
      geometry_msgs::msg::Pose pose = raw[segment - 1U];
      pose.position.x += ratio * (raw[segment].position.x - raw[segment - 1U].position.x);
      pose.position.y += ratio * (raw[segment].position.y - raw[segment - 1U].position.y);
      resampled.push_back(pose);
    }
    resampled.push_back(raw.back());
    const auto original = resampled;
    updatePathHeadings(resampled, raw.front().orientation, raw.back().orientation);
    if (!pathIsSafe(resampled) || !pathRespectsUsvKinematics(resampled)) {
      // 重采样弦线若切入障碍区或超出USV艏摇能力，退回已验证的原始曲线。
      return raw;
    }

    std::vector<geometry_msgs::msg::Pose> smoothed = resampled;
    const int count = std::max(0, iterations);
    for (int iteration = 0; iteration < count; ++iteration) {
      auto candidate = smoothed;
      for (std::size_t i = 1U; i + 1U < candidate.size(); ++i) {
        // 相邻点项处理短小锯齿；跨多个采样点的弦中点项处理几十米尺度的
        // 宽大S弯。后者不会把航道中心线的缓弯误当成单点噪声。
        const std::size_t radius = std::min<std::size_t>(
          std::max(0, long_range_points), std::min(i, candidate.size() - 1U - i));
        double long_dx = 0.0;
        double long_dy = 0.0;
        if (radius > 0U) {
          long_dx = 0.5 * (smoothed[i - radius].position.x +
            smoothed[i + radius].position.x) - smoothed[i].position.x;
          long_dy = 0.5 * (smoothed[i - radius].position.y +
            smoothed[i + radius].position.y) - smoothed[i].position.y;
        }
        double x = smoothed[i].position.x + smooth_weight *
          (smoothed[i - 1U].position.x + smoothed[i + 1U].position.x -
          2.0 * smoothed[i].position.x) +
          data_weight * (original[i].position.x - smoothed[i].position.x) +
          long_range_weight * long_dx;
        double y = smoothed[i].position.y + smooth_weight *
          (smoothed[i - 1U].position.y + smoothed[i + 1U].position.y -
          2.0 * smoothed[i].position.y) +
          data_weight * (original[i].position.y - smoothed[i].position.y) +
          long_range_weight * long_dy;
        const double dx = x - original[i].position.x;
        const double dy = y - original[i].position.y;
        const double deviation = std::hypot(dx, dy);
        if (max_deviation > 0.0 && deviation > max_deviation) {
          x = original[i].position.x + dx * max_deviation / deviation;
          y = original[i].position.y + dy * max_deviation / deviation;
        }
        candidate[i].position.x = x;
        candidate[i].position.y = y;
      }
      updatePathHeadings(candidate, raw.front().orientation, raw.back().orientation);
      // 每轮都重新检查静态障碍、动态船时间同步安全域和COLREG航向。
      // 若完整平滑步长过大，则逐级缩小步长，而不是让一个临障点冻结整条
      // 路线；这样无障碍长段仍可继续消除低频蛇形，同时不削弱避碰能力。
      if (pathIsSafe(candidate) && pathRespectsUsvKinematics(candidate)) {
        smoothed = std::move(candidate);
      } else {
        for (const double scale : {0.5, 0.25, 0.125}) {
          auto reduced = smoothed;
          for (std::size_t i = 1U; i + 1U < reduced.size(); ++i) {
            reduced[i].position.x += scale *
              (candidate[i].position.x - smoothed[i].position.x);
            reduced[i].position.y += scale *
              (candidate[i].position.y - smoothed[i].position.y);
          }
          updatePathHeadings(reduced, raw.front().orientation, raw.back().orientation);
          if (pathIsSafe(reduced) && pathRespectsUsvKinematics(reduced)) {
            smoothed = std::move(reduced);
            break;
          }
        }
      }
    }
    return smoothed;
  }


private:
  // [功能与联系] 对遇尚未安全通过时，将候选保持在目标预测位置的右舷侧，
  // 且不允许越过锁定原航向向左；避免达到初始转角后又从船首左侧抄近路。
  bool headOnStarboardPassAllowed(
    const DynamicObstacle & obstacle, double x, double y, double heading,
    double target_x, double target_y, double safe) const
  {
    const auto &policy=obstacle.policy;
    if(!enforce_head_on_starboard_ || !policy.locked || policy.recovering ||
      policy.type!=EncounterType::HEAD_ON) return true;
    const double dx=target_x-x,dy=target_y-y;
    const double forward=dx*std::cos(policy.reference)+dy*std::sin(policy.reference);
    if(forward < -safe) return true;
    const double own_lateral=-std::sin(policy.reference)*(x-target_x)+
      std::cos(policy.reference)*(y-target_y);
    const double predicted_start_x=obstacle.state.x+obstacle.state.vx*search_time_offset_;
    const double predicted_start_y=obstacle.state.y+obstacle.state.vy*search_time_offset_;
    const double initial_lateral=-std::sin(policy.reference)*
      (search_start_x_-predicted_start_x)+std::cos(policy.reference)*
      (search_start_y_-predicted_start_y);
    // 允许从轻微偏置的初始相对位置逐步右转，但到正横前必须进入右侧。
    const double allowed_lateral=forward<=0.0 ? 0.25 :
      std::max(0.25,initial_lateral+0.25);
    if(own_lateral>allowed_lateral) return false;
    return hybrid_a_star_planner::normalizeAngle(heading-policy.reference)<=
      2.0*hybrid_a_star_planner::deg;
  }

  // [功能与联系] 由平滑点邻域切向更新中间姿态，保留起终姿态；为headingAllowed与输出轨迹提供一致航向。
  void updatePathHeadings(
    std::vector<geometry_msgs::msg::Pose> & path,
    const geometry_msgs::msg::Quaternion & start_orientation,
    const geometry_msgs::msg::Quaternion & goal_orientation) const
  {
    if (path.empty()) {return;}
    path.front().orientation = start_orientation;
    path.back().orientation = goal_orientation;
    for (std::size_t i = 1U; i + 1U < path.size(); ++i) {
      const double yaw = std::atan2(
        path[i + 1U].position.y - path[i - 1U].position.y,
        path[i + 1U].position.x - path[i - 1U].position.x);
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, yaw);
      path[i].orientation = tf2::toMsg(quaternion);
    }
  }

  // [功能与联系] 沿每段加密取样，重新计算时间、静态危险和目标同步净空/COLREG约束；是平滑候选接受门槛。
  bool pathIsSafe(const std::vector<geometry_msgs::msg::Pose> & path) const
  {
    double elapsed = search_time_offset_;
    for (std::size_t i = 1U; i < path.size(); ++i) {
      const double dx = path[i].position.x - path[i - 1U].position.x;
      const double dy = path[i].position.y - path[i - 1U].position.y;
      const double length = std::hypot(dx, dy);
      if (length < 1.0e-8) {continue;}
      const double heading = std::atan2(dy, dx);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(length / std::max(0.25F, 0.5F * resolution_))));
      double px=path[i-1U].position.x,py=path[i-1U].position.y;
      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        const double x = path[i - 1U].position.x + ratio * dx;
        const double y = path[i - 1U].position.y + ratio * dy;
        if (isLethal(static_cast<float>(x), static_cast<float>(y))) {return false;}
        elapsed+=estimateSegmentTime(px,py,x,y,length/samples);px=x;py=y;
        const double time=elapsed;
        for (const auto & obstacle : dynamic_obstacles_) {
          if (!hybrid_a_star_planner::headingAllowed(
              obstacle.policy, heading, time, 0.0, search_heading_))
          {
            return false;
          }
          const double target_x = obstacle.state.x + obstacle.state.vx * time;
          const double target_y = obstacle.state.y + obstacle.state.vy * time;
          const double safe = std::max({
            own_ship_radius_ + obstacle.state.radius + safety_buffer_, dynamic_hard_distance_,
            obstacle.policy.type == EncounterType::OVERTAKING ? overtaking_safe_distance_ : 0.0});
          if (std::hypot(x - target_x, y - target_y) < safe) {return false;}
          if(!headOnStarboardPassAllowed(
              obstacle,x,y,heading,target_x,target_y,safe)) return false;
          if (obstacle.policy.locked &&
            obstacle.policy.type == EncounterType::CROSSING_STARBOARD)
          {
            const double speed = std::hypot(obstacle.state.vx, obstacle.state.vy);
            if (speed > 0.05) {
              const double along = ((x - target_x) * obstacle.state.vx +
                (y - target_y) * obstacle.state.vy) / speed;
              const double across = ((x - target_x) * obstacle.state.vy -
                (y - target_y) * obstacle.state.vx) / speed;
              if (std::abs(across) < safe && along > -safe) {return false;}
            }
          }
        }
      }
    }
    return true;
  }

  // [功能与联系] 用所选模型对应的最小转弯半径生成近终点Dubins尾段，沿曲线复查静态/动态/规则安全。
  std::vector<geometry_msgs::msg::Pose> connect(const Node3D &n,double gx,double gy,double yaw) const {
    auto space=std::make_shared<ompl::base::DubinsStateSpace>(dubinsTurningRadius());
    ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space),s(space);
    a->setXY(n.x,n.y);a->setYaw(n.theta);b->setXY(gx,gy);b->setYaw(yaw);
    const double length=space->distance(a.get(),b.get());
    if(length>std::hypot(gx-n.x,gy-n.y)+20) return {};
    const int samples=std::max(1,static_cast<int>(std::ceil(length/0.15)));
    std::vector<geometry_msgs::msg::Pose> result;
    bool first=true;ompl::base::DubinsStateSpace::DubinsPath curve;
    double time=n.t,px=n.x,py=n.y;
    for(int i=1;i<=samples;++i) {
      const double fraction=static_cast<double>(i)/samples;
      space->interpolate(a.get(),b.get(),fraction,first,curve,s.get());
      double x=s->getX(),y=s->getY(),heading=s->getYaw();
      time+=estimateSegmentTime(px,py,x,y,length/samples);px=x;py=y;
      if(isLethal(x,y))return {};
      for(const auto &o:dynamic_obstacles_) {
        if(!hybrid_a_star_planner::headingAllowed(o.policy,heading,time,0,search_heading_))return {};
        const double tx=o.state.x+o.state.vx*time,ty=o.state.y+o.state.vy*time;
        const double safe=std::max({own_ship_radius_+o.state.radius+safety_buffer_, dynamic_hard_distance_,
          o.policy.type==EncounterType::OVERTAKING ? overtaking_safe_distance_:0.0});
        if(std::hypot(x-tx,y-ty)<safe+0.15)return {};
        if(!headOnStarboardPassAllowed(o,x,y,heading,tx,ty,safe))return {};
        const double speed=std::hypot(o.state.vx,o.state.vy);
        if(o.policy.locked && o.policy.type==EncounterType::CROSSING_STARBOARD && speed>0.05) {
          const double along=((x-tx)*o.state.vx+(y-ty)*o.state.vy)/speed;
          const double across=((x-tx)*o.state.vy-(y-ty)*o.state.vx)/speed;
          if(std::abs(across)<safe && along> -safe)return {};
        }
      }
      geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;
      tf2::Quaternion q;q.setRPY(0,0,heading);p.orientation=tf2::toMsg(q);result.push_back(p);
    }
    return result;
  }
  // [功能与联系] 将世界点换算到主物理栅格并检查边界；供硬碰撞和状态索引使用。
  bool worldToMap(float x, float y, int & gx, int & gy) const
  {
    const double dx = x - origin_x_;
    const double dy = y - origin_y_;
    const double local_x = std::cos(origin_yaw_) * dx + std::sin(origin_yaw_) * dy;
    const double local_y = -std::sin(origin_yaw_) * dx + std::cos(origin_yaw_) * dy;
    gx = static_cast<int>(std::floor(local_x / resolution_));
    gy = static_cast<int>(std::floor(local_y / resolution_));
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
  }

  enum Region {MAIN, OUTSIDE, OPPOSITE, UNKNOWN};

  // [功能与联系] 按地图原点姿态/分辨率查询代价，越界/缺图返回保守或未提供值；支撑多源融合和候选淘汰。
  static int costAt(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
  {
    if (map.info.width == 0 || map.info.height == 0 || map.data.empty()) {
      return -1;
    }
    const double yaw = tf2::getYaw(map.info.origin.orientation);
    const double dx = x - map.info.origin.position.x;
    const double dy = y - map.info.origin.position.y;
    const int gx = static_cast<int>(std::floor(
      (std::cos(yaw) * dx + std::sin(yaw) * dy) / map.info.resolution));
    const int gy = static_cast<int>(std::floor(
      (-std::sin(yaw) * dx + std::cos(yaw) * dy) / map.info.resolution));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.info.width) ||
      gy >= static_cast<int>(map.info.height))
    {
      return -1;
    }
    return map.data[static_cast<std::size_t>(gy) * map.info.width + gx];
  }

  // [功能与联系] 按地图原点姿态/分辨率查询代价，越界/缺图返回保守或未提供值；支撑多源融合和候选淘汰。
  int costAt(const nav_msgs::msg::OccupancyGrid::SharedPtr & map, double x, double y) const
  {
    return map ? costAt(*map, x, y) : -1;
  }

  // [功能与联系] 由语义栅格编码判定MAIN/OUTSIDE/OPPOSITE/UNKNOWN；navigationCostAt和isLethal分开使用区域与安全含义。
  bool mainLaneEntryApplicable(double sx,double sy,double gx,double gy) const {
    if(!channel_enabled_ || !lane_costmap_ || main_entry_weight_<=0) return false;
    const auto start=laneRegionAt(sx,sy),goal=laneRegionAt(gx,gy);
    if(start==Region::UNKNOWN || goal==Region::UNKNOWN) return false;
    if(goal==Region::MAIN) return start!=Region::MAIN;
    // A short opposite-lane destination may be approached directly. For a long
    // task, main-lane progress and a late exit are only a preference.
    return goal==Region::OPPOSITE && std::hypot(gx-sx,gy-sy)>main_entry_direct_distance_;
  }

  Region laneRegionAt(double x, double y) const
  {
    if (!channel_enabled_ || !lane_costmap_) {
      return Region::UNKNOWN;
    }
    const int value = costAt(*lane_costmap_, x, y);
    if (value < 0) {
      return Region::UNKNOWN;
    }
    if (value <= main_lane_max_) {
      return Region::MAIN;
    }
    if (value >= opposite_lane_min_) {
      return Region::OPPOSITE;
    }
    return Region::OUTSIDE;
  }

  // [功能与联系] 累计主航道中心、对向、航道外岸边和独立障碍膨胀软代价；不能替代isLethal硬检查。
  double navigationCostAt(double x, double y) const
  {
    double cost = 0.0;
    const Region region = laneRegionAt(x, y);
    if (region == Region::MAIN) {
      // lane map 的 0..main_lane_max_ 是“中心到边沿”的软梯度：中心为 0，
      // 边沿较高。它不会禁用边沿，只让无避障需求的路径远离边界。
      const int lane_value = costAt(lane_costmap_, x, y);
      if (lane_value > 0) {
        cost += main_center_weight_ * lane_value / std::max(1, main_lane_max_);
      }
    } else if (region == Region::OUTSIDE || (region == Region::UNKNOWN && channel_enabled_)) {
      cost += outside_lane_weight_;
    } else if (region == Region::OPPOSITE) {
      cost += opposite_lane_weight_*(compact_opposite_maneuver_ ? compact_opposite_scale_:1.0);
    }
    if(main_entry_active_ && main_entry_weight_>0 && region!=Region::MAIN) {
      const double d=main_lane_distance_.distance(x,y);
      const double initial=main_lane_distance_.distance(search_start_x_,search_start_y_);
      if(std::isfinite(d) && std::isfinite(initial)) {
        const double blend=std::max(0.0,1.0-std::hypot(x-search_start_x_,y-search_start_y_)/main_entry_horizon_);
        cost+=main_entry_weight_*blend*std::min(2.0,d/std::max(5.0,initial))/main_entry_horizon_;
      }
    }
    // 岸边膨胀只在航道外参与软代价；主航道和对向航道内部视为自由水域。
    if (region == Region::OUTSIDE || region == Region::UNKNOWN) {
      const int raw = costAt(costmap_, x, y);
      if (raw > 0) {
        cost += outside_map_weight_ * std::min(raw, shoreline_threshold_ - 1) /
          std::max(1, shoreline_threshold_ - 1);
      }
    }
    // 独立航道障碍物层在所有区域都保留软膨胀代价。
    const int obstacle = costAt(channel_obstacle_map_, x, y);
    if (obstacle > 0) {
      cost += channel_obstacle_weight_ * obstacle /
        std::max(1, channel_obstacle_threshold_ - 1);
    }
    if (!reference_distance_.empty()) {
      int gx = 0;
      int gy = 0;
      if (worldToMap(static_cast<float>(x), static_cast<float>(y), gx, gy)) {
        const float distance = reference_distance_[gy * width_ + gx];
        if (std::isfinite(distance)) {
          // 只在恢复方向有动态风险时增强近端旧轨迹的软吸引；安全后立即归零。
          // 随空间距离衰减，不要求继续直航，障碍物硬检查始终优先。
          const double blend=recovery_risk_ && recovery_horizon_>0 ? std::max(0.0,
            1.0-std::hypot(x-search_start_x_,y-search_start_y_)/(cruise_speed_*recovery_horizon_)):0.0;
          // buildReferenceDistance已经把栅格距离换算成米。
          const double deviation = std::min(
            static_cast<double>(distance), reference_path_max_distance_);
          const double continuity_weight = reference_soft_conflict_ ? 0.0 :
            ((recovery_complete_ ? 0.0 : reference_path_weight_)+recovery_weight_*blend);
          cost += continuity_weight *
            deviation / reference_path_max_distance_;

          // 连续两次搜索最影响操船的是本船近前方：MPC前视点若在这里从
          // 旧轨迹左侧跳到右侧，会直接造成反向打舵。近场项从本船位置
          // 向外线性衰减，并对横向偏离使用平方代价；远端仍允许为后续
          // 障碍预先调整。这里只增加搜索代价，不会放行任何不安全基元，
          // 旧路线被动态船占据时，硬碰撞/COLREG检查会迫使搜索安全偏离。
          const double start_distance = std::hypot(
            x - search_start_x_, y - search_start_y_);
          const double near_blend = std::max(
            0.0, 1.0 - start_distance / near_field_reference_horizon_);
          const double normalized_deviation = std::min(
            deviation / near_field_deviation_scale_, 2.0);
          // 安全通过后立刻放开旧避让路径，避免恢复阶段继续被旧路线拖住。
          const double near_weight = (recovery_complete_ || reference_soft_conflict_) ?
            0.0 : near_field_reference_weight_;
          cost += near_weight * near_blend *
            normalized_deviation * normalized_deviation;
        }
      }
    }
    return cost;
  }

  // [功能与联系] 结合二维可通行代价场与当前模型的最小转弯半径估计剩余成本；用于open队列优先级。
  float heuristic(float x, float y, float theta, float gx, float gy, float gtheta) const
  {
    const float distance = std::hypot(gx - x, gy - y);
    const float angle = std::abs(hybrid_a_star_planner::normalizeAngle(gtheta - theta));
    int map_x = 0;
    int map_y = 0;
    float water_distance = distance;
    if (worldToMap(x, y, map_x, map_y) && !holonomic_cost_.empty()) {
      const float candidate = holonomic_cost_[map_y * width_ + map_x];
      if (std::isfinite(candidate)) {water_distance = std::max(distance, candidate);}
    }
    if(maneuver_active_ && large_turn_search_) {
      // 低速掉头不能只看欧氏距离：否则队列反复靠近终点却没有转好艏向。
      // Dubins仅提高搜索方向性，真实扩展和最终输出仍使用欠驱动积分。
      auto space=std::make_shared<ompl::base::DubinsStateSpace>(dubinsTurningRadius());
      ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space);
      a->setXY(x,y);a->setYaw(theta);b->setXY(gx,gy);b->setYaw(gtheta);
      // 原搜索本来使用加权启发式；掉头回退增强方向引导，不承诺全局最优。
      return std::max(water_distance,static_cast<float>((close_turn_search_ ? 4.0 : 1.5)*
        space->distance(a.get(),b.get())));
    }
    return water_distance + 0.2F * angle * heuristicTurningRadius();
  }

  // [功能与联系] 将物理栅格中心转换为世界位置，供参考距离场和启发场构建。
  void mapToWorld(const int gx, const int gy, double & x, double & y) const
  {
    const double local_x = (static_cast<double>(gx) + 0.5) * resolution_;
    const double local_y = (static_cast<double>(gy) + 0.5) * resolution_;
    x = origin_x_ + std::cos(origin_yaw_) * local_x - std::sin(origin_yaw_) * local_y;
    y = origin_y_ + std::sin(origin_yaw_) * local_x + std::cos(origin_yaw_) * local_y;
  }

  // [功能与联系] 为旧安全路径生成距离参考场；对恢复仍有风险的近端施加额外连续性软偏好。
  void buildReferenceDistance()
  {
    if (!reference_distance_dirty_) {return;}
    reference_distance_dirty_ = false;
    reference_distance_.clear();
    if (reference_path_.empty() || width_ <= 0 || height_ <= 0) {return;}
    const float infinity = std::numeric_limits<float>::infinity();
    reference_distance_.assign(static_cast<std::size_t>(width_ * height_), infinity);
    for (const auto & pose : reference_path_) {
      int gx = 0;
      int gy = 0;
      if (worldToMap(pose.position.x, pose.position.y, gx, gy)) {
        reference_distance_[gy * width_ + gx] = 0.0F;
      }
    }
    const float diagonal = std::sqrt(2.0F);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int i = y * width_ + x;
        float best = reference_distance_[i];
        if (x > 0) {best = std::min(best, reference_distance_[i - 1] + 1.0F);}
        if (y > 0) {
          best = std::min(best, reference_distance_[i - width_] + 1.0F);
          if (x > 0) {best = std::min(best, reference_distance_[i - width_ - 1] + diagonal);}
          if (x + 1 < width_) {
            best = std::min(best, reference_distance_[i - width_ + 1] + diagonal);
          }
        }
        reference_distance_[i] = best;
      }
    }
    for (int y = height_ - 1; y >= 0; --y) {
      for (int x = width_ - 1; x >= 0; --x) {
        const int i = y * width_ + x;
        float best = reference_distance_[i];
        if (x + 1 < width_) {best = std::min(best, reference_distance_[i + 1] + 1.0F);}
        if (y + 1 < height_) {
          best = std::min(best, reference_distance_[i + width_] + 1.0F);
          if (x > 0) {best = std::min(best, reference_distance_[i + width_ - 1] + diagonal);}
          if (x + 1 < width_) {
            best = std::min(best, reference_distance_[i + width_ + 1] + diagonal);
          }
        }
        reference_distance_[i] = best;
      }
    }
    for (auto & value : reference_distance_) {value *= resolution_;}
  }

  // [功能与联系] 从终点反向建立二维安全/区域代价场，排除无静态连接的目标；供search启发函数使用。
  bool buildHolonomicHeuristic(const double goal_x, const double goal_y)
  {
    int gx = 0;
    int gy = 0;
    if (!worldToMap(goal_x, goal_y, gx, gy)) {return false;}
    const int goal_index = gy * width_ + gx;
    const std::size_t expected_size = static_cast<std::size_t>(width_ * height_);
    if (holonomic_cache_revision_ == static_map_revision_ &&
      holonomic_cache_goal_index_ == goal_index && holonomic_cost_.size() == expected_size &&
      holonomic_cache_compact_opposite_ == compact_opposite_maneuver_ &&
      holonomic_cache_main_entry_ == main_entry_active_ &&
      (!main_entry_active_ || std::hypot(search_start_x_-holonomic_cache_start_x_,
        search_start_y_-holonomic_cache_start_y_)<.01))
    {
      return true;
    }
    const float infinity = std::numeric_limits<float>::infinity();
    holonomic_cost_.assign(static_cast<std::size_t>(width_ * height_), infinity);
    ++holonomic_build_count_;
    using QueueItem = std::pair<float, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pending;
    holonomic_cost_[goal_index] = 0.0F;
    // 每个栅格的安全/代价只求一次，避免Dijkstra八邻域重复执行海图变换及多层查询。
    std::vector<float> cell_cost(expected_size, infinity);
    for (int y=0; y<height_; ++y) {
      if (searchTimeExpired()) {return false;}
      for (int x=0; x<width_; ++x) {
        double wx=0.0, wy=0.0;
        mapToWorld(x,y,wx,wy);
        if (!isLethal(wx,wy)) {cell_cost[y*width_+x]=1.0F+navigationCostAt(wx,wy);}
      }
    }
    pending.emplace(0.0F, goal_index);
    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    while (!pending.empty()) {
      if (searchTimeExpired()) {return false;}
      const auto [current_cost, index] = pending.top();
      pending.pop();
      if (current_cost > holonomic_cost_[index] + 1.0e-5F) {continue;}
      const int x = index % width_;
      const int y = index / width_;
      for (int direction = 0; direction < 8; ++direction) {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {continue;}
        const float next_cost=cell_cost[ny*width_+nx];
        if (!std::isfinite(next_cost)) {continue;}
        const float geometric = resolution_ * (direction >= 4 ? std::sqrt(2.0F) : 1.0F);
        // 同一张二维引导图同时绕开河岸并偏向主航道中心，避免欧氏启发式
        // 在弯曲河道里反复朝岸边扩展。
        const float transition = geometric * next_cost;
        const int next = ny * width_ + nx;
        const float candidate = current_cost + transition;
        if (candidate + 1.0e-5F < holonomic_cost_[next]) {
          holonomic_cost_[next] = candidate;
          pending.emplace(candidate, next);
        }
      }
    }
    holonomic_cache_revision_ = static_map_revision_;
    holonomic_cache_goal_index_ = goal_index;
    holonomic_cache_compact_opposite_ = compact_opposite_maneuver_;
    holonomic_cache_main_entry_=main_entry_active_;
    holonomic_cache_start_x_=search_start_x_;holonomic_cache_start_y_=search_start_y_;
    return true;
  }

  // [功能与联系] 离散化x/y/yaw成为状态索引；有动态目标时search另外加入到达时间桶。
  int getIdx(float x, float y, float theta) const
  {
    int gx = 0;
    int gy = 0;
    if (!worldToMap(x, y, gx, gy)) {
      return -1;
    }
    double angle = std::fmod(theta, 2.0 * M_PI);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    const int heading_index = static_cast<int>(std::round(angle / d_theta_)) % theta_size_;
    return (heading_index * height_ + gy) * width_ + gx;
  }

  // 返回与当前模型匹配的几何曲率上限；USV艏摇率还受最小转弯半径约束。
  double maximumCurvature() const
  {
    if (motion_model_ == MotionModel::USV_KINEMATIC ||
      motion_model_ == MotionModel::USV_DYNAMIC) {
      const double speed = std::max(0.05, planningSpeed());
      const double radius_limited_rate = speed / std::max(0.1, planningRadius());
      return std::min(usv_max_yaw_rate_, radius_limited_rate) / speed;
    }
    return std::tan(std::abs(max_steering_angle_)) /
      std::max(0.1F, wheelbase_);
  }

  double heuristicTurningRadius() const
  {
    return (motion_model_ == MotionModel::USV_KINEMATIC ||
      motion_model_ == MotionModel::USV_DYNAMIC) ?
      1.0 / std::max(1.0e-6, maximumCurvature()) : min_turning_radius_;
  }

  double dubinsTurningRadius() const
  {
    return 1.0 / std::max(1.0e-6, maximumCurvature());
  }

  double curvatureForControl(float turn_control) const
  {
    const double normalized = std::clamp(
      static_cast<double>(turn_control), -1.0, 1.0);
    if (motion_model_ == MotionModel::USV_KINEMATIC ||
      motion_model_ == MotionModel::USV_DYNAMIC) {
      return normalized * maximumCurvature();
    }
    return std::tan(normalized * max_steering_angle_) /
      std::max(0.1F, wheelbase_);
  }

  // Smoothed USV paths must remain inside the same yaw-rate/speed envelope as
  // search primitives; otherwise post-processing falls back to the raw path.
  bool pathRespectsUsvKinematics(
    const std::vector<geometry_msgs::msg::Pose> & path) const
  {
    if (motion_model_ != MotionModel::USV_KINEMATIC) {return true;}
    const double allowed_curvature = maximumCurvature() * 1.20 + 0.01;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double distance = std::hypot(
        path[i].position.x - path[i - 1].position.x,
        path[i].position.y - path[i - 1].position.y);
      if (distance < 1.0e-5) {continue;}
      const double previous_yaw = tf2::getYaw(path[i - 1].orientation);
      const double current_yaw = tf2::getYaw(path[i].orientation);
      const double yaw_change = std::abs(
        hybrid_a_star_planner::normalizeAngle(current_yaw - previous_yaw));
      if (yaw_change / distance > allowed_curvature) {return false;}
    }
    return true;
  }

  // [功能与联系] 按选中模型积分一段定曲率基元；USV控制表示艏摇角速度，汽车控制仍表示前轮转角。
  void simulateDistance(
    float x, float y, float theta, float turn_control, float distance,
    float & nx, float & ny, float & ntheta) const
  {
    const double curvature = curvatureForControl(turn_control);
    const double end=theta+distance*curvature;
    nx = std::abs(curvature)<1e-8 ? x+distance*std::cos(theta) :
      x+(std::sin(end)-std::sin(theta))/curvature;
    ny = std::abs(curvature)<1e-8 ? y+distance*std::sin(theta) :
      y-(std::cos(end)-std::cos(theta))/curvature;
    ntheta = static_cast<float>(hybrid_a_star_planner::normalizeAngle(
      theta + distance * curvature));
  }

  hybrid_a_star_planner::dynamics::State dynamicState(const Node3D & node) const
  {
    return {node.x, node.y, node.theta, node.u, node.v, node.r};
  }

  double dynamicPrimitiveDuration() const
  {
    return std::max(0.1, static_cast<double>(step_size_) / std::max(0.1, planningSpeed()));
  }

  double dynamicYawRateReference(const Node3D & state, float turn_control) const
  {
    // A stationary vessel cannot make a finite-radius ground-track turn. Scale
    // the yaw-rate command by actual body-frame resultant speed (including
    // sway) so a turn grows smoothly with propulsion instead of pivoting at
    // the start of a path. This also keeps the requested turn radius >= R_min.
    const double speed = std::hypot(state.u, state.v);
    const double speed_limited = speed /
      std::max(0.1, planningRadius());
    const double limit = std::min(usv_max_yaw_rate_, speed_limited);
    return std::clamp(static_cast<double>(turn_control), -1.0, 1.0) * limit;
  }

  // Bounded fallback: circle words seed a six-state shooting solve. The only
  // accepted output is re-integrated by primitiveIsSafe, including dynamic
  // obstacles, current task constraints, force limits and measured u/v/r.
  std::vector<geometry_msgs::msg::Pose> rescueShooting(
    float sx,float sy,float stheta,float gx,float gy,float goal_yaw,
    double initial_time,double start_u,double start_v,double start_r)
  {
    if(isLethal(sx,sy) || isLethal(gx,gy)) {return {};}
    const auto expired=[&](){return searchTimeExpired();};
    const dynamics::State start{sx,sy,stheta,start_u,start_v,start_r};
    std::vector<geometry_msgs::msg::Pose> best;
    double best_cost=std::numeric_limits<double>::infinity(),best_speed=0,best_radius=0;
    std::vector<dynamics::State> best_trace;ShootingControl best_control;
    // Only exploit the existing endpoint POSITION tolerance when a bank is
    // close: move the shooting target up to 0.65m toward nearby free water.
    double away_x=0,away_y=0;
    for(double dx:{-4.0,0.,4.0})for(double dy:{-4.0,0.,4.0}) {
      if(isLethal(gx+dx,gy+dy)){away_x-=dx;away_y-=dy;}
    }
    const double away=std::hypot(away_x,away_y);
    const double tx=gx+(away>1e-6?.65*away_x/away:0),ty=gy+(away>1e-6?.65*away_y/away:0);
    const std::vector<double> offsets=away>1e-6 ? std::vector<double>{-.48,.48,0.,-.25,.25}:std::vector<double>{0.,-.25,.25,-.48,.48};
    for(double offset:offsets) {
      if(expired())break;
      const double heading=normalizeAngle(goal_yaw+offset);
      if(std::abs(normalizeAngle(heading-goal_yaw))>goal_heading_tolerance_)continue;
      for(double speed:{std::min(1.2,close_turn_speed_),std::max(rescue_min_speed_,.9),rescue_min_speed_}) {
        if(expired())break;
        attempted_speeds_.push_back(speed);
        for(auto candidate:shootingSeeds(start,tx,ty,heading,speed)) {
          if(expired())break;
          if(candidate.radius<rescue_min_radius_)continue;
          if(!optimizeShooting(start,tx,ty,heading,candidate,usv_dynamics_,expired))continue;
          std::vector<dynamics::State> trace;
          const auto predicted=shootingRollout(start,candidate,usv_dynamics_,&trace);
          if(std::hypot(predicted.x-gx,predicted.y-gy)>.95 ||
            std::abs(normalizeAngle(predicted.yaw-goal_yaw))>goal_heading_tolerance_)continue;
          active_turn_speed_=speed;active_turn_radius_=candidate.radius;
          maneuver_active_=true;tight_maneuver_=true;
          auto node=std::make_shared<Node3D>();node->x=sx;node->y=sy;node->theta=stheta;
          node->u=start_u;node->v=start_v;node->r=start_r;node->t=initial_time;
          double total=0,length=0;bool safe=!isLethal(sx,sy);
          double applied_command=std::clamp(start_r/std::max(.01,std::min(usv_max_yaw_rate_,std::hypot(start_u,start_v)/candidate.radius)),-1.,1.);
          for(int phase=0;phase<4 && safe;++phase) {
            const double duration=phase<3?candidate.duration[phase]:candidate.settle;
            const double target_command=phase<3?candidate.turn[phase]:0;
            const int steps=std::max(1,int(std::ceil(duration/.2)));
            for(int i=0;i<steps && duration>0;++i) {
              if(expired()){safe=false;break;}
              Node3D state;double cost=0,arrival=node->t,travelled=0;
              applied_command=std::clamp(target_command,applied_command-candidate.turn_slew*duration/steps,applied_command+candidate.turn_slew*duration/steps);
              if(!primitiveIsSafe(*node,applied_command,cost,arrival,state,travelled,false,-1,duration/steps)) {safe=false;break;}
              auto next=std::make_shared<Node3D>(state);next->parent=node;node=next;
              total+=travelled+cost;length+=travelled;
            }
          }
          if(!safe || length>std::max<double>(20.,std::hypot(gx-sx,gy-sy))*maximum_path_ratio_+maximum_path_extra_ ||
            std::hypot(node->x-gx,node->y-gy)>.95 ||
            std::abs(normalizeAngle(node->theta-goal_yaw))>goal_heading_tolerance_)continue;
          auto path=extractPath(node);
          if(pathHasAvoidableGoalLoop(path,loop_check_radius_,loop_check_arc_))continue;
          total+=mainLaneEntryCost(path);
          if(total<best_cost) {best_cost=total;best=std::move(path);best_speed=speed;best_radius=candidate.radius;best_trace=std::move(trace);best_control=candidate;}
          // Keep time for the old final-speed grid search if the fallback fails;
          // one validated candidate is enough to rescue a previously empty task.
          if(!best.empty())break;
        }
        if(!best.empty())break;
      }
      if(!best.empty())break;
    }
    if(!best.empty()) {
      active_turn_speed_=best_speed;active_turn_radius_=best_radius;
      maneuver_active_=true;tight_maneuver_=true;last_shooting_trace_=std::move(best_trace);
      rescue_selected_=true;c_turn_selected_=false;last_shooting_control_=best_control;
    }
    return best;
  }

  // [功能与联系] C弯连接采用“定向转艏脉冲+自然收舵”的六状态射击，不直接
  // 输出几何Dubins。先按模型计算转向位移，再解入弯/出弯直段长度；180度
  // 目标通过二分转艏幅度匹配横向位移。全路径从实测状态重新逐子步验安全。
  std::vector<geometry_msgs::msg::Pose> connectMonotoneTurn(
    const Node3D &start,double gx,double gy,double goal_yaw,double *cost_out=nullptr) const
  {
    constexpr double dt=0.2;
    auto prefix=std::make_shared<Node3D>(start);
    double accumulated_cost=0;
    auto advance=[&](std::shared_ptr<Node3D> &node,float command,double duration) {
      Node3D next;double cost=0,arrival=node->t,length=0;
      if(searchTimeExpired() || !primitiveIsSafe(*node,command,cost,arrival,next,length,false,-1,duration)) {return false;}
      accumulated_cost+=length+cost+colregsCost(next.theta,arrival)+
        steering_change_weight_*std::abs(command-node->turn_control)+
        steeringMagnitudeWeightForGoal(start.theta,goal_yaw)*length*command*command;
      auto child=std::make_shared<Node3D>(next);child->parent=node;node=child;return true;
    };
    // 仅低速起步/残余侧滑需要短过渡。高速不先强迫直航等到参考速度，
    // 直接在转弯积分中减速，避免“等待减速的直段”反而扩大掉头空间。
    const double entry_speed=std::min(1.2,std::max(.1,planningSpeed()-.01));
    // 正常航点交接允许带着实测侧滑/转艏率直接入弯；不能为了等待
    // v/r归零而先直航数十米。运动积分仍保留它们，并逐步验碰撞。
    for(int i=0;i<80 && prefix->u<entry_speed;++i) {
      if(!advance(prefix,0,dt)) {return {};}
    }
    const double heading=prefix->theta;
    const double prefix_cost=accumulated_cost;
    const double delta=hybrid_a_star_planner::normalizeAngle(goal_yaw-heading);
    const double x=(gx-prefix->x)*std::cos(heading)+(gy-prefix->y)*std::sin(heading);
    const double y=-(gx-prefix->x)*std::sin(heading)+(gy-prefix->y)*std::cos(heading);
    struct Turn {Node3D end;std::vector<float> commands;};
    double yaw_bias=0.0;
    auto predict=[&](const Node3D &entry,int sign,double alpha) {
      Turn out;out.end=entry;bool off=false;double angle=0,time=0;
      // 在统一的终点艏向容差内采样；输出仍须满足原目标方向的验收。
      const double entry_delta=hybrid_a_star_planner::normalizeAngle(goal_yaw-entry.theta);
      double directed_delta=sign*entry_delta;
      if(lane_turn_direction_==sign && directed_delta<0) {directed_delta+=2*M_PI;}
      const double required=(std::abs(std::abs(entry_delta)-M_PI)<.05 ? M_PI : directed_delta)+yaw_bias;
      const double budget=std::min(240.0,3.0*required/
        std::max(.015,alpha*dynamicYawRateReference(entry,1))+20.0);
      for(int i=0;i<static_cast<int>(budget/dt);++i) {
        if(searchTimeExpired()) {break;}
        if(required-angle<=(usv_dynamics_.yaw_response_time+0.5*dt)*sign*out.end.r) {off=true;}
        const float command=off ? 0.0F : static_cast<float>(sign*alpha*std::min(1.0,(time+dt)/1.0));
        const auto state=hybrid_a_star_planner::dynamics::stepSetpoint(dynamicState(out.end),planningSpeed(),
          dynamicYawRateReference(out.end,command),usv_dynamics_,dt);
        angle+=sign*hybrid_a_star_planner::normalizeAngle(state.yaw-out.end.theta);
        out.end.x=state.x;out.end.y=state.y;out.end.theta=state.yaw;
        out.end.u=state.u;out.end.v=state.v;out.end.r=state.r;out.commands.push_back(command);time+=dt;
        if(off && std::abs(state.r)<.001 && std::abs(state.v)<.015) {break;}
      }
      return out;
    };
    auto lengths=[&](const Turn &curve) {
      const double dx=(curve.end.x-prefix->x)*std::cos(heading)+(curve.end.y-prefix->y)*std::sin(heading);
      const double dy=-(curve.end.x-prefix->x)*std::sin(heading)+(curve.end.y-prefix->y)*std::cos(heading);
      const double end_course=curve.end.theta-heading+std::atan2(curve.end.v,std::max(.1,curve.end.u));
      const double a=std::cos(end_course),b=std::sin(end_course);
      double before=0,after=0,residual=0;
      if(std::abs(std::sin(delta))<.05) {
        // 接近180度时矩阵退化，横向位移靠转艏幅度匹配；纵向可由非负直段配平。
        after=std::max(0.0,(x-dx)/std::min(-.5,a));before=std::max(0.0,x-dx-after*a);
        residual=dy+after*b-y;
      } else {
        after=(y-dy)/b;before=x-dx-after*a;
      }
      return std::array<double,3>{before,after,residual};
    };
    // 旧候选最多偏置0.18rad，即使容差为30度也会漏掉合法单向弯，
    // 随后回退到低速多弧连接。采样覆盖已配置的容差，不扩大验收范围。
    std::vector<double> yaw_biases{0.0, 0.08, 0.16, 0.18,
      goal_heading_tolerance_ * 0.5, goal_heading_tolerance_ * 0.75,
      goal_heading_tolerance_ * 0.95};
    std::sort(yaw_biases.begin(), yaw_biases.end());
    yaw_biases.erase(std::unique(yaw_biases.begin(), yaw_biases.end()), yaw_biases.end());
    for(const double bias:yaw_biases) {
    yaw_bias=bias;
    const int first_sign=lane_turn_direction_==0 ? 1:lane_turn_direction_;
    for(int sign:{first_sign,-first_sign}) {
      if(std::abs(std::abs(delta)-M_PI)>=.05 && sign*delta<=0 && lane_turn_direction_!=sign) {continue;}
      std::vector<double> alphas;
      if(std::abs(std::sin(delta))<.05) {
        if(sign*y<=0.1) {continue;} // 无横向空间的同轴反向姿态一般不能用单一C弯满足。
        double lo=.05,hi=1;
        double e_lo=lengths(predict(*prefix,sign,lo))[2],e_hi=lengths(predict(*prefix,sign,hi))[2];
        RCLCPP_DEBUG(rclcpp::get_logger("HybridAStar"),"C shooting v=%.2f R=%.2f y=%.2f bracket=%.2f,%.2f",
          planningSpeed(),planningRadius(),y,e_lo,e_hi);
        if(e_lo*e_hi>0 && std::abs(e_hi)>.35) {continue;}
        if(std::abs(e_hi)<=.35) {alphas={hi};}
        else {
        for(int i=0;i<14 && !searchTimeExpired();++i) {
          const double mid=.5*(lo+hi),error=lengths(predict(*prefix,sign,mid))[2];
          if(error*e_lo>0) {lo=mid;e_lo=error;} else {hi=mid;}
        }
        alphas={.5*(lo+hi)};
        }
      } else {alphas=special_maneuver_ ? std::vector<double>{1.0,.8,.65,.5,.35,.25,.15} :
          std::vector<double>{.15,.25,.35,.5,.65,.8,1.0};}
      for(const double alpha:alphas) {
        if(searchTimeExpired()) {return {};}
        const auto curve=predict(*prefix,sign,alpha);auto distance=lengths(curve);
        if(distance[0]<-.05 || distance[1]<-.05 || std::abs(distance[2])>.35 ||
          std::abs(hybrid_a_star_planner::normalizeAngle(curve.end.theta-goal_yaw))>goal_heading_tolerance_) {continue;}
        auto current=prefix;
        accumulated_cost=prefix_cost;
        auto straight=[&](double length) {
          double travelled=0;
          // 毫米级尾差不再调用“位移必须>1mm”的运动基元，避免安全候选
          // 因浮点尾差被误拒绝；5mm远小于0.4m端点验收。
          while(travelled<length-0.005) {
            const double old_x=current->x,old_y=current->y;
            const double duration=std::min(dt,(length-travelled)/std::max(.1,std::hypot(current->u,current->v)));
            if(!advance(current,0,duration)) {return false;}
            travelled+=std::hypot(current->x-old_x,current->y-old_y);
          }
          return true;
        };
        // 前置直段会改变u/v/r，原稳态几何解的L/F不能直接用于实船。
        // 用轻量无碰撞预测迭代修正入弯直段；最终仍逐段执行硬安全检查。
        for(int iteration=0;iteration<5;++iteration) {
          Node3D entry=*prefix;double travelled=0;
          while(travelled<distance[0]-.005 && !searchTimeExpired()) {
            const double duration=std::min(dt,(distance[0]-travelled)/std::max(.1,std::hypot(entry.u,entry.v)));
            const auto state=hybrid_a_star_planner::dynamics::stepSetpoint(dynamicState(entry),planningSpeed(),
              dynamicYawRateReference(entry,0),usv_dynamics_,duration);
            travelled+=std::hypot(state.x-entry.x,state.y-entry.y);
            entry.x=state.x;entry.y=state.y;entry.theta=state.yaw;entry.u=state.u;entry.v=state.v;entry.r=state.r;
          }
          const auto endpoint=predict(entry,sign,alpha).end;
          const double ex=gx-endpoint.x,ey=gy-endpoint.y;
          const double course=endpoint.theta+std::atan2(endpoint.v,std::max(.1,endpoint.u));
          const double determinant=std::sin(course-heading);
          if(std::abs(determinant)<.05) {break;}
          const double after=(ey*std::cos(heading)-ex*std::sin(heading))/determinant;
          const double correction=(ex*std::sin(course)-ey*std::cos(course))/determinant;
          distance[1]=after;
          if(std::abs(correction)<.03) {break;}
          distance[0]=std::max(0.0,distance[0]+correction);
        }
        if(distance[1]<-.05 || searchTimeExpired() || !straight(std::max(0.0,distance[0]))) {continue;}
        // 入弯直段后重新预测，保留实际u/v/r而非强制重置为稳态。
        const auto actual=predict(*current,sign,alpha);bool safe=true;
        for(float command:actual.commands) {if(!advance(current,command,dt)) {safe=false;break;}}
        if(!safe || !straight(std::max(0.0,distance[1]))) {continue;}
        RCLCPP_DEBUG(rclcpp::get_logger("HybridAStar"),"C shooting end v=%.2f alpha=%.4f L=%.2f F=%.2f end=%.2f,%.2f yaw=%.3f",
          planningSpeed(),alpha,distance[0],distance[1],current->x,current->y,current->theta);
        if(std::hypot(current->x-gx,current->y-gy)>.4 ||
          std::abs(hybrid_a_star_planner::normalizeAngle(current->theta-goal_yaw))>goal_heading_tolerance_) {continue;}
        auto path=extractPath(current);double length=0,previous_course=0,left=0,right=0;
        bool has_course=false;
        for(std::size_t i=1;i<path.size();++i) {
          const double dx=path[i].position.x-path[i-1].position.x,dy=path[i].position.y-path[i-1].position.y;
          const double ds=std::hypot(dx,dy);length+=ds;if(ds<.001) {continue;}
          const double course=std::atan2(dy,dx);
          if(has_course) {const double change=hybrid_a_star_planner::normalizeAngle(course-previous_course);
            left+=std::max(0.0,change);right+=std::max(0.0,-change);}
          previous_course=course;has_course=true;
        }
        const double required=std::max({1.0,std::hypot(gx-start.x,gy-start.y),planningRadius()*std::abs(delta)});
        if(length<=required*maximum_path_ratio_+maximum_path_extra_ && std::min(left,right)<.04 &&
          !pathHasAvoidableGoalLoop(path,loop_check_radius_,loop_check_arc_)) {
          if(cost_out) {*cost_out=accumulated_cost;}return path;
        }
      }
    }
    }
    return {};
  }

  // [功能与联系] Hybrid搜索找到可达路径后尝试宽弧优化：Bezier仅作引导，
  // 输出坐标仍由同一个欠驱动船模、实际初始u/v/r逐步积分；不直接移动路径点。
  // 每一步复用primitiveIsSafe的全部静态/动态/COLREG检查，候选代价必须低于
  // 搜索结果才接受。受窄河弯或障碍限制时保持原搜索路径及原最小半径能力。
  // maneuver=true使用Dubins作为掉头引导，允许安全借道，尝试末端侧滑切线偏置；
  // 输出仍是RK4真实船模积分；位置与硬安全门槛不变，艏向使用统一配置容差。
  std::vector<geometry_msgs::msg::Pose> refineBroadChannelCurve(
    const Node3D & start, double goal_x, double goal_y, double goal_yaw,
    double search_cost, bool maneuver=false, bool relax_center=false, bool entry_guide=false) const
  {
    const double chord = std::hypot(goal_x - start.x, goal_y - start.y);
    if (!maneuver && chord < 10.0) {return {};}
    const double forward = (goal_x-start.x)*std::cos(start.theta) +
      (goal_y-start.y)*std::sin(start.theta);
    const double lateral = -(goal_x-start.x)*std::sin(start.theta) +
      (goal_y-start.y)*std::cos(start.theta);
    // 后方偏侧目标的尾部修正容易反复反向。近同轴/原地转向需要
    // 多弧连接，不能用这项偏好降低其可达性。
    const bool stabilize_rear_turn = chord >= 10.0 && forward < 0.0 && std::abs(lateral) > 1.0;
    auto connection_start=std::make_shared<Node3D>(start);
    double prefix_cost=0.0;
    if(maneuver && close_turn_search_ && tight_maneuver_ && start.u>planningSpeed()+0.2) {
      // 高速近距离掉头最多1秒短过渡，之后在转弯中继续减速，不能先直航
      // 4秒等待稳态。所有段从实测状态积分并验安全，不是耗时补偿起点。
      for(int step=0;step<4 && connection_start->u>planningSpeed()+0.2;++step) {
        Node3D propagated;double arrival=connection_start->t,length=0.0,cost=0.0;
        if(searchTimeExpired() || !primitiveIsSafe(*connection_start,0.0F,cost,
            arrival,propagated,length,false,-1,0.25)) {return {};}
        auto next=std::make_shared<Node3D>(propagated);next->parent=connection_start;
        prefix_cost+=length+cost;connection_start=next;
      }
    }
    const double start_course = connection_start->theta +
      std::atan2(connection_start->v,std::max(0.1,connection_start->u));
    const double effort_weight = steeringMagnitudeWeightForGoal(start.theta, goal_yaw);
    const bool stay_in_main_lane = !maneuver && !special_maneuver_ && laneRegionAt(start.x, start.y) == Region::MAIN &&
      laneRegionAt(goal_x, goal_y) == Region::MAIN;
    const int center_limit = static_cast<int>(std::lround(
      main_lane_max_ * channel_curve_center_fraction_));
    // 中部起终点的宽弧优化不得为了增大半径而贴边；该质量门槛只约束
    // 优化候选，原搜索仍可为窄弯/避障靠边或借道。
    const int maximum_lane_cost = !relax_center && stay_in_main_lane &&
      costAt(lane_costmap_, start.x, start.y) <= center_limit &&
      costAt(lane_costmap_, goal_x, goal_y) <= center_limit ? center_limit : -1;
    double best_cost = search_cost;
    std::shared_ptr<Node3D> best;
    const int guide_samples = std::clamp(static_cast<int>(std::ceil(chord)), 30, 600);
    const int max_steps = std::clamp(static_cast<int>(
      std::ceil(2.0 * chord / std::max(0.1F, step_size_))) + 30, 30, 1200);
    for (const double first_handle : {0.25, 0.4, 0.55, 0.7}) {
      // 20m左右的正交目标也必须用短前视/控制外推连接；6~10m远前视
      // 无法准确跟踪低速3m半径，会漏过目标并产生不必要的大绕圈。
      const bool compact=maneuver && (close_turn_search_ ||
        (special_maneuver_ && chord<=compact_guidance_distance_));
      // Under the bounded entry budget, try short entry/exit handles first.
      // Long alignment tails remain available, but cannot starve the early
      // main-lane candidates simply because they occur first in the list.
      const std::vector<double> last_handles=compact ?
        (entry_guide ? std::vector<double>{0.3,0.6,0.0,-0.3,-0.6,1.0,2.0,3.0} :
        std::vector<double>{1.0,2.0,3.0,-0.6,-0.3,0.0,0.3,0.6}) : std::vector<double>{0.25,0.4,0.55,0.7};
      for (const double last_handle : last_handles) {
        const bool terminal_straight=compact && last_handle>=1.0;
        const double lookahead=compact ? (terminal_straight ? 4.0 :
          std::min(4.0,0.8*dubinsTurningRadius())) : (maneuver ? (last_handle==0.7 ? 10.0 : 6.0) : 12.0);
        // 艏向不等于地面航迹切线：末端仍可能有侧滑，尝试少量切线偏置，
        // 但最终接受条件仍严格比较原始目标艏向，不修改用户目标。
        const double guide_bias=compact ? (terminal_straight ? 0.0 : last_handle) :
          (maneuver && last_handle!=0.7 ? 2.0*(last_handle-0.4) : 0.0);
        // Entry candidates must mirror their terminal course bias with the
        // main-lane side; a fixed positive bias favors only one physical side.
        const double guide_goal_course=goal_yaw+guide_bias*
          (entry_guide && lane_turn_direction_ ? lane_turn_direction_:1);
        if (searchTimeExpired()) {return best ? extractPath(best) : std::vector<geometry_msgs::msg::Pose>{};}
        const double p1x = start.x + chord * first_handle * std::cos(start_course);
        const double p1y = start.y + chord * first_handle * std::sin(start_course);
        const double p2x = goal_x - chord * last_handle * std::cos(goal_yaw);
        const double p2y = goal_y - chord * last_handle * std::sin(goal_yaw);
        std::vector<std::pair<double, double>> guide;
        double guide_length=chord;
        guide.reserve(guide_samples + 1U);
        for (int i = 0; i <= guide_samples; ++i) {
          const double t = static_cast<double>(i) / guide_samples, q = 1.0 - t;
          guide.emplace_back(q*q*q*start.x + 3.0*q*q*t*p1x + 3.0*q*t*t*p2x + t*t*t*goal_x,
            q*q*q*start.y + 3.0*q*q*t*p1y + 3.0*q*t*t*p2y + t*t*t*goal_y);
        }
        if(maneuver && !entry_guide) {
          auto space=std::make_shared<ompl::base::DubinsStateSpace>(
            dubinsTurningRadius()*(first_handle/0.25));
          ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space),s(space);
          a->setXY(connection_start->x,connection_start->y);a->setYaw(start_course);
          // 近距离反向目标常需RLR/LRL，最短几何曲线末端仍在转弯且侧滑未消失。
          // 增加“先到终点前方的对准点，再顺目标方向进入”的候选，给侧滑自然衰减空间。
          const double tail=terminal_straight ? std::max(4.0,planningSpeed()*2.0)+2.0*(last_handle-1.0) : 0.0;
          const double approach_x=goal_x-tail*std::cos(goal_yaw);
          const double approach_y=goal_y-tail*std::sin(goal_yaw);
          b->setXY(approach_x,approach_y);b->setYaw(guide_goal_course);
          const double length=space->distance(a.get(),b.get());
          guide_length=length+tail;
          const int samples=std::clamp(static_cast<int>(std::ceil(length/0.5)),20,2000);
          guide.clear();bool first=true;ompl::base::DubinsStateSpace::DubinsPath curve;
          for(int i=0;i<=samples;++i) {
            space->interpolate(a.get(),b.get(),static_cast<double>(i)/samples,first,curve,s.get());
            guide.emplace_back(s->getX(),s->getY());
          }
          if(terminal_straight) {
            const int count=std::max(1,static_cast<int>(std::ceil(tail/0.5)));
            for(int i=1;i<=count;++i) {
              const double along=tail*static_cast<double>(i)/count;
              guide.emplace_back(approach_x+along*std::cos(goal_yaw),approach_y+along*std::sin(goal_yaw));
            }
          }
        }
        auto current = std::make_shared<Node3D>(*connection_start);
        double cost = prefix_cost;
        std::size_t nearest = 0U;
        // 近距离大艏向差的可行航程可远大于直线距离，不能沿用短弦长的步数预算。
        const double maximum_duration=2.5*guide_length/std::max(0.5,planningSpeed())+10.0;
        const int candidate_steps=maneuver ? std::clamp(static_cast<int>(
          std::ceil(maximum_duration/0.1)),max_steps,5000) : max_steps;
        for (int step = 0; step < candidate_steps; ++step) {
          if (searchTimeExpired()) {break;}
          if(maneuver && current->t-start.t>maximum_duration) {break;}
          const double remaining = std::hypot(goal_x - current->x, goal_y - current->y);
          // 机动端点收紧到0.4m，避免规划位置容差叠加MPC的0.5m减速容差后停得太远。
          if (remaining < (maneuver ? 0.4 : 1.0) && std::abs(hybrid_a_star_planner::normalizeAngle(
              goal_yaw - current->theta)) < goal_heading_tolerance_)
          {
            if(maneuver) {
              // 在常速分支内先做质量验收，异常长的近距离候选才能继续尝试低速，
              // 不能先返回给节点拒绝，再下一周期重复同一条常速长绕路。
              auto candidate=extractPath(current);double length=0.0;
              for(std::size_t i=1;i<candidate.size();++i) {
                length+=std::hypot(candidate[i].position.x-candidate[i-1].position.x,
                  candidate[i].position.y-candidate[i-1].position.y);
              }
              // 近距离180度机动的必要航程不能用几乎为零的弦长衡量。
              // R*艏向差是转向尺度下界；仍保留长度上限及“已到达却继续绕圈”的拒绝。
              const double required=std::max({1.0,chord,planningRadius()*std::abs(
                hybrid_a_star_planner::normalizeAngle(goal_yaw-start.theta))});
              if(length>required*maximum_path_ratio_+maximum_path_extra_ ||
                pathHasAvoidableGoalLoop(candidate,loop_check_radius_,loop_check_arc_)) {
                RCLCPP_DEBUG(rclcpp::get_logger("HybridAStar"),
                  "Rejected maneuver: length=%.2f, chord=%.2f, speed=%.2f",length,chord,planningSpeed());
                break;
              }
            }
            if (cost < best_cost) {best_cost = cost; best = current;}
            break;
          }
          double nearest_distance = std::numeric_limits<double>::infinity();
          // 掉头曲线可能与自身接近，最近点只能沿引导局部前进，不能跳到后半段。
          const std::size_t nearest_end=maneuver ? std::min(guide.size(),nearest+17U) : guide.size();
          for (std::size_t i = nearest; i < nearest_end; ++i) {
            const double d = std::hypot(guide[i].first - current->x, guide[i].second - current->y);
            if (d < nearest_distance) {nearest_distance = d; nearest = i;}
          }
          std::size_t target = nearest;
          double distance_ahead = 0.0;
          while (target + 1U < guide.size() && distance_ahead < lookahead) {
            distance_ahead += std::hypot(guide[target+1U].first - guide[target].first,
              guide[target+1U].second - guide[target].second);
            ++target;
          }
          double tx = guide[target].first, ty = guide[target].second;
          if (target + 1U == guide.size() && distance_ahead < lookahead) {
            // 延续终点切线，避免前视距离在终点缩为零而突然急打舵/绕圈。
            tx += (lookahead - distance_ahead) * std::cos(guide_goal_course);
            ty += (lookahead - distance_ahead) * std::sin(guide_goal_course);
          }
          const double course = current->theta + std::atan2(current->v, std::max(0.1, current->u));
          const double bearing = std::atan2(ty - current->y, tx - current->x);
          const double speed = std::hypot(current->u, current->v);
          const double rate = 2.0 * speed * std::sin(
            hybrid_a_star_planner::normalizeAngle(bearing - course)) /
            std::max(1.0, std::hypot(tx - current->x, ty - current->y));
          const double limit = dynamicYawRateReference(*current, 1.0F);
          float control = limit > 1.0e-6 ?
            static_cast<float>(std::clamp(rate / limit, -1.0, 1.0)) : 0.0F;
          if(compact && tight_maneuver_) {
            // 近距离RLR/LRL转向含相邻反向弧，单纯追一个远前视点会切角或绕圈。
            // 用同一六状态船模短时外推，选与引导位置/航迹切线匹配的连续艏摇指令。
            // 这是连接候选内的小规模控制采样，不替代Hybrid，也不跳过硬安全检查。
            double selected_cost=std::numeric_limits<double>::infinity();
            for(float command:std::vector<float>{-1.0F,-0.75F,-0.5F,-0.25F,0.0F,
                0.25F,0.5F,0.75F,1.0F,control}) {
              Node3D predicted=*current;std::size_t index=nearest;
              // 局部指令选择也要考虑连续性和反向，不能只在整条候选
              // 完成后才罚反向。六个预测点累计位置误差，反向项按六点
              // 归一化；这是软偏好，必要的单次S弯仍可胜出。
              double score=(stabilize_rear_turn ? std::max(0.08,steering_change_weight_) : 0.08)*
                std::pow(command-current->turn_control,2);
              if(stabilize_rear_turn && command*current->turn_direction<0) {
                score+=turn_reversal_weight_/6.0;
              }
              for(int rollout=0;rollout<6;++rollout) {
                auto state=hybrid_a_star_planner::dynamics::stepSetpoint(dynamicState(predicted),
                  planningSpeed(),dynamicYawRateReference(predicted,command),usv_dynamics_,0.25);
                predicted.x=state.x;predicted.y=state.y;predicted.theta=state.yaw;
                predicted.u=state.u;predicted.v=state.v;predicted.r=state.r;
                double distance=std::numeric_limits<double>::infinity();
                const auto end=std::min(guide.size(),index+9U);
                for(auto i=index;i<end;++i) {
                  const double d=std::hypot(guide[i].first-state.x,guide[i].second-state.y);
                  if(d<distance) {distance=d;index=i;}
                }
                const auto before=std::min(index,guide.size()-2U);
                const auto after=std::min(before+3U,guide.size()-1U);
                const double tangent=std::atan2(guide[after].second-guide[before].second,
                  guide[after].first-guide[before].first);
                const double predicted_course=state.yaw+std::atan2(state.v,std::max(0.1,state.u));
                const double error=hybrid_a_star_planner::normalizeAngle(tangent-predicted_course);
                score+=4.0*distance*distance+5.0*error*error;
              }
              score-=0.5*static_cast<double>(index-nearest);
              if(score<selected_cost) {selected_cost=score;control=command;}
            }
          }
          Node3D propagated;
          double proximity_cost = 0.0, arrival = current->t, length = 0.0;
          // 起点本就靠近终点时仍需先转向；只有沿引导接近末段才缩短积分，
          // 避免在错误艏向下反复做0.1s短步、耗光引导预算。
          const double duration=compact ? std::min(0.35,dynamicPrimitiveDuration()) :
            (remaining<8.0 && target+1U==guide.size() ?
              std::clamp(0.5*remaining/std::max(0.5,speed),0.1,dynamicPrimitiveDuration()) : 0.0);
          if (!primitiveIsSafe(*current, control, proximity_cost, arrival, propagated, length,
              stay_in_main_lane, maximum_lane_cost,duration)) {break;}
          const int direction = control > 1.0e-4F ? 1 : (control < -1.0e-4F ? -1 : 0);
          const bool reverses = direction != 0 && current->turn_direction != 0 &&
            direction != current->turn_direction;
          cost += length + proximity_cost + colregsCost(propagated.theta, arrival) +
            steering_change_weight_ * std::abs(control - current->turn_control) +
            effort_weight * length * control * control + (reverses ? turn_reversal_weight_ : 0.0);
          if (cost >= best_cost || nearest_distance > 15.0) {break;}
          auto next = std::make_shared<Node3D>(propagated);
          next->turn_direction = direction != 0 ? direction : current->turn_direction;
          next->parent = current;
          current = next;
        }
      }
    }
    return best ? extractPath(best) : std::vector<geometry_msgs::msg::Pose>{};
  }

  // [功能与联系] 对子基元沿途加密取样，累积时间及区域/动态软代价，硬拒绝同步碰撞和规则违规；不是只检查末端。
  bool primitiveIsSafe(
    const Node3D & parent, float turn_control, double & cost,double & arrival,
    Node3D & result, double & travelled, bool stay_in_main_lane = false,
    int maximum_lane_cost = -1,double duration_override=0.0) const
  {
    cost = 0.0;
    double time=parent.t,px=parent.x,py=parent.y;
    travelled = 0.0;
    auto sampleIsSafe = [&](double x, double y, double heading,
        double sample_time, double ds) {
      if (isLethal(x, y) || ((stay_in_main_lane || strict_main_lane_) && laneRegionAt(x, y) != Region::MAIN)) {
        return false;
      }
      if (maximum_lane_cost >= 0 && costAt(lane_costmap_, x, y) > maximum_lane_cost) {return false;}
      cost += navigationCostAt(x, y) * ds;
      if(lane_turn_direction_!=0) {
        const double dx=x-search_start_x_,dy=y-search_start_y_;
        const double lateral=-dx*std::sin(search_heading_)+dy*std::cos(search_heading_);
        const double blend=std::max(0.0,1.0-std::hypot(dx,dy)/30.0);
        cost+=lane_turn_weight_*blend*std::max(0.0,-lane_turn_direction_*lateral)/5.0*ds;
      }
      for (const auto & obstacle : dynamic_obstacles_) {
        const bool heading_allowed = usesUsvDynamicModel() ?
          hybrid_a_star_planner::headingAllowedDynamic(obstacle.policy, heading,
            sample_time, search_heading_, usv_max_yaw_rate_,
            1.2 * usv_dynamics_.yaw_response_time) :
          hybrid_a_star_planner::headingAllowed(
            obstacle.policy,heading,sample_time,0,search_heading_);
        if (!heading_allowed)
          return false;
        // Never drop a target at the horizon boundary: long overtaking paths
        // can still be alongside it after 120 s.
        const double target_x = obstacle.state.x + obstacle.state.vx * sample_time;
        const double target_y = obstacle.state.y + obstacle.state.vy * sample_time;
        const double distance = std::hypot(x - target_x, y - target_y);
        const double safe_distance = std::max({own_ship_radius_ + obstacle.state.radius + safety_buffer_, dynamic_hard_distance_,
          obstacle.policy.type==EncounterType::OVERTAKING ? overtaking_safe_distance_ : 0.0});
        if (distance < safe_distance) {
          return false;
        }
        if(!headOnStarboardPassAllowed(
            obstacle,x,y,heading,target_x,target_y,safe_distance)) return false;
        if(obstacle.policy.locked && obstacle.policy.type==EncounterType::CROSSING_STARBOARD) {
          const double speed=std::hypot(obstacle.state.vx,obstacle.state.vy);
          if(speed>0.05) {
            const double dx=x-target_x,dy=y-target_y;
            const double along=(dx*obstacle.state.vx+dy*obstacle.state.vy)/speed;
            const double across=(dx*obstacle.state.vy-dy*obstacle.state.vx)/speed;
            if(std::abs(across)<safe_distance && along> -safe_distance) return false;
          }
        }
        const double soft_distance = std::max({safe_distance,avoidance_radius_,
          (own_ship_radius_ + obstacle.state.radius + safety_buffer_)*soft_distance_factor_});
        if ((obstacle.policy.locked || obstacle.encounter.active) &&
          distance < soft_distance && soft_distance > safe_distance) {
          cost += dynamic_weight_ *
            (soft_distance - distance) / (soft_distance - safe_distance) /
            ds / std::max(1.0, soft_distance - safe_distance);
        }
      }
      return true;
    };

    if (usesUsvDynamicModel()) {
      const double duration = duration_override>0.0 ? duration_override : dynamicPrimitiveDuration();
      const double dt = std::max(0.01, usv_dynamics_.integration_step);
      const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));
      const double substep = duration / samples;
      auto state = dynamicState(parent);
      for (int sample = 0; sample < samples; ++sample) {
        const auto previous_state = state;
        state = hybrid_a_star_planner::dynamics::stepSetpoint(
          state, planningSpeed(), dynamicYawRateReference(parent, turn_control),
          usv_dynamics_, substep);
        const double ds = std::hypot(state.x - px, state.y - py);
        time += substep;
        travelled += ds;
        if (!sampleIsSafe(state.x, state.y, state.yaw, time, ds)) {return false;}
        // Penalize hull sideslip as an accumulated squared slip angle. This
        // favors broad, dynamically natural C turns while leaving obstacle
        // clearance and COLREG checks as hard constraints.
        const double slip_angle = std::atan2(std::abs(state.v),
          std::max(0.5, std::abs(state.u)));
        cost += sideslip_weight_ * slip_angle * slip_angle * ds;
        if (prefer_broad_channel_curve_ && ds > 1.0e-4) {
          const double previous_course = previous_state.yaw +
            std::atan2(previous_state.v, std::max(0.1, previous_state.u));
          const double course = state.yaw + std::atan2(state.v, std::max(0.1, state.u));
          const double curvature = hybrid_a_star_planner::normalizeAngle(
            course - previous_course) / ds;
          const double normalized_curvature = channel_curve_radius_ * curvature;
          cost += channel_curve_weight_ * (large_turn_search_ ? 0.03 : 1.0) * normalized_curvature * normalized_curvature * ds;
        }
        px = state.x;
        py = state.y;
      }
      result.x = static_cast<float>(state.x);
      result.y = static_cast<float>(state.y);
      result.theta = static_cast<float>(state.yaw);
      result.u = state.u;
      result.v = state.v;
      result.r = state.r;
      result.t = time;
      result.turn_control = turn_control;
      arrival = time;
      return travelled > 1.0e-3;
    }

    const int samples = std::max(2, static_cast<int>(std::ceil(
      step_size_ / std::max(0.25F, 0.5F * resolution_))));
    for (int sample = 1; sample <= samples; ++sample) {
      const float ratio = static_cast<float>(sample) / samples;
      float x = 0.0F;
      float y = 0.0F;
      float heading = 0.0F;
      simulateDistance(parent.x, parent.y, parent.theta, turn_control,
        step_size_ * ratio, x, y, heading);
      const double ds = std::hypot(x - px, y - py);
      time += estimateSegmentTime(px,py,x,y,ds);
      travelled += ds;
      if (!sampleIsSafe(x, y, heading, time, ds)) {return false;}
      px = x;
      py = y;
    }
    arrival=time;
    result.x = px;
    result.y = py;
    result.theta = static_cast<float>(hybrid_a_star_planner::normalizeAngle(
      parent.theta + travelled * curvatureForControl(turn_control)));
    result.u = parent.u;
    result.v = parent.v;
    result.r = parent.r;
    result.t = time;
    result.turn_control = turn_control;
    return true;
  }

  // [功能与联系] 累计违背会遇转向偏好的软惩罚；headingAllowed与净空检查仍可硬拒绝分支。
  double colregsCost(double heading, double time) const
  {
    double cost = 0.0;
    for (const auto & obstacle : dynamic_obstacles_) {
      if (!obstacle.encounter.active || obstacle.policy.recovering || time > prediction_horizon_ ||
        time > obstacle.encounter.tcpa + 8.0)
      {
        continue;
      }
      cost += hybrid_a_star_planner::headingPreferenceCost(
        heading, obstacle.policy.reference, obstacle.encounter.preference,
        obstacle.encounter.urgency, colregs_weight_, colregs_deadband_);
    }
    return cost;
  }

  // [功能与联系] 沿父节点回溯生成起点到终点的姿态序列；供后处理和Path输出。
  std::vector<geometry_msgs::msg::Pose> extractPath(std::shared_ptr<Node3D> node) const
  {
    std::vector<geometry_msgs::msg::Pose> path;
    std::vector<std::shared_ptr<Node3D>> nodes;
    for(auto p=node;p;p=p->parent) nodes.push_back(p);
    std::reverse(nodes.begin(),nodes.end());
    if (nodes.empty()) {return path;}
    auto appendPose = [&path](double x, double y, double yaw) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.orientation = tf2::toMsg(q);
      path.push_back(pose);
    };
    appendPose(nodes.front()->x, nodes.front()->y, nodes.front()->theta);
    for (std::size_t i = 1; i < nodes.size(); ++i) {
      if (usesUsvDynamicModel()) {
        auto state = dynamicState(*nodes[i - 1U]);
        const double duration = nodes[i]->t-nodes[i-1]->t;
        const double dt = std::max(0.01, usv_dynamics_.integration_step);
        const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));
        const double substep = duration / samples;
        for (int j = 0; j < samples; ++j) {
          state = hybrid_a_star_planner::dynamics::stepSetpoint(
            state, planningSpeed(), dynamicYawRateReference(*nodes[i - 1U], nodes[i]->turn_control),
            usv_dynamics_, substep);
          appendPose(state.x, state.y, state.yaw);
        }
      } else {
        const int samples = static_cast<int>(std::ceil(step_size_ / 0.2));
        for (int j = 1; j <= samples; ++j) {
          float x = 0.0F, y = 0.0F, heading = 0.0F;
          simulateDistance(nodes[i - 1U]->x, nodes[i - 1U]->y, nodes[i - 1U]->theta,
            nodes[i]->turn_control, step_size_ * j / samples, x, y, heading);
          appendPose(x, y, heading);
        }
      }
    }
    return path;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr lane_costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_land_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr channel_obstacle_map_;
  std::vector<DynamicObstacle> dynamic_obstacles_;
  int width_{0};
  int height_{0};
  float resolution_{0.1F};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  float step_size_{0.2F};
  float min_turning_radius_{0.5F};
  float wheelbase_{1.5F};
  float max_steering_angle_{0.8F};
  float d_theta_{0.15F};
  int theta_size_{42};
  int max_iterations_{20000};
  double search_time_limit_{0.0};
  bool fast_guided_connection_{false};
  bool large_turn_enabled_{false},maneuver_active_{false},large_turn_search_{false};
  bool close_turn_search_{false},tight_maneuver_{false};
  double close_turn_range_{15.0},close_turn_speed_{1.2},close_turn_radius_{3.0},close_turn_angle_{0.7853981634};
  double large_turn_speed_{2.0},large_turn_radius_{5.0},large_turn_angle_{1.57};
  bool prefer_single_turn_{true};
  bool c_turn_selected_{false};

  double active_turn_speed_{0.0},active_turn_radius_{0.0};
  std::chrono::steady_clock::time_point overall_deadline_{std::chrono::steady_clock::time_point::max()};
  bool rescue_enabled_{true},rescue_selected_{false};
  double rescue_time_{1.2},rescue_min_speed_{.6},rescue_min_radius_{1.5};
  std::vector<dynamics::State> last_shooting_trace_;
  ShootingControl last_shooting_control_;
  double requested_speed_{4.0};
  int speed_samples_{10};
  std::vector<double> attempted_speeds_;
  bool heuristic_attempted_{false},heuristic_ready_{false};
  bool compact_opposite_maneuver_{false};
  double lane_turn_weight_{30.0},compact_opposite_scale_{0.1};
  int lane_turn_direction_{0};
  MainLaneDistanceField main_lane_distance_;
  bool main_entry_active_{false};
  double main_entry_weight_{30.0},main_entry_direct_distance_{30.0},main_entry_horizon_{30.0};
  double speed_stage_time_limit_{0.12};
  double goal_heading_tolerance_{kDefaultGoalHeadingTolerance};
  double maneuver_straight_limit_{6.0},maneuver_detour_ratio_{1.7};
  double compact_guidance_distance_{30.0};
  bool special_maneuver_{false};
  mutable bool strict_main_lane_{false};
  double maximum_path_ratio_{3.0},maximum_path_extra_{40.0},loop_check_radius_{3.0},loop_check_arc_{8.0};
  std::chrono::steady_clock::time_point search_deadline_;
  std::shared_ptr<std::atomic<bool>> search_cancel_;
  int lethal_cost_threshold_{60};
  bool channel_enabled_{false};
  int shoreline_threshold_{88};
  int channel_obstacle_threshold_{85};
  int main_lane_max_{20};
  int opposite_lane_min_{65};
  double outside_lane_weight_{2.5};
  double opposite_lane_weight_{4.0};
  double main_center_weight_{2.0};
  double channel_curve_radius_{60.0};
  double channel_curve_weight_{1.0};
  double channel_curve_center_fraction_{0.6};
  bool prefer_broad_channel_curve_{false};
  double outside_map_weight_{2.0};
  double channel_obstacle_weight_{5.0};
  double analytic_expansion_distance_{18.0};
  MotionModel motion_model_{MotionModel::USV_DYNAMIC};
  std::string motion_model_name_{"USV_DYNAMIC"};
  double usv_max_yaw_rate_{0.45};
  bool gentle_turn_sampling_{true};
  double cruise_speed_{4.0};
  hybrid_a_star_planner::dynamics::Parameters usv_dynamics_;
  double own_ship_radius_{0.5};
  double safety_buffer_{1.0};
  double dynamic_hard_distance_{2.0};
  double prediction_horizon_{120.0};
  double soft_distance_factor_{4.0};
  double avoidance_radius_{7.0};
  double dynamic_weight_{16.0};
  double overtaking_safe_distance_{3.0};
  double colregs_weight_{12.0};
  std::vector<float> holonomic_cost_;
  std::size_t static_map_revision_{0U};
  std::size_t holonomic_cache_revision_{std::numeric_limits<std::size_t>::max()};
  int holonomic_cache_goal_index_{-1};
  bool holonomic_cache_compact_opposite_{false};
  // Entry cost is tied to the live start and task role, not just map/goal.
  bool holonomic_cache_main_entry_{false};
  double holonomic_cache_start_x_{0},holonomic_cache_start_y_{0};
  std::size_t holonomic_build_count_{0U};
  std::vector<geometry_msgs::msg::Pose> reference_path_;
  std::vector<float> reference_distance_;
  bool reference_distance_dirty_{true};
  double reference_path_weight_{0.5};
  double reference_path_max_distance_{12.0};
  double steering_change_weight_{1.0};
  double steering_magnitude_weight_{12.0};
  double turnaround_heading_threshold_{2.0943951024};
  double turnaround_steering_scale_{0.25};
  double turn_reversal_weight_{24.0};
  double sideslip_weight_{6.0};
  double near_field_reference_weight_{12.0};
  double near_field_reference_horizon_{45.0};
  double near_field_deviation_scale_{3.0};
  double colregs_deadband_{0.05};
  double search_heading_{0.0};
  double search_time_offset_{0.0};
  double search_start_x_{0},search_start_y_{0},search_goal_x_{0},search_goal_y_{0};
  double recovery_weight_{0},recovery_horizon_{6},arrival_deceleration_{0.5};
  double arrival_min_speed_{0.3},arrival_goal_tolerance_{0.4};
  bool recovery_risk_{false},arrival_braking_{false};
  bool recovery_complete_{false};
  bool reference_soft_conflict_{false};
  bool enforce_head_on_starboard_{true};
};

}  // namespace hybrid_a_star_planner::planning
