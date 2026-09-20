#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
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
#include "hybrid_a_star_planner/policy.hpp"
#include "hybrid_a_star_planner/msg/encounter_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hybrid_a_star_planner/colregs.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using hybrid_a_star_planner::EncounterAssessment;
using hybrid_a_star_planner::EncounterType;
using hybrid_a_star_planner::VesselState;

namespace
{

struct DynamicObstacle
{
  std::string name;
  VesselState state;
  EncounterAssessment encounter;
  hybrid_a_star_planner::Policy policy;
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
  float steering{0};
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
    double steering_change_weight, double steering_magnitude_weight = 0.0)
  {
    reference_path_weight_ = std::max(0.0, reference_path_weight);
    reference_path_max_distance_ = std::max(0.1, reference_path_max_distance);
    steering_change_weight_ = std::max(0.0, steering_change_weight);
    steering_magnitude_weight_ = std::max(0.0, steering_magnitude_weight);
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
    reference_path_ = path;
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
  // [功能与联系] 返回search更新后的策略/目标快照；节点将恢复标志同步发布给DWA，避免两级约束不一致。
  const std::vector<DynamicObstacle> & dynamicObstacles() const {return dynamic_obstacles_;}

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
  // 与DWA终点速度上限一致的名义到达时间；最低预测速度避免终点除零。
  // 这是预测模型，不是强制本船速度，也不构成实船动力学保证。
  // [功能与联系] 由段长及巡航/终点制动模型估计耗时；运动基元、尾段和平滑复查使用相同时间基准。
  double estimateSegmentTime(double ax,double ay,double bx,double by,double length) const {
    double speed=cruise_speed_;
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
  }

  // [功能与联系] 注入原始陆地核心层；航道内清除岸边膨胀也不能覆盖真实陆地。
  void setStaticLandMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    static_land_map_ = map;
  }

  // [功能与联系] 注入独立障碍膨胀层；航道自由水域规则不能消除这一来源的障碍。
  void setChannelObstacleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
  {
    channel_obstacle_map_ = map;
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
  std::vector<geometry_msgs::msg::Pose> search(
    float sx, float sy, float stheta, float gx, float gy, float gtheta)
  {
    search_heading_=stheta;
    search_start_x_=sx;search_start_y_=sy;search_goal_x_=gx;search_goal_y_=gy;
    // 当前CPA变安全不等于可以安全恢复。按恢复方向的时间同步净空
    // 决定是否加连续性软代价；每次事件搜索重算，不靠固定等待锁航向。
    recovery_risk_=false;
    const double recovery_length=std::hypot(gx-sx,gy-sy);
    const int recovery_samples=std::max(1,static_cast<int>(std::ceil(recovery_length)));
    double recovery_time=0,px=sx,py=sy;
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
      o.policy.recovering=o.policy.safeToRecover(sx,sy,o.state,safe,recovery_risk_);
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
      double reference_time=0.0,reference_length=0.0,rx=sx,ry=sy;
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
    if (isLethal(gx, gy) || !buildHolonomicHeuristic(gx, gy)) {
      RCLCPP_WARN(rclcpp::get_logger("HybridAStar"), "Goal is outside the reachable water region");
      return {};
    }
    int start_grid_x = 0;
    int start_grid_y = 0;
    worldToMap(sx, sy, start_grid_x, start_grid_y);
    if (!std::isfinite(holonomic_cost_[start_grid_y * width_ + start_grid_x])) {
      RCLCPP_WARN(rclcpp::get_logger("HybridAStar"), "No static-water connection to the goal");
      return {};
    }

    const std::size_t total_states = static_cast<std::size_t>(width_) *
      static_cast<std::size_t>(height_) * static_cast<std::size_t>(theta_size_);
    std::unordered_map<std::uint64_t,float> g_values;
    // 没有动态船时，时间不会改变碰撞结果。若仍把时间放入状态键，同一
    // (x,y,yaw) 会以不同到达时刻反复进入 open set，远目标很容易耗尽迭代数。
    // 只有动态避碰真正需要时间同步时才扩展时间维度。
    const bool use_time_dimension = !dynamic_obstacles_.empty();
    auto key=[total_states, use_time_dimension](int index,double time) {
      const auto time_bucket = use_time_dimension ?
        static_cast<std::uint64_t>(std::floor(time / 2.0)) : 0U;
      return static_cast<std::uint64_t>(index) + total_states * time_bucket;
    };
    std::priority_queue<
      std::shared_ptr<Node3D>, std::vector<std::shared_ptr<Node3D>>, CompareNode> open_set;

    auto start = std::make_shared<Node3D>();
    start->x = sx;
    start->y = sy;
    start->theta = stheta;
    start->h = heuristic(sx, sy, stheta, gx, gy, gtheta);
    start->f = start->h;
    start->idx = getIdx(sx, sy, stheta);
    if (start->idx < 0) {
      return {};
    }
    g_values[key(start->idx,0)] = 0.0F;
    open_set.push(start);

    const std::vector<float> steerings = {
      -max_steering_angle_, -max_steering_angle_ * 0.6F,
      -max_steering_angle_ * 0.3F, 0.0F,
      max_steering_angle_ * 0.3F, max_steering_angle_ * 0.6F,
      max_steering_angle_};

    int iterations = 0;
    while (!open_set.empty() && iterations++ < max_iterations_) {
      const auto current = open_set.top();
      open_set.pop();
      if (current->g > g_values[key(current->idx,current->t)] + 1.0e-5F) {
        continue;
      }
      const float distance_to_goal = std::hypot(current->x - gx, current->y - gy);
      const float angle_difference = std::abs(hybrid_a_star_planner::normalizeAngle(
        gtheta - current->theta));
      // 解析连接会立即返回，若距离很远会绕过航道软代价，因此只在终点附近使用。
      if(distance_to_goal < analytic_expansion_distance_ && iterations%10==1) {
        auto tail=connect(*current,gx,gy,gtheta);
        if(!tail.empty()) {
          auto path=extractPath(current);
          path.insert(path.end(),tail.begin(),tail.end());
          return path;
        }
      }
      if (distance_to_goal < 0.6F && angle_difference < 0.15F) {
        return extractPath(current);
      }

      for (const float steering : steerings) {
        float nx = 0.0F;
        float ny = 0.0F;
        float ntheta = 0.0F;
        simulateDistance(
          current->x, current->y, current->theta, steering, step_size_, nx, ny, ntheta);
        double proximity_cost = 0.0;
        double arrival_time=current->t;
        if (!primitiveIsSafe(*current, steering, proximity_cost,arrival_time)) {
          continue;
        }
        const int next_index = getIdx(nx, ny, ntheta);
        if (next_index < 0) {
          continue;
        }
        // 大惯性船舶不宜在相邻运动基元间频繁反向打舵。该项只抑制曲率
        // 跳变，不禁止为了避障或COLREG进行必要的大幅机动。
        const double steering_change = steering_change_weight_ *
          std::abs(steering - current->steering) /
          std::max(0.05F, max_steering_angle_);
        // 变化不频繁但长期保持大舵角同样会形成宽大的S弯。平方项优先奖励
        // 直行和小舵角；它只是软代价，障碍物及COLREG仍可要求大幅转向。
        const double steering_ratio = steering / std::max(0.05F, max_steering_angle_);
        const double steering_magnitude = steering_magnitude_weight_ * step_size_ *
          steering_ratio * steering_ratio;
        const float next_g = current->g + step_size_ + static_cast<float>(
          proximity_cost + colregsCost(ntheta, arrival_time) + steering_change +
          steering_magnitude);
        const auto state_key=key(next_index,arrival_time);
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
        // A modest weighted heuristic keeps one-second replanning practical on wide maps.
        next->f = next->g + 1.5F * next->h;
        next->t = arrival_time;
        next->steering=steering;
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

  // [功能与联系] 统一弧长采样并迭代短/长尺度平滑；每次候选由pathIsSafe复查，不安全缩步或退回原安全结果。
  std::vector<geometry_msgs::msg::Pose> postProcessPath(
    const std::vector<geometry_msgs::msg::Pose> & raw, double requested_spacing,
    int iterations, double smooth_weight, double data_weight, double max_deviation,
    int long_range_points = 0, double long_range_weight = 0.0) const
  {
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
    if (!pathIsSafe(resampled)) {
      // 重采样弦线若切入障碍区，完全退回已由Hybrid A*验证过的原路径。
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
      if (pathIsSafe(candidate)) {
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
          if (pathIsSafe(reduced)) {
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
    const double initial_lateral=-std::sin(policy.reference)*
      (search_start_x_-obstacle.state.x)+std::cos(policy.reference)*
      (search_start_y_-obstacle.state.y);
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
    double elapsed = 0.0;
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

  // [功能与联系] 生成近终点Dubins尾段，限制过长绕行，并沿曲线复查静态/动态/规则安全；不是可绕过碰撞检查的直连。
  std::vector<geometry_msgs::msg::Pose> connect(const Node3D &n,double gx,double gy,double yaw) const {
    auto space=std::make_shared<ompl::base::DubinsStateSpace>(wheelbase_/std::tan(max_steering_angle_));
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
    } else if (region == Region::OUTSIDE) {
      cost += outside_lane_weight_;
    } else if (region == Region::OPPOSITE) {
      cost += opposite_lane_weight_;
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

          // 连续两次搜索最影响操船的是本船近前方：DWA前视点若在这里从
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

  // [功能与联系] 结合二维可通行代价场与运动学距离估计剩余搜索成本；用于open队列优先级。
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
    return water_distance + 0.2F * angle * min_turning_radius_;
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
    const float infinity = std::numeric_limits<float>::infinity();
    holonomic_cost_.assign(static_cast<std::size_t>(width_ * height_), infinity);
    using QueueItem = std::pair<float, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pending;
    const int goal_index = gy * width_ + gx;
    holonomic_cost_[goal_index] = 0.0F;
    pending.emplace(0.0F, goal_index);
    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    while (!pending.empty()) {
      const auto [current_cost, index] = pending.top();
      pending.pop();
      if (current_cost > holonomic_cost_[index] + 1.0e-5F) {continue;}
      const int x = index % width_;
      const int y = index / width_;
      for (int direction = 0; direction < 8; ++direction) {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {continue;}
        double wx = 0.0;
        double wy = 0.0;
        mapToWorld(nx, ny, wx, wy);
        if (isLethal(static_cast<float>(wx), static_cast<float>(wy))) {continue;}
        const float geometric = resolution_ * (direction >= 4 ? std::sqrt(2.0F) : 1.0F);
        // 同一张二维引导图同时绕开河岸并偏向主航道中心，避免欧氏启发式
        // 在弯曲河道里反复朝岸边扩展。
        const float transition = geometric * static_cast<float>(
          1.0 + navigationCostAt(wx, wy));
        const int next = ny * width_ + nx;
        const float candidate = current_cost + transition;
        if (candidate + 1.0e-5F < holonomic_cost_[next]) {
          holonomic_cost_[next] = candidate;
          pending.emplace(candidate, next);
        }
      }
    }
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

  // [功能与联系] 按船舶简化转向模型积分一小段基元，得到子节点姿态；被primitiveIsSafe及扩展逻辑调用。
  void simulateDistance(
    float x, float y, float theta, float steering, float distance,
    float & nx, float & ny, float & ntheta) const
  {
    const double curvature=std::tan(steering)/wheelbase_;
    const double end=theta+distance*curvature;
    nx = std::abs(curvature)<1e-8 ? x+distance*std::cos(theta) :
      x+(std::sin(end)-std::sin(theta))/curvature;
    ny = std::abs(curvature)<1e-8 ? y+distance*std::sin(theta) :
      y-(std::cos(end)-std::cos(theta))/curvature;
    ntheta = static_cast<float>(hybrid_a_star_planner::normalizeAngle(
      theta + (distance / wheelbase_) * std::tan(steering)));
  }

  // [功能与联系] 对子基元沿途加密取样，累积时间及区域/动态软代价，硬拒绝同步碰撞和规则违规；不是只检查末端。
  bool primitiveIsSafe(const Node3D & parent, float steering, double & cost,double & arrival) const
  {
    const int samples = std::max(2, static_cast<int>(std::ceil(
      step_size_ / std::max(0.25F, 0.5F * resolution_))));
    cost = 0.0;
    double time=parent.t,px=parent.x,py=parent.y;
    for (int sample = 1; sample <= samples; ++sample) {
      const float ratio = static_cast<float>(sample) / samples;
      float x = 0.0F;
      float y = 0.0F;
      float heading = 0.0F;
      simulateDistance(
        parent.x, parent.y, parent.theta, steering, step_size_ * ratio, x, y, heading);
      if (isLethal(x, y)) {
        return false;
      }
      cost += navigationCostAt(x, y) * step_size_ /
        static_cast<double>(samples);
      time+=estimateSegmentTime(px,py,x,y,step_size_/samples);px=x;py=y;
      for (const auto & obstacle : dynamic_obstacles_) {
        if (!hybrid_a_star_planner::headingAllowed(obstacle.policy,heading,time,0,search_heading_))
          return false;
        // Never drop a target at the horizon boundary: long overtaking paths
        // can still be alongside it after 120 s.
        const double target_x = obstacle.state.x + obstacle.state.vx * time;
        const double target_y = obstacle.state.y + obstacle.state.vy * time;
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
            static_cast<double>(samples);
        }
      }
    }
    arrival=time;
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
    for(std::size_t i=0;i<nodes.size();++i) {
      const int samples=i==0 ? 1 : static_cast<int>(std::ceil(step_size_/0.2));
      for(int j=1;j<=samples;++j) {
        float x=nodes[i]->x,y=nodes[i]->y,heading=nodes[i]->theta;
        if(i) simulateDistance(nodes[i-1]->x,nodes[i-1]->y,nodes[i-1]->theta,
          nodes[i]->steering,step_size_*j/samples,x,y,heading);
        geometry_msgs::msg::Pose p;
        p.position.x=x;p.position.y=y;
        tf2::Quaternion q;q.setRPY(0,0,heading);p.orientation=tf2::toMsg(q);
        path.push_back(p);
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
  int lethal_cost_threshold_{60};
  bool channel_enabled_{false};
  int shoreline_threshold_{88};
  int channel_obstacle_threshold_{85};
  int main_lane_max_{20};
  int opposite_lane_min_{65};
  double outside_lane_weight_{2.5};
  double opposite_lane_weight_{4.0};
  double main_center_weight_{2.0};
  double outside_map_weight_{2.0};
  double channel_obstacle_weight_{5.0};
  double analytic_expansion_distance_{18.0};
  double cruise_speed_{0.8};
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
  std::vector<geometry_msgs::msg::Pose> reference_path_;
  std::vector<float> reference_distance_;
  double reference_path_weight_{0.5};
  double reference_path_max_distance_{12.0};
  double steering_change_weight_{1.0};
  double steering_magnitude_weight_{0.0};
  double near_field_reference_weight_{12.0};
  double near_field_reference_horizon_{45.0};
  double near_field_deviation_scale_{3.0};
  double colregs_deadband_{0.05};
  double search_heading_{0.0};
  double search_start_x_{0},search_start_y_{0},search_goal_x_{0},search_goal_y_{0};
  double recovery_weight_{0},recovery_horizon_{6},arrival_deceleration_{0.5};
  double arrival_min_speed_{0.3},arrival_goal_tolerance_{0.4};
  bool recovery_risk_{false},arrival_braking_{false};
  bool recovery_complete_{false};
  bool reference_soft_conflict_{false};
  bool enforce_head_on_starboard_{true};
};

class HybridAStarNode : public rclcpp::Node
{
public:
  // [功能与联系] 加载搜索/航道/动态/平滑参数，建立TF、各安全输入和风险输出。
  // timer持续做安全监测，但事件模式只在路径风险变化时运行Hybrid A*搜索。
  HybridAStarNode()
  : Node("hybrid_a_star_node")
  {
    const double frequency = declare_parameter("planner_frequency", 2.0);
    path_failure_hold_time_ = declare_parameter("path_failure_hold_time", 3.0);
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
    declare_parameter("d_theta", 0.15);
    declare_parameter("max_iterations", 20000);
    declare_parameter("lethal_cost_threshold", 60);

    channel_enabled_ = declare_parameter("channel.enabled", true);
    const int shoreline_threshold = declare_parameter("channel.shoreline_cost_threshold", 88);
    const int channel_obstacle_threshold = declare_parameter(
      "channel.obstacle_cost_threshold", 85);
    const int main_lane_max = declare_parameter("channel.main_lane_value_max", 20);
    const int opposite_lane_min = declare_parameter("channel.opposite_lane_value_min", 65);
    const double outside_lane_weight = declare_parameter("channel.outside_lane_weight", 2.5);
    const double opposite_lane_weight = declare_parameter("channel.opposite_lane_weight", 4.0);
    const double main_center_weight = declare_parameter("channel.main_center_weight", 2.0);
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
      "stability.steering_magnitude_weight", 1.0);
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

    cruise_speed_ = declare_parameter("cruise_speed", 0.8);
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
    planner_->setParams(
      get_parameter("step_size").as_double(), get_parameter("min_turning_radius").as_double(),
      get_parameter("wheelbase").as_double(), get_parameter("max_steering_angle").as_double(),
      get_parameter("d_theta").as_double(), static_cast<int>(get_parameter("max_iterations").as_int()),
      static_cast<int>(get_parameter("lethal_cost_threshold").as_int()));
    planner_->setDynamicParams(
      cruise_speed_, own_ship_radius_, safety_buffer_, prediction_horizon_, soft_factor,
      avoidance_radius_,
      dynamic_weight, colregs_weight, colregs_deadband, overtaking_distance);
    planner_->setDynamicHardDistance(dynamic_hard_distance_);
    planner_->setChannelParams(
      channel_enabled_, shoreline_threshold, channel_obstacle_threshold,
      main_lane_max, opposite_lane_min, outside_lane_weight, opposite_lane_weight,
      main_center_weight, outside_map_weight, channel_obstacle_weight,
      analytic_expansion_distance);
    planner_->setStabilityParams(
      reference_path_weight, reference_path_max_distance, steering_change_weight,
      steering_magnitude_weight);
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
          if(route->poses.empty()) {
            has_goal_=false;last_planning_path_.clear();publishEmpty();
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
    encounter_pub_ = create_publisher<std_msgs::msg::String>("/colregs/encounter", 10);
    risk_marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/colregs/risk_markers",10);
    avoidance_envelope_pub_=create_publisher<visualization_msgs::msg::Marker>("/colregs/avoidance_envelope",10);
    policy_pub_=create_publisher<hybrid_a_star_planner::msg::EncounterArray>("/colregs/policies",10);
    own_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",20,
      [this](nav_msgs::msg::Odometry::SharedPtr m){ own_odom_=m; });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, frequency)),
      std::bind(&HybridAStarNode::planningTimerCallback, this));
    RCLCPP_INFO(
      get_logger(), "Hybrid A* ready: %zu target topic(s), horizon %.1f s",
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

  // [功能与联系] 接收活动目标并使旧任务路径失效；下一次安全监测timer触发一次搜索，本回调不驱动本船。
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
      return;
    }
    goal_ = *message;
    has_goal_ = true;
    // 新目标不能继续执行旧目标路径；只在目标切换这一刻清空一次。之后偶发
    // 单周期搜索失败会短暂保留最近有效路径，避免 RViz 周期性闪烁。
    has_valid_path_for_goal_ = false;
    last_planning_path_.clear();
    last_output_path_.poses.clear();
    planner_->setReferencePath({});
    publishEmpty();
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
  // 变得不安全时执行搜索；安全缓存路径只刷新时间戳供DWA持续跟踪。
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
    if (path.size() < 4) return false;
    const auto & goal = path.back().position;
    for (std::size_t i = 1; i + 2 < path.size(); ++i) {
      if (std::hypot(path[i].position.x-goal.x, path[i].position.y-goal.y) > goal_loop_radius_) continue;
      double remaining = 0.0;
      for (std::size_t j = i + 1; j < path.size(); ++j) {
        remaining += std::hypot(path[j].position.x-path[j-1].position.x,
          path[j].position.y-path[j-1].position.y);
      }
      if (remaining > goal_loop_min_arc_) return true;
    }
    return false;
  }

  bool pathQualityAcceptable(const std::vector<geometry_msgs::msg::Pose> & path,
    double sx, double sy, double gx, double gy) const
  {
    if (path.size() < 2 || pathHasGoalLoop(path)) return false;
    const double direct = std::max(1.0, std::hypot(gx-sx, gy-sy));
    const double length = pathLength(path);
    return length <= direct * max_path_length_ratio_ + max_path_extra_distance_;
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
      base_pose.header.frame_id = base_frame_;
      base_pose.header.stamp = latest;
      base_pose.pose.orientation.w = 1.0;
      geometry_msgs::msg::PoseStamped start;
      tf_buffer_->transform(base_pose, start, planning_frame, timeout);

      geometry_msgs::msg::PoseStamped goal = goal_;
      goal.header.stamp = latest;
      geometry_msgs::msg::PoseStamped transformed_goal;
      tf_buffer_->transform(goal, transformed_goal, planning_frame, timeout);
      const double reference_heading = tf2::getYaw(start.pose.orientation);
      // 远目标搜索可能占用约1秒；单线程执行器在搜索期间不能派发 odom 回调。
      // 因此这里使用独立容忍时间，避免把计算阻塞误判为传感器失联并交替清空路径。
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
      const bool path_requires_replan = !has_valid_path_for_goal_ || new_encounter_event ||
        planner_->remainingPathNeedsReplan(
          last_planning_path_, start.pose.position.x, start.pose.position.y,
          reference_heading, event_collision_trigger_distance_, event_cross_track_limit_, false);
      if (event_driven_enabled_ && !path_requires_replan) {
        // 当前轨迹仍安全时不运行搜索。历史会遇锁仍可保留诊断，但目标已安全
        // 通过时立即标记恢复，DWA不必等待release计时才继续跟踪原避让轨迹。
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

      // 新任务、静态阻断、预测碰撞、规则冲突或明显偏航才执行完整搜索。
      // 上一条轨迹仍只是安全可覆盖的软参考，不是不可突破的硬走廊。
      planner_->setReferencePath(last_planning_path_);

      auto poses = planner_->search(
        start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation),
        transformed_goal.pose.position.x, transformed_goal.pose.position.y,
        tf2::getYaw(transformed_goal.pose.orientation));
      // 对遇首先强制右舷通过。只有地形或障碍令右侧走廊完全无解时，
      // 才按“安全优先于规则”临时放开侧向限制执行一次回退搜索。
      if(poses.empty() && std::any_of(obstacles.begin(),obstacles.end(),[](const auto &o){
          return o.policy.locked && !o.policy.recovering &&
            o.policy.type==hybrid_a_star_planner::EncounterType::HEAD_ON;})) {
        planner_->setHeadOnStarboardEnforced(false);
        poses=planner_->search(
          start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation),
          transformed_goal.pose.position.x,transformed_goal.pose.position.y,
          tf2::getYaw(transformed_goal.pose.orientation));
        planner_->setHeadOnStarboardEnforced(true);
        if(!poses.empty()) RCLCPP_WARN(
          get_logger(),"Starboard head-on corridor infeasible; safety fallback used");
      }
      // A single deterministic search can occasionally select a poor but valid
      // branch.  Run up to three bounded candidates and retain the shortest
      // non-looping one.  Later attempts deliberately remove the soft previous
      // path reference, while all hard safety/COLREG checks remain active.
      if (!poses.empty() && event_search_attempts_ > 1) {
        std::vector<geometry_msgs::msg::Pose> best = poses;
        double best_length = pathLength(best);
        for (int attempt = 1; attempt < event_search_attempts_; ++attempt) {
          planner_->setDynamicObstacles(obstacles);
          planner_->setReferencePath({});
          auto candidate = planner_->search(
            start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation),
            transformed_goal.pose.position.x, transformed_goal.pose.position.y,
            tf2::getYaw(transformed_goal.pose.orientation));
          if (!candidate.empty() && pathQualityAcceptable(candidate,
              start.pose.position.x, start.pose.position.y,
              transformed_goal.pose.position.x, transformed_goal.pose.position.y)) {
            const double length = pathLength(candidate);
            if (length < best_length) { best = std::move(candidate); best_length = length; }
          }
        }
        poses = std::move(best);
        planner_->setReferencePath(last_planning_path_);
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
          for(auto &track:target_tracks_) {
            if(track.policy.standOn()) {
              track.policy.takeover=true;
              track.policy.level=hybrid_a_star_planner::Risk::EMERGENCY;
            }
          }
          planner_->setDynamicObstacles(obstacles);
          poses=planner_->search(
            start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation),
            transformed_goal.pose.position.x,transformed_goal.pose.position.y,
            tf2::getYaw(transformed_goal.pose.orientation));
          RCLCPP_WARN(get_logger(),"Stand-on corridor infeasible; Rule 17 takeover activated");
        }
      }
      obstacles=planner_->dynamicObstacles();
      syncRecoveryFlags(obstacles);
      publishPolicies(planning_frame,obstacles);
      if (poses.empty()) {
        const bool traffic_relevant=std::any_of(obstacles.begin(),obstacles.end(),[this](const auto &o){
          return o.policy.locked || o.policy.metric.range<=risk_thresholds_.monitor_range;});
        handlePlanningFailure(traffic_relevant);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Hybrid A* found no safe path");
        return;
      }
      if (smoothing_enabled_) {
        poses = planner_->postProcessPath(
          poses, smoothing_spacing_, smoothing_iterations_, smoothing_weight_,
          smoothing_data_weight_, smoothing_max_deviation_,
          smoothing_long_range_points_, smoothing_long_range_weight_);
      }
      if (!pathQualityAcceptable(poses, start.pose.position.x, start.pose.position.y,
          transformed_goal.pose.position.x, transformed_goal.pose.position.y)) {
        const bool traffic_relevant = std::any_of(obstacles.begin(), obstacles.end(),
          [](const auto & o) { return o.policy.locked || o.encounter.active; });
        handlePlanningFailure(traffic_relevant);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Rejected looping/excessively long Hybrid A* path near goal; waiting for a safer replan");
        return;
      }
      publishAvoidanceEnvelope(planning_frame,poses,std::any_of(
        obstacles.begin(),obstacles.end(),[](const auto &o){return o.policy.locked;}));
      nav_msgs::msg::Path path;
      path.header.frame_id = global_frame_;
      path.header.stamp = now();
      path.poses.reserve(poses.size());
      for (const auto & pose : poses) {
        geometry_msgs::msg::PoseStamped source;
        source.header.frame_id = planning_frame;
        source.header.stamp = latest;
        source.pose = pose;
        geometry_msgs::msg::PoseStamped output;
        tf_buffer_->transform(source, output, global_frame_, timeout);
        output.header = path.header;
        path.poses.push_back(output);
      }
      path_pub_->publish(path);
      last_planning_path_ = poses;
      last_output_path_ = path;
      has_valid_path_for_goal_ = true;
      last_path_success_time_ = now();
      recordPlannedEncounters(obstacles);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Published V4 constrained path with %zu poses", poses.size());
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Planning transform failed; retaining last RViz path: %s", exception.what());
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
  // [功能与联系] 将每艘目标的会遇类型、风险、恢复状态及运动状态发布给DWA，统一两级动态约束。
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
  // [功能与联系] 显式发布空局部Path以撤销任务并使DWA停车；普通搜索失败采用短过期保护，不周期性擦除RViz结果。
  void publishEmpty() {
    nav_msgs::msg::Path p;p.header.frame_id=global_frame_;p.header.stamp=now();
    if(path_pub_)path_pub_->publish(p);
  }
  // [功能与联系] 不重新搜索，只给同一条全局坐标路径刷新消息时间戳；
  // 防止DWA把事件模式下仍有效的路径误判为过期，不改变任何轨迹点。
  void refreshCachedPath() {
    if(!has_valid_path_for_goal_ || last_output_path_.poses.size()<2U) return;
    last_output_path_.header.stamp=now();
    for(auto &pose:last_output_path_.poses) pose.header=last_output_path_.header;
    path_pub_->publish(last_output_path_);
  }
  // [功能与联系] search内部算出的即时恢复状态写回目标轨迹状态机，确保下一次
  // 只做轻量监测时，Hybrid和DWA仍使用同一会遇阶段。
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
  // [功能与联系] 区分动态交通与普通静态搜索失败：附近有船时立即撤销未经本周期
  // 验证的旧路径并通知DWA停车；无动态交通时才短暂保留显示，避免静态搜索抖动。
  void handlePlanningFailure(bool traffic_present) {
    if(traffic_present) {
      // 动态交通存在时绝不继续执行一条已无法重新验证的旧直线路径。
      // 下一周期照常重试；本周期显式通知DWA停车，而不是等待路径超时。
      has_valid_path_for_goal_=false;
      last_planning_path_.clear();
      planner_->setReferencePath({});
      publishEmpty();
      return;
    }
    if (!has_valid_path_for_goal_ ||
      (now() - last_path_success_time_).seconds() > path_failure_hold_time_)
    {
      has_valid_path_for_goal_ = false;
    }
  }
  bool inputs_valid_{false};
  nav_msgs::msg::Odometry::SharedPtr own_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_sub_;
  rclcpp::Publisher<hybrid_a_star_planner::msg::EncounterArray>::SharedPtr policy_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> target_subs_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr encounter_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr avoidance_envelope_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
  double path_failure_hold_time_{3.0};
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
  std::vector<geometry_msgs::msg::Pose> last_planning_path_;
  nav_msgs::msg::Path last_output_path_;
  std::unique_ptr<HybridAStarPlanner> planner_;
};

}  // namespace

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridAStarNode>());
  rclcpp::shutdown();
  return 0;
}
