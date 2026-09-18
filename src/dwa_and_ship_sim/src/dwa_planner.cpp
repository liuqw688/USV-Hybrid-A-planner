#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "hybrid_a_star_planner/msg/encounter_array.hpp"
#include "hybrid_a_star_planner/policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class DwaPlanner : public rclcpp::Node
{
public:
  // [功能与联系] 读取控制/动态窗口/风险/航道参数，订阅Hybrid路径和策略，建立控制timer与RViz输出；不负责生成稀疏全局航点。
  DwaPlanner()
  : Node("dwa_planner")
  {
    // 话题参数
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/rrt_star_path");
    costmap_topic_ = declare_parameter<std::string>(
      "costmap_topic", "/local_costmap");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel");
    control_frame_ = declare_parameter<std::string>(
      "control_frame", "odom");
    channel_enabled_ = declare_parameter<bool>("channel.enabled", true);
    const std::string lane_costmap_topic = declare_parameter<std::string>(
      "channel.lane_costmap_topic", "/channel/lane_costmap");
    const std::string channel_obstacle_topic = declare_parameter<std::string>(
      "channel.obstacle_costmap_topic", "/channel/obstacle_costmap");
    main_lane_value_max_ = declare_parameter<int>("channel.main_lane_value_max", 20);
    opposite_lane_value_min_ = declare_parameter<int>("channel.opposite_lane_value_min", 65);
    center_cost_scale_ = declare_parameter<double>("channel.center_cost_scale", 2.5);
    channel_obstacle_threshold_ = declare_parameter<int>(
      "channel.obstacle_cost_threshold", 85);

    // 运动学参数
    control_frequency_ = declare_parameter<double>(
      "control_frequency", 20.0);
    max_vel_x_ = declare_parameter<double>(
      "max_vel_x", 0.8);
    min_vel_x_ = declare_parameter<double>(
      "min_vel_x", 0.0);
    max_vel_theta_ = declare_parameter<double>(
      "max_vel_theta", 1.2);
    acc_lim_x_ = declare_parameter<double>(
      "acc_lim_x", 0.5);
    goal_deceleration_ = declare_parameter<double>("goal_deceleration", 0.5);
    acc_lim_theta_ = declare_parameter<double>(
      "acc_lim_theta", 1.8);

    // DWA 候选轨迹参数
    sim_time_ = declare_parameter<double>(
      "sim_time", 2.0);
    sim_granularity_ = declare_parameter<double>(
      "sim_granularity", 0.05);
    v_samples_ = declare_parameter<int>(
      "v_samples", 11);
    w_samples_ = declare_parameter<int>(
      "w_samples", 31);

    // 路径跟踪参数
    lookahead_distance_ = declare_parameter<double>(
      "lookahead_distance", 2.0);
    max_lookahead_distance_ = declare_parameter<double>(
      "max_lookahead_distance", 10.0);
    goal_tolerance_ = declare_parameter<double>(
      "goal_tolerance", 0.4);
    min_cruise_speed_ = declare_parameter<double>(
      "min_cruise_speed", 0.12);
    own_ship_radius_ = declare_parameter<double>("own_ship_radius", 1.5);
    safety_buffer_ = declare_parameter<double>("safety_buffer", 4.0);
    stop_distance_ = declare_parameter<double>("colregs_risk_distance", 8.0);
    target_state_timeout_ = declare_parameter<double>("target_state_timeout", 2.0);
    overtaking_safe_distance_ = declare_parameter<double>("overtaking_safe_distance", 9.0);
    const auto target_topics = declare_parameter<std::vector<std::string>>(
      "target_odom_topics", {"/target_boat/odom"});
    target_messages_.resize(target_topics.size());
    for (std::size_t i = 0; i < target_topics.size(); ++i) {
      target_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        target_topics[i], 20, [this, i](nav_msgs::msg::Odometry::SharedPtr message) {
          target_messages_[i] = message;
        }));
    }

    // 代价地图参数：100 为硬障碍物，99 为膨胀区
    lethal_cost_threshold_ = declare_parameter<int>(
      "lethal_cost_threshold", 100);
    unknown_is_obstacle_ = declare_parameter<bool>(
      "unknown_is_obstacle", true);

    // 评分权重
    path_weight_ = declare_parameter<double>(
      "path_weight", 3.5);
    progress_weight_ = declare_parameter<double>(
      "progress_weight", 4.5);
    heading_weight_ = declare_parameter<double>(
      "heading_weight", 1.2);
    obstacle_weight_ = declare_parameter<double>(
      "obstacle_weight", 4.0);
    speed_weight_ = declare_parameter<double>(
      "speed_weight", 1.5);
    smooth_weight_ = declare_parameter<double>(
      "smooth_weight", 0.45);
    target_weight_ = declare_parameter<double>(
      "target_weight", 1.5);
    policy_sub_=create_subscription<hybrid_a_star_planner::msg::EncounterArray>("/colregs/policies",10,
      [this](hybrid_a_star_planner::msg::EncounterArray::SharedPtr m){policies_=m;});

    // Input freshness limits; exceeding either limit commands a stop.
    path_stale_timeout_ = declare_parameter<double>(
      "path_stale_timeout", 30.0);
    costmap_stale_timeout_ = declare_parameter<double>(
      "costmap_stale_timeout", 10.0);

    auto path_qos = rclcpp::QoS(10);
    path_qos.reliable();

    // local_costmap 常见为 Best Effort
    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    costmap_qos.best_effort();
    costmap_qos.durability_volatile();
    auto costmap_change_qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
    costmap_change_qos_.reliable();
    costmap_change_qos_.transient_local();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      path_qos,
      std::bind(
        &DwaPlanner::pathCallback,
        this,
        std::placeholders::_1));

    costmap_sub_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      costmap_change_qos_,
      std::bind(
        &DwaPlanner::costmapCallback,
        this,
        std::placeholders::_1));
    lane_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      lane_costmap_topic, costmap_change_qos_,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        lane_costmap_ = *message;
      });
    channel_obstacle_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      channel_obstacle_topic, costmap_change_qos_,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        channel_obstacle_map_ = *message;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(20),
      std::bind(
        &DwaPlanner::odomCallback,
        this,
        std::placeholders::_1));

    cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10));

    optimal_path_pub_ =
      create_publisher<nav_msgs::msg::Path>(
      "/optimal_path",
      rclcpp::QoS(10));

    optimal_trajectory_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
      "/optimal_trajectory",
      rclcpp::QoS(10));

    candidate_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/candidate_trajectories",
      rclcpp::QoS(10));

    target_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
      "/dwa_lookahead_target",
      rclcpp::QoS(10));

    const double frequency =
      std::max(1.0, control_frequency_);

    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / frequency)),
      std::bind(&DwaPlanner::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "DWA planner started: path=%s costmap=%s odom=%s",
      path_topic_.c_str(),
      costmap_topic_.c_str(),
      odom_topic_.c_str());
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  struct Point
  {
    double x{0.0};
    double y{0.0};
  };

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // 点投影到全局路径后的结果
  struct PathProjection
  {
    double distance{
      std::numeric_limits<double>::max()};
    double arc_length{0.0};
  };

  // 一组 DWA 速度对应的预测轨迹
  struct Candidate
  {
    double v{0.0};
    double w{0.0};

    double score{
      -std::numeric_limits<double>::infinity()};

    double average_cost{1.0};
    double maximum_cost{1.0};

    std::vector<Pose2D> trajectory;
  };

  // [功能与联系] 接收Hybrid连续路径；少于两点立即清除可跟踪路径并发布STOP，正常路径更新接收时间。
  void pathCallback(
    const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      {std::lock_guard<std::mutex> lock(path_mutex_);path_=*msg;has_path_=false;}
      publishStop();
      // 1 个点是 Hybrid A* 到达目标后的正常输出；空路径才表示无解/输入未就绪。
      if (msg->poses.empty()) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000, "Received an empty global path; stopping");
      }
      return;
    }

    std::lock_guard<std::mutex> lock(path_mutex_);
    path_ = *msg;
    has_path_ = true;
    path_receive_time_ = now();
  }

  // [功能与联系] 缓存物理地图及其网格元数据；航道节点用于对齐语义图，规划/控制节点用于碰撞和数据就绪检查。
  void costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {

    if (msg->data.empty() ||
      msg->info.width == 0 ||
      msg->info.height == 0 ||
      msg->info.resolution <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received invalid local costmap");
      return;
    }

    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ = *msg;
    has_costmap_ = true;
    costmap_receive_time_ = now();
  }

  // [功能与联系] 缓存里程计和接收时间供controlLoop使用；实测速率用于动态窗口及失联保护。
  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf2::getYaw(msg->pose.pose.orientation);

    std::lock_guard<std::mutex> lock(odom_mutex_);
    robot_pose_ = pose;
    measured_v_=msg->twist.twist.linear.x;
    measured_w_=msg->twist.twist.angular.z;
    odom_receive_time_=now();
    has_odom_ = true;
  }

  // [功能与联系] 先做输入过期/当前距离保护，再投影路径与前视、建立动态窗口、生成评分候选，安全最佳候选输出Twist，否则STOP。
  void controlLoop()
  {
    if(!policies_ || !policies_->valid || policies_->header.frame_id!=control_frame_ ||
      (now()-rclcpp::Time(policies_->header.stamp)).seconds()>2.5 ||
      (has_odom_ && (now()-odom_receive_time_).seconds()>1.0)) {publishStop();return;}
    nav_msgs::msg::Path path;
    nav_msgs::msg::OccupancyGrid costmap;
    Pose2D robot;

    rclcpp::Time path_time;
    rclcpp::Time costmap_time;

    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);

      if (!has_costmap_) {
        RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "***********************NO costmap msg");
        publishStop();
        return;
      }

      costmap = costmap_;
      costmap_time = costmap_receive_time_;
    }

    {
      std::lock_guard<std::mutex> lock(path_mutex_);

      if (!has_path_) {
        RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "***********************NO path msg");       
        publishStop();
        return;
      }

      path = path_;
      path_time = path_receive_time_;
    }

    {
      std::lock_guard<std::mutex> lock(odom_mutex_);

      if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "***********************NO odom msg");
        publishStop();
        return;
      }

      robot = robot_pose_;
      last_cmd_v_=measured_v_;
      last_cmd_w_=measured_w_;
    }

    // 独立20Hz距离停车，直接读取目标odom，不等待1Hz Hybrid A*风险分级。
    // 从未发布的虚拟船不影响静态航道测试；已有目标失联则停车。
    for (const auto & target : target_messages_) {
      if (!target) {continue;}
      const double age = (now() - rclcpp::Time(target->header.stamp)).seconds();
      if (target->header.frame_id != control_frame_ || age < -0.1 ||
        age > target_state_timeout_) {publishStop();return;}
      const double target_yaw = tf2::getYaw(target->pose.pose.orientation);
      const double vx = target->twist.twist.linear.x * std::cos(target_yaw) -
        target->twist.twist.linear.y * std::sin(target_yaw);
      const double vy = target->twist.twist.linear.x * std::sin(target_yaw) +
        target->twist.twist.linear.y * std::cos(target_yaw);
      if (hybrid_a_star_planner::insideStopDistance(robot.x, robot.y,
          target->pose.pose.position.x + vx * std::max(0.0, age),
          target->pose.pose.position.y + vy * std::max(0.0, age), stop_distance_)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Target inside %.1fm current-distance stop circle; commanding STOP", stop_distance_);
        publishStop();return;
      }
    }

    // Fail safe: stale planning or map data must never keep an old command alive.
    const double path_age = (now() - path_time).seconds();
    const double costmap_age = (now() - costmap_time).seconds();

    if (path_age > path_stale_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Path is stale (%.2f sec); stopping",
        path_age);
      publishStop();
      return;
    }
    // 河道静态图使用 Transient Local 只发布一次；timeout<=0 表示静态地图
    // 不按接收时间过期，但坐标系检查始终保留。
    if((costmap_stale_timeout_>0.0 && costmap_age>costmap_stale_timeout_) ||
      costmap.header.frame_id!=control_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Costmap is stale or has the wrong frame; stopping");
      publishStop();return;
    }

    // 目前要求全局路径、局部地图和 odom 都在同一坐标系
    if (path.header.frame_id != control_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Path frame '%s' does not match '%s'",
        path.header.frame_id.c_str(),
        control_frame_.c_str());

      publishStop();
      return;
    }

    // if (costmap.header.frame_id != control_frame_) {
    //   RCLCPP_WARN_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     3000,
    //     "Costmap frame '%s' does not match '%s'",
    //     costmap.header.frame_id.c_str(),
    //     control_frame_.c_str());

    //   //publishStop();
    //   //return;
    // }

    const Point robot_point{robot.x, robot.y};
    const Point goal = pathPoint(path, path.poses.size() - 1);
    const double goal_distance = distance(robot_point, goal);

    // 到达最终目标才停车
    if (goal_distance <= goal_tolerance_) {
      publishStop();
      publishOptimalPath(path);
      return;
    }

    // 当前船的位置投影到 RRT* 路径
    const PathProjection robot_projection =
      projectToPath(robot_point, path);

    /*
     * 自适应前视距离：
     * 当前速度越大，前视点越远；
     * 船会在到达路径折角前提前转向。
     */
    const double adaptive_lookahead = clamp(
      lookahead_distance_ + 0.8 * last_cmd_v_,
      1.2,
      std::max(1.2, max_lookahead_distance_));

    const Point target = lookaheadPoint(
      path,
      robot_projection.arc_length + adaptive_lookahead);

    publishLookaheadTarget(target);

    const double target_yaw = std::atan2(
      target.y - robot.y,
      target.x - robot.x);

    const double heading_error =
      normalizeAngle(target_yaw - robot.yaw);

    const double dt =
      1.0 / std::max(1.0, control_frequency_);

    /*
     * 转角速度限制：
     * 航向误差大时自动减速，但不再执行 v=0 原地转向。
     */
    const double heading_ratio = clamp(
      std::abs(heading_error) / (0.85 * kPi),
      0.0,
      1.0);

    const double corner_speed_limit =
      max_vel_x_ -
      (max_vel_x_ - min_cruise_speed_) * heading_ratio;
    // 在剩余距离内留出制动空间，避免高速船围绕终点循环。
    // 仅限制终点接近速度；巡航和避碰候选仍按原DWA评分与安全检查执行。
    const double goal_speed_limit = std::min(0.5 *
      std::max(0.0, goal_distance - goal_tolerance_), std::sqrt(2.0 *
      std::max(0.01, std::min(goal_deceleration_, acc_lim_x_)) *
      std::max(0.0, goal_distance - goal_tolerance_)));

    /*
     * 当前controlLoop已将last_cmd_v_/w_更新为odom实测速度，窗口围绕实测速率建立。
     * 变量名称仍沿用last_cmd，不能将其误读为始终只使用上一周期发布命令。
     */
    double v_min = std::max(
      min_vel_x_,
      last_cmd_v_ - acc_lim_x_ * dt);

    double v_max = std::min(
      std::min(corner_speed_limit, goal_speed_limit),
      last_cmd_v_ + acc_lim_x_ * dt);

    /*
     * 启动和大折角时，保留最小巡航速度候选。
     * 这样不会在路径折线拐角处直接停止。
     */
    const double minimum_turn_speed = std::min(
      min_cruise_speed_,
      corner_speed_limit);

    // Preserve the acceleration window, including deceleration/zero candidates.
    v_min = std::min(v_min, v_max);

    const double w_min = std::max(
      -max_vel_theta_,
      last_cmd_w_ - acc_lim_theta_ * dt);

    const double w_max = std::min(
      max_vel_theta_,
      last_cmd_w_ + acc_lim_theta_ * dt);

    const int v_count = std::max(2, v_samples_);
    const int w_count = std::max(3, w_samples_);

    std::vector<Candidate> candidates;
    candidates.reserve(
      static_cast<std::size_t>(v_count * w_count));

    Candidate best;

    // 枚举动态窗口内的速度组合
    for (int vi = 0; vi < v_count; ++vi) {
      double v = interpolate(
        v_min,
        v_max,
        vi,
        v_count);

      // 未到终点时强制保留前进候选

      for (int wi = 0; wi < w_count; ++wi) {
        const double w = interpolate(
          w_min,
          w_max,
          wi,
          w_count);

        Candidate candidate;
        candidate.v = v;
        candidate.w = w;

        candidate.trajectory =
          generateTrajectory(robot, v, w);

        candidate.score = scoreCandidate(
          candidate,
          path,
          robot_projection.arc_length,
          target,
          costmap);

        candidates.push_back(candidate);

        if (candidate.score > best.score) {
          best = candidate;
        }
      }
    }

    /*
     * 只有全部预测轨迹都碰撞时才停止。
     * 普通高代价区域不会停车，而是选择绕开的弧线轨迹。
     */
    if (!std::isfinite(best.score)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "All candidate trajectories are blocked");

      publishStop();
      return;
    }

    geometry_msgs::msg::Twist desired;
    desired.linear.x = best.v;
    desired.angular.z = best.w;

    publishSmoothCommand(desired);

    publishOptimalPath(path);
    publishOptimalTrajectory(best);
    publishCandidates(candidates);
  }

  // 根据 v、w 生成预测轨迹
  // [功能与联系] 按候选常速度/角速度预测sim_time内姿态；scoreCandidate逐点检查静态/动态安全。
  std::vector<Pose2D> generateTrajectory(
    const Pose2D & start,
    const double v,
    const double w) const
  {
    std::vector<Pose2D> trajectory;

    const int steps = std::max(
      1,
      static_cast<int>(
        std::ceil(sim_time_ / sim_granularity_)));

    const double dt =
      sim_time_ / static_cast<double>(steps);

    Pose2D state = start;
    trajectory.push_back(state);

    for (int i = 0; i < steps; ++i) {
      state.x += v * std::cos(state.yaw) * dt;
      state.y += v * std::sin(state.yaw) * dt;
      state.yaw = normalizeAngle(state.yaw + w * dt);

      trajectory.push_back(state);
    }

    return trajectory;
  }

  // 对候选轨迹评分：路径、进度、方向、避障、速度、平滑性
  // [功能与联系] 先用COLREG、同步目标净空和地图淘汰危险候选，再累计路径/进度/朝向/避障/速度/连续性评分，选择分数最大的候选。
  double scoreCandidate(
    Candidate & candidate,
    const nav_msgs::msg::Path & path,
    const double robot_arc_length,
    const Point & target,
    const nav_msgs::msg::OccupancyGrid & costmap) const
  {
    if (candidate.trajectory.empty()) {
      return -std::numeric_limits<double>::infinity();
    }
    const double policy_age=(now()-rclcpp::Time(policies_->header.stamp)).seconds();
    for(const auto &e:policies_->encounters) {
      hybrid_a_star_planner::Policy p;
      p.type=static_cast<hybrid_a_star_planner::EncounterType>(e.type);
      p.level=static_cast<hybrid_a_star_planner::Risk>(e.risk);
      p.locked=e.locked;p.takeover=e.takeover;p.reference=e.reference;p.speed=e.speed;
      p.recovering=e.recovering;
      if(p.standOn() && p.speed>0.1 && std::abs(candidate.v-p.speed)>0.08)
        return -std::numeric_limits<double>::infinity();
      for(std::size_t i=1;i<candidate.trajectory.size();++i) {
        const auto &s=candidate.trajectory[i];
        const double time=sim_time_*i/(candidate.trajectory.size()-1);
        if(!hybrid_a_star_planner::headingAllowed(p,s.yaw,time,0,candidate.trajectory.front().yaw))
          return -std::numeric_limits<double>::infinity();
        const double tx=e.x+e.vx*(policy_age+time),ty=e.y+e.vy*(policy_age+time);
        // Includes reaction/braking travel of the accelerated simulator.
        const double safe=std::max(own_ship_radius_+safety_buffer_+e.radius,
          p.type==hybrid_a_star_planner::EncounterType::OVERTAKING ? overtaking_safe_distance_ : 0.0);
        if(std::hypot(s.x-tx,s.y-ty)<safe) return -std::numeric_limits<double>::infinity();
      }
      // 追越后立即到达终点时，减速会使后船重新追上。
      // 将近终点候选的动态检查延长至10秒，提前保留侧向净空；
      // 不改变普通DWA显示/静态轨迹时域，也不绕过8m当前距离停车保护。
      const Point goal = pathPoint(path, path.poses.size()-1);
      if(p.locked && p.type==hybrid_a_star_planner::EncounterType::OVERTAKING &&
        distance(Point{candidate.trajectory.front().x,candidate.trajectory.front().y},goal)<20.0) {
        Pose2D future=candidate.trajectory.back();
        for(double time=sim_time_+0.1;time<=10.0;time+=0.1) {
          future.x+=candidate.v*std::cos(future.yaw)*0.1;
          future.y+=candidate.v*std::sin(future.yaw)*0.1;
          future.yaw+=candidate.w*0.1;
          const double tx=e.x+e.vx*(policy_age+time),ty=e.y+e.vy*(policy_age+time);
          if(std::hypot(future.x-tx,future.y-ty)<
            std::max(own_ship_radius_+safety_buffer_+e.radius,overtaking_safe_distance_))
            return -std::numeric_limits<double>::infinity();
        }
      }
    }

    double total_cost = 0.0;
    double maximum_cost = 0.0;

    // 对候选轨迹每个点查询 local_costmap
    for (const Pose2D & state : candidate.trajectory) {
      int raw_cost = costAt(
        Point{state.x, state.y},
        costmap);

      // Hybrid A* 与 DWA 使用相同的分源规则：已确认的 TSSLPT 水域只消除
      // 岸边膨胀，独立航道障碍物层仍可淘汰候选轨迹或产生软惩罚。
      if (channel_enabled_ && !lane_costmap_.data.empty()) {
        const int lane = costAt(Point{state.x, state.y}, lane_costmap_);
        const bool inside_lane = lane >= 0 &&
          (lane <= main_lane_value_max_ || lane >= opposite_lane_value_min_);
        if (inside_lane) {
          raw_cost = 0;
          // 主航道的 0..main_lane_value_max 是中心软梯度。DWA 仍以全局路径为主，同时在
          // 局部候选相近时更偏好航道中心；对向航道类别值不当作障碍代价。
          if (lane <= main_lane_value_max_) {
            raw_cost = static_cast<int>(std::round(lane * center_cost_scale_));
          }
        }
      }
      if (!channel_obstacle_map_.data.empty()) {
        const int obstacle_cost = costAt(Point{state.x, state.y}, channel_obstacle_map_);
        if (obstacle_cost >= channel_obstacle_threshold_) {
          return -std::numeric_limits<double>::infinity();
        }
        raw_cost = std::max(raw_cost, obstacle_cost);
      }

      // 未知区域可根据参数视为障碍
      if (raw_cost < 0 && unknown_is_obstacle_) {
        return -std::numeric_limits<double>::infinity();
      }

      // 100 为硬障碍物，候选轨迹直接淘汰
      if (raw_cost >= lethal_cost_threshold_) {
        return -std::numeric_limits<double>::infinity();
      }

      // 99 膨胀区域允许，但会降低分数
      const double normalized_cost =
        static_cast<double>(
          std::clamp(raw_cost, 0, 99)) / 99.0;

      total_cost += normalized_cost;
      maximum_cost = std::max(
        maximum_cost,
        normalized_cost);
    }

    candidate.average_cost =
      total_cost /
      static_cast<double>(candidate.trajectory.size());

    candidate.maximum_cost = maximum_cost;

    const Pose2D & end_pose =
      candidate.trajectory.back();

    const Point end_point{
      end_pose.x,
      end_pose.y};

    const PathProjection end_projection =
      projectToPath(end_point, path);

    // 终点靠近全局路径
    const double path_score = std::exp(
      -2.0 *
      end_projection.distance *
      end_projection.distance);

    // 终点沿全局路径向前推进
    const double progress =
      end_projection.arc_length - robot_arc_length;

    const double expected_progress = std::max(
      0.1,
      candidate.v * sim_time_);

    const double progress_score = clamp(
      progress / expected_progress,
      0.0,
      1.0);

    // 终点朝向和路径切线保持一致
    const double path_yaw = pathHeadingAt(
      path,
      end_projection.arc_length);

    const double heading_score =
      0.5 *
      (1.0 +
      std::cos(normalizeAngle(path_yaw - end_pose.yaw)));

    // 越远离高代价区域越好
    const double obstacle_score = clamp(
      1.0 -
      0.35 * candidate.average_cost -
      0.65 * candidate.maximum_cost,
      0.0,
      1.0);

    // 朝向前视点推进
    const double target_distance =
      distance(end_point, target);

    const double target_score = std::exp(
      -0.7 *
      target_distance *
      target_distance);

    // 速度较高有奖励，但不会超过拐角限速
    const double speed_score = clamp(
      candidate.v / std::max(0.01, max_vel_x_),
      0.0,
      1.0);

    // 与上一次命令变化越小越平滑
    const double velocity_change =
      std::abs(candidate.v - last_cmd_v_) /
      std::max(0.01, max_vel_x_);

    const double angular_change =
      std::abs(candidate.w - last_cmd_w_) /
      std::max(0.01, max_vel_theta_);

    const double smooth_score =
      1.0 - clamp(
      0.5 * velocity_change +
      0.5 * angular_change,
      0.0,
      1.0);

    // 终点附近优先收敛位置，而非继续追随要求最终朝向的长绕行尾段。
    // 只加软评分，以上静态、动态及COLREG硬约束均已通过后才能到达这里。
    const Point goal = pathPoint(path, path.poses.size() - 1);
    const double goal_distance = distance(
      Point{candidate.trajectory.front().x, candidate.trajectory.front().y}, goal);
    const double terminal_score = goal_distance < 10.0 ?
      20.0 * (goal_distance - distance(end_point, goal)) /
      std::max(1.0, candidate.v * sim_time_) : 0.0;
    return terminal_score +
      path_weight_ * path_score +
      progress_weight_ * progress_score +
      heading_weight_ * heading_score +
      obstacle_weight_ * obstacle_score +
      speed_weight_ * speed_score +
      smooth_weight_ * smooth_score +
      target_weight_ * target_score;
  }

  // 将点投影到全局路径，得到最近距离和沿路径长度
  // [功能与联系] 把船或候选终点投影到最近路径段，得到横向距离与弧长进度；控制前视和评分共用。
  PathProjection projectToPath(
    const Point & point,
    const nav_msgs::msg::Path & path) const
  {
    PathProjection best;
    double accumulated_length = 0.0;

    for (std::size_t i = 0;
      i + 1 < path.poses.size();
      ++i)
    {
      const Point a = pathPoint(path, i);
      const Point b = pathPoint(path, i + 1);

      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double segment_length = std::hypot(dx, dy);

      if (segment_length < 1.0e-6) {
        continue;
      }

      const double segment_length_sq =
        segment_length * segment_length;

      const double t = clamp(
        ((point.x - a.x) * dx +
        (point.y - a.y) * dy) /
        segment_length_sq,
        0.0,
        1.0);

      const Point projection{
        a.x + t * dx,
        a.y + t * dy};

      const double projection_distance =
        distance(point, projection);

      if (projection_distance < best.distance) {
        best.distance = projection_distance;
        best.arc_length =
          accumulated_length + t * segment_length;
      }

      accumulated_length += segment_length;
    }

    return best;
  }

  // 获取指定路径长度处的前视点
  // [功能与联系] 按路径累计弧长插值前视目标；controlLoop结合速度自适应前视距离使用。
  Point lookaheadPoint(
    const nav_msgs::msg::Path & path,
    const double requested_arc) const
  {
    double accumulated_length = 0.0;

    for (std::size_t i = 0;
      i + 1 < path.poses.size();
      ++i)
    {
      const Point a = pathPoint(path, i);
      const Point b = pathPoint(path, i + 1);
      const double segment_length = distance(a, b);

      if (segment_length < 1.0e-6) {
        continue;
      }

      if (accumulated_length + segment_length >= requested_arc) {
        const double t =
          (requested_arc - accumulated_length) /
          segment_length;

        return Point{
          a.x + t * (b.x - a.x),
          a.y + t * (b.y - a.y)};
      }

      accumulated_length += segment_length;
    }

    return pathPoint(path, path.poses.size() - 1);
  }

  // 获取路径某位置的切线方向
  // [功能与联系] 按指定弧长查询路径切向，用于候选终点朝向评分。
  double pathHeadingAt(
    const nav_msgs::msg::Path & path,
    const double requested_arc) const
  {
    double accumulated_length = 0.0;

    for (std::size_t i = 0;
      i + 1 < path.poses.size();
      ++i)
    {
      const Point a = pathPoint(path, i);
      const Point b = pathPoint(path, i + 1);
      const double segment_length = distance(a, b);

      if (segment_length < 1.0e-6) {
        continue;
      }

      if (accumulated_length + segment_length >= requested_arc) {
        return std::atan2(
          b.y - a.y,
          b.x - a.x);
      }

      accumulated_length += segment_length;
    }

    const Point a = pathPoint(path, path.poses.size() - 2);
    const Point b = pathPoint(path, path.poses.size() - 1);

    return std::atan2(
      b.y - a.y,
      b.x - a.x);
  }

  // 查询某个世界坐标点对应的地图代价
  // [功能与联系] 按地图原点姿态/分辨率查询代价，越界/缺图返回保守或未提供值；支撑多源融合和候选淘汰。
  int costAt(
    const Point & point,
    const nav_msgs::msg::OccupancyGrid & map) const
  {
    const auto & info = map.info;

    if (info.width == 0 ||
      info.height == 0 ||
      info.resolution <= 0.0)
    {
      return lethal_cost_threshold_;
    }

    const double map_yaw = tf2::getYaw(
      info.origin.orientation);

    const double dx =
      point.x - info.origin.position.x;

    const double dy =
      point.y - info.origin.position.y;

    // 世界坐标转换为地图局部坐标
    const double local_x =
      std::cos(map_yaw) * dx +
      std::sin(map_yaw) * dy;

    const double local_y =
      -std::sin(map_yaw) * dx +
      std::cos(map_yaw) * dy;

    const int map_x = static_cast<int>(
      std::floor(local_x / info.resolution));

    const int map_y = static_cast<int>(
      std::floor(local_y / info.resolution));

    // 地图范围外视为不可通行
    if (map_x < 0 ||
      map_y < 0 ||
      map_x >= static_cast<int>(info.width) ||
      map_y >= static_cast<int>(info.height))
    {
      return lethal_cost_threshold_;
    }

    const std::size_t index =
      static_cast<std::size_t>(map_y) *
      static_cast<std::size_t>(info.width) +
      static_cast<std::size_t>(map_x);

    if (index >= map.data.size()) {
      return lethal_cost_threshold_;
    }

    return static_cast<int>(map.data[index]);
  }

  // 输出平滑后的速度命令
  // [功能与联系] 按变化限幅输出最佳Twist；使用实测速度建立约束，紧急STOP不经过这一平滑流程。
  void publishSmoothCommand(
    const geometry_msgs::msg::Twist & desired)
  {
    const double dt =
      1.0 / std::max(1.0, control_frequency_);

    geometry_msgs::msg::Twist command;

    // 线速度加速度限制
    command.linear.x = clamp(
      desired.linear.x,
      last_cmd_v_ - acc_lim_x_ * dt,
      last_cmd_v_ + acc_lim_x_ * dt);

    // 角速度加速度限制
    command.angular.z = clamp(
      desired.angular.z,
      last_cmd_w_ - acc_lim_theta_ * dt,
      last_cmd_w_ + acc_lim_theta_ * dt);

    command.linear.x = clamp(
      command.linear.x,
      min_vel_x_,
      max_vel_x_);

    command.angular.z = clamp(
      command.angular.z,
      -max_vel_theta_,
      max_vel_theta_);

    cmd_vel_pub_->publish(command);

    last_cmd_v_ = command.linear.x;
    last_cmd_w_ = command.angular.z;
  }

  // [功能与联系] 直接输出零线速度和零角速度并重置命令记录；物理仿真仍按惯性制动，不代表瞬时停止。
  void publishStop()
  {
    geometry_msgs::msg::Twist command;

    command.linear.x = 0.0;
    command.angular.z = 0.0;

    cmd_vel_pub_->publish(command);

    last_cmd_v_ = 0.0;
    last_cmd_w_ = 0.0;
  }

  // [功能与联系] 发布跟踪输入Path作为控制诊断；不是另一层全局搜索。
  void publishOptimalPath(
    const nav_msgs::msg::Path & path)
  {
    nav_msgs::msg::Path output = path;
    output.header.stamp = now();
    optimal_path_pub_->publish(output);
  }

  // 发布最优轨迹，RViz 中显示为红线
  // [功能与联系] 将最优局部预测候选绘为Marker，供RViz观察短时控制意图。
  void publishOptimalTrajectory(
    const Candidate & candidate)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = control_frame_;
    marker.header.stamp = now();
    marker.ns = "optimal_trajectory";
    marker.id = 0;
    marker.type =
      visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.06;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.lifetime =
      rclcpp::Duration::from_seconds(0.3);

    for (const Pose2D & state : candidate.trajectory) {
      geometry_msgs::msg::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = 0.05;
      marker.points.push_back(point);
    }

    optimal_trajectory_pub_->publish(marker);
  }

  /*
   * 发布每条有效 DWA 候选轨迹，而不只是终点。
   * RViz 中显示为半透明蓝线，红线仍表示最终选中的轨迹。
   */
  // [功能与联系] 绘制候选轨迹并区分有效/无效或评分；显示开销随速度采样数量增加。
  void publishCandidates(
    const std::vector<Candidate> & candidates)
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action =
      visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    int marker_id = 0;

    for (const Candidate & candidate : candidates) {
      if (!std::isfinite(candidate.score) ||
        candidate.trajectory.empty())
      {
        continue;
      }

      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = control_frame_;
      marker.header.stamp = now();
      marker.ns = "candidate_trajectories";
      marker.id = marker_id++;
      marker.type =
        visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action =
        visualization_msgs::msg::Marker::ADD;

      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.025;

      marker.color.a = 0.35;
      marker.color.r = 0.1;
      marker.color.g = 0.5;
      marker.color.b = 1.0;

      marker.lifetime =
        rclcpp::Duration::from_seconds(0.2);

      for(const auto &state:candidate.trajectory) {
        geometry_msgs::msg::Point point;
        point.x=state.x;point.y=state.y;point.z=0.03;
        marker.points.push_back(point);
      }

      markers.markers.push_back(marker);
    }

    candidate_pub_->publish(markers);
  }

  // 发布真实前视目标点，RViz 中显示为绿色球
  // [功能与联系] 显示当前前视点，辅助定位跟踪方向与转弯响应。
  void publishLookaheadTarget(
    const Point & target)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = control_frame_;
    marker.header.stamp = now();
    marker.ns = "dwa_lookahead_target";
    marker.id = 0;
    marker.type =
      visualization_msgs::msg::Marker::SPHERE;
    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.pose.position.z = 0.10;

    marker.scale.x = 0.18;
    marker.scale.y = 0.18;
    marker.scale.z = 0.18;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime =
      rclcpp::Duration::from_seconds(0.3);

    target_marker_pub_->publish(marker);
  }

  // [功能与联系] 按索引读取Path坐标；用于路径投影、前视插值及终点判断。
  static Point pathPoint(
    const nav_msgs::msg::Path & path,
    const std::size_t index)
  {
    return Point{
      path.poses[index].pose.position.x,
      path.poses[index].pose.position.y};
  }

  // [功能与联系] 计算两点欧氏距离，用于目标容差和路径段长度。
  static double distance(
    const Point & a,
    const Point & b)
  {
    return std::hypot(
      a.x - b.x,
      a.y - b.y);
  }

  // [功能与联系] 将标量限制到上下界，供速度/评分/插值计算共用。
  static double clamp(
    const double value,
    const double lower,
    const double upper)
  {
    return std::max(
      lower,
      std::min(value, upper));
  }

  // [功能与联系] 归一化角度，消除跨±pi的跳变；方向比较、运动积分和COLREG约束共同使用。
  static double normalizeAngle(
    double angle)
  {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }

    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }

    return angle;
  }

  // [功能与联系] 在上下界内按采样索引生成速度或角速度候选。
  static double interpolate(
    const double lower,
    const double upper,
    const int index,
    const int count)
  {
    if (count <= 1) {
      return 0.5 * (lower + upper);
    }

    return lower +
      (upper - lower) *
      static_cast<double>(index) /
      static_cast<double>(count - 1);
  }

  // ROS 参数
  std::string path_topic_;
  hybrid_a_star_planner::msg::EncounterArray::SharedPtr policies_;
  rclcpp::Subscription<hybrid_a_star_planner::msg::EncounterArray>::SharedPtr policy_sub_;
  double measured_v_{0},measured_w_{0};
  rclcpp::Time odom_receive_time_;
  std::string costmap_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string control_frame_;

  double control_frequency_{20.0};

  double max_vel_x_{0.8};
  double min_vel_x_{0.0};
  double max_vel_theta_{1.2};

  double acc_lim_x_{0.5};
  double goal_deceleration_{0.5};
  double acc_lim_theta_{1.8};

  double sim_time_{2.0};
  double sim_granularity_{0.05};

  int v_samples_{11};
  int w_samples_{31};

  double lookahead_distance_{2.0};
  double max_lookahead_distance_{10.0};
  double goal_tolerance_{0.4};
  double min_cruise_speed_{0.12};
  double own_ship_radius_{2.0};
  double safety_buffer_{3.0};
  double stop_distance_{8.0};
  double target_state_timeout_{2.0};
  double overtaking_safe_distance_{9.0};
  std::vector<nav_msgs::msg::Odometry::SharedPtr> target_messages_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> target_subs_;

  int lethal_cost_threshold_{100};
  bool unknown_is_obstacle_{true};
  bool channel_enabled_{true};
  int main_lane_value_max_{20};
  int opposite_lane_value_min_{65};
  int channel_obstacle_threshold_{85};
  double center_cost_scale_{2.5};

  double path_weight_{3.5};
  double progress_weight_{4.5};
  double heading_weight_{1.2};
  double obstacle_weight_{4.0};
  double speed_weight_{1.5};
  double smooth_weight_{0.45};
  double target_weight_{1.5};

  double path_stale_timeout_{30.0};
  double costmap_stale_timeout_{10.0};

  // 最新输入数据
  bool has_path_{false};
  bool has_costmap_{false};
  bool has_odom_{false};

  nav_msgs::msg::Path path_;
  nav_msgs::msg::OccupancyGrid costmap_;
  nav_msgs::msg::OccupancyGrid lane_costmap_;
  nav_msgs::msg::OccupancyGrid channel_obstacle_map_;
  Pose2D robot_pose_;

  rclcpp::Time path_receive_time_;
  rclcpp::Time costmap_receive_time_;

  // 前一帧实际发布的速度，用于速度平滑和动态窗口
  double last_cmd_v_{0.0};
  double last_cmd_w_{0.0};

  std::mutex path_mutex_;
  std::mutex costmap_mutex_;
  std::mutex odom_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    path_sub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lane_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr channel_obstacle_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    optimal_path_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    optimal_trajectory_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    candidate_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    target_marker_pub_;

  rclcpp::TimerBase::SharedPtr
    control_timer_;
};

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaPlanner>());
  rclcpp::shutdown();
  return 0;
}
