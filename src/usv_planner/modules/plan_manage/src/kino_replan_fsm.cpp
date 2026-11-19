/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/




#include <plan_manage/kino_replan_fsm.h>
#include <path_searching/usv_dynamics.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace fast_planner {

void KinoReplanFSM::init(rclcpp::Node& nh) {
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  trigger_     = false;
  node_        = &nh;
  clock_       = nh.get_clock();

  /*  fsm param  */
  goal_mode_ = nh.declare_parameter<int>("goal/input_mode", 1);
  replan_thresh_ = nh.declare_parameter<double>("fsm/thresh_replan", -1.0);
  no_replan_thresh_ = nh.declare_parameter<double>("fsm/thresh_no_replan", -1.0);
  no_replan_yaw_thresh_ = nh.declare_parameter<double>("fsm/thresh_no_replan_yaw", -1.0);
  arrival_pos_tolerance_ = nh.declare_parameter<double>("fsm/arrival_pos_tolerance", -1.0);
  arrival_yaw_tolerance_ = nh.declare_parameter<double>("fsm/arrival_yaw_tolerance", -1.0);
  path_speed_mps_ = nh.declare_parameter<double>("goal/path_speed_mps", 1.0);

  if (goal_mode_ != static_cast<int>(GoalMode::POSE) &&
    goal_mode_ != static_cast<int>(GoalMode::PATH))
  {
    throw std::invalid_argument("goal/input_mode must be 1 (pose) or 2 (path)");
  }
  if (!std::isfinite(path_speed_mps_) || path_speed_mps_ <= 0.0) {
    throw std::invalid_argument("goal/path_speed_mps must be finite and positive");
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  route_range_m_ = nh.get_parameter("search/horizon").as_double();
  if (!std::isfinite(route_range_m_) || route_range_m_ <= 0.0) {
    throw std::invalid_argument("search/horizon must be finite and positive");
  }
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  using namespace std::chrono_literals;
  exec_timer_   = nh.create_wall_timer(100ms, std::bind(&KinoReplanFSM::execFSMCallback, this));
  safety_timer_ = nh.create_wall_timer(50ms, std::bind(&KinoReplanFSM::checkCollisionCallback, this));
  bspline_timer_ = nh.create_wall_timer(100ms, std::bind(&KinoReplanFSM::publishActiveBspline, this));

  const auto pose_topic = nh.declare_parameter<std::string>("goal/pose_topic", "/goal_pose");
  const auto path_topic = nh.declare_parameter<std::string>("goal/path_topic", "/path");
  const auto odom_topic = nh.declare_parameter<std::string>("topics/odom", "/odom");
  goal_pose_sub_ = nh.create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&KinoReplanFSM::goalPoseCallback, this, std::placeholders::_1));
  goal_path_sub_ = nh.create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      std::bind(&KinoReplanFSM::goalPathCallback, this, std::placeholders::_1));
  odom_sub_ = nh.create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_  = nh.create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  new_pub_     = nh.create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  replan_failed_pub_ = nh.create_publisher<std_msgs::msg::Header>("/planning/replan_failed", 10);
  const auto bspline_topic = nh.declare_parameter<std::string>("topics/bspline", "/planning/bspline");
  bspline_pub_ = nh.create_publisher<usv_interfaces::msg::UsvBspline>(bspline_topic, 10);
  bspline_path_pub_ = nh.create_publisher<nav_msgs::msg::Path>(
      "/planning/bspline_path", rclcpp::QoS(1).reliable().transient_local());
}

void KinoReplanFSM::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
  if (goal_mode_ != static_cast<int>(GoalMode::POSE)) return;
  const std::string& frame = planner_manager_->edt_environment_->sdf_map_->getFrameId();
  if (msg->header.frame_id != frame) {
    RCLCPP_WARN(node_->get_logger(), "Pose goal frame must be '%s'.", frame.c_str());
    return;
  }

  const auto& q = msg->pose.orientation;
  Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6 ||
    !std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y))
  {
    RCLCPP_WARN(node_->get_logger(), "Pose goal contains invalid position or orientation.");
    return;
  }
  orientation.normalize();
  const Eigen::Vector3d arrow = orientation.toRotationMatrix().col(0);
  const Eigen::Vector3d goal(
    msg->pose.position.x, msg->pose.position.y,
    usv_dynamics::headingFromForwardVector(arrow.x(), arrow.y()));
  route_active_ = false;
  planner_manager_->setGlobalWaypoints({});
  acceptGoal(goal, "POSE");
}

void KinoReplanFSM::goalPathCallback(const nav_msgs::msg::Path::ConstSharedPtr& msg)
{
  if (goal_mode_ != static_cast<int>(GoalMode::PATH)) return;
  const std::string& frame = planner_manager_->edt_environment_->sdf_map_->getFrameId();
  if (msg->header.frame_id != frame || msg->poses.size() < 2) {
    RCLCPP_WARN(node_->get_logger(), "Path goal needs at least two poses in frame '%s'.", frame.c_str());
    return;
  }

  std::vector<Eigen::Vector2d> positions;
  positions.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != frame) ||
      !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y))
    {
      RCLCPP_WARN(node_->get_logger(), "Path goal contains a frame mismatch or invalid point.");
      return;
    }
    const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    if (positions.empty() || (point - positions.back()).norm() > 1e-3) {
      positions.push_back(point);
    }
  }
  if (positions.size() < 2) {
    RCLCPP_WARN(node_->get_logger(), "Path goal has fewer than two distinct points.");
    return;
  }

  std::vector<Eigen::Vector3d> waypoints;
  waypoints.reserve(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const std::size_t from = i + 1 < positions.size() ? i : i - 1;
    const std::size_t to = i + 1 < positions.size() ? i + 1 : i;
    const Eigen::Vector2d direction = positions[to] - positions[from];
    waypoints.emplace_back(positions[i].x(), positions[i].y(),
      usv_dynamics::headingFromForwardVector(direction.x(), direction.y()));
  }
  route_.reset(positions);
  route_active_ = true;
  route_terminal_ = false;
  planner_manager_->setGlobalWaypoints(waypoints);
  end_pt_ = waypoints.front();
  if (have_odom_) updateRouteGoal();
  acceptGoal(end_pt_, "PATH");
}

void KinoReplanFSM::acceptGoal(const Eigen::Vector3d& goal, const std::string& source)
{
  holding_terminal_ = false;
  end_pt_ = goal;
  trigger_ = true;
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  if (route_active_ && !route_terminal_) {
    end_vel_ << -path_speed_mps_ * std::sin(goal(2)),
      path_speed_mps_ * std::cos(goal(2)), 0.0;
  }
  have_target_ = true;
  retry_not_before_ = std::chrono::steady_clock::time_point{};
  RCLCPP_INFO(node_->get_logger(), "Accepted %s goal at (%.2f, %.2f), phi=%.1f deg.",
    source.c_str(), end_pt_(0), end_pt_(1), end_pt_(2) * 180.0 / std::acos(-1.0));

  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "GOAL");
  } else if (exec_state_ == EXEC_TRAJ) {
    changeFSMExecState(REPLAN_TRAJ, "GOAL");
  }
}

bool KinoReplanFSM::updateRouteGoal()
{
  if (!route_active_) return true;
  if (!have_odom_) return false;
  route_.update(odom_pos_.head<2>());
  const auto map = planner_manager_->edt_environment_->sdf_map_;
  if (!map->isMapReady()) return false;
  const auto target = route_.safeTarget(odom_pos_.head<2>(), route_range_m_,
    [&map](const Eigen::Vector3d& pose) {
      const Eigen::Vector3d point(pose.x(), pose.y(), 0.0);
      return map->getInflateOccupancy(point) == 0 && map->getDistance(point) > 0.5;
    }, std::min(0.5, 0.5 * map->getResolution()));
  if (!target) {
    route_terminal_ = false;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000,
      "No safe Path target in %.1f m perception range; waiting for a reachable route or map update.",
      route_range_m_);
    return false;
  }
  end_pt_ = target->pose;
  route_terminal_ = target->terminal;
  end_vel_.setZero();
  if (!route_terminal_) {
    const double speed = planner_manager_->limitGoalSpeed(end_pt_, path_speed_mps_);
    end_vel_ << -speed * std::sin(end_pt_(2)), speed * std::cos(end_pt_(2)), 0.0;
  }
  const auto frontier = route_.target(odom_pos_.head<2>(), route_range_m_);
  if (!end_pt_.isApprox(frontier.pose)) {
    RCLCPP_INFO(node_->get_logger(),
      "Blocked Path frontier (%.2f, %.2f); selected safe route target (%.2f, %.2f).",
      frontier.pose.x(), frontier.pose.y(), end_pt_.x(), end_pt_.y());
  }
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  RCLCPP_INFO(node_->get_logger(),
    "Path rolling target: progress=%.2f m, range=%.2f m, goal=(%.2f, %.2f), terminal=%s.",
    route_.progress(), route_range_m_, end_pt_(0), end_pt_(1), route_terminal_ ? "true" : "false");
  return true;
}

void KinoReplanFSM::completeCurrentGoal(const std::string& source)
{
  route_active_ = false;
  holding_terminal_ = exec_state_ == EXEC_TRAJ && planner_manager_->hasTerminalTrajectory();
  have_target_ = false;
  changeFSMExecState(WAIT_TARGET, source);
}

void KinoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  if (planner_manager_->edt_environment_->sdf_map_->usesGridMap() &&
      msg->header.frame_id != planner_manager_->edt_environment_->sdf_map_->getFrameId())
  {
    have_odom_ = false;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 2000,
                         "USV odometry must be in the planning frame; TF conversion is not enabled yet.");
    return;
  }
  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  const Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
  const double phi = usv_dynamics::headingFromRightVector(rot_x(0), rot_x(1));
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = phi;
  const Eigen::Vector3d body_velocity(
      msg->twist.twist.linear.y, msg->twist.twist.linear.x,
      msg->twist.twist.angular.z);
  odom_vel_ = usv_dynamics::toWorldVelocity(odom_pos_, body_velocity);

  have_odom_ = true;
  if (route_active_) route_.update(odom_pos_.head<2>());
  last_odom_received_ = std::chrono::steady_clock::now();
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, const std::string& pos_call) {
  std::string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  std::cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

/**
 * @brief 按里程计位置和航向判断是否到达当前目标。
 * @param distance 输出平面距离，单位 m。
 * @param heading_error 输出最短航向误差的绝对值，单位 rad。
 * @return 位姿满足容差，且最终目标的实测合速度不大于 0.2 m/s 时返回 true。
 */
bool KinoReplanFSM::reachedGoal(double& distance, double& heading_error) const {
  distance = (end_pt_.head(2) - odom_pos_.head(2)).norm();
  const double delta = end_pt_(2) - odom_pos_(2);
  heading_error = std::abs(std::atan2(std::sin(delta), std::cos(delta)));
  if (route_active_ && !route_terminal_) return false;
  const bool final_goal = end_vel_.head<2>().norm() < 1e-6;
  if (final_goal && planner_manager_->hasTerminalTrajectory()) {
    auto& trajectory = planner_manager_->local_data_;
    const Eigen::VectorXd end = trajectory.position_traj_.evaluateDeBoorT(trajectory.duration_);
    // 旧任务的终端轨迹不参与新目标的到达判断。
    if ((end.head<2>() - end_pt_.head<2>()).norm() <= arrival_pos_tolerance_ &&
        (end.head<2>() - odom_pos_.head<2>()).norm() > 2.0) return false;
  }
  return arrival_pos_tolerance_ > 0.0 && arrival_yaw_tolerance_ > 0.0 &&
         distance <= arrival_pos_tolerance_ && heading_error <= arrival_yaw_tolerance_ &&
         (!final_goal || odom_vel_.head<2>().norm() <= 0.2);
}

void KinoReplanFSM::execFSMCallback() {
  static auto last_status = std::chrono::steady_clock::now();
  const auto status_now = std::chrono::steady_clock::now();
  if (status_now - last_status >= std::chrono::seconds(5)) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    last_status = status_now;
  }

  if (!planner_manager_->edt_environment_->sdf_map_->isMapReady() || !have_odom_)
  {
    return;  // 收到地图、完成距离场更新且里程计坐标正确后才推进状态机。
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      // Avoid a 100 Hz failure loop while the vessel brakes back into the model limits.
      if (std::chrono::steady_clock::now() < retry_not_before_) return;
      const bool goal_available = updateRouteGoal();
      double actual_distance, actual_yaw_error;
      if (goal_available && reachedGoal(actual_distance, actual_yaw_error))
      {
        completeCurrentGoal("ARRIVED");
        return;
      }
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool success = planOrKeepSafeTrajectory(goal_available);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        retry_not_before_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      rclcpp::Time   time_now = clock_->now();
      double         t_cur    = (time_now - info->start_time_).seconds();
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      double goal_distance = (end_pt_.head(2) - pos.head(2)).norm();
      double start_distance = (info->start_pos_.head(2) - pos.head(2)).norm();
      double goal_yaw_error = end_pt_(2) - pos(2);
      goal_yaw_error = std::atan2(std::sin(goal_yaw_error), std::cos(goal_yaw_error));

      if (t_cur > info->duration_ - 1e-2)
      {
        double actual_distance, actual_yaw_error;
        if (reachedGoal(actual_distance, actual_yaw_error))
        {
          completeCurrentGoal("ARRIVED");
        }
        else if (planner_manager_->hasTerminalTrajectory())
        {
          // 终端零速参考继续发布，等待控制器完成停车；安全回调仍检查终点。
          double collision_distance;
          if (!planner_manager_->checkTrajCollision(collision_distance, &odom_pos_))
            changeFSMExecState(GEN_NEW_TRAJ, "TERMINAL_INVALID");
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(), "Local trajectory ended: actual goal distance %.2f m, heading error %.2f rad; replanning.",
                      actual_distance, actual_yaw_error);
          changeFSMExecState(GEN_NEW_TRAJ, "LOCAL_END");
        }
        return;

      }
      else if (planner_manager_->hasTerminalTrajectory())
      {
        // 时间参考明显超前时，从实船状态恢复，避免按旧时间表停在终点之前。
        if ((pos.head<2>() - odom_pos_.head<2>()).norm() > 2.0)
          changeFSMExecState(REPLAN_TRAJ, "REFERENCE_DEVIATION");
        return;
      }
      else if (goal_distance < no_replan_thresh_ &&
               std::abs(goal_yaw_error) < no_replan_yaw_thresh_)
      {
        // 目标距离和航向误差均小于配置阈值时，暂不触发普通重规划。
        return;
      }
      else if (start_distance < replan_thresh_)
      {
        // 轨迹起点的平面距离小于配置阈值时，暂不触发普通重规划。
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      const bool goal_available = updateRouteGoal();
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      // 正常滚动沿用旧参考当前时刻的状态；失效或偏离后才从实测状态恢复。
      planner_manager_->sampleReplanStart(odom_pos_, start_pt_, start_vel_, start_acc_);

      std_msgs::msg::Empty replan_msg;
      replan_pub_->publish(replan_msg);

      bool success = planOrKeepSafeTrajectory(goal_available);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::publishActiveBspline() {
  const bool terminal_hold = exec_state_ == WAIT_TARGET && !have_target_ && holding_terminal_;
  if ((!terminal_hold && (exec_state_ != EXEC_TRAJ || !have_target_)) || !have_odom_ ||
      active_bspline_.control_points.empty() ||
      !planner_manager_->edt_environment_->sdf_map_->isMapReady() ||
      std::chrono::steady_clock::now() - last_odom_received_ > std::chrono::milliseconds(500)) {
    return;
  }

  double collision_distance;
  if (!planner_manager_->checkTrajCollision(collision_distance, &odom_pos_)) {
    if (terminal_hold) holding_terminal_ = false;
    else {
      std_msgs::msg::Header failure;
      failure.stamp = clock_->now();
      replan_failed_pub_->publish(failure);
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
    return;
  }
  bspline_pub_->publish(active_bspline_);
}

void KinoReplanFSM::checkCollisionCallback() {
  if (!planner_manager_->edt_environment_->sdf_map_->isMapReady())
  {
    return;
  }
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    // 只用目标 x、y 查询障碍；三维地图接口暂取 z=0，不能将航向当作高度。
    Eigen::Vector3d goal_map_pt(end_pt_(0), end_pt_(1), 0.0);
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(goal_map_pt, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(goal_map_pt, -1.0);

    if (dist <= 0.5 && route_active_) {
      // 航路目标由规划前的沿线检索统一选择，避免径向修正被前视刷新覆盖。
      if (exec_state_ == EXEC_TRAJ) changeFSMExecState(REPLAN_TRAJ, "BLOCKED_PATH_GOAL");
    } else if (dist <= 0.5) {
      // 只在 x、y 平面内选择新目标，保持原目标航向。
      const double dr = 0.5, dtheta = 30;
      double new_x, new_y, max_dist = -1.0;
      Eigen::Vector3d goal = end_pt_;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
      {
        for (double theta = -90; theta <= 270; theta += dtheta)
        {
          new_x = end_pt_(0) + r * cos(theta / 57.3);
          new_y = end_pt_(1) + r * sin(theta / 57.3);

          Eigen::Vector3d new_map_pt(new_x, new_y, 0.0);
          dist = planner_manager_->pp_.dynamic_ ?
              edt_env->evaluateCoarseEDT(new_map_pt, /* time to program start + */ info->duration_) :
              edt_env->evaluateCoarseEDT(new_map_pt, -1.0);

          if (dist > max_dist)
          {
            goal(0) = new_x;
            goal(1) = new_y;
            // goal(2) 保持原目标航向，不参与地图位置查询。
            max_dist = dist;
          }
        }
      }

      if (max_dist > 0.5) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        if (!route_active_ || route_terminal_) end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        cout << "goal near collision, keep retry" << endl;
        if (info->traj_id_ > 0)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");  // 尚无旧轨迹，不能采样空样条。
        }

        std_msgs::msg::Empty emt;
        replan_pub_->publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist, &odom_pos_);

    if (!safe) {
      std_msgs::msg::Header failure;
      failure.stamp = clock_->now();
      replan_failed_pub_->publish(failure);
      RCLCPP_WARN(node_->get_logger(), "current trajectory violates obstacle or speed limits.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::planOrKeepSafeTrajectory(bool goal_available) {
  KinodynamicAstar::computeUsvStateAndCommand(
      start_pt_, start_vel_, start_acc_, start_body_vel_, start_cmd_);
  end_body_vel_ = KinodynamicAstar::computeBodyVelocity(end_pt_, end_vel_);
  bool plan_success = goal_available &&
      planner_manager_->kinodynamicReplan(start_pt_, start_body_vel_, start_cmd_, end_pt_, end_body_vel_);

  if (plan_success) {
    if (planner_manager_->trajectoryWasReused()) return true;

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    usv_interfaces::msg::UsvBspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      usv_interfaces::msg::UsvBsplineControlPoint point;
      point.x = pos_pts(i, 0);
      point.y = pos_pts(i, 1);
      point.phi = pos_pts(i, 2);
      bspline.control_points.push_back(point);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    active_bspline_ = std::move(bspline);
    bspline_pub_->publish(active_bspline_);

    // 对已发布样条按不超过 0.1 s 的间隔采样，包含首尾点，供 RViz 显示。
    nav_msgs::msg::Path path;
    path.header.frame_id = planner_manager_->edt_environment_->sdf_map_->getFrameId();
    path.header.stamp = info->start_time_;
    const int samples = std::max(1, static_cast<int>(std::ceil(info->duration_ / 0.1)));
    path.poses.reserve(samples + 1);
    for (int i = 0; i <= samples; ++i) {
      const double time = info->duration_ * i / samples;
      const Eigen::Vector3d state = info->position_traj_.evaluateDeBoorT(time);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp = info->start_time_ + rclcpp::Duration::from_seconds(time);
      pose.pose.position.x = state(0);
      pose.pose.position.y = state(1);
      // 第三分量是原生船首航向 phi，单位 rad；平面轨迹高度保持为零。
      pose.pose.orientation.z = std::sin(0.5 * state(2));
      pose.pose.orientation.w = std::cos(0.5 * state(2));
      path.poses.push_back(pose);
    }
    bspline_path_pub_->publish(path);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    double collision_distance;
    if (planner_manager_->trajectoryMatchesGoalHeading(end_pt_(2)) &&
        planner_manager_->checkTrajCollision(collision_distance, &odom_pos_)) {
      RCLCPP_WARN(node_->get_logger(), "Replan failed; continuing validated previous trajectory while retrying.");
      return true;
    }
    std_msgs::msg::Header failure;
    failure.stamp = clock_->now();
    replan_failed_pub_->publish(failure);
    RCLCPP_WARN(node_->get_logger(), "generate new traj fail; old trajectory unsafe or incompatible with goal heading, withdrawn.");
    return false;
  }
}

}  // namespace fast_planner
