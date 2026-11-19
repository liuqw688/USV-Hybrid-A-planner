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
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <cmath>

namespace fast_planner {

void KinoReplanFSM::init(rclcpp::Node& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  trigger_     = false;
  node_        = &nh;
  clock_       = nh.get_clock();

  /*  fsm param  */
  target_type_ = nh.declare_parameter<int>("fsm/flight_type", -1);
  replan_thresh_ = nh.declare_parameter<double>("fsm/thresh_replan", -1.0);
  no_replan_thresh_ = nh.declare_parameter<double>("fsm/thresh_no_replan", -1.0);
  no_replan_yaw_thresh_ = nh.declare_parameter<double>("fsm/thresh_no_replan_yaw", -1.0);
  arrival_pos_tolerance_ = nh.declare_parameter<double>("fsm/arrival_pos_tolerance", -1.0);
  arrival_yaw_tolerance_ = nh.declare_parameter<double>("fsm/arrival_yaw_tolerance", -1.0);

  waypoint_num_ = nh.declare_parameter<int>("fsm/waypoint_num", -1);
  for (int i = 0; i < waypoint_num_; i++) {
    waypoints_[i][0] = nh.declare_parameter<double>("fsm/waypoint" + std::to_string(i) + "_x", -1.0);
    waypoints_[i][1] = nh.declare_parameter<double>("fsm/waypoint" + std::to_string(i) + "_y", -1.0);
    waypoints_[i][2] = nh.declare_parameter<double>("fsm/waypoint" + std::to_string(i) + "_z", -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  using namespace std::chrono_literals;
  exec_timer_   = nh.create_wall_timer(10ms, std::bind(&KinoReplanFSM::execFSMCallback, this));
  safety_timer_ = nh.create_wall_timer(50ms, std::bind(&KinoReplanFSM::checkCollisionCallback, this));

  waypoint_sub_ = nh.create_subscription<nav_msgs::msg::Path>(
      "/waypoint_generator/waypoints", 10,
      std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1));
  odom_sub_ = nh.create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 10,
      std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_  = nh.create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  new_pub_     = nh.create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  replan_failed_pub_ = nh.create_publisher<std_msgs::msg::Header>("/planning/replan_failed", 10);
  bspline_pub_ = nh.create_publisher<plan_manage::msg::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr& msg) {
  if (msg->poses.empty())
  {
    return;
  }
  if (planner_manager_->edt_environment_->sdf_map_->usesGridMap())
  {
    const std::string& frame = planner_manager_->edt_environment_->sdf_map_->getFrameId();
    const std::string& pose_frame = msg->poses[0].header.frame_id;
    if ((msg->header.frame_id.empty() && pose_frame.empty()) ||
        (!msg->header.frame_id.empty() && msg->header.frame_id != frame) ||
        (!pose_frame.empty() && pose_frame != frame))
    {
      RCLCPP_WARN(node_->get_logger(), "USV goal must be expressed in planning frame '%s'.", frame.c_str());
      return;
    }
  }
  if (msg->poses[0].pose.position.z < -0.1) return;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    const auto& q = msg->poses[0].pose.orientation;
    Eigen::Quaterniond goal_orientation(q.w, q.x, q.y, q.z);
    if (!std::isfinite(goal_orientation.norm()) || goal_orientation.norm() < 1e-6) {
      RCLCPP_WARN(node_->get_logger(), "USV goal has an invalid orientation.");
      return;
    }
    goal_orientation.normalize();
    const Eigen::Vector3d goal_heading = goal_orientation.toRotationMatrix().col(0);
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y,
        std::atan2(goal_heading.y(), goal_heading.x());

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  cout << "Triggered!" << endl;
  trigger_ = true;
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;
  retry_not_before_ = std::chrono::steady_clock::time_point{};

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
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
  const double phi = std::atan2(rot_x(1), rot_x(0));
  // This odometry publisher supplies linear velocity in the world frame.

  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = phi;
  odom_vel_(0) = msg->twist.twist.linear.x;                          // Reference-frame dot_x.
  odom_vel_(1) = msg->twist.twist.linear.y;                          // Reference-frame dot_y.
  odom_vel_(2) = msg->twist.twist.angular.z;    // Planar yaw rate r.

  have_odom_ = true;
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
      const double actual_distance = (end_pt_.head(2) - odom_pos_.head(2)).norm();
      const double heading_delta = end_pt_(2) - odom_pos_(2);
      const double actual_yaw_error = std::abs(std::atan2(std::sin(heading_delta), std::cos(heading_delta)));
      if (arrival_pos_tolerance_ > 0.0 && arrival_yaw_tolerance_ > 0.0 &&
          actual_distance <= arrival_pos_tolerance_ && actual_yaw_error <= arrival_yaw_tolerance_)
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "ARRIVED");
        return;
      }
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
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
        const double actual_distance = (end_pt_.head(2) - odom_pos_.head(2)).norm();
        const double heading_delta = end_pt_(2) - odom_pos_(2);
        const double actual_yaw_error = std::abs(std::atan2(std::sin(heading_delta), std::cos(heading_delta)));
        if (arrival_pos_tolerance_ > 0.0 && arrival_yaw_tolerance_ > 0.0 &&
            actual_distance <= arrival_pos_tolerance_ && actual_yaw_error <= arrival_yaw_tolerance_)
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "ARRIVED");
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(), "Local trajectory ended: actual goal distance %.2f m, heading error %.2f rad; replanning.",
                      actual_distance, actual_yaw_error);
          changeFSMExecState(GEN_NEW_TRAJ, "LOCAL_END");
        }
        return;

      }
      else if (goal_distance < no_replan_thresh_ &&
               std::abs(goal_yaw_error) < no_replan_yaw_thresh_)
      {
        // 距目标小于 5 m，且最短航向误差小于 30°，暂不触发普通重规划。
        return;
      }
      else if (start_distance < replan_thresh_)
      {
        // 距当前轨迹起点小于 4 m，暂不触发普通重规划，不混入航向角。
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      // The tracker may lag or lead the old time-parameterized reference.
      // Anchor replans to the latest measured position and world-frame velocity.
      // Odometry does not provide acceleration, so retain the initial-plan zero assumption.
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      std_msgs::msg::Empty replan_msg;
      replan_pub_->publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
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

    if (dist <= 1.5) {
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

      if (max_dist > 1.5) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
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
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      RCLCPP_WARN(node_->get_logger(), "current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  KinodynamicAstar::computeUsvStateAndCommand(
      start_pt_, start_vel_, start_acc_, start_body_vel_, start_cmd_);
  end_body_vel_ = KinodynamicAstar::computeBodyVelocity(end_pt_, end_vel_);
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_body_vel_, start_cmd_, end_pt_, end_body_vel_);

  if (plan_success) {

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::msg::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    // A consumer following the old path must brake if a new plan failed.
    std_msgs::msg::Header failure;
    failure.stamp = clock_->now();
    replan_failed_pub_->publish(failure);
    RCLCPP_WARN(node_->get_logger(), "generate new traj fail; old trajectory withdrawn.");
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
