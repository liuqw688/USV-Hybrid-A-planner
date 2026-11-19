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




#include <plan_manage/topo_replan_fsm.h>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>

namespace fast_planner {

void TopoReplanFSM::init(rclcpp::Node& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  collide_     = false;
  emergency_replan_count_ = 0;
  node_        = &nh;
  clock_       = nh.get_clock();

  /*  fsm param  */
  target_type_               = nh.declare_parameter<int>("fsm/flight_type", -1);
  replan_time_threshold_     = nh.declare_parameter<double>("fsm/thresh_replan", -1.0);
  replan_distance_threshold_ = nh.declare_parameter<double>("fsm/thresh_no_replan", -1.0);
  waypoint_num_              = nh.declare_parameter<int>("fsm/waypoint_num", -1);
  act_map_                   = nh.declare_parameter<bool>("fsm/act_map", false);
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
  exec_timer_   = nh.create_wall_timer(10ms, std::bind(&TopoReplanFSM::execFSMCallback, this));
  safety_timer_ = nh.create_wall_timer(20ms, std::bind(&TopoReplanFSM::checkCollisionCallback, this));

  waypoint_sub_ = nh.create_subscription<nav_msgs::msg::Path>(
      "/waypoint_generator/waypoints", 10,
      std::bind(&TopoReplanFSM::waypointCallback, this, std::placeholders::_1));
  odom_sub_ = nh.create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 10,
      std::bind(&TopoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_  = nh.create_publisher<std_msgs::msg::Empty>("/planning/replan", 20);
  new_pub_     = nh.create_publisher<std_msgs::msg::Empty>("/planning/new", 20);
  bspline_pub_ = nh.create_publisher<plan_manage::msg::Bspline>("/planning/bspline", 20);
}

void TopoReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;
  cout << "Triggered!" << endl;

  vector<Eigen::Vector3d> global_wp;
  if (target_type_ == TARGET_TYPE::REFENCE_PATH) {
    // Use ALL waypoints from the incoming Path message (like waypoint_generator does)
    cout << "[FSM]: REFERENCE_PATH mode, received " << msg->poses.size() << " poses" << endl;
    for (size_t i = 0; i < msg->poses.size(); ++i) {
      Eigen::Vector3d pt;
      pt(0) = msg->poses[i].pose.position.x;
      pt(1) = msg->poses[i].pose.position.y;
      pt(2) = msg->poses[i].pose.position.z;
      global_wp.push_back(pt);
      cout << "  Waypoint " << i << ": " << pt.transpose() << endl;
    }
    cout << "[FSM]: Parsed " << global_wp.size() << " waypoints" << endl;
  } else {

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
      target_point_(0) = msg->poses[0].pose.position.x;
      target_point_(1) = msg->poses[0].pose.position.y;
      target_point_(2) = msg->poses[0].pose.position.z;  
      std::cout << "manual: " << target_point_.transpose() << std::endl;

    } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
      target_point_(0) = waypoints_[current_wp_][0];
      target_point_(1) = waypoints_[current_wp_][1];
      target_point_(2) = waypoints_[current_wp_][2];

      current_wp_ = (current_wp_ + 1) % waypoint_num_;
      std::cout << "preset: " << target_point_.transpose() << std::endl;
    }

    global_wp.push_back(target_point_);
    visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  }

  planner_manager_->setGlobalWaypoints(global_wp);
  end_vel_.setZero();
  have_target_ = true;
  trigger_     = true;
  emergency_replan_count_ = 0;  // Reset emergency replan counter on new waypoints

  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }
}

void TopoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[6] = { "INIT",
                          "WAIT_TARGET",
                          "GEN_NEW_TRAJ",
                          "REPLAN_TRAJ",
                          "EXEC_TRAJ",
                          "REPLAN_"
                          "NEW" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void TopoReplanFSM::printFSMExecState() {
  string state_str[6] = { "INIT",
                          "WAIT_TARGET",
                          "GEN_NEW_TRAJ",
                          "REPLAN_TRAJ",
                          "EXEC_TRAJ",
                          "REPLAN_"
                          "NEW" };
  cout << "state: " + state_str[int(exec_state_)] << endl;
}

void TopoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "no trigger_." << endl;
    fsm_num = 0;
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
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      new_pub_->publish(std_msgs::msg::Empty());
      /* topo path finding and optimization */
      bool success = callTopologicalTraj(1);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */

      GlobalTrajData* global_data = &planner_manager_->global_data_;
      rclcpp::Time    time_now    = clock_->now();
      double          t_cur       = (time_now - global_data->global_start_time_).seconds();

      if (t_cur > global_data->global_duration_ - 1e-2) {
        // Check if we've actually reached the final waypoint
        LocalTrajData* info = &planner_manager_->local_data_;
        Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
        
        // Check if we have waypoints to compare against
        if (planner_manager_->plan_data_.global_waypoints_.size() > 0) {
          Eigen::Vector3d final_wp = planner_manager_->plan_data_.global_waypoints_.back();
          double dist_to_final = (cur_pos - final_wp).norm();
          
          if (dist_to_final < 1.0) {
            // Close enough to final waypoint - mission complete
            RCLCPP_INFO(node_->get_logger(), "Reached final waypoint (dist=%.3f m). Mission complete.", dist_to_final);
            have_target_ = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
          } else {
            // Global traj expired but not at final waypoint - continue replanning
            RCLCPP_WARN(node_->get_logger(), "Global traj expired (t_cur=%.2f, duration=%.2f) but not at final waypoint (dist=%.3f m). Continuing replan...", 
                       t_cur, global_data->global_duration_, dist_to_final);
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        } else {
          // No waypoints - just stop
          RCLCPP_INFO(node_->get_logger(), "Global traj expired. No more waypoints.");
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        return;

      } else {
        LocalTrajData*  info      = &planner_manager_->local_data_;
        Eigen::Vector3d start_pos = info->start_pos_;
        t_cur                     = (time_now - info->start_time_).seconds();

        if (t_cur > replan_time_threshold_) {

          if (!global_data->localTrajReachTarget()) {
            changeFSMExecState(REPLAN_TRAJ, "FSM");

          } else {
            Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
            Eigen::Vector3d end_pos = info->position_traj_.evaluateDeBoorT(info->duration_);
            if ((cur_pos - end_pos).norm() > replan_distance_threshold_)
              changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      rclcpp::Time      time_now = clock_->now();
      double         t_cur    = (time_now - info->start_time_).seconds();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      bool success = callTopologicalTraj(2);
      if (success) {
        // Reset emergency replan tracking on successful replan
        emergency_replan_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        RCLCPP_WARN(node_->get_logger(), "Replan fail, retrying...");
      }

      break;
    }
    case REPLAN_NEW: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      rclcpp::Time      time_now = clock_->now();
      double         t_cur    = (time_now - info->start_time_).seconds();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      /* inform server */
      new_pub_->publish(std_msgs::msg::Empty());

      bool success = callTopologicalTraj(1);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }
  }
}

void TopoReplanFSM::checkCollisionCallback() {
  LocalTrajData* info = &planner_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  if (false) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ info->duration_) :
        edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool         new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset target_point_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        target_point_ = goal;
        have_target_  = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      if (dist > 0.5) {
        RCLCPP_WARN(node_->get_logger(), "[SAFETY] current traj %.3f m to collision (state=%d), triggering REPLAN_TRAJ", 
                   dist, (int)exec_state_);
        collide_ = true;
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      } else {
        // Very close to collision (<0.5m) - try aggressive replan instead of stopping
        emergency_replan_count_++;
        
        RCLCPP_ERROR(node_->get_logger(), "[SAFETY] current traj %.3f m to collision, attempting emergency replan! (state=%d, count=%d)", 
                    dist, (int)exec_state_, emergency_replan_count_);
        replan_pub_->publish(std_msgs::msg::Empty());
        
        // Stop after 3 failed attempts
        if (emergency_replan_count_ > 3) {
          RCLCPP_ERROR(node_->get_logger(), "[SAFETY] Emergency replan failed %d times. Stopping.", 
                      emergency_replan_count_);
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "SAFETY");
          emergency_replan_count_ = 0;
        } else {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
      }
    } else {
      if (collide_) {
        // Just cleared collision - reset emergency replan tracking
        RCLCPP_INFO(node_->get_logger(), "[SAFETY] Trajectory now safe (dist > 6m), clearing collision flag");
        emergency_replan_count_ = 0;  // Reset on trajectory becoming safe
      }
      collide_ = false;
    }
  }
}


bool TopoReplanFSM::callTopologicalTraj(int step) {
  bool plan_success;

  if (step == 1) {
    plan_success = planner_manager_->planGlobalTraj(start_pt_);
  } else {
    plan_success = planner_manager_->topoReplan(collide_);
  }

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    LocalTrajData* locdat = &planner_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    plan_manage::msg::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = locdat->start_time_;
    bspline.traj_id    = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getInterval();

    bspline_pub_->publish(bspline);

    /* visualize new trajectories */

    MidPlanData* plan_data = &planner_manager_->plan_data_;
    visualization_->drawPolynomialTraj(planner_manager_->global_data_.global_traj_, 0.05,
                                       Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), false,
                                0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos2_, 0.075);
    visualization_->drawYawTraj(locdat->position_traj_, locdat->yaw_traj_, plan_data->dt_yaw_);

    return true;
  } else {
    return false;
  }
}
// TopoReplanFSM::
}  // namespace fast_planner
