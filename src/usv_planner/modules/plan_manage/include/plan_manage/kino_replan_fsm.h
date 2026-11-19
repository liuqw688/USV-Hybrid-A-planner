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



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <string>

#include <geometry_msgs/msg/point.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <usv_interfaces/msg/usv_bspline.hpp>
#include <usv_planner/waypoint_route.hpp>

using std::vector;

namespace fast_planner {

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ };
  enum class GoalMode { POSE = 1, PATH = 2 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int goal_mode_;
  double no_replan_thresh_, replan_thresh_;  // 平面距离阈值，单位 m。
  double arrival_pos_tolerance_, arrival_yaw_tolerance_;  // Actual odometry arrival criteria.
  double no_replan_yaw_thresh_;  // 接近目标的最短航向误差阈值，单位 rad。
  double path_speed_mps_;  ///< 滚动目标的通过速度，单位 m/s；任务末点要求零速。
  double route_range_m_;   ///< 与搜索前视及局部感知栅格共用的前视距离，单位 m。
  usv_planner::WaypointRoute route_;
  bool route_active_{false};
  bool route_terminal_{false};  ///< 当前局部目标已覆盖完整任务的末点。

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;
  std::chrono::steady_clock::time_point retry_not_before_{};
  std::chrono::steady_clock::time_point last_odom_received_{};
  usv_interfaces::msg::UsvBspline active_bspline_;
  bool holding_terminal_ {false};  ///< 最终停车后保留已验证参考，供控制器完成到达确认。

  Eigen::Vector3d odom_pos_;  // [x,y,phi], heading extracted from odometry orientation.
  Eigen::Vector3d odom_vel_;  // ENU pose derivative [x_dot_E,y_dot_N,r], converted from RFU odometry twist.
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_;   // [x,y,phi].
  Eigen::Vector3d start_vel_;  // Reference-frame pose derivative [dot_x,dot_y,r] for both planning modes.
  Eigen::Vector3d start_acc_;  // Zero assumption when acceleration is not measured.
  Eigen::Vector3d start_body_vel_, start_cmd_;  // Converted [u,v,r] and [v_cmd,w_cmd,0].
  Eigen::Vector3d end_pt_;   // Target [x,y,phi]; target source still needs later adaptation.
  Eigen::Vector3d end_vel_;  // Target world pose derivative; converted before manager call (currently zero).
  Eigen::Vector3d end_body_vel_;  // Converted target body velocity [u,v,r].
  int current_wp_;

  /* ROS2 utils */
  rclcpp::Node* node_ {nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, bspline_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr replan_failed_pub_;
  rclcpp::Publisher<usv_interfaces::msg::UsvBspline>::SharedPtr bspline_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bspline_path_pub_;

  /* helper functions */
  /** @brief 生成新轨迹；失败时仅保留仍通过安全检查的旧参考。
   *  @return 存在可继续执行的轨迹时返回 true，新搜索结果单独记录于规划日志。
   */
  bool planOrKeepSafeTrajectory(bool goal_available);
  void changeFSMExecState(FSM_EXEC_STATE new_state, const std::string& pos_call);
  void printFSMExecState();
  /** @brief 计算当前船位到目标的位置距离和最短航向误差。 */
  bool reachedGoal(double& distance, double& heading_error) const;

  /* ROS2 functions */
  void execFSMCallback();
  void checkCollisionCallback();
  /** @brief 定期发布仍可安全执行的轨迹，保持原始轨迹时间基准。 */
  void publishActiveBspline();
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void goalPathCallback(const nav_msgs::msg::Path::ConstSharedPtr& msg);
  void acceptGoal(const Eigen::Vector3d& goal, const std::string& source);
  /** @brief 沿实船进度选择感知范围内的安全航路目标；无候选时返回 false。 */
  bool updateRouteGoal();
  void completeCurrentGoal(const std::string& source);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

public:
  KinoReplanFSM(/* args */) {
  }
  ~KinoReplanFSM() {
  }

  void init(rclcpp::Node& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
