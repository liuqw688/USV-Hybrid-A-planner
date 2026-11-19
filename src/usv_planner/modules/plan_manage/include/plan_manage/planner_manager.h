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



#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <plan_manage/plan_container.hpp>

#include <rclcpp/rclcpp.hpp>

namespace fast_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  friend class FastPlannerManagerTest;
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  // Poses: [x,y,phi] in the reference frame; velocities: body-frame [u,v,r].
  // start_cmd: [v_cmd, w_cmd, unused], not world-frame acceleration.
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_body_vel, Eigen::Vector3d start_cmd,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_body_vel);
  /** @brief 本次调用是否仅验证并继续完整终端参考，未生成新样条。 */
  bool trajectoryWasReused() const { return trajectory_reused_; }
  /** @brief 当前参考是否为已验证的任务终端停车轨迹。 */
  bool hasTerminalTrajectory() const { return accepted_terminal_; }
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);
  bool topoReplan(bool collide);

  void planYaw(const Eigen::Vector3d& start_yaw);

  void initPlanModules(rclcpp::Node& nh);
  void setGlobalWaypoints(const vector<Eigen::Vector3d>& waypoints);

  /** @brief 按目标到输入地图障碍的距离限制通过速度，单位 m/s。 */
  double limitGoalSpeed(const Eigen::Vector3d& pose, double requested_speed) const;

  /** @brief 校验有效旧轨迹；提供实测位姿时，先沿上次投影进度匹配当前船位。 */
  bool checkTrajCollision(double& distance, const Eigen::Vector3d* current_pose = nullptr);
  /** @brief 判断当前样条终点航向是否满足当前目标航向容差。 */
  bool trajectoryMatchesGoalHeading(double goal_heading);

  /**
   * @brief 按旧参考的原始时钟采样重规划初态，避免刷新消息重置执行进度。
   * @param current_pose 实测 [x,y,phi]，用于检查参考是否仍在跟踪范围内。
   * @param position 输出参考位姿；失败时不修改。
   * @param velocity 输出世界系一阶导数；失败时不修改。
   * @param acceleration 输出世界系二阶导数；失败时不修改。
   * @return 参考有效、初态安全且符合搜索状态边界时返回 true。
   */
  bool sampleReplanStart(const Eigen::Vector3d& current_pose, Eigen::Vector3d& position,
                         Eigen::Vector3d& velocity, Eigen::Vector3d& acceleration);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;

private:
  /* main planning algorithms & modules */
  SDFMap::Ptr sdf_map_;
  rclcpp::Clock::SharedPtr clock_;
  double safety_replan_distance_ {0.5};
  double obstacle_slowdown_distance_ {5.0};  ///< 输入地图占用区域周围的限速距离，单位 m。
  double obstacle_max_speed_ {1.0};  ///< 限速区的对地合速度上限，单位 m/s。
  double min_turn_radius_ {0.0};  ///< 发布前对真实曲线校验的最小半径，单位 m。
  /** @brief 密集检查真实样条曲率和艏摇转弯率；停车段采用统一低速正则化。 */
  bool turningFeasible(NonUniformBspline& trajectory) const;
  double collision_check_time_ {0.0};  ///< 当前轨迹的单调投影进度，单位 s。
  bool accepted_terminal_ {false};
  bool trajectory_reused_ {false};
  Eigen::Matrix<double, 6, 1> accepted_goal_;

  /** @brief 目标不变且完整终端参考仍可执行时，保留其原始曲线与时钟。 */
  bool reuseTerminalTrajectory(const Eigen::Vector3d& current_pose,
                              const Eigen::Vector3d& goal_pose,
                              const Eigen::Vector3d& goal_velocity);

  /** @brief 检查样条区间的碰撞和障碍附近速度，包含区间两端。 */
  bool collisionFree(NonUniformBspline& trajectory, double begin, double end,
                     double& distance) const;

  unique_ptr<Astar> geo_path_finder_;
  unique_ptr<KinodynamicAstar> kino_path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  void selectBestTraj(NonUniformBspline& traj);
  void refineTraj(NonUniformBspline& best_traj, double& time_inc);
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  // heading planning
  void calcNextYaw(const double& last_yaw, double& yaw);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION
};
}  // namespace fast_planner

#endif
