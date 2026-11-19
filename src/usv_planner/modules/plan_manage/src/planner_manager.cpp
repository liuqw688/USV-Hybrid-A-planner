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



#include <plan_manage/planner_manager.h>
#include <path_searching/kinodynamic_astar_ros.hpp>
#include <path_searching/usv_dynamics.hpp>
#include <path_searching/turning_constraint.hpp>
#include <path_searching/obstacle_speed_limit.hpp>
#include <bspline_opt/bspline_optimizer_ros.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace fast_planner {

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(rclcpp::Node& nh) {
  /* read algorithm parameters */

  pp_.max_vel_          = nh.declare_parameter<double>("manager/max_vel", -1.0);
  pp_.max_acc_          = nh.declare_parameter<double>("manager/max_acc", -1.0);
  pp_.max_jerk_         = nh.declare_parameter<double>("manager/max_jerk", -1.0);
  pp_.max_u_            = nh.declare_parameter<double>("search/max_u", -1.0);
  pp_.max_v_            = nh.declare_parameter<double>("search/max_v", -1.0);
  pp_.max_r_            = nh.declare_parameter<double>("search/max_r", -1.0);
  pp_.max_global_vel_   = nh.declare_parameter<double>("manager/max_global_vel", -1.0);
  pp_.max_yaw_rate_     = nh.declare_parameter<double>("manager/max_yaw_rate", -1.0);
  pp_.max_global_acc_   = nh.declare_parameter<double>("manager/max_global_acc", -1.0);
  pp_.max_yaw_acc_      = nh.declare_parameter<double>("manager/max_yaw_acc", -1.0);
  pp_.dynamic_          = nh.declare_parameter<int>("manager/dynamic_environment", -1);
  pp_.clearance_        = nh.declare_parameter<double>("manager/clearance_threshold", -1.0);
  pp_.local_traj_len_   = nh.declare_parameter<double>("manager/local_segment_length", -1.0);
  pp_.ctrl_pt_dist      = nh.declare_parameter<double>("manager/control_points_distance", -1.0);
  safety_replan_distance_ = nh.declare_parameter<double>("manager/safety_replan_distance", 0.5);
  obstacle_slowdown_distance_ = nh.declare_parameter<double>("manager/obstacle_slowdown_distance", 5.0);
  obstacle_max_speed_ = nh.declare_parameter<double>("manager/obstacle_max_speed", 1.0);
  if (!std::isfinite(obstacle_slowdown_distance_) || obstacle_slowdown_distance_ <= 0.0 ||
      !std::isfinite(obstacle_max_speed_) || obstacle_max_speed_ <= 0.0)
    throw std::invalid_argument("obstacle slowdown distance and speed must be positive and finite.");
  min_turn_radius_ = nh.declare_parameter<double>("manager/min_turn_radius", 8.0);
  if (!std::isfinite(min_turn_radius_) || min_turn_radius_ < 0.0)
    throw std::invalid_argument("manager/min_turn_radius must be finite and nonnegative.");
  if (!std::isfinite(safety_replan_distance_) || safety_replan_distance_ < 0.0)
  {
    throw std::invalid_argument("manager/safety_replan_distance must be finite and nonnegative.");
  }

  bool use_geometric_path   = nh.declare_parameter<bool>("manager/use_geometric_path", false);
  bool use_kinodynamic_path = nh.declare_parameter<bool>("manager/use_kinodynamic_path", false);
  bool use_topo_path        = nh.declare_parameter<bool>("manager/use_topo_path", false);
  bool use_optimization     = nh.declare_parameter<bool>("manager/use_optimization", false);

  clock_ = nh.get_clock();

  local_data_.traj_id_ = 0;
  local_data_.duration_ = 0.0;
  sdf_map_.reset(new SDFMap);
  // SDFMap expects a shared_ptr node in the current port
  sdf_map_->initMap(nh.shared_from_this());
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    configureKinodynamicAstar(nh, *kino_path_finder_);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      configureBsplineOptimizer(nh, *bspline_optimizers_[i]);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }
}

void FastPlannerManager::setGlobalWaypoints(const vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

double FastPlannerManager::limitGoalSpeed(const Eigen::Vector3d& pose, double requested_speed) const {
  const Eigen::Vector3d point(pose.x(), pose.y(), 0.0);
  return sdf_map_->getObstacleDistance(point) <= obstacle_slowdown_distance_ ?
    std::min(requested_speed, obstacle_max_speed_) : requested_speed;
}

bool FastPlannerManager::turningFeasible(NonUniformBspline& trajectory) const {
  if (min_turn_radius_ <= 0.0) return true;
  auto velocity = trajectory.getDerivative();
  auto acceleration = velocity.getDerivative();
  const double duration = trajectory.getTimeSum();
  const int count = std::max(1, static_cast<int>(std::ceil(duration / 0.025)));
  for (int i = 0; i <= count; ++i) {
    const double time = duration * i / count;
    const Eigen::Vector3d v = velocity.evaluateDeBoorT(time);
    const Eigen::Vector3d a = acceleration.evaluateDeBoorT(time);
    const auto ratios = turningRatios(v, a, min_turn_radius_);
    if (!v.allFinite() || !a.allFinite() || ratios.cwiseAbs().maxCoeff() > 1.005) {
      RCLCPP_WARN(rclcpp::get_logger("plan_manage"),
        "B-spline turn limit: t=%.3f speed=%.3f ground_ratio=%.3f yaw_ratio=%.3f",
        time, v.head<2>().norm(), ratios(0), ratios(1));
      return false;
    }
  }
  return true;
}

bool FastPlannerManager::collisionFree(NonUniformBspline& trajectory, double begin,
                                        double end, double& distance) const {
  const int count = std::max(1, static_cast<int>(std::ceil((end - begin) / 0.025)));
  auto velocity = trajectory.getDerivative();
  Eigen::VectorXd previous = trajectory.evaluateDeBoorT(begin);
  distance = 0.0;
  for (int i = 0; i <= count; ++i) {
    const double time = begin + (end - begin) * i / count;
    const Eigen::VectorXd pose = trajectory.evaluateDeBoorT(time);
    const Eigen::VectorXd speed = velocity.evaluateDeBoorT(time);
    if (!pose.allFinite() || !speed.allFinite()) return false;
    distance += (pose.head<2>() - previous.head<2>()).norm();
    previous = pose;
    const Eigen::Vector3d point(pose(0), pose(1), 0.0);
    if (sdf_map_->getInflateOccupancy(point) != 0 ||
        sdf_map_->getDistance(point) < safety_replan_distance_ ||
        !obstacleSpeedFeasible(sdf_map_->getObstacleDistance(point), speed.head<2>().norm(),
            obstacle_slowdown_distance_, obstacle_max_speed_)) return false;
  }
  return true;
}

bool FastPlannerManager::checkTrajCollision(double& distance, const Eigen::Vector3d* current_pose) {
  distance = 0.0;
  // 没有已发布轨迹时不读取默认 start_time_，避免首次规划失败后的
  // ROS 时间源不一致异常。
  if (local_data_.traj_id_ <= 0) return false;
  const double elapsed = (clock_->now() - local_data_.start_time_).seconds();
  if (elapsed < 0.0) return false;
  if (elapsed >= local_data_.duration_) {
    // 只有末速为零的终端轨迹可在结束后继续作为停车参考。
    const auto end_velocity = local_data_.velocity_traj_.evaluateDeBoorT(local_data_.duration_);
    if (!accepted_terminal_ || !end_velocity.allFinite() || end_velocity.norm() > 1e-6) return false;
    const auto end_pose = local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);
    if (current_pose != nullptr && (end_pose.head<2>() - current_pose->head<2>()).norm() > 2.0)
      return false;
    return collisionFree(local_data_.position_traj_, local_data_.duration_,
                         local_data_.duration_, distance);
  }
  double progress = elapsed;
  if (current_pose != nullptr) {
    double best_score = std::numeric_limits<double>::infinity();
    progress = collision_check_time_;
    const double end = std::min(local_data_.duration_, collision_check_time_ + 5.0);
    const int count = std::max(1, static_cast<int>(std::ceil((end - progress) / 0.05)));
    const double begin = progress;
    for (int i = 0; i <= count; ++i) {
      const double time = begin + (end - begin) * i / count;
      const Eigen::VectorXd pose = local_data_.position_traj_.evaluateDeBoorT(time);
      const double yaw = pose(2) - (*current_pose)(2);
      const double heading = std::atan2(std::sin(yaw), std::cos(yaw));
      const double score = (pose.head<2>() - current_pose->head<2>()).squaredNorm() +
          4.0 * heading * heading;
      if (score < best_score) { best_score = score; progress = time; }
    }
    const Eigen::VectorXd matched = local_data_.position_traj_.evaluateDeBoorT(progress);
    if ((matched.head<2>() - current_pose->head<2>()).norm() > 2.0) return false;
    collision_check_time_ = progress;
  }
  return collisionFree(local_data_.position_traj_, progress, local_data_.duration_, distance);
}

bool FastPlannerManager::sampleReplanStart(const Eigen::Vector3d& current_pose,
    Eigen::Vector3d& position, Eigen::Vector3d& velocity, Eigen::Vector3d& acceleration) {
  if (local_data_.traj_id_ <= 0 || !current_pose.allFinite()) return false;
  const double elapsed = (clock_->now() - local_data_.start_time_).seconds();
  if (elapsed < 0.0 || elapsed >= local_data_.duration_) return false;
  const Eigen::Vector3d pose = local_data_.position_traj_.evaluateDeBoorT(elapsed);
  const Eigen::Vector3d speed = local_data_.velocity_traj_.evaluateDeBoorT(elapsed);
  const Eigen::Vector3d accel = local_data_.acceleration_traj_.evaluateDeBoorT(elapsed);
  if (!pose.allFinite() || !speed.allFinite() || !accel.allFinite()) return false;
  const double yaw = pose(2) - current_pose(2);
  if ((pose.head<2>() - current_pose.head<2>()).norm() > 2.0 ||
      std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) > std::acos(-1.0) / 6.0) return false;
  const Eigen::Vector3d body = KinodynamicAstar::computeBodyVelocity(pose, speed);
  if (std::abs(body(0)) > pp_.max_u_ || std::abs(body(1)) > pp_.max_v_ ||
      std::abs(body(2)) > pp_.max_r_) return false;
  const Eigen::Vector3d point(pose(0), pose(1), 0.0);
  if (sdf_map_->getInflateOccupancy(point) != 0 ||
      sdf_map_->getDistance(point) < safety_replan_distance_) return false;
  position = pose;
  velocity = speed;
  acceleration = accel;
  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::trajectoryMatchesGoalHeading(double goal_heading) {
  if (local_data_.traj_id_ <= 0 || !kino_path_finder_ || !std::isfinite(goal_heading)) return false;
  const double heading = local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_)(2);
  const double delta = heading - goal_heading;
  return std::isfinite(delta) &&
      std::abs(std::atan2(std::sin(delta), std::cos(delta))) < kino_path_finder_->getShotYawTolerance();
}

bool FastPlannerManager::reuseTerminalTrajectory(const Eigen::Vector3d& current_pose,
    const Eigen::Vector3d& goal_pose, const Eigen::Vector3d& goal_velocity) {
  if (!accepted_terminal_ || !goal_pose.allFinite() || !goal_velocity.allFinite()) return false;
  const double yaw = goal_pose(2) - accepted_goal_(2);
  if ((goal_pose.head<2>() - accepted_goal_.head<2>()).norm() > 1e-6 ||
      std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) > 1e-6 ||
      (goal_velocity - accepted_goal_.tail<3>()).norm() > 1e-6) return false;
  Eigen::Vector3d position, velocity, acceleration;
  double distance;
  return sampleReplanStart(current_pose, position, velocity, acceleration) &&
      checkTrajCollision(distance, &current_pose);
}

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_body_vel,
                                           Eigen::Vector3d start_cmd, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_body_vel) {
  // 使用稳态时钟统计完整规划调用，包括失败、参数化和轨迹信息更新。
  struct PlanningTiming {
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    bool success = false;
    ~PlanningTiming() {
      const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "Planning wall time: %.3f ms success=%d", ms, success);
    }
  } timing;

  trajectory_reused_ = false;

  if (!sdf_map_->isMapReady())
  {
    return false;
  }
  if (reuseTerminalTrajectory(start_pt, end_pt, end_body_vel)) {
    trajectory_reused_ = true;
    timing.success = true;
    RCLCPP_INFO(rclcpp::get_logger("plan_manage"),
                "Validated terminal reference reused; curve and original clock unchanged.");
    return true;
  }
  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_body_vel.transpose() << ", "
       << "start_cmd: " << start_cmd.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_body_vel.transpose()
       << endl;

  rclcpp::Time t1;

  // 暂存规划开始时间，失败时保留旧轨迹的时间信息。
  rclcpp::Time plan_start_time = clock_->now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_body_vel = start_body_vel;
  Eigen::Vector3d init_cmd = start_cmd;

  // kinodynamic path searching

  t1 = clock_->now();

  kino_path_finder_->reset();

  // 前端最多使用总软预算的 80%，为样条拟合、优化和发布前验碰保留时间。
  const auto search_deadline = timing.start +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(0.8 * kino_path_finder_->getSearchTimeLimit()));
  int status = kino_path_finder_->search(start_pt, start_body_vel, start_cmd,
    end_pt, end_body_vel, true, false, -1.0, search_deadline);

  if (status == KinodynamicAstar::TIMEOUT) return false;

  if (status == KinodynamicAstar::NEAR_END) {
    RCLCPP_ERROR(rclcpp::get_logger("plan_manage"),
                 "Terminal shot fallback failed; refusing an incomplete goal trajectory.");
    return false;
  }

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // Retry without restricting the first expansion to start_cmd.
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_body_vel, start_cmd,
      end_pt, end_body_vel, false, false, -1.0, search_deadline);

    if (status == KinodynamicAstar::NO_PATH || status == KinodynamicAstar::NEAR_END ||
        status == KinodynamicAstar::TIMEOUT) {
      cout << "[kino replan]: search and terminal shot failed." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.1);

  t_search = (clock_->now() - t1).seconds();

  // parameterize the path to bspline

  double                  ts = pp_.ctrl_pt_dist / pp_.max_u_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);
  const bool terminal_stop = status == KinodynamicAstar::REACH_END && end_body_vel.norm() <= 1e-6;
  Eigen::Vector3d end_pose_velocity = start_end_derivatives[1];
  if (terminal_stop) end_pose_velocity.setZero();

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts,
                                           terminal_stop);
  if (ctrl_pts.rows() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("plan_manage"), "B-spline parameterization failed.");
    return false;
  }
  if (!NonUniformBspline::enforceEndVelocity(ctrl_pts, ts, end_pose_velocity)) {
    RCLCPP_ERROR(rclcpp::get_logger("plan_manage"), "Failed to enforce the requested terminal velocity.");
    return false;
  }
  NonUniformBspline initial_fit(ctrl_pts, 3, ts);
  if (!initial_fit.enforceStartState(start_pt, start_end_derivatives[0], start_end_derivatives[2]))
    return false;
  ctrl_pts = initial_fit.getControlPoint();
  const Eigen::MatrixXd initial_ctrl_pts = ctrl_pts;

  // bspline trajectory optimization

  t1 = clock_->now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  // 首尾各三个控制点保留搜索给出的位姿及导数，避免优化后再投影末速引入曲率尖峰。

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);
  if (!NonUniformBspline::enforceEndVelocity(ctrl_pts, ts, end_pose_velocity)) {
    RCLCPP_ERROR(rclcpp::get_logger("plan_manage"),
                 "Failed to preserve the requested terminal velocity after B-spline optimization.");
    return false;
  }

  t_opt = (clock_->now() - t1).seconds();

  // iterative time adjustment

  t1                    = clock_->now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();
  Eigen::Vector3d velocity_limits(pp_.max_global_vel_, pp_.max_global_vel_, pp_.max_yaw_rate_);
  Eigen::Vector3d acceleration_limits(pp_.max_global_acc_, pp_.max_global_acc_, pp_.max_yaw_acc_);
  pos.setPhysicalLimits(velocity_limits, acceleration_limits);
  // 时间重分配会改变首端导数；每次调整后按实际节点恢复实测初态及船模加速度。
  const auto restore_boundaries = [&](NonUniformBspline& curve) {
    return curve.enforceStartState(start_pt, start_end_derivatives[0], start_end_derivatives[2]) &&
        curve.enforceEndVelocity(end_pose_velocity);
  };
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible) {

    pos.reallocateTime();
    if (!restore_boundaries(pos)) {
      RCLCPP_ERROR(rclcpp::get_logger("plan_manage"),
                   "Failed to restore B-spline boundaries after time reallocation.");
      return false;
    }
    feasible = pos.checkFeasibility(false);

    if (++iter_num >= 3) break;
  }

  double collision_distance;
  if (!collisionFree(pos, 0.0, pos.getTimeSum(), collision_distance) || !turningFeasible(pos)) {
    RCLCPP_WARN(rclcpp::get_logger("plan_manage"),
                "Optimized B-spline violates clearance/turning limit; validating the search fit.");
    pos = NonUniformBspline(initial_ctrl_pts, 3, ts);
    pos.setPhysicalLimits(velocity_limits, acceleration_limits);
    for (int i = 0; i < 3 && !pos.checkFeasibility(false); ++i) {
      pos.reallocateTime();
      if (!restore_boundaries(pos)) return false;
    }
    if (!collisionFree(pos, 0.0, pos.getTimeSum(), collision_distance) || !turningFeasible(pos)) {
      RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "Search B-spline fit also violates clearance/turning limit; refusing publication.");
      return false;
    }
  }

  if (terminal_stop) {
    const Eigen::VectorXd end_velocity = pos.getDerivative().evaluateDeBoorT(pos.getTimeSum());
    if (!end_velocity.allFinite() || end_velocity.norm() > 1e-8) {
      RCLCPP_ERROR(rclcpp::get_logger("plan_manage"),
                   "Terminal B-spline velocity is not zero: %.3e", end_velocity.norm());
      return false;
    }
  }

  // 优化和时间调整可能改变曲线末端，最终按样条位姿而非搜索节点验收。
  const double end_heading = pos.evaluateDeBoorT(pos.getTimeSum())(2);
  const double heading_error = std::atan2(std::sin(end_heading - end_pt(2)),
                                          std::cos(end_heading - end_pt(2)));
  if (!std::isfinite(heading_error) || std::abs(heading_error) >=
      kino_path_finder_->getShotYawTolerance()) {
    RCLCPP_WARN(rclcpp::get_logger("plan_manage"),
                "B-spline endpoint heading differs from goal by %.3f rad; refusing publication.",
                heading_error);
    return false;
  }

  // pos.checkFeasibility(true);

  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) RCLCPP_ERROR(rclcpp::get_logger("plan_manage"), "reallocate error.");

  t_adjust = (clock_->now() - t1).seconds();

  // save planned results

  local_data_.position_traj_ = pos;
  local_data_.start_time_ = plan_start_time;
  accepted_terminal_ = terminal_stop;
  accepted_goal_ << end_pt, end_body_vel;
  collision_check_time_ = 0.0;
  kino_path_finder_->cacheSuccessfulPath();

  double t_total = t_search + t_opt + t_adjust;
  const auto segments = kino_path_finder_->getSegmentCounts();
  const char* source = segments.first && segments.second ? "search+shot" :
                       segments.second ? "shot" : segments.first ? "search" : "empty";
  cout << "[kino replan]: source: " << source << " (search segments: " << segments.first
       << ", shot segments: " << segments.second << "), time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  timing.success = true;
  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // generate global reference trajectory

  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // write position matrix
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  
  std::cout << "[planGlobalTraj]: max_vel=" << pp_.max_vel_ << ", max_acc=" << pp_.max_acc_ << std::endl;
  
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
    std::cout << "  Segment " << i << ": dist=" << (pos.row(i + 1) - pos.row(i)).norm() 
              << ", time=" << time(i) << std::endl;
  }

  time(0) *= 2.0;
  time(0) = max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = max(1.0, time(time.rows() - 1));
  
  std::cout << "[planGlobalTraj]: Total time allocation=" << time.sum() << "s" << std::endl;

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = clock_->now();
  global_data_.setGlobalTraj(gl_traj, time_now);
  
  std::cout << "[planGlobalTraj]: Global traj duration=" << global_data_.global_duration_ 
            << "s, with " << pt_num << " waypoints" << std::endl;

  // truncate a local trajectory

  double            dt, duration;
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  rclcpp::Time t1;

  /* truncate a new local segment for replanning */
  rclcpp::Time time_now = clock_->now();
  double    t_now    = (time_now - global_data_.global_start_time_).seconds();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;

  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  // Hybrid approach: leverage 50ms timer's work for optimization
  // Fast path: if collide flag is false, 50ms timer already confirmed no collisions
  if (!collide) {
    RCLCPP_DEBUG(rclcpp::get_logger("plan_manage"), "[Topo] Fast path: no collisions (from 50ms timer), using refineTraj");
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);
    return true;
  }
  
  // Slow path: collide flag is true - do full collision range computation
  plan_data_.initial_local_segment_ = init_traj;
  vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
  findCollisionRange(colli_start, colli_end, start_pts, end_pts);
  
  // Determine if topological replanning is needed based on actual collision detection
  // Valid collision range: must have both start and end points (trajectory enters and exits obstacle)
  bool has_collision = (colli_start.size() > 0 && colli_end.size() > 0);
  
  if (!has_collision) {
    // Full check found no valid collision range (edge case)
    RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "[Topo] Full check: no collision range found, using refineTraj");
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);
    return true;
  }
  
  // Valid collision range found - use topological replanning
  RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Topo] Collisions confirmed (start=%zu, end=%zu), triggering topological replan", 
              colli_start.size(), colli_end.size());

  if (colli_start.size() == 1 && colli_end.size() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "Init traj ends in obstacle, no replanning.");
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
    updateTrajInfo();
    return true;

  } else if (colli_end.size() == 0) {
    // Trajectory ends in obstacle with multiple collision segments - can't replan
    RCLCPP_WARN(rclcpp::get_logger("plan_manage"), 
                "Trajectory ends in obstacle (colli_start=%zu, colli_end=%zu), no replanning.",
                colli_start.size(), colli_end.size());
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
    updateTrajInfo();
    return true;

  } else {
      NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "[Topo] No path found! Raw paths: %zu, Filtered: %zu, Select: %zu", 
                    raw_paths.size(), filtered_paths.size(), select_paths.size());
        RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "[Topo] Collision range: start=%zu points, end=%zu points", 
                    colli_start.size(), colli_end.size());
        if (colli_start.size() > 0 && colli_end.size() > 0) {
          RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "[Topo] Collision start: (%.2f, %.2f, %.2f), end: (%.2f, %.2f, %.2f)",
                    colli_start.front()(0), colli_start.front()(1), colli_start.front()(2),
                    colli_end.back()(0), colli_end.back()(1), colli_end.back()(2));
        }
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("plan_manage"), 
                  "[Topo] Found %zu candidate paths (raw: %zu, filtered: %zu)", 
                  select_paths.size(), raw_paths.size(), filtered_paths.size());
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Optimize]: ---------");
      t1 = clock_->now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (clock_->now() - t1).seconds();
      cout << "[planner]: optimization time: " << t_opt << endl;
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
      updateTrajInfo();
      return true;
    }
  
  // Should never reach here, but add return for safety
  updateTrajInfo();
  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  rclcpp::Time t1 = clock_->now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "[Refine]: cost %.3f seconds, time change: %.3f",
              (clock_->now() - t1).seconds(), time_inc);
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = min(1.01, ratio);
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  rclcpp::Time t1;
  double    tm1, tm2, tm3;

  t1 = clock_->now();

  // parameterize B-spline according to the length of guide path
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "what guide");

  tm1 = (clock_->now() - t1).seconds();
  t1  = clock_->now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (clock_->now() - t1).seconds();
  t1  = clock_->now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (clock_->now() - t1).seconds();
  RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "optimization %d cost %.3f, %.3f, %.3f seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  std::cout << "[reparamLocalTraj]: Sampled " << point_set.size() << " points, dt=" << dt 
            << ", duration=" << duration << std::endl;
  
  if (point_set.size() < 2) {
    std::cerr << "[reparamLocalTraj ERROR]: Insufficient points (" << point_set.size() 
              << "), need at least 2!" << std::endl;
  }

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int    sn = ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "plan yaw");
  auto t1 = clock_->now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = usv_dynamics::headingFromForwardVector(pd(0), pd(1));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(
      usv_dynamics::headingFromForwardVector(end_v(0), end_v(1)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (clock_->now() - t1).seconds() << std::endl;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
