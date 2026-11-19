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

#include <path_searching/kinodynamic_astar.h>
#include <cmath>
#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <Eigen/Cholesky>
#include <stdexcept>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_body_vel, Eigen::Vector3d start_cmd,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_body_vel, bool init, bool dynamic, double time_start)
{
  if (!edt_environment_->sdf_map_->isMapReady())
  {
    return NO_PATH;
  }
  if (edt_environment_->sdf_map_->usesGridMap())
  {
    // 初始化早于第一张地图；每次搜索刷新实际地图原点和范围。
    edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);
    if (edt_environment_->sdf_map_->getInflateOccupancy(start_pt) != 0 ||
        edt_environment_->sdf_map_->getInflateOccupancy(end_pt) != 0)
    {
      return NO_PATH;
    }
  }
  start_vel_ = computeGlobalVelocity(start_pt, start_body_vel);
  start_cmd_ = start_cmd;
  start_cmd_(0) = std::clamp(start_cmd_(0), min_v_cmd_, max_v_cmd_);
  start_cmd_(2) = 0.0;  // Reserved component is not a control input.

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->is_shot_segment = false;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_body_vel;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_body_vel;
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.insert(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr neighbor = NULL;
  bool init_search = init;
  int shot_failures = 0;  // Reset for each search, including a later replan.
  constexpr int max_shot_failures = 3;
  PathNodePtr best_progress = nullptr;
  double best_distance = (start_pt.head(2) - end_pt.head(2)).norm();

  while (!open_set_.empty())
  {
    cur_node = *open_set_.begin();

    // Terminate?
    const double distance_from_start = (cur_node->state.head(2) - start_pt.head(2)).norm();
    const double distance_to_goal = (cur_node->state.head(2) - end_pt.head(2)).norm();
    const double delta_phi = end_pt(2) - cur_node->state(2);
    const double heading_error = std::atan2(std::sin(delta_phi), std::cos(delta_phi));

    if (cur_node->parent != nullptr && distance_to_goal + resolution_ < best_distance)
    {
      best_progress = cur_node;
      best_distance = distance_to_goal;
    }

    bool reach_horizon = distance_from_start >= horizon_;
    bool near_end = distance_to_goal <= goal_pos_tolerance_ &&
                    std::abs(heading_error) <= goal_yaw_tolerance_;

    // Try each shot at a distinct expanded state. A failed shot must not
    // immediately terminate the search or repeat at the exact same state.
    if (near_end && shot_failures < max_shot_failures &&
        use_node_num_ + 4 <= allocate_num_)
    {
      estimateHeuristic(cur_node->state, end_state, time_to_goal);
      if (computeShotTraj(cur_node->state, end_state, time_to_goal))
      {
        std::cout << "reach end (shot)" << std::endl;
        return REACH_END;
      }
      ++shot_failures;
      RCLCPP_INFO(rclcpp::get_logger("path_searching"),
                  "Shot failed %d/%d at distance %.2f m; continuing search%s",
                  shot_failures, max_shot_failures, distance_to_goal,
                  shot_failures == max_shot_failures ? " without further shots" : "");
    }

    // Ordinary search may terminate only at a collision-checked node meeting
    // the same terminal pose and body-velocity tolerances as the shot.
    if (shot_failures >= max_shot_failures && cur_node->parent != nullptr &&
        distance_to_goal <= shot_pos_tolerance_ &&
        std::abs(heading_error) <= shot_yaw_tolerance_ &&
        std::abs(cur_node->state(3) - end_state(3)) <= shot_u_tolerance_ &&
        std::abs(cur_node->state(4) - end_state(4)) <= shot_v_tolerance_ &&
        std::abs(cur_node->state(5) - end_state(5)) <= shot_r_tolerance_)
    {
      retrievePath(cur_node);
      std::cout << "reach end (search; terminal tolerances met)" << std::endl;
      return REACH_END;
    }
    if (reach_horizon)
    {
      retrievePath(cur_node);
      std::cout << "reach horizon" << std::endl;
      return REACH_HORIZON;
    }
    open_set_.erase(open_set_.begin());
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    const double time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    const int res = 5; //sample tate 
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_cmd_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      // 两路命令各取 5 个样本，包含上下限；转向中间样本显式置零。
      for (int i = 0; i <= res - 1; ++i)
      {
        double v_cmd = min_v_cmd_ + (max_v_cmd_ - min_v_cmd_) * i / (res - 1);
        for (int j = 0; j <= res - 1; ++j)
        {
          double w_cmd = -max_w_cmd_ + 2.0 * max_w_cmd_ * j / (res - 1);
          if (j == res / 2)
          {
            w_cmd = 0.0;
          }
          um << v_cmd, w_cmd, 0.0;
          inputs.push_back(um);
        }
      }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // Check if in close set
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check body-frame surge, sway, and yaw rate separately.
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_u_ || fabs(pro_v(1)) > max_v_ || fabs(pro_v(2)) > max_r_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um(0) * um(0) + 3.0 * um(1) * um(1) + um(2) * um(2) + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              open_set_.erase(expand_node);
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
              open_set_.insert(expand_node);
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->is_shot_segment = false;
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.insert(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              if (shot_failures >= max_shot_failures && best_progress != nullptr)
              {
                retrievePath(best_progress);
                cout << "return partial search after node limit; goal distance: " << best_distance << endl;
                return NEAR_END;
              }
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // Remove before changing the ordering key, then reinsert.
              open_set_.erase(pro_node);
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
              open_set_.insert(pro_node);
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  if (shot_failures >= max_shot_failures && best_progress != nullptr)
  {
    retrievePath(best_progress);
    cout << "return partial search after open set exhausted; goal distance: " << best_distance << endl;
    return NEAR_END;
  }
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(rclcpp::Node& nh)
{
  max_tau_ = nh.declare_parameter<double>("search/max_tau", -1.0);
  init_max_tau_ = nh.declare_parameter<double>("search/init_max_tau", -1.0);
  rk4_steps_ = nh.declare_parameter<int>("search/rk4_steps", -1);
  max_vel_ = nh.declare_parameter<double>("search/max_vel", -1.0);
  max_acc_ = nh.declare_parameter<double>("search/max_acc", -1.0);
  max_u_ = nh.get_parameter("search/max_u").as_double();
  max_v_ = nh.get_parameter("search/max_v").as_double();
  max_r_ = nh.get_parameter("search/max_r").as_double();
  min_v_cmd_ = std::max(0.0, nh.declare_parameter<double>("search/min_v_cmd", 0.0));
  max_v_cmd_ = nh.declare_parameter<double>("search/max_v_cmd", -1.0);
  max_w_cmd_ = nh.declare_parameter<double>("search/max_w_cmd", -1.0);
  max_ground_speed_ = nh.declare_parameter<double>("search/max_ground_speed", -1.0);
  max_yaw_rate_ = nh.declare_parameter<double>("search/max_yaw_rate", -1.0);
  w_time_ = nh.declare_parameter<double>("search/w_time", -1.0);
  horizon_ = nh.declare_parameter<double>("search/horizon", -1.0);
  goal_pos_tolerance_ = nh.declare_parameter<double>("search/goal_pos_tolerance", -1.0);
  goal_yaw_tolerance_ = nh.declare_parameter<double>("search/goal_yaw_tolerance", -1.0);
  resolution_ = nh.declare_parameter<double>("search/resolution_astar", -1.0);
  yaw_bins_ = nh.declare_parameter<int>("search/yaw_bins", -1);
  time_resolution_ = nh.declare_parameter<double>("search/time_resolution", -1.0);
  lambda_heu_ = nh.declare_parameter<double>("search/lambda_heu", -1.0);
  allocate_num_ = nh.declare_parameter<int>("search/allocate_num", -1);
  check_num_ = nh.declare_parameter<int>("search/check_num", -1);
  optimistic_ = nh.declare_parameter<bool>("search/optimistic", true);
  shot_durations_ = nh.declare_parameter<std::vector<double>>("search/shot_durations", std::vector<double>{});
  shot_cruise_speed_ = nh.declare_parameter<double>("search/shot_cruise_speed", -1.0);
  shot_integration_step_ = nh.declare_parameter<double>("search/shot_integration_step", -1.0);
  shot_collision_step_ = nh.declare_parameter<double>("search/shot_collision_step", -1.0);
  shot_pos_tolerance_ = nh.declare_parameter<double>("search/shot_pos_tolerance", -1.0);
  shot_yaw_tolerance_ = nh.declare_parameter<double>("search/shot_yaw_tolerance", -1.0);
  shot_u_tolerance_ = nh.declare_parameter<double>("search/shot_u_tolerance", -1.0);
  shot_v_tolerance_ = nh.declare_parameter<double>("search/shot_v_tolerance", -1.0);
  shot_r_tolerance_ = nh.declare_parameter<double>("search/shot_r_tolerance", -1.0);
  shot_min_v_cmd_ = std::max(0.0, nh.declare_parameter<double>("search/shot_min_v_cmd", 0.0));
  shot_max_v_cmd_ = nh.declare_parameter<double>("search/shot_max_v_cmd", -1.0);
  shot_max_w_cmd_ = nh.declare_parameter<double>("search/shot_max_w_cmd", -1.0);
  shot_command_scale_ = nh.declare_parameter<double>("search/shot_command_scale", -1.0);
  shot_max_u_ = nh.declare_parameter<double>("search/shot_max_u", -1.0);
  shot_max_v_ = nh.declare_parameter<double>("search/shot_max_v", -1.0);
  shot_max_r_ = nh.declare_parameter<double>("search/shot_max_r", -1.0);
  shot_map_height_ = nh.declare_parameter<double>("search/shot_map_height", std::numeric_limits<double>::quiet_NaN());
  shot_fd_epsilon_ = nh.declare_parameter<double>("search/shot_fd_epsilon", -1.0);
  shot_damping_ = nh.declare_parameter<double>("search/shot_damping", -1.0);
  shot_time_limit_ = nh.declare_parameter<double>("search/shot_time_limit", -1.0);
  shot_max_iterations_ = nh.declare_parameter<int>("search/shot_max_iterations", -1);
  if (!std::isfinite(max_ground_speed_) || max_ground_speed_ <= 0.0 ||
      !std::isfinite(max_yaw_rate_) || max_yaw_rate_ <= 0.0 ||
      !std::isfinite(w_time_) || w_time_ < 0.0)
  {
    RCLCPP_WARN(nh.get_logger(),
                "USV heuristic disabled: configure positive search/max_ground_speed and "
                "search/max_yaw_rate, and nonnegative search/w_time.");
  }

  double vel_margin;
  vel_margin = nh.declare_parameter<double>("search/vel_margin", 0.0);
  max_vel_ += vel_margin;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  optimal_time = 0.0;
  // Without trusted bounds, use h=0 rather than inventing a physical limit.
  if (!std::isfinite(max_ground_speed_) || max_ground_speed_ <= 0.0 ||
      !std::isfinite(max_yaw_rate_) || max_yaw_rate_ <= 0.0 ||
      !std::isfinite(w_time_) || w_time_ < 0.0)
  {
    return 0.0;
  }

  const double distance = (x2.head(2) - x1.head(2)).norm();
  const double delta_psi = x2(2) - x1(2);
  // Shortest signed angular error in [-pi, pi], including unwrapped headings.
  const double heading_error = std::atan2(std::sin(delta_psi), std::cos(delta_psi));
  const double position_time = distance / max_ground_speed_;
  const double heading_time = std::abs(heading_error) / max_yaw_rate_;

  // This is a lower bound, not the old double-integrator optimal shot time.
  optimal_time = std::max(position_time, heading_time);
  return w_time_ * optimal_time;
}

// Integrate long shooting segments with bounded substeps. Each substep uses the
// same RK4 stateTransit() as the ordinary search, including its rk4_steps_.
void KinodynamicAstar::propagateShot(const Matrix<double, 6, 1>& start,
                                    const Vector3d& command, double duration,
                                    Matrix<double, 6, 1>& result)
{
  result = start;
  double elapsed = 0.0;
  while (elapsed < duration - 1e-12)
  {
    const double dt = std::min(shot_integration_step_, duration - elapsed);
    Matrix<double, 6, 1> next;
    stateTransit(result, next, command, dt);
    result = next;
    elapsed += dt;
  }
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  (void)time_to_goal;  // The connection uses explicit candidate durations instead.
  using State = Matrix<double, 6, 1>;
  using Controls = Matrix<double, 8, 1>;
  constexpr int pieces = 4;
  if (state1.size() != 6 || state2.size() != 6 || !state1.allFinite() || !state2.allFinite())
    return false;
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  if (shot_durations_.empty() || shot_max_iterations_ <= 0 ||
      !positive(shot_cruise_speed_) || !positive(shot_integration_step_) || !positive(shot_collision_step_) ||
      !positive(shot_pos_tolerance_) || !positive(shot_yaw_tolerance_) || !positive(shot_u_tolerance_) ||
      !positive(shot_v_tolerance_) || !positive(shot_r_tolerance_) || !positive(shot_max_w_cmd_) ||
      !positive(shot_max_u_) || !positive(shot_max_v_) || !positive(shot_max_r_) ||
      !positive(shot_fd_epsilon_) || !positive(shot_damping_) || !positive(shot_time_limit_) ||
      !positive(shot_command_scale_) || shot_command_scale_ > 1.0 ||
      !std::isfinite(shot_min_v_cmd_) || !std::isfinite(shot_max_v_cmd_) ||
      shot_min_v_cmd_ >= shot_max_v_cmd_ || !std::isfinite(shot_map_height_) ||
      std::any_of(shot_durations_.begin(), shot_durations_.end(), [&](double d) { return !positive(d); }))
  {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("rclcpp"), "Invalid or missing search/shot_* parameters; shooting disabled.");
    return false;
  }
  const State start = state1, goal = state2;
  const Vector3d start_map_point(start(0), start(1), shot_map_height_);
  if (std::abs(start(3)) > shot_max_u_ || std::abs(start(4)) > shot_max_v_ ||
      std::abs(start(5)) > shot_max_r_ ||
      edt_environment_->sdf_map_->getInflateOccupancy(start_map_point) != 0)
    return false;
  const auto wrap = [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); };
  // Select the nearest equivalent target heading relative to the shot start.
  // A wrapped residual alone also accepts endpoints one or more full turns away.
  const double goal_yaw = start(2) + wrap(goal(2) - start(2));
  const auto at_goal = [&](const State& s) {
    return (s.head<2>() - goal.head<2>()).norm() <= shot_pos_tolerance_ &&
           std::abs(s(2) - goal_yaw) <= shot_yaw_tolerance_ &&
           std::abs(s(3) - goal(3)) <= shot_u_tolerance_ &&
           std::abs(s(4) - goal(4)) <= shot_v_tolerance_ &&
           std::abs(s(5) - goal(5)) <= shot_r_tolerance_;
  };
  const auto residual = [&](const State& s) {
    State e = s - goal;
    e(2) = s(2) - goal_yaw;
    return e;
  };
  const double margin = (shot_max_v_cmd_ - shot_min_v_cmd_) * (1.0 - shot_command_scale_) / 2.0;
  const auto clamp_command = [&](Vector3d c) {
    c(0) = std::max(shot_min_v_cmd_, std::min(shot_max_v_cmd_ - margin, c(0)));
    c(1) = std::max(-shot_max_w_cmd_ * shot_command_scale_,
                    std::min(shot_max_w_cmd_ * shot_command_scale_, c(1)));
    c(2) = 0.0;
    return c;
  };
  const auto began = std::chrono::steady_clock::now();
  const auto expired = [&]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count() >= shot_time_limit_;
  };
  for (const double total : shot_durations_)
  {
    if (expired()) return false;
    const double h = total / pieces;
    Controls controls;
    State seed = start;
    for (int i = 0; i < pieces; ++i)
    {
      const double distance = (goal.head<2>() - seed.head<2>()).norm();
      const double bearing = std::atan2(goal(1) - seed(1), goal(0) - seed(0));
      const double error = wrap((i == pieces - 1 ? goal(2) : bearing) - seed(2));
      const double desired_u = i == pieces - 1 ? goal(3) :
          std::min(shot_cruise_speed_, distance * 0.6) * std::max(0.0, std::cos(error));
      const double target_r = std::max(-0.8 * shot_max_r_,
          std::min(0.8 * shot_max_r_, error / std::max(1.0, h)));
      const double response = std::max(1.0, h * 0.4);
      const double du = (desired_u - seed(3)) / response;
      const double dr = (target_r - seed(5)) / response;
      Vector3d command;
      command << (du + 0.433264281 * seed(3) - 0.032263988) / 0.400421432,
                 (dr - 0.166118168 * seed(4) + 0.544509453 * seed(5) - 0.000626529) / 0.837536662,
                 0.0;
      command = clamp_command(command);
      controls.segment<2>(2 * i) = command.head<2>();
      State next;
      propagateShot(seed, command, h, next);
      seed = next;
    }
    const auto endpoint = [&](const Controls& values) {
      State s = start;
      for (int i = 0; i < pieces; ++i)
      {
        Vector3d command(values(2*i), values(2*i+1), 0.0);
        State next;
        propagateShot(s, command, h, next);
        s = next;
      }
      return s;
    };
    for (int iteration = 0; iteration < shot_max_iterations_; ++iteration)
    {
      if (expired()) return false;
      const State end = endpoint(controls);
      if (!end.allFinite()) break;
      if (at_goal(end))
      {
        // Replay densely; never interpret heading as map height.
        State s = start;
        std::array<State, pieces> ends;
        // Permit ordinary course corrections but reject an additional full turn
        // beyond the shortest net heading change during this shot.
        const double max_total_turn = std::abs(goal_yaw - start(2)) + 2.0 * std::acos(-1.0);
        double total_turn = 0.0;
        bool safe = true;
        for (int i = 0; i < pieces && safe; ++i)
        {
          const Vector3d command(controls(2*i), controls(2*i+1), 0.0);
          const int n = static_cast<int>(std::ceil(h / shot_collision_step_));
          for (int j = 0; j < n; ++j)
          {
            if (expired()) return false;
            State next;
            propagateShot(s, command, h / n, next);
            total_turn += std::abs(next(2) - s(2));
            s = next;
            const Vector3d map_point(s(0), s(1), shot_map_height_);
            if (!s.allFinite() || total_turn >= max_total_turn || std::abs(s(3)) > shot_max_u_ ||
                std::abs(s(4)) > shot_max_v_ || std::abs(s(5)) > shot_max_r_ ||
                edt_environment_->sdf_map_->getInflateOccupancy(map_point) != 0)
            { safe = false; break; }
          }
          ends[i] = s;
        }
        if (safe && at_goal(s))
        {
          // Four normal parent-linked nodes let both existing output APIs replay the shot.
          PathNodePtr parent = *open_set_.begin();
          int search_segments = 0;
          for (PathNodePtr p = parent; p->parent != nullptr; p = p->parent) ++search_segments;
          RCLCPP_INFO(rclcpp::get_logger("path_searching"),
                      "Shot accepted: search_segments=%d shot_segments=4 duration=%.2f yaw_start=%.3f yaw_end=%.3f total_turn=%.3f",
                      search_segments, total, start(2), s(2), total_turn);
          for (int i = 0; i < pieces; ++i)
          {
            PathNodePtr node = path_node_pool_[use_node_num_++];
            node->state = ends[i];
            node->input = Vector3d(controls(2*i), controls(2*i+1), 0.0);
            node->duration = h;
            node->parent = parent;
            node->index = posToIndex(node->state.head<3>());
            node->is_shot_segment = true;
            node->g_score = parent->g_score + h * (node->input.squaredNorm() + w_time_);
            node->f_score = node->g_score;
            node->time = 0.0;  // Shot nodes are not inserted into the dynamic hash.
            node->node_state = NOT_EXPAND;
            parent = node;
          }
          retrievePath(parent);
          is_shot_succ_ = true;
          return true;
        }
        break;
      }
      const State e = residual(end);
      Matrix<double, 6, 8> jacobian;
      for (int j = 0; j < 8; ++j)
      {
        if (expired()) return false;
        Controls perturbed = controls;
        perturbed(j) += shot_fd_epsilon_;
        State delta = endpoint(perturbed) - end;
        delta(2) = wrap(delta(2));
        jacobian.col(j) = delta / shot_fd_epsilon_;
      }
      const Matrix<double, 6, 6> normal = jacobian * jacobian.transpose() +
          shot_damping_ * Matrix<double, 6, 6>::Identity();
      const Matrix<double, 8, 1> change = -jacobian.transpose() * normal.ldlt().solve(e);
      if (!change.allFinite()) break;
      bool improved = false;
      for (double alpha : {1.0, 0.5, 0.25})
      {
        if (expired()) return false;
        Controls candidate = controls + alpha * change;
        for (int i = 0; i < pieces; ++i)
        {
          const Vector3d c = clamp_command(Vector3d(candidate(2*i), candidate(2*i+1), 0.0));
          candidate.segment<2>(2*i) = c.head<2>();
        }
        const State test = endpoint(candidate);
        if (test.allFinite() && residual(test).squaredNorm() < e.squaredNorm())
        { controls = candidate; improved = true; break; }
      }
      if (!improved) break;
    }
  }
  return false;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  if (!std::isfinite(resolution_) || resolution_ <= 0.0 || yaw_bins_ <= 0)
    throw std::invalid_argument("search/resolution_astar and search/yaw_bins must be positive.");
  if (!std::isfinite(horizon_) || horizon_ <= 0.0 ||
      !std::isfinite(goal_pos_tolerance_) || goal_pos_tolerance_ < 0.0 ||
      !std::isfinite(goal_yaw_tolerance_) || goal_yaw_tolerance_ < 0.0)
    throw std::invalid_argument("search/horizon must be positive and goal tolerances must be nonnegative finite values.");

  const double two_pi = 2.0 * std::acos(-1.0);
  this->inv_resolution_ = 1.0 / resolution_;
  yaw_resolution_ = two_pi / static_cast<double>(yaw_bins_);
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  open_set_.clear();

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

// Acceleration of [x,y,psi] in the reference frame, from the same boat model.
Eigen::Vector3d KinodynamicAstar::poseAcceleration(const Matrix<double, 6, 1>& state,
                                                  const Vector3d& command) const
{
  const auto d = usvStateDerivative(state, command);
  const double c = std::cos(state(2)), s = std::sin(state(2));
  return Vector3d(c*d(3) - s*d(4) - state(5)*(s*state(3) + c*state(4)),
                  s*d(3) + c*d(4) + state(5)*(c*state(3) - s*state(4)), d(5));
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> result;
  if (path_nodes_.empty() || !std::isfinite(delta_t) || delta_t <= 0.0) return result;
  result.push_back(path_nodes_.front()->state.head<3>());
  for (size_t i = 1; i < path_nodes_.size(); ++i)
  {
    PathNodePtr node = path_nodes_[i];
    Matrix<double, 6, 1> start = node->parent->state, sample;
    Matrix<double, 6, 1> previous = start;
    double previous_t = 0.0;
    for (double t = delta_t; t < node->duration; t += delta_t)
    {
      if (node->is_shot_segment)
      {
        propagateShot(previous, node->input, t - previous_t, sample);
        previous = sample;
        previous_t = t;
      }
      else stateTransit(start, sample, node->input, t);
      result.push_back(sample.head<3>());
    }
    result.push_back(node->state.head<3>());
  }
  return result;
}

void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  if (path_nodes_.size() < 2 || !std::isfinite(ts) || ts <= 0.0)
    throw std::runtime_error("Cannot sample a zero-duration kinodynamic path.");
  double total = 0.0;
  for (size_t i = 1; i < path_nodes_.size(); ++i) total += path_nodes_[i]->duration;
  if (!std::isfinite(total) || total <= 0.0)
    throw std::runtime_error("Cannot sample a zero-duration kinodynamic path.");
  const int count = std::max(8, static_cast<int>(std::floor(total / ts)));
  ts = total / count;
  size_t segment = 1;
  double segment_start = 0.0;
  for (int i = 0; i <= count; ++i)
  {
    const double time = i * ts;
    while (segment + 1 < path_nodes_.size() &&
           time > segment_start + path_nodes_[segment]->duration)
    {
      segment_start += path_nodes_[segment]->duration;
      ++segment;
    }
    PathNodePtr node = path_nodes_[segment];
    const double local = std::max(0.0, std::min(node->duration, time - segment_start));
    Matrix<double, 6, 1> start = node->parent->state, sample;
    if (local >= node->duration - 1e-10)
    {
      sample = node->state;
    }
    else if (node->is_shot_segment)
    {
      propagateShot(start, node->input, local, sample);
    }
    else
    {
      stateTransit(start, sample, node->input, local);
    }
    point_set.push_back(sample.head<3>());
  }
  const auto* first = path_nodes_[1];
  const auto* last = path_nodes_.back();
  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(computeGlobalVelocity(last->state.head<3>(), last->state.tail<3>()));
  start_end_derivatives.push_back(poseAcceleration(first->parent->state, first->input));
  start_end_derivatives.push_back(poseAcceleration(last->state, last->input));
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  // x/y are map coordinates; phi is a periodic angle, not map height.
  const double two_pi = 2.0 * std::acos(-1.0);
  double phi = std::fmod(pt(2), two_pi);
  if (phi < 0.0)
    phi += two_pi;

  int yaw_index = static_cast<int>(std::floor(phi / yaw_resolution_));
  // Rounding near 2*pi can produce yaw_bins_; keep the result in range.
  if (yaw_index >= yaw_bins_)
    yaw_index = yaw_bins_ - 1;

  Vector3i idx;
  idx << static_cast<int>(std::floor((pt(0) - origin_(0)) * inv_resolution_)),
         static_cast<int>(std::floor((pt(1) - origin_(1)) * inv_resolution_)),
         yaw_index;

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

Eigen::Vector3d KinodynamicAstar::computeBodyVelocity(const Eigen::Vector3d& pose,
                                                    const Eigen::Vector3d& pose_vel) {
  const double cos_phi = std::cos(pose(2));
  const double sin_phi = std::sin(pose(2));
  Eigen::Vector3d body_vel;
  body_vel(0) = cos_phi * pose_vel(0) + sin_phi * pose_vel(1);
  body_vel(1) = -sin_phi * pose_vel(0) + cos_phi * pose_vel(1);
  body_vel(2) = pose_vel(2);
  return body_vel;
}

Eigen::Vector3d KinodynamicAstar::computeGlobalVelocity(const Eigen::Vector3d& pose,
                                                      const Eigen::Vector3d& body_vel) {
  const double cos_phi = std::cos(pose(2));
  const double sin_phi = std::sin(pose(2));
  Eigen::Vector3d global_vel;
  global_vel(0) = cos_phi * body_vel(0) - sin_phi * body_vel(1);
  global_vel(1) = sin_phi * body_vel(0) + cos_phi * body_vel(1);
  global_vel(2) = body_vel(2);  // Preserve yaw rate r; do not output heading phi.
  return global_vel;
}

void KinodynamicAstar::computeUsvStateAndCommand(const Eigen::Vector3d& pose,
                                               const Eigen::Vector3d& pose_vel,
                                               const Eigen::Vector3d& pose_acc,
                                               Eigen::Vector3d& body_vel,
                                               Eigen::Vector3d& command) {
  // pose=[x,y,phi], pose_vel=[dot_x,dot_y,r], pose_acc=[ddot_x,ddot_y,dot_r].
  // Translational derivatives are in the world frame.
  const double psi = pose(2);
  const double r = pose_vel(2);
  const double dot_r = pose_acc(2);
  const double cos_psi = std::cos(psi);
  const double sin_psi = std::sin(psi);
  body_vel = computeBodyVelocity(pose, pose_vel);
  const double u = body_vel(0);
  const double v = body_vel(1);
  const double dot_u = cos_psi * pose_acc(0) + sin_psi * pose_acc(1) + r * v;

  // Inverse of the surge/yaw equations in usvStateDerivative().
  // Sway consistency and command limits require separate validation.
  command(0) = (dot_u + 0.433264281 * u - 0.032263988) / 0.400421432;
  command(1) = (dot_r - 0.166118168 * v + 0.544509453 * r - 0.000626529)
               / 0.837536662;
  command(2) = 0.0;
}

Eigen::Matrix<double, 6, 1> KinodynamicAstar::usvStateDerivative(
    const Eigen::Matrix<double, 6, 1>& state, const Eigen::Vector3d& um) const
{
  const double psi = state(2);
  const double u = state(3);
  const double v = state(4);
  const double r = state(5);
  const double v_cmd = um(0);
  const double w_cmd = um(1);  // um(2) is reserved and is not read.

  Eigen::Matrix<double, 6, 1> derivative;
  derivative(0) = u * std::cos(psi) - v * std::sin(psi);
  derivative(1) = u * std::sin(psi) + v * std::cos(psi);
  derivative(2) = r;
  derivative(3) = -0.433264281 * u + 0.400421432 * v_cmd + 0.032263988;
  derivative(4) = -0.313324916 * v - 0.006741061 * r
                  - 0.407958500 * w_cmd - 0.000993619;
  derivative(5) = 0.166118168 * v - 0.544509453 * r
                  + 0.837536662 * w_cmd + 0.000626529;
  return derivative;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  if (rk4_steps_ <= 0)
    throw std::invalid_argument("search/rk4_steps must be a positive integer before stateTransit().");

  const double h = tau / static_cast<double>(rk4_steps_);
  Eigen::Matrix<double, 6, 1> state = state0;

  // Keep heading continuous; angle wrapping belongs to indexing/error handling.
  for (int i = 0; i < rk4_steps_; ++i)
  {
    const Eigen::Matrix<double, 6, 1> k1 = usvStateDerivative(state, um);
    const Eigen::Matrix<double, 6, 1> k2 = usvStateDerivative(state + 0.5 * h * k1, um);
    const Eigen::Matrix<double, 6, 1> k3 = usvStateDerivative(state + 0.5 * h * k2, um);
    const Eigen::Matrix<double, 6, 1> k4 = usvStateDerivative(state + h * k3, um);
    state += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

  state1 = state;
}

}  // namespace fast_planner
