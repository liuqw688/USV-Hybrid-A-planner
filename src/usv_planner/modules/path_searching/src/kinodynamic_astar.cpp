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
#include <path_searching/turning_constraint.hpp>
#include <path_searching/obstacle_speed_limit.hpp>
#include <path_searching/usv_dynamics.hpp>
#include <cmath>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <limits>
#include <Eigen/Cholesky>
#include <stdexcept>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (PathNodePtr node : path_node_pool_) delete node;
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_body_vel, Eigen::Vector3d start_cmd,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_body_vel, bool init, bool dynamic, double time_start,
                             std::chrono::steady_clock::time_point deadline)
{
  replay_end_ = nullptr;
  // 分项统计覆盖成功及失败搜索；其余耗时包含积分、碰撞检查和队列维护。
  struct SearchTiming {
    using Clock = std::chrono::steady_clock;
    Clock::time_point start = Clock::now();
    double field_ms = 0.0;
    double shot_ms = 0.0;
    int shots = 0;
    int closed_prunes = 0;
    int velocity_prunes = 0;
    int same_state_prunes = 0;
    int collision_prunes = 0;
    int duplicate_prunes = 0;
    int& nodes;
    int& iterations;
    ~SearchTiming() {
      const double total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
      std::fprintf(stderr,
          "[path_searching] Search profile: total=%.3f ms field=%.3f ms shot=%.3f ms other=%.3f ms shots=%d nodes=%d iterations=%d prunes[closed=%d velocity=%d same=%d collision=%d duplicate=%d]\n",
          total_ms, field_ms, shot_ms, total_ms - field_ms - shot_ms, shots, nodes, iterations,
          closed_prunes, velocity_prunes, same_state_prunes, collision_prunes, duplicate_prunes);
    }
  } timing{SearchTiming::Clock::now(), 0.0, 0.0, 0, 0, 0, 0, 0, 0, use_node_num_, iter_num_};
  search_deadline_ = std::min(deadline, timing.start +
    std::chrono::duration_cast<SearchTiming::Clock::duration>(std::chrono::duration<double>(search_time_limit_)));
  if (timing.start >= search_deadline_) return TIMEOUT;
  if (!environment_ || !environment_->isReady())
  {
    return NO_PATH;
  }
  if (environment_->usesGridMap())
  {
    // 初始化早于第一张地图；每次搜索刷新实际地图原点和范围。
    environment_->getRegion(origin_, map_size_3d_);
    if (environment_->getInflateOccupancy(start_pt) != 0 ||
        environment_->getInflateOccupancy(end_pt) != 0)
    {
      return NO_PATH;
    }
  }
  start_vel_ = computeGlobalVelocity(start_pt, start_body_vel);
  start_cmd_ = start_cmd;
  start_cmd_(0) = std::clamp(start_cmd_(0), min_v_cmd_, max_v_cmd_);
  start_cmd_(2) = 0.0;  // Reserved component is not a control input.

  const double heuristic_resolution = std::max(resolution_, environment_->getResolution());
  const Eigen::Vector2d lower = origin_.head<2>().cwiseMax(
      start_pt.head<2>().cwiseMin(end_pt.head<2>()) - Eigen::Vector2d::Constant(horizon_));
  const Eigen::Vector2d upper = (origin_ + map_size_3d_).head<2>().cwiseMin(
      start_pt.head<2>().cwiseMax(end_pt.head<2>()) + Eigen::Vector2d::Constant(horizon_));
  const Eigen::Vector2i dimensions = ((upper - lower) / heuristic_resolution).array().ceil().cast<int>();
  const auto field_start = SearchTiming::Clock::now();
  obstacle_heuristic_.build(lower, dimensions.x(), dimensions.y(), heuristic_resolution, end_pt.head<2>(),
      [this](const Eigen::Vector2d& xy) {
        const Eigen::Vector3d point(xy.x(), xy.y(), 0.0);
        return environment_->getInflateOccupancy(point) == 0 &&
               environment_->getDistance(point) >= search_clearance_;
      }, [this, heuristic_resolution](const Eigen::Vector2d& xy) {
        const double clearance = environment_->getDistance(
            Eigen::Vector3d(xy.x(), xy.y(), 0.0)) - search_clearance_;
        const double deficit = std::max(0.0, 1.0 - clearance / (2.0 * heuristic_resolution));
        const double map_cost = environment_->getTravelCost(
            Eigen::Vector3d(xy.x(), xy.y(), 0.0));
        return mapTravelCostMultiplier(map_cost) + 4.0 * deficit * deficit;
      });
  timing.field_ms = std::chrono::duration<double, std::milli>(SearchTiming::Clock::now() - field_start).count();

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->is_shot_segment = false;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_body_vel;
  cur_node->index = stateToIndex(cur_node->state);
  cur_node->g_score = 0.0;

  Eigen::Matrix<double, 6, 1> end_state;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_body_vel;
  current_goal_ = end_state;
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
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr seed = dynamic ? cur_node : seedPreviousPath(cur_node);
  const auto has_braking_room = [&](PathNodePtr node) {
    if (!std::isfinite(shot_u_tolerance_) || shot_u_tolerance_ <= 0.0) return false;
    if (std::abs(end_state(3)) > shot_u_tolerance_) return true;
    const auto braking = usv_dynamics::estimatePassiveSurgeBraking(node->state(3), shot_u_tolerance_);
    return (node->state.head<2>() - end_state.head<2>()).norm() >= braking.distance_m + 1.0;
  };
  const auto heading_matches_goal = [&](PathNodePtr node) {
    const double delta = node->state(2) - end_state(2);
    return std::abs(std::atan2(std::sin(delta), std::cos(delta))) < shot_yaw_tolerance_;
  };
  if (std::chrono::steady_clock::now() >= search_deadline_) {
    // 旧轨迹重放已经得到安全前缀时，交给滚动规划继续执行；不能把
    // 局部规划预算耗尽误报成没有可执行路径。
    while (seed != cur_node && (!has_braking_room(seed) || !heading_matches_goal(seed)))
      seed = seed->parent;
    if (seed != cur_node && seed->parent != nullptr) {
      retrievePath(seed);
      return REACH_HORIZON;
    }
    return TIMEOUT;
  }
  if (seed != cur_node) {
    if (terminalState(seed->state)) {
      retrievePath(seed);
      std::fprintf(stderr, "[path_searching] Warm start reached terminal state; no new shot required.\n");
      return REACH_END;
    }
    open_set_.insert(seed);
    if ((seed->index - cur_node->index).norm() != 0)
      expanded_nodes_.insert(seed->index, seed);
  }

  bool init_search = init && seed == cur_node;
  int shot_failures = 0;  // Reset for each search, including a later replan.
  Eigen::Matrix<double, 6, 1> last_shot_state = Eigen::Matrix<double, 6, 1>::Zero();
  PathNodePtr last_shot_node = nullptr;
  PathNodePtr best_progress = nullptr;
  double best_distance = (start_pt.head(2) - end_pt.head(2)).norm();
  double best_remaining_cost = std::numeric_limits<double>::infinity();
  constexpr int surge_samples = 5;
  constexpr int steering_samples = 7;
  vector<PathNodePtr> tmp_expand_nodes;
  tmp_expand_nodes.reserve(surge_samples * steering_samples);
  vector<Eigen::Vector3d> inputs;
  inputs.reserve(surge_samples * steering_samples);
  vector<double> durations;
  durations.reserve(20);
  vector<usv_dynamics::State> collision_samples;
  if (check_num_ > 0)
    collision_samples.reserve(static_cast<std::size_t>(check_num_));
  std::array<std::vector<usv_dynamics::State>, steering_samples> min_samples, max_samples;
  const auto try_final_shot = [&](PathNodePtr candidate) {
    if (candidate == nullptr || candidate == last_shot_node ||
        static_cast<std::size_t>(use_node_num_) + kShotPieces > path_node_pool_.size())
    {
      return false;
    }
    last_shot_node = candidate;
    const auto shot_start = SearchTiming::Clock::now();
    const bool succeeded = computeShotTraj(candidate, end_state, time_to_goal);
    timing.shot_ms += std::chrono::duration<double, std::milli>(
        SearchTiming::Clock::now() - shot_start).count();
    ++timing.shots;
    if (!succeeded)
      std::fprintf(stderr,
                  "[path_searching] Final shot fallback failed at goal distance %.2f m; no incomplete terminal path will be returned.\n",
                  (candidate->state.head<2>() - end_pt.head<2>()).norm());
    return succeeded;
  };

  while (!open_set_.empty())
  {
    if (SearchTiming::Clock::now() >= search_deadline_) {
      if (best_progress != nullptr && best_progress->parent != nullptr) {
        retrievePath(best_progress);
        std::fprintf(stderr,
                    "[path_searching] Search deadline reached; returning safe local prefix: nodes=%d iterations=%d.\n",
                    use_node_num_, iter_num_);
        return REACH_HORIZON;
      }
      std::fprintf(stderr,
        "[path_searching] Search deadline reached: nodes=%d iterations=%d; no incomplete trajectory returned.\n",
        use_node_num_, iter_num_);
      return TIMEOUT;
    }
    cur_node = open_set_.top();

    // Terminate?
    const double distance_from_start = (cur_node->state.head(2) - start_pt.head(2)).norm();
    const double distance_to_goal = (cur_node->state.head(2) - end_pt.head(2)).norm();
    const double delta_phi = end_pt(2) - cur_node->state(2);
    const double heading_error = std::atan2(std::sin(delta_phi), std::cos(delta_phi));

    // 局部前缀沿用搜索的绕障引导，避免只按直线距离选择障碍前的死端。
    double remaining_time;
    const double remaining_cost = estimateHeuristic(cur_node->state, end_state, remaining_time);
    // 最终目标需停车时，超时回退不能选取已耗尽被动制动距离的前缀。
    if (cur_node->parent != nullptr && has_braking_room(cur_node) &&
        heading_matches_goal(cur_node) && remaining_cost < best_remaining_cost)
    {
      best_progress = cur_node;
      best_distance = distance_to_goal;
      best_remaining_cost = remaining_cost;
    }

    bool reach_horizon = distance_from_start >= horizon_;
    const auto& state = cur_node->state;
    // The shot solves terminal heading and velocity together; do not wait for the search state to align first.
    const bool near_end = distance_to_goal <= shotTriggerDistance(state);

    // 达到完整终态容差的搜索节点无需再次求解连接。
    if (cur_node->parent != nullptr && terminalState(cur_node->state))
    {
      retrievePath(cur_node);
      std::cout << "reach end (search; terminal tolerances met)" << std::endl;
      return REACH_END;
    }

    // 状态显著变化后可重新连接，避免三次相近失败耗尽整次搜索的 shot 机会。
    const double yaw_change = std::atan2(std::sin(state(2) - last_shot_state(2)),
                                         std::cos(state(2) - last_shot_state(2)));
    const bool changed = shot_failures == 0 ||
        (state.head<2>() - last_shot_state.head<2>()).norm() >=
            std::max({2.0, 2.0 * resolution_, min_turn_radius_}) ||
        std::abs(yaw_change) >= std::max(2.0 * yaw_resolution_, std::acos(-1.0) / 12.0) ||
        (state.tail<3>() - last_shot_state.tail<3>()).cwiseAbs().cwiseQuotient(velocity_resolution_).maxCoeff() >= 5.0;
    if (near_end && timing.shot_ms < 500.0 * search_time_limit_ &&
        changed && static_cast<std::size_t>(use_node_num_) + kShotPieces <= path_node_pool_.size())
    {
      last_shot_state = state;
      last_shot_node = cur_node;
      estimateHeuristic(cur_node->state, end_state, time_to_goal);
      const auto shot_start = SearchTiming::Clock::now();
      const bool shot_succeeded = computeShotTraj(cur_node, end_state, time_to_goal);
      timing.shot_ms += std::chrono::duration<double, std::milli>(SearchTiming::Clock::now() - shot_start).count();
      ++timing.shots;
      if (shot_succeeded)
      {
        std::cout << "reach end (shot)" << std::endl;
        return REACH_END;
      }
      ++shot_failures;
      std::fprintf(stderr,
                  "[path_searching] Shot failed %d at distance %.2f m yaw_error=%.3f u=%.3f v=%.3f r=%.3f; continuing search\n",
                  shot_failures, distance_to_goal, heading_error, state(3), state(4), state(5));
    }
    if (reach_horizon && has_braking_room(cur_node) && heading_matches_goal(cur_node))
    {
      retrievePath(cur_node);
      std::cout << "reach horizon" << std::endl;
      return REACH_HORIZON;
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    const double time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    Eigen::Vector3d um;
    double pro_t;
    tmp_expand_nodes.clear();
    inputs.clear();
    durations.clear();
    collision_samples.clear();
    for (int i = 0; i < steering_samples; ++i)
    {
      min_samples[i].clear();
      max_samples[i].clear();
    }
    const bool reuse_integration = check_num_ > 0 && rk4_steps_ > 0 && rk4_steps_ % check_num_ == 0;
    const bool ordinary_expansion = !init_search;
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
      // 转向采用对称平方采样，保留极值并提高小转向分辨率。
      for (int i = 0; i < surge_samples; ++i)
      {
        double v_cmd = min_v_cmd_ + (max_v_cmd_ - min_v_cmd_) * i / (surge_samples - 1);
        for (int j = 0; j < steering_samples; ++j)
        {
          const double fraction = -1.0 + 2.0 * j / (steering_samples - 1);
          const double w_cmd = max_w_cmd_ * fraction * std::abs(fraction);
          um << v_cmd, w_cmd, 0.0;
          inputs.push_back(um);
        }
      }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        if (reuse_integration && ordinary_expansion)
        {
          // 固定转向时，船模和 RK4 对前向指令是仿射的，可由两个端点恢复中间采样。
          const int steering_index = i % steering_samples;
          const int surge_index = i / steering_samples;
          if (min_samples[steering_index].empty())
          {
            Eigen::Vector3d command = um;
            command(0) = min_v_cmd_;
            usv_dynamics::integrateRK4Samples(cur_state, command, tau, rk4_steps_, check_num_,
                                               min_samples[steering_index]);
            command(0) = max_v_cmd_;
            usv_dynamics::integrateRK4Samples(cur_state, command, tau, rk4_steps_, check_num_,
                                               max_samples[steering_index]);
          }
          const double fraction = static_cast<double>(surge_index) / (surge_samples - 1);
          collision_samples.resize(check_num_);
          for (int k = 0; k < check_num_; ++k)
            collision_samples[k] = min_samples[steering_index][k] + fraction *
                (max_samples[steering_index][k] - min_samples[steering_index][k]);
          pro_state = collision_samples.back();
        }
        else if (reuse_integration)
        {
          // 共用同一条 RK4 轨迹；检查时刻和末端总步数保持不变。
          usv_dynamics::integrateRK4Samples(cur_state, um, tau, rk4_steps_, check_num_, collision_samples);
          pro_state = collision_samples.back();
        }
        else
          stateTransit(cur_state, pro_state, um, tau);
        pro_t = dynamic ? cur_node->time + tau : 0.0;

        // Check if in close set
        SearchStateIndex pro_id = stateToIndex(pro_state);
        int pro_t_id = dynamic ? timeToIndex(pro_t) : 0;
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          ++timing.closed_prunes;
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check body-frame surge, sway, and yaw rate separately.
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_u_ || fabs(pro_v(1)) > max_v_ || fabs(pro_v(2)) > max_r_)
        {
          ++timing.velocity_prunes;
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // 保留同位姿但速度不同的节点，仅剪枝完整六状态同格的扩展。
        SearchStateIndex diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          ++timing.same_state_prunes;
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        double map_cost_sum = 0.0;
        double braking_cost_sum = 0.0;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          // 末端已在候选生成时积分，复用完全相同的状态。
          if (reuse_integration)
            xt = collision_samples[k - 1];
          else if (k == check_num_)
            xt = pro_state;
          else
            stateTransit(cur_state, xt, um, dt);
          pos << xt(0), xt(1), 0.0;
          map_cost_sum += environment_->getTravelCost(pos);
          const double obstacle_distance = environment_->getObstacleDistance(pos);
          const double speed = std::hypot(xt(3), xt(4));
          const double excess = obstacleBrakingExcess(obstacle_distance, speed,
              obstacle_slowdown_distance_, obstacle_max_speed_);
          braking_cost_sum += excess * excess;
          // 为后续三次样条拟合保留曲率余量，避免离散输入切换后产生局部过弯。
          if (turningRatios(usv_dynamics::toWorldVelocity(xt.head<3>(), xt.tail<3>()),
                  usv_dynamics::poseAcceleration(xt, um), 1.1 * min_turn_radius_).cwiseAbs().maxCoeff() > 1.0 ||
              !obstacleSpeedFeasible(obstacle_distance, speed, obstacle_slowdown_distance_,
                  obstacle_max_speed_) ||
              environment_->getInflateOccupancy(pos) != 0 ||
              environment_->evaluateCoarseDistance(pos, -1.0) < search_clearance_)
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          ++timing.collision_prunes;
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        const double steering = um(1) / max_w_cmd_;
        const double average_map_cost = map_cost_sum / std::max(1, check_num_);
        const double travel_multiplier = mapTravelCostMultiplier(average_map_cost);
        tmp_g_score = (um(0) * um(0) + 3.0 * steering * steering +
                       w_time_ * travel_multiplier + 1000.0 * braking_cost_sum /
                       std::max(1, check_num_)) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            ++timing.duplicate_prunes;
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
              expanded_nodes_.insert(pro_id, pro_node->time_idx, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              if (shot_failures > 0 && best_progress != nullptr)
              {
                if (try_final_shot(best_progress)) return REACH_END;
                cout << "terminal shot fallback failed after node limit; goal distance: " << best_distance << endl;
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

  if (shot_failures > 0 && best_progress != nullptr)
  {
    if (try_final_shot(best_progress)) return REACH_END;
    cout << "terminal shot fallback failed after open set exhaustion; goal distance: " << best_distance << endl;
    return NEAR_END;
  }
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setConfig(const KinodynamicAstarConfig& config)
{
  max_tau_ = config.max_tau;
  init_max_tau_ = config.init_max_tau;
  rk4_steps_ = config.rk4_steps;
  search_time_limit_ = config.search_time_limit;
  max_vel_ = config.max_velocity + config.velocity_margin;
  max_acc_ = config.max_acceleration;
  max_u_ = config.max_u;
  max_v_ = config.max_v;
  max_r_ = config.max_r;
  min_v_cmd_ = std::max(0.0, config.min_v_cmd);
  max_v_cmd_ = config.max_v_cmd;
  max_w_cmd_ = config.max_w_cmd;
  search_clearance_ = config.search_clearance;
  obstacle_slowdown_distance_ = config.obstacle_slowdown_distance;
  obstacle_max_speed_ = config.obstacle_max_speed;
  max_ground_speed_ = config.max_ground_speed;
  max_yaw_rate_ = config.max_yaw_rate;
  min_turn_radius_ = config.min_turn_radius;
  w_time_ = config.time_weight;
  horizon_ = config.horizon;
  shot_trigger_distance_ = config.shot_trigger_distance;
  resolution_ = config.position_resolution;
  yaw_bins_ = config.yaw_bins;
  velocity_resolution_ = config.velocity_resolution;
  time_resolution_ = config.time_resolution;
  lambda_heu_ = config.heuristic_weight;
  allocate_num_ = config.node_capacity;
  check_num_ = config.collision_check_count;
  optimistic_ = config.optimistic;
  shot_durations_ = config.shot_durations;
  shot_cruise_speed_ = config.shot_cruise_speed;
  shot_integration_step_ = config.shot_integration_step;
  shot_collision_step_ = config.shot_collision_step;
  shot_pos_tolerance_ = config.shot_position_tolerance;
  shot_yaw_tolerance_ = config.shot_yaw_tolerance;
  shot_u_tolerance_ = config.shot_u_tolerance;
  shot_v_tolerance_ = config.shot_v_tolerance;
  shot_r_tolerance_ = config.shot_r_tolerance;
  shot_min_v_cmd_ = std::max(0.0, config.shot_min_v_cmd);
  shot_max_v_cmd_ = config.shot_max_v_cmd;
  shot_max_w_cmd_ = config.shot_max_w_cmd;
  shot_command_scale_ = config.shot_command_scale;
  shot_max_u_ = config.shot_max_u;
  shot_max_v_ = config.shot_max_v;
  shot_max_r_ = config.shot_max_r;
  shot_map_height_ = config.shot_map_height;
  shot_fd_epsilon_ = config.shot_fd_epsilon;
  shot_damping_ = config.shot_damping;
  shot_time_limit_ = config.shot_time_limit;
  shot_max_iterations_ = config.shot_max_iterations;
}

bool KinodynamicAstar::terminalState(const Eigen::Matrix<double, 6, 1>& state) const
{
  const double yaw = state(2) - current_goal_(2);
  return (state.head<2>() - current_goal_.head<2>()).norm() <= shot_pos_tolerance_ &&
      std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) <= shot_yaw_tolerance_ &&
      std::abs(state(3) - current_goal_(3)) <= shot_u_tolerance_ &&
      std::abs(state(4) - current_goal_(4)) <= shot_v_tolerance_ &&
      std::abs(state(5) - current_goal_(5)) <= shot_r_tolerance_;
}

double KinodynamicAstar::shotTriggerDistance(const Eigen::Matrix<double, 6, 1>& state) const
{
  double stopping_distance = 0.0;
  double stopping_time = 0.0;
  if (std::abs(current_goal_(3)) <= shot_u_tolerance_ &&
      std::isfinite(shot_u_tolerance_) && shot_u_tolerance_ > 0.0) {
    const auto braking = usv_dynamics::estimatePassiveSurgeBraking(state(3), shot_u_tolerance_);
    stopping_distance = braking.distance_m;
    stopping_time = braking.time_s;
  }
  const double yaw = current_goal_(2) - state(2);
  const double heading_error = std::atan2(std::sin(yaw), std::cos(yaw));
  const double ground_speed = std::hypot(state(3), state(4));
  const double turn_time = std::isfinite(max_yaw_rate_) && max_yaw_rate_ > 0.0 ?
      std::abs(heading_error) / max_yaw_rate_ : 0.0;
  const double expansion_time = std::isfinite(max_tau_) && max_tau_ > 0.0 ? max_tau_ : 0.0;
  // 大航向差需要为转向及接近目标预留距离；该尺度用于触发连接，并非路径长度保证。
  return std::min(shot_trigger_distance_,
      std::max({shot_pos_tolerance_, ground_speed * stopping_time,
        stopping_distance + ground_speed * (turn_time + expansion_time),
        2.0 * min_turn_radius_ * std::abs(heading_error) + stopping_distance}));
}

void KinodynamicAstar::cacheSuccessfulPath()
{
  if (!path_nodes_.empty() && path_nodes_.back() == replay_end_) {
    // 完整复用只消耗旧序列的前缀，保持名义终态，避免反复以实测误差平移缓存。
    previous_path_.erase(previous_path_.begin(), previous_path_.begin() + replay_begin_segment_);
    if (!previous_path_.empty()) {
      auto& first = previous_path_.front();
      first.start = usv_dynamics::integrateRK4(first.start, first.command, replay_begin_time_,
          std::max(1, static_cast<int>(std::ceil(replay_begin_time_ / shot_collision_step_))));
      first.duration -= replay_begin_time_;
      if (first.duration <= 1e-8) previous_path_.erase(previous_path_.begin());
    }
    return;
  }
  previous_path_.clear();
  previous_goal_ = current_goal_;
  for (size_t i = 1; i < path_nodes_.size(); ++i) {
    const auto* node = path_nodes_[i];
    previous_path_.push_back({node->parent->state, node->input, node->duration, node->is_shot_segment});
  }
}

PathNodePtr KinodynamicAstar::seedPreviousPath(PathNodePtr root)
{
  using State = usv_dynamics::State;
  if (previous_path_.empty()) return root;
  const double goal_yaw = previous_goal_(2) - current_goal_(2);
  if ((previous_goal_.head<2>() - current_goal_.head<2>()).norm() > 1e-6 ||
      std::abs(std::atan2(std::sin(goal_yaw), std::cos(goal_yaw))) > 1e-6 ||
      (previous_goal_.tail<3>() - current_goal_.tail<3>()).norm() > 1e-6) {
    previous_path_.clear();
    return root;
  }

  // 只匹配旧路径起始五秒；位置与航向联合匹配，防止掉头交叠处跳到末段。
  size_t best_segment = 0;
  double best_time = 0.0, best_score = std::numeric_limits<double>::infinity();
  State matched = previous_path_.front().start;
  double elapsed = 0.0;
  for (size_t i = 0; i < previous_path_.size() && elapsed < 5.0; ++i) {
    const auto& segment = previous_path_[i];
    const double window = std::min(segment.duration, 5.0 - elapsed);
    const int count = std::max(1, static_cast<int>(std::ceil(window / 0.1)));
    for (int j = 0; j <= count; ++j) {
      const double t = window * j / count;
      const State state = usv_dynamics::integrateRK4(segment.start, segment.command, t,
          std::max(1, static_cast<int>(std::ceil(t / 0.1))));
      const double yaw = state(2) - root->state(2);
      const double heading = std::atan2(std::sin(yaw), std::cos(yaw));
      const double score = (state.head<2>() - root->state.head<2>()).squaredNorm() +
          4.0 * heading * heading + 0.25 * (state.tail<3>() - root->state.tail<3>()).squaredNorm();
      if (score < best_score) {
        best_score = score;
        best_segment = i;
        best_time = t;
        matched = state;
      }
    }
    elapsed += segment.duration;
  }
  const double yaw = matched(2) - root->state(2);
  if ((matched.head<2>() - root->state.head<2>()).norm() > 2.0 ||
      std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) > std::acos(-1.0) / 6.0)
    return root;

  PathNodePtr parent = root;
  int segments = 0;
  bool blocked = false;
  const auto& last = previous_path_.back();
  const State cached_end = usv_dynamics::integrateRK4(last.start, last.command, last.duration,
      std::max(1, static_cast<int>(std::ceil(last.duration / shot_collision_step_))));
  const bool complete_terminal = terminalState(cached_end);
  bool entered_terminal_zone = false;
  for (size_t i = best_segment; i < previous_path_.size() && !blocked; ++i) {
    const auto& segment = previous_path_[i];
    double remaining = segment.duration - (i == best_segment ? best_time : 0.0);
    while (remaining > 1e-8 && use_node_num_ + kShotPieces < allocate_num_) {
      if (std::chrono::steady_clock::now() >= search_deadline_) return parent;
      // 未完成任务的旧前缀到达制动区后交回搜索，保留正常的终端连接时机。
      if (!complete_terminal &&
          (parent->state.head<2>() - current_goal_.head<2>()).norm() <=
              shotTriggerDistance(parent->state)) {
        entered_terminal_zone = true;
        break;
      }
      const double duration = std::min(max_tau_, remaining);
      const int count = std::max(1, static_cast<int>(std::ceil(duration / shot_collision_step_)));
      State state = parent->state;
      double map_cost_sum = 0.0;
      for (int j = 0; j < count; ++j) {
        state = usv_dynamics::integrateRK4(state, segment.command, duration / count, 1);
        const Eigen::Vector3d point(state(0), state(1), 0.0);
        map_cost_sum += environment_->getTravelCost(point);
        if (!state.allFinite() || std::abs(state(3)) > max_u_ || std::abs(state(4)) > max_v_ ||
            std::abs(state(5)) > max_r_ ||
            turningRatios(usv_dynamics::toWorldVelocity(state.head<3>(), state.tail<3>()),
                usv_dynamics::poseAcceleration(state, segment.command), min_turn_radius_).
                cwiseAbs().maxCoeff() > 1.0 ||
            environment_->getInflateOccupancy(point) != 0 ||
            environment_->getDistance(point) < search_clearance_ ||
            !obstacleSpeedFeasible(environment_->getObstacleDistance(point),
                std::hypot(state(3), state(4)), obstacle_slowdown_distance_, obstacle_max_speed_)) {
          blocked = true;
          break;
        }
      }
      if (blocked) break;
      PathNodePtr node = path_node_pool_[use_node_num_++];
      node->state = state;
      node->input = segment.command;
      node->duration = duration;
      node->parent = parent;
      node->index = stateToIndex(state);
      node->is_shot_segment = segment.shot;
      const double steering = segment.command(1) / max_w_cmd_;
      const double average_map_cost = map_cost_sum / count;
      node->g_score = parent->g_score + duration *
          (segment.command(0) * segment.command(0) + 3.0 * steering * steering +
           w_time_ * mapTravelCostMultiplier(average_map_cost));
      double time_to_goal;
      node->f_score = node->g_score + lambda_heu_ * estimateHeuristic(state, current_goal_, time_to_goal);
      node->node_state = NOT_EXPAND;
      node->time = 0.0;
      node->time_idx = 0;
      parent = node;
      remaining -= duration;
      ++segments;
      // 原始 shot 可越过普通搜索半径完成掉头，重放保持相同范围语义。
      if (!segment.shot && (state.head<2>() - root->state.head<2>()).norm() >= horizon_) {
        blocked = true;
        break;
      }
    }
    if (entered_terminal_zone) break;
  }
  if (parent != root) parent->node_state = IN_OPEN_SET;
  if (!blocked && !entered_terminal_zone && parent != root && use_node_num_ + kShotPieces < allocate_num_) {
    replay_end_ = parent;
    replay_begin_segment_ = best_segment;
    replay_begin_time_ = best_time;
  }
  std::fprintf(stderr,
      "[path_searching] Warm start: segments=%d blocked=%d match_error=%.3f m\n", segments, blocked,
      (matched.head<2>() - root->state.head<2>()).norm());
  return parent;
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
double KinodynamicAstar::estimateHeuristic(const Eigen::Matrix<double, 6, 1>& x1,
                                           const Eigen::Matrix<double, 6, 1>& x2,
                                           double& optimal_time)
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
  const double position_time = (distance + obstacle_heuristic_.detour(x1.head<2>())) / max_ground_speed_;
  const Eigen::Vector2d direction = obstacle_heuristic_.descentDirection(x1.head<2>(), x2.head<2>());
  const double bearing = distance > resolution_ && direction.norm() > 1e-9 ?
      usv_dynamics::headingFromForwardVector(direction.x(), direction.y()) : x2(2);
  const auto angle_distance = [](double angle) {
    return std::abs(std::atan2(std::sin(angle), std::cos(angle)));
  };
  // 几何半径约束可能使可用转向率低于角速度上限；使用全局速度上限
  // 估计引导方向的转向时间，避免因使用当前速度而奖励不必要的加速。
  const double usable_yaw_rate = min_turn_radius_ > 0.0 ?
      std::min(max_yaw_rate_, max_ground_speed_ / (1.1 * min_turn_radius_)) : max_yaw_rate_;
  // 大航向偏差时加强转向引导，使背向目标的船有机会先驶离目标再转回。
  const double bearing_error = angle_distance(bearing - x1(2));
  const double heading_time = (1.0 + 3.0 * bearing_error / std::acos(-1.0)) * bearing_error / usable_yaw_rate;
  // 计入朝目标转向的时间，防止已绕过障碍的节点继续向外偏航。
  // 转向和前进实际可同时发生，此项仅作加权引导，不是可采纳下界。
  // 制动由终端连接处理；在启发值中取制动时间最大值会产生与位置无关的平台。
  optimal_time = std::max({position_time + heading_time,
                           std::abs(heading_error) / max_yaw_rate_});
  return w_time_ * optimal_time;
}

// shot_integration_step_ 是 RK4 实际步长上限，不再重复细分普通搜索步数。
void KinodynamicAstar::propagateShot(const Matrix<double, 6, 1>& start,
                                    const Vector3d& command, double duration,
                                    Matrix<double, 6, 1>& result)
{
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / shot_integration_step_)));
  result = usv_dynamics::integrateRK4(start, command, duration, steps);
}

bool KinodynamicAstar::computeShotTraj(PathNodePtr start_node,
                                      const Eigen::Matrix<double, 6, 1>& goal_state,
                                      double time_to_goal)
{
  (void)time_to_goal;  // The connection uses explicit candidate durations instead.
  using State = Matrix<double, 6, 1>;
  // 维持无半径约束场景的四段兼容性；启用最小半径时增加为六段，给掉头留下
  // 足够的转向自由度。矩阵按最大段数分配，未使用列保持为零并由阻尼项稳定。
  const int pieces = min_turn_radius_ > 0.0 ? kShotPieces : 4;
  constexpr int control_size = 2 * kShotPieces;
  using Controls = Matrix<double, control_size, 1>;
  constexpr int samples_per_piece = 4;
  constexpr int residual_size = 6 + kShotPieces * samples_per_piece * 4;
  using Residual = Matrix<double, residual_size, 1>;
  if (start_node == nullptr || !goal_state.allFinite() ||
      static_cast<std::size_t>(use_node_num_) + pieces > path_node_pool_.size())
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
    static bool warned = false;
    if (!warned) {
      std::fprintf(stderr, "[path_searching] Invalid search/shot configuration; shooting disabled.\n");
      warned = true;
    }
    return false;
  }
  const State start = start_node->state, goal = goal_state;
  if (!start.allFinite()) return false;
  const Vector3d start_map_point(start(0), start(1), shot_map_height_);
  if (std::abs(start(3)) > shot_max_u_ || std::abs(start(4)) > shot_max_v_ ||
      std::abs(start(5)) > shot_max_r_ ||
      environment_->getInflateOccupancy(start_map_point) != 0)
    return false;
  const auto wrap = [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); };
  // 沿接近目标的方向展开航向，避免强制最短首尾角差产生反向绕圈。
  const Eigen::Vector2d displacement = goal.head<2>() - start.head<2>();
  const double approach_yaw = displacement.norm() > shot_pos_tolerance_ ?
      start(2) + wrap(usv_dynamics::headingFromForwardVector(
          displacement.x(), displacement.y()) - start(2)) : start(2);
  const double goal_yaw = approach_yaw + wrap(goal(2) - approach_yaw);
  const auto at_goal = [&](const State& s) {
    return (s.head<2>() - goal.head<2>()).norm() <= shot_pos_tolerance_ &&
           std::abs(s(2) - goal_yaw) <= shot_yaw_tolerance_ &&
           std::abs(s(3) - goal(3)) <= shot_u_tolerance_ &&
           std::abs(s(4) - goal(4)) <= shot_v_tolerance_ &&
           std::abs(s(5) - goal(5)) <= shot_r_tolerance_;
  };
  const auto terminal_residual = [&](const State& s) {
    State e = s - goal;
    e(2) = s(2) - goal_yaw;
    // 按终态容差无量纲化，避免米制位置误差掩盖速度和角速度误差。
    e(0) /= shot_pos_tolerance_;
    e(1) /= shot_pos_tolerance_;
    e(2) /= shot_yaw_tolerance_;
    e(3) /= shot_u_tolerance_;
    e(4) /= shot_v_tolerance_;
    e(5) /= shot_r_tolerance_;
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
    const auto now = std::chrono::steady_clock::now();
    return now >= search_deadline_ || std::chrono::duration<double>(now - began).count() >= shot_time_limit_;
  };
  // 按估计平均速度排序，仍保留全部配置的时长作为后备。
  std::vector<double> durations = shot_durations_;
  const double estimated_time = std::max(
      (goal.head<2>() - start.head<2>()).norm() / (0.75 * shot_cruise_speed_),
      min_turn_radius_ * std::abs(goal_yaw - start(2)) / shot_cruise_speed_ + 12.0);
  std::stable_sort(durations.begin(), durations.end(), [estimated_time](double a, double b) {
    return std::abs(a - estimated_time) < std::abs(b - estimated_time);
  });
  for (const double total : durations)
  {
    if (expired()) return false;
    const double h = total / pieces;
    Controls controls;
    State seed = start;
    const bool stop_goal = std::abs(goal(3)) <= shot_u_tolerance_;
    const auto braking = usv_dynamics::estimatePassiveSurgeBraking(
        std::max(std::abs(start(3)), shot_cruise_speed_), shot_u_tolerance_);
    // 航向变化越大，越需先保留机动速度；终端求解仍同时校验所有动力学约束。
    const double brake_seed_weight = stop_goal ?
        std::clamp(1.0 - std::abs(goal_yaw - start(2)) / std::acos(-1.0), 0.0, 1.0) : 0.0;
    const int braking_pieces = std::min(pieces - 1,
        std::max(1, static_cast<int>(std::ceil(braking.time_s / h))));
    const int settle_pieces = 1 + static_cast<int>(std::round(
        brake_seed_weight * (braking_pieces - 1)));
    for (int i = 0; i < pieces; ++i)
    {
      const double distance = (goal.head<2>() - seed.head<2>()).norm();
      const double bearing = usv_dynamics::headingFromForwardVector(
          goal(0) - seed(0), goal(1) - seed(1));
      // 按模型的被动制动时间预留末段，并以剩余航程初始化前向指令。
      const bool settling = i >= pieces - settle_pieces;
      const double error = wrap((i >= pieces - settle_pieces - 1 ? goal_yaw : bearing) - seed(2));
      const double desired_u = settling ? goal(3) :
          std::min(shot_cruise_speed_, distance * 0.6) *
          (min_turn_radius_ > 0.0 ? 1.0 : std::max(0.0, std::cos(error)));
      const double turn_rate = min_turn_radius_ > 0.0 ?
          std::min(0.8 * shot_max_r_, 0.75 * std::max(kTurningSpeedFloor, desired_u) / min_turn_radius_) :
          0.8 * shot_max_r_;
      const double target_r = settling ? goal(5) : std::clamp(error / std::max(1.0, h), -turn_rate, turn_rate);
      const double response = std::max(1.0, h * 0.4);
      const double du = (desired_u - seed(3)) / response;
      const double dr = (target_r - seed(5)) / response;
      Vector3d command = usv_dynamics::commandFromBodyAccelerations(
          seed(3), seed(4), seed(5), du, dr);
      if (brake_seed_weight > 0.0) {
        const double drive_time = std::max(h, (pieces - settle_pieces - i) * h);
        const double damping = -usv_dynamics::kModelParameters.au;
        const double gain = usv_dynamics::kModelParameters.bu;
        const double desired_command = (distance * damping - std::max(0.0, seed(3))) /
            (gain * drive_time);
        const double braking_command = settling ? 0.0 :
            std::min(command(0), std::max(0.0, desired_command));
        command(0) += brake_seed_weight * (braking_command - command(0));
      }
      command = clamp_command(command);
      controls.segment<2>(2 * i) = command.head<2>();
      State next;
      propagateShot(seed, command, h, next);
      seed = next;
    }
    // 缓存各段起点，有限差分仅重算发生变化的段和后续段。
    const auto residual = [&](const Controls& values, int first_piece, State s,
        std::array<State, kShotPieces + 1>* prefix, Residual e) {
      if (prefix != nullptr) (*prefix)[first_piece] = s;
      for (int i = first_piece; i < pieces; ++i)
      {
        Vector3d command(values(2*i), values(2*i+1), 0.0);
        for (int sample = 0; sample < samples_per_piece; ++sample) {
          State next;
          propagateShot(s, command, h / samples_per_piece, next);
          s = next;
          const int row = 6 + 4 * (i * samples_per_piece + sample);
          const auto ratios = turningRatios(usv_dynamics::toWorldVelocity(s.head<3>(), s.tail<3>()),
              usv_dynamics::poseAcceleration(s, command), min_turn_radius_);
          e(row) = 10.0 * std::max(0.0, std::abs(ratios(0)) - 0.95);
          e(row + 1) = 10.0 * std::max(0.0, std::abs(ratios(1)) - 0.95);
          const double map_cost = environment_->getTravelCost({s(0), s(1), shot_map_height_});
          e(row + 2) = 0.5 * std::sqrt(std::max(0.0, map_cost) / 20.0);
          const double obstacle_distance = environment_->getObstacleDistance(
              {s(0), s(1), shot_map_height_});
          e(row + 3) = 5.0 * obstacleBrakingExcess(obstacle_distance,
              std::hypot(s(3), s(4)), obstacle_slowdown_distance_, obstacle_max_speed_);
        }
        if (prefix != nullptr) (*prefix)[i + 1] = s;
      }
      e.head<6>() = terminal_residual(s);
      return e;
    };
    for (int iteration = 0; iteration < shot_max_iterations_; ++iteration)
    {
      if (expired()) return false;
      std::array<State, kShotPieces + 1> prefix;
      const Residual e = residual(controls, 0, start, &prefix, Residual::Zero());
      const State end = prefix[pieces];
      if (!e.allFinite() || !end.allFinite()) break;
      // 航道偏好是软代价；不能因为无法继续降低软代价而丢弃已可行的连接。
      if (at_goal(end))
      {
        // Replay densely; never interpret heading as map height.
        State s = start;
        std::array<State, kShotPieces> ends;
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
                turningRatios(usv_dynamics::toWorldVelocity(s.head<3>(), s.tail<3>()),
                    usv_dynamics::poseAcceleration(s, command), min_turn_radius_).
                    cwiseAbs().maxCoeff() > 1.0 ||
                environment_->getInflateOccupancy(map_point) != 0 ||
                environment_->getDistance(map_point) < search_clearance_ ||
                !obstacleSpeedFeasible(environment_->getObstacleDistance(map_point),
                    std::hypot(s(3), s(4)), obstacle_slowdown_distance_, obstacle_max_speed_))
            { safe = false; break; }
          }
          ends[i] = s;
        }
        if (safe && at_goal(s))
        {
          // 使用普通父子节点存储各段，统一搜索路径与终端连接的回放方式。
          PathNodePtr parent = start_node;
          int search_segments = 0;
          for (PathNodePtr p = parent; p->parent != nullptr; p = p->parent) ++search_segments;
          std::fprintf(stderr,
                      "[path_searching] Shot accepted: search_segments=%d shot_segments=%d duration=%.2f yaw_start=%.3f yaw_end=%.3f total_turn=%.3f\n",
                      search_segments, pieces, total, start(2), s(2), total_turn);
          for (int i = 0; i < pieces; ++i)
          {
            PathNodePtr node = path_node_pool_[use_node_num_++];
            node->state = ends[i];
            node->input = Vector3d(controls(2*i), controls(2*i+1), 0.0);
            node->duration = h;
            node->parent = parent;
            node->index = stateToIndex(node->state);
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
      }
      Matrix<double, residual_size, control_size> jacobian =
          Matrix<double, residual_size, control_size>::Zero();
      for (int j = 0; j < 2 * pieces; ++j)
      {
        if (expired()) return false;
        Controls perturbed = controls;
        perturbed(j) += shot_fd_epsilon_;
        const int first_piece = j / 2;
        const Residual delta = residual(perturbed, first_piece, prefix[first_piece], nullptr, e) - e;
        jacobian.col(j) = delta / shot_fd_epsilon_;
      }
      const Matrix<double, control_size, control_size> normal = jacobian.transpose() * jacobian +
          shot_damping_ * Matrix<double, control_size, control_size>::Identity();
      const Controls change = -normal.ldlt().solve(jacobian.transpose() * e);
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
        const Residual test = residual(candidate, 0, start, nullptr, Residual::Zero());
        if (test.allFinite() && test.squaredNorm() < e.squaredNorm())
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
  if (!std::isfinite(search_time_limit_) || search_time_limit_ <= 0.0)
    throw std::invalid_argument("search/time_limit must be positive and finite.");
  if (!std::isfinite(horizon_) || horizon_ <= 0.0 ||
      !std::isfinite(shot_trigger_distance_) || shot_trigger_distance_ < 0.0)
    throw std::invalid_argument("search/horizon must be positive and search/shot_trigger_distance must be nonnegative and finite.");

  if (!std::isfinite(max_w_cmd_) || max_w_cmd_ <= 0.0)
    throw std::invalid_argument("search/max_w_cmd must be positive and finite.");
  if (!velocity_resolution_.allFinite() || (velocity_resolution_.array() <= 0.0).any())
    throw std::invalid_argument("search/u_resolution, v_resolution and r_resolution must be positive finite values.");

  const double two_pi = 2.0 * std::acos(-1.0);
  this->inv_resolution_ = 1.0 / resolution_;
  yaw_resolution_ = two_pi / static_cast<double>(yaw_bins_);
  inv_time_resolution_ = 1.0 / time_resolution_;
  environment_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  expanded_nodes_.reserve(static_cast<std::size_t>(allocate_num_));
  open_set_.reserve(static_cast<std::size_t>(allocate_num_));
  path_node_pool_.resize(allocate_num_ + kShotPieces);
  for (std::size_t i = 0; i < path_node_pool_.size(); i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(std::shared_ptr<SearchEnvironment> environment)
{
  environment_ = std::move(environment);
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
  start_end_derivatives.push_back(
      usv_dynamics::poseAcceleration(first->parent->state, first->input));
  start_end_derivatives.push_back(
      usv_dynamics::poseAcceleration(last->state, last->input));
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

SearchStateIndex KinodynamicAstar::stateToIndex(const Eigen::Matrix<double, 6, 1>& state) const
{
  return discretizeSearchState(state, origin_, resolution_, yaw_bins_, velocity_resolution_);
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

Eigen::Vector3d KinodynamicAstar::computeBodyVelocity(const Eigen::Vector3d& pose,
                                                    const Eigen::Vector3d& pose_vel) {
  return usv_dynamics::toBodyVelocity(pose, pose_vel);
}

Eigen::Vector3d KinodynamicAstar::computeGlobalVelocity(const Eigen::Vector3d& pose,
                                                      const Eigen::Vector3d& body_vel) {
  return usv_dynamics::toWorldVelocity(pose, body_vel);
}

void KinodynamicAstar::computeUsvStateAndCommand(const Eigen::Vector3d& pose,
                                               const Eigen::Vector3d& pose_vel,
                                               const Eigen::Vector3d& pose_acc,
                                               Eigen::Vector3d& body_vel,
                                               Eigen::Vector3d& command) {
  usv_dynamics::stateAndCommandFromPoseDerivatives(
      pose, pose_vel, pose_acc, body_vel, command);
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  state1 = usv_dynamics::integrateRK4(state0, um, tau, rk4_steps_);
}

}  // namespace fast_planner
