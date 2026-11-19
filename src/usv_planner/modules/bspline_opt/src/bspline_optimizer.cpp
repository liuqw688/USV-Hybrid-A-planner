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



#include "bspline_opt/bspline_optimizer.h"
#include <path_searching/turning_constraint.hpp>
#include <nlopt.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>
// using namespace std;

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setConfig(const BsplineOptimizerConfig& config) {
  lambda1_ = config.lambda1;
  lambda2_ = config.lambda2;
  lambda3_ = config.lambda3;
  lambda4_ = config.lambda4;
  lambda5_ = config.lambda5;
  lambda6_ = config.lambda6;
  lambda7_ = config.lambda7;
  lambda8_ = config.lambda8;
  dist0_ = config.distance_margin;
  max_vel_ = config.max_velocity;
  max_acc_ = config.max_acceleration;
  max_global_vel_ = config.max_global_velocity;
  max_yaw_rate_ = config.max_yaw_rate;
  min_turn_radius_ = config.min_turn_radius;
  channel_weight_ = config.channel_weight;
  max_global_acc_ = config.max_global_acceleration;
  max_yaw_acc_ = config.max_yaw_acceleration;
  yaw_rate_weight_ = config.yaw_rate_weight;
  yaw_acc_weight_ = config.yaw_acceleration_weight;
  yaw_smoothness_weight_ = config.yaw_smoothness_weight;
  yaw_endpoint_weight_ = config.yaw_endpoint_weight;
  yaw_guide_weight_ = config.yaw_guide_weight;
  yaw_waypoint_weight_ = config.yaw_waypoint_weight;
  yaw_bound_ = config.yaw_bound;
  visib_min_ = config.visibility_min;
  dlmin_ = config.visibility_spacing;
  wnl_ = config.visibility_weight;
  std::copy(config.max_iterations.begin(), config.max_iterations.end(), max_iteration_num_);
  std::copy(config.max_iteration_time.begin(), config.max_iteration_time.end(), max_iteration_time_);
  algorithm1_ = config.quadratic_algorithm;
  algorithm2_ = config.general_algorithm;
  order_ = config.spline_order;
}

void BsplineOptimizer::setEnvironment(std::shared_ptr<SearchEnvironment> environment) {
  environment_ = std::move(environment);
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = std::max(0, dim_ * (pt_num - 2 * order_));
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  // Initialize best_variable_ with initial values in case optimization fails
  best_variable_ = q;

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    for (int i = 0; i < variable_num_; ++i) {
      double bound = 5.0;  // x、y 控制点相对初值的调整范围，单位 m。
      if (dim_ == 3 && i % dim_ == 2)
      {
        bound = yaw_bound_;  // 当前配置 ±15°，相对初值的航向调整范围，不是角速度。
      }
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    double        final_cost;
    (void)opt.optimize(q, final_cost);
  } catch (const std::exception&) {
    // Keep the best valid iterate if NLopt stops with an exception.
  }

  if (best_variable_.size() != variable_num_) {
    best_variable_ = q;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk(0) * jerk(0) + jerk(1) * jerk(1)
          + yaw_smoothness_weight_ * jerk(2) * jerk(2);  // 当前配置：平移权重 1.0，航向权重 4.0，实际以参数为准。
    temp_j = 2.0 * jerk;
    temp_j(2) *= yaw_smoothness_weight_;  // 航向损失加权后，其梯度也需乘相同权重，并非再次累加损失。
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d map_pos(q[i](0), q[i](1), 0.0);  // 暂用 z=0 地图切片，航向不是高度。
    environment_->evaluateDistanceWithGradient(map_pos, -1.0, dist, dist_grad);
    dist_grad(2) = 0.0;  // 障碍代价暂不考虑航向，仅保留 x、y 方向梯度。
    if (dist_grad.norm() > 1e-4)
    {
      dist_grad.normalize();
    }

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i](0) += 2.0 * (dist - dist0_) * dist_grad(0);
      gradient[i](1) += 2.0 * (dist - dist0_) * dist_grad(1);
      // 航向梯度保持为 0；船体轮廓及航向对避障的影响后续再处理。
    }
  }
}

void BsplineOptimizer::calcNavigationCost(const vector<Eigen::Vector3d>& q, double& cost,
    vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Eigen::Vector3d::Zero());
  const double dt = bspline_interval_;
  // 三点 Gauss 积分覆盖每段实际曲线，避免只检查不在曲线上的控制点。
  const double samples[] = {0.1127016654, 0.5, 0.8872983346};
  const double weights[] = {5.0 / 18.0, 4.0 / 9.0, 5.0 / 18.0};
  for (std::size_t i = 0; i + 3 < q.size(); ++i) {
    for (int sample = 0; sample < 3; ++sample) {
      const double t = samples[sample], t2 = t * t, t3 = t2 * t;
      const double b[] = {std::pow(1.0-t, 3)/6.0, (3*t3-6*t2+4)/6.0,
          (-3*t3+3*t2+3*t+1)/6.0, t3/6.0};
      const double d[] = {-0.5*(1-t)*(1-t)/dt, (1.5*t2-2*t)/dt,
          (-1.5*t2+t+0.5)/dt, 0.5*t2/dt};
      const double a[] = {(1-t)/(dt*dt), (3*t-2)/(dt*dt),
          (1-3*t)/(dt*dt), t/(dt*dt)};
      Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = p, acc = p;
      for (int j = 0; j < 4; ++j) {p += b[j]*q[i+j]; v += d[j]*q[i+j]; acc += a[j]*q[i+j];}
      double travel_cost = 0.0;
      Eigen::Vector3d map_gradient = Eigen::Vector3d::Zero(), gv, ga;
      if (environment_ && channel_weight_ > 0.0)
        environment_->evaluateTravelCostWithGradient({p.x(), p.y(), 0.0}, travel_cost, map_gradient);
      map_gradient.z() = 0.0;
      // 优化留出半径余量，发布前仍独立密集校验。
      const double turn = turningCost(v, acc, 1.2 * min_turn_radius_, gv, ga);
      const double weight = dt * weights[sample];
      cost += weight * (channel_weight_ * travel_cost / 20.0 + 100.0 * turn);
      for (int j = 0; j < 4; ++j)
        gradient[i+j] += weight * (channel_weight_ / 20.0 * b[j] * map_gradient +
            100.0 * (d[j] * gv + a[j] * ga));
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {
      double weight = 1.0;
      if (j == 2)
      {
        vm2 = max_yaw_rate_ * max_yaw_rate_;
        weight = yaw_rate_weight_;
      }
      else
      {
        vm2 = max_global_vel_ * max_global_vel_;
      }
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += weight * pow(vd, 2);  // 当前配置：平移权重 1.0，偏航角速度权重 1975.3，实际以参数为准。

        double temp_v = weight * 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double weight = 1.0;
      if (j == 2)
      {
        am2 = max_yaw_acc_ * max_yaw_acc_;
        weight = yaw_acc_weight_;
      }
      else
      {
        am2 = max_global_acc_ * max_global_acc_;
      }
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += weight * pow(ad, 2);  // 当前配置：平移权重 1.0，偏航角加速度权重 625.0，实际以参数为准。

        double temp_a = weight * 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  dq(2) = std::atan2(std::sin(dq(2)), std::cos(dq(2)));  // 避免额外转动。
  cost += dq(0) * dq(0) + dq(1) * dq(1)
        + yaw_endpoint_weight_ * dq(2) * dq(2);  // 权重 3.64756：1 m 位置与 30° 航向

  Eigen::Vector3d temp = 2.0 * dq;
  temp(2) *= yaw_endpoint_weight_;  
  gradient[q.size() - 3] += temp * (1 / 6.0);
  gradient[q.size() - 2] += temp * (4 / 6.0);
  gradient[q.size() - 1] += temp * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    // waypt=[x,y,phi]，第三项必须是期望航向，不是地图高度。
    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    dq(2) = std::atan2(std::sin(dq(2)), std::cos(dq(2)));  // 相差整圈视为相同朝向。
    cost += dq(0) * dq(0) + dq(1) * dq(1)
          + yaw_waypoint_weight_ * dq(2) * dq(2);  // 当前权重约 3.64756：1 m 位置与 30° 航向，实际以参数为准。

    Eigen::Vector3d temp = 2.0 * dq;
    temp(2) *= yaw_waypoint_weight_;  // 梯度与加权损失保持一致。
    gradient[idx] += temp * (1 / 6.0);
    gradient[idx + 1] += temp * (4 / 6.0);
    gradient[idx + 2] += temp * (1 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];  // gpt=[x,y,phi]，第三项必须是参考航向。
    Eigen::Vector3d dq = q[i] - gpt;
    dq(2) = std::atan2(std::sin(dq(2)), std::cos(dq(2)));  // 相差整圈视为相同朝向。
    cost += dq(0) * dq(0) + dq(1) * dq(1)
          + yaw_guide_weight_ * dq(2) * dq(2);  // 当前权重约 3.64756：1 m 位置与 30° 航向，实际以参数为准。

    Eigen::Vector3d temp = 2.0 * dq;
    temp(2) *= yaw_guide_weight_;  // 梯度与加权损失保持一致。
    gradient[i] += temp;
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
    calcNavigationCost(g_q_, f_distance, g_distance_);
    f_combine += f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (dim_ == 3 && ((cost_function_ & GUIDE) || (cost_function_ & WAYPOINTS)))
  {
    // 最短航向误差存在角度分界，不能再将这些三维代价视为全局二次函数。
    return false;
  }
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner
