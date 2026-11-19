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



#include "bspline/non_uniform_bspline.h"
#include <cmath>

namespace fast_planner {

NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                          const double& interval) {
  control_points_ = points;
  p_              = order;
  interval_       = interval;

  n_ = points.rows() - 1;
  m_ = n_ + p_ + 1;

  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) {

    if (i <= p_) {
      u_(i) = double(-p_ + i) * interval_;
    } else if (i > p_ && i <= m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    } else if (i > m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  if (u_.size() == 0 || p_ < 0 || m_ < p_) {
    std::cerr << "[B-spline ERROR]: Invalid spline state in getTimeSpan! u_.size=" << u_.size() 
              << ", p_=" << p_ << ", m_=" << m_ << std::endl;
    um = 0.0;
    um_p = 0.0;
    return;
  }
  um   = u_(p_);
  um_p = u_(m_ - p_);
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {

  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // determine which [ui,ui+1] lay in
  int k = p_;
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  /* deBoor's alg */
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
  }

  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) {
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  return evaluateDeBoor(t + u_(p_));
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p_-1
  // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) =
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  /* cut the first and last knot */
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double NonUniformBspline::getInterval() { return interval_; }

void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  limit_vel_ = Eigen::VectorXd::Constant(control_points_.cols(), vel);
  limit_acc_ = Eigen::VectorXd::Constant(control_points_.cols(), acc);
  limit_ratio_ = 1.1;
}

void NonUniformBspline::setPhysicalLimits(const Eigen::Vector3d& vel,
                                         const Eigen::Vector3d& acc) {
  limit_vel_ = vel;
  limit_acc_ = acc;
  limit_ratio_ = 1.1;
}

bool NonUniformBspline::checkFeasibility(bool show) {
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_(0) + 1e-4 || fabs(vel(1)) > limit_vel_(1) + 1e-4 ||
        fabs(vel(2)) > limit_vel_(2) + 1e-4) {

      if (show) cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)) / limit_vel_(j));
      }
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_(0) + 1e-4 || fabs(acc(1)) > limit_acc_(1) + 1e-4 ||
        fabs(acc(2)) > limit_acc_(2) + 1e-4) {

      if (show) cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)) / limit_acc_(j));
      }
    }
  }

  return fea;
}

double NonUniformBspline::checkRatio() {
  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  // find max vel
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)) / limit_vel_(j));
    }
  }
  // find max acc
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)) / limit_acc_(j));
    }
  }
  double ratio = max(max_vel, sqrt(fabs(max_acc)));
  if (ratio > 2.0) {
    std::cerr << "[B-spline] max velocity/limit: " << max_vel
              << ", max acceleration/limit: " << max_acc << "." << std::endl;
  }

  return ratio;
}

bool NonUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  bool fea = true;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  double max_vel, max_acc;

  /* check vel feasibility and insert points */
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_(0) + 1e-4 || fabs(vel(1)) > limit_vel_(1) + 1e-4 ||
        fabs(vel(2)) > limit_vel_(2) + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      max_vel = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)) / limit_vel_(j));
      }

      double ratio = max_vel + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 1);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_);

      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
        }
      }

      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;
      }
    }
  }

  /* acc feasibility */
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_(0) + 1e-4 || fabs(acc(1)) > limit_acc_(1) + 1e-4 ||
        fabs(acc(2)) > limit_acc_(2) + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)) / limit_acc_(j));
      }

      double ratio = sqrt(max_acc) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_ - 1);

      if (i == 1 || i == 2) {
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {

        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

void NonUniformBspline::lengthenTime(const double& ratio) {
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

void NonUniformBspline::recomputeInit() {}

void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts,
                                              bool zero_end_velocity) {
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
    return;
  }

  int K = point_set.size();

  // write A
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();


  // 每列对应 x、y 或连续航向；使用同一个分解求解三个右端项。
  Eigen::MatrixXd samples(K + 4, 3);
  for (int i = 0; i < K; ++i) {
    samples.row(i) = point_set[i].transpose();
    if (i > 0) {
      const double delta = samples(i, 2) - samples(i - 1, 2);
      samples(i, 2) = samples(i - 1, 2) + std::atan2(std::sin(delta), std::cos(delta));
    }
  }
  for (int i = 0; i < 4; ++i) {
    samples.row(K + i) = start_end_derivative[i].transpose();
  }
  if (zero_end_velocity) samples.row(K + 1).setZero();
  ctrl_pts = A.colPivHouseholderQr().solve(samples);
  if (zero_end_velocity && !enforceEndVelocity(ctrl_pts, ts, Eigen::Vector3d::Zero()))
  {
    ctrl_pts.resize(0, 0);
    cout << "[B-spline]: failed to constrain terminal velocity." << endl;
  }
}

bool NonUniformBspline::enforceEndVelocity(Eigen::MatrixXd& ctrl_pts, const double& ts,
                                           const Eigen::Vector3d& end_velocity) {
  if (ctrl_pts.rows() < 3 || ctrl_pts.cols() != 3 || !std::isfinite(ts) || ts <= 0.0 ||
      !end_velocity.allFinite())
  {
    return false;
  }

  const Eigen::Index last = ctrl_pts.rows() - 1;
  const Eigen::RowVector3d end_position =
      (ctrl_pts.row(last - 2) + 4.0 * ctrl_pts.row(last - 1) + ctrl_pts.row(last)) / 6.0;
  ctrl_pts.row(last) = ctrl_pts.row(last - 2) + 2.0 * ts * end_velocity.transpose();
  ctrl_pts.row(last - 1) =
      (6.0 * end_position - ctrl_pts.row(last - 2) - ctrl_pts.row(last)) / 4.0;
  return ctrl_pts.allFinite();
}

bool NonUniformBspline::enforceEndVelocity(const Eigen::Vector3d& end_velocity) {
  if (p_ < 1 || control_points_.rows() < 3 || control_points_.cols() != 3 ||
      u_.size() != control_points_.rows() + p_ + 1 || !end_velocity.allFinite()) {
    return false;
  }

  const double end_time = getTimeSum();
  const Eigen::Index last = control_points_.rows() - 1;
  const Eigen::MatrixXd original_points = control_points_;
  const Eigen::VectorXd target_position = evaluateDeBoorT(end_time);
  Eigen::MatrixXd base_points = original_points;
  base_points.bottomRows(2).setZero();

  NonUniformBspline base(base_points, p_, interval_);
  base.setKnot(u_);
  const Eigen::VectorXd base_position = base.evaluateDeBoorT(end_time);
  const Eigen::VectorXd base_velocity =
      base.getDerivative().evaluateDeBoorT(end_time);

  Eigen::Matrix2d coefficients;
  for (int column = 0; column < 2; ++column) {
    Eigen::MatrixXd basis_points = Eigen::MatrixXd::Zero(control_points_.rows(), 1);
    basis_points(last - 1 + column, 0) = 1.0;
    NonUniformBspline basis(basis_points, p_, interval_);
    basis.setKnot(u_);
    coefficients(0, column) = basis.evaluateDeBoorT(end_time)(0);
    coefficients(1, column) =
        basis.getDerivative().evaluateDeBoorT(end_time)(0);
  }

  Eigen::FullPivLU<Eigen::Matrix2d> solver(coefficients);
  if (solver.rank() < 2 || !coefficients.allFinite() || !base_position.allFinite() ||
      !base_velocity.allFinite()) {
    return false;
  }

  Eigen::MatrixXd updated_points = original_points;
  for (Eigen::Index dimension = 0; dimension < control_points_.cols(); ++dimension) {
    Eigen::Vector2d rhs;
    rhs << target_position(dimension) - base_position(dimension),
        end_velocity(dimension) - base_velocity(dimension);
    const Eigen::Vector2d terminal_points = solver.solve(rhs);
    if (!terminal_points.allFinite()) return false;
    updated_points(last - 1, dimension) = terminal_points(0);
    updated_points(last, dimension) = terminal_points(1);
  }

  control_points_ = updated_points;
  const Eigen::VectorXd actual_position = evaluateDeBoorT(end_time);
  const Eigen::VectorXd actual_velocity =
      getDerivative().evaluateDeBoorT(end_time);
  if (!actual_position.allFinite() || !actual_velocity.allFinite() ||
      (actual_position - target_position).norm() > 1e-8 ||
      (actual_velocity - end_velocity).norm() > 1e-8) {
    control_points_ = original_points;
    return false;
  }
  return true;
}

bool NonUniformBspline::enforceStartState(const Eigen::Vector3d& position,
                                          const Eigen::Vector3d& velocity,
                                          const Eigen::Vector3d& acceleration) {
  if (p_ != 3 || control_points_.rows() < 6 || control_points_.cols() != 3 ||
      u_.size() != control_points_.rows() + p_ + 1 || !position.allFinite() ||
      !velocity.allFinite() || !acceleration.allFinite() || !u_.allFinite()) return false;

  const Eigen::MatrixXd original = control_points_;
  Eigen::MatrixXd base_points = original;
  base_points.topRows(3).setZero();
  NonUniformBspline base(base_points, p_, interval_);
  base.setKnot(u_);
  Eigen::Matrix3d rhs;
  rhs.row(0) = (position - base.evaluateDeBoorT(0.0)).transpose();
  rhs.row(1) = (velocity - base.getDerivative().evaluateDeBoorT(0.0)).transpose();
  rhs.row(2) = (acceleration -
      base.getDerivative().getDerivative().evaluateDeBoorT(0.0)).transpose();
  Eigen::Matrix3d coefficients;
  for (int i = 0; i < 3; ++i) {
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(original.rows(), 1);
    points(i, 0) = 1.0;
    NonUniformBspline basis(points, p_, interval_);
    basis.setKnot(u_);
    coefficients(0, i) = basis.evaluateDeBoorT(0.0)(0);
    coefficients(1, i) = basis.getDerivative().evaluateDeBoorT(0.0)(0);
    coefficients(2, i) = basis.getDerivative().getDerivative().evaluateDeBoorT(0.0)(0);
  }
  Eigen::FullPivLU<Eigen::Matrix3d> solver(coefficients);
  if (!coefficients.allFinite() || !rhs.allFinite() || solver.rank() < 3) return false;
  const Eigen::Matrix3d solution = solver.solve(rhs);
  if (!solution.allFinite()) return false;
  control_points_.topRows(3) = solution;
  const Eigen::VectorXd actual_position = evaluateDeBoorT(0.0);
  const Eigen::VectorXd actual_velocity = getDerivative().evaluateDeBoorT(0.0);
  const Eigen::VectorXd actual_acceleration = getDerivative().getDerivative().evaluateDeBoorT(0.0);
  if (!actual_position.allFinite() || !actual_velocity.allFinite() || !actual_acceleration.allFinite() ||
      (actual_position - position).norm() > 1e-8 || (actual_velocity - velocity).norm() > 1e-8 ||
      (actual_acceleration - acceleration).norm() > 1e-8) {
    control_points_ = original;
    return false;
  }
  return true;
}

double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double& res) {
  double          length = 0.0;
  double          dur    = getTimeSum();
  Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk() {
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
  int             dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  NonUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
