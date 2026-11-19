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



#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {
// An implementation of non-uniform B-spline with different dimensions
// It also represents uniform B-spline which is a special case of non-uniform
class NonUniformBspline {
private:
  // control points for B-spline with different dimensions.
  // Each row represents one single control point
  // The dimension is determined by column number
  // e.g. B-spline with N points in 3D space -> Nx3 matrix
  Eigen::MatrixXd control_points_;

  int             p_, n_, m_;  // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd u_;          // knots vector
  double          interval_;   // knot span \delta t

  Eigen::MatrixXd getDerivativeControlPoints();

  Eigen::VectorXd limit_vel_, limit_acc_;  // Limits for each curve component.
  double limit_ratio_;  // Maximum time adjustment ratio per step.

public:
  NonUniformBspline() {}
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);
  ~NonUniformBspline();

  // initialize as an uniform B-spline
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  // get / set basic bspline info

  void                                   setKnot(const Eigen::VectorXd& knot);
  Eigen::VectorXd                        getKnot();
  Eigen::MatrixXd                        getControlPoint();
  double                                 getInterval();
  void                                   getTimeSpan(double& um, double& um_p);
  pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts();

  // compute position / derivative

  Eigen::VectorXd   evaluateDeBoor(const double& u);   // use u \in [up, u_mp]
  Eigen::VectorXd   evaluateDeBoorT(const double& t);  // use t \in [0, duration]
  NonUniformBspline getDerivative();

  /**
   * @brief 以最小二乘拟合均匀三次样条的控制点。
   * @param ts 相邻采样时刻间隔，单位 s，必须大于 0。
   * @param point_set 按时间排列的 K 个 [x_E, y_N, phi] 样本；航向单位 rad。
   * @param start_end_derivative 起、末速度及起、末加速度，依次共四项。
   * @param ctrl_pts 输出 (K+2)×3 控制点；位置及导数为软拟合而非硬约束。
   * @param zero_end_velocity 是否将末端速度设为精确零，仅用于任务终段。
   * @note 航向按相邻样本的最短角度差展开；调用方应保证采样足够密集。
   */
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    Eigen::MatrixXd&               ctrl_pts,
                                    bool zero_end_velocity = false);

  /**
   * @brief 精确设置均匀三次样条的末端速度，并保持末端位置不变。
   * @param ctrl_pts 三列控制点，依次为 x_E、y_N、phi。
   * @param ts 均匀 knot 间隔，单位 s。
   * @param end_velocity 末端 [x_dot,y_dot,phi_dot]，单位 m/s、rad/s。
   * @return 控制点形状及输入有效时返回 true。
   */
  static bool enforceEndVelocity(Eigen::MatrixXd& ctrl_pts, const double& ts,
                                 const Eigen::Vector3d& end_velocity);

  /**
   * @brief 按当前 knot vector 设置末端速度，并保持末端位置不变。
   * @param end_velocity 末端 [x_dot,y_dot,phi_dot]，单位 m/s、rad/s。
   * @return spline 有效且末端约束可解时返回 true。
   */
  bool enforceEndVelocity(const Eigen::Vector3d& end_velocity);

  /**
   * @brief 按当前节点向量恢复起始位姿、速度和加速度，避免重规划时重置运动状态。
   * @param position 起始 [x_E,y_N,phi]，航向与本曲线采用同一展开分支。
   * @param velocity 起始世界系 [x_dot,y_dot,phi_dot]。
   * @param acceleration 起始世界系 [x_ddot,y_ddot,phi_ddot]。
   * @return 三次样条起始约束可解且校验通过时返回 true，失败保持原控制点。
   */
  bool enforceStartState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                         const Eigen::Vector3d& acceleration);

  /* check feasibility, adjust time */

  void   setPhysicalLimits(const double& vel, const double& acc);
  void   setPhysicalLimits(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc);
  bool   checkFeasibility(bool show = false);
  double checkRatio();
  void   lengthenTime(const double& ratio);
  bool   reallocateTime(bool show = false);

  /* for performance evaluation */

  double getTimeSum();
  double getLength(const double& res = 0.01);
  double getJerk();
  void   getMeanAndMaxVel(double& mean_v, double& max_v);
  void   getMeanAndMaxAcc(double& mean_a, double& max_a);

  void recomputeInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif
