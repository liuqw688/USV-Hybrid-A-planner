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



#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include "bspline_opt/bspline_optimizer_config.hpp"
#include "path_searching/search_environment.hpp"

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace fast_planner {
using std::vector;
class BsplineOptimizer {

public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /**
   * @brief 设置优化器查询障碍距离所使用的地图抽象。
   * @param[in] environment 提供距离及其梯度查询的地图接口。
   */
  void setEnvironment(std::shared_ptr<SearchEnvironment> environment);
  /**
   * @brief 注入与 ROS 2 无关的优化参数。
   * @param[in] config 优化权重、运动边界和求解器设置。
   */
  void setConfig(const BsplineOptimizerConfig& config);
  /**
   * @brief 优化 B 样条控制点。
   * @param[in] points 初始控制点矩阵，每行一个控制点。
   * @param[in] ts 样条节点间隔，单位 s。
   * @param[in] cost_function 启用的代价位掩码。
   * @param[in] max_num_id 最大评估次数配置索引，范围为 0 至 3。
   * @param[in] max_time_id 最大求解时间配置索引，范围为 0 至 3。
   * @return 优化后的控制点矩阵。
   */
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setCostFunction(const int& cost_function);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd         getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  friend class BsplineNavigationTest;
  std::shared_ptr<SearchEnvironment> environment_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function

  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // acc smoothness
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double max_global_vel_, max_yaw_rate_;  // Limits for vx/vy and r
  double min_turn_radius_{0.0}, channel_weight_{2.0};
  double max_global_acc_, max_yaw_acc_;   // Limits for ax/ay and dot_r
  double yaw_rate_weight_, yaw_acc_weight_;  // Yaw feasibility cost weights
  double yaw_smoothness_weight_;  // Heading smoothness cost weight
  double yaw_endpoint_weight_;    // Heading endpoint cost weight
  double yaw_guide_weight_;       // Heading guide cost weight
  double yaw_waypoint_weight_;    // Heading waypoint cost weight
  double yaw_bound_;              // Heading control point adjustment bound, rad
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int    algorithm1_;             // optimization algorithms for quadratic cost
  int    algorithm2_;             // optimization algorithms for general cost
  int    max_iteration_num_[4];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_waypoints_;

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  std::vector<double> best_variable_;  //
  double              min_cost_;       //

  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);
  /** @brief 对实际曲线积分航道通行及转弯超限代价，并传播控制点梯度。 */
  void calcNavigationCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  void calcVisibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                         vector<Eigen::Vector3d>& gradient);
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);
  bool isQuadratic();

  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  using Ptr = std::unique_ptr<BsplineOptimizer>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif
