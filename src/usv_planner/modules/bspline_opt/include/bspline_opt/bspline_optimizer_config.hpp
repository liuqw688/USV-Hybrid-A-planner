/**
 * @file bspline_optimizer_config.hpp
 * @brief B 样条优化器的 ROS 无关配置。
 */

#ifndef BSPLINE_OPT__BSPLINE_OPTIMIZER_CONFIG_HPP_
#define BSPLINE_OPT__BSPLINE_OPTIMIZER_CONFIG_HPP_

#include <array>

namespace fast_planner
{

/** @brief 位置、航向和避障代价及其求解边界。 */
struct BsplineOptimizerConfig
{
  double lambda1{-1.0};  ///< 三阶差分平滑代价权重。
  double lambda2{-1.0};  ///< 障碍距离代价权重。
  double lambda3{-1.0};  ///< 速度与加速度可行性代价权重。
  double lambda4{-1.0};  ///< 末端位姿代价权重。
  double lambda5{-1.0};  ///< 引导路径代价权重。
  double lambda6{-1.0};  ///< 可视代价权重，保留旧配置接口。
  double lambda7{-1.0};  ///< 中间航点代价权重。
  double lambda8{-1.0};  ///< 加速度平滑代价权重，保留旧配置接口。
  double distance_margin{-1.0};  ///< 控制点障碍物软约束距离，单位 m。
  double max_velocity{-1.0};  ///< 样条单轴速度上限，单位 m/s。
  double max_acceleration{-1.0};  ///< 样条单轴加速度上限，单位 m/s^2。
  double max_global_velocity{-1.0};  ///< 平面位置速度上限，单位 m/s。
  double max_yaw_rate{-1.0};  ///< 航向变化率上限，单位 rad/s。
  double min_turn_radius{0.0};  ///< 与搜索共用的最小转弯半径，单位 m。
  double channel_weight{2.0};  ///< 实际样条通行代价的积分权重。
  double max_global_acceleration{-1.0};  ///< 平面位置加速度上限，单位 m/s^2。
  double max_yaw_acceleration{-1.0};  ///< 航向角加速度上限，单位 rad/s^2。
  double yaw_rate_weight{-1.0};  ///< 航向变化率可行性代价权重。
  double yaw_acceleration_weight{-1.0};  ///< 航向角加速度可行性代价权重。
  double yaw_smoothness_weight{-1.0};  ///< 平滑代价中航向相对平移的权重。
  double yaw_endpoint_weight{-1.0};  ///< 末端代价中航向误差的权重。
  double yaw_guide_weight{-1.0};  ///< 引导代价中航向误差的权重。
  double yaw_waypoint_weight{-1.0};  ///< 航点代价中航向误差的权重。
  double yaw_bound{-1.0};  ///< 航向控制点单次调整界，单位 rad。
  double visibility_min{-1.0};  ///< 可视代价距离阈值，单位 m。
  double visibility_spacing{-1.0};  ///< 可视代价采样间距，单位 m。
  double visibility_weight{-1.0};  ///< 可视代价权重。
  std::array<int, 4> max_iterations{{-1, -1, -1, -1}};  ///< 四阶段最大目标函数评估次数。
  std::array<double, 4> max_iteration_time{{-1.0, -1.0, -1.0, -1.0}};  ///< 四阶段最大求解时长，单位 s。
  int quadratic_algorithm{-1};  ///< 二次目标使用的 NLopt 算法编号。
  int general_algorithm{-1};  ///< 一般目标使用的 NLopt 算法编号。
  int spline_order{-1};  ///< 样条次数，当前规划器使用三次样条。
};

}  // namespace fast_planner

#endif  // BSPLINE_OPT__BSPLINE_OPTIMIZER_CONFIG_HPP_
