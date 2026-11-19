/**
 * @file kinodynamic_astar_config.hpp
 * @brief 六状态动力学 A* 的 ROS 无关配置。
 */

#ifndef PATH_SEARCHING__KINODYNAMIC_ASTAR_CONFIG_HPP_
#define PATH_SEARCHING__KINODYNAMIC_ASTAR_CONFIG_HPP_

#include <Eigen/Core>
#include <vector>

namespace fast_planner
{

/** @brief 动力学搜索及终端连接的参数，长度单位 m，时间单位 s。 */
struct KinodynamicAstarConfig
{
  double max_tau{0.5};
  double init_max_tau{2.0};
  int rk4_steps{10};
  double search_time_limit{0.4};
  double max_velocity{3.0};
  double max_acceleration{2.0};
  double max_u{2.0};
  double max_v{0.5};
  double max_r{0.3};
  double min_v_cmd{0.0};
  double max_v_cmd{2.0};
  double max_w_cmd{0.3};
  double search_clearance{0.5};
  double obstacle_slowdown_distance{5.0};
  double obstacle_max_speed{1.0};
  double max_ground_speed{2.0};
  double max_yaw_rate{0.3};
  double min_turn_radius{0.0};  ///< 对地曲线及艏摇转弯半径，非正值关闭，单位 m。
  double time_weight{10.0};
  double horizon{60.0};
  double shot_trigger_distance{30.0};
  double position_resolution{1.0};
  int yaw_bins{72};
  Eigen::Vector3d velocity_resolution{0.2, 0.1, 0.05};
  double time_resolution{0.8};
  double heuristic_weight{5.0};
  int node_capacity{100000};
  int collision_check_count{5};
  bool optimistic{true};
  double velocity_margin{0.0};
  std::vector<double> shot_durations{24.0, 36.0, 48.0, 64.0, 80.0, 100.0};
  double shot_cruise_speed{1.1};
  double shot_integration_step{0.1};
  double shot_collision_step{0.04};
  double shot_position_tolerance{3.5};
  double shot_yaw_tolerance{0.17453292519943295};
  double shot_u_tolerance{0.08};
  double shot_v_tolerance{0.06};
  double shot_r_tolerance{0.04};
  double shot_min_v_cmd{0.0};
  double shot_max_v_cmd{2.0};
  double shot_max_w_cmd{0.3};
  double shot_command_scale{0.85};
  double shot_max_u{2.0};
  double shot_max_v{0.5};
  double shot_max_r{0.3};
  double shot_map_height{0.0};
  double shot_fd_epsilon{0.001};
  double shot_damping{0.02};
  double shot_time_limit{0.05};
  int shot_max_iterations{16};
};

}  // namespace fast_planner

#endif  // PATH_SEARCHING__KINODYNAMIC_ASTAR_CONFIG_HPP_
