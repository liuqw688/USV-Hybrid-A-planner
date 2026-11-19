/**
 * @file usv_dynamics.hpp
 * @brief 无人船三自由度模型及参考系转换接口。
 */

#ifndef PATH_SEARCHING__USV_DYNAMICS_HPP_
#define PATH_SEARCHING__USV_DYNAMICS_HPP_

#include <Eigen/Core>
#include <usv_model_sim/dynamics.hpp>
#include <vector>

namespace fast_planner
{
namespace usv_dynamics
{

using State = Eigen::Matrix<double, 6, 1>;  ///< [x_E, y_N, phi, u_forward, v_starboard, r].
using Command = Eigen::Vector3d;  ///< [V_cmd, W_cmd, reserved].

/** @brief 与 USV_systems 的仿真器和 NMPC 一致的辨识模型系数。 */
struct ModelParameters
{
  double au;  ///< u_dot 中前向速度 u 的系数。
  double bu;  ///< u_dot 中前向速度命令 V_cmd 的系数。
  double bu0;  ///< u_dot 的常值偏置。
  double av1;  ///< v_dot 中右向速度 v 的系数。
  double av2;  ///< v_dot 中艏摇角速度 r 的系数。
  double bv;  ///< v_dot 中角速度命令 W_cmd 的系数。
  double bv0;  ///< v_dot 的常值偏置。
  double ar1;  ///< r_dot 中右向速度 v 的系数。
  double ar2;  ///< r_dot 中艏摇角速度 r 的系数。
  double br;  ///< r_dot 中角速度命令 W_cmd 的系数。
  double br0;  ///< r_dot 的常值偏置。
};

/** @brief 零前向命令下的纵向自然制动估算。 */
struct SurgeBrakingEnvelope
{
  double time_s;  ///< 达到目标速度容差所需时间，单位 s。
  double distance_m;  ///< 速度衰减期间的前向行驶距离，单位 m。
};

constexpr ModelParameters kModelParameters{
  usv_model_sim::kModelParameters.au,
  usv_model_sim::kModelParameters.bu,
  usv_model_sim::kModelParameters.bu0,
  usv_model_sim::kModelParameters.av1,
  usv_model_sim::kModelParameters.av2,
  usv_model_sim::kModelParameters.bv,
  usv_model_sim::kModelParameters.bv0,
  usv_model_sim::kModelParameters.ar1,
  usv_model_sim::kModelParameters.ar2,
  usv_model_sim::kModelParameters.br,
  usv_model_sim::kModelParameters.br0};

/**
 * @brief 计算 [x_E, y_N, phi, u, v, r] 的连续时间导数。
 * @param[in] state 状态；位置为 ENU，速度为 RFU，phi 北零西正。
 * @param[in] command 输入 [V_cmd, W_cmd, reserved]；第三项不使用。
 * @return 按状态顺序排列的导数。
 */
State derivative(const State & state, const Command & command);

/**
 * @brief 按零前向命令估算纵向速度的自然制动时间和距离。
 * @param[in] initial_u 当前船体纵向速度，单位 m/s。
 * @param[in] target_abs_u 终端纵向速度绝对值容差，必须大于零，单位 m/s。
 * @return 达到目标速度容差的时间和行驶距离。
 * @note 适用于 V_cmd=0、纵向常值偏置为零的当前船模；横向和艏摇状态不计入该估算。
 */
SurgeBrakingEnvelope estimatePassiveSurgeBraking(double initial_u, double target_abs_u);

/**
 * @brief 将 RFU 船体速度转换为 ENU 位置速度。
 * @param[in] pose 位姿 [x_E, y_N, phi]。
 * @param[in] body_velocity 船体速度 [u_forward, v_starboard, r]。
 * @return ENU 速度 [x_dot_E, y_dot_N, r]。
 */
Eigen::Vector3d toWorldVelocity(
  const Eigen::Vector3d & pose, const Eigen::Vector3d & body_velocity);

/**
 * @brief 将 ENU 位置速度转换为 RFU 船体速度。
 * @param[in] pose 位姿 [x_E, y_N, phi]。
 * @param[in] world_velocity ENU 速度 [x_dot_E, y_dot_N, r]。
 * @return 船体速度 [u_forward, v_starboard, r]。
 */
Eigen::Vector3d toBodyVelocity(
  const Eigen::Vector3d & pose, const Eigen::Vector3d & world_velocity);

/**
 * @brief 从船体右轴方向提取导航航向。
 * @param[in] east_right_axis 船体右轴在 ENU 中的东向分量。
 * @param[in] north_right_axis 船体右轴在 ENU 中的北向分量。
 * @return 北为 0、向西为正的航向，单位 rad。
 */
double headingFromRightVector(double east_right_axis, double north_right_axis);

/**
 * @brief 从路径前向切线提取导航航向。
 * @param[in] east_forward_axis 前向切线的东向分量。
 * @param[in] north_forward_axis 前向切线的北向分量。
 * @return 北为 0、向东为 -pi/2、向西为 pi/2 的航向，单位 rad。
 */
double headingFromForwardVector(double east_forward_axis, double north_forward_axis);

/**
 * @brief 根据船体前向加速度及艏摇加速度反算两路控制输入。
 * @param[in] forward_velocity 船体前向速度 u，单位 m/s。
 * @param[in] starboard_velocity 船体右向速度 v，单位 m/s。
 * @param[in] yaw_rate 艏摇角速度 r，单位 rad/s。
 * @param[in] forward_acceleration 前向加速度 u_dot，单位 m/s^2。
 * @param[in] yaw_acceleration 艏摇角加速度 r_dot，单位 rad/s^2。
 * @return 控制输入 [V_cmd, W_cmd, 0]。
 */
Command commandFromBodyAccelerations(
  double forward_velocity, double starboard_velocity, double yaw_rate,
  double forward_acceleration, double yaw_acceleration);

/**
 * @brief 将船体状态导数转换为 ENU 位姿加速度。
 * @param[in] state 当前六状态。
 * @param[in] command 当前恒定输入。
 * @return [x_ddot_E, y_ddot_N, r_dot]。
 */
Eigen::Vector3d poseAcceleration(const State & state, const Command & command);

/**
 * @brief 根据 ENU 位姿速度和加速度反算船体速度及两路控制输入。
 * @param[in] pose 位姿 [x_E, y_N, phi]。
 * @param[in] pose_velocity ENU 位姿一阶导数 [x_dot_E, y_dot_N, r]。
 * @param[in] pose_acceleration ENU 位姿二阶导数 [x_ddot_E, y_ddot_N, r_dot]。
 * @param[out] body_velocity 船体速度 [u_forward, v_starboard, r]。
 * @param[out] command 控制输入 [V_cmd, W_cmd, 0]。
 */
void stateAndCommandFromPoseDerivatives(
  const Eigen::Vector3d & pose,
  const Eigen::Vector3d & pose_velocity,
  const Eigen::Vector3d & pose_acceleration,
  Eigen::Vector3d & body_velocity,
  Eigen::Vector3d & command);

/**
 * @brief 按给定子步数使用经典 RK4 积分船体状态。
 * @param[in] state 初始状态。
 * @param[in] command 积分期間保持不变的控制输入。
 * @param[in] duration_s 积分时长，有限且非负，单位 s。
 * @param[in] steps 积分子步数，必须为正整数。
 * @return 积分后的状态；航向不归一化，第三控制分量不使用。
 * @throws std::invalid_argument 时长无效或子步数非正。
 */
State integrateRK4(
  const State & state, const Command & command, double duration_s, int steps);

/**
 * @brief 积分一次并输出等时间间隔的碰撞检查状态。
 * @param[in] state 初始六状态。
 * @param[in] command 积分期间恒定的控制输入。
 * @param[in] duration_s 总时长，单位 s。
 * @param[in] steps 总 RK4 步数。
 * @param[in] count 输出点数；必须整除总步数。
 * @param[out] samples 不含初始状态、包含末状态的检查点。
 */
void integrateRK4Samples(const State & state, const Command & command, double duration_s,
  int steps, int count, std::vector<State> & samples);

}  // namespace usv_dynamics
}  // namespace fast_planner

#endif  // PATH_SEARCHING__USV_DYNAMICS_HPP_
