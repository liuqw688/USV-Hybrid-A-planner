/**
 * @file dynamics.hpp
 * @brief 无人艇三自由度动力学、运动学与命令限幅。
 */

#ifndef USV_MODEL__DYNAMICS_HPP_
#define USV_MODEL__DYNAMICS_HPP_

#include <cmath>

namespace usv_model_sim
{

constexpr double kPi = 3.14159265358979323846;

/** @brief 船体辨识得到的三自由度模型系数，须与 NMPC 模型的系数保持一致。 */
struct ModelParameters
{
  double au{-0.431930955};  ///< u_dot 中 u 的系数。
  double bu{0.437133189};  ///< u_dot 中前向速度命令的系数。
  double bu0{0.0};  ///< u_dot 常值偏置。
  double av1{-0.313324916};  ///< v_dot 中 v 的系数。
  double av2{0.006741061};  ///< v_dot 中 r 的系数。
  double bv{0.407958500};  ///< v_dot 中角速度命令的系数。
  double bv0{0.000993619};  ///< v_dot 常值偏置。
  double ar1{-0.166118168};  ///< r_dot 中 v 的系数。
  double ar2{-0.544509453};  ///< r_dot 中 r 的系数。
  double br{0.837536662};  ///< r_dot 中角速度命令的系数。
  double br0{0.000626529};  ///< r_dot 常值偏置。
};

inline constexpr ModelParameters kModelParameters{};

/** @brief 局部 ENU 位姿及船体坐标系速度状态。 */
struct State
{
  double east{0.0};  ///< 东向位置，单位 m。
  double north{0.0};  ///< 北向位置，单位 m。
  double yaw{0.0};  ///< 北向为零、西向为正的船首航向，单位 rad。
  double u{0.0};  ///< 船体前向速度，单位 m/s。
  double v{0.0};  ///< 船体右向速度，单位 m/s。
  double yaw_rate{0.0};  ///< 转艏角速度，单位 rad/s。
};

/** @brief 船舶模型的前向速度和转艏角速度命令。 */
struct Command
{
  double forward_speed{0.0};  ///< 前向速度命令，单位 m/s。
  double yaw_rate{0.0};  ///< 转艏角速度命令，单位 rad/s。
};

/** @brief 仿真模型的命令幅值上限。 */
struct Parameters
{
  double max_speed_mps{5.0};  ///< 前向速度命令绝对值上限，单位 m/s。
  double max_yaw_rate_rad_s{15.0 * kPi / 180.0};  ///< 转艏角速度命令绝对值上限，单位 rad/s。
};

/** @brief 船体参数报告中的三自由度动力学和运动学模型。 */
class Dynamics
{
public:
  /**
   * @brief 使用指定命令上限构造模型。
   * @param[in] parameters 命令幅值上限；调用方须保证为有限非负数。
   */
  explicit Dynamics(Parameters parameters = Parameters());

  /**
   * @brief 计算连续时间状态导数，不对命令或状态限幅。
   * @param[in] state 当前状态。
   * @param[in] command 模型输入。
   * @return 状态导数；位置导数采用 ENU 坐标，船体 v 以右向为正。
   */
  static State evaluate(const State & state, const Command & command);
  /**
   * @brief 使用经典四阶 Runge-Kutta 法推进状态，不做限幅。
   * @param[in] state 当前状态。
   * @param[in] command 模型输入。
   * @param[in] dt 积分步长，必须为有限正数，单位 s。
   * @return 积分后的状态。
   * @throws std::invalid_argument 积分步长无效。
   */
  static State integrate(const State & state, const Command & command, double dt);
  /**
   * @brief 分别限制前向速度和转艏角速度命令的幅值。
   * @param[in] command 待限幅命令。
   * @return 按 Parameters 对称限幅后的命令。
   */
  Command limit_command(Command command) const;
  /**
   * @brief 对命令限幅后计算状态导数。
   * @param[in] state 当前状态。
   * @param[in] command 模型命令。
   * @return 与 State 字段顺序相同的时间导数。
   */
  State derivative(const State & state, const Command & command) const;
  /**
   * @brief 对命令限幅、积分并限制前向速度和转艏角速度状态。
   * @param[in,out] state 待推进状态；更新后航向归一化到 [-pi, pi]。
   * @param[in] command 此时间步内的模型命令。
   * @param[in] dt 时间步长，单位 s；非有限数或非正数时不更新状态。
   */
  void step(State & state, const Command & command, double dt) const;

private:
  Parameters parameters_;
};

}  // namespace usv_model_sim

#endif  // USV_MODEL__DYNAMICS_HPP_
