/**
 * @file usv_model.hpp
 * @brief 无人船公共动力学的 NMPC 适配接口。
 */

#pragma once

#include <casadi/casadi.hpp>

#include "usv_nmpc_controller/common_types.hpp"

namespace usv_nmpc
{

/** @brief 船体辨识得到的三自由度动力学系数，供控制器预测模型使用。 */
struct ModelParameters
{
  double au{-0.431930955};
  double bu{0.437133189};
  double bu0{0.0};
  double av1{-0.313324916};
  double av2{0.006741061};
  double bv{0.407958500};
  double bv0{0.000993619};
  double ar1{-0.166118168};
  double ar2{-0.544509453};
  double br{0.837536662};
  double br0{0.000626529};
};

inline constexpr ModelParameters kModelParameters{};

/**
 * @brief 根据 u_dot=0 计算稳态前向速度指令，支持数值和 CasADi 符号量。
 * @param[in] u 期望船体前向速度，单位 m/s。
 * @return 未限幅的前向速度指令，单位 m/s。
 */
template<typename T>
T steadySurgeCommand(const T & u)
{
  return (-kModelParameters.au * u - kModelParameters.bu0) / kModelParameters.bu;
}

/**
 * @brief 根据 r_dot=0 计算稳态转艏角速度指令。
 * @param[in] v 期望船体右向速度，单位 m/s。
 * @param[in] r 期望转艏角速度，单位 rad/s。
 * @return 未限幅的转艏角速度指令，单位 rad/s。
 */
template<typename T>
T steadyYawCommand(const T & v, const T & r)
{
  return (-kModelParameters.ar1 * v - kModelParameters.ar2 * r -
         kModelParameters.br0) / kModelParameters.br;
}

/**
 * @brief 求同时满足 v_dot=0 和 r_dot=0 的稳态侧向速度。
 * @param[in] yaw_rate 期望转艏角速度，单位 rad/s。
 * @return 船体右向速度，单位 m/s；不限制对应的稳态指令幅值。
 */
double steadySwayVelocity(double yaw_rate);

/** @brief 无人船六状态动力学模型。 */
class USVModel
{
public:
  /**
   * @brief 构建六状态连续时间动力学的 CasADi 表达式。
   * @param[in] state 状态列向量 [x, y, psi, u, v, r]，单位同 State。
   * @param[in] control 指令列向量 [speed, yaw_rate]，单位为 m/s 和 rad/s。
   * @return 按相同状态顺序排列的时间导数列向量，不作限幅。
   */
  static casadi::SX dynamicsSX(
    const casadi::SX & state,
    const casadi::SX & control);

  /**
   * @brief 计算六状态连续时间导数，不作限幅。
   * @param[in] state 当前状态。
   * @param[in] control 模型指令。
   * @return 与 State 字段顺序相同的时间导数。
   */
  static State derivative(
    const State & state,
    const Control & control);

  /**
   * @brief 在恒定指令下使用四阶 Runge-Kutta 法推进状态，不作限幅。
   * @param[in] state 当前状态。
   * @param[in] control 此时间步内的模型指令。
   * @param[in] dt 积分步长，必须为有限正数，单位 s。
   * @return 积分后的状态；航向保持连续，不执行角度归一化。
   * @throws std::invalid_argument 积分步长无效。
   */
  static State stepRK4(
    const State & state,
    const Control & control,
    double dt);
};

}  // namespace usv_nmpc
