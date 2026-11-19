/**
 * @file common_types.hpp
 * @brief NMPC 使用的状态、控制量和基础角度函数。
 */

#pragma once

#include <cmath>

namespace usv_nmpc
{

constexpr double PI = 3.14159265358979323846;
/**
 * @brief 地速接近零时的航迹角正则化速度，单位：m/s。
 *
 * 常规速度下补偿侧滑角，停车时连续过渡到船体航向，避免航向误差跳变。
 */
constexpr double kCourseSpeedFloorMps = 0.3;

/**
 * @brief 无人船平面状态。
 * @details x、y 为 ENU 位置(m)；psi 为北零西正航向(rad)；u、v 为船体系前向、右向速度(m/s)；r 为转艏角速度(rad/s)。
 */
struct State
{
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double u = 0.0;
  double v = 0.0;
  double r = 0.0;
};

/**
 * @brief 控制输入。
 * @details speed 为前向速度指令(m/s)，yaw_rate 为向西转艏为正的角速度指令(rad/s)。
 */
struct Control
{
  double speed = 0.0;
  double yaw_rate = 0.0;
};

/** @brief NMPC 的位置、姿态及速度参考；曲率和地速切线角仅用于诊断。 */
struct RefPoint
{
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double u = 0.0;
  double v = 0.0;
  double r = 0.0;
  /** 转艏率与地速之比，单位 1/m；忽略侧滑角变化率，不传入优化器。 */
  double curvature = 0.0;
  /** 路径切线角元数据，北为零/向西为正；不传入优化器。 */
  double course = 0.0;
};

/**
 * @brief 将角度归一化到 [-PI, PI]。
 * @param[in] angle 输入角度，单位为 rad。
 * @return 与输入角等价的归一化角，单位为 rad。
 */
inline double wrapAngle(double angle)
{
  while (angle > PI) {
    angle -= 2.0 * PI;
  }

  while (angle < -PI) {
    angle += 2.0 * PI;
  }

  return angle;
}

/**
 * @brief 计算低速平滑后的侧滑补偿角。
 *
 * 正常前进时近似 atan2(v, u)；速度趋近于零时连续趋近于零。该正则化角用于
 * 航向代价，不作为倒车时精确地速方向的定义。
 * @param[in] u 船体前向速度，单位为 m/s。
 * @param[in] v 船体右向速度，单位为 m/s。
 * @return 侧滑补偿角，单位为 rad。
 */
inline double courseSideslipAngle(double u, double v)
{
  const double speed_squared = u * u + v * v;
  const double floor_squared =
    kCourseSpeedFloorMps * kCourseSpeedFloorMps;
  const double blend = speed_squared / (speed_squared + floor_squared);
  const double regularized_sideslip = std::atan2(
    v, std::sqrt(u * u + floor_squared));
  return blend * regularized_sideslip;
}

/** @brief 相对参考切线的沿程与右向横向误差，单位为 m。 */
struct PathFrameError
{
  double along = 0.0;
  double lateral = 0.0;
};

/**
 * @brief 计算状态相对参考地速切线 Frenet 坐标系的沿程和横向误差。
 *
 * 参考 psi 是船首航向，参考 beta 是由参考 u/v 得到的侧滑角。
 * 在 ENU 平面中（x 向东、y 向北、psi 北为 0，向东为负），地速
 * 切线航向为 chi = psi - beta。因此位置误差必须投影到 chi，而不是
 * 直接投影到船首方向 psi；否则存在侧滑时，纯粹沿轨迹前进的参考点
 * 会被误判为有横向误差。
 * @param[in] state 当前船舶状态。
 * @param[in] reference 当前参考状态。
 * @return 沿程误差和横向误差，单位均为 m。
 */
inline PathFrameError pathFrameError(
  const State & state, const RefPoint & reference)
{
  const double dx = state.x - reference.x;
  const double dy = state.y - reference.y;
  const double beta_ref = courseSideslipAngle(reference.u, reference.v);
  const double chi_ref = reference.psi - beta_ref;
  return {
    -std::sin(chi_ref) * dx + std::cos(chi_ref) * dy,
    std::cos(chi_ref) * dx + std::sin(chi_ref) * dy};
}

/**
 * @brief 计算实际船首相对参考船首的最短有符号航向误差。
 * @param[in] state 当前船舶状态。
 * @param[in] reference 当前参考状态。
 * @return 归一化航向差，单位为 rad。
 */
inline double referenceHeadingError(
  const State & state, const RefPoint & reference)
{
  return wrapAngle(reference.psi - state.psi);
}

}  // namespace usv_nmpc
