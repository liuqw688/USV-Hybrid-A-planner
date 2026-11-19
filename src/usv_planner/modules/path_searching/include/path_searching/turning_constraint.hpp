/**
 * @file turning_constraint.hpp
 * @brief 搜索与样条共用的平面转弯约束；航向为北零西正。
 */
#ifndef PATH_SEARCHING__TURNING_CONSTRAINT_HPP_
#define PATH_SEARCHING__TURNING_CONSTRAINT_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace fast_planner
{

/** @brief 小于此速度时正则化曲率分母，避免停车处的零除，单位 m/s。 */
constexpr double kTurningSpeedFloor = 0.3;

/**
 * @brief 计算对地曲率和艏摇转弯率相对于最小半径的有符号倍率。
 * @param[in] velocity 位姿一阶导数 [x_dot_E, y_dot_N, phi_dot]。
 * @param[in] acceleration 位姿二阶导数。
 * @param[in] radius 最小转弯半径，单位 m；非正值关闭约束。
 * @return 两项绝对值均不大于 1 时满足约束；低速采用正则化速度。
 * @note 第一项包含侧滑变化的影响；第二项防止独立航向样条原地快速掉头。
 */
inline Eigen::Vector2d turningRatios(const Eigen::Vector3d & velocity,
  const Eigen::Vector3d & acceleration, double radius)
{
  if (radius <= 0.0) return Eigen::Vector2d::Zero();
  const double speed = std::max(kTurningSpeedFloor, velocity.head<2>().norm());
  return {radius * (velocity.x() * acceleration.y() - velocity.y() * acceleration.x()) /
      (speed * speed * speed), radius * velocity.z() / speed};
}

/**
 * @brief 计算转弯超限平方代价及其对位姿速度、加速度的解析梯度。
 * @param[in] velocity 位姿一阶导数。
 * @param[in] acceleration 位姿二阶导数。
 * @param[in] radius 最小转弯半径，单位 m。
 * @param[out] velocity_gradient 代价对位姿速度的导数。
 * @param[out] acceleration_gradient 代价对位姿加速度的导数。
 * @return 非负超限代价。
 */
inline double turningCost(const Eigen::Vector3d & velocity,
  const Eigen::Vector3d & acceleration, double radius,
  Eigen::Vector3d & velocity_gradient, Eigen::Vector3d & acceleration_gradient)
{
  velocity_gradient.setZero();
  acceleration_gradient.setZero();
  const auto ratios = turningRatios(velocity, acceleration, radius);
  const double speed = std::max(kTurningSpeedFloor, velocity.head<2>().norm());
  Eigen::Vector3d speed_gradient = Eigen::Vector3d::Zero();
  if (velocity.head<2>().norm() > kTurningSpeedFloor)
    speed_gradient.head<2>() = velocity.head<2>() / speed;
  double cost = 0.0;
  for (int i = 0; i < 2; ++i) {
    const double excess = std::abs(ratios(i)) - 1.0;
    if (excess <= 0.0) continue;
    const double scale = 2.0 * excess * std::copysign(1.0, ratios(i));
    cost += excess * excess;
    if (i == 0) {
      const double factor = radius / (speed * speed * speed);
      velocity_gradient += scale * (factor * Eigen::Vector3d(acceleration.y(), -acceleration.x(), 0.0) -
          3.0 * ratios(i) / speed * speed_gradient);
      acceleration_gradient += scale * factor * Eigen::Vector3d(-velocity.y(), velocity.x(), 0.0);
    } else {
      velocity_gradient += scale * (Eigen::Vector3d(0.0, 0.0, radius / speed) -
          ratios(i) / speed * speed_gradient);
    }
  }
  return cost;
}
}  // namespace fast_planner
#endif  // PATH_SEARCHING__TURNING_CONSTRAINT_HPP_
