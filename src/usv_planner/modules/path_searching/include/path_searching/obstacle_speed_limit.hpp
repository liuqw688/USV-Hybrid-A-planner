/**
 * @file obstacle_speed_limit.hpp
 * @brief 输入地图占用区域附近的对地合速度限制与制动引导。
 */
#ifndef PATH_SEARCHING__OBSTACLE_SPEED_LIMIT_HPP_
#define PATH_SEARCHING__OBSTACLE_SPEED_LIMIT_HPP_

#include <algorithm>
#include <cmath>

namespace fast_planner
{
/** @brief 限速区使用对地合速度，速度分量可取船体系或世界系。 */
inline bool obstacleSpeedFeasible(double distance, double speed, double radius, double limit)
{
  return std::isfinite(distance) && std::isfinite(speed) &&
      (distance > radius || speed <= limit);
}

/** @brief 零前向指令被动制动的速度包络，用于搜索软引导而非安全证明。 */
inline double obstacleBrakingExcess(double distance, double speed, double radius, double limit)
{
  constexpr double kSurgeDamping = 0.431930955;
  return std::max(0.0, speed - limit - kSurgeDamping * std::max(0.0, distance - radius));
}
}  // namespace fast_planner
#endif  // PATH_SEARCHING__OBSTACLE_SPEED_LIMIT_HPP_
