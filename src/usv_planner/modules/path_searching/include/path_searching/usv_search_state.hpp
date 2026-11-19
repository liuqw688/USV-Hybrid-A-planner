/**
 * @file usv_search_state.hpp
 * @brief 六状态搜索的离散键，区分同一位姿处不同的运动状态。
 */
#ifndef PATH_SEARCHING_USV_SEARCH_STATE_HPP_
#define PATH_SEARCHING_USV_SEARCH_STATE_HPP_

#include <Eigen/Core>
#include <cmath>

namespace fast_planner
{
using SearchStateIndex = Eigen::Matrix<int, 6, 1>;

/**
 * @brief 将 [x_E, y_N, phi, u, v, r] 转换为搜索格索引。
 * @param state 连续六状态，航向单位为 rad。
 * @param origin 地图原点，仅使用 x/y 分量。
 * @param position_resolution 平面位置分辨率，单位 m，须为正数。
 * @param yaw_bins 一周的航向格数量，须为正数。
 * @param velocity_resolution u/v/r 分辨率，单位分别为 m/s、m/s、rad/s，须为正数。
 * @return 六维离散索引；零航向居于周期格中心，速度使用就近取整。
 */
inline SearchStateIndex discretizeSearchState(
    const Eigen::Matrix<double, 6, 1>& state, const Eigen::Vector3d& origin,
    double position_resolution, int yaw_bins, const Eigen::Vector3d& velocity_resolution)
{
  const double two_pi = 2.0 * std::acos(-1.0);
  double heading = std::fmod(state(2), two_pi);
  if (heading < 0.0) heading += two_pi;
  SearchStateIndex index;
  index(0) = static_cast<int>(std::floor((state(0) - origin(0)) / position_resolution));
  index(1) = static_cast<int>(std::floor((state(1) - origin(1)) / position_resolution));
  index(2) = static_cast<int>(std::floor(heading * yaw_bins / two_pi + 0.5)) % yaw_bins;
  for (int i = 0; i < 3; ++i)
    index(i + 3) = static_cast<int>(std::floor(state(i + 3) / velocity_resolution(i) + 0.5));
  return index;
}
}  // namespace fast_planner
#endif  // PATH_SEARCHING_USV_SEARCH_STATE_HPP_
