/**
 * @file search_environment.hpp
 * @brief 动力学搜索使用的二维地图查询接口。
 */

#ifndef PATH_SEARCHING__SEARCH_ENVIRONMENT_HPP_
#define PATH_SEARCHING__SEARCH_ENVIRONMENT_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace fast_planner
{

/** @brief 将 OccupancyGrid 通行代价换算为无量纲路程倍率。 */
inline double mapTravelCostMultiplier(const double cost)
{
  return 1.0 + std::max(0.0, std::isfinite(cost) ? cost : 0.0) / 20.0;
}

/** @brief 为 A* 提供地图范围、占用和距离查询的适配接口。 */
class SearchEnvironment
{
public:
  virtual ~SearchEnvironment() = default;

  virtual bool isReady() const = 0;
  virtual bool usesGridMap() const = 0;
  virtual void getRegion(Eigen::Vector3d& origin, Eigen::Vector3d& size) = 0;
  virtual double getResolution() = 0;
  virtual int getInflateOccupancy(const Eigen::Vector3d& position) = 0;
  virtual double getDistance(const Eigen::Vector3d& position) = 0;
  /** @brief 返回到原始障碍的保守距离，单位 m。 */
  virtual double getObstacleDistance(const Eigen::Vector3d& position) { return getDistance(position); }
  virtual double getTravelCost(const Eigen::Vector3d&) { return 0.0; }
  /** @brief 返回与通行代价一致的空间梯度；未提供软代价的地图保持为零。 */
  virtual void evaluateTravelCostWithGradient(const Eigen::Vector3d& position,
      double& cost, Eigen::Vector3d& gradient)
  {
    cost = getTravelCost(position);
    gradient.setZero();
  }
  virtual double evaluateCoarseDistance(const Eigen::Vector3d& position, double time) = 0;
  virtual void evaluateDistanceWithGradient(const Eigen::Vector3d& position, double time,
                                            double& distance, Eigen::Vector3d& gradient) = 0;
};

}  // namespace fast_planner

#endif  // PATH_SEARCHING__SEARCH_ENVIRONMENT_HPP_
