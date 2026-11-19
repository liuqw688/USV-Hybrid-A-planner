/**
 * @file planar_distance_field.hpp
 * @brief 用二维距离场提供障碍绕行的搜索启发。
 */
#ifndef PATH_SEARCHING_PLANAR_DISTANCE_FIELD_HPP_
#define PATH_SEARCHING_PLANAR_DISTANCE_FIELD_HPP_

#include <Eigen/Core>
#include <functional>
#include <vector>

namespace fast_planner
{
class PlanarDistanceField
{
public:
  /** @brief 在指定矩形中由目标反向运行八邻域 Dijkstra，禁止穿越障碍角点。 */
  void build(const Eigen::Vector2d& origin, int width, int height, double resolution,
             const Eigen::Vector2d& goal,
             const std::function<bool(const Eigen::Vector2d&)>& is_free,
             const std::function<double(const Eigen::Vector2d&)>& travel_cost = {});

  /** @brief 双线性插值额外绕行距离；无可用距离场时返回零。单位为 m。 */
  double detour(const Eigen::Vector2d& position) const;

  /** @brief 沿可通行的距离场后继给出绕障方向；空旷区域保持目标直线方向。 */
  Eigen::Vector2d descentDirection(const Eigen::Vector2d& position,
                                   const Eigen::Vector2d& goal) const;

private:
  Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
  double resolution_ = 1.0;
  int width_ = 0;
  int height_ = 0;
  std::vector<double> detours_;
  std::vector<int> successors_;
};
}  // namespace fast_planner
#endif  // PATH_SEARCHING_PLANAR_DISTANCE_FIELD_HPP_
