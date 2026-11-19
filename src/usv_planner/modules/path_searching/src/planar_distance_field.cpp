/**
 * @file planar_distance_field.cpp
 * @brief 二维障碍距离启发的构建和连续查询。
 */
#include "path_searching/planar_distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace fast_planner
{
void PlanarDistanceField::build(
    const Eigen::Vector2d& origin, int width, int height, double resolution,
    const Eigen::Vector2d& goal,
    const std::function<bool(const Eigen::Vector2d&)>& is_free,
    const std::function<double(const Eigen::Vector2d&)>& travel_cost)
{
  if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0)
    throw std::invalid_argument("Planar distance field requires positive dimensions and resolution.");
  origin_ = origin;
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  const double infinity = std::numeric_limits<double>::infinity();
  detours_.assign(width * height, infinity);
  successors_.assign(width * height, -1);
  std::vector<bool> free(width * height);
  std::vector<double> costs(width * height, 1.0);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      const Eigen::Vector2d center = origin + resolution * Eigen::Vector2d(x + 0.5, y + 0.5);
      free[y * width + x] = is_free(center);
      if (free[y * width + x] && travel_cost) {
        const double cost = travel_cost(center);
        if (!std::isfinite(cost) || cost < 1.0)
          throw std::invalid_argument("Planar travel cost must be finite and at least one.");
        costs[y * width + x] = cost;
      }
    }

  const Eigen::Vector2i target = ((goal - origin) / resolution).array().floor().cast<int>();
  if (target.x() < 0 || target.x() >= width || target.y() < 0 || target.y() >= height ||
      !free[target.y() * width + target.x()]) return;

  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  const int goal_index = target.y() * width + target.x();
  detours_[goal_index] = 0.0;
  queue.emplace(0.0, goal_index);
  while (!queue.empty()) {
    const auto entry = queue.top();
    queue.pop();
    if (entry.first > detours_[entry.second]) continue;
    const int x = entry.second % width, y = entry.second / width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = x + dx, ny = y + dy;
        if ((dx == 0 && dy == 0) || nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        const int next = ny * width + nx;
        if (!free[next] || !free[y * width + nx] || !free[ny * width + x]) continue;
        // 每条边按通行代价倍率计长，距离场因此同时引导绕障和低代价航道选择。
        const double distance = entry.first + resolution * std::hypot(dx, dy) *
            0.5 * (costs[entry.second] + costs[next]);
        if (distance >= detours_[next]) continue;
        detours_[next] = distance;
        successors_[next] = entry.second;
        queue.emplace(distance, next);
      }
    }
  }

  // 扣除无障碍八邻域距离，仅保留绕障增量，避免空旷水域被栅格方向拉偏。
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int dx = std::abs(x - target.x()), dy = std::abs(y - target.y());
      const double unobstructed = resolution * (std::max(dx, dy) +
          (std::sqrt(2.0) - 1.0) * std::min(dx, dy));
      detours_[y * width + x] = std::max(0.0, detours_[y * width + x] - unobstructed);
    }
  }
}

double PlanarDistanceField::detour(const Eigen::Vector2d& position) const
{
  const Eigen::Vector2d cell = (position - origin_) / resolution_ - Eigen::Vector2d::Constant(0.5);
  if (cell.x() < 0.0 || cell.y() < 0.0 || cell.x() > width_ - 1 || cell.y() > height_ - 1)
    return 0.0;
  const int x = static_cast<int>(std::floor(cell.x())), y = static_cast<int>(std::floor(cell.y()));
  const Eigen::Vector2d fraction = cell - Eigen::Vector2d(x, y);
  double weighted = 0.0, total = 0.0;
  for (int dy = 0; dy <= 1; ++dy) {
    for (int dx = 0; dx <= 1; ++dx) {
      const int nx = x + dx, ny = y + dy;
      if (nx >= width_ || ny >= height_) continue;
      const double value = detours_[ny * width_ + nx];
      const double weight = (dx ? fraction.x() : 1.0 - fraction.x()) *
                            (dy ? fraction.y() : 1.0 - fraction.y());
      if (!std::isfinite(value)) continue;
      weighted += weight * value;
      total += weight;
    }
  }
  return total > 0.0 ? weighted / total : 0.0;
}

Eigen::Vector2d PlanarDistanceField::descentDirection(
    const Eigen::Vector2d& position, const Eigen::Vector2d& goal) const
{
  Eigen::Vector2d direction = goal - position;
  if (direction.norm() < 1e-9) return direction;
  direction.normalize();
  if (detour(position) <= 1e-9) return direction;
  const Eigen::Vector2i cell = ((position - origin_) / resolution_).array().floor().cast<int>();
  if (cell.x() < 0 || cell.x() >= width_ || cell.y() < 0 || cell.y() >= height_) return direction;
  int next = cell.y() * width_ + cell.x();
  if (successors_[next] < 0) return direction;
  // 沿无碰撞图边前视，避免对障碍内的无效插值做差分而指向障碍。
  for (int step = 0; step < 5 && successors_[next] >= 0; ++step)
    next = successors_[next];
  return origin_ + resolution_ * Eigen::Vector2d(next % width_ + 0.5, next / width_ + 0.5) - position;
}
}  // namespace fast_planner
