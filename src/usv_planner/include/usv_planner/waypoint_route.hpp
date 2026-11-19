/**
 * @file waypoint_route.hpp
 * @brief 航点折线的单调进度与局部前视目标，不依赖 ROS。
 */
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace usv_planner {

/** @brief 沿输入顺序推进航点折线；闭环终点不得被误认为当前进度。 */
class WaypointRoute {
public:
  struct Target {
    Eigen::Vector3d pose;  ///< [东向位置(m), 北向位置(m), 北零西正航向(rad)]。
    bool terminal;        ///< 前视目标已覆盖整条任务的最后一个航点。
  };

  /**
   * @brief 替换任务；相邻重复点合并，非相邻重复点保留以表达闭环。
   * @param[in] points 地图 ENU 平面航点，至少包含两个不同的相邻点。
   */
  void reset(const std::vector<Eigen::Vector2d>& points)
  {
    std::vector<Eigen::Vector2d> cleaned;
    for (const auto& point : points) {
      if (!point.allFinite()) throw std::invalid_argument("Waypoint must be finite.");
      if (cleaned.empty() || (point - cleaned.back()).norm() > 1e-3) cleaned.push_back(point);
    }
    if (cleaned.size() < 2) throw std::invalid_argument("Route needs two distinct points.");
    points_ = std::move(cleaned);
    lengths_.clear();
    progress_ = 0.0;
  }

  /**
   * @brief 用真实船位推进折线投影；任务开始时将船位到首航点作为入航段。
   * @param[in] position 东、北位置，单位 m。
   * @note 投影只向前搜索，范围不超过相邻船位位移的两倍，防止闭环及交叉处跳段。
   */
  void update(const Eigen::Vector2d& position)
  {
    if (!position.allFinite() || points_.size() < 2) {
      throw std::invalid_argument("Valid route and position are required.");
    }
    if (lengths_.empty()) {
      if ((position - points_.front()).norm() > 1e-3) points_.insert(points_.begin(), position);
      lengths_.push_back(0.0);
      for (std::size_t i = 1; i < points_.size(); ++i) {
        lengths_.push_back(lengths_.back() + (points_[i] - points_[i - 1]).norm());
      }
      last_position_ = position;
      return;
    }

    const double limit = std::min(lengths_.back(), progress_ + 2.0 * (position - last_position_).norm());
    last_position_ = position;
    double best_progress = progress_;
    double best_distance = (sample(progress_).pose.head<2>() - position).squaredNorm();
    for (std::size_t i = segment(progress_); i + 1 < points_.size() && lengths_[i] < limit; ++i) {
      const Eigen::Vector2d delta = points_[i + 1] - points_[i];
      const double length = lengths_[i + 1] - lengths_[i];
      const double lower = std::max(0.0, (progress_ - lengths_[i]) / length);
      const double upper = std::min(1.0, (limit - lengths_[i]) / length);
      const double ratio = std::clamp((position - points_[i]).dot(delta) / delta.squaredNorm(), lower, upper);
      const double distance = (points_[i] + ratio * delta - position).squaredNorm();
      if (distance < best_distance) {
        best_distance = distance;
        best_progress = lengths_[i] + ratio * length;
      }
    }
    progress_ = best_progress;
  }

  /**
   * @brief 沿折线前视；横向偏离占用前视距离，目标始终位于船周围的感知圆内。
   * @param[in] position 当前船位，单位 m。
   * @param[in] range 感知半径及前视弧长上限，单位 m，必须为正。
   * @return 局部目标位姿及是否为最终目标。剩余路线不足前视距离时返回任务末点。
   */
  Target target(const Eigen::Vector2d& position, double range) const
  {
    if (!position.allFinite() || lengths_.empty() || !std::isfinite(range) || range <= 0.0) {
      throw std::invalid_argument("Initialize route and use a finite positive range.");
    }
    const Target projected = sample(progress_);
    const double distance = (projected.pose.head<2>() - position).norm();
    if (distance > range) {
      const Eigen::Vector2d direction = (projected.pose.head<2>() - position) / distance;
      return {{position.x() + range * direction.x(), position.y() + range * direction.y(),
               std::atan2(-direction.x(), direction.y())}, false};
    }
    return sample(std::min(lengths_.back(), progress_ + range - distance));
  }

  /**
   * @brief 选择无碰撞的前视目标，受阻时从该障碍段前方开始沿航路检索。
   * @param[in] position 当前东、北位置，单位 m。
   * @param[in] range 感知半径及前视弧长上限，单位 m。
   * @param[in] is_safe 位姿安全检查；调用方须以 x、y 查询地图，不得将航向作为高度。
   * @param[in] step 障碍段与候选点的弧长采样间隔，单位 m。
   * @param[in] lead 受阻段入口前的检索起点距离，单位 m，默认 10 m。
   * @return 安全目标；范围内无可推进候选点时返回空值。
   * @note 前沿安全时直接使用前沿，允许搜索绕过中间障碍；修正目标不改变航路进度。
   *       障碍后方尚未进入前视范围时先使用障碍前目标，且不将其视为任务终点。
   */
  template <typename SafetyCheck>
  std::optional<Target> safeTarget(const Eigen::Vector2d& position, double range,
                                  SafetyCheck is_safe, double step = 0.5,
                                  double lead = 10.0) const
  {
    if (!std::isfinite(step) || step <= 0.0 || !std::isfinite(lead) || lead < 0.0) {
      throw std::invalid_argument("Search step must be positive and lead nonnegative.");
    }
    const Target frontier = target(position, range);
    if (is_safe(frontier.pose)) return frontier;

    const double offset = (sample(progress_).pose.head<2>() - position).norm();
    if (offset > range) return std::nullopt;
    const double limit = std::min(lengths_.back(), progress_ + range - offset);
    double entry = limit;
    while (entry > progress_ && !is_safe(sample(entry).pose)) {
      entry = std::max(progress_, entry - step);
    }

    const double begin = std::max(progress_, entry - lead);
    const int count = static_cast<int>(std::ceil((limit - begin) / step));
    for (int i = 0; i <= count; ++i) {
      const double distance = std::min(limit, begin + i * step);
      if (distance <= progress_ + 1e-3) continue;
      const Target candidate = sample(distance);
      if (is_safe(candidate.pose)) return candidate;
    }
    return std::nullopt;
  }

  double progress() const { return progress_; }

private:
  /** @brief 找到弧长所在的出航段；末点使用最后一个入航段。 */
  std::size_t segment(double distance) const
  {
    return std::min(points_.size() - 2, static_cast<std::size_t>(
      std::upper_bound(lengths_.begin(), lengths_.end(), distance) - lengths_.begin() - 1));
  }

  Target sample(double distance) const
  {
    const std::size_t i = segment(distance);
    const Eigen::Vector2d delta = points_[i + 1] - points_[i];
    const double ratio = (distance - lengths_[i]) / (lengths_[i + 1] - lengths_[i]);
    const Eigen::Vector2d position = points_[i] + ratio * delta;
    return {{position.x(), position.y(), std::atan2(-delta.x(), delta.y())}, distance >= lengths_.back()};
  }

  std::vector<Eigen::Vector2d> points_;
  std::vector<double> lengths_;
  Eigen::Vector2d last_position_{Eigen::Vector2d::Zero()};
  double progress_{0.0};  ///< 从任务入航段起点量起的已通过弧长，单位 m。
};

}  // namespace usv_planner
