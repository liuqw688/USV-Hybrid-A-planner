#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace channel_navigation_manager
{

constexpr int kMainLane = 0;
constexpr int kOutsideChannel = 45;
constexpr int kOppositeLane = 75;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Lane
{
  int lane_id{-1};
  double orient_deg{0.0};
  std::vector<Point2D> points;
};

// [功能与联系] 归一化角度，消除跨±pi的跳变；方向比较、运动积分和COLREG约束共同使用。
inline double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// [功能与联系] 将真北为0顺时针的S-57 ORIENT换算为ROS东为0逆时针yaw；避免海图与运动学方向定义混淆。
inline double s57OrientationToYaw(const double orient_deg)
{
  return normalizeAngle((90.0 - orient_deg) * M_PI / 180.0);
}

// [功能与联系] 以方向余弦是否非负判断同向候选（约90度以内）；当前连续算法仅用它筛选种子航段。
inline bool isSameNavigationDirection(const double course_yaw, const double orient_deg)
{
  return std::cos(normalizeAngle(s57OrientationToYaw(orient_deg) - course_yaw)) >= 0.0;
}

// [功能与联系] 直接方向分类的辅助函数；当前主流程采用continuousMainLaneMask，不应误认为所有航段仍独立比较船头角。
inline int laneValue(const double course_yaw, const Lane & lane)
{
  return isSameNavigationDirection(course_yaw, lane.orient_deg) ? kMainLane : kOppositeLane;
}

inline bool pointInPolygon(const double x, const double y, const std::vector<Point2D> & points);

// [功能与联系] 计算点到线段的最近距离；支撑多边形距离和邻接传播，不是动态船碰撞检查。
inline double pointSegmentDistance(
  const Point2D & point, const Point2D & first, const Point2D & second)
{
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1.0e-12) {
    return std::hypot(point.x - first.x, point.y - first.y);
  }
  const double ratio = std::clamp(
    ((point.x - first.x) * dx + (point.y - first.y) * dy) / length_squared, 0.0, 1.0);
  return std::hypot(point.x - (first.x + ratio * dx), point.y - (first.y + ratio * dy));
}

// [功能与联系] 计算点到航段多边形的距离；种子选择用它寻找本船附近同向航段。
inline double pointPolygonDistance(
  const double x, const double y, const std::vector<Point2D> & polygon)
{
  if (pointInPolygon(x, y, polygon)) {return 0.0;}
  if (polygon.empty()) {return std::numeric_limits<double>::infinity();}
  const Point2D point{x, y};
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    distance = std::min(
      distance, pointSegmentDistance(point, polygon[index], polygon[(index + 1U) % polygon.size()]));
  }
  return distance;
}

// [功能与联系] 估计两个航段多边形的空间距离；与连续方向阈值联合决定传播邻接。
inline double polygonDistance(
  const std::vector<Point2D> & first, const std::vector<Point2D> & second)
{
  if (first.empty() || second.empty()) {return std::numeric_limits<double>::infinity();}
  double distance = std::numeric_limits<double>::infinity();
  for (const auto & point : first) {
    distance = std::min(distance, pointPolygonDistance(point.x, point.y, second));
  }
  for (const auto & point : second) {
    distance = std::min(distance, pointPolygonDistance(point.x, point.y, first));
  }
  return distance;
}

// Follow locally continuous S-57 ORIENT values from the lane nearest the
// vessel. Comparing every polygon with one initial heading fails after a large
// accumulated bend; local propagation preserves the same traffic stream.
// [功能与联系] 从最近同向种子以队列沿空间邻接和局部ORIENT连续性传播主航道；避免累计大弯时被初始船向误分类。
inline std::vector<bool> continuousMainLaneMask(
  const std::vector<Lane> & lanes, const double policy_yaw,
  const double own_x, const double own_y, const double adjacency_distance,
  const double continuity_angle)
{
  std::vector<bool> main(lanes.size(), false);
  if (lanes.empty()) {return main;}

  std::size_t seed = lanes.size();
  double seed_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < lanes.size(); ++index) {
    if (!isSameNavigationDirection(policy_yaw, lanes[index].orient_deg)) {continue;}
    const double distance = pointPolygonDistance(own_x, own_y, lanes[index].points);
    if (distance < seed_distance) {
      seed = index;
      seed_distance = distance;
    }
  }
  if (seed == lanes.size()) {
    for (std::size_t index = 0; index < lanes.size(); ++index) {
      const double distance = pointPolygonDistance(own_x, own_y, lanes[index].points);
      if (distance < seed_distance) {
        seed = index;
        seed_distance = distance;
      }
    }
  }

  std::queue<std::size_t> pending;
  main[seed] = true;
  pending.push(seed);
  while (!pending.empty()) {
    const std::size_t current = pending.front();
    pending.pop();
    const double current_yaw = s57OrientationToYaw(lanes[current].orient_deg);
    for (std::size_t candidate = 0; candidate < lanes.size(); ++candidate) {
      if (main[candidate] || candidate == current) {continue;}
      const double candidate_yaw = s57OrientationToYaw(lanes[candidate].orient_deg);
      if (std::abs(normalizeAngle(candidate_yaw - current_yaw)) > continuity_angle) {continue;}
      if (polygonDistance(lanes[current].points, lanes[candidate].points) > adjacency_distance) {
        continue;
      }
      main[candidate] = true;
      pending.push(candidate);
    }
  }
  return main;
}

// [功能与联系] 由两点求方位与中心距离；反向任务判定调用它，要求坐标系一致。
inline std::pair<double, double> bearingAndDistance(
  const double own_x, const double own_y, const double goal_x, const double goal_y)
{
  const double dx = goal_x - own_x;
  const double dy = goal_y - own_y;
  return {std::atan2(dy, dx), std::hypot(dx, dy)};
}

// [功能与联系] 联合目标方位差及最小目标距离判断反向任务；正常左右前方目标不会直接翻转全河道角色。
inline bool isReverseTask(
  const double course_yaw, const double own_x, const double own_y,
  const double goal_x, const double goal_y, const double enter_angle,
  const double minimum_distance)
{
  const auto [bearing, distance] = bearingAndDistance(own_x, own_y, goal_x, goal_y);
  return distance >= minimum_distance &&
         std::abs(normalizeAngle(bearing - course_yaw)) >= enter_angle;
}

// [功能与联系] 射线交点法判断点在多边形内部；用于区域状态和距离计算。
inline bool pointInPolygon(const double x, const double y, const std::vector<Point2D> & points)
{
  if (points.size() < 3U) {
    return false;
  }
  bool inside = false;
  Point2D previous = points.back();
  for (const auto & current : points) {
    if ((previous.y > y) != (current.y > y)) {
      const double crossing_x = previous.x +
        (y - previous.y) * (current.x - previous.x) / (current.y - previous.y);
      if (x < crossing_x) {
        inside = !inside;
      }
    }
    previous = current;
  }
  return inside;
}

// [功能与联系] 扫描线填充航道语义区域；仅绘制区域类别，不覆盖物理陆地或独立障碍层。
inline void fillPolygon(
  std::vector<int8_t> & data, const int width, const int height, const double resolution,
  const double origin_x, const double origin_y, std::vector<Point2D> polygon, const int value)
{
  if (polygon.size() > 1U && polygon.front().x == polygon.back().x &&
    polygon.front().y == polygon.back().y)
  {
    polygon.pop_back();
  }
  if (polygon.size() < 3U) {
    return;
  }
  double min_y = polygon.front().y;
  double max_y = polygon.front().y;
  for (const auto & point : polygon) {
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  const int low_y = std::max(0, static_cast<int>(std::floor((min_y - origin_y) / resolution)));
  const int high_y = std::min(
    height - 1, static_cast<int>(std::floor((max_y - origin_y) / resolution)));
  for (int iy = low_y; iy <= high_y; ++iy) {
    const double world_y = origin_y + (static_cast<double>(iy) + 0.5) * resolution;
    std::vector<double> intersections;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const auto & first = polygon[i];
      const auto & second = polygon[(i + 1U) % polygon.size()];
      if ((first.y > world_y) != (second.y > world_y)) {
        intersections.push_back(
          first.x + (world_y - first.y) * (second.x - first.x) / (second.y - first.y));
      }
    }
    std::sort(intersections.begin(), intersections.end());
    for (std::size_t i = 0; i + 1U < intersections.size(); i += 2U) {
      const int low_x = std::max(
        0, static_cast<int>(std::floor((intersections[i] - origin_x) / resolution)));
      const int high_x = std::min(
        width - 1,
        static_cast<int>(std::floor((intersections[i + 1U] - origin_x) / resolution)));
      for (int ix = low_x; ix <= high_x; ++ix) {
        const std::size_t index = static_cast<std::size_t>(iy * width + ix);
        if (value == kMainLane || data[index] != kMainLane) {
          data[index] = static_cast<int8_t>(value);
        }
      }
    }
  }
}

// 对主航道联合区域做两遍 chamfer 距离变换。中心代价低、边沿代价高，
// 但所有 0..edge_value 栅格仍是可通行主航道，不改变安全层的硬碰撞判定。
// [功能与联系] 在主航道范围内形成0..边缘值的软代价梯度；保留全部主航道可通行，规划器用权重偏向中心。
inline void applyMainLaneCenterGradient(
  std::vector<int8_t> & data, const int width, const int height, const double resolution,
  const int requested_edge_value, const double clearance)
{
  const int edge_value = std::clamp(requested_edge_value, 0, kOutsideChannel - 1);
  if (edge_value == 0 || clearance <= 0.0 || data.empty()) {
    return;
  }
  const float infinity = static_cast<float>(width + height + 1);
  const float diagonal = std::sqrt(2.0F);
  std::vector<float> distance(data.size(), 0.0F);
  for (std::size_t i = 0; i < data.size(); ++i) {
    if (data[i] == kMainLane) {
      distance[i] = infinity;
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;
      if (data[index] != kMainLane) {continue;}
      float best = distance[index];
      if (x > 0) {best = std::min(best, distance[index - 1] + 1.0F);}
      if (y > 0) {
        best = std::min(best, distance[index - width] + 1.0F);
        if (x > 0) {best = std::min(best, distance[index - width - 1] + diagonal);}
        if (x + 1 < width) {best = std::min(best, distance[index - width + 1] + diagonal);}
      }
      distance[index] = best;
    }
  }
  for (int y = height - 1; y >= 0; --y) {
    for (int x = width - 1; x >= 0; --x) {
      const int index = y * width + x;
      if (data[index] != kMainLane) {continue;}
      float best = distance[index];
      if (x + 1 < width) {best = std::min(best, distance[index + 1] + 1.0F);}
      if (y + 1 < height) {
        best = std::min(best, distance[index + width] + 1.0F);
        if (x > 0) {best = std::min(best, distance[index + width - 1] + diagonal);}
        if (x + 1 < width) {best = std::min(best, distance[index + width + 1] + diagonal);}
      }
      distance[index] = best;
    }
  }
  // 不同河段宽度不同。若直接除以固定 clearance，窄航道即使位于几何中心
  // 也得不到 0。这里对每个连续主航道求实际最大内切距离，再按该宽度归一化。
  std::vector<int> component(data.size(), -1);
  std::vector<float> component_max;
  const int dx[4] = {-1, 1, 0, 0};
  const int dy[4] = {0, 0, -1, 1};
  for (int seed = 0; seed < width * height; ++seed) {
    if (data[seed] != kMainLane || component[seed] >= 0) {continue;}
    const int id = static_cast<int>(component_max.size());
    component_max.push_back(0.0F);
    std::queue<int> pending;
    pending.push(seed);
    component[seed] = id;
    while (!pending.empty()) {
      const int index = pending.front();
      pending.pop();
      component_max[id] = std::max(component_max[id], distance[index]);
      const int x = index % width;
      const int y = index / width;
      for (int direction = 0; direction < 4; ++direction) {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {continue;}
        const int next = ny * width + nx;
        if (data[next] == kMainLane && component[next] < 0) {
          component[next] = id;
          pending.push(next);
        }
      }
    }
  }
  const double clearance_cells = clearance / resolution;
  for (std::size_t i = 0; i < data.size(); ++i) {
    if (data[i] != kMainLane) {continue;}
    const int id = component[i];
    const double normalizer = std::max(
      1.0, std::min(clearance_cells, static_cast<double>(component_max[id])));
    const double ratio = std::max(0.0, 1.0 - static_cast<double>(distance[i]) / normalizer);
    data[i] = static_cast<int8_t>(std::lround(static_cast<double>(edge_value) * ratio));
  }
}

}  // namespace channel_navigation_manager
