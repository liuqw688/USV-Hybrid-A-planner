/**
 * @file geometry.hpp
 * @brief 航道方向判定、多边形距离及栅格覆盖计算。
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace channel_navigation_manager
{

constexpr int kSameDirectionCost = 0;
constexpr int kOppositeDirectionCost = 20;
constexpr int kOpenWaterCost = 20;
constexpr int kObstacleCost = 100;

/** @brief 地图平面点，x 向东、y 向北，单位 m。 */
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

using Polygon = std::vector<Point2D>;

/** @brief 航道轮廓及 S-57 航向、通行方向属性。 */
struct Lane
{
  int lane_id{-1};
  double orient_deg{0.0};
  Polygon points;
  // S-57 DIRECTION：CW 顺时针、CCW 逆时针，转弯前后保持一致。
  std::string direction;
};

/**
 * @brief 将角度归一化至 [-pi, pi]。
 * @param[in] angle 待归一化角度，单位 rad。
 * @return 归一化角度，单位 rad。
 */
inline double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/**
 * @brief 将 S-57 北零顺时针航向转换为 ROS 东零逆时针偏航角。
 * @param[in] orient_deg S-57 ORIENT 属性，单位 deg。
 * @return 地图偏航角，范围 [-pi, pi]，单位 rad；正北对应 pi/2。
 */
inline double s57OrientationToYaw(const double orient_deg)
{
  return normalizeAngle((90.0 - orient_deg) * M_PI / 180.0);
}

/**
 * @brief 将通行方向属性转换为大写，供不区分大小写的比较使用。
 * @param[in] value 通行方向字符串。
 * @return 大写字符串；不校验是否为 CW 或 CCW。
 */
inline std::string normalizedTrafficDirection(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

/**
 * @brief 校验通行方向是否为 CW 或 CCW，不区分大小写。
 * @param[in] value 待校验字符串。
 * @return 有效返回 true，否则返回 false。
 */
inline bool isTrafficDirection(const std::string & value)
{
  const std::string normalized = normalizedTrafficDirection(value);
  return normalized == "CW" || normalized == "CCW";
}

/**
 * @brief 比较航道通行方向；方向属性缺失时使用小于 pi/2 的航向差判定。
 * @param[in] first 第一条航道。
 * @param[in] second 第二条航道。
 * @return 同向返回 true，否则返回 false。
 */
inline bool sameTrafficDirection(const Lane & first, const Lane & second)
{
  if (!first.direction.empty() && !second.direction.empty()) {
    return normalizedTrafficDirection(first.direction) == normalizedTrafficDirection(second.direction);
  }
  return std::abs(normalizeAngle(
    s57OrientationToYaw(first.orient_deg) - s57OrientationToYaw(second.orient_deg))) < M_PI / 2.0;
}

/**
 * @brief 判断地图行驶方向与 S-57 航道方向的差值是否在容差内。
 * @param[in] travel_yaw ROS 地图偏航角，东为零、逆时针为正，单位 rad。
 * @param[in] orient_deg S-57 航道航向，北为零、顺时针为正，单位 deg。
 * @param[in] tolerance_rad 允许的航向差绝对值，单位 rad。
 * @return 差值不大于容差时返回 true。
 */
inline bool isSameNavigationDirection(
  const double travel_yaw, const double orient_deg, const double tolerance_rad = M_PI / 2.0)
{
  return std::abs(normalizeAngle(s57OrientationToYaw(orient_deg) - travel_yaw)) <= tolerance_rad;
}

/**
 * @brief 根据行驶方向计算航道栅格代价。
 * @param[in] travel_yaw ROS 地图偏航角，单位 rad。
 * @param[in] lane 航道属性。
 * @param[in] tolerance_rad 同向判定的航向差容差，单位 rad。
 * @return 同向返回 kSameDirectionCost，反向返回 kOppositeDirectionCost。
 */
inline int laneValue(
  const double travel_yaw, const Lane & lane, const double tolerance_rad = M_PI / 2.0)
{
  return isSameNavigationDirection(travel_yaw, lane.orient_deg, tolerance_rad) ?
    kSameDirectionCost : kOppositeDirectionCost;
}

/**
 * @brief 使用射线交点奇偶规则判断平面点是否位于多边形内。
 * @param[in] x 查询点东向坐标，单位 m。
 * @param[in] y 查询点北向坐标，单位 m。
 * @param[in] polygon 有序顶点，可省略重复闭合点；少于三个点时返回 false。
 * @return 内部返回 true；边界按射线交点规则处理，不保证统一包含。
 */
inline bool pointInPolygon(const double x, const double y, const Polygon & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }
  bool inside = false;
  Point2D previous = polygon.back();
  for (const auto & current : polygon) {
    if ((previous.y > y) != (current.y > y)) {
      const double x_at_y = previous.x +
        (y - previous.y) * (current.x - previous.x) / (current.y - previous.y);
      if (x < x_at_y) {
        inside = !inside;
      }
    }
    previous = current;
  }
  return inside;
}

/**
 * @brief 计算点到有限线段的最短距离，退化线段按端点处理。
 * @param[in] point 查询点，单位 m。
 * @param[in] first 线段起点，单位 m。
 * @param[in] second 线段终点，单位 m。
 * @return 非负距离，单位 m。
 */
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
  return std::hypot(
    point.x - (first.x + ratio * dx), point.y - (first.y + ratio * dy));
}

/**
 * @brief 计算查询点到多边形区域的最短距离。
 * @param[in] x 查询点东向坐标，单位 m。
 * @param[in] y 查询点北向坐标，单位 m。
 * @param[in] polygon 有序顶点，单位 m。
 * @return 内部为 0，空轮廓为正无穷，其余为到闭合边界的距离，单位 m。
 */
inline double pointPolygonDistance(const double x, const double y, const Polygon & polygon)
{
  if (pointInPolygon(x, y, polygon)) {
    return 0.0;
  }
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const Point2D point{x, y};
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    distance = std::min(
      distance, pointSegmentDistance(point, polygon[i], polygon[(i + 1U) % polygon.size()]));
  }
  return distance;
}

/**
 * @brief 沿航道法线求当前位置所在横断面的归一化偏心量。
 * @param[in] point 航道内的地图平面点，单位 m。
 * @param[in] lane 轮廓及 S-57 方向；方向仅用于地图几何，不改变船舶状态定义。
 * @return 中心为 0、两侧边缘为 1；无有效横断面时返回 0。
 * @note 使用包含查询点的交点区间，适用于凹多边形；不将航段端盖当作侧岸。
 */
inline double laneCenterOffset(const Point2D & point, const Lane & lane)
{
  const double angle = lane.orient_deg * M_PI / 180.0;
  const Point2D tangent{std::sin(angle), std::cos(angle)};
  const Point2D normal{tangent.y, -tangent.x};
  const double along = point.x * tangent.x + point.y * tangent.y;
  const double across = point.x * normal.x + point.y * normal.y;
  std::vector<double> intersections;
  for (std::size_t i = 0; i < lane.points.size(); ++i) {
    const auto & a = lane.points[i];
    const auto & b = lane.points[(i + 1U) % lane.points.size()];
    const double ta = a.x * tangent.x + a.y * tangent.y;
    const double tb = b.x * tangent.x + b.y * tangent.y;
    if ((ta > along) != (tb > along)) {
      const double fraction = (along - ta) / (tb - ta);
      intersections.push_back((a.x + fraction * (b.x - a.x)) * normal.x +
        (a.y + fraction * (b.y - a.y)) * normal.y);
    }
  }
  std::sort(intersections.begin(), intersections.end());
  for (std::size_t i = 0; i + 1U < intersections.size(); i += 2U) {
    const double lo = intersections[i], hi = intersections[i + 1U];
    if (across >= lo - 1.0e-8 && across <= hi + 1.0e-8 && hi - lo > 1.0e-8)
      return std::clamp(std::abs(2.0 * across - lo - hi) / (hi - lo), 0.0, 1.0);
  }
  return 0.0;
}

/**
 * @brief 按栅格行中心扫描多边形，访问交点区间覆盖的栅格。
 * @tparam VisitCell 接收一个行优先线性下标的可调用类型。
 * @param[in] width 栅格列数，须为正数。
 * @param[in] height 栅格行数，须为正数。
 * @param[in] resolution 栅格分辨率，须为正数，单位 m。
 * @param[in] origin_x 地图原点东向坐标，单位 m。
 * @param[in] origin_y 地图原点北向坐标，单位 m。
 * @param[in] polygon 待扫描多边形；少于三个顶点时不访问栅格。
 * @param[in] visit 栅格访问回调；下标为 row * width + column。
 */
template<typename VisitCell>
inline void forEachPolygonCell(
  const int width, const int height, const double resolution,
  const double origin_x, const double origin_y, const Polygon & polygon, VisitCell visit)
{
  if (polygon.size() < 3U || width <= 0 || height <= 0 || resolution <= 0.0) {
    return;
  }
  double min_y = polygon.front().y;
  double max_y = polygon.front().y;
  for (const auto & point : polygon) {
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  const int first_y = std::max(0, static_cast<int>(std::floor((min_y - origin_y) / resolution)));
  const int last_y = std::min(
    height - 1, static_cast<int>(std::floor((max_y - origin_y) / resolution)));
  // 扫描线交点缓冲区跨行复用，避免每行重新分配。
  std::vector<double> intersections;
  intersections.reserve(polygon.size());
  for (int iy = first_y; iy <= last_y; ++iy) {
    const double world_y = origin_y + (static_cast<double>(iy) + 0.5) * resolution;
    intersections.clear();
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
      const int first_x = std::max(
        0, static_cast<int>(std::floor((intersections[i] - origin_x) / resolution)));
      const int last_x = std::min(
        width - 1, static_cast<int>(std::floor((intersections[i + 1U] - origin_x) / resolution)));
      for (int ix = first_x; ix <= last_x; ++ix) {
        visit(static_cast<std::size_t>(iy) * static_cast<std::size_t>(width) + ix);
      }
    }
  }
}

/**
 * @brief 计算多边形覆盖的栅格下标，供静态轮廓缓存复用。
 * @param[in] width 栅格列数。
 * @param[in] height 栅格行数。
 * @param[in] resolution 分辨率，单位 m。
 * @param[in] origin_x 地图原点东向坐标，单位 m。
 * @param[in] origin_y 地图原点北向坐标，单位 m。
 * @param[in] polygon 多边形轮廓，单位 m。
 * @return 行优先线性下标；无有效扫描区域时返回空数组。
 */
inline std::vector<std::size_t> polygonCells(
  const int width, const int height, const double resolution,
  const double origin_x, const double origin_y, const Polygon & polygon)
{
  std::vector<std::size_t> cells;
  forEachPolygonCell(width, height, resolution, origin_x, origin_y, polygon,
    [&cells](std::size_t index) {cells.push_back(index);});
  return cells;
}

/**
 * @brief 将多边形覆盖的栅格写为指定代价。
 * @param[in,out] data 行优先栅格数组，容量须至少为 width * height。
 * @param[in] width 栅格列数。
 * @param[in] height 栅格行数。
 * @param[in] resolution 分辨率，单位 m。
 * @param[in] origin_x 地图原点东向坐标，单位 m。
 * @param[in] origin_y 地图原点北向坐标，单位 m。
 * @param[in] polygon 多边形轮廓，单位 m。
 * @param[in] value 写入的代价，调用方须保证可用 int8_t 表示。
 */
inline void fillPolygon(
  std::vector<int8_t> & data, const int width, const int height, const double resolution,
  const double origin_x, const double origin_y, const Polygon & polygon, const int value)
{
  forEachPolygonCell(width, height, resolution, origin_x, origin_y, polygon,
    [&data, value](std::size_t index) {data[index] = static_cast<int8_t>(value);});
}

}  // namespace channel_navigation_manager
