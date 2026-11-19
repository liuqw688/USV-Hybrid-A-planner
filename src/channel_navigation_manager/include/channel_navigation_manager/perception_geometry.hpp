/**
 * @file perception_geometry.hpp
 * @brief 水平 RFU 方位/距离转换、圆形禁行区及消息校验；代价端不读取地图真值。
 */
#pragma once

#include <cmath>
#include <stdexcept>
#include <tf2/LinearMath/Transform.hpp>
#include <usv_interfaces/msg/obstacle_detection.hpp>
#include "channel_navigation_manager/geometry.hpp"

namespace channel_navigation_manager
{
/**
 * @brief 计算多边形面积重心，重复闭合点不影响结果。
 * @param[in] polygon 地图平面轮廓，坐标单位 m。
 * @return 面积重心，坐标单位 m。
 * @throws std::invalid_argument 有向双倍面积非有限值或绝对值小于 1e-9。
 */
inline Point2D polygonCentroid(const Polygon & polygon)
{
  double twice_area = 0.0, x = 0.0, y = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1U) % polygon.size()];
    const double cross = a.x * b.y - b.x * a.y;
    twice_area += cross;
    x += (a.x + b.x) * cross;
    y += (a.y + b.y) * cross;
  }
  if (!std::isfinite(twice_area) || std::abs(twice_area) < 1.0e-9) {
    throw std::invalid_argument("degenerate obstacle footprint");
  }
  return {x / (3.0 * twice_area), y / (3.0 * twice_area)};
}

/**
 * @brief 将 RFU 的 +y 船首轴投影到地图水平面，求 ROS 偏航角。
 * @param[in] rotation RFU 传感器帧到地图帧的单位四元数。
 * @return 水平船首方向，东为零、逆时针为正，单位 rad。
 */
inline double bowYaw(const tf2::Quaternion & rotation)
{
  const auto forward = tf2::quatRotate(rotation, tf2::Vector3(0.0, 1.0, 0.0));
  return std::atan2(forward.y(), forward.x());
}

/**
 * @brief 根据仿真真值轮廓生成方位、距离及包围半径观测。
 * @param[in] label 障碍物标签。
 * @param[in] polygon 地图平面轮廓，坐标单位 m。
 * @param[in] map_from_sensor RFU 传感器帧到地图帧的变换。
 * @return 相对船首向右为正的水平观测，方位单位 rad，距离和半径单位 m。
 * @throws std::invalid_argument 轮廓面积退化或面积非有限值。
 */
inline usv_interfaces::msg::ObstacleDetection detectObstacle(
  const std::string & label, const Polygon & polygon, const tf2::Transform & map_from_sensor)
{
  usv_interfaces::msg::ObstacleDetection result;
  result.label = label;
  const auto center = polygonCentroid(polygon);
  const double dx = center.x - map_from_sensor.getOrigin().x();
  const double dy = center.y - map_from_sensor.getOrigin().y();
  result.range_m = std::hypot(dx, dy);
  result.bearing_rad = normalizeAngle(bowYaw(map_from_sensor.getRotation()) - std::atan2(dy, dx));
  for (const auto & point : polygon) {
    result.radius_m = std::max(result.radius_m, std::hypot(point.x - center.x, point.y - center.y));
  }
  return result;
}

/** @brief 地图平面圆形区域，圆心坐标和半径单位均为 m。 */
struct Circle
{
  Point2D center;
  double radius_m;
};

/**
 * @brief 将水平观测转换为地图圆形区域，并叠加安全膨胀半径。
 * @param[in] detection 方位须在 [-pi, pi]，距离非负，半径为正，数值均须有限。
 * @param[in] map_from_sensor RFU 传感器帧到地图帧的变换，仅使用水平船首方向。
 * @param[in] inflation_m 非负且有限的半径增量，单位 m。
 * @return 地图圆形区域，坐标和半径单位 m。
 * @throws std::invalid_argument 输入观测无效，或转换后的圆心、半径非有限值。
 */
inline Circle detectionCircle(
  const usv_interfaces::msg::ObstacleDetection & detection,
  const tf2::Transform & map_from_sensor, const double inflation_m)
{
  if (!std::isfinite(detection.bearing_rad) || std::abs(detection.bearing_rad) > M_PI ||
    !std::isfinite(detection.range_m) || detection.range_m < 0.0 ||
    !std::isfinite(detection.radius_m) || detection.radius_m <= 0.0 ||
    !std::isfinite(inflation_m) || inflation_m < 0.0)
  {
    throw std::invalid_argument("invalid bearing/range/radius/inflation");
  }
  const double yaw = bowYaw(map_from_sensor.getRotation()) - detection.bearing_rad;
  Circle circle{{map_from_sensor.getOrigin().x() + detection.range_m * std::cos(yaw),
      map_from_sensor.getOrigin().y() + detection.range_m * std::sin(yaw)},
    detection.radius_m + inflation_m};
  if (!std::isfinite(circle.center.x) || !std::isfinite(circle.center.y) ||
    !std::isfinite(circle.radius_m))
  {
    throw std::invalid_argument("non-finite obstacle circle");
  }
  return circle;
}

/**
 * @brief 在裁剪后的包围盒内按栅格中心距离计算圆形覆盖区域。
 * @param[in] width 栅格列数，须为正数。
 * @param[in] height 栅格行数，须为正数。
 * @param[in] resolution 正且有限的栅格分辨率，单位 m。
 * @param[in] origin_x 有限的地图原点东向坐标，单位 m。
 * @param[in] origin_y 有限的地图原点北向坐标，单位 m。
 * @param[in] circle 圆心须有限，半径须为正且有限，单位 m。
 * @return 圆内及圆周上的栅格中心下标；输入无效时返回空数组。
 */
inline std::vector<std::size_t> circleCells(
  const int width, const int height, const double resolution,
  const double origin_x, const double origin_y, const Circle & circle)
{
  std::vector<std::size_t> cells;
  if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(circle.center.x) || !std::isfinite(circle.center.y) ||
    !std::isfinite(circle.radius_m) || circle.radius_m <= 0.0) {return cells;}
  const auto first = [resolution](double edge, double origin, int count) {
      return static_cast<int>(std::clamp(std::floor((edge - origin) / resolution), 0.0,
               static_cast<double>(count)));
    };
  const auto last = [resolution](double edge, double origin, int count) {
      return static_cast<int>(std::clamp(std::floor((edge - origin) / resolution), -1.0,
               static_cast<double>(count - 1)));
    };
  const int x0 = first(circle.center.x - circle.radius_m, origin_x, width);
  const int x1 = last(circle.center.x + circle.radius_m, origin_x, width);
  const int y0 = first(circle.center.y - circle.radius_m, origin_y, height);
  const int y1 = last(circle.center.y + circle.radius_m, origin_y, height);
  for (int y = y0; y <= y1; ++y) {
    const double dy = origin_y + (y + 0.5) * resolution - circle.center.y;
    for (int x = x0; x <= x1; ++x) {
      const double dx = origin_x + (x + 0.5) * resolution - circle.center.x;
      if (std::hypot(dx, dy) <= circle.radius_m) {
        cells.push_back(static_cast<std::size_t>(y) * width + x);
      }
    }
  }
  return cells;
}
}  // namespace channel_navigation_manager
