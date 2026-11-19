/**
 * @file route_water.hpp
 * @brief 基于 GEOS 的水域、陆地和航道净距检查。
 */

#pragma once

#include "clockwise_route.hpp"

#include <memory>
#include <vector>

#include <usv_map/parser.hpp>

namespace usv_map_tools
{
/**
 * @brief 基于 map 米制矢量几何检查水域覆盖和净距。
 * @details DEPARE 表示已知水域，LNDARE 陆地优先；包含岛屿、内环及手工障碍物。
 */
class RouteWater
{
public:
  /**
   * @brief 构建收缩水域和膨胀障碍物的几何索引。
   * @param[in] chart 已解析的海图。
   * @param[in] clearance_m 安全裕量，必须为有限非负数，单位 m。
   * @throws std::exception 海图几何无效、裕量无效或不存在满足裕量的水域。
   */
  RouteWater(const usv_map::Chart & chart, double clearance_m);
  ~RouteWater();
  /**
   * @brief 计算折线位于安全水域之外的长度。
   * @param[in] line 至少含两个点的局部米制折线。
   * @return 完全覆盖时为零；否则至少为 1e-6 m，以区分零长度相切越界。
   * @throws std::exception 输入折线无效或几何运算失败。
   */
  double unsafe_length(const std::vector<RoutePoint> & line) const;
  /**
   * @brief 计算折线到岸线及陆地障碍的最小距离。
   * @param[in] line 至少含两个点的局部米制折线。
   * @return 净距，单位 m；离开已知水域或接触陆地障碍时返回 -1。
   * @throws std::exception 输入折线无效或几何运算失败。
   */
  double clearance(const std::vector<RoutePoint> & line) const;
  /**
   * @brief 计算完整环形航道到岸线及陆地障碍的最小距离。
   * @param[in] outer 航道外环，局部坐标单位 m。
   * @param[in] inner 航道内环，局部坐标单位 m；首尾未闭合时自动补齐。
   * @return 净距，单位 m；覆盖陆地或超出已知水域时返回 -1。
   * @throws std::exception 边界交叉、环形区域无效或几何运算失败。
   */
  double corridor_clearance(
    const std::vector<RoutePoint> & outer, const std::vector<RoutePoint> & inner) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace usv_map_tools
