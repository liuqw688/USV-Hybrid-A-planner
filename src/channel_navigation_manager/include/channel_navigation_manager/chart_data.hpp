/**
 * @file chart_data.hpp
 * @brief 海图 JSON 点集读取；航道与仿真真值共用，感知消息不依赖此格式。
 */
#pragma once
#include <cmath>
#include <stdexcept>
#include <boost/property_tree/ptree.hpp>
#include "channel_navigation_manager/geometry.hpp"

namespace channel_navigation_manager
{
/**
 * @brief 从海图对象的 points 数组读取平面轮廓，跳过不足两个分量的点。
 * @param[in] item 含 points 子节点的海图对象，坐标单位 m。
 * @return 有序轮廓；缺少 points 时返回空数组。
 * @throws std::invalid_argument 坐标包含非有限值。
 * @throws boost::property_tree::ptree_bad_data 坐标无法转换为 double。
 */
inline Polygon readPolygon(const boost::property_tree::ptree & item)
{
  Polygon polygon;
  if (const auto points = item.get_child_optional("points")) {
    polygon.reserve(points->size());
    for (const auto & point : *points) {
      if (point.second.size() < 2U) {continue;}
      auto value = point.second.begin();
      const double x = value->second.get_value<double>();
      const double y = (++value)->second.get_value<double>();
      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument("non-finite chart polygon");
      }
      polygon.push_back({x, y});
    }
  }
  return polygon;
}
}  // namespace channel_navigation_manager
