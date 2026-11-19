/**
 * @file fairway_chart.hpp
 * @brief 读写与 changjiang.xml 格式一致的航道要素。
 */

#pragma once

#include "clockwise_route.hpp"

#include <filesystem>
#include <usv_map/parser.hpp>

namespace usv_map_tools
{
/**
 * @brief 复制源图并加入标准航道分组，保留水域、陆地、障碍和泊位。
 * @param[in] input 源 XML 路径。
 * @param[in] output 输出 XML 路径，必须与源路径不同。
 * @param[in] chart 对应源图的局部米制海图。
 * @param[in] fairway 待保存的航道。
 * @param[in] edge_clearance_m 保存到属性中的边缘安全净距，单位 m。
 * @throws std::exception 路径无效、源图不可读或保存失败。
 */
void save_fairway_chart(
  const std::filesystem::path & input, const std::filesystem::path & output,
  const usv_map::Chart & chart, const Fairway & fairway, double edge_clearance_m);

/**
 * @brief 恢复本工具导出的航道，用于障碍物放置测试中的固定显示。
 * @param[in] chart 含已保存航道要素的局部米制海图。
 * @return 恢复的航道；净距为未知，不重新进行安全校验。
 * @throws std::exception 缺少必要航道要素或属性无效。
 */
Fairway load_saved_fairway(const usv_map::Chart & chart);
}  // namespace usv_map_tools
