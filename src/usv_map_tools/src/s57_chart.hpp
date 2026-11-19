/**
 * @file s57_chart.hpp
 * @brief 将 XML 海图转换为 S-57 3.1 基础单元。
 */

#ifndef USV_MAP_TOOLS__S57_CHART_HPP_
#define USV_MAP_TOOLS__S57_CHART_HPP_

#include <map>
#include <string>

#include "usv_map/parser.hpp"

namespace usv_map_tools
{

/** @brief S-57 输出统计及被修复的无效源面要素数量。 */
struct S57ExportResult
{
  std::map<std::string, int> feature_counts;
  int invalid_source_areas{0};
};

/**
 * @brief 导出 S-57 3.1 基础单元并回读验证生成文件。
 * @param[in] chart 已解析的局部米制海图；源几何不会被修改。
 * @param[in] output 输出文件路径。
 * @return 各要素数量及无效源面统计。
 * @throws std::exception 输出路径不可写、GDAL 导出失败或回读校验失败。
 * @details 自定义 XML 属性写入 INFORM；输出覆盖范围由要素空间范围生成。
 */
S57ExportResult export_s57(const usv_map::Chart & chart, const std::string & output);

}  // namespace usv_map_tools

#endif  // USV_MAP_TOOLS__S57_CHART_HPP_
