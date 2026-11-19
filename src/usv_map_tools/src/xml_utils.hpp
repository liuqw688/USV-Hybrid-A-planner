/**
 * @file xml_utils.hpp
 * @brief 地图编辑工具共用的 XML 文本、数值与原子保存。
 */

#pragma once

#include <algorithm>
#include <array>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <tinyxml2.h>

namespace usv_map_tools::xml
{

/**
 * @brief 将有限数值格式化为固定小数文本。
 * @param[in] value 待转换数值。
 * @param[in] precision 小数位数，默认 10 位。
 * @return 使用句点作为小数点的文本，不受系统区域设置影响。
 */
inline std::string number(double value, int precision = 10)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

/**
 * @brief 在父节点下追加带文本内容的 XML 子元素。
 * @param[in,out] parent 父元素，不能为空；新元素追加为其末尾子节点。
 * @param[in] name 子元素名称。
 * @param[in] value 子元素文本。
 * @return 新建的子元素。
 */
inline tinyxml2::XMLElement * text(
  tinyxml2::XMLElement * parent, const char * name, const std::string & value)
{
  auto * child = parent->GetDocument()->NewElement(name);
  child->SetText(value.c_str());
  parent->InsertEndChild(child);
  return child;
}

/**
 * @brief 追加多边形几何、WKT 轮廓及源坐标包围盒。
 * @param[in,out] feature 目标海图要素，追加几何节点。
 * @param[in] points 闭合轮廓采样点。
 * @param[in] project 调用方提供的坐标投影函数。
 * @param[in] precision 坐标小数位数，默认 10 位。
 */
template<typename Points, typename Project>
void polygon(tinyxml2::XMLElement * feature, const Points & points, Project project,
  int precision = 10)
{
  const double infinity = std::numeric_limits<double>::infinity();
  std::array<double, 4> bounds{infinity, infinity, -infinity, -infinity};
  std::string wkt = "POLYGON ((";
  bool first = true;
  for (const auto & point : points) {
    const auto p = project(point);
    wkt += (first ? "" : ", ") + number(p.x, precision) + " " + number(p.y, precision);
    first = false;
    bounds[0] = std::min(bounds[0], p.x);
    bounds[1] = std::min(bounds[1], p.y);
    bounds[2] = std::max(bounds[2], p.x);
    bounds[3] = std::max(bounds[3], p.y);
  }
  auto * geometry = feature->GetDocument()->NewElement("Geometry");
  feature->InsertEndChild(geometry);
  text(geometry, "BoundingBox", number(bounds[0], precision) + "," +
    number(bounds[1], precision) + "," + number(bounds[2], precision) + "," +
    number(bounds[3], precision));
  auto * level = feature->GetDocument()->NewElement("Geometries");
  geometry->InsertEndChild(level);
  level->SetAttribute("level", "0");
  text(level, "Polygon", wkt + "))");
}

/**
 * @brief 通过临时文件原子替换 XML 输出。
 * @param[in] document 待保存文档。
 * @param[in] output 目标文件路径。
 * @throws std::runtime_error 写入临时文件失败；原输出文件保持不变。
 * @throws std::filesystem::filesystem_error 创建目录或替换文件失败。
 * @details 同一输出文件应由单个编辑器写入，避免临时文件名冲突。
 */
inline void save_atomic(tinyxml2::XMLDocument & document, const std::filesystem::path & output)
{
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  auto temporary = output;
  temporary += ".tmp";
  try {
    if (document.SaveFile(temporary.c_str()) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("cannot write XML: " + temporary.string());
    }
    std::filesystem::rename(temporary, output);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}

}  // namespace usv_map_tools::xml
