/// @file parser.hpp
/// @brief 解析 GDAL/S-57 XML 海图导出文件（C++ 版，行为等价于原 Python parser.py）。
///
/// 导出文件里少量空属性被写成 ``<></>``，标准 XML 解析器会拒绝这种拼写，
/// 因此读取后先做一次规范化，再交给 tinyxml2 解析，不依赖 GDAL/Shapely。

/// @date 2026-08-28

#ifndef RIVER_CHART__PARSER_HPP_
#define RIVER_CHART__PARSER_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace river_chart
{

/// @brief 局部东-北平面坐标点，单位米。
struct Point
{
  double x{0.0};
  double y{0.0};
};

/// @brief 一条 WKT 几何：线、多边形环或单点路径的集合。
///
/// ``parts`` 中线要素为折线，面要素为外环及其后的内环，点为单点路径。
struct Geometry
{
  std::string type;                       // WKT 类型名，如 POLYGON
  std::vector<std::vector<Point>> parts;  // 线/环/单点路径
};

/// @brief 单个 S-57 要素及其转换后的几何。
struct ChartFeature
{
  int map_index{0};
  std::string group;         // FeatureGroup acronym
  std::string rcid;
  std::string feature_id;
  std::string name;
  std::string feature_type;  // A/L/P/S/O
  std::vector<Geometry> geometry;
  std::map<std::string, std::string> attributes;
};

/// @brief 解码后的海图，几何已转换为局部米制坐标。
struct Chart
{
  std::vector<ChartFeature> features;
  int map_count{0};
  std::array<double, 4> bounds{0.0, 0.0, 0.0, 0.0};  // min_lon, min_lat, max_lon, max_lat
  std::vector<std::map<std::string, std::string>> metadata;
  double origin_longitude{0.0};
  double origin_latitude{0.0};
  double scale{1.0};
  bool source_is_projected{false};
  std::string source_path;

  /// @brief 按首次出现顺序返回要素分组名。
  std::vector<std::string> groups() const;

  /// @brief 要素数量。
  std::size_t feature_count() const { return features.size(); }

  /// @brief 生成 /chart_metadata 使用的 JSON 文本（按键名排序）。
  std::string metadata_json() const;
};

/// @brief 海图无法解码时抛出。
class ChartParseError : public std::runtime_error
{
public:
  explicit ChartParseError(const std::string & what) : std::runtime_error(what) {}
};

/// @brief 解析 WKT 字符串，支持 POINT/MULTIPOINT/LINESTRING/MULTILINESTRING/POLYGON/MULTIPOLYGON。
Geometry parse_wkt(const std::string & text);

/// @brief 读取 XML 海图并把所有几何转换为局部米制坐标。
///
/// 投影坐标按数值大小自动识别：导出器使用 Web-Mercator 的 X 与取负的 Y，
/// 这里先还原经/纬度，再用局部东-北近似换算为米。
///
/// @param path              XML 文件路径，支持开头的 ``~``。
/// @param origin_longitude 原点经度；非有限值时取 Bounds 中心。
/// @param origin_latitude  原点纬度；非有限值时取 Bounds 中心。
/// @param scale            坐标缩放，1.0 表示米。
Chart load_chart(
  const std::string & path,
  double origin_longitude = std::numeric_limits<double>::quiet_NaN(),
  double origin_latitude = std::numeric_limits<double>::quiet_NaN(),
  double scale = 1.0);

}  // namespace river_chart

#endif  // RIVER_CHART__PARSER_HPP_
