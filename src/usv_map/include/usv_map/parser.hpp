/**
 * @file parser.hpp
 * @brief 解析 S-57 XML 海图并转换为局部东、北米制坐标。
 */

#ifndef USV_MAP__PARSER_HPP_
#define USV_MAP__PARSER_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace usv_map
{

/** @brief 局部东-北平面坐标点，单位米。 */
struct Point
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief 一条 WKT 几何：线、多边形环或单点路径的集合。
 *
 * parts 中线要素为折线，面要素为外环及其后的内环，点为单点路径。
 */
struct Geometry
{
  std::string type;                       // WKT 类型名，如 POLYGON
  std::vector<std::vector<Point>> parts;  // 线/环/单点路径
  // 每个多边形外环在 parts 中的下标；其后的环是该多边形的孔洞。
  std::vector<std::size_t> polygon_starts{};
};

/** @brief 单个 S-57 要素及其转换后的几何。 */
struct ChartFeature
{
  int map_index{0};
  std::string group;         // S-57 要素分组缩写
  std::string rcid;
  std::string feature_id;
  std::string name;
  std::string feature_type;  // 源要素类型代码：A/L/P/S/O。
  std::vector<Geometry> geometry;
  std::map<std::string, std::string> attributes;
};

/** @brief 解码后的海图，几何已转换为局部米制坐标。 */
struct Chart
{
  std::vector<ChartFeature> features;
  int map_count{0};
  std::array<double, 4> bounds{0.0, 0.0, 0.0, 0.0};  // 源图地理边界：最小经度、最小纬度、最大经度、最大纬度，单位 °。
  std::vector<std::map<std::string, std::string>> metadata;
  double origin_longitude{0.0};
  double origin_latitude{0.0};
  bool source_is_projected{false};
  std::string source_path;
  // 显示允许跳过损坏要素；航线生成必须拒绝缺失的水域/陆地几何，
  // 避免将无法解析的区域误当成可通行水域。
  bool has_unreadable_navigation_geometry{false};

  /** @brief 按首次出现顺序返回要素分组名。 */
  std::vector<std::string> groups() const;

  /** @brief 要素数量。 */
  std::size_t feature_count() const { return features.size(); }

  /** @brief 生成 /chart_metadata 使用的 JSON 文本（按键名排序）。 */
  std::string metadata_json() const;
};

/** @brief 海图无法解码时抛出。 */
class ChartParseError : public std::runtime_error
{
public:
  explicit ChartParseError(const std::string & what) : std::runtime_error(what) {}
};

/**
 * @brief 解析点、多点、线、多线、多边形及多多边形 WKT 几何。
 * @param[in] text WKT 文本；支持 EMPTY 以及 Z、M、ZM 后缀，额外维度不保留。
 * @return 保留源坐标的二维几何；多边形记录外环下标和后续内环。
 * @throws ChartParseError 类型不支持或几何文本无法解析。
 */
Geometry parse_wkt(const std::string & text);

/**
 * @brief 读取 XML 海图并把所有几何转换为局部米制坐标。
 *
 * 投影坐标按数值大小自动识别：导出器使用 Web-Mercator 的 X 与取负的 Y，
 * 这里先还原经/纬度，再用局部东-北近似换算为米。
 *
 * @param[in] path XML 文件路径，支持开头的 ~。
 * @param[in] origin_longitude 原点经度，单位 °；非有限值时取 Bounds 中心。
 * @param[in] origin_latitude 原点纬度，单位 °；非有限值时取 Bounds 中心。
 * @return 几何已转换为局部东、北坐标的海图，位置单位 m。
 * @throws ChartParseError 文件不可读、XML 无效或缺少有效的地图范围。
 * @note 单个几何解析失败时跳过；水域或陆地几何损坏会设置导航几何异常标志。
 */
Chart load_chart(
  const std::string & path,
  double origin_longitude = std::numeric_limits<double>::quiet_NaN(),
  double origin_latitude = std::numeric_limits<double>::quiet_NaN());

}  // namespace usv_map

#endif  // USV_MAP__PARSER_HPP_
