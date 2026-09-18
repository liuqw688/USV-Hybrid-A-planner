/// @file parser.cpp
/// @brief S-57 XML 海图解析实现。
///

/// @date 2026-08-28

#include "river_chart/parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <tinyxml2.h>

#include "json_util.hpp"

namespace river_chart
{
namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kEarthRadius = 6378137.0;
constexpr double kMetersPerDegree = 111319.49079327358;

/// @brief 去掉首尾空白。
std::string strip(const std::string & value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  return first < last ? std::string(first, last) : std::string();
}

/// @brief 转大写。
std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

/// @brief 模仿 Python ``float()`` 的严格数值解析。
double parse_double(const std::string & text)
{
  const char * begin = text.c_str();
  char * end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin) {
    throw std::invalid_argument("not a number");
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end)) != 0) {
    ++end;
  }
  if (*end != '\0') {
    throw std::invalid_argument("trailing characters");
  }
  return value;
}

/// @brief 把导出器写出的空标签 ``<></>`` 规范化为 ``<Empty></Empty>``。
std::string sanitize_empty_tags(const std::string & raw)
{
  std::string out;
  out.reserve(raw.size());
  const auto skip_space = [&raw](std::size_t pos) {
    while (pos < raw.size() && std::isspace(static_cast<unsigned char>(raw[pos])) != 0) {
      ++pos;
    }
    return pos;
  };
  std::size_t i = 0;
  while (i < raw.size()) {
    if (raw[i] != '<') {
      out += raw[i++];
      continue;
    }
    std::size_t p = skip_space(i + 1);                       // <\s*
    if (p >= raw.size() || raw[p] != '>') {                  // >
      out += raw[i++];
      continue;
    }
    p = skip_space(p + 1);                                   // \s*
    if (p + 1 >= raw.size() || raw[p] != '<' || raw[p + 1] != '/') {  // </
      out += raw[i++];
      continue;
    }
    p = skip_space(p + 2);                                   // \s*
    if (p >= raw.size() || raw[p] != '>') {                  // >
      out += raw[i++];
      continue;
    }
    out += "<Empty></Empty>";
    i = p + 1;
  }
  return out;
}

/// @brief WKT 值：一个坐标点，或一个括号分组。
struct WktValue
{
  bool is_point{false};
  Point point;
  std::vector<WktValue> children;
};

/// @brief WKT 递归下降解析器（对应 Python 的 tokenizer + _parse_group）。
class WktReader
{
public:
  explicit WktReader(const std::string & body) : body_(body) {}

  /// @brief 解析一个 ``( ... )`` 分组。
  std::vector<WktValue> parse_group()
  {
    skip_space();
    if (peek() != '(') {
      throw ChartParseError("WKT coordinate group must start with '('");
    }
    ++pos_;
    std::vector<WktValue> values;
    while (true) {
      skip_space();
      const char c = peek();
      if (c == '\0') {
        throw ChartParseError("unterminated WKT coordinate group");
      }
      if (c == ')') {
        ++pos_;
        break;
      }
      if (c == '(') {
        WktValue child;
        child.children = parse_group();
        values.push_back(std::move(child));
      } else {
        values.push_back(parse_point());
      }
      skip_space();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      if (peek() == ')') {
        ++pos_;
        break;
      }
      throw ChartParseError("expected ',' or ')' in WKT");
    }
    return values;
  }

  /// @brief 确认几何串已全部消费。
  void finish()
  {
    skip_space();
    if (pos_ != body_.size()) {
      throw ChartParseError("unexpected tokens after WKT geometry");
    }
  }

private:
  void skip_space()
  {
    while (pos_ < body_.size() && std::isspace(static_cast<unsigned char>(body_[pos_])) != 0) {
      ++pos_;
    }
  }

  char peek() const { return pos_ < body_.size() ? body_[pos_] : '\0'; }

  /// @brief 读取一个数值；非数值开头立即报错。
  double read_number()
  {
    skip_space();
    if (pos_ >= body_.size()) {
      throw ChartParseError("truncated WKT coordinate");
    }
    const char c = body_[pos_];
    if (std::isdigit(static_cast<unsigned char>(c)) == 0 && c != '+' && c != '-' && c != '.') {
      throw ChartParseError("WKT coordinate is not numeric");
    }
    const char * start = body_.c_str() + pos_;
    char * end = nullptr;
    const double value = std::strtod(start, &end);
    if (end == start) {
      throw ChartParseError("WKT coordinate is not numeric");
    }
    pos_ += static_cast<std::size_t>(end - start);
    return value;
  }

  /// @brief 解析 ``x y [z [m]]``，最多忽略两个额外数值。
  WktValue parse_point()
  {
    WktValue value;
    value.is_point = true;
    value.point.x = read_number();
    value.point.y = read_number();
    for (int i = 0; i < 2; ++i) {
      skip_space();
      const char c = peek();
      if (c == '\0' || c == ',' || c == ')' || c == '(') {
        break;
      }
      if (std::isdigit(static_cast<unsigned char>(c)) != 0 || c == '+' || c == '-' || c == '.') {
        read_number();
      } else if (std::isalpha(static_cast<unsigned char>(c)) != 0 || c == '_') {
        while (std::isalnum(static_cast<unsigned char>(peek())) != 0 || peek() == '_') {
          ++pos_;
        }
      } else {
        throw ChartParseError("invalid WKT coordinate");
      }
    }
    return value;
  }

  const std::string & body_;
  std::size_t pos_{0};
};

/// @brief 把分组值转成路径，遇到坐标点即报错。
std::vector<Point> path_of(const std::vector<WktValue> & items)
{
  std::vector<Point> path;
  path.reserve(items.size());
  for (const auto & item : items) {
    if (!item.is_point) {
      throw ChartParseError("expected a WKT point");
    }
    path.push_back(item.point);
  }
  return path;
}

/// @brief 取分组子项，坐标点视为非法。
const std::vector<WktValue> & group_of(const WktValue & value)
{
  if (value.is_point) {
    throw ChartParseError("expected a WKT point path");
  }
  return value.children;
}

/// @brief 对几何的每个坐标点应用坐标变换。
Geometry transform_geometry(const Geometry & geometry, const std::function<Point(double, double)> & transform)
{
  Geometry result;
  result.type = geometry.type;
  result.parts.reserve(geometry.parts.size());
  for (const auto & part : geometry.parts) {
    std::vector<Point> points;
    points.reserve(part.size());
    for (const auto & point : part) {
      points.push_back(transform(point.x, point.y));
    }
    result.parts.push_back(std::move(points));
  }
  return result;
}

/// @brief 读取 ``./Geometry/Geometries/*`` 中所有非空 WKT 文本。
std::vector<std::string> wkt_texts(const tinyxml2::XMLElement * feature)
{
  std::vector<std::string> texts;
  const tinyxml2::XMLElement * geometry = feature ? feature->FirstChildElement("Geometry") : nullptr;
  const tinyxml2::XMLElement * geometries = geometry ? geometry->FirstChildElement("Geometries") : nullptr;
  for (const tinyxml2::XMLElement * element = geometries ? geometries->FirstChildElement() : nullptr;
    element != nullptr; element = element->NextSiblingElement())
  {
    const std::string text = strip(element->GetText() ? element->GetText() : "");
    if (!text.empty()) {
      texts.push_back(text);
    }
  }
  return texts;
}

/// @brief 返回直系子元素的文本；元素缺失或文本为空时使用默认值。
std::string child_text(const tinyxml2::XMLElement * parent, const char * name, const std::string & fallback)
{
  const tinyxml2::XMLElement * element = parent ? parent->FirstChildElement(name) : nullptr;
  if (element == nullptr || element->GetText() == nullptr || element->GetText()[0] == '\0') {
    return fallback;
  }
  return element->GetText();
}

}  // namespace

std::vector<std::string> Chart::groups() const
{
  std::vector<std::string> result;
  std::unordered_set<std::string> seen;
  for (const auto & feature : features) {
    if (seen.insert(feature.group).second) {
      result.push_back(feature.group);
    }
  }
  return result;
}

std::string Chart::metadata_json() const
{
  std::string out = "{\"bounds_lon_lat\": [";
  for (std::size_t i = 0; i < bounds.size(); ++i) {
    out += (i == 0 ? "" : ", ") + json::number(bounds[i]);
  }
  out += "], \"feature_count\": " + std::to_string(features.size());
  out += ", \"groups\": [";
  const auto names = groups();
  for (std::size_t i = 0; i < names.size(); ++i) {
    out += (i == 0 ? "" : ", ") + json::text(names[i]);
  }
  out += "], \"map_count\": " + std::to_string(map_count);
  out += ", \"origin_latitude\": " + json::number(origin_latitude);
  out += ", \"origin_longitude\": " + json::number(origin_longitude);
  out += ", \"scale\": " + json::number(scale);
  out += ", \"source\": " + json::text(source_path);
  out += ", \"source_is_projected\": " + std::string(source_is_projected ? "true" : "false");
  out += "}";
  return out;
}

Geometry parse_wkt(const std::string & text)
{
  const std::string value = strip(text);
  if (value.empty()) {
    throw ChartParseError("empty WKT geometry");
  }

  // 类型名是开头的连续字母；可选的 Z/M/ZM 后缀仅在字母后有空白时成立。
  std::size_t index = 0;
  while (index < value.size() && std::isalpha(static_cast<unsigned char>(value[index])) != 0) {
    ++index;
  }
  if (index == 0) {
    throw ChartParseError("invalid WKT geometry '" + value.substr(0, 40) + "'");
  }
  const std::string type = upper(value.substr(0, index));
  const auto skip_space = [&value](std::size_t pos) {
    while (pos < value.size() && std::isspace(static_cast<unsigned char>(value[pos])) != 0) {
      ++pos;
    }
    return pos;
  };
  index = skip_space(index);
  for (const std::string suffix : {"ZM", "Z", "M"}) {
    if (value.compare(index, suffix.size(), suffix) == 0) {
      index += suffix.size();
      break;
    }
  }
  index = skip_space(index);
  const std::string body = value.substr(index);
  if (upper(strip(body)) == "EMPTY") {
    return Geometry{type, {}};
  }

  WktReader reader(body);
  const std::vector<WktValue> values = reader.parse_group();
  reader.finish();

  Geometry result;
  result.type = type;
  if (type == "POINT") {
    if (!values.empty()) {
      result.parts.push_back(path_of({values[0]}));
    }
  } else if (type == "MULTIPOINT") {
    for (const auto & item : values) {
      // 同时接受 MULTIPOINT (1 2, 3 4) 与 MULTIPOINT ((1 2), (3 4))。
      if (item.is_point) {
        result.parts.push_back({item.point});
      } else {
        if (item.children.empty()) {
          throw ChartParseError("expected a WKT point");
        }
        result.parts.push_back(path_of({item.children[0]}));
      }
    }
  } else if (type == "LINESTRING") {
    result.parts.push_back(path_of(values));
  } else if (type == "MULTILINESTRING" || type == "POLYGON") {
    for (const auto & item : values) {
      result.parts.push_back(path_of(group_of(item)));
    }
  } else if (type == "MULTIPOLYGON") {
    for (const auto & polygon : values) {
      for (const auto & ring : group_of(polygon)) {
        result.parts.push_back(path_of(group_of(ring)));
      }
    }
  } else {
    throw ChartParseError("unsupported WKT geometry type '" + type + "'");
  }
  return result;
}

Chart load_chart(const std::string & path, double origin_longitude, double origin_latitude, double scale)
{
  std::string source_path = path;
  if (!source_path.empty() && source_path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      source_path = std::string(home) + source_path.substr(1);
    }
  }

  std::ifstream stream(source_path, std::ios::binary);
  if (!stream) {
    throw ChartParseError("cannot read chart '" + source_path + "': no such file or directory");
  }
  const std::string raw((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());

  tinyxml2::XMLDocument document;
  const std::string sanitized = sanitize_empty_tags(raw);
  if (document.Parse(sanitized.c_str(), sanitized.size()) != tinyxml2::XML_SUCCESS) {
    throw ChartParseError(
      "invalid XML in '" + source_path + "': " +
      (document.ErrorStr() ? document.ErrorStr() : "parse error"));
  }
  const tinyxml2::XMLElement * root = document.RootElement();
  if (root == nullptr) {
    throw ChartParseError("XML chart contains no Map elements");
  }

  std::vector<const tinyxml2::XMLElement *> maps;
  for (const tinyxml2::XMLElement * element = root->FirstChildElement("Map");
    element != nullptr; element = element->NextSiblingElement("Map"))
  {
    maps.push_back(element);
  }
  if (maps.empty()) {
    throw ChartParseError("XML chart contains no Map elements");
  }

  // 全部 Map 的 Bounds 合并成一张海图范围。
  std::vector<std::array<double, 4>> rows;
  for (const tinyxml2::XMLElement * map : maps) {
    const tinyxml2::XMLElement * bounds = map->FirstChildElement("Bounds");
    if (bounds == nullptr) {
      continue;
    }
    try {
      rows.push_back({
        parse_double(child_text(bounds, "MinX", "nan")),
        parse_double(child_text(bounds, "MinY", "nan")),
        parse_double(child_text(bounds, "MaxX", "nan")),
        parse_double(child_text(bounds, "MaxY", "nan")),
      });
    } catch (const std::exception &) {
      continue;
    }
  }
  bool bounds_ok = !rows.empty();
  for (const auto & row : rows) {
    for (const double value : row) {
      bounds_ok = bounds_ok && std::isfinite(value);
    }
  }
  if (!bounds_ok) {
    throw ChartParseError("chart contains no valid geographic Bounds");
  }
  std::array<double, 4> bounds{
    rows[0][0], rows[0][1], rows[0][2], rows[0][3]};
  for (const auto & row : rows) {
    bounds[0] = std::min(bounds[0], row[0]);
    bounds[1] = std::min(bounds[1], row[1]);
    bounds[2] = std::max(bounds[2], row[2]);
    bounds[3] = std::max(bounds[3], row[3]);
  }

  const double auto_longitude = (bounds[0] + bounds[2]) / 2.0;
  const double auto_latitude = (bounds[1] + bounds[3]) / 2.0;
  const double longitude = std::isfinite(origin_longitude) ? origin_longitude : auto_longitude;
  const double latitude = std::isfinite(origin_latitude) ? origin_latitude : auto_latitude;
  if (!std::isfinite(scale) || scale <= 0.0) {
    throw ChartParseError("scale must be a finite number greater than zero");
  }

  // 用第一个可解析的坐标判断源文件是投影坐标还是经纬度。
  bool have_sample = false;
  Point sample{};
  for (const tinyxml2::XMLElement * map : maps) {
    const tinyxml2::XMLElement * features = map->FirstChildElement("Features");
    for (const tinyxml2::XMLElement * group =
      features ? features->FirstChildElement("FeatureGroup") : nullptr;
      group != nullptr && !have_sample; group = group->NextSiblingElement("FeatureGroup"))
    {
      for (const tinyxml2::XMLElement * feature = group->FirstChildElement("Feature");
        feature != nullptr && !have_sample; feature = feature->NextSiblingElement("Feature"))
      {
        for (const std::string & wkt : wkt_texts(feature)) {
          try {
            const Geometry geometry = parse_wkt(wkt);
            if (!geometry.parts.empty() && !geometry.parts[0].empty()) {
              sample = geometry.parts[0][0];
              have_sample = true;
              break;
            }
          } catch (const ChartParseError &) {
            continue;
          }
        }
      }
    }
    if (have_sample) {
      break;
    }
  }
  const bool source_is_projected =
    have_sample && std::max(std::abs(sample.x), std::abs(sample.y)) > 1000.0;

  std::function<Point(double, double)> transform;
  if (source_is_projected) {
    const double origin_longitude_radians = longitude * kDegToRad;
    const double origin_latitude_radians = latitude * kDegToRad;
    const double cos_origin_latitude = std::cos(origin_latitude_radians);
    transform = [origin_longitude_radians, origin_latitude_radians, cos_origin_latitude, scale](
      double x, double y) {
        // 导出器把 EPSG:3857 的 northing 存成 -Y，这里先还原纬度。
        const double longitude_radians = x / kEarthRadius;
        const double latitude_radians = std::atan(std::sinh(-y / kEarthRadius));
        return Point{
          kEarthRadius * (longitude_radians - origin_longitude_radians) * cos_origin_latitude / scale,
          kEarthRadius * (latitude_radians - origin_latitude_radians) / scale};
      };
  } else {
    const double cos_latitude = std::cos(latitude * kDegToRad);
    transform = [longitude, latitude, cos_latitude, scale](double x, double y) {
        return Point{
          (x - longitude) * kMetersPerDegree * cos_latitude / scale,
          (y - latitude) * kMetersPerDegree / scale};
      };
  }

  Chart chart;
  chart.bounds = bounds;
  chart.map_count = static_cast<int>(maps.size());
  chart.origin_longitude = longitude;
  chart.origin_latitude = latitude;
  chart.scale = scale;
  chart.source_is_projected = source_is_projected;
  chart.source_path = source_path;

  for (std::size_t map_index = 0; map_index < maps.size(); ++map_index) {
    const tinyxml2::XMLElement * map = maps[map_index];
    std::map<std::string, std::string> map_metadata;
    const tinyxml2::XMLElement * metadata = map->FirstChildElement("Metadata");
    for (const tinyxml2::XMLElement * item = metadata ? metadata->FirstChildElement() : nullptr;
      item != nullptr; item = item->NextSiblingElement())
    {
      if (std::string(item->Name()) == "Empty") {
        continue;
      }
      map_metadata[item->Name()] = strip(item->GetText() ? item->GetText() : "");
    }
    chart.metadata.push_back(std::move(map_metadata));

    const tinyxml2::XMLElement * features = map->FirstChildElement("Features");
    for (const tinyxml2::XMLElement * group = features ? features->FirstChildElement("FeatureGroup") : nullptr;
      group != nullptr; group = group->NextSiblingElement("FeatureGroup"))
    {
      const char * acronym = group->Attribute("acronym");
      std::string group_name = acronym != nullptr ? acronym : "";
      if (group_name.empty()) {
        group_name = child_text(group, "Name", "");
      }
      if (group_name.empty()) {
        group_name = "Generic";
      }

      for (const tinyxml2::XMLElement * feature = group->FirstChildElement("Feature");
        feature != nullptr; feature = feature->NextSiblingElement("Feature"))
      {
        std::vector<Geometry> geometries;
        for (const std::string & wkt : wkt_texts(feature)) {
          try {
            geometries.push_back(transform_geometry(parse_wkt(wkt), transform));
          } catch (const ChartParseError &) {
            // 单个坏要素不应影响整份海图里其余数百个有效对象。
            continue;
          }
        }
        if (geometries.empty()) {
          continue;
        }

        ChartFeature chart_feature;
        chart_feature.map_index = static_cast<int>(map_index);
        chart_feature.group = group_name;
        chart_feature.rcid = child_text(feature, "RCID", "");
        chart_feature.feature_id = child_text(feature, "ID", "");
        chart_feature.name = child_text(feature, "Name", group_name);
        chart_feature.feature_type = upper(child_text(feature, "Type", ""));
        chart_feature.geometry = std::move(geometries);

        const tinyxml2::XMLElement * attributes = feature->FirstChildElement("Attributes");
        for (const tinyxml2::XMLElement * item = attributes ? attributes->FirstChildElement() : nullptr;
          item != nullptr; item = item->NextSiblingElement())
        {
          if (std::string(item->Name()) == "Empty") {
            continue;
          }
          chart_feature.attributes[item->Name()] = strip(item->GetText() ? item->GetText() : "");
        }
        chart.features.push_back(std::move(chart_feature));
      }
    }
  }
  return chart;
}

}  // namespace river_chart
