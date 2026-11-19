/**
 * @file renderer.cpp
 * @brief 海图渲染实现。
 */


#include "usv_map/renderer.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "json_util.hpp"

namespace usv_map
{
namespace
{

using visualization_msgs::msg::Marker;

/** 已知 S-57 分组配色；未列出的分组由 SHA-1 派生稳定颜色。 */
const std::map<std::string, std::array<double, 3>> kGroupColors = {
  {"DEPARE", {0.16, 0.47, 0.90}},
  {"LNDARE", {0.72, 0.55, 0.24}},
  {"LNDRGN", {0.78, 0.62, 0.28}},
  {"DEPCNT", {0.25, 0.72, 0.90}},
  {"TSSBND", {0.10, 1.00, 0.35}},
  {"TSELNE", {0.10, 1.00, 0.35}},
  {"TSSLPT", {0.10, 1.00, 0.35}},
  {"ACHARE", {1.00, 0.55, 0.05}},
  {"BERTHS", {1.00, 0.10, 0.75}},
  {"BRIDGE", {1.00, 0.10, 0.10}},
  {"BOYSPP", {1.00, 0.90, 0.05}},
  {"BUAARE", {0.85, 0.35, 0.22}},
  {"ROADWY", {0.90, 0.85, 0.22}},
  {"RESARE", {0.65, 0.40, 0.85}},
  {"SLOTOP", {0.96, 0.40, 0.70}},
  {"SOUNDG", {0.25, 0.86, 0.65}},
  {"LIGHTS", {1.00, 0.95, 0.35}},
};

const std::unordered_set<std::string> kHiddenVisualGroups = {
  "M_NPUB", "M_NSYS", "M_QUAL", "GENERIC", "ACHARE"};
const std::unordered_set<std::string> kNavigationFocusGroups = {
  "TSSBND", "TSELNE", "TSSLPT", "BOYSPP", "BRIDGE", "BERTHS", "DEPCNT"};
const std::unordered_set<std::string> kStaticObstacleGroups = {"LNDARE"};
const std::unordered_set<std::string> kChannelGroups = {"TSSBND", "TSELNE", "TSSLPT"};
constexpr double kDirectionArrowLength = 8.0;
constexpr double kDirectionArrowHeight = 0.35;
constexpr double kPi = 3.14159265358979323846;

std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

/** @brief 严格解析完整的浮点数字符串。 */
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

/** @brief 计算非负余数，结果落在 [0, modulus)。 */
double positive_mod(double value, double modulus)
{
  return value - modulus * std::floor(value / modulus);
}

/** @brief 保留两位小数的日志文本。 */
std::string fixed2(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", value);
  return buffer;
}

/** @brief 内联 SHA-1，仅用于未配色分组的稳定派生颜色。 */
std::array<unsigned char, 20> sha1(const std::string & text)
{
  const auto rotate_left = [](std::uint32_t value, int bits) {
    return (value << bits) | (value >> (32 - bits));
  };
  std::vector<unsigned char> message(text.begin(), text.end());
  const std::uint64_t bit_length = static_cast<std::uint64_t>(message.size()) * 8u;
  message.push_back(0x80);
  while (message.size() % 64 != 56) {
    message.push_back(0x00);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    message.push_back(static_cast<unsigned char>((bit_length >> shift) & 0xFFu));
  }

  std::uint32_t hash[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
  for (std::size_t offset = 0; offset < message.size(); offset += 64) {
    std::uint32_t words[80];
    for (int i = 0; i < 16; ++i) {
      const std::size_t base = offset + static_cast<std::size_t>(i) * 4;
      words[i] = (static_cast<std::uint32_t>(message[base]) << 24) |
        (static_cast<std::uint32_t>(message[base + 1]) << 16) |
        (static_cast<std::uint32_t>(message[base + 2]) << 8) |
        static_cast<std::uint32_t>(message[base + 3]);
    }
    for (int i = 16; i < 80; ++i) {
      words[i] = rotate_left(
        words[i - 3] ^ words[i - 8] ^ words[i - 14] ^ words[i - 16], 1);
    }
    std::uint32_t a = hash[0], b = hash[1], c = hash[2], d = hash[3], e = hash[4];
    for (int i = 0; i < 80; ++i) {
      std::uint32_t f = 0, k = 0;
      if (i < 20) {
        f = (b & c) | ((~b) & d);
        k = 0x5A827999u;
      } else if (i < 40) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1u;
      } else if (i < 60) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCu;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6u;
      }
      const std::uint32_t temp = rotate_left(a, 5) + f + e + k + words[i];
      e = d;
      d = c;
      c = rotate_left(b, 30);
      b = a;
      a = temp;
    }
    hash[0] += a;
    hash[1] += b;
    hash[2] += c;
    hash[3] += d;
    hash[4] += e;
  }

  std::array<unsigned char, 20> digest{};
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 4; ++j) {
      digest[static_cast<std::size_t>(i) * 4 + j] =
        static_cast<unsigned char>((hash[i] >> (24 - j * 8)) & 0xFFu);
    }
  }
  return digest;
}

std::array<double, 3> color_for_group(const std::string & group)
{
  const auto known = kGroupColors.find(upper(group));
  if (known != kGroupColors.end()) {
    return known->second;
  }
  // 哈希使用原始大小写，确保分组颜色稳定。
  const auto digest = sha1(group);
  return {
    0.30 + digest[0] / 255.0 * 0.65,
    0.30 + digest[1] / 255.0 * 0.65,
    0.30 + digest[2] / 255.0 * 0.65};
}

geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

/** @brief 遍历海图坐标，避免为边界计算复制全部点。 */
template<typename Function>
void for_each_point(const Chart & chart, Function function)
{
  for (const auto & feature : chart.features) {
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        for (const auto & point : part) {
          function(point);
        }
      }
    }
  }
}

/** @brief 栅格几何尺寸。 */
struct GridShape
{
  double min_x{0.0};
  double min_y{0.0};
  double resolution{1.0};
  int width{1};
  int height{1};
};

/** @brief 规划栅格范围；无坐标时退回 (-100,-100)-(100,100)。 */
std::optional<std::array<double, 4>> chart_bounds(const Chart & chart)
{
  std::array<double, 4> bounds{};
  bool has_point = false;
  for_each_point(chart, [&](const Point & point) {
    if (!has_point) {
      bounds = {point.x, point.y, point.x, point.y};
      has_point = true;
      return;
    }
    bounds[0] = std::min(bounds[0], point.x);
    bounds[1] = std::min(bounds[1], point.y);
    bounds[2] = std::max(bounds[2], point.x);
    bounds[3] = std::max(bounds[3], point.y);
  });
  return has_point ? std::optional<std::array<double, 4>>(bounds) : std::nullopt;
}

std::array<double, 4> planning_bounds(const Chart & chart, const RenderConfig & config)
{
  const std::array<double, 4> full_bounds =
    chart_bounds(chart).value_or(std::array<double, 4>{-100.0, -100.0, 100.0, 100.0});
  const auto valid_fraction = [](double value) {
      return std::isfinite(value) && value > 0.0 && value <= 1.0;
    };
  const double width_fraction = valid_fraction(config.planning_crop_width_fraction) ?
    config.planning_crop_width_fraction : 1.0;
  const double height_fraction = valid_fraction(config.planning_crop_height_fraction) ?
    config.planning_crop_height_fraction : 1.0;
  return {
    full_bounds[0],
    full_bounds[1],
    full_bounds[0] + (full_bounds[2] - full_bounds[0]) * width_fraction,
    full_bounds[1] + (full_bounds[3] - full_bounds[1]) * height_fraction};
}

/** @brief 计算规划栅格分辨率与宽高，超过上限时自动降低分辨率。 */
GridShape planning_shape(
  const Chart & chart, const RenderConfig & config, const Renderer::WarnFn & warn)
{
  const std::array<double, 4> bounds = planning_bounds(chart, config);
  double resolution = config.planning_resolution;
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    resolution = 1.0;
  }
  int width = std::max(
    1, static_cast<int>(std::ceil((bounds[2] - bounds[0]) / resolution)) + 1);
  int height = std::max(
    1, static_cast<int>(std::ceil((bounds[3] - bounds[1]) / resolution)) + 1);
  const long long max_cells = std::max(1LL, config.planning_max_cells);
  const long long cells = static_cast<long long>(width) * static_cast<long long>(height);
  if (cells > max_cells) {
    resolution *= std::sqrt(static_cast<double>(cells) / static_cast<double>(max_cells));
    width = std::max(
      1, static_cast<int>(std::ceil((bounds[2] - bounds[0]) / resolution)) + 1);
    height = std::max(
      1, static_cast<int>(std::ceil((bounds[3] - bounds[1]) / resolution)) + 1);
    if (warn) {
      warn(
        "Planning grid exceeded planning_max_cells; using " + fixed2(resolution) +
        " m resolution (" + std::to_string(width) + "x" + std::to_string(height) + ")");
    }
  }
  return GridShape{bounds[0], bounds[1], resolution, width, height};
}

double distance_squared_to_segment(
  double x, double y, const Point & start, const Point & end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-12) {
    const double distance_x = x - start.x;
    const double distance_y = y - start.y;
    return distance_x * distance_x + distance_y * distance_y;
  }
  const double ratio = std::clamp(
    ((x - start.x) * dx + (y - start.y) * dy) / length_squared, 0.0, 1.0);
  const double nearest_x = start.x + ratio * dx;
  const double nearest_y = start.y + ratio * dy;
  const double distance_x = x - nearest_x;
  const double distance_y = y - nearest_y;
  return distance_x * distance_x + distance_y * distance_y;
}

/** @brief 按线段胶囊区域栅格化，避免逐采样点重复绘制圆盘。 */
void raster_segment(
  std::vector<int8_t> & data, const GridShape & shape, const Point & start, const Point & end,
  int8_t value, double radius = 0.0)
{
  const double safe_radius = std::max(0.0, radius);
  const double low_x = std::min(start.x, end.x) - safe_radius;
  const double high_x = std::max(start.x, end.x) + safe_radius;
  const double low_y = std::min(start.y, end.y) - safe_radius;
  const double high_y = std::max(start.y, end.y) + safe_radius;
  const int first_x = std::max(
    0, static_cast<int>(std::ceil((low_x - shape.min_x) / shape.resolution)));
  const int last_x = std::min(
    shape.width - 1, static_cast<int>(std::floor((high_x - shape.min_x) / shape.resolution)));
  const int first_y = std::max(
    0, static_cast<int>(std::ceil((low_y - shape.min_y) / shape.resolution)));
  const int last_y = std::min(
    shape.height - 1, static_cast<int>(std::floor((high_y - shape.min_y) / shape.resolution)));
  if (first_x > last_x || first_y > last_y) {
    return;
  }

  const double radius_squared = safe_radius * safe_radius;
  for (int cy = first_y; cy <= last_y; ++cy) {
    const double y = shape.min_y + static_cast<double>(cy) * shape.resolution;
    for (int cx = first_x; cx <= last_x; ++cx) {
      const double x = shape.min_x + static_cast<double>(cx) * shape.resolution;
      if (distance_squared_to_segment(x, y, start, end) <= radius_squared + 1e-9) {
        data[static_cast<std::size_t>(cy) * shape.width + cx] = value;
      }
    }
  }
}

/** @brief 以不超过膨胀半径的间距简化折线，再绘制其胶囊边界。 */
void raster_polyline(
  std::vector<int8_t> & data, const GridShape & shape, const std::vector<Point> & part,
  int8_t value, double radius)
{
  if (part.empty()) {
    return;
  }
  if (part.size() == 1) {
    raster_segment(data, shape, part.front(), part.front(), value, radius);
    return;
  }

  const bool closed = part.front().x == part.back().x && part.front().y == part.back().y;
  const std::size_t point_count = part.size() - static_cast<std::size_t>(closed);
  if (point_count == 0) {
    return;
  }
  const double spacing = std::max(
    shape.resolution * 0.5, std::max(0.5, std::max(0.0, radius) * 0.25));
  std::vector<Point> simplified;
  simplified.reserve(point_count);
  simplified.push_back(part.front());
  for (std::size_t i = 1; i < point_count; ++i) {
    const Point & candidate = part[i];
    const Point & anchor = simplified.back();
    if (std::hypot(candidate.x - anchor.x, candidate.y - anchor.y) >= spacing ||
      i + 1 == point_count)
    {
      simplified.push_back(candidate);
    }
  }
  if (simplified.size() == 1) {
    raster_segment(data, shape, simplified.front(), simplified.front(), value, radius);
    return;
  }
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    raster_segment(data, shape, simplified[i - 1], simplified[i], value, radius);
  }
  if (closed) {
    raster_segment(data, shape, simplified.back(), simplified.front(), value, radius);
  }
}

/** @brief 按奇偶规则填充多环多边形，内环保留已有的水域代价。 */
void fill_polygon(
  std::vector<int8_t> & data, const GridShape & shape,
  const std::vector<std::vector<Point>> & rings,
  int8_t value)
{
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & ring : rings) {
    const bool closed = ring.size() >= 2 && ring.front().x == ring.back().x &&
      ring.front().y == ring.back().y;
    const std::size_t point_count = ring.size() - static_cast<std::size_t>(closed);
    if (point_count < 3) {
      continue;
    }
    for (std::size_t i = 0; i < point_count; ++i) {
      min_y = std::min(min_y, ring[i].y);
      max_y = std::max(max_y, ring[i].y);
    }
  }
  if (!std::isfinite(min_y) || !std::isfinite(max_y)) {
    return;
  }
  const int low_y = std::max(
    0, static_cast<int>(std::floor((min_y - shape.min_y) / shape.resolution)));
  const int high_y = std::min(
    shape.height - 1, static_cast<int>(std::ceil((max_y - shape.min_y) / shape.resolution)));

  std::vector<double> intersections;
  for (int iy = low_y; iy <= high_y; ++iy) {
    const double y = shape.min_y + (iy + 0.5) * shape.resolution;
    intersections.clear();
    for (const auto & ring : rings) {
      const bool closed = ring.size() >= 2 && ring.front().x == ring.back().x &&
        ring.front().y == ring.back().y;
      const std::size_t point_count = ring.size() - static_cast<std::size_t>(closed);
      if (point_count < 3) {
        continue;
      }
      for (std::size_t i = 0; i < point_count; ++i) {
        const Point & first = ring[i];
        const Point & second = ring[(i + 1) % point_count];
        if ((first.y > y) != (second.y > y)) {
          intersections.push_back(
            first.x + (y - first.y) * (second.x - first.x) / (second.y - first.y));
        }
      }
    }
    std::sort(intersections.begin(), intersections.end());
    for (std::size_t i = 0; i + 1 < intersections.size(); i += 2) {
      const double left = intersections[i];
      const double right = intersections[i + 1];
      const int low_x = std::max(
        0, static_cast<int>(std::floor((left - shape.min_x) / shape.resolution)));
      const int high_x = std::min(
        shape.width - 1, static_cast<int>(std::ceil((right - shape.min_x) / shape.resolution)));
      for (int ix = low_x; ix <= high_x; ++ix) {
        data[static_cast<std::size_t>(iy) * shape.width + ix] = value;
      }
    }
  }
}

}  // namespace

Renderer::Renderer(RenderConfig config, WarnFn warn)
: config_(std::move(config)), warn_(std::move(warn))
{
}

visualization_msgs::msg::Marker Renderer::base_marker(
  const std::string & ns, int id, int type, const builtin_interfaces::msg::Time & stamp) const
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 0.9;
  marker.scale.x = std::max(0.001, config_.line_width);
  return marker;
}

void Renderer::set_color(Marker & marker, const std::string & group, double alpha) const
{
  const std::array<double, 3> color = color_for_group(group);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  if (alpha >= 0.0) {
    marker.color.a = alpha;
  }
}

visualization_msgs::msg::MarkerArray Renderer::build_markers(
  const Chart & chart, const builtin_interfaces::msg::Time & stamp) const
{
  visualization_msgs::msg::MarkerArray result;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  result.markers.push_back(delete_all);

  int marker_id = 1;
  for (const auto & feature : chart.features) {
    const std::string group_upper = upper(feature.group);
    if (kHiddenVisualGroups.count(group_upper) != 0) {
      continue;
    }
    if (config_.focus_navigation_features && kNavigationFocusGroups.count(group_upper) == 0 &&
      kStaticObstacleGroups.count(group_upper) == 0)
    {
      continue;
    }
    const std::string ns = "s57/" + feature.group;
    for (const auto & geometry : feature.geometry) {
      const bool polygon_geometry =
        geometry.type == "POLYGON" || geometry.type == "MULTIPOLYGON";
      if (feature.feature_type == "P" || geometry.type == "POINT" || geometry.type == "MULTIPOINT") {
        Marker marker = base_marker(ns, marker_id, Marker::POINTS, stamp);
        marker.scale.x = config_.point_size;
        marker.scale.y = config_.point_size;
        set_color(marker, feature.group);
        for (const auto & part : geometry.parts) {
          for (const auto & point : part) {
            marker.points.push_back(make_point(point.x, point.y));
          }
        }
        if (!marker.points.empty()) {
          result.markers.push_back(std::move(marker));
          ++marker_id;
        }
        continue;
      }

      for (const auto & ring : geometry.parts) {
        if (feature.feature_type == "P" || ring.size() == 1) {
          Marker marker = base_marker(ns, marker_id, Marker::POINTS, stamp);
          marker.scale.x = config_.point_size;
          marker.scale.y = config_.point_size;
          set_color(marker, feature.group);
          marker.points.push_back(make_point(ring[0].x, ring[0].y));
          result.markers.push_back(std::move(marker));
          ++marker_id;
          continue;
        }
        if (ring.size() < 2) {
          continue;
        }
        std::size_t drawable_size = ring.size();
        if (feature.feature_type == "L" && ring.front().x == ring.back().x &&
          ring.front().y == ring.back().y)
        {
          --drawable_size;
        }
        if (drawable_size < 2) {
          continue;
        }
        Marker marker = base_marker(ns, marker_id, Marker::LINE_STRIP, stamp);
        set_color(marker, feature.group);
        for (std::size_t i = 0; i < drawable_size; ++i) {
          marker.points.push_back(make_point(ring[i].x, ring[i].y));
        }
        const bool closed = ring.front().x == ring.back().x && ring.front().y == ring.back().y;
        const bool open_kind =
          feature.feature_type == "L" || feature.feature_type == "S" || feature.feature_type == "O";
        if (!open_kind && polygon_geometry && !closed) {
          marker.points.push_back(marker.points.front());
        }
        result.markers.push_back(std::move(marker));
        ++marker_id;

        // TSSLPT.ORIENT 以北为零、顺时针为正；多边形中心箭头表示航道代价使用的方向。
        if (group_upper == "TSSLPT" && polygon_geometry && ring.size() >= 3U) {
          const auto orient_attribute = feature.attributes.find("ORIENT");
          if (orient_attribute != feature.attributes.end()) {
            try {
              const double orient_deg = parse_double(orient_attribute->second);
              if (std::isfinite(orient_deg)) {
                const std::size_t point_count =
                  ring.size() > 1U && ring.front().x == ring.back().x &&
                  ring.front().y == ring.back().y ? ring.size() - 1U : ring.size();
                if (point_count >= 3U) {
                  double center_x = 0.0;
                  double center_y = 0.0;
                  for (std::size_t i = 0; i < point_count; ++i) {
                    center_x += ring[i].x;
                    center_y += ring[i].y;
                  }
                  center_x /= static_cast<double>(point_count);
                  center_y /= static_cast<double>(point_count);

                  const double angle = positive_mod(orient_deg, 360.0) * kPi / 180.0;
                  Marker direction = base_marker(
                    "s57/TSSLPT_DIRECTION", marker_id, Marker::ARROW, stamp);
                  direction.scale.x = 0.35;
                  direction.scale.y = 0.75;
                  direction.scale.z = kDirectionArrowHeight;
                  set_color(direction, feature.group, 1.0);
                  direction.points.push_back(make_point(center_x, center_y, 0.2));
                  direction.points.push_back(make_point(
                    center_x + std::sin(angle) * kDirectionArrowLength,
                    center_y + std::cos(angle) * kDirectionArrowLength, 0.2));
                  result.markers.push_back(std::move(direction));
                  ++marker_id;
                }
              }
            } catch (const std::exception &) {
              // lane_data_json() 同样忽略无效 ORIENT，保持显示与航道数据一致。
            }
          }
        }

      }
    }
  }
  return result;
}

std::string Renderer::lane_data_json(const Chart & chart) const
{
  std::string out = "{\"frame_id\":\"map\",\"lanes\":[";
  int lane_id = 0;
  bool first_lane = true;
  for (const auto & feature : chart.features) {
    if (upper(feature.group) != "TSSLPT") {
      continue;
    }
    const auto attribute = feature.attributes.find("ORIENT");
    if (attribute == feature.attributes.end()) {
      continue;
    }
    double orient = 0.0;
    try {
      orient = parse_double(attribute->second);
    } catch (const std::exception &) {
      continue;
    }
    if (!std::isfinite(orient)) {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      if (geometry.type != "POLYGON" && geometry.type != "MULTIPOLYGON") {
        continue;
      }
      for (const auto & ring : geometry.parts) {
        if (ring.size() < 4) {
          continue;
        }
        if (!first_lane) {
          out += ",";
        }
        first_lane = false;
        out += "{\"lane_id\":" + std::to_string(lane_id);
        out += ",\"orient_deg\":" + json::number(positive_mod(orient, 360.0));
        const auto direction = feature.attributes.find("DIRECTION");
        if (direction != feature.attributes.end() && !direction->second.empty()) {
          out += ",\"direction\":" + json::text(direction->second);
        }
        out += ",\"points\":[";
        for (std::size_t i = 0; i < ring.size(); ++i) {
          out += (i == 0 ? "" : ",");
          out += "[" + json::number(ring[i].x) + "," + json::number(ring[i].y) + "]";
        }
        out += "]}";
        ++lane_id;
      }
    }
  }
  out += "],\"obstacles\":[";
  bool first_obstacle = true;
  for (const auto & feature : chart.features) {
    const std::string group = upper(feature.group);
    const auto reef_attribute = feature.attributes.find("OBSTACLE_KIND");
    const bool reef = reef_attribute != feature.attributes.end() &&
      upper(reef_attribute->second) == "REEF";
    // LNDARE 同时包含陆地和测试暗礁；仅 REEF 交给管理器动态感知，
    // 陆地始终保留在静态代价地图中，代价为 100。
    if (group != "LNDARE" && !reef) {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      for (const auto & ring : geometry.parts) {
        if (ring.size() < 3U) {
          continue;
        }
        if (!first_obstacle) {
          out += ",";
        }
        first_obstacle = false;
        out += "{\"kind\":" + json::text(reef ? "REEF" : "LAND") + ",\"points\":[";
        for (std::size_t i = 0; i < ring.size(); ++i) {
          out += (i == 0 ? "" : ",");
          out += "[" + json::number(ring[i].x) + "," + json::number(ring[i].y) + "]";
        }
        out += "]}";
      }
    }
  }
  out += "]}";
  return out;
}

nav_msgs::msg::OccupancyGrid Renderer::build_costmap(
  const Chart & chart, const builtin_interfaces::msg::Time & stamp) const
{
  const GridShape shape = planning_shape(chart, config_, warn_);
  nav_msgs::msg::OccupancyGrid cost_grid;
  cost_grid.header.frame_id = "map";
  cost_grid.header.stamp = stamp;
  cost_grid.info.resolution = shape.resolution;
  cost_grid.info.width = shape.width;
  cost_grid.info.height = shape.height;
  cost_grid.info.origin.position.x = shape.min_x;
  cost_grid.info.origin.position.y = shape.min_y;
  cost_grid.info.origin.orientation.w = 1.0;

  const std::size_t cell_count = static_cast<std::size_t>(shape.width) * shape.height;
  const int outside_cost = std::max(-1, std::min(99, config_.channel_outside_cost));
  cost_grid.data.assign(cell_count, static_cast<int8_t>(outside_cost));

  // 水域要素可通行；航道走廊仍保留最低代价。
  for (const auto & feature : chart.features) {
    if (upper(feature.group) != "DEPARE") {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      fill_polygon(cost_grid.data, shape, geometry.parts, 0);
    }
  }

  // 航道走廊优先设为最低代价，后续陆地仍会覆盖为障碍。
  const double channel_radius = std::max(0.0, config_.channel_corridor_width);
  for (const auto & feature : chart.features) {
    if (kChannelGroups.count(upper(feature.group)) == 0) {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        raster_polyline(cost_grid.data, shape, part, 0, channel_radius);
      }
    }
  }

  // 暗礁保留矢量显示；陆地和禁行区写入静态代价地图并膨胀边界。
  for (const auto & feature : chart.features) {
    if (kStaticObstacleGroups.count(upper(feature.group)) == 0) {
      continue;
    }
    const auto kind = feature.attributes.find("OBSTACLE_KIND");
    if (kind != feature.attributes.end() && upper(kind->second) == "REEF") {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      fill_polygon(cost_grid.data, shape, geometry.parts, 100);
      for (const auto & part : geometry.parts) {
        if (part.size() < 3) {
          continue;
        }
        raster_polyline(cost_grid.data, shape, part, 100, config_.obstacle_inflation);
      }
    }
  }
  return cost_grid;
}

}  // namespace usv_map
