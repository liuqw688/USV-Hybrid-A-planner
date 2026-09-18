/// @file renderer.cpp
/// @brief 海图渲染实现。
///

/// @date 2026-08-28

#include "river_chart/renderer.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "json_util.hpp"

namespace river_chart
{
namespace
{

using visualization_msgs::msg::Marker;

/// 已知 S-57 分组配色；未列出的分组由 SHA-1 派生稳定颜色。
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
const std::unordered_set<std::string> kLandBoundaryGroups = {"LNDARE", "LNDRGN"};
const std::unordered_set<std::string> kNavigationFocusGroups = {
  "TSSBND", "TSELNE", "TSSLPT", "BOYSPP", "BRIDGE", "BERTHS", "DEPCNT"};
const std::unordered_set<std::string> kStaticObstacleGroups = {"LNDARE"};
const std::unordered_set<std::string> kChannelGroups = {"TSSBND", "TSELNE", "TSSLPT"};

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

/// @brief Python 风格取模，结果始终落在 [0, modulus)。
double python_mod(double value, double modulus)
{
  return value - modulus * std::floor(value / modulus);
}

/// @brief 保留两位小数的日志文本。
std::string fixed2(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", value);
  return buffer;
}

/// @brief 内联 SHA-1，仅用于未配色分组的稳定派生颜色。
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
  // 哈希使用原始大小写，与 Python hashlib.sha1(group) 一致。
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

/// @brief 收集海图内所有坐标点。
std::vector<Point> collect_points(const Chart & chart)
{
  std::vector<Point> points;
  for (const auto & feature : chart.features) {
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        points.insert(points.end(), part.begin(), part.end());
      }
    }
  }
  return points;
}

/// @brief 要素所有坐标点的平均值；无坐标时返回空。
std::optional<Point> centroid(const ChartFeature & feature)
{
  std::size_t count = 0;
  double x = 0.0;
  double y = 0.0;
  for (const auto & geometry : feature.geometry) {
    for (const auto & part : geometry.parts) {
      for (const auto & point : part) {
        x += point.x;
        y += point.y;
        ++count;
      }
    }
  }
  if (count == 0) {
    return std::nullopt;
  }
  return Point{x / static_cast<double>(count), y / static_cast<double>(count)};
}

/// @brief 三角扇填充；闭合环会先去掉重复的末点。
std::vector<geometry_msgs::msg::Point> fan_triangles(const std::vector<Point> & ring)
{
  std::vector<Point> points = ring;
  if (points.size() < 3) {
    return {};
  }
  if (points.front().x == points.back().x && points.front().y == points.back().y) {
    points.pop_back();
  }
  if (points.size() < 3) {
    return {};
  }
  std::vector<geometry_msgs::msg::Point> result;
  result.reserve((points.size() - 2) * 3);
  const auto anchor = make_point(points[0].x, points[0].y);
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    result.push_back(anchor);
    result.push_back(make_point(points[i].x, points[i].y));
    result.push_back(make_point(points[i + 1].x, points[i + 1].y));
  }
  return result;
}

/// @brief 栅格几何尺寸。
struct GridShape
{
  double min_x{0.0};
  double min_y{0.0};
  double resolution{1.0};
  int width{1};
  int height{1};
};

/// @brief 规划栅格范围；无坐标时退回 (-100,-100)-(100,100)。
std::array<double, 4> planning_bounds(const Chart & chart)
{
  const std::vector<Point> points = collect_points(chart);
  if (points.empty()) {
    return {-100.0, -100.0, 100.0, 100.0};
  }
  std::array<double, 4> bounds{
    points[0].x, points[0].y, points[0].x, points[0].y};
  for (const auto & point : points) {
    bounds[0] = std::min(bounds[0], point.x);
    bounds[1] = std::min(bounds[1], point.y);
    bounds[2] = std::max(bounds[2], point.x);
    bounds[3] = std::max(bounds[3], point.y);
  }
  return bounds;
}

/// @brief 计算规划栅格分辨率与宽高，超过上限时自动降低分辨率。
GridShape planning_shape(
  const Chart & chart, const RenderConfig & config, const Renderer::WarnFn & warn)
{
  const std::array<double, 4> bounds = planning_bounds(chart);
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

/// @brief 依次访问折线的相邻点对；单点路径退化为自配对，空路径不产生点对。
template<typename Function>
void for_each_pair(const std::vector<Point> & part, Function function)
{
  if (part.empty()) {
    return;
  }
  if (part.size() == 1) {
    function(part[0], part[0]);
    return;
  }
  for (std::size_t i = 0; i + 1 < part.size(); ++i) {
    function(part[i], part[i + 1]);
  }
}

/// @brief 以圆形笔刷把线段栅格化到网格。
void raster_segment(
  std::vector<int8_t> & data, const GridShape & shape, const Point & start, const Point & end,
  int8_t value, double radius = 0.0)
{
  const double distance = std::hypot(end.x - start.x, end.y - start.y);
  const int steps = std::max(
    1, static_cast<int>(
      std::ceil(distance / std::max(shape.resolution * 0.5, 1e-6))));
  const int cell_radius = static_cast<int>(std::ceil(radius / shape.resolution));
  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
    const double x = start.x + (end.x - start.x) * ratio;
    const double y = start.y + (end.y - start.y) * ratio;
    const int ix = static_cast<int>((x - shape.min_x) / shape.resolution);
    const int iy = static_cast<int>((y - shape.min_y) / shape.resolution);
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        if (dx * dx + dy * dy > cell_radius * cell_radius) {
          continue;
        }
        const int cx = ix + dx;
        const int cy = iy + dy;
        if (cx >= 0 && cx < shape.width && cy >= 0 && cy < shape.height) {
          data[static_cast<std::size_t>(cy) * shape.width + cx] = value;
        }
      }
    }
  }
}

/// @brief 扫描线填充多边形环。
void fill_polygon(
  std::vector<int8_t> & data, const GridShape & shape, std::vector<Point> ring, int8_t value)
{
  if (ring.size() >= 2 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) {
    ring.pop_back();
  }
  if (ring.size() < 3) {
    return;
  }
  double min_y = ring[0].y;
  double max_y = ring[0].y;
  for (const auto & point : ring) {
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  const int low_y = std::max(
    0, static_cast<int>(std::floor((min_y - shape.min_y) / shape.resolution)));
  const int high_y = std::min(
    shape.height - 1, static_cast<int>(std::ceil((max_y - shape.min_y) / shape.resolution)));

  std::vector<double> intersections;
  for (int iy = low_y; iy <= high_y; ++iy) {
    const double y = shape.min_y + (iy + 0.5) * shape.resolution;
    intersections.clear();
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const Point & first = ring[i];
      const Point & second = ring[(i + 1) % ring.size()];
      if ((first.y > y) != (second.y > y)) {
        intersections.push_back(
          first.x + (y - first.y) * (second.x - first.x) / (second.y - first.y));
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
  marker.header.frame_id = config_.frame_id;
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
    if (!config_.show_land_boundaries && kLandBoundaryGroups.count(group_upper) != 0) {
      continue;
    }
    if (config_.focus_navigation_features && kNavigationFocusGroups.count(group_upper) == 0) {
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
        std::vector<Point> drawable = ring;
        if (feature.feature_type == "L" && ring.front().x == ring.back().x &&
          ring.front().y == ring.back().y)
        {
          drawable.pop_back();
        }
        if (drawable.size() < 2) {
          continue;
        }
        Marker marker = base_marker(ns, marker_id, Marker::LINE_STRIP, stamp);
        set_color(marker, feature.group);
        for (const auto & point : drawable) {
          marker.points.push_back(make_point(point.x, point.y));
        }
        const bool closed = ring.front().x == ring.back().x && ring.front().y == ring.back().y;
        const bool open_kind =
          feature.feature_type == "L" || feature.feature_type == "S" || feature.feature_type == "O";
        if (!open_kind && polygon_geometry && !closed) {
          marker.points.push_back(marker.points.front());
        }
        result.markers.push_back(std::move(marker));
        ++marker_id;

        if (config_.fill_polygons && feature.feature_type == "A" && polygon_geometry) {
          std::vector<geometry_msgs::msg::Point> triangles = fan_triangles(ring);
          if (!triangles.empty()) {
            Marker fill = base_marker(ns, marker_id, Marker::TRIANGLE_LIST, stamp);
            set_color(fill, feature.group, 0.20);
            fill.points = std::move(triangles);
            result.markers.push_back(std::move(fill));
            ++marker_id;
          }
        }
      }
    }

    if (config_.publish_labels) {
      const std::optional<Point> center = centroid(feature);
      if (center && !feature.name.empty()) {
        Marker text = base_marker("labels/" + feature.group, marker_id, Marker::TEXT_VIEW_FACING, stamp);
        set_color(text, feature.group);
        text.pose.position.x = center->x;
        text.pose.position.y = center->y;
        text.scale.z = std::max(0.5, config_.point_size * 1.5);
        text.text = feature.name;
        result.markers.push_back(std::move(text));
        ++marker_id;
      }
    }
  }
  return result;
}

std::string Renderer::lane_data_json(const Chart & chart) const
{
  std::string out = "{\"frame_id\":" + json::text(config_.frame_id) + ",\"lanes\":[";
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
        out += ",\"orient_deg\":" + json::number(python_mod(orient, 360.0));
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
  out += "]}";
  return out;
}

visualization_msgs::msg::MarkerArray Renderer::build_direction_markers(
  const Chart & chart, const builtin_interfaces::msg::Time & stamp) const
{
  visualization_msgs::msg::MarkerArray result;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  result.markers.push_back(delete_all);

  int marker_id = 1;
  for (const auto & feature : chart.features) {
    if (kChannelGroups.count(upper(feature.group)) == 0) {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        for (std::size_t i = 0; i + 1 < part.size(); ++i) {
          const Point & start = part[i];
          const Point & end = part[i + 1];
          const double length = std::hypot(end.x - start.x, end.y - start.y);
          if (length < 30.0) {
            continue;
          }
          Marker marker = base_marker("channel_direction", marker_id, Marker::ARROW, stamp);
          marker.pose.position.x = (start.x + end.x) / 2.0;
          marker.pose.position.y = (start.y + end.y) / 2.0;
          const double yaw = std::atan2(end.y - start.y, end.x - start.x);
          marker.pose.orientation.z = std::sin(yaw / 2.0);
          marker.pose.orientation.w = std::cos(yaw / 2.0);
          marker.scale.x = std::min(250.0, length * 0.7);
          marker.scale.y = 3.0;
          marker.scale.z = 3.0;
          set_color(marker, "TSELNE");
          result.markers.push_back(std::move(marker));
          ++marker_id;
        }
      }
    }
  }
  return result;
}

nav_msgs::msg::OccupancyGrid Renderer::build_occupancy_grid(
  const Chart & chart, const builtin_interfaces::msg::Time & stamp) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = config_.frame_id;
  grid.header.stamp = stamp;

  const std::vector<Point> points = collect_points(chart);
  if (points.empty()) {
    return grid;
  }
  double resolution = config_.grid_resolution;
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    resolution = 0.2;
  }
  double min_x = points[0].x;
  double min_y = points[0].y;
  double max_x = points[0].x;
  double max_y = points[0].y;
  for (const auto & point : points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  int width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1);
  int height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1);
  const long long max_cells = std::max(1LL, config_.max_grid_cells);
  const long long cells = static_cast<long long>(width) * static_cast<long long>(height);
  if (cells > max_cells) {
    resolution *= std::sqrt(static_cast<double>(cells) / static_cast<double>(max_cells));
    width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1);
    height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1);
    if (warn_) {
      warn_(
        "Occupancy grid exceeded max_grid_cells; using " + fixed2(resolution) +
        " m resolution (" + std::to_string(width) + "x" + std::to_string(height) + ")");
    }
  }
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = min_x;
  grid.info.origin.position.y = min_y;
  grid.info.origin.orientation.w = 1.0;

  std::vector<int8_t> data(static_cast<std::size_t>(width) * height, -1);
  const auto mark = [&](double x, double y) {
      const int ix = static_cast<int>((x - min_x) / resolution);
      const int iy = static_cast<int>((y - min_y) / resolution);
      if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
        data[static_cast<std::size_t>(iy) * width + ix] = 100;
      }
    };
  for (const auto & feature : chart.features) {
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        if (part.empty()) {
          continue;
        }
        mark(part[0].x, part[0].y);
        for (std::size_t i = 0; i + 1 < part.size(); ++i) {
          const Point & start = part[i];
          const Point & end = part[i + 1];
          const double distance = std::hypot(end.x - start.x, end.y - start.y);
          const int steps = std::max(
            1, static_cast<int>(
              std::ceil(distance / std::max(resolution * 0.5, 1e-6))));
          for (int step = 1; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            mark(
              start.x + (end.x - start.x) * ratio,
              start.y + (end.y - start.y) * ratio);
          }
        }
      }
    }
  }
  grid.data = std::move(data);
  return grid;
}

void Renderer::build_planning_grids(
  const Chart & chart, const std::vector<Point> & future_obstacles,
  const builtin_interfaces::msg::Time & stamp,
  nav_msgs::msg::OccupancyGrid & static_grid,
  nav_msgs::msg::OccupancyGrid & cost_grid) const
{
  const GridShape shape = planning_shape(chart, config_, warn_);
  static_grid = nav_msgs::msg::OccupancyGrid();
  static_grid.header.frame_id = config_.frame_id;
  static_grid.header.stamp = stamp;
  static_grid.info.resolution = shape.resolution;
  static_grid.info.width = shape.width;
  static_grid.info.height = shape.height;
  static_grid.info.origin.position.x = shape.min_x;
  static_grid.info.origin.position.y = shape.min_y;
  static_grid.info.origin.orientation.w = 1.0;

  const std::size_t cell_count = static_cast<std::size_t>(shape.width) * shape.height;
  std::vector<int8_t> static_data(cell_count, -1);

  // 陆地多边形按不可通行处理，并按安全半径膨胀边界。
  for (const auto & feature : chart.features) {
    if (kStaticObstacleGroups.count(upper(feature.group)) == 0) {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        if (part.size() < 3) {
          continue;
        }
        fill_polygon(static_data, shape, part, 100);
        for_each_pair(part, [&](const Point & start, const Point & end) {
          raster_segment(
            static_data, shape, start, end, 100, config_.obstacle_inflation);
        });
      }
    }
  }

  // 代价地图：航道外为低速代价，航道走廊为 0，陆地固定 100。
  cost_grid = nav_msgs::msg::OccupancyGrid();
  cost_grid.header = static_grid.header;
  cost_grid.info = static_grid.info;
  const int outside_cost = std::max(-1, std::min(99, config_.channel_outside_cost));
  std::vector<int8_t> cost_data(cell_count, static_cast<int8_t>(outside_cost));
  const double channel_radius = std::max(0.0, config_.channel_corridor_width);
  for (const auto & feature : chart.features) {
    if (kChannelGroups.count(upper(feature.group)) == 0) {
      continue;
    }
    for (const auto & geometry : feature.geometry) {
      for (const auto & part : geometry.parts) {
        for_each_pair(part, [&](const Point & start, const Point & end) {
          raster_segment(cost_data, shape, start, end, 0, channel_radius);
        });
      }
    }
  }
  for (std::size_t i = 0; i < static_data.size(); ++i) {
    if (static_data[i] == 100) {
      cost_data[i] = 100;
    }
  }
  // 动态障碍物叠加为不可通行。
  for (const auto & obstacle : future_obstacles) {
    raster_segment(
      cost_data, shape, obstacle, obstacle, 100, config_.dynamic_obstacle_radius);
  }

  cost_grid.data = std::move(cost_data);
  static_grid.data = std::move(static_data);
}

}  // namespace river_chart
