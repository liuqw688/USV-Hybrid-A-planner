/**
 * @file reef_chart.cpp
 * @brief 光滑凸暗礁生成、坐标转换与 XML 持久化。
 */

#include "reef_chart.hpp"
#include "xml_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace usv_map_tools
{
namespace
{
using usv_map::Point;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 6378137.0;
constexpr std::size_t kSamples = 512;
constexpr const char * kGenerator = "RVIZ_REEF_DESIGNER";

Point centroid(const std::vector<Point> & ring)
{
  // 先减去附近参考点，减少大坐标计算面积重心时的浮点消减误差。
  const auto origin = ring.front();
  long double area = 0, x = 0, y = 0;
  for (std::size_t i = 1; i < ring.size(); ++i) {
    const Point a{ring[i - 1].x - origin.x, ring[i - 1].y - origin.y};
    const Point b{ring[i].x - origin.x, ring[i].y - origin.y};
    const long double cross = static_cast<long double>(a.x) * b.y -
      static_cast<long double>(b.x) * a.y;
    area += cross;
    x += (a.x + b.x) * cross;
    y += (a.y + b.y) * cross;
  }
  if (!std::isfinite(area) || area <= 0) {
    throw std::runtime_error("reef boundary must have positive area");
  }
  return {origin.x + static_cast<double>(x / (3 * area)),
    origin.y + static_cast<double>(y / (3 * area))};
}

std::array<double, 2> radii(const std::vector<Point> & ring, double stretch)
{
  double minimum = INFINITY, maximum = 0;
  for (std::size_t i = 1; i < ring.size(); ++i) {
    const Point a{ring[i - 1].x * stretch, ring[i - 1].y};
    const Point b{ring[i].x * stretch, ring[i].y};
    const Point d{b.x - a.x, b.y - a.y};
    const double t = std::clamp(-(a.x * d.x + a.y * d.y) / (d.x * d.x + d.y * d.y), 0.0, 1.0);
    minimum = std::min(minimum, std::hypot(a.x + t * d.x, a.y + t * d.y));
    maximum = std::max(maximum, std::hypot(a.x, a.y));
  }
  return {minimum, maximum};
}

std::string number(double value) {return xml::number(value, 12);}

using xml::text;
}  // namespace

Reef make_reef(Point center, uint64_t seed, double min_radius_m, double max_radius_m)
{
  if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
    !std::isfinite(min_radius_m) || !std::isfinite(max_radius_m) ||
    min_radius_m <= 0 || max_radius_m <= min_radius_m)
  {
    throw std::invalid_argument("centre must be finite; require 0 < min_radius_m < max_radius_m");
  }
  // 给源坐标舍入保留 0.1 mm 裕量，半径约束包含完整边界线段。
  const double margin = std::min(0.0001, (max_radius_m - min_radius_m) * 0.001);
  const double target = (max_radius_m - margin) / (min_radius_m + margin);
  if (!std::isfinite(target) || target <= 1.0 / std::cos(kPi / kSamples)) {
    throw std::invalid_argument("radius range is too narrow for a sampled irregular contour");
  }
  std::mt19937_64 random(seed);
  const auto uniform = [&random]() {return std::generate_canonical<double, 53>(random);};
  std::array<double, 3> weights{}, phases{};
  double sum = 0;
  for (std::size_t i = 0; i < weights.size(); ++i) {
    sum += weights[i] = 0.5 + uniform();
    phases[i] = 2 * kPi * uniform();
  }
  const double budget = std::min(0.9, (target - 1) * 1.5);
  std::vector<Point> ring;
  // 支撑函数满足 h+h''>0 时轮廓严格凸且光滑。
  // 混合随机谐波打破对称性；限制曲率预算，确保所有随机种子均满足凸性。
  for (double strength = budget; ; strength *= 0.5) {
    ring.clear();
    for (std::size_t i = 0; i < kSamples; ++i) {
      const double angle = 2 * kPi * i / kSamples;
      double h = 1, derivative = 0;
      for (std::size_t j = 0; j < weights.size(); ++j) {
        const double k = j + 3;
        const double amplitude = strength * weights[j] / (sum * (k * k - 1));
        h += amplitude * std::cos(k * angle + phases[j]);
        derivative -= k * amplitude * std::sin(k * angle + phases[j]);
      }
      ring.push_back({h * std::cos(angle) - derivative * std::sin(angle),
        h * std::sin(angle) + derivative * std::cos(angle)});
    }
    ring.push_back(ring.front());
    const auto c = centroid(ring);
    for (auto & p : ring) {p.x -= c.x; p.y -= c.y;}
    const auto r = radii(ring, 1);
    if (r[1] / r[0] < target) {break;}
  }
  // 仿射拉伸保持凸性、光滑性及面积重心。
  double low = 1, high = 2;
  while (true) {
    const auto r = radii(ring, high);
    if (r[1] / r[0] >= target) {break;}
    high *= 2;
    if (!std::isfinite(high)) {throw std::invalid_argument("radius ratio is too large");}
  }
  for (int i = 0; i < 60; ++i) {
    const double middle = (low + high) / 2;
    const auto r = radii(ring, middle);
    if (r[1] / r[0] < target) {low = middle;} else {high = middle;}
  }
  const double stretch = (low + high) / 2;
  const double scale = (max_radius_m - margin) / radii(ring, stretch)[1];
  const double angle = 2 * kPi * uniform(), c = std::cos(angle), s = std::sin(angle);
  for (auto & p : ring) {
    const double x = p.x * stretch * scale, y = p.y * scale;
    p = {center.x + c * x - s * y, center.y + s * x + c * y};
  }
  return {0, center, std::move(ring)};
}

ReefChart::ReefChart(const std::filesystem::path & input)
: chart_(usv_map::load_chart(input.string()))
{
  if (chart_.has_unreadable_navigation_geometry) {
    throw std::runtime_error("input chart has unreadable water/obstacle geometry");
  }
  if (document_.LoadFile(input.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot load chart XML: " + input.string());
  }
  auto * first_map = document_.RootElement()->FirstChildElement("Map");
  features_ = first_map->FirstChildElement("Features");
  if (!features_) {
    features_ = document_.NewElement("Features");
    first_map->InsertEndChild(features_);
  }
  for (const auto * map = first_map; map; map = map->NextSiblingElement("Map")) {
    const auto * features = map->FirstChildElement("Features");
    for (const auto * group = features ? features->FirstChildElement("FeatureGroup") : nullptr;
      group; group = group->NextSiblingElement("FeatureGroup"))
    {
      for (const auto * feature = group->FirstChildElement("Feature"); feature;
        feature = feature->NextSiblingElement("Feature"))
      {
        for (const char * tag : {"ID", "RCID"}) {
          const auto * id = feature->FirstChildElement(tag);
          int value = 0;
          if (id && id->QueryIntText(&value) == tinyxml2::XML_SUCCESS) {
            if (value == std::numeric_limits<int>::max()) {
              throw std::runtime_error("chart feature ID limit reached");
            }
            next_id_ = std::max(next_id_, value + 1);
          }
        }
      }
    }
  }
  for (const auto & feature : chart_.features) {
    const auto generator = feature.attributes.find("GENERATOR");
    if (feature.group != "LNDARE" || generator == feature.attributes.end() ||
      generator->second != kGenerator)
    {
      continue;
    }
    if (feature.geometry.size() != 1 || feature.geometry.front().type != "POLYGON" ||
      feature.geometry.front().parts.size() != 1)
    {
      throw std::runtime_error("invalid saved reef polygon");
    }
    const auto & boundary = feature.geometry.front().parts.front();
    reefs_.push_back({std::stoi(feature.feature_id), centroid(boundary), boundary});
  }
}

Point ReefChart::geographic(Point p) const
{
  return {longitude() + p.x / (kRadius * std::cos(latitude() * kPi / 180)) * 180 / kPi,
    latitude() + p.y / kRadius * 180 / kPi};
}

Point ReefChart::source(Point p) const
{
  const auto g = geographic(p);
  return chart_.source_is_projected ?
    Point{kRadius * g.x * kPi / 180, -kRadius * std::asinh(std::tan(g.y * kPi / 180))} : g;
}

Reef ReefChart::add(Point center, uint64_t seed, double minimum, double maximum,
  const std::filesystem::path & output)
{
  auto reef = make_reef(center, seed, minimum, maximum);
  for (const auto p : reef.boundary) {
    const auto g = geographic(p);
    if (g.x < chart_.bounds[0] || g.x > chart_.bounds[2] ||
      g.y < chart_.bounds[1] || g.y > chart_.bounds[3])
    {
      throw std::invalid_argument("the entire reef must be inside the chart Bounds");
    }
  }
  if (next_id_ == std::numeric_limits<int>::max()) {
    throw std::runtime_error("chart feature ID limit reached");
  }
  reef.id = next_id_;
  // 修改文档前保存待恢复的标记状态，写文件失败时可回滚。
  reefs_.push_back(reef);
  auto * group = features_->FirstChildElement("FeatureGroup");
  while (group && !group->Attribute("acronym", "LNDARE")) {
    group = group->NextSiblingElement("FeatureGroup");
  }
  const bool new_group = !group;
  if (!group) {
    group = document_.NewElement("FeatureGroup");
    group->SetAttribute("acronym", "LNDARE");
    group->SetAttribute("priority", "2");
    features_->InsertEndChild(group);
  }
  const bool had_count = group->Attribute("count") != nullptr;
  const std::string old_count = had_count ? group->Attribute("count") : "";
  int count = 0;
  for (auto * item = group->FirstChildElement("Feature"); item;
    item = item->NextSiblingElement("Feature")) {++count;}
  auto * feature = document_.NewElement("Feature");
  group->InsertEndChild(feature);
  try {
    const auto name = "RVIZ_REEF_" + std::to_string(reef.id);
    text(feature, "RCID", std::to_string(reef.id));
    text(feature, "ID", std::to_string(reef.id));
    text(feature, "Name", name);
    text(feature, "Type", "A");
    text(feature, "TableName", "O");
    auto * attributes = document_.NewElement("Attributes");
    feature->InsertEndChild(attributes);
    text(attributes, "OBJNAM", name);
    text(attributes, "GENERATOR", kGenerator);
    text(attributes, "OBSTACLE_KIND", "REEF");
    text(attributes, "INFORM", "Synthetic reef; smooth convex contour sampled as a closed polygon");
    const auto g = geographic(center);
    text(attributes, "CENTER_LON", number(g.x));
    text(attributes, "CENTER_LAT", number(g.y));
    text(attributes, "MIN_RADIUS_M", number(minimum));
    text(attributes, "MAX_RADIUS_M", number(maximum));
    text(attributes, "SHAPE_SEED", std::to_string(seed));
    xml::polygon(feature, reef.boundary, [this](Point p) {return source(p);}, 12);
    group->SetAttribute("count", count + 1);
    save(output);
  } catch (...) {
    reefs_.pop_back();
    group->DeleteChild(feature);
    if (new_group) {features_->DeleteChild(group);} else if (had_count) {
      group->SetAttribute("count", old_count.c_str());
    } else {group->DeleteAttribute("count");}
    throw;
  }
  ++next_id_;
  return reef;
}

void ReefChart::save(const std::filesystem::path & output)
{
  xml::save_atomic(document_, output);
}
}  // namespace usv_map_tools
