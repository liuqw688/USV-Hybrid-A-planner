/**
 * @file fairway_chart.cpp
 * @brief 读写闭合航线、航道边界与双向通行属性。
 */

#include "fairway_chart.hpp"
#include "xml_utils.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace usv_map_tools
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 6378137.0;
constexpr const char * kGenerator = "GNSS_SMOOTH_FAIRWAY";

using xml::number;

using xml::text;

RoutePoint source_point(RoutePoint point, const usv_map::Chart & chart)
{
  const double latitude = chart.origin_latitude * kPi / 180 + point.y / kRadius;
  const double longitude = chart.origin_longitude * kPi / 180 +
    point.x / (kRadius * std::cos(chart.origin_latitude * kPi / 180));
  if (chart.source_is_projected) {
    return {kRadius * longitude, -kRadius * std::asinh(std::tan(latitude))};
  }
  return {longitude * 180 / kPi, latitude * 180 / kPi};
}

void geometry(tinyxml2::XMLElement * feature, const std::vector<RoutePoint> & points,
  const usv_map::Chart & chart)
{
  xml::polygon(feature, points, [&chart](RoutePoint p) {return source_point(p, chart);});
}

tinyxml2::XMLElement * feature(tinyxml2::XMLElement * group, int id, int priority,
  const char * instruction, bool area)
{
  auto * item = group->GetDocument()->NewElement("Feature");
  group->InsertEndChild(item);
  text(item, "RCID", std::to_string(id));
  text(item, "ID", std::to_string(id));
  text(item, "Name", group->Attribute("acronym"));
  text(item, "Type", area ? "A" : "L");
  text(item, "TableName", area ? "O" : "S");
  auto * display = item->GetDocument()->NewElement("DisplayProperties");
  item->InsertEndChild(display);
  text(display, "Priority", std::to_string(priority));
  text(display, "Category", "S");
  text(display, "RadarPriority", area ? "S" : "O");
  text(display, "Instruction", instruction);
  text(display, "Comment", area ? "25010" : "15010");
  auto * attributes = item->GetDocument()->NewElement("Attributes");
  item->InsertEndChild(attributes);
  for (const char * name : {"CATTSS", "DATEND", "DATSTA", "STATUS"}) {
    text(attributes, name, "");
  }
  text(attributes, "INFORM", kGenerator);
  return item;
}
}  // namespace

Fairway load_saved_fairway(const usv_map::Chart & chart)
{
  Fairway fairway{};
  // 已存航道可能新增了障碍，恢复显示时不声明安全净距。
  fairway.minimum_clearance_m = fairway.centerline_clearance_m =
    std::numeric_limits<double>::quiet_NaN();
  std::vector<std::vector<RoutePoint>> boundaries;
  std::vector<RoutePoint> center;
  bool has_clockwise = false, has_counterclockwise = false;
  const auto points = [](const usv_map::ChartFeature & feature) {
      if (feature.geometry.size() != 1 || feature.geometry.front().parts.size() != 1) {
        throw std::runtime_error("saved fairway must have one contour per feature");
      }
      std::vector<RoutePoint> result;
      for (const auto p : feature.geometry.front().parts.front()) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
          throw std::runtime_error("saved fairway has nonfinite coordinates");
        }
        result.push_back({p.x, p.y});
      }
      if (result.size() < 4 ||
        std::hypot(result.front().x - result.back().x, result.front().y - result.back().y) > 2e-5)
      {
        throw std::runtime_error("saved fairway contour is not closed");
      }
      return result;
    };
  for (const auto & feature : chart.features) {
    const auto inform = feature.attributes.find("INFORM");
    if (inform == feature.attributes.end() || inform->second != kGenerator) {continue;}
    if (feature.group == "TSELNE") {
      if (!center.empty()) {throw std::runtime_error("saved fairway has multiple centrelines");}
      center = points(feature);
    } else if (feature.group == "TSSBND") {
      boundaries.push_back(points(feature));
    } else if (feature.group == "TSSLPT") {
      const auto & direction = feature.attributes.at("DIRECTION");
      if (direction != "CW" && direction != "CCW") {
        throw std::runtime_error("saved fairway has an invalid lane direction");
      }
      const bool clockwise = direction == "CW";
      has_clockwise |= clockwise;
      has_counterclockwise |= !clockwise;
      const auto width = std::stod(feature.attributes.at("WIDTH_M"));
      const auto heading = std::stod(feature.attributes.at("ORIENT"));
      if (!std::isfinite(width) || width <= 0 || !std::isfinite(heading) ||
        (!fairway.sections.empty() && std::abs(width - fairway.lane_width_m) > 1e-6))
      {
        throw std::runtime_error("saved fairway has invalid or inconsistent lane widths/headings");
      }
      fairway.lane_width_m = width;
      fairway.sections.push_back({points(feature), (90 - heading) * kPi / 180, clockwise});
    }
  }
  // 导出顺序固定为内边界在前、外边界在后。
  if (center.empty() || boundaries.size() != 2 || !has_clockwise || !has_counterclockwise ||
    boundaries[0].size() != center.size() || boundaries[1].size() != center.size())
  {
    throw std::runtime_error("saved fairway is missing its centreline, boundaries or directional lanes");
  }
  fairway.inner = std::move(boundaries[0]);
  fairway.outer = std::move(boundaries[1]);
  for (std::size_t i = 0; i < center.size(); ++i) {
    const auto c = center[i], inner = fairway.inner[i], outer = fairway.outer[i];
    if (std::abs(std::hypot(inner.x - c.x, inner.y - c.y) - fairway.lane_width_m) > 1e-3 ||
      std::abs(inner.x + outer.x - 2 * c.x) > 1e-3 ||
      std::abs(inner.y + outer.y - 2 * c.y) > 1e-3)
    {
      throw std::runtime_error("saved fairway boundaries do not match its centreline and width");
    }
    const double yaw = std::atan2(inner.x - c.x, c.y - inner.y);
    fairway.center.push_back({c, yaw});
    fairway.clockwise.push_back({{(c.x + inner.x) / 2, (c.y + inner.y) / 2}, yaw});
    fairway.counterclockwise.push_back({{(c.x + outer.x) / 2, (c.y + outer.y) / 2},
      std::remainder(yaw + kPi, 2 * kPi)});
  }
  std::reverse(fairway.counterclockwise.begin(), fairway.counterclockwise.end());
  return fairway;
}

void save_fairway_chart(
  const std::filesystem::path & input, const std::filesystem::path & output,
  const usv_map::Chart & chart, const Fairway & fairway, double edge_clearance_m)
{
  if (std::filesystem::weakly_canonical(input) == std::filesystem::weakly_canonical(output)) {
    throw std::invalid_argument("fairway output must differ from source chart");
  }
  tinyxml2::XMLDocument document;
  if (document.LoadFile(input.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot read source XML for fairway export");
  }
  auto * root = document.RootElement();
  auto * map = root ? root->FirstChildElement("Map") : nullptr;
  auto * features = map ? map->FirstChildElement("Features") : nullptr;
  if (!features || fairway.sections.empty()) {
    throw std::runtime_error("source chart or fairway has no features");
  }
  int next_id = 1;
  // 只替换本工具上次生成的要素，保留手工航道。
  for (auto * m = map; m; m = m->NextSiblingElement("Map")) {
    auto * fs = m->FirstChildElement("Features");
    for (auto * group = fs ? fs->FirstChildElement("FeatureGroup") : nullptr;
      group; group = group->NextSiblingElement("FeatureGroup"))
    {
      int count = 0;
      const bool navigation = group->Attribute("acronym", "TSSLPT") ||
        group->Attribute("acronym", "TSSBND") || group->Attribute("acronym", "TSELNE");
      for (auto * item = group->FirstChildElement("Feature"); item; ) {
        auto * next = item->NextSiblingElement("Feature");
        const auto * attributes = item->FirstChildElement("Attributes");
        const auto * inform = attributes ? attributes->FirstChildElement("INFORM") : nullptr;
        if (navigation && inform && inform->GetText() &&
          std::string(inform->GetText()) == kGenerator)
        {
          group->DeleteChild(item);
        } else {
          ++count;
          for (const char * name : {"ID", "RCID"}) {
            int value = 0;
            const auto * id = item->FirstChildElement(name);
            if (id && id->QueryIntText(&value) == tinyxml2::XML_SUCCESS) {
              if (value > std::numeric_limits<int>::max() - 10000) {
                throw std::runtime_error("chart feature ID limit reached");
              }
              next_id = std::max(next_id, value + 1);
            }
          }
        }
        item = next;
      }
      if (navigation) {group->SetAttribute("count", count);}
    }
  }
  const auto get_group = [&](const char * acronym, int priority) {
      for (auto * group = features->FirstChildElement("FeatureGroup"); group;
        group = group->NextSiblingElement("FeatureGroup"))
      {
        if (group->Attribute("acronym", acronym)) {return group;}
      }
      auto * group = document.NewElement("FeatureGroup");
      group->SetAttribute("acronym", acronym);
      group->SetAttribute("priority", priority);
      group->SetAttribute("count", 0);
      features->InsertEndChild(group);
      return group;
    };
  auto * lanes = get_group("TSSLPT", 6);
  for (const auto & section : fairway.sections) {
    auto * item = feature(lanes, next_id++, 6, "SY(TSSLPT51,ORIENT);CS(RESTRN01)", true);
    auto * attributes = item->FirstChildElement("Attributes");
    const double heading = std::fmod(90 - section.yaw * 180 / kPi + 720, 360);
    text(attributes, "ORIENT", number(heading));
    text(attributes, "RESTRN", "");
    text(attributes, "DIRECTION", section.clockwise ? "CW" : "CCW");
    text(attributes, "WIDTH_M", number(fairway.lane_width_m));
    text(attributes, "EDGE_CLEARANCE_M", number(edge_clearance_m));
    geometry(item, section.polygon, chart);
    lanes->SetAttribute("count", lanes->IntAttribute("count") + 1);
  }
  auto * boundaries = get_group("TSSBND", 7);
  for (const auto * points : {&fairway.inner, &fairway.outer}) {
    auto * item = feature(boundaries, next_id++, 7, "LS(DASH,2,TRFCD)", false);
    geometry(item, *points, chart);
    boundaries->SetAttribute("count", boundaries->IntAttribute("count") + 1);
  }
  auto * separation = get_group("TSELNE", 8);
  auto * item = feature(separation, next_id++, 8, "LS(SOLD,6,TRFCF)", false);
  std::vector<RoutePoint> center;
  for (const auto & pose : fairway.center) {center.push_back(pose.position);}
  geometry(item, center, chart);
  separation->SetAttribute("count", separation->IntAttribute("count") + 1);
  xml::save_atomic(document, output);
}
}  // namespace usv_map_tools
