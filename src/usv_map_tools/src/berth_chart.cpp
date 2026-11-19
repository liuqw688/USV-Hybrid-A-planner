/**
 * @file berth_chart.cpp
 * @brief 泊位几何计算、历史要素合并与 XML 持久化。
 */

#include "berth_chart.hpp"
#include "xml_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace usv_map_tools
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kEarthRadius = 6378137.0;

using xml::number;

double read_number(const tinyxml2::XMLElement * parent, const char * tag)
{
  const auto * element = parent ? parent->FirstChildElement(tag) : nullptr;
  double value = 0.0;
  if (!element || element->QueryDoubleText(&value) != tinyxml2::XML_SUCCESS ||
    !std::isfinite(value))
  {
    throw std::runtime_error(std::string("missing or invalid XML field: ") + tag);
  }
  return value;
}

using xml::text;

// 与主解析器一致，识别 Y 轴取负的 Web Mercator 源坐标。
bool projected_geometry(const tinyxml2::XMLElement * element, bool & found)
{
  if (!element) {
    return false;
  }
  const char * raw = element->GetText();
  if (raw) {
    const std::string text(raw);
    const auto bracket = text.find('(');
    const auto start = text.find_first_of("-+0123456789.", bracket);
    if (bracket != std::string::npos && start != std::string::npos) {
      double x = 0.0;
      double y = 0.0;
      std::istringstream stream(text.substr(start));
      if (stream >> x >> y) {
        found = true;
        return std::max(std::abs(x), std::abs(y)) > 1000.0;
      }
    }
  }
  for (const auto * child = element->FirstChildElement(); child;
    child = child->NextSiblingElement())
  {
    const bool projected = projected_geometry(child, found);
    if (found) {
      return projected;
    }
  }
  return false;
}
}  // namespace

std::array<BerthPoint, 5> Berth::corners() const
{
  if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(yaw) ||
    !std::isfinite(length) || !std::isfinite(width) || length <= 0.0 || width <= 0.0)
  {
    throw std::invalid_argument("berth pose must be finite; length_m and width_m must be positive");
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::array<BerthPoint, 5> result{{
    {-length / 2, -width / 2}, {length / 2, -width / 2},
    {length / 2, width / 2}, {-length / 2, width / 2}, {-length / 2, -width / 2}}};
  for (auto & point : result) {
    point = {center.x + c * point.x - s * point.y, center.y + s * point.x + c * point.y};
  }
  return result;
}

BerthChart::BerthChart(const std::filesystem::path & input, const std::filesystem::path & base)
{
  const auto & source = base.empty() ? input : base;
  const bool merge_berths = !base.empty() &&
    std::filesystem::weakly_canonical(input) != std::filesystem::weakly_canonical(base);
  if (document_.LoadFile(source.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot load chart XML: " + source.string());
  }
  auto * root = document_.RootElement();
  auto * first_map = root ? root->FirstChildElement("Map") : nullptr;
  if (!first_map) {
    throw std::runtime_error("chart XML has no Map element");
  }
  const double infinity = std::numeric_limits<double>::infinity();
  bounds_ = {infinity, infinity, -infinity, -infinity};
  bool found_geometry = false;
  std::set<int> used_ids;
  for (auto * map = first_map; map; map = map->NextSiblingElement("Map")) {
    const auto * bounds = map->FirstChildElement("Bounds");
    const double min_lon = read_number(bounds, "MinX");
    const double min_lat = read_number(bounds, "MinY");
    const double max_lon = read_number(bounds, "MaxX");
    const double max_lat = read_number(bounds, "MaxY");
    if (min_lon >= max_lon || min_lat >= max_lat || min_lon < -180 || max_lon > 180 ||
      min_lat <= -85.0 || max_lat >= 85.0)
    {
      throw std::runtime_error("invalid geographic chart Bounds");
    }
    bounds_[0] = std::min(bounds_[0], min_lon);
    bounds_[1] = std::min(bounds_[1], min_lat);
    bounds_[2] = std::max(bounds_[2], max_lon);
    bounds_[3] = std::max(bounds_[3], max_lat);
    auto * features = map->FirstChildElement("Features");
    if (merge_berths && features) {
      // 保留最新基础地图，仅用已保存泊位替换对应要素。
      for (auto * group = features->FirstChildElement("FeatureGroup"); group; ) {
        auto * next = group->NextSiblingElement("FeatureGroup");
        if (group->Attribute("acronym", "BERTHS")) {
          features->DeleteChild(group);
        }
        group = next;
      }
    }
    if (!found_geometry) {
      projected_ = projected_geometry(features, found_geometry);
    }
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
            used_ids.insert(value);
          }
        }
      }
    }
  }
  longitude_ = (bounds_[0] + bounds_[2]) * 0.5;
  latitude_ = (bounds_[1] + bounds_[3]) * 0.5;
  features_ = first_map->FirstChildElement("Features");
  if (!features_) {
    features_ = document_.NewElement("Features");
    first_map->InsertEndChild(features_);
  }

  if (merge_berths) {
    const BerthChart saved(input);
    for (std::size_t i = 0; i < bounds_.size(); ++i) {
      if (std::abs(bounds_[i] - saved.bounds_[i]) > 1e-9) {
        throw std::runtime_error("saved berths and base chart must have the same geographic Bounds");
      }
    }
    if (projected_ != saved.projected_) {
      throw std::runtime_error("saved berths and base chart must use the same source coordinates");
    }
    next_id_ = std::max(next_id_, saved.next_id_);
    for (const auto * map = saved.document_.RootElement()->FirstChildElement("Map"); map;
      map = map->NextSiblingElement("Map"))
    {
      const auto * features = map->FirstChildElement("Features");
      for (const auto * group = features ? features->FirstChildElement("FeatureGroup") : nullptr;
        group; group = group->NextSiblingElement("FeatureGroup"))
      {
        if (!group->Attribute("acronym", "BERTHS")) {
          continue;
        }
        auto * copy = group->DeepClone(&document_)->ToElement();
        features_->InsertEndChild(copy);
        int count = 0;
        for (auto * feature = copy->FirstChildElement("Feature"); feature;
          feature = feature->NextSiblingElement("Feature"))
        {
          ++count;
          bool conflict = false;
          for (const char * tag : {"ID", "RCID"}) {
            const auto * id = feature->FirstChildElement(tag);
            int value = 0;
            conflict = conflict || (id && id->QueryIntText(&value) == tinyxml2::XML_SUCCESS &&
              used_ids.count(value) != 0);
          }
          if (conflict) {
            if (next_id_ == std::numeric_limits<int>::max()) {
              throw std::runtime_error("chart feature ID limit reached");
            }
            const int replacement = next_id_++;
            for (const char * tag : {"ID", "RCID"}) {
              auto * id = feature->FirstChildElement(tag);
              if (id) {
                id->SetText(replacement);
              } else {
                text(feature, tag, std::to_string(replacement));
              }
            }
            auto * attributes = feature->FirstChildElement("Attributes");
            for (auto * name : {feature->FirstChildElement("Name"),
                attributes ? attributes->FirstChildElement("OBJNAM") : nullptr})
            {
              if (name && name->GetText() && std::string(name->GetText()).find("RVIZ_BERTH_") == 0) {
                name->SetText(("RVIZ_BERTH_" + std::to_string(replacement)).c_str());
              }
            }
          }
          for (const char * tag : {"ID", "RCID"}) {
            const auto * id = feature->FirstChildElement(tag);
            int value = 0;
            if (id && id->QueryIntText(&value) == tinyxml2::XML_SUCCESS) {
              used_ids.insert(value);
            }
          }
        }
        copy->SetAttribute("count", count);
      }
    }
  }

  // 恢复本工具保存的泊位标记，叠加在海图轮廓上。
  for (const auto * group = features_->FirstChildElement("FeatureGroup"); group;
    group = group->NextSiblingElement("FeatureGroup"))
  {
    if (!group->Attribute("acronym", "BERTHS")) {
      continue;
    }
    for (const auto * feature = group->FirstChildElement("Feature"); feature;
      feature = feature->NextSiblingElement("Feature"))
    {
      const auto * attributes = feature->FirstChildElement("Attributes");
      if (!attributes || !attributes->FirstChildElement("LENGTH_M")) {
        continue;
      }
      const double lon = read_number(attributes, "CENTER_LON");
      const double lat = read_number(attributes, "CENTER_LAT");
      Berth berth{
        static_cast<int>(read_number(feature, "ID")),
        {(lon - longitude_) * kDegToRad * kEarthRadius * std::cos(latitude_ * kDegToRad),
          (lat - latitude_) * kDegToRad * kEarthRadius},
        (90.0 - read_number(attributes, "ORIENT")) * kDegToRad,
        read_number(attributes, "LENGTH_M"), read_number(attributes, "WIDTH_M")};
      berth.corners();
      berths_.push_back(berth);
    }
  }
}

BerthPoint BerthChart::to_geographic(BerthPoint point) const
{
  return {
    longitude_ + point.x / (kEarthRadius * std::cos(latitude_ * kDegToRad)) / kDegToRad,
    latitude_ + point.y / kEarthRadius / kDegToRad};
}

BerthPoint BerthChart::to_source(BerthPoint point) const
{
  const auto geographic = to_geographic(point);
  if (!projected_) {
    return geographic;
  }
  return {
    kEarthRadius * geographic.x * kDegToRad,
    -kEarthRadius * std::log(std::tan(kPi / 4.0 + geographic.y * kDegToRad / 2.0))};
}

Berth BerthChart::add(Berth berth, const std::filesystem::path & output)
{
  const auto corners = berth.corners();
  for (const auto & corner : corners) {
    const auto geographic = to_geographic(corner);
    if (geographic.x < bounds_[0] || geographic.x > bounds_[2] ||
      geographic.y < bounds_[1] || geographic.y > bounds_[3])
    {
      throw std::invalid_argument("the entire berth must be inside the chart Bounds");
    }
  }
  if (next_id_ == std::numeric_limits<int>::max()) {
    throw std::runtime_error("chart feature ID limit reached");
  }
  auto * group = features_->FirstChildElement("FeatureGroup");
  while (group && !group->Attribute("acronym", "BERTHS")) {
    group = group->NextSiblingElement("FeatureGroup");
  }
  const bool new_group = !group;
  if (!group) {
    group = document_.NewElement("FeatureGroup");
    group->SetAttribute("acronym", "BERTHS");
    group->SetAttribute("priority", "3");
    features_->InsertEndChild(group);
  }
  int count = 0;
  for (auto * feature = group->FirstChildElement("Feature"); feature;
    feature = feature->NextSiblingElement("Feature"))
  {
    ++count;
  }
  const bool had_count = group->Attribute("count") != nullptr;
  const std::string previous_count = had_count ? group->Attribute("count") : "";
  auto * feature = document_.NewElement("Feature");
  group->InsertEndChild(feature);
  berth.id = next_id_;
  text(feature, "RCID", std::to_string(berth.id));
  text(feature, "ID", std::to_string(berth.id));
  text(feature, "Name", "RVIZ_BERTH_" + std::to_string(berth.id));
  text(feature, "Type", "A");
  text(feature, "TableName", "O");
  auto * attributes = document_.NewElement("Attributes");
  feature->InsertEndChild(attributes);
  text(attributes, "OBJNAM", "RVIZ_BERTH_" + std::to_string(berth.id));
  text(attributes, "INFORM", "Designed in RViz2; dimensions in metres; orientation clockwise from north");
  text(attributes, "LENGTH_M", number(berth.length));
  text(attributes, "WIDTH_M", number(berth.width));
  const double heading = std::fmod(90.0 - std::fmod(berth.yaw, 2.0 * kPi) / kDegToRad + 360.0, 360.0);
  text(attributes, "ORIENT", number(heading));
  const auto center = to_geographic(berth.center);
  text(attributes, "CENTER_LON", number(center.x));
  text(attributes, "CENTER_LAT", number(center.y));
  xml::polygon(feature, corners, [this](BerthPoint p) {return to_source(p);});
  group->SetAttribute("count", count + 1);
  try {
    save(output);
  } catch (...) {
    group->DeleteChild(feature);
    if (new_group) {
      features_->DeleteChild(group);
    } else if (had_count) {
      group->SetAttribute("count", previous_count.c_str());
    } else {
      group->DeleteAttribute("count");
    }
    throw;
  }
  ++next_id_;
  berths_.push_back(berth);
  return berth;
}

void BerthChart::save(const std::filesystem::path & output)
{
  xml::save_atomic(document_, output);
}
}  // namespace usv_map_tools
