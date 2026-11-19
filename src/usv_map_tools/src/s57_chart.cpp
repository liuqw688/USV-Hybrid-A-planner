/**
 * @file s57_chart.cpp
 * @brief 将海图要素写为 S-57 并回读校验输出。
 */

#include "s57_chart.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cpl_json.h>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

namespace usv_map_tools
{
namespace
{

using GeometryPtr = std::unique_ptr<OGRGeometry>;
using FeaturePtr = std::unique_ptr<OGRFeature>;
using DatasetPtr = std::unique_ptr<GDALDataset, decltype(&GDALClose)>;

constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
constexpr double kMetersPerDegree = 6378137.0 * kRadiansPerDegree;
constexpr double kCoordinateFactor = 10000000.0;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Object
{
  std::string group;
  std::string name;
  std::string source_rcid;
  std::string source_id;
  std::map<std::string, std::string> attributes;
  GeometryPtr geometry;
  std::vector<int> ids;
  std::vector<int> usages;
  int primitive{3};  // S-57 PRIM: 1 点、2 线、3 面。
};

// 恢复 load_chart 转换前的经纬度，再量化到 S-57 的 1e-7 度精度。
OGRPoint geographic_point(const usv_map::Point & point, const usv_map::Chart & chart)
{
  const double lon = chart.origin_longitude +
    point.x / (kMetersPerDegree * std::cos(chart.origin_latitude * kRadiansPerDegree));
  const double lat = chart.origin_latitude + point.y / kMetersPerDegree;
  require(std::isfinite(lon) && std::isfinite(lat) &&
    std::abs(lon) <= 180.0 && std::abs(lat) <= 90.0, "Invalid geographic coordinate");
  return OGRPoint(std::round(lon * kCoordinateFactor) / kCoordinateFactor,
    std::round(lat * kCoordinateFactor) / kCoordinateFactor);
}

GeometryPtr make_geometry(const usv_map::ChartFeature & feature, const usv_map::Chart & chart)
{
  auto collection = std::make_unique<OGRGeometryCollection>();
  for (const auto & geometry : feature.geometry) {
    const bool points = feature.feature_type == "P" || geometry.type == "POINT" ||
      geometry.type == "MULTIPOINT";
    const bool lines = feature.feature_type == "L" || geometry.type == "LINESTRING" ||
      geometry.type == "MULTILINESTRING";
    OGRPolygon * polygon = nullptr;
    for (std::size_t i = 0; i < geometry.parts.size(); ++i) {
      const auto & part = geometry.parts[i];
      require(!part.empty(), "Empty geometry part in " + feature.group);
      if (points) {
        for (const auto & point : part) {
          collection->addGeometryDirectly(new OGRPoint(geographic_point(point, chart)));
        }
        continue;
      }
      if (lines) {
        auto line = std::make_unique<OGRLineString>();
        std::size_t size = part.size();
        // XML 导出器把线包装为多边形；移除它额外添加的闭合点。
        if ((geometry.type == "POLYGON" || geometry.type == "MULTIPOLYGON") && size > 2 &&
          part.front().x == part.back().x && part.front().y == part.back().y)
        {
          --size;
        }
        require(size >= 2, "Line has fewer than two vertices");
        for (std::size_t j = 0; j < size; ++j) {
          const auto point = geographic_point(part[j], chart);
          line->addPoint(&point);
        }
        collection->addGeometryDirectly(line.release());
        continue;
      }
      require(geometry.type == "POLYGON" || geometry.type == "MULTIPOLYGON",
        "Unsupported geometry type: " + geometry.type);
      const bool exterior = i == 0 || std::find(geometry.polygon_starts.begin(),
        geometry.polygon_starts.end(), i) != geometry.polygon_starts.end();
      if (exterior) {
        polygon = new OGRPolygon();
        collection->addGeometryDirectly(polygon);
      }
      auto ring = std::make_unique<OGRLinearRing>();
      for (const auto & point : part) {
        const auto geographic = geographic_point(point, chart);
        ring->addPoint(&geographic);
      }
      ring->closeRings();
      require(ring->getNumPoints() >= 4, "Area has fewer than three vertices");
      // S-57 面沿外环顺时针、内环逆时针编码。
      if (static_cast<bool>(ring->isClockwise()) != exterior) {
        ring->reversePoints();
      }
      polygon->addRingDirectly(ring.release());
    }
  }
  require(collection->getNumGeometries() > 0, "Feature has no geometry");
  if (collection->getNumGeometries() == 1) {
    return GeometryPtr(collection->getGeometryRef(0)->clone());
  }
  const auto type = wkbFlatten(collection->getGeometryRef(0)->getGeometryType());
  if (type == wkbPolygon) {
    return GeometryPtr(OGRGeometryFactory::forceToMultiPolygon(collection.release()));
  }
  if (type == wkbLineString) {
    return GeometryPtr(OGRGeometryFactory::forceToMultiLineString(collection.release()));
  }
  throw std::runtime_error("Multiple point features must be split before export");
}

void insert(OGRLayer * layer, OGRFeature & feature)
{
  require(layer->CreateFeature(&feature) == OGRERR_NONE,
    "Cannot write S-57 record to " + std::string(layer->GetName()));
}

struct Edge
{
  OGRLineString interior;
  int start;
  int end;
};

// GDAL 的 S57 写接口要求显式的节点、边与要素指针，仅写 Polygon 会得到空面。
struct Topology
{
  std::vector<OGRPoint> isolated;
  std::vector<OGRPoint> connected;
  std::vector<Edge> edges;
  std::map<std::pair<long long, long long>, int> node_ids;

  int node(const OGRPoint & point)
  {
    const auto key = std::make_pair(std::llround(point.getX() * kCoordinateFactor),
      std::llround(point.getY() * kCoordinateFactor));
    const auto found = node_ids.find(key);
    if (found != node_ids.end()) {
      return found->second;
    }
    connected.push_back(point);
    const int id = static_cast<int>(connected.size());
    node_ids.emplace(key, id);
    return id;
  }

  void add(Object & object, const OGRGeometry & geometry, int usage = 1)
  {
    const auto type = wkbFlatten(geometry.getGeometryType());
    if (type == wkbPoint) {
      isolated.push_back(*geometry.toPoint());
      object.ids.push_back(static_cast<int>(isolated.size()));
      object.usages.push_back(1);
    } else if (type == wkbLineString || type == wkbLinearRing) {
      const auto & line = *geometry.toLineString();
      const int size = line.getNumPoints();
      require(size >= 2, "Invalid edge");
      OGRPoint point;
      line.getPoint(0, &point);
      const int start = node(point);
      line.getPoint(size - 1, &point);
      const int end = node(point);
      Edge edge{OGRLineString(), start, end};
      for (int i = 1; i + 1 < size; ++i) {
        line.getPoint(i, &point);
        edge.interior.addPoint(&point);
      }
      edges.push_back(std::move(edge));
      object.ids.push_back(static_cast<int>(edges.size()));
      object.usages.push_back(usage);
    } else if (type == wkbPolygon) {
      const auto & polygon = *geometry.toPolygon();
      add(object, *polygon.getExteriorRing(), 1);
      for (int i = 0; i < polygon.getNumInteriorRings(); ++i) {
        add(object, *polygon.getInteriorRing(i), 2);
      }
    } else if (type == wkbMultiPolygon || type == wkbMultiLineString) {
      for (const auto * part : *geometry.toGeometryCollection()) {
        add(object, *part);
      }
    } else {
      throw std::runtime_error("Unsupported S-57 primitive");
    }
  }

  void write(GDALDataset & dataset) const
  {
    const auto write_nodes = [&dataset](const char * layer_name, int rcnm,
        const std::vector<OGRPoint> & points) {
        auto * layer = dataset.GetLayerByName(layer_name);
        for (std::size_t i = 0; i < points.size(); ++i) {
          OGRFeature feature(layer->GetLayerDefn());
          feature.SetField("RCNM", rcnm);
          feature.SetField("RCID", static_cast<int>(i + 1));
          feature.SetField("RVER", 1);
          feature.SetField("RUIN", 1);
          feature.SetGeometry(&points[i]);
          insert(layer, feature);
        }
      };
    write_nodes("IsolatedNode", 110, isolated);
    write_nodes("ConnectedNode", 120, connected);
    auto * layer = dataset.GetLayerByName("Edge");
    for (std::size_t i = 0; i < edges.size(); ++i) {
      OGRFeature feature(layer->GetLayerDefn());
      feature.SetField("RCNM", 130);
      feature.SetField("RCID", static_cast<int>(i + 1));
      feature.SetField("RVER", 1);
      feature.SetField("RUIN", 1);
      for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const std::string suffix = "_" + std::to_string(endpoint);
        feature.SetField(("NAME_RCNM" + suffix).c_str(), 120);
        feature.SetField(("NAME_RCID" + suffix).c_str(),
          endpoint == 0 ? edges[i].start : edges[i].end);
        feature.SetField(("TOPI" + suffix).c_str(), endpoint + 1);
        for (const auto * field : {"ORNT", "USAG", "MASK"}) {
          feature.SetField((field + suffix).c_str(), 255);
        }
      }
      feature.SetGeometry(&edges[i].interior);
      insert(layer, feature);
    }
  }
};

void set_attributes(OGRFeature & feature, const Object & object)
{
  CPLJSONObject provenance;
  provenance.Add("source_name", object.name);
  provenance.Add("source_rcid", object.source_rcid);
  provenance.Add("source_id", object.source_id);
  std::string information;
  for (const auto & [name, value] : object.attributes) {
    if (value.empty()) {
      continue;  // 空数值不能转换成 0，0 米水深等有效值仍正常保留。
    }
    if (name == "INFORM") {
      information = value;
      continue;
    }
    const int index = feature.GetFieldIndex(name.c_str());
    if (index < 0) {
      provenance.Add(name, value);
    } else if (feature.GetFieldDefnRef(index)->GetType() == OFTStringList) {
      char ** values = CSLTokenizeString2(value.c_str(), ",;", 0);
      feature.SetField(index, values);
      CSLDestroy(values);
    } else {
      feature.SetField(index, value.c_str());
    }
  }
  if (!information.empty()) {
    information += "\n";
  }
  information += "USV_XML:" + provenance.Format(CPLJSONObject::PrettyFormat::Plain);
  require(feature.GetFieldIndex("INFORM") >= 0, "S-57 object has no INFORM attribute");
  // S-57 ATTF 使用 Latin-1。当前资源均为 ASCII；明确拒绝无法无损编码的文本。
  require(std::all_of(information.begin(), information.end(),
    [](unsigned char c) {return c < 128;}), "Non-ASCII attribute text is not supported");
  feature.SetField("INFORM", information.c_str());
  if (!object.name.empty() && object.name != object.group &&
    feature.GetFieldIndex("OBJNAM") >= 0 &&
    !feature.IsFieldSetAndNotNull(feature.GetFieldIndex("OBJNAM")))
  {
    feature.SetField("OBJNAM", object.name.c_str());
  }
}

std::string issue_date()
{
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  char date[9];
  std::strftime(date, sizeof(date), "%Y%m%d", &utc);
  return date;
}

void verify(const std::string & path, const std::vector<Object> & objects,
  const S57ExportResult & result)
{
  DatasetPtr dataset(static_cast<GDALDataset *>(GDALOpenEx(path.c_str(),
    GDAL_OF_VECTOR | GDAL_OF_READONLY, nullptr, nullptr, nullptr)), GDALClose);
  require(dataset != nullptr, "Cannot reopen exported S-57 chart");
  for (const auto & [group, count] : result.feature_counts) {
    auto * layer = dataset->GetLayerByName(group.c_str());
    require(layer && layer->GetFeatureCount() == count, "Feature count mismatch: " + group);
    while (FeaturePtr feature{layer->GetNextFeature()}) {
      const int index = feature->GetFieldAsInteger("RCID") - 1;
      require(index >= 0 && static_cast<std::size_t>(index) < objects.size(), "Invalid RCID");
      auto * geometry = feature->GetGeometryRef();
      require(geometry && !geometry->IsEmpty(), "Missing geometry: " + group);
      require(geometry->getDimension() == objects[index].geometry->getDimension(),
        "Geometry dimension mismatch: " + group);
      OGREnvelope actual, expected;
      geometry->getEnvelope(&actual);
      objects[index].geometry->getEnvelope(&expected);
      require(std::abs(actual.MinX - expected.MinX) < 1e-6 &&
        std::abs(actual.MaxX - expected.MaxX) < 1e-6 &&
        std::abs(actual.MinY - expected.MinY) < 1e-6 &&
        std::abs(actual.MaxY - expected.MaxY) < 1e-6, "Geographic extent mismatch: " + group);
    }
  }
}

}  // namespace

S57ExportResult export_s57(const usv_map::Chart & chart, const std::string & output)
{
  require(!chart.features.empty() && !chart.has_unreadable_navigation_geometry,
    "Source chart is empty or has unreadable navigation geometry");
  require(std::filesystem::path(output).extension() == ".000", "Output extension must be .000");
  require(!std::filesystem::exists(output), "Output exists; choose another path: " + output);
  GDALAllRegister();
  auto * driver = GetGDALDriverManager()->GetDriverByName("S57");
  require(driver != nullptr, "GDAL S57 driver is unavailable");
  std::vector<Object> objects;
  S57ExportResult result;
  for (const auto & feature : chart.features) {
    auto geometry = make_geometry(feature, chart);
    const auto append = [&](GeometryPtr part) {
        Object object;
        object.group = feature.group;
        object.name = feature.name;
        object.source_rcid = feature.rcid;
        object.source_id = feature.feature_id;
        object.attributes = feature.attributes;
        object.primitive = part->getDimension() + 1;
        if (object.primitive == 3) {
          CPLPushErrorHandler(CPLQuietErrorHandler);
          const bool valid = part->IsValid();
          CPLPopErrorHandler();
          result.invalid_source_areas += !valid;
        }
        object.geometry = std::move(part);
        objects.push_back(std::move(object));
      };
    // S57 面读取器把后续外环当作内环；分离的多面必须拆成独立要素。
    if (wkbFlatten(geometry->getGeometryType()) == wkbMultiPolygon) {
      for (const auto * polygon : *geometry->toMultiPolygon()) {
        append(GeometryPtr(polygon->clone()));
      }
    } else {
      append(std::move(geometry));
    }
  }
  // 覆盖范围是整个 XML Bounds，水域、陆地和禁行区均有数据覆盖。
  auto ring = std::make_unique<OGRLinearRing>();
  ring->addPoint(chart.bounds[0], chart.bounds[1]);
  ring->addPoint(chart.bounds[0], chart.bounds[3]);
  ring->addPoint(chart.bounds[2], chart.bounds[3]);
  ring->addPoint(chart.bounds[2], chart.bounds[1]);
  ring->closeRings();
  auto coverage = std::make_unique<OGRPolygon>();
  coverage->addRingDirectly(ring.release());
  Object cover;
  cover.group = "M_COVR";
  cover.attributes["CATCOV"] = "1";
  cover.geometry = std::move(coverage);
  objects.push_back(std::move(cover));

  Topology topology;
  for (auto & object : objects) {
    topology.add(object, *object.geometry);
    ++result.feature_counts[object.group];
  }
  CPLStringList options;
  for (const auto & [name, value] : std::map<std::string, std::string>{
      {"S57_AGEN", "0"}, {"S57_INTU", "5"}, {"S57_EDTN", "1"}, {"S57_UPDN", "0"},
      {"S57_ISDT", issue_date()}, {"S57_UADT", issue_date()}, {"S57_AALL", "1"},
      {"S57_NALL", "1"}, {"S57_NOMR", "1"},
      {"S57_NOGR", std::to_string(objects.size() - 1)},
      {"S57_NOIN", std::to_string(topology.isolated.size())},
      {"S57_NOCN", std::to_string(topology.connected.size())},
      {"S57_NOED", std::to_string(topology.edges.size())},
      {"S57_CSCL", "5000"}, {"S57_HDAT", "2"}, {"S57_VDAT", "255"}, {"S57_SDAT", "255"},
      {"S57_COMT", "USV reservoir simulation chart; derived from XML, not a surveyed ENC"}})
  {
    options.SetNameValue(name.c_str(), value.c_str());
  }
  // 不继承 XML 里原官方海图的机构、版次和过期要素计数。
  DatasetPtr dataset(driver->Create(output.c_str(), 0, 0, 0, GDT_Unknown, options.List()), GDALClose);
  require(dataset != nullptr, "Cannot create S-57 output: " + output);
  try {
    topology.write(*dataset);
    for (std::size_t i = 0; i < objects.size(); ++i) {
      const auto & object = objects[i];
      auto * layer = dataset->GetLayerByName(object.group.c_str());
      require(layer != nullptr, "Unsupported S-57 object class: " + object.group);
      OGRFeature feature(layer->GetLayerDefn());
      feature.SetField("RCID", static_cast<int>(i + 1));
      feature.SetField("PRIM", object.primitive);
      feature.SetField("GRUP", object.group == "DEPARE" || object.group == "LNDARE" ? 1 : 2);
      feature.SetField("AGEN", 0);
      feature.SetField("FIDN", static_cast<int>(i + 1));
      feature.SetField("FIDS", 1);
      feature.SetField("RVER", 1);
      const int size = static_cast<int>(object.ids.size());
      const std::vector<int> names(size, object.primitive == 1 ? 110 : 130);
      const std::vector<int> orientations(size, 1);
      const std::vector<int> masks(size, 2);
      feature.SetField("NAME_RCNM", size, names.data());
      feature.SetField("NAME_RCID", size, object.ids.data());
      feature.SetField("ORNT", size, orientations.data());
      feature.SetField("USAG", size, object.usages.data());
      feature.SetField("MASK", size, masks.data());
      set_attributes(feature, object);
      insert(layer, feature);
    }
    dataset.reset();
    verify(output, objects, result);
  } catch (...) {
    dataset.reset();
    std::filesystem::remove(output);
    throw;
  }
  return result;
}

}  // namespace usv_map_tools
