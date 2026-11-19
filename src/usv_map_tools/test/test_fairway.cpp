/**
 * @file test_fairway.cpp
 * @brief 验证航道要素导出、恢复及属性保存。
 */

#include "clockwise_route.hpp"
#include "fairway_chart.hpp"
#include "route_water.hpp"

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tuple>

namespace
{
using namespace usv_map_tools;
constexpr double kPi = 3.14159265358979323846;

std::vector<RoutePoint> reference()
{
  std::vector<RoutePoint> points;
  for (int i = 0; i < 120; ++i) {
    const double a = 2 * kPi * i / 120;
    points.push_back({100 * std::cos(a), 200 * std::sin(a)});
  }
  return points;
}

void add(usv_map::Chart & chart, const std::string & group, const std::string & wkt)
{
  usv_map::ChartFeature f;
  f.group = group;
  f.geometry.push_back(usv_map::parse_wkt(wkt));
  chart.features.push_back(f);
}

usv_map::Chart water_chart()
{
  usv_map::Chart chart;
  add(chart, "DEPARE", "POLYGON ((-400 -400,400 -400,400 400,-400 400,-400 -400))");
  add(chart, "LNDARE", "POLYGON ((-15 -20,15 -20,15 20,-15 20,-15 -20))");
  return chart;
}

double signed_area(const std::vector<RoutePose> & poses)
{
  double sum = 0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const auto a = poses[i - 1].position, b = poses[i].position;
    sum += a.x * b.y - b.x * a.y;
  }
  return sum / 2;
}

class FairwayGeometry : public ::testing::TestWithParam<std::tuple<double, double>> {};

TEST_P(FairwayGeometry, ExactLaneWidthsOppositeTravelAndClosedTangentPaths)
{
  const auto [width, inset] = GetParam();
  const auto chart = water_chart();
  const auto route = make_safe_clockwise_route(reference(), 2, chart, 15);
  const auto f = make_fairway(route, width, 2, chart, 5, inset);
  ASSERT_GE(f.minimum_clearance_m, 5);
  ASSERT_GE(f.centerline_clearance_m, width + 5);
  ASSERT_EQ(f.center.size(), f.clockwise.size());
  ASSERT_EQ(f.center.size(), f.counterclockwise.size());
  for (std::size_t i = 0; i < f.center.size(); ++i) {
    const auto c = f.center[i];
    const auto reverse = f.counterclockwise[f.center.size() - i - 1];
    const auto check = [&](RoutePoint p, double expected) {
        const double dx = p.x - c.position.x, dy = p.y - c.position.y;
        EXPECT_NEAR(dx * std::cos(c.yaw) + dy * std::sin(c.yaw), 0, 1e-10);
        EXPECT_NEAR(dx * std::sin(c.yaw) - dy * std::cos(c.yaw), expected, 1e-10);
      };
    check(f.inner[i], width);
    check(f.outer[i], -width);
    check(f.clockwise[i].position, width / 2);
    check(reverse.position, -width / 2);
    EXPECT_NEAR(std::abs(std::remainder(reverse.yaw - c.yaw, 2 * kPi)), kPi, 1e-12);
  }
  EXPECT_LT(signed_area(f.clockwise), 0);
  EXPECT_GT(signed_area(f.counterclockwise), 0);
  for (const auto * poses : {&f.clockwise, &f.counterclockwise}) {
    EXPECT_DOUBLE_EQ(poses->front().position.x, poses->back().position.x);
    EXPECT_DOUBLE_EQ(poses->front().position.y, poses->back().position.y);
    EXPECT_DOUBLE_EQ(poses->front().yaw, poses->back().yaw);
    double turning = 0;
    for (std::size_t i = 1; i < poses->size(); ++i) {
      const auto a = (*poses)[i - 1], b = (*poses)[i];
      const double dx = b.position.x - a.position.x, dy = b.position.y - a.position.y;
      EXPECT_GT((dx * std::cos(a.yaw) + dy * std::sin(a.yaw)) / std::hypot(dx, dy), .99);
      turning += std::remainder(b.yaw - a.yaw, 2 * kPi);
    }
    EXPECT_NEAR(std::abs(turning), 2 * kPi, 1e-9);
  }
  if (inset > 0) {
    const auto original = make_fairway(route, width, 2, chart, 5);
    const auto perimeter = [](const auto & poses) {
        double sum = 0;
        for (std::size_t i = 1; i < poses.size(); ++i) {
          sum += std::hypot(poses[i].position.x - poses[i - 1].position.x,
            poses[i].position.y - poses[i - 1].position.y);
        }
        return sum;
      };
    // 凸闭环的规则内偏移使周长减少 2*pi*inset；独立采样的折线允许离散误差。
    EXPECT_NEAR(perimeter(original.center) - perimeter(f.center), 2 * kPi * inset, .05);
    EXPECT_LT(std::abs(signed_area(f.center)), std::abs(signed_area(original.center)));
  }
}
INSTANTIATE_TEST_SUITE_P(WidthsAndInsets, FairwayGeometry,
  ::testing::Values(std::make_tuple(10., 0.), std::make_tuple(15., 0.),
    std::make_tuple(15., 5.)));

TEST(Fairway, InsetRechecksTwentyMetreCentrelineInsteadOfUsingOldClearance)
{
  const auto points = reference();
  const auto poses = make_clockwise_route(points, 2);
  double x = 0, y = 0;
  for (const auto & p : poses) {
    x = std::max(x, std::abs(p.position.x));
    y = std::max(y, std::abs(p.position.y));
  }
  x += 15.5;
  y += 30;
  std::ostringstream bank;
  bank << "POLYGON ((" << -x << ' ' << -y << ',' << x << ' ' << -y << ','
       << x << ' ' << y << ',' << -x << ' ' << y << ',' << -x << ' ' << -y << "))";
  usv_map::Chart chart;
  add(chart, "DEPARE", bank.str());
  const auto route = make_safe_clockwise_route(points, 2, chart, 15);
  ASSERT_GE(route.minimum_clearance_m, 15);
  ASSERT_LT(route.minimum_clearance_m, 20);
  EXPECT_THROW(make_fairway(route, 15, 2, chart, 5), std::runtime_error);
  const auto f = make_fairway(route, 15, 2, chart, 5, 5);
  EXPECT_GE(f.centerline_clearance_m, 20);
  EXPECT_GE(f.minimum_clearance_m, 5);
}

TEST(Fairway, InsetTowardsIslandRejectsInsufficientFinalClearance)
{
  auto chart = water_chart();
  const auto route = make_safe_clockwise_route(reference(), 2, chart, 20);
  const auto f = make_fairway(route, 15, 2, chart, 5);
  // 内侧 21 m 处的岛屿不影响原航道；内缩 5 m 后，15 m 内边界将违反 5 m 净距。
  const auto p = f.center.front();
  const double x = p.position.x + 21 * std::sin(p.yaw);
  const double y = p.position.y - 21 * std::cos(p.yaw);
  std::ostringstream island;
  island << "POLYGON ((" << x-.1 << ' ' << y-.1 << ',' << x+.1 << ' ' << y-.1 << ','
         << x+.1 << ' ' << y+.1 << ',' << x-.1 << ' ' << y+.1 << ','
         << x-.1 << ' ' << y-.1 << "))";
  add(chart, "LNDARE", island.str());
  EXPECT_NO_THROW(make_fairway(route, 15, 2, chart, 5));
  EXPECT_THROW(make_fairway(route, 15, 2, chart, 5, 5), std::runtime_error);
}

TEST(Fairway, WholeAreaCheckDetectsAnIslandBetweenClearBoundaries)
{
  auto chart = water_chart();
  const std::vector<RoutePoint> outer{{-50,-50},{50,-50},{50,50},{-50,50},{-50,-50}};
  const std::vector<RoutePoint> inner{{-30,-30},{30,-30},{30,30},{-30,30},{-30,-30}};
  EXPECT_GT(RouteWater(chart, 0).corridor_clearance(outer, inner), 5);
  // 岛屿位于航道内部，仅检查两侧边界会漏检。
  add(chart, "LNDARE", "POLYGON ((39 -1,41 -1,41 1,39 1,39 -1))");
  const RouteWater water(chart, 0);
  EXPECT_GE(water.clearance(outer), 9);
  EXPECT_GE(water.clearance(inner), 9);
  EXPECT_LT(water.corridor_clearance(outer, inner), 0);
}

TEST(Fairway, RefusesInsufficientClearanceCuspsAndInvalidWidths)
{
  const auto chart = water_chart();
  auto route = make_safe_clockwise_route(reference(), 2, chart, 15);
  route.minimum_clearance_m = 14.9;
  EXPECT_THROW(make_fairway(route, 10, 2, chart, 5), std::runtime_error);
  route.minimum_clearance_m = 100;
  route.controls = {{-2,-2},{-2,2},{2,2},{2,-2}};
  EXPECT_THROW(make_fairway(route, 10, 2, chart, 5), std::runtime_error);
  for (double width : {0., -1., std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(make_fairway(route, width, 2, chart, 5), std::invalid_argument);
  }
  for (double inset : {-1., std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(make_fairway(route, 15, 2, chart, 5, inset), std::invalid_argument);
  }
  // 10 m 偏移可保持规则；更大的总内偏移若超过曲率半径，应拒绝生成。
  const auto regular = make_safe_clockwise_route(reference(), 2, chart, 15);
  EXPECT_THROW(make_fairway(regular, 15, 2, chart, 5, 100), std::runtime_error);
}

class FairwayXml : public ::testing::TestWithParam<bool>
{
protected:
  std::filesystem::path directory;
  void SetUp() override
  {
    char pattern[] = "/tmp/fairway_xml_XXXXXX";
    const auto * path = mkdtemp(pattern);
    ASSERT_NE(path, nullptr);
    directory = path;
  }
  void TearDown() override {std::filesystem::remove_all(directory);}
};

TEST_P(FairwayXml, ChangjiangSchemaRoundTripPreservesChartAndReplacesOnlyGeneratedLanes)
{
  const auto input = directory / "source.xml", output = directory / "fairway.xml";
  // 分别验证解析器支持的经纬度和投影坐标。
  const auto polygon = [&](double x, double y, double half) {
      std::ostringstream wkt;
      wkt << std::fixed << std::setprecision(10) << "POLYGON ((";
      for (auto p : std::vector<RoutePoint>{{x-half,y-half},{x+half,y-half},
          {x+half,y+half},{x-half,y+half},{x-half,y-half}})
      {
        const double lon = 106.7 * kPi / 180 + p.x / (6378137 * std::cos(29.5 * kPi / 180));
        const double lat = 29.5 * kPi / 180 + p.y / 6378137;
        if (GetParam()) {
          wkt << lon * 6378137 << ' ' << -6378137 * std::asinh(std::tan(lat));
        } else {
          wkt << lon * 180 / kPi << ' ' << lat * 180 / kPi;
        }
        wkt << ", ";
      }
      auto s = wkt.str(); s.resize(s.size() - 2); return s + "))";
    };
  std::ofstream source(input);
  source << "<S57ChartData version='1.0' totalMaps='1'><Map index='0'><Metadata>"
         << "<DSID_DSNM>source</DSID_DSNM></Metadata><Bounds><MinX>106.69</MinX>"
         << "<MinY>29.49</MinY><MaxX>106.71</MaxX><MaxY>29.51</MaxY></Bounds><Features>";
  for (const auto & name : {"DEPARE", "LNDARE", "BERTHS", "TSSLPT"}) {
    source << "<FeatureGroup acronym='" << name << "' count='1'><Feature><ID>1</ID>"
           << "<Name>original</Name><Type>A</Type><Attributes><ORIENT>42</ORIENT></Attributes>"
           << "<Geometry><Geometries level='0'><Polygon>"
           << polygon(0, 0, std::string(name) == "DEPARE" ? 400 : 10)
           << "</Polygon></Geometries></Geometry></Feature></FeatureGroup>";
  }
  source << "</Features></Map></S57ChartData>";
  source.close();
  const auto chart = usv_map::load_chart(input.string(), 106.7, 29.5);
  const auto route = make_safe_clockwise_route(reference(), 2, chart, 15);
  const auto f = make_fairway(route, 15, 2, chart, 5, 5);
  save_fairway_chart(input, output, chart, f, 5);
  auto loaded = usv_map::load_chart(output.string(), 106.7, 29.5);
  ASSERT_EQ(loaded.features.size(), chart.features.size() + f.sections.size() + 3);
  std::size_t section_index = 0;
  for (const auto & item : loaded.features) {
    if (item.group != "TSSLPT" || item.name == "original") {continue;}
    ASSERT_LT(section_index, f.sections.size());
    const auto & section = f.sections[section_index++];
    EXPECT_EQ(item.feature_type, "A");
    EXPECT_EQ(item.attributes.at("DIRECTION"), section.clockwise ? "CW" : "CCW");
    EXPECT_DOUBLE_EQ(std::stod(item.attributes.at("WIDTH_M")), 15);
    const double orient = std::stod(item.attributes.at("ORIENT"));
    EXPECT_NEAR(std::remainder((90 - orient) * kPi / 180 - section.yaw, 2 * kPi), 0, 1e-10);
    const auto & points = item.geometry.at(0).parts.at(0);
    ASSERT_EQ(points.size(), section.polygon.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      EXPECT_NEAR(points[i].x, section.polygon[i].x, 2e-5);
      EXPECT_NEAR(points[i].y, section.polygon[i].y, 2e-5);
    }
  }
  EXPECT_EQ(section_index, f.sections.size());
  const auto restored = load_saved_fairway(loaded);
  EXPECT_TRUE(std::isnan(restored.minimum_clearance_m));
  EXPECT_TRUE(std::isnan(restored.centerline_clearance_m));
  EXPECT_DOUBLE_EQ(restored.lane_width_m, f.lane_width_m);
  ASSERT_EQ(restored.sections.size(), f.sections.size());
  for (const auto & paths : {std::make_pair(&f.center, &restored.center),
      std::make_pair(&f.clockwise, &restored.clockwise),
      std::make_pair(&f.counterclockwise, &restored.counterclockwise)})
  {
    ASSERT_EQ(paths.first->size(), paths.second->size());
    for (std::size_t i = 0; i < paths.first->size(); ++i) {
      const auto original = paths.first->at(i), saved = paths.second->at(i);
      EXPECT_NEAR(saved.position.x, original.position.x, 2e-5);
      EXPECT_NEAR(saved.position.y, original.position.y, 2e-5);
      EXPECT_NEAR(std::remainder(saved.yaw - original.yaw, 2 * kPi), 0, 2e-6);
    }
  }
  // 显示用测试航道保持固定，航道内放置障碍物不触发重新生成。
  auto obstructed = loaded;
  const auto reef = restored.clockwise.front().position;
  std::ostringstream reef_wkt;
  reef_wkt << "POLYGON ((" << reef.x - 3 << ' ' << reef.y - 3 << ','
           << reef.x + 3 << ' ' << reef.y - 3 << ','
           << reef.x + 3 << ' ' << reef.y + 3 << ','
           << reef.x - 3 << ' ' << reef.y + 3 << ','
           << reef.x - 3 << ' ' << reef.y - 3 << "))";
  add(obstructed, "LNDARE", reef_wkt.str());
  EXPECT_DOUBLE_EQ(load_saved_fairway(obstructed).clockwise.front().position.x, reef.x);
  auto incomplete = loaded;
  incomplete.features.erase(std::remove_if(incomplete.features.begin(), incomplete.features.end(),
      [](const auto & feature) {return feature.group == "TSELNE";}), incomplete.features.end());
  EXPECT_THROW(load_saved_fairway(incomplete), std::runtime_error);
  auto malformed = loaded;
  for (auto & feature : malformed.features) {
    if (feature.group == "TSSBND") {feature.geometry.front().parts.front().pop_back(); break;}
  }
  EXPECT_THROW(load_saved_fairway(malformed), std::runtime_error);
  tinyxml2::XMLDocument xml;
  ASSERT_EQ(xml.LoadFile(output.c_str()), tinyxml2::XML_SUCCESS);
  auto * features = xml.RootElement()->FirstChildElement("Map")->FirstChildElement("Features");
  for (auto * group = features->FirstChildElement("FeatureGroup"); group;
    group = group->NextSiblingElement("FeatureGroup"))
  {
    int count = 0;
    for (auto * feature = group->FirstChildElement("Feature"); feature;
      feature = feature->NextSiblingElement("Feature")) {++count;}
    EXPECT_EQ(count, group->IntAttribute("count"));
  }
  const auto updated = directory / "updated.xml";
  save_fairway_chart(output, updated, loaded, f, 5);
  EXPECT_EQ(usv_map::load_chart(updated.string()).features.size(), loaded.features.size());
  EXPECT_THROW(save_fairway_chart(input, input, chart, f, 5), std::invalid_argument);
  // 重命名失败时，应保留已有输出目录及其内容。
  EXPECT_THROW(save_fairway_chart(input, directory, chart, f, 5), std::filesystem::filesystem_error);
  EXPECT_TRUE(std::filesystem::is_regular_file(input));
  EXPECT_FALSE(std::filesystem::exists(directory.string() + ".tmp"));
}
INSTANTIATE_TEST_SUITE_P(SourceCoordinates, FairwayXml, ::testing::Bool());
}  // namespace
