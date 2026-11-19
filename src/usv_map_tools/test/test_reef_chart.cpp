/**
 * @file test_reef_chart.cpp
 * @brief 验证暗礁几何生成、边界约束及 XML 保存。
 */

#include "reef_chart.hpp"

#include <gtest/gtest.h>
#include <geos_c.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include <usv_map/renderer.hpp>

namespace
{
using usv_map::Point;
using usv_map_tools::make_reef;
using usv_map_tools::ReefChart;

double distance_to_edge(Point c, Point a, Point b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double t = std::clamp(((c.x - a.x) * dx + (c.y - a.y) * dy) /
      (dx * dx + dy * dy), 0.0, 1.0);
  return std::hypot(a.x + t * dx - c.x, a.y + t * dy - c.y);
}

void verify(const std::vector<Point> & ring, Point center)
{
  ASSERT_EQ(ring.size(), 513u);
  EXPECT_DOUBLE_EQ(ring.front().x, ring.back().x);
  EXPECT_DOUBLE_EQ(ring.front().y, ring.back().y);
  double minimum = INFINITY, maximum = 0, asymmetry = 0;
  const auto count = ring.size() - 1;
  auto context = GEOS_init_r();
  auto * coordinates = GEOSCoordSeq_create_r(context, ring.size(), 2);
  for (std::size_t i = 0; i < ring.size(); ++i) {
    GEOSCoordSeq_setX_r(context, coordinates, i, ring[i].x);
    GEOSCoordSeq_setY_r(context, coordinates, i, ring[i].y);
  }
  auto * polygon = GEOSGeom_createPolygon_r(context,
    GEOSGeom_createLinearRing_r(context, coordinates), nullptr, 0);
  ASSERT_NE(polygon, nullptr);
  EXPECT_EQ(GEOSisValid_r(context, polygon), 1);
  auto * c = GEOSGetCentroid_r(context, polygon);
  double x = 0, y = 0;
  GEOSGeomGetX_r(context, c, &x); GEOSGeomGetY_r(context, c, &y);
  EXPECT_NEAR(x, center.x, 1e-7);
  EXPECT_NEAR(y, center.y, 1e-7);
  GEOSGeom_destroy_r(context, c);
  GEOSGeom_destroy_r(context, polygon);
  GEOS_finish_r(context);
  for (std::size_t i = 0; i < count; ++i) {
    const auto a = ring[i], b = ring[i + 1], next = ring[(i + 2) % count];
    const Point d{b.x - a.x, b.y - a.y}, e{next.x - b.x, next.y - b.y};
    const double cross = d.x * e.y - d.y * e.x;
    // 包括闭合处在内的所有转角均应严格凸，且不出现尖角。
    EXPECT_GT(cross, 0);
    EXPECT_LT(std::atan2(cross, d.x * e.x + d.y * e.y), 0.05);
    minimum = std::min(minimum, distance_to_edge(center, a, b));
    maximum = std::max(maximum, std::hypot(a.x - center.x, a.y - center.y));
    const auto opposite = ring[(i + count / 2) % count];
    asymmetry = std::max(asymmetry,
      std::hypot(a.x + opposite.x - 2 * center.x, a.y + opposite.y - 2 * center.y));
  }
  EXPECT_GE(minimum, 3.0);
  EXPECT_LE(maximum, 5.0);
  EXPECT_NEAR(minimum, 3.0, 0.0002);
  EXPECT_NEAR(maximum, 5.0, 0.0002);
  EXPECT_GT(asymmetry, 0.01);
}

TEST(ReefGeometry, RandomShapesStaySmoothConvexAndCentredWithinTheWholeEdgeLimits)
{
  for (uint64_t seed = 0; seed < 128; ++seed) {
    SCOPED_TRACE(seed);
    const Point center{123.4 - static_cast<double>(seed), -432.1};
    const auto reef = make_reef(center, seed);
    verify(reef.boundary, center);
  }
}

TEST(ReefGeometry, SeedsReproduceShapesAndDifferentSeedsVaryThem)
{
  const auto first = make_reef({0, 0}, 123), again = make_reef({0, 0}, 123);
  const auto other = make_reef({0, 0}, 124);
  double difference = 0;
  for (std::size_t i = 0; i < first.boundary.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.boundary[i].x, again.boundary[i].x);
    EXPECT_DOUBLE_EQ(first.boundary[i].y, again.boundary[i].y);
    difference += std::hypot(first.boundary[i].x - other.boundary[i].x,
      first.boundary[i].y - other.boundary[i].y);
  }
  EXPECT_GT(difference, 10);
  const auto smaller = make_reef({0, 0}, 0, 2, 3);
  for (std::size_t i = 1; i < smaller.boundary.size(); ++i) {
    EXPECT_GE(distance_to_edge({0, 0}, smaller.boundary[i - 1], smaller.boundary[i]), 2);
    EXPECT_LE(std::hypot(smaller.boundary[i].x, smaller.boundary[i].y), 3);
  }
}

TEST(ReefGeometry, RejectsInvalidCentresAndRadiusRanges)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(make_reef({nan, 0}, 0), std::invalid_argument);
  EXPECT_THROW(make_reef({0, INFINITY}, 0), std::invalid_argument);
  for (const auto range : {Point{0, 8}, Point{-5, 8}, Point{8, 5}, Point{5, 5},
      Point{nan, 8}, Point{5, INFINITY}, Point{5, 5.000001}})
  {
    EXPECT_THROW(make_reef({0, 0}, 0, range.x, range.y), std::invalid_argument);
  }
}

const tinyxml2::XMLElement * group(const tinyxml2::XMLDocument & document, const char * name)
{
  const auto * features = document.RootElement()->FirstChildElement("Map")->FirstChildElement("Features");
  for (const auto * item = features->FirstChildElement("FeatureGroup"); item;
    item = item->NextSiblingElement("FeatureGroup"))
  {
    if (item->Attribute("acronym", name)) {return item;}
  }
  return nullptr;
}

std::string xml(const tinyxml2::XMLNode * node)
{
  tinyxml2::XMLPrinter printer;
  node->Accept(&printer);
  return printer.CStr();
}

class ReefChartTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/usv_map_reef_XXXXXX";
    const auto * directory = ::mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    directory_ = directory;
    input_ = directory_ / "input.xml";
    output_ = directory_ / "output.xml";
    write_source();
  }

  void TearDown() override {std::filesystem::remove_all(directory_);}

  void write_source(bool projected = false, bool land = true)
  {
    const auto polygon = projected ?
      "POLYGON ((11879000 -3438000, 11879020 -3438000, 11879000 -3438020, 11879000 -3438000))" :
      "POLYGON ((106.70 29.49, 106.701 29.49, 106.70 29.491, 106.70 29.49))";
    std::ofstream stream(input_);
    stream << "<S57ChartData><Map><Metadata><DSID_DSNM>keep me</DSID_DSNM></Metadata>"
      "<Bounds><MinX>106.70</MinX><MinY>29.49</MinY><MaxX>106.74</MaxX><MaxY>29.53</MaxY></Bounds>"
      "<Features><FeatureGroup acronym=\"DEPARE\" count=\"1\"><Feature>"
      "<ID>7</ID><RCID>7</RCID><Geometry><Geometries><Polygon>" << polygon <<
      "</Polygon></Geometries></Geometry></Feature></FeatureGroup>";
    if (land) {
      stream << "<FeatureGroup acronym=\"LNDARE\" count=\"1\"><Feature><ID>21</ID><RCID>31</RCID>"
        "<Attributes><INFORM>original obstacle</INFORM></Attributes><Geometry><Geometries><Polygon>" <<
        polygon << "</Polygon></Geometries></Geometry></Feature></FeatureGroup>";
    }
    stream << "<FeatureGroup acronym=\"BERTHS\" count=\"1\"><Feature><ID>8</ID><RCID>8</RCID>"
      "<Geometry><Geometries><Polygon>" << polygon <<
      "</Polygon></Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>";
  }

  std::filesystem::path directory_, input_, output_;
};

TEST_F(ReefChartTest, PreservesTheWholeBaseAndResumesWithUniqueIds)
{
  tinyxml2::XMLDocument original;
  ASSERT_EQ(original.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  ReefChart chart(input_);
  EXPECT_EQ(chart.add({12, -25}, 42, 3, 5, output_).id, 32);
  tinyxml2::XMLDocument saved;
  ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  for (const char * name : {"DEPARE", "BERTHS"}) {
    EXPECT_EQ(xml(group(original, name)), xml(group(saved, name)));
  }
  EXPECT_EQ(xml(group(original, "LNDARE")->FirstChildElement("Feature")),
    xml(group(saved, "LNDARE")->FirstChildElement("Feature")));
  EXPECT_EQ(group(saved, "LNDARE")->IntAttribute("count"), 2);
  EXPECT_EQ(xml(original.RootElement()->FirstChildElement("Map")->FirstChildElement("Metadata")),
    xml(saved.RootElement()->FirstChildElement("Map")->FirstChildElement("Metadata")));
  ReefChart resumed(output_);
  ASSERT_EQ(resumed.reefs().size(), 1u);
  verify(resumed.reefs().front().boundary, {12, -25});
  EXPECT_EQ(resumed.add({20, 30}, 43, 3, 5, output_).id, 33);
  EXPECT_EQ(ReefChart(output_).reefs().size(), 2u);
  ASSERT_EQ(original.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(group(original, "LNDARE")->IntAttribute("count"), 1);
}

TEST_F(ReefChartTest, SourceCoordinateRoundTripsAndReefRemainsUnoccupied)
{
  for (const bool projected : {false, true}) {
    write_source(projected);
    ReefChart(input_).add({0, 0}, 123, 3, 5, output_);
    const auto parsed = usv_map::load_chart(output_.string());
    EXPECT_EQ(parsed.source_is_projected, projected);
    ASSERT_FALSE(parsed.has_unreadable_navigation_geometry);
    const auto found = std::find_if(parsed.features.begin(), parsed.features.end(),
      [](const auto & feature) {return feature.attributes.count("OBSTACLE_KIND") != 0;});
    ASSERT_NE(found, parsed.features.end());
    const auto & feature = *found;
    ASSERT_EQ(feature.group, "LNDARE");
    EXPECT_EQ(feature.attributes.at("OBSTACLE_KIND"), "REEF");
    verify(feature.geometry.front().parts.front(), {0, 0});
    usv_map::RenderConfig config;
    config.planning_resolution = 2;
    config.obstacle_inflation = 0;
    const auto costmap = usv_map::Renderer(config).build_costmap(
      parsed, builtin_interfaces::msg::Time{});
    const auto x = static_cast<int>(-costmap.info.origin.position.x / costmap.info.resolution);
    const auto y = static_cast<int>(-costmap.info.origin.position.y / costmap.info.resolution);
    ASSERT_GE(x, 0); ASSERT_GE(y, 0);
    ASSERT_LT(x, static_cast<int>(costmap.info.width));
    ASSERT_LT(y, static_cast<int>(costmap.info.height));
    EXPECT_EQ(costmap.data[y * costmap.info.width + x], config.channel_outside_cost);
  }
}

TEST_F(ReefChartTest, FailedSaveRollsBackWithOrWithoutAnExistingLandGroup)
{
  for (const bool land : {false, true}) {
    write_source(false, land);
    ReefChart chart(input_);
    EXPECT_THROW(chart.add({0, 0}, 42, 3, 5, directory_), std::runtime_error);
    EXPECT_TRUE(chart.reefs().empty());
    EXPECT_FALSE(std::filesystem::exists(directory_.string() + ".tmp"));
    const int first_id = land ? 32 : 9;
    EXPECT_EQ(chart.add({0, 0}, 42, 3, 5, output_).id, first_id);
    EXPECT_THROW(chart.add({20, 0}, 43, 3, 5, directory_), std::runtime_error);
    EXPECT_EQ(chart.reefs().size(), 1u);
    EXPECT_EQ(chart.add({20, 0}, 43, 3, 5, output_).id, first_id + 1);
    EXPECT_EQ(ReefChart(output_).reefs().size(), 2u);
  }
}

TEST_F(ReefChartTest, RejectsInvalidOrPartlyOutsideWithoutWriting)
{
  ReefChart chart(input_);
  // 中心位于西边界内，但半径 3～5 m 的轮廓越界，应拒绝保存。
  const double west = (106.70 - chart.longitude()) * 3.14159265358979323846 / 180 *
    6378137.0 * std::cos(chart.latitude() * 3.14159265358979323846 / 180);
  EXPECT_THROW(chart.add({west + 1, 0}, 42, 3, 5, output_), std::invalid_argument);
  EXPECT_THROW(chart.add({1e8, 0}, 42, 3, 5, output_), std::invalid_argument);
  EXPECT_THROW(chart.add({0, 0}, 42, 8, 5, output_), std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(output_));
  EXPECT_TRUE(chart.reefs().empty());
  EXPECT_EQ(chart.add({0, 0}, 42, 3, 5, output_).id, 32);
}
}  // namespace
