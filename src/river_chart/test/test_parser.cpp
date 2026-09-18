/// @file test_parser.cpp
/// @brief 海图 XML、WKT 和坐标转换测试（对应原 Python test_parser.py）。

/// @date 2026-08-28

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "river_chart/parser.hpp"

namespace
{

using river_chart::Chart;
using river_chart::ChartFeature;
using river_chart::ChartParseError;
using river_chart::Geometry;
using river_chart::load_chart;
using river_chart::parse_wkt;

/// @brief 写入临时海图并返回路径。
std::string write_chart(const std::string & name, const std::string & content)
{
  const std::filesystem::path path = std::filesystem::path(::testing::TempDir()) / name;
  std::ofstream stream(path, std::ios::binary);
  stream << content;
  return path.string();
}

}  // namespace

TEST(Parser, ParsePolygonAndHole)
{
  const Geometry geometry = parse_wkt("POLYGON ((0 0, 5 0, 5 5, 0 0), (1 1, 2 1, 1 1))");
  EXPECT_EQ(geometry.type, "POLYGON");
  ASSERT_EQ(geometry.parts.size(), 2u);
  EXPECT_DOUBLE_EQ(geometry.parts[0][1].x, 5.0);
  EXPECT_DOUBLE_EQ(geometry.parts[0][1].y, 0.0);
}

TEST(Parser, ParseMultiGeometries)
{
  const Geometry geometry =
    parse_wkt("MULTIPOLYGON (((0 0, 1 0, 0 0)), ((2 2, 3 2, 2 2)))");
  EXPECT_EQ(geometry.type, "MULTIPOLYGON");
  ASSERT_EQ(geometry.parts.size(), 2u);
}

TEST(Parser, ParseBadWktIsActionable)
{
  EXPECT_THROW(parse_wkt("POLYGON (not-a-coordinate)"), ChartParseError);
  try {
    parse_wkt("POLYGON (not-a-coordinate)");
  } catch (const ChartParseError & error) {
    const std::string message = error.what();
    EXPECT_TRUE(
      message.find("invalid") != std::string::npos ||
      message.find("coordinate") != std::string::npos ||
      message.find("expected") != std::string::npos ||
      message.find("unterminated") != std::string::npos);
  }
}

TEST(Parser, LoadChartSanitizesEmptyTagsAndConvertsLonLat)
{
  const std::string source = write_chart(
    "river_chart_sanitize.xml",
    R"XML(<?xml version="1.0"?>
<S57ChartData totalMaps="1">
  <Map index="0">
    <Metadata><DSID_DSNM>TEST.000</DSID_DSNM></Metadata>
    <Bounds><MinX>100</MinX><MinY>20</MinY><MaxX>101</MaxX><MaxY>21</MaxY></Bounds>
    <Features><FeatureGroup acronym="DEPARE"><Feature>
      <RCID>1</RCID><ID>2</ID><Name>DEPARE</Name>
      <Attributes><></></Attributes>
      <Geometry><Geometries><Polygon>POLYGON ((0 0, 10 0, 0 10, 0 0))</Polygon></Geometries></Geometry>
    </Feature></FeatureGroup></Features>
  </Map>
</S57ChartData>)XML");

  const Chart chart = load_chart(source, 100.0, 20.0);
  EXPECT_EQ(chart.map_count, 1);
  EXPECT_EQ(chart.feature_count(), 1u);
  EXPECT_FALSE(chart.source_is_projected);
  const river_chart::Point first = chart.features[0].geometry[0].parts[0][0];
  EXPECT_NEAR(first.x, -10'460'610.0, 100.0);
  EXPECT_NEAR(first.y, -2'226'390.0, 100.0);
}

TEST(Parser, LoadChartUsesMapBoundsAsDefaultOrigin)
{
  const std::string source = write_chart(
    "river_chart_default_origin.xml",
    R"XML(<S57ChartData><Map><Bounds><MinX>120</MinX><MinY>30</MinY><MaxX>122</MaxX><MaxY>32</MaxY></Bounds>
<Features><FeatureGroup acronym="TEST"><Feature><Geometry><Geometries>
<Polygon>POLYGON ((121 31, 121.1 31, 121 31.1, 121 31))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>)XML");

  const Chart chart = load_chart(source);
  EXPECT_DOUBLE_EQ(chart.origin_longitude, 121.0);
  EXPECT_DOUBLE_EQ(chart.origin_latitude, 31.0);
  const river_chart::Point first = chart.features[0].geometry[0].parts[0][0];
  EXPECT_NEAR(first.x, 0.0, 1e-6);
  EXPECT_NEAR(first.y, 0.0, 1e-6);
}

TEST(Parser, LoadChartRetainsS57FeatureType)
{
  const std::string source = write_chart(
    "river_chart_feature_type.xml",
    R"XML(<S57ChartData><Map><Bounds><MinX>120</MinX><MinY>30</MinY><MaxX>122</MaxX><MaxY>32</MaxY></Bounds>
<Features><FeatureGroup acronym="LIGHTS"><Feature><Type>P</Type><Geometry><Geometries>
<Polygon>POLYGON ((121 31))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>)XML");

  const Chart chart = load_chart(source);
  EXPECT_EQ(chart.features[0].feature_type, "P");
  ASSERT_EQ(chart.features[0].geometry[0].parts.size(), 1u);
  ASSERT_EQ(chart.features[0].geometry[0].parts[0].size(), 1u);
  EXPECT_NEAR(chart.features[0].geometry[0].parts[0][0].x, 0.0, 1e-6);
}

TEST(Parser, LoadChartConvertsInvertedWebMercatorToLocalMeters)
{
  // 100 E / 30 N 与 101 E / 30 N，编码为取负 Y 的 Web-Mercator。
  const std::string source = write_chart(
    "river_chart_mercator.xml",
    R"XML(<S57ChartData><Map><Bounds><MinX>100</MinX><MinY>30</MinY><MaxX>101</MaxX><MaxY>30</MaxY></Bounds>
<Features><FeatureGroup acronym="DEPCNT"><Feature><Type>L</Type><Geometry><Geometries>
<Polygon>POLYGON ((11131949.079327 -3503549.843504, 11243268.570120 -3503549.843504))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup></Features></Map></S57ChartData>)XML");

  const Chart chart = load_chart(source, 100.0, 30.0);
  const auto & points = chart.features[0].geometry[0].parts[0];
  EXPECT_TRUE(chart.source_is_projected);
  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].x, 0.0, 0.01);
  EXPECT_NEAR(points[0].y, 0.0, 0.01);
  const double expected_x = 111'319.49079327358 * std::cos(30.0 * M_PI / 180.0);
  EXPECT_NEAR(points[1].x, expected_x, 0.01);
  EXPECT_NEAR(points[1].y, 0.0, 0.01);
}

TEST(Parser, RejectsInvalidScale)
{
  const std::string source = write_chart(
    "river_chart_bad_scale.xml",
    R"XML(<S57ChartData><Map><Bounds><MinX>1</MinX><MinY>1</MinY><MaxX>2</MaxX><MaxY>2</MaxY></Bounds>
<Features/></Map></S57ChartData>)XML");

  try {
    load_chart(source, std::nan(""), std::nan(""), 0.0);
    FAIL() << "load_chart should reject scale=0";
  } catch (const ChartParseError & error) {
    EXPECT_NE(std::string(error.what()).find("scale"), std::string::npos);
  }
}

TEST(Parser, MetadataJsonMatchesPythonLayout)
{
  ChartFeature feature;
  feature.group = "TEST";
  feature.geometry = {Geometry{"POLYGON", {{{0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}}}}};

  Chart chart;
  chart.features = {feature};
  chart.map_count = 2;
  chart.bounds = {1.0, 2.0, 3.0, 4.0};
  chart.origin_longitude = 6.0;
  chart.origin_latitude = 5.0;
  chart.scale = 7.0;
  chart.source_is_projected = false;
  chart.source_path = "/tmp/a.xml";

  EXPECT_EQ(
    chart.metadata_json(),
    "{\"bounds_lon_lat\": [1.0, 2.0, 3.0, 4.0], \"feature_count\": 1, "
    "\"groups\": [\"TEST\"], \"map_count\": 2, \"origin_latitude\": 5.0, "
    "\"origin_longitude\": 6.0, \"scale\": 7.0, \"source\": \"/tmp/a.xml\", "
    "\"source_is_projected\": false}");
}
