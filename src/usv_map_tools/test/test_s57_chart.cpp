/**
 * @file test_s57_chart.cpp
 * @brief 验证 S-57 导出结果及要素回读。
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>

#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include "s57_chart.hpp"

namespace
{

using DatasetPtr = std::unique_ptr<GDALDataset, decltype(&GDALClose)>;
using FeaturePtr = std::unique_ptr<OGRFeature>;

class S57Chart : public ::testing::Test
{
protected:
  std::filesystem::path directory;
  std::string input;
  std::string output;

  void SetUp() override
  {
    directory = std::filesystem::path(::testing::TempDir()) /
      ("usv_s57_" + std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    input = (directory / "source.xml").string();
    output = (directory / "ZZ5TEST1.000").string();
  }

  void TearDown() override {std::filesystem::remove_all(directory);}

  usv_map::Chart source(const std::string & features)
  {
    std::ofstream stream(input);
    stream << "<S57ChartData><Map><Bounds><MinX>106</MinX><MinY>29</MinY>"
      "<MaxX>107</MaxX><MaxY>30</MaxY></Bounds><Features>" << features <<
      "</Features></Map></S57ChartData>";
    stream.close();
    return usv_map::load_chart(input);
  }

  DatasetPtr reopen()
  {
    return DatasetPtr(static_cast<GDALDataset *>(GDALOpenEx(output.c_str(),
      GDAL_OF_VECTOR, nullptr, nullptr, nullptr)), GDALClose);
  }
};

TEST_F(S57Chart, RoundTripPreservesHolesLinesPointsAttributesAndRecordCounts)
{
  const auto chart = source(R"XML(
<FeatureGroup acronym="LNDARE"><Feature><RCID>1</RCID><ID>same</ID>
<Name>test land</Name><Type>A</Type><Attributes><OBSTACLE_KIND>REEF</OBSTACLE_KIND>
<INFORM>Original note</INFORM><STATUS>1,2</STATUS></Attributes><Geometry><Geometries>
<Polygon>MULTIPOLYGON (((106.1 29.1,106.3 29.1,106.3 29.3,106.1 29.3,106.1 29.1),
(106.15 29.15,106.2 29.15,106.2 29.2,106.15 29.2,106.15 29.15)),
((106.4 29.4,106.5 29.4,106.5 29.5,106.4 29.5,106.4 29.4)))</Polygon>
</Geometries></Geometry></Feature>
<Feature><RCID>1</RCID><Type>P</Type><Geometry><Geometries>
<Point>POINT (106.6 29.6)</Point></Geometries></Geometry></Feature></FeatureGroup>
<FeatureGroup acronym="TSSBND"><Feature><RCID>1</RCID><Type>L</Type><Geometry>
<Geometries><Polygon>POLYGON ((106.1 29.1,106.2 29.2,106.1 29.1))</Polygon>
</Geometries></Geometry></Feature></FeatureGroup>)XML");
  const auto report = usv_map_tools::export_s57(chart, output);
  EXPECT_EQ(report.invalid_source_areas, 0);
  EXPECT_EQ(report.feature_counts.at("LNDARE"), 3);
  auto dataset = reopen();
  ASSERT_NE(dataset, nullptr);
  EXPECT_STREQ(dataset->GetDriver()->GetDescription(), "S57");
  auto * land_layer = dataset->GetLayerByName("LNDARE");
  FeaturePtr land(land_layer->GetNextFeature());
  ASSERT_NE(land, nullptr);
  ASSERT_EQ(wkbFlatten(land->GetGeometryRef()->getGeometryType()), wkbPolygon);
  const auto * polygon = land->GetGeometryRef()->toPolygon();
  EXPECT_EQ(polygon->getNumInteriorRings(), 1);
  EXPECT_NEAR(polygon->get_Area(), 0.0375, 1e-7);
  EXPECT_EQ(land->GetFieldAsInteger("OBJL"), 71);
  const std::string information = land->GetFieldAsString("INFORM");
  EXPECT_NE(information.find("Original note"), std::string::npos);
  EXPECT_NE(information.find("\"OBSTACLE_KIND\":\"REEF\""), std::string::npos);
  EXPECT_STREQ(land->GetFieldAsString("OBJNAM"), "test land");
  char ** status = land->GetFieldAsStringList("STATUS");
  ASSERT_NE(status, nullptr);
  EXPECT_STREQ(status[0], "1");
  EXPECT_STREQ(status[1], "2");
  FeaturePtr second_area(land_layer->GetNextFeature());
  ASSERT_NE(second_area, nullptr);
  ASSERT_EQ(wkbFlatten(second_area->GetGeometryRef()->getGeometryType()), wkbPolygon);
  EXPECT_NEAR(second_area->GetGeometryRef()->toPolygon()->get_Area(), 0.01, 1e-7);
  FeaturePtr point(land_layer->GetNextFeature());
  ASSERT_NE(point, nullptr);
  EXPECT_NEAR(point->GetGeometryRef()->toPoint()->getX(), 106.6, 1e-7);
  FeaturePtr line(dataset->GetLayerByName("TSSBND")->GetNextFeature());
  ASSERT_NE(line, nullptr);
  EXPECT_EQ(line->GetGeometryRef()->toLineString()->getNumPoints(), 2);
  EXPECT_NEAR(line->GetGeometryRef()->toLineString()->getX(1), 106.2, 1e-7);
  FeaturePtr cover(dataset->GetLayerByName("M_COVR")->GetNextFeature());
  EXPECT_EQ(cover->GetFieldAsInteger("CATCOV"), 1);
  FeaturePtr metadata(dataset->GetLayerByName("DSID")->GetNextFeature());
  EXPECT_EQ(metadata->GetFieldAsInteger("DSSI_NOGR"), 4);
  EXPECT_EQ(metadata->GetFieldAsInteger("DSSI_NOMR"), 1);
  EXPECT_EQ(metadata->GetFieldAsInteger("DSSI_NOIN"), 1);
  EXPECT_EQ(metadata->GetFieldAsInteger("DSSI_NOED"), 5);
  EXPECT_EQ(metadata->GetFieldAsInteger("DSID_AGEN"), 0);
  EXPECT_STREQ(metadata->GetFieldAsString("DSID_DSNM"), "ZZ5TEST1.000");
}

TEST_F(S57Chart, RestoresNegativeMercatorNorthingAndDoesNotOverwriteOutput)
{
  const auto chart = source(R"XML(
<FeatureGroup acronym="BERTHS"><Feature><Type>A</Type><Attributes>
<WIDTH_M>3</WIDTH_M><ORIENT>180</ORIENT></Attributes><Geometry><Geometries>
<Polygon>POLYGON ((11879896 -3440323,11879900 -3440323,11879900 -3440311,
11879896 -3440311,11879896 -3440323))</Polygon></Geometries></Geometry>
</Feature></FeatureGroup>)XML");
  ASSERT_TRUE(chart.source_is_projected);
  usv_map_tools::export_s57(chart, output);
  auto dataset = reopen();
  ASSERT_NE(dataset, nullptr);
  FeaturePtr berth(dataset->GetLayerByName("BERTHS")->GetNextFeature());
  OGREnvelope envelope;
  berth->GetGeometryRef()->getEnvelope(&envelope);
  EXPECT_NEAR(envelope.MinX, 106.718921505504, 1e-7);
  EXPECT_NEAR(envelope.MinY, 29.506809243032, 1e-7);
  const std::string information = berth->GetFieldAsString("INFORM");
  EXPECT_NE(information.find("\"WIDTH_M\":\"3\""), std::string::npos);
  EXPECT_NE(information.find("\"ORIENT\":\"180\""), std::string::npos);
  const auto size = std::filesystem::file_size(output);
  EXPECT_THROW(usv_map_tools::export_s57(chart, output), std::runtime_error);
  EXPECT_EQ(std::filesystem::file_size(output), size);
}

TEST_F(S57Chart, FailureRemovesIncompleteChart)
{
  auto chart = source(R"XML(
<FeatureGroup acronym="LNDARE"><Feature><Type>P</Type><Geometry><Geometries>
<Point>POINT (106.6 29.6)</Point></Geometries></Geometry></Feature></FeatureGroup>)XML");
  chart.features[0].group = "UNKNOWN_CLASS";
  EXPECT_THROW(usv_map_tools::export_s57(chart, output), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(output));
}

}  // namespace
