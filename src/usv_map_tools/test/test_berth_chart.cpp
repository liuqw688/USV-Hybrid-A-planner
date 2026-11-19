/**
 * @file test_berth_chart.cpp
 * @brief 验证泊位构造、海图边界校验及 XML 保存。
 */

#include "berth_chart.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace
{
using usv_map_tools::Berth;
using usv_map_tools::BerthChart;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 6378137.0;

const tinyxml2::XMLElement * group(const tinyxml2::XMLDocument & document, const char * name)
{
  const auto * features = document.RootElement()->FirstChildElement("Map")->FirstChildElement("Features");
  for (const auto * item = features->FirstChildElement("FeatureGroup"); item;
    item = item->NextSiblingElement("FeatureGroup"))
  {
    if (item->Attribute("acronym", name)) {
      return item;
    }
  }
  return nullptr;
}

std::string xml(const tinyxml2::XMLNode * node)
{
  tinyxml2::XMLPrinter printer;
  node->Accept(&printer);
  return printer.CStr();
}

class BerthChartTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/usv_map_berth_XXXXXX";
    const auto * directory = ::mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    directory_ = directory;
    input_ = directory_ / "input.xml";
    output_ = directory_ / "output.xml";
    write_source();
  }

  void TearDown() override
  {
    if (!directory_.empty()) {
      std::filesystem::remove_all(directory_);
    }
  }

  void write_source(bool projected = false)
  {
    const std::string polygon = projected ?
      "POLYGON ((11879500 -3438000, 11879520 -3438000, 11879500 -3438020, 11879500 -3438000))" :
      "POLYGON ((106.71 29.50, 106.72 29.50, 106.71 29.51, 106.71 29.50))";
    std::ofstream stream(input_);
    stream << "<S57ChartData><Map><Metadata><DSID_DSNM>keep me</DSID_DSNM></Metadata>"
      "<Bounds><MinX>106.70</MinX><MinY>29.49</MinY><MaxX>106.74</MaxX><MaxY>29.53</MaxY></Bounds>"
      "<Features><FeatureGroup acronym=\"DEPARE\" count=\"1\"><Feature>"
      "<ID>7</ID><RCID>7</RCID><Geometry><Geometries><Polygon>" << polygon <<
      "</Polygon></Geometries></Geometry></Feature></FeatureGroup>"
      "<FeatureGroup acronym=\"LNDARE\" count=\"1\"><Feature><ID>21</ID><RCID>31</RCID>"
      "<Attributes><INFORM>hand selected</INFORM></Attributes><Geometry><Geometries><Polygon>" <<
      polygon << "</Polygon></Geometries></Geometry></Feature></FeatureGroup>"
      "</Features></Map></S57ChartData>";
  }

  std::filesystem::path directory_;
  std::filesystem::path input_;
  std::filesystem::path output_;
};

TEST(BerthGeometry, RotatesRectangleAroundCentre)
{
  const auto corners = Berth{0, {10.0, 20.0}, kPi / 2, 10.0, 3.0}.corners();
  EXPECT_NEAR(corners[0].x, 11.5, 1e-12);
  EXPECT_NEAR(corners[0].y, 15.0, 1e-12);
  EXPECT_NEAR(corners[1].x, 11.5, 1e-12);
  EXPECT_NEAR(corners[1].y, 25.0, 1e-12);
  EXPECT_NEAR(std::hypot(corners[2].x - corners[1].x, corners[2].y - corners[1].y), 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(corners.front().x, corners.back().x);
  EXPECT_DOUBLE_EQ(corners.front().y, corners.back().y);
}

TEST_F(BerthChartTest, PreservesLandWaterAndMetadataAndResumes)
{
  tinyxml2::XMLDocument original;
  ASSERT_EQ(original.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  BerthChart chart(input_);
  EXPECT_EQ(chart.add({0, {12.0, -25.0}, kPi / 4, 12.0, 4.0}, output_).id, 32);
  tinyxml2::XMLDocument saved;
  ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(xml(group(original, "DEPARE")), xml(group(saved, "DEPARE")));
  EXPECT_EQ(xml(group(original, "LNDARE")), xml(group(saved, "LNDARE")));
  EXPECT_EQ(
    xml(original.RootElement()->FirstChildElement("Map")->FirstChildElement("Metadata")),
    xml(saved.RootElement()->FirstChildElement("Map")->FirstChildElement("Metadata")));
  ASSERT_NE(group(saved, "BERTHS"), nullptr);
  EXPECT_EQ(group(saved, "BERTHS")->IntAttribute("count"), 1);
  BerthChart resumed(output_);
  ASSERT_EQ(resumed.berths().size(), 1u);
  const auto berth = resumed.berths().front();
  EXPECT_NEAR(berth.center.x, 12.0, 1e-5);
  EXPECT_NEAR(berth.center.y, -25.0, 1e-5);
  EXPECT_NEAR(berth.yaw, kPi / 4, 1e-10);
  EXPECT_DOUBLE_EQ(berth.length, 12.0);
  EXPECT_DOUBLE_EQ(berth.width, 4.0);
  EXPECT_EQ(resumed.add({0, {-10.0, 20.0}, kPi / 2, 10.0, 3.0}, output_).id, 33);
  BerthChart reloaded(output_);
  EXPECT_EQ(reloaded.berths().size(), 2u);
  ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(group(saved, "BERTHS")->IntAttribute("count"), 2);
  // 使用独立输出路径时，源海图应保持不变。
  ASSERT_EQ(original.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(group(original, "BERTHS"), nullptr);
}

TEST_F(BerthChartTest, RejectsInvalidOrOutOfBoundsWithoutWriting)
{
  BerthChart chart(input_);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & berth : {
      Berth{0, {0, 0}, 0, 0, 3}, Berth{0, {0, 0}, 0, 10, -1},
      Berth{0, {nan, 0}, 0, 10, 3}, Berth{0, {0, 0}, nan, 10, 3},
      Berth{0, {0, 0}, 0, 10, nan}, Berth{0, {1e8, 0}, 0, 10, 3},
      Berth{0, {0, 0}, 0, 10000, 3}})
  {
    EXPECT_THROW(chart.add(berth, output_), std::invalid_argument);
  }
  EXPECT_FALSE(std::filesystem::exists(output_));
  EXPECT_TRUE(chart.berths().empty());
  EXPECT_EQ(chart.add({0, {0, 0}, 0, 10, 3}, output_).id, 32);
}

TEST_F(BerthChartTest, FailedSaveRollsBackNewAndExistingGroups)
{
  BerthChart chart(input_);
  const Berth berth{0, {0, 0}, 0, 10, 3};
  EXPECT_THROW(chart.add(berth, directory_), std::runtime_error);
  EXPECT_TRUE(chart.berths().empty());
  EXPECT_FALSE(std::filesystem::exists(directory_.string() + ".tmp"));
  EXPECT_EQ(chart.add(berth, output_).id, 32);
  EXPECT_THROW(chart.add(berth, directory_), std::runtime_error);
  EXPECT_EQ(chart.berths().size(), 1u);
  EXPECT_EQ(chart.add(berth, output_).id, 33);
  tinyxml2::XMLDocument saved;
  ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(group(saved, "BERTHS")->IntAttribute("count"), 2);
  EXPECT_EQ(BerthChart(output_).berths().size(), 2u);
}

TEST_F(BerthChartTest, WritesCoordinatesInTheSourceFormat)
{
  for (const bool projected : {false, true}) {
    write_source(projected);
    BerthChart chart(input_);
    chart.add({0, {0, 0}, 0, 10, 4}, output_);
    tinyxml2::XMLDocument saved;
    ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
    const auto * feature = group(saved, "BERTHS")->FirstChildElement("Feature");
    const auto * geometry = feature->FirstChildElement("Geometry");
    ASSERT_NE(geometry->FirstChildElement("BoundingBox"), nullptr);
    const std::string wkt = geometry->FirstChildElement("Geometries")->FirstChildElement("Polygon")->GetText();
    std::istringstream coordinates(wkt.substr(wkt.find_first_of("0123456789-")));
    double x = 0, y = 0;
    ASSERT_TRUE(static_cast<bool>(coordinates >> x >> y));
    const double lon_rad = projected ? x / kRadius : x * kPi / 180;
    const double lat_rad = projected ? std::atan(std::sinh(-y / kRadius)) : y * kPi / 180;
    EXPECT_NEAR(
      (lon_rad - chart.longitude() * kPi / 180) * kRadius * std::cos(chart.latitude() * kPi / 180),
      -5.0, 1e-5);
    EXPECT_NEAR((lat_rad - chart.latitude() * kPi / 180) * kRadius, -2.0, 1e-5);
  }
}

TEST_F(BerthChartTest, LatestBaseReplacesStaleLandAndKeepsBerthsWithUniqueIds)
{
  BerthChart original(input_);
  original.add({0, {12, -25}, kPi / 4, 12, 4}, output_);
  tinyxml2::XMLDocument old;
  ASSERT_EQ(old.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  const auto old_geometry = xml(group(old, "BERTHS")->FirstChildElement("Feature")->FirstChildElement("Geometry"));

  tinyxml2::XMLDocument latest;
  ASSERT_EQ(latest.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  auto * map = latest.RootElement()->FirstChildElement("Map");
  auto * land = map->FirstChildElement("Features")->LastChildElement("FeatureGroup");
  land->FirstChildElement("Feature")->FirstChildElement("Attributes")->FirstChildElement("INFORM")->SetText("updated land");
  auto * obstacle = land->FirstChildElement("Feature")->DeepClone(&latest)->ToElement();
  obstacle->FirstChildElement("ID")->SetText(32);
  obstacle->FirstChildElement("RCID")->SetText(32);
  land->InsertEndChild(obstacle);
  land->SetAttribute("count", 2);
  const auto base = directory_ / "latest.xml";
  ASSERT_EQ(latest.SaveFile(base.c_str()), tinyxml2::XML_SUCCESS);

  BerthChart merged(output_, base);
  ASSERT_EQ(merged.berths().size(), 1u);
  EXPECT_EQ(merged.berths().front().id, 33);  // 泊位与障碍物共用要素编号空间；编号 32 已由障碍物占用。
  EXPECT_NEAR(merged.berths().front().center.x, 12, 1e-5);
  EXPECT_NEAR(merged.berths().front().yaw, kPi / 4, 1e-10);
  EXPECT_EQ(merged.add({0, {0, 0}, 0, 10, 3}, output_).id, 34);
  tinyxml2::XMLDocument saved;
  ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(xml(group(latest, "LNDARE")), xml(group(saved, "LNDARE")));
  EXPECT_EQ(xml(group(latest, "DEPARE")), xml(group(saved, "DEPARE")));
  const auto * berth = group(saved, "BERTHS")->FirstChildElement("Feature");
  EXPECT_EQ(old_geometry, xml(berth->FirstChildElement("Geometry")));
  EXPECT_STREQ(berth->FirstChildElement("Name")->GetText(), "RVIZ_BERTH_33");
  EXPECT_EQ(group(saved, "BERTHS")->IntAttribute("count"), 2);
  EXPECT_EQ(BerthChart(output_, base).berths().front().id, 33);  // 再次加载后编号分配应保持稳定。
  EXPECT_EQ(BerthChart(output_, output_).berths().size(), 2u);
}

TEST_F(BerthChartTest, LatestBaseHonorsObstacleDeletion)
{
  BerthChart(input_).add({0, {0, 0}, 0, 10, 3}, output_);
  tinyxml2::XMLDocument latest;
  ASSERT_EQ(latest.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  auto * land = latest.RootElement()->FirstChildElement("Map")->FirstChildElement("Features")->LastChildElement("FeatureGroup");
  land->DeleteChildren();
  land->SetAttribute("count", 0);
  ASSERT_EQ(latest.SaveFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  BerthChart merged(output_, input_);
  ASSERT_EQ(merged.berths().size(), 1u);
  EXPECT_EQ(merged.berths().front().id, 32);
  merged.add({0, {10, 0}, 0, 10, 3}, output_);
  tinyxml2::XMLDocument saved;
  ASSERT_EQ(saved.LoadFile(output_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_EQ(group(saved, "LNDARE")->FirstChildElement("Feature"), nullptr);
  EXPECT_EQ(group(saved, "LNDARE")->IntAttribute("count"), 0);
  EXPECT_EQ(group(saved, "BERTHS")->IntAttribute("count"), 2);
}

TEST_F(BerthChartTest, RefusesToMergeDifferentMapsOrSourceCoordinates)
{
  BerthChart(input_).add({0, {0, 0}, 0, 10, 3}, output_);
  tinyxml2::XMLDocument changed;
  ASSERT_EQ(changed.LoadFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  changed.RootElement()->FirstChildElement("Map")->FirstChildElement("Bounds")->FirstChildElement("MinX")->SetText(106.69);
  ASSERT_EQ(changed.SaveFile(input_.c_str()), tinyxml2::XML_SUCCESS);
  EXPECT_THROW(BerthChart(output_, input_), std::runtime_error);
  write_source(true);
  EXPECT_THROW(BerthChart(output_, input_), std::runtime_error);
  EXPECT_EQ(BerthChart(output_).berths().size(), 1u);
}
}  // namespace
