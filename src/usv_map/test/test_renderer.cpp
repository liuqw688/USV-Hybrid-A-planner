/**
 * @file test_renderer.cpp
 * @brief Marker / 规划 JSON 构造测试。
 */


#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>

#include "usv_map/parser.hpp"
#include "usv_map/renderer.hpp"

namespace
{

using usv_map::Chart;
using usv_map::ChartFeature;
using usv_map::Geometry;
using usv_map::Point;
using usv_map::RenderConfig;
using usv_map::Renderer;
using visualization_msgs::msg::Marker;

/** @brief 渲染单元测试覆盖任意 S-57 分组，因此关闭运营视图过滤。 */
RenderConfig test_config()
{
  RenderConfig config;
  config.focus_navigation_features = false;
  config.point_size = 2.0;
  return config;
}

builtin_interfaces::msg::Time stamp()
{
  return builtin_interfaces::msg::Time();
}

Chart chart_with(ChartFeature feature)
{
  Chart chart;
  chart.features.push_back(std::move(feature));
  chart.map_count = 1;
  chart.bounds = {108.0, 30.0, 109.0, 31.0};
  chart.origin_longitude = 108.5;
  chart.origin_latitude = 30.5;
  chart.source_is_projected = true;
  return chart;
}

}  // namespace

TEST(Renderer, PointFeatureWrappedAsPolygonPublishesRvizPoints)
{
  // S-57 点要素被导出成 POLYGON WKT 时不能消失。
  ChartFeature feature;
  feature.group = "LIGHTS";
  feature.feature_type = "P";
  feature.name = "LIGHTS";
  feature.geometry = {Geometry{"POLYGON", {{{12.5, -7.25}}}}};

  const Renderer renderer(test_config());
  const auto markers = renderer.build_markers(chart_with(feature), stamp());

  ASSERT_EQ(markers.markers.size(), 2u);
  const Marker & marker = markers.markers[1];
  EXPECT_EQ(marker.action, Marker::ADD);
  EXPECT_EQ(marker.type, Marker::POINTS);
  EXPECT_EQ(marker.ns, "s57/LIGHTS");
  EXPECT_NEAR(marker.scale.x, 2.0, 1e-9);
  EXPECT_NEAR(marker.scale.y, 2.0, 1e-9);
  ASSERT_EQ(marker.points.size(), 1u);
  EXPECT_NEAR(marker.points[0].x, 12.5, 1e-9);
  EXPECT_NEAR(marker.points[0].y, -7.25, 1e-9);
  EXPECT_NEAR(marker.points[0].z, 0.0, 1e-9);
}

TEST(Renderer, LineFeatureDropsExporterAddedClosingVertex)
{
  // S-57 线要素被导出成闭合 POLYGON 时，在 RViz 中仍需保持开放。
  ChartFeature feature;
  feature.group = "DEPCNT";
  feature.feature_type = "L";
  feature.name = "DEPCNT";
  feature.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {0.0, 0.0}}}}};

  const Renderer renderer(test_config());
  const auto markers = renderer.build_markers(chart_with(feature), stamp());

  ASSERT_EQ(markers.markers.size(), 2u);
  const Marker & marker = markers.markers[1];
  EXPECT_EQ(marker.type, Marker::LINE_STRIP);
  ASSERT_EQ(marker.points.size(), 3u);
  EXPECT_NEAR(marker.points[0].x, 0.0, 1e-9);
  EXPECT_NEAR(marker.points[0].y, 0.0, 1e-9);
  EXPECT_NEAR(marker.points[1].x, 10.0, 1e-9);
  EXPECT_NEAR(marker.points[1].y, 0.0, 1e-9);
  EXPECT_NEAR(marker.points[2].x, 10.0, 1e-9);
  EXPECT_NEAR(marker.points[2].y, 5.0, 1e-9);
}

TEST(Renderer, LaneDataPreservesTsslptOrientation)
{
  // 规划用静态消息必须保留 Marker 中没有的 ORIENT 属性。
  ChartFeature feature;
  feature.group = "TSSLPT";
  feature.feature_type = "A";
  feature.name = "TSSLPT";
  feature.attributes["ORIENT"] = "327";
  feature.attributes["DIRECTION"] = "CW";
  feature.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {5.0, 0.0}, {5.0, 4.0}, {0.0, 0.0}}}}};

  const Renderer renderer(test_config());
  EXPECT_EQ(
    renderer.lane_data_json(chart_with(feature)),
    "{\"frame_id\":\"map\",\"lanes\":[{\"lane_id\":0,\"orient_deg\":327.0,"
    "\"direction\":\"CW\",\"points\":[[0.0,0.0],[5.0,0.0],[5.0,4.0],[0.0,0.0]]}],"
    "\"obstacles\":[]}");
}

TEST(Renderer, LaneDataPreservesBothTrafficDirections)
{
  ChartFeature clockwise;
  clockwise.group = "TSSLPT";
  clockwise.feature_type = "A";
  clockwise.attributes["ORIENT"] = "90";
  clockwise.attributes["DIRECTION"] = "CW";
  clockwise.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {5.0, 0.0}, {5.0, 2.0}, {0.0, 0.0}}}}};

  ChartFeature counterclockwise = clockwise;
  counterclockwise.attributes["ORIENT"] = "270";
  counterclockwise.attributes["DIRECTION"] = "CCW";
  counterclockwise.geometry = {
    Geometry{"POLYGON", {{{0.0, 3.0}, {5.0, 3.0}, {5.0, 5.0}, {0.0, 3.0}}}}};

  Chart chart = chart_with(clockwise);
  chart.features.push_back(std::move(counterclockwise));
  const std::string json = Renderer(test_config()).lane_data_json(chart);
  EXPECT_NE(json.find("\"direction\":\"CW\""), std::string::npos);
  EXPECT_NE(json.find("\"direction\":\"CCW\""), std::string::npos);
}

TEST(Renderer, TsslptMarkerShowsDirectionArrow)
{
  ChartFeature feature;
  feature.group = "TSSLPT";
  feature.feature_type = "A";
  feature.attributes["ORIENT"] = "90";
  feature.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}}}}};

  const Renderer renderer(test_config());
  const auto markers = renderer.build_markers(chart_with(feature), stamp());

  ASSERT_EQ(markers.markers.size(), 3u);
  const Marker & direction = markers.markers[2];
  EXPECT_EQ(direction.ns, "s57/TSSLPT_DIRECTION");
  EXPECT_EQ(direction.type, Marker::ARROW);
  ASSERT_EQ(direction.points.size(), 2u);
  EXPECT_NEAR(direction.points[1].x - direction.points[0].x, 8.0, 1e-9);
  EXPECT_NEAR(direction.points[1].y - direction.points[0].y, 0.0, 1e-9);
}

TEST(Renderer, PlanningGridMarksLandAsObstacleAndChannelAsFree)
{
  // 航道与陆地重叠时，陆地及其膨胀边界仍不可通行。
  ChartFeature land;
  land.group = "LNDARE";
  land.feature_type = "A";
  land.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}}}}};

  ChartFeature channel;
  channel.group = "TSSLPT";
  channel.feature_type = "L";
  channel.geometry = {Geometry{"LINESTRING", {{{5.0, 5.0}, {40.0, 5.0}}}}};

  Chart chart = chart_with(land);
  chart.features.push_back(channel);

  RenderConfig config = test_config();
  config.planning_resolution = 1.0;
  config.obstacle_inflation = 1.0;
  config.channel_corridor_width = 2.0;

  const Renderer renderer(config);
  auto cost_grid = renderer.build_costmap(chart, stamp());

  EXPECT_EQ(cost_grid.header.frame_id, "map");
  ASSERT_EQ(cost_grid.info.width, 41u);
  ASSERT_EQ(cost_grid.info.height, 11u);
  ASSERT_EQ(cost_grid.data.size(), 41u * 11u);
  EXPECT_EQ(cost_grid.data[5u * cost_grid.info.width + 5u], 100);
  EXPECT_EQ(cost_grid.data[5u * cost_grid.info.width + 11u], 100);
  EXPECT_EQ(cost_grid.data[5u * cost_grid.info.width + 12u], 0);
  EXPECT_EQ(cost_grid.data[20u], config.channel_outside_cost);
}

TEST(Renderer, PlanningGridPreservesWaterInsideLandPolygonHole)
{
  ChartFeature water;
  water.group = "DEPARE";
  water.feature_type = "A";
  water.geometry = {
    Geometry{"POLYGON", {{{10.0, 10.0}, {30.0, 10.0}, {30.0, 30.0},
                           {10.0, 30.0}, {10.0, 10.0}}}}};

  ChartFeature land;
  land.group = "LNDARE";
  land.feature_type = "A";
  land.geometry = {Geometry{"POLYGON", {
      {{0.0, 0.0}, {40.0, 0.0}, {40.0, 40.0}, {0.0, 40.0}, {0.0, 0.0}},
      {{10.0, 10.0}, {30.0, 10.0}, {30.0, 30.0}, {10.0, 30.0}, {10.0, 10.0}}}}};

  Chart chart = chart_with(land);
  chart.features.push_back(water);
  RenderConfig config = test_config();
  config.planning_resolution = 1.0;
  config.obstacle_inflation = 0.0;
  config.channel_outside_cost = 60;

  const Renderer renderer(config);
  const auto grid = renderer.build_costmap(chart, stamp());
  ASSERT_EQ(grid.info.width, 41u);
  const auto cell = [&grid](unsigned int x, unsigned int y) {
      return grid.data[static_cast<std::size_t>(y) * grid.info.width + x];
    };
  EXPECT_EQ(cell(5, 5), 100);
  EXPECT_EQ(cell(20, 20), 0);
  EXPECT_EQ(cell(40, 40), 100);
}

TEST(Renderer, ReefsRemainVisibleWithoutChangingStaticCosts)
{
  ChartFeature land;
  land.group = "LNDARE";
  land.name = "RVIZ_NO_GO";
  land.feature_type = "A";
  land.geometry = {Geometry{"POLYGON", {
      {{20.0, 20.0}, {30.0, 20.0}, {30.0, 30.0}, {20.0, 30.0}, {20.0, 20.0}}}}};

  ChartFeature water;
  water.group = "DEPARE";
  water.feature_type = "A";
  water.geometry = {Geometry{"POLYGON", {
      {{0.0, 0.0}, {40.0, 0.0}, {40.0, 40.0}, {0.0, 40.0}, {0.0, 0.0}}}}};

  ChartFeature reef;
  reef.group = "LNDARE";
  reef.feature_type = "A";
  reef.attributes["OBSTACLE_KIND"] = "REEF";
  // 暗礁与陆地重叠；省略暗礁时仍应保留陆地障碍。
  reef.geometry = {Geometry{"POLYGON", {
      {{5.0, 5.0}, {25.0, 5.0}, {25.0, 25.0}, {5.0, 25.0}, {5.0, 5.0}}}}};

  Chart chart = chart_with(land);
  chart.features.push_back(water);
  RenderConfig config;
  config.planning_resolution = 1.0;
  config.obstacle_inflation = 1.0;
  const Renderer renderer(config);
  const auto baseline = renderer.build_costmap(chart, stamp());
  chart.features.push_back(reef);
  const auto grid = renderer.build_costmap(chart, stamp());

  EXPECT_EQ(grid.data, baseline.data);
  EXPECT_EQ(grid.data[10u * grid.info.width + 10u], 0);
  EXPECT_EQ(grid.data[23u * grid.info.width + 23u], 100);
  const auto markers = renderer.build_markers(chart, stamp());
  ASSERT_EQ(markers.markers.size(), 3u);
  const auto & reef_marker = markers.markers.back();
  EXPECT_EQ(reef_marker.ns, "s57/LNDARE");
  EXPECT_EQ(reef_marker.type, Marker::LINE_STRIP);
  ASSERT_EQ(reef_marker.points.size(), 5u);
  EXPECT_DOUBLE_EQ(reef_marker.points.front().x, 5.0);
  EXPECT_DOUBLE_EQ(reef_marker.points.front().y, 5.0);
  EXPECT_DOUBLE_EQ(reef_marker.points.front().x, reef_marker.points.back().x);
  EXPECT_DOUBLE_EQ(reef_marker.points.front().y, reef_marker.points.back().y);
}

TEST(Renderer, PlanningGridCropsLowerLeftAtConfiguredResolution)
{
  ChartFeature land;
  land.group = "LNDARE";
  land.feature_type = "A";
  land.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {90.0, 0.0}, {90.0, 60.0},
                           {0.0, 60.0}, {0.0, 0.0}}}}};

  RenderConfig config = test_config();
  config.planning_resolution = 1.0;
  config.planning_max_cells = 1000000;
  config.planning_crop_width_fraction = 1.0 / 3.0;
  config.planning_crop_height_fraction = 1.0 / 3.0;

  const Renderer renderer(config);
  const auto grid = renderer.build_costmap(chart_with(land), stamp());

  EXPECT_DOUBLE_EQ(grid.info.resolution, 1.0);
  EXPECT_EQ(grid.info.width, 31u);
  EXPECT_EQ(grid.info.height, 21u);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, 0.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 0.0);
  EXPECT_EQ(grid.data.size(), 31u * 21u);
}
