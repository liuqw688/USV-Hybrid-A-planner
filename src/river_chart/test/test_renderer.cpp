/// @file test_renderer.cpp
/// @brief Marker / 规划 JSON 构造测试（对应原 Python test_node.py）。
///
/// @author susheng
/// @date 2026-08-28

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>

#include "river_chart/parser.hpp"
#include "river_chart/renderer.hpp"

namespace
{

using river_chart::Chart;
using river_chart::ChartFeature;
using river_chart::Geometry;
using river_chart::Point;
using river_chart::RenderConfig;
using river_chart::Renderer;
using visualization_msgs::msg::Marker;

/// @brief 渲染单元测试覆盖任意 S-57 分组，因此关闭运营视图过滤。
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
  chart.scale = 1.0;
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
  feature.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {5.0, 0.0}, {5.0, 4.0}, {0.0, 0.0}}}}};

  const Renderer renderer(test_config());
  EXPECT_EQ(
    renderer.lane_data_json(chart_with(feature)),
    "{\"frame_id\":\"map\",\"lanes\":[{\"lane_id\":0,\"orient_deg\":327.0,"
    "\"points\":[[0.0,0.0],[5.0,0.0],[5.0,4.0],[0.0,0.0]]}]}");
}

TEST(Renderer, PlanningGridMarksLandAsObstacleAndChannelAsFree)
{
  // 陆地 LNDARE 不可通行，航道 TSSBPT 走廊代价为 0。
  ChartFeature land;
  land.group = "LNDARE";
  land.feature_type = "A";
  land.geometry = {
    Geometry{"POLYGON", {{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}}}}};

  ChartFeature channel;
  channel.group = "TSSLPT";
  channel.feature_type = "A";
  channel.geometry = {
    Geometry{"POLYGON", {{{20.0, 0.0}, {40.0, 0.0}, {40.0, 10.0}, {20.0, 0.0}}}}};

  Chart chart = chart_with(land);
  chart.features.push_back(channel);

  RenderConfig config = test_config();
  config.planning_resolution = 1.0;
  config.obstacle_inflation = 0.0;
  config.channel_corridor_width = 2.0;

  const Renderer renderer(config);
  nav_msgs::msg::OccupancyGrid static_grid;
  nav_msgs::msg::OccupancyGrid cost_grid;
  renderer.build_planning_grids(chart, {}, stamp(), static_grid, cost_grid);

  ASSERT_GT(static_grid.data.size(), 0u);
  EXPECT_EQ(static_grid.data.size(), cost_grid.data.size());
  EXPECT_TRUE(
    std::find(static_grid.data.begin(), static_grid.data.end(), 100) != static_grid.data.end());
  EXPECT_TRUE(std::find(cost_grid.data.begin(), cost_grid.data.end(), 0) != cost_grid.data.end());
  EXPECT_TRUE(std::find(cost_grid.data.begin(), cost_grid.data.end(), 100) != cost_grid.data.end());
}
