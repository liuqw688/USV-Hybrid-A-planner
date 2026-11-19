/**
 * @file test_geometry.cpp
 * @brief 验证航道方向判定、多边形距离及栅格覆盖。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "channel_navigation_manager/geometry.hpp"

namespace cnm = channel_navigation_manager;

TEST(ChannelGeometry, ConvertsS57DirectionAndClassifiesTraffic)
{
  EXPECT_NEAR(cnm::s57OrientationToYaw(90.0), 0.0, 1.0e-9);
  EXPECT_NEAR(cnm::s57OrientationToYaw(0.0), M_PI / 2.0, 1.0e-9);
  cnm::Lane lane{1, 90.0, {{0, 0}, {5, 0}, {5, 5}, {0, 5}, {0, 0}}, "CW"};
  EXPECT_EQ(cnm::laneValue(0.0, lane), cnm::kSameDirectionCost);
  EXPECT_EQ(cnm::laneValue(M_PI, lane), cnm::kOppositeDirectionCost);
}

TEST(ChannelGeometry, TrafficDirectionIsStableAcrossCurvedLaneSections)
{
  cnm::Lane first;
  first.direction = "cw";
  cnm::Lane second;
  second.direction = "CW";
  cnm::Lane opposite;
  opposite.direction = "CCW";

  EXPECT_TRUE(cnm::sameTrafficDirection(first, second));
  EXPECT_FALSE(cnm::sameTrafficDirection(first, opposite));
}

TEST(ChannelGeometry, FillsPolygon)
{
  const cnm::Polygon polygon{{1, 1}, {4, 1}, {4, 4}, {1, 4}, {1, 1}};
  EXPECT_TRUE(cnm::pointInPolygon(2.0, 2.0, polygon));
  EXPECT_FALSE(cnm::pointInPolygon(5.0, 2.0, polygon));
  std::vector<int8_t> data(36, static_cast<int8_t>(cnm::kOpenWaterCost));
  cnm::fillPolygon(data, 6, 6, 1.0, 0.0, 0.0, polygon, cnm::kSameDirectionCost);
  EXPECT_EQ(data[2 * 6 + 2], cnm::kSameDirectionCost);
  EXPECT_EQ(data[0], cnm::kOpenWaterCost);
}

TEST(ChannelGeometry, SensorDistanceIncludesPolygonInterior)
{
  const cnm::Polygon reef{{10, 10}, {12, 10}, {12, 12}, {10, 12}, {10, 10}};
  EXPECT_DOUBLE_EQ(cnm::pointPolygonDistance(11.0, 11.0, reef), 0.0);
  EXPECT_LE(cnm::pointPolygonDistance(0.0, 11.0, reef), 40.0);
  EXPECT_GT(cnm::pointPolygonDistance(100.0, 100.0, reef), 40.0);
}

// 覆盖缓存必须使用地图原点和米制分辨率，不能将世界坐标直接作为栅格下标。
TEST(ChannelGeometry, CachedCellsUseGridOriginAndResolution)
{
  const cnm::Polygon polygon{{-3, 10}, {1, 10}, {1, 14}, {-3, 14}, {-3, 10}};
  const std::vector<std::size_t> expected{1, 2, 4, 5};
  EXPECT_EQ(cnm::polygonCells(3, 3, 2.0, -5.0, 10.0, polygon), expected);
}

TEST(ChannelGeometry, CachedCellsClipOutsideMapAndIgnoreEmptyGeometry)
{
  const cnm::Polygon polygon{{-10, -10}, {1, -10}, {1, 1}, {-10, 1}, {-10, -10}};
  const std::vector<std::size_t> expected{0, 1};
  EXPECT_EQ(cnm::polygonCells(3, 3, 1.0, 0.0, 0.0, polygon), expected);
  EXPECT_TRUE(cnm::polygonCells(3, 3, 1.0, 0.0, 0.0, {}).empty());
  EXPECT_TRUE(cnm::polygonCells(0, 3, 1.0, 0.0, 0.0, polygon).empty());
}

TEST(ChannelGeometry, CenterOffsetUsesBanksInsteadOfSectionEndCaps)
{
  cnm::Lane lane{1, 0.0, {{-10, 0}, {10, 0}, {10, 100}, {-10, 100}}, "CW"};
  EXPECT_NEAR(cnm::laneCenterOffset({0, 0.01}, lane), 0.0, 1e-12);
  EXPECT_NEAR(cnm::laneCenterOffset({0, 99.99}, lane), 0.0, 1e-12);
  EXPECT_NEAR(cnm::laneCenterOffset({5, 40}, lane), 0.5, 1e-12);
  EXPECT_NEAR(cnm::laneCenterOffset({-10, 40}, lane), 1.0, 1e-12);
}

TEST(ChannelGeometry, CenterOffsetFollowsRotatedAndTaperedSections)
{
  cnm::Lane lane{1, 90.0, {{0, -4}, {40, -8}, {40, 8}, {0, 4}}, "CW"};
  EXPECT_NEAR(cnm::laneCenterOffset({20, 0}, lane), 0.0, 1e-12);
  EXPECT_NEAR(cnm::laneCenterOffset({20, 3}, lane), 0.5, 1e-12);
  for (auto & p : lane.points) {
    const double x = p.x, y = p.y;
    p = {(x - y) / std::sqrt(2.0), (x + y) / std::sqrt(2.0)};
  }
  lane.orient_deg = 45.0;
  EXPECT_NEAR(cnm::laneCenterOffset({17/std::sqrt(2.0), 23/std::sqrt(2.0)}, lane),
    0.5, 1e-12);
}

TEST(ChannelGeometry, CenterOffsetSelectsContainingIntervalInConcavePolygon)
{
  cnm::Lane lane{1, 0.0,
    {{-10, 0}, {10, 0}, {10, 20}, {4, 20}, {4, 4}, {-4, 4}, {-4, 20}, {-10, 20}}, "CW"};
  EXPECT_NEAR(cnm::laneCenterOffset({-7, 10}, lane), 0.0, 1e-12);
  EXPECT_NEAR(cnm::laneCenterOffset({7, 10}, lane), 0.0, 1e-12);
  EXPECT_NEAR(cnm::laneCenterOffset({5.5, 10}, lane), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(cnm::laneCenterOffset({0, 0}, {}), 0.0);
}
