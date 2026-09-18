#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "channel_navigation_manager/geometry.hpp"

namespace cnm = channel_navigation_manager;

TEST(ChannelGeometry, S57OrientationAndLaneRole)
{
  EXPECT_NEAR(cnm::s57OrientationToYaw(90.0), 0.0, 1.0e-9);
  EXPECT_NEAR(cnm::s57OrientationToYaw(0.0), M_PI / 2.0, 1.0e-9);
  cnm::Lane lane{1, 90.0, {{0, 0}, {5, 0}, {5, 5}, {0, 5}, {0, 0}}};
  EXPECT_EQ(cnm::laneValue(0.0, lane), cnm::kMainLane);
  EXPECT_EQ(cnm::laneValue(M_PI, lane), cnm::kOppositeLane);
}

TEST(ChannelGeometry, PolygonFill)
{
  const std::vector<cnm::Point2D> polygon{{1, 1}, {4, 1}, {4, 4}, {1, 4}, {1, 1}};
  EXPECT_TRUE(cnm::pointInPolygon(2.0, 2.0, polygon));
  EXPECT_FALSE(cnm::pointInPolygon(5.0, 2.0, polygon));
  std::vector<int8_t> data(36, 45);
  cnm::fillPolygon(data, 6, 6, 1.0, 0.0, 0.0, polygon, cnm::kMainLane);
  EXPECT_EQ(data[2 * 6 + 2], 0);
  EXPECT_EQ(data[0], 45);
}

TEST(ChannelGeometry, CenterIsStronglyCheaperThanEdgeButEdgeRemainsMain)
{
  std::vector<int8_t> data(24 * 24, 45);
  const std::vector<cnm::Point2D> polygon{{2, 2}, {22, 2}, {22, 22}, {2, 22}, {2, 2}};
  cnm::fillPolygon(data, 24, 24, 1.0, 0.0, 0.0, polygon, cnm::kMainLane);
  cnm::applyMainLaneCenterGradient(data, 24, 24, 1.0, 20, 16.0);
  const int edge = data[12 * 24 + 2];
  const int center = data[12 * 24 + 12];
  EXPECT_LT(center, edge);
  EXPECT_EQ(center, 0);
  EXPECT_LE(edge, 20);
  EXPECT_LT(edge, cnm::kOutsideChannel);
  EXPECT_EQ(data[0], 45);
}

TEST(ChannelGeometry, NarrowLaneCenterIsAlsoZero)
{
  std::vector<int8_t> data(20 * 20, 45);
  const std::vector<cnm::Point2D> lane{{6, 2}, {14, 2}, {14, 18}, {6, 18}, {6, 2}};
  cnm::fillPolygon(data, 20, 20, 1.0, 0.0, 0.0, lane, cnm::kMainLane);
  cnm::applyMainLaneCenterGradient(data, 20, 20, 1.0, 20, 16.0);
  EXPECT_EQ(data[10 * 20 + 10], 0);
  EXPECT_GT(data[10 * 20 + 6], data[10 * 20 + 10]);
}

TEST(ChannelGeometry, OnlyRearGoalPreselectsReverseDirection)
{
  const double north = M_PI / 2.0;
  const double threshold = 110.0 * M_PI / 180.0;
  EXPECT_FALSE(cnm::isReverseTask(north, 0, 0, 10, 10, threshold, 5.0));
  EXPECT_FALSE(cnm::isReverseTask(north, 0, 0, -10, 10, threshold, 5.0));
  EXPECT_TRUE(cnm::isReverseTask(north, 0, 0, 0, -20, threshold, 5.0));
}

TEST(ChannelGeometry, WindingLaneRolePropagatesLocallyBeyondInitialNinetyDegrees)
{
  auto square = [](double x, double y) {
      return std::vector<cnm::Point2D>{{x, y}, {x + 8, y}, {x + 8, y + 8},
        {x, y + 8}, {x, y}};
    };
  // S-57 directions 90 -> 35 -> 350 accumulate a 100 degree bend.  The last
  // segment is opposite to the initial course if classified in one step, but
  // it is a valid continuation through two locally smooth bends.
  std::vector<cnm::Lane> lanes{
    {0, 90.0, square(0, 0)}, {1, 35.0, square(15, 0)}, {2, 350.0, square(30, 0)},
    {3, 270.0, square(0, 10)}, {4, 215.0, square(15, 10)}, {5, 170.0, square(30, 10)}};
  const auto main = cnm::continuousMainLaneMask(
    lanes, cnm::s57OrientationToYaw(90.0), 4.0, 4.0, 20.0, 80.0 * M_PI / 180.0);
  ASSERT_EQ(main.size(), lanes.size());
  EXPECT_TRUE(main[0]); EXPECT_TRUE(main[1]); EXPECT_TRUE(main[2]);
  EXPECT_FALSE(main[3]); EXPECT_FALSE(main[4]); EXPECT_FALSE(main[5]);
}
