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

TEST(ChannelGeometry, GoalBehindSelectsTheOppositeRightHandLane)
{
  const auto [goal_course, distance] = cnm::bearingAndDistance(0.0, 0.0, 0.0, -20.0);
  ASSERT_NEAR(goal_course, -M_PI / 2.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(distance, 20.0);
  const cnm::Lane northbound{1, 0.0, {}};
  const cnm::Lane southbound{2, 180.0, {}};
  EXPECT_EQ(cnm::laneValue(goal_course, southbound), cnm::kMainLane);
  EXPECT_EQ(cnm::laneValue(goal_course, northbound), cnm::kOppositeLane);
  const std::vector<cnm::Lane> lanes{
    {1, 0.0, {{0, -10}, {4, -10}, {4, 10}, {0, 10}, {0, -10}}},
    {2, 180.0, {{-5, -10}, {-1, -10}, {-1, 10}, {-5, 10}, {-5, -10}}}};
  const auto main = cnm::continuousMainLaneMask(
    lanes, goal_course, 2.0, 0.0, 10.0, 80.0 * M_PI / 180.0);
  ASSERT_EQ(main.size(), 2U);
  EXPECT_FALSE(main[0]);
  EXPECT_TRUE(main[1]);
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

TEST(ChannelGeometry, UnequalLaneExtentsDoNotTurnLongitudinalOffsetIntoWrongRightSide)
{
  const std::vector<cnm::Lane> lanes{
    {1,0,{{0,-10},{4,-10},{4,30},{0,30},{0,-10}}},
    {2,180,{{-5,-10},{-1,-10},{-1,300},{-5,300},{-5,-10}}}};
  // 目标在西北，但两条分道质心y相差很大；不能因此误选西侧南向分道。
  const auto main=cnm::continuousMainLaneMask(lanes,2.4,2,0,10,80*M_PI/180);
  ASSERT_EQ(main.size(),2U);EXPECT_TRUE(main[0]);EXPECT_FALSE(main[1]);
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

TEST(ChannelGeometry, CompactPhysicalSeedCannotOverridePrescribedDirection) {
  const std::vector<cnm::Lane> lanes{
    {1,90,{{-100,-20},{100,-20},{100,0},{-100,0},{-100,-20}}},
    {2,270,{{-100,0},{100,0},{100,20},{-100,20},{-100,0}}}};
  for(double side:{-1.0,1.0}) for(double gx:{-10.0,10.0}) {
    const auto seed=cnm::compactMissionLaneSeed(lanes,0,10*side,gx,10*side,30);
    ASSERT_LT(seed,lanes.size());
    const double bearing=std::atan2(0.0,gx);
    const auto mask=cnm::continuousMainLaneMask(lanes,bearing,0,10*side,160,80*M_PI/180,seed);
    EXPECT_EQ(mask[0],gx>0);EXPECT_EQ(mask[1],gx<0);
  }
  EXPECT_GE(cnm::compactMissionLaneSeed(lanes,0,-10,-10,10,30),lanes.size());
  EXPECT_GE(cnm::compactMissionLaneSeed(lanes,0,-10,-50,-10,30),lanes.size());
  EXPECT_GE(cnm::compactMissionLaneSeed(lanes,0,-10,0,-10,0),lanes.size());
}

TEST(ChannelTopology, SameGoalAcrossStartingSidesKeepsTrafficStream) {
  const std::vector<cnm::Lane> lanes{
    {1,90,{{-100,-10},{100,-10},{100,0},{-100,0},{-100,-10}}},
    {2,270,{{-100,0},{100,0},{100,10},{-100,10},{-100,0}}}};
  for(double y:{-5.,5.}) {
    auto direction=cnm::missionRouteDirection(lanes,0,y,80,-5,M_PI,5);
    ASSERT_TRUE(direction.connected);EXPECT_NEAR(direction.yaw,0,1e-8);
    auto mask=cnm::continuousMainLaneMask(lanes,direction.yaw,0,y,5,80*M_PI/180);
    EXPECT_TRUE(mask[0]);EXPECT_FALSE(mask[1]);
  }
}
TEST(ChannelTopology, ConnectionPointBeatsLongNextPolygonCentroid) {
  const std::vector<cnm::Lane> original{
    {1,315,{{0,0},{-100,100},{-90,110},{10,10},{0,0}}},
    {2,135,{{0,0},{-10,-10},{-110,90},{-100,100},{0,0}}},
    {3,90,{{-100,100},{500,100},{500,110},{-100,110},{-100,100}}}};
  for(double scale:{1.,3.}) {
    auto lanes=original;for(auto &l:lanes)for(auto &p:l.points){p.x*=scale;p.y*=scale;}
    auto direction=cnm::missionRouteDirection(lanes,-20*scale,30*scale,490*scale,105*scale,0,5);
    ASSERT_TRUE(direction.connected);ASSERT_TRUE(direction.topology_progress);
    EXPECT_NEAR(direction.yaw,3*M_PI/4,1e-8);
    std::reverse(lanes.begin(),lanes.end());
    auto reordered=cnm::missionRouteDirection(lanes,-20*scale,30*scale,490*scale,105*scale,0,5);
    EXPECT_NEAR(reordered.yaw,direction.yaw,1e-8);
  }
}
TEST(ChannelTopology, NeverInventsReverseTrafficForAnUnmatchedOneWayLane) {
  std::vector<cnm::Lane> lanes{{1,90,{{0,0},{100,0},{100,10},{0,10},{0,0}}}};
  auto mask=cnm::continuousMainLaneMask(lanes,M_PI,80,5,5,80*M_PI/180);
  EXPECT_FALSE(mask[0]);
}
