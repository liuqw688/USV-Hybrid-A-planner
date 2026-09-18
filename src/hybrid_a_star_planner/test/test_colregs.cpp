#include <gtest/gtest.h>

#include "hybrid_a_star_planner/colregs.hpp"

using hybrid_a_star_planner::EncounterType;
using hybrid_a_star_planner::TurnPreference;
using hybrid_a_star_planner::VesselState;

namespace
{
constexpr double kDeg = hybrid_a_star_planner::kPi / 180.0;

auto assess(const VesselState & target, double own_speed = 2.0)
{
  return hybrid_a_star_planner::assessEncounter(
    0.0, 0.0, 0.0, own_speed, target, 120.0, 20.0,
    15.0 * kDeg, 30.0 * kDeg, 112.5 * kDeg, 0.2);
}
}  // namespace

TEST(Colregs, HeadOnRequiresStarboardTurn)
{
  const auto result = assess(VesselState{100.0, 0.0, -2.0, 0.0, 3.0});
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.type, EncounterType::HEAD_ON);
  EXPECT_EQ(result.preference, TurnPreference::RIGHT);
}

TEST(Colregs, StarboardCrossingIsGiveWay)
{
  const auto result = assess(VesselState{50.0, -50.0, 0.0, 2.0, 3.0});
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.type, EncounterType::CROSSING_STARBOARD);
  EXPECT_EQ(result.preference, TurnPreference::RIGHT);
}

TEST(Colregs, PortCrossingIsStandOn)
{
  const auto result = assess(VesselState{50.0, 50.0, 0.0, -2.0, 3.0});
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.type, EncounterType::CROSSING_PORT);
  EXPECT_EQ(result.preference, TurnPreference::KEEP_COURSE);
}

TEST(Colregs, OvertakingDoesNotMandateASide)
{
  const auto result = assess(VesselState{40.0, 0.0, 1.0, 0.0, 3.0}, 2.0);
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.type, EncounterType::OVERTAKING);
  EXPECT_EQ(result.preference, TurnPreference::NONE);
}

TEST(Colregs, RightTurnCostPenalizesOnlyWrongTurn)
{
  const double left_cost = hybrid_a_star_planner::headingPreferenceCost(
    20.0 * kDeg, 0.0, TurnPreference::RIGHT, 1.0, 10.0, 2.0 * kDeg);
  const double right_cost = hybrid_a_star_planner::headingPreferenceCost(
    -20.0 * kDeg, 0.0, TurnPreference::RIGHT, 1.0, 10.0, 2.0 * kDeg);
  EXPECT_GT(left_cost, 0.0);
  EXPECT_DOUBLE_EQ(right_cost, 0.0);
}
