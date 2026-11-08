#include <gtest/gtest.h>

#include "usv_mpc_control/path_tracking.hpp"

TEST(PathTracking, CrossTrackCorrectionPointsBackTowardPath)
{
  const double north_error = usv_mpc_control::signedCrossTrackError(0.0, 0.0, 0.0, 0.0, 5.0);
  EXPECT_GT(north_error, 0.0);
  EXPECT_LT(usv_mpc_control::correctedTrackCourse(0.0, north_error, 10.0), 0.0);
  EXPECT_GT(usv_mpc_control::correctedTrackCourse(0.0, -north_error, 10.0), 0.0);
}

TEST(PathTracking, CorrectionIsBoundedByRecoveryDistance)
{
  const double correction = usv_mpc_control::correctedTrackCourse(0.0, 5.0, 10.0);
  EXPECT_NEAR(correction, -std::atan2(5.0, 10.0), 1.0e-12);
  EXPECT_NEAR(usv_mpc_control::correctedTrackCourse(0.3, 0.0, 10.0), 0.3, 1.0e-12);
}

TEST(PathTracking, BrakesOnlyWhenActiveGoalAndLocalPathReachMissionEnd)
{
  EXPECT_FALSE(usv_mpc_control::isTerminalMissionPath(10, 0, 10, 0, 8, 0, 0.5));
  EXPECT_FALSE(usv_mpc_control::isTerminalMissionPath(8, 0, 10, 0, 10, 0, 0.5));
  EXPECT_TRUE(usv_mpc_control::isTerminalMissionPath(10, 0, 10, 0, 10.2, 0, 0.5));
}

TEST(PathTracking, TerminalPathAcceptsHybridOneMetreToleranceOnlyForFinalWaypoint)
{
  EXPECT_TRUE(usv_mpc_control::isTerminalMissionPath(10, 0, 10, 0, 9.2, 0, 0.5, 1.05));
  EXPECT_FALSE(usv_mpc_control::isTerminalMissionPath(8, 0, 10, 0, 9.2, 0, 0.5, 1.05));
  EXPECT_FALSE(usv_mpc_control::isTerminalMissionPath(10, 0, 10, 0, 8.8, 0, 0.5, 1.05));
}

TEST(PathTracking, TerminalProgressStopsInsteadOfDemandingCruiseSpeed)
{
  EXPECT_DOUBLE_EQ(usv_mpc_control::terminalReferenceSpeed(0.5, 0.5, 0.5, 4.0), 0.0);
  EXPECT_DOUBLE_EQ(usv_mpc_control::terminalReferenceTravel(0.5, 0.5, 0.5, 4.0, 5), 0.0);
  EXPECT_LE(usv_mpc_control::terminalReferenceTravel(2.5, 0.5, 0.5, 4.0, 5), 2.0);
  EXPECT_NEAR(usv_mpc_control::terminalReferenceTravel(100, 0.5, 0.5, 4.0, 5), 20.0, 1.0e-9);
}

TEST(PathTracking, LowSpeedSideslipCorrectionIsContinuousAndFadesToZero)
{
  EXPECT_DOUBLE_EQ(usv_mpc_control::regularizedSideslip(0, 0, 0.3), 0.0);
  EXPECT_LT(std::abs(usv_mpc_control::regularizedSideslip(0.001, 0.001, 0.3)), 0.001);
  EXPECT_NEAR(usv_mpc_control::regularizedSideslip(0.2499, 0.03, 0.3),
    usv_mpc_control::regularizedSideslip(0.2501, 0.03, 0.3), 0.0002);
}

TEST(PathTracking,PreviewShrinksForSlowTightManeuvers) {
  EXPECT_DOUBLE_EQ(usv_mpc_control::speedAdaptivePreview(4),8);
  EXPECT_DOUBLE_EQ(usv_mpc_control::speedAdaptivePreview(.6),1.2);
  EXPECT_DOUBLE_EQ(usv_mpc_control::speedAdaptivePreview(0),1);
}

TEST(PathTracking,LowSpeedProgressNormalizationIsBounded) {
  EXPECT_DOUBLE_EQ(usv_mpc_control::slowProgressScale(4),1);
  EXPECT_NEAR(usv_mpc_control::slowProgressScale(.6),1/.36,1e-12);
  EXPECT_DOUBLE_EQ(usv_mpc_control::slowProgressScale(0),4);
}
