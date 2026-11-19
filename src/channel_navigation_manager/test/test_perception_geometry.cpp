/**
 * @file test_perception_geometry.cpp
 * @brief 验证 RFU 水平观测转换、障碍物轮廓及圆形栅格覆盖。
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "channel_navigation_manager/perception_geometry.hpp"

namespace cnm = channel_navigation_manager;

TEST(PerceptionGeometry, CentroidIgnoresRepeatedClosingPoint)
{
  const cnm::Polygon polygon{{8, 18}, {12, 18}, {12, 22}, {8, 22}, {8, 18}};
  const auto center = cnm::polygonCentroid(polygon);
  EXPECT_NEAR(center.x, 10.0, 1.0e-10);
  EXPECT_NEAR(center.y, 20.0, 1.0e-10);
  EXPECT_THROW(cnm::polygonCentroid({{1, 1}, {2, 2}, {3, 3}}), std::invalid_argument);
  EXPECT_THROW(cnm::polygonCentroid({}), std::invalid_argument);
}

TEST(PerceptionGeometry, NorthFacingRfuReportsBowRightLeftAndStern)
{
  const tf2::Transform map_from_sensor(tf2::Quaternion::getIdentity(), {10, 20, 3});
  struct Case {double x, y, bearing;};
  const Case cases[] = {{10, 30, 0}, {20, 20, M_PI / 2},
    {0, 20, -M_PI / 2}, {10, 10, M_PI}};
  for (const auto & sample : cases) {
    const cnm::Polygon polygon{{sample.x - 1, sample.y - 1}, {sample.x + 1, sample.y - 1},
      {sample.x + 1, sample.y + 1}, {sample.x - 1, sample.y + 1}};
    const auto detection = cnm::detectObstacle("REEF", polygon, map_from_sensor);
    EXPECT_NEAR(detection.range_m, 10.0, 1.0e-9);
    EXPECT_NEAR(detection.bearing_rad, sample.bearing, 1.0e-9);
    EXPECT_NEAR(detection.radius_m, std::sqrt(2.0), 1.0e-9);
    const auto circle = cnm::detectionCircle(detection, map_from_sensor, 2.0);
    EXPECT_NEAR(circle.center.x, sample.x, 1.0e-9);
    EXPECT_NEAR(circle.center.y, sample.y, 1.0e-9);
    EXPECT_NEAR(circle.radius_m, std::sqrt(2.0) + 2.0, 1.0e-9);
  }
}

TEST(PerceptionGeometry, RotatingShipPreservesHorizontalCenterAndCircleRadius)
{
  const cnm::Polygon polygon{{19, 29}, {21, 29}, {21, 31}, {19, 31}};
  for (double angle : {0.0, -M_PI / 2, M_PI / 2, M_PI}) {
    tf2::Quaternion rotation;
    rotation.setRPY(0.15, -0.1, angle);  // 水平测量不因横摇/纵摇变成椭圆。
    const tf2::Transform pose(rotation, {10, 20, 3});
    const auto detection = cnm::detectObstacle("REEF", polygon, pose);
    const auto circle = cnm::detectionCircle(detection, pose, 2.0);
    EXPECT_NEAR(circle.center.x, 20.0, 1.0e-9);
    EXPECT_NEAR(circle.center.y, 30.0, 1.0e-9);
    EXPECT_NEAR(circle.radius_m, std::sqrt(2.0) + 2.0, 1.0e-9);
  }
}

TEST(PerceptionGeometry, BowYawUsesPositiveYForRfu)
{
  for (double yaw : {0.0, -M_PI / 2, M_PI / 2, M_PI}) {
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, yaw);
    EXPECT_NEAR(cnm::normalizeAngle(cnm::bowYaw(rotation) - yaw - M_PI / 2), 0, 1.0e-10);
  }
}

TEST(PerceptionGeometry, ReportedSizeIsRadiusWithAdditionalInflation)
{
  usv_interfaces::msg::ObstacleDetection detection;
  detection.range_m = 10;
  detection.bearing_rad = M_PI / 2;
  detection.radius_m = 5;
  const auto circle = cnm::detectionCircle(detection, tf2::Transform::getIdentity(), 2.0);
  EXPECT_NEAR(circle.center.x, 10, 1.0e-10);
  EXPECT_NEAR(circle.center.y, 0, 1.0e-10);
  EXPECT_DOUBLE_EQ(circle.radius_m, 7.0);
  EXPECT_DOUBLE_EQ(cnm::detectionCircle(detection, tf2::Transform::getIdentity(), 0.0).radius_m, 5.0);
}

TEST(PerceptionGeometry, RejectsInvalidSensorData)
{
  usv_interfaces::msg::ObstacleDetection detection;
  const auto pose = tf2::Transform::getIdentity();
  EXPECT_THROW(cnm::detectionCircle(detection, pose, 2.0), std::invalid_argument);
  detection.radius_m = 1;
  detection.range_m = -1;
  EXPECT_THROW(cnm::detectionCircle(detection, pose, 2.0), std::invalid_argument);
  detection.range_m = 1;
  detection.bearing_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cnm::detectionCircle(detection, pose, 2.0), std::invalid_argument);
  detection.bearing_rad = 0;
  EXPECT_THROW(cnm::detectionCircle(detection, pose, -1.0), std::invalid_argument);
  detection.radius_m = std::numeric_limits<double>::infinity();
  EXPECT_THROW(cnm::detectionCircle(detection, pose, 2.0), std::invalid_argument);
}

TEST(PerceptionGeometry, CircleRasterUsesGridCentersAndClipsToMap)
{
  // 非零原点、2 m 栅格；半径 2 m 包含四邻接中心，不包含对角中心。
  const std::vector<std::size_t> expected{7, 11, 12, 13, 17};
  EXPECT_EQ(cnm::circleCells(5, 5, 2.0, -10.0, 20.0, {{-5, 25}, 2.0}), expected);
  EXPECT_EQ(cnm::circleCells(2, 2, 2.0, 0.0, 0.0, {{1, 1}, 2.0}),
    (std::vector<std::size_t>{0, 1, 2}));
  EXPECT_TRUE(cnm::circleCells(2, 2, 2.0, 0.0, 0.0, {{100, 100}, 2.0}).empty());
  EXPECT_TRUE(cnm::circleCells(0, 2, 2.0, 0.0, 0.0, {{1, 1}, 2.0}).empty());
}
