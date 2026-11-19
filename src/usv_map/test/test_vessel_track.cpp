/**
 * @file test_vessel_track.cpp
 * @brief 验证融合状态解析、坐标转换及船舶轨迹维护。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <tf2/LinearMath/Matrix3x3.hpp>

#include "usv_map/vessel_track.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kLatitude = 29.506912001 * kPi / 180.0;
constexpr double kLongitude = 106.716974974 * kPi / 180.0;
constexpr double kHeight = 247.0;

usv_map::VesselTrack make_track(double min_distance = 0.5, std::size_t max_points = 10000)
{
  usv_map::VesselTrack track(min_distance, max_points);
  track.set_origin(kLatitude, kLongitude, kHeight);
  return track;
}

std_msgs::msg::Float64MultiArray state(
  double east, double time, double north = 0.0, double up = 0.0)
{
  std_msgs::msg::Float64MultiArray message;
  message.data.resize(19, 0.0);
  message.data[2] = -kPi * 0.5;  // RFU 前向轴指向东。
  message.data[6] = kLatitude + north / kEarthRadiusM;
  message.data[7] = kLongitude + east / (kEarthRadiusM * std::cos(kLatitude));
  message.data[8] = kHeight + up;
  message.data[17] = 4.0;
  message.data[18] = time;
  return message;
}

builtin_interfaces::msg::Time stamp(int seconds)
{
  builtin_interfaces::msg::Time result;
  result.sec = seconds;
  return result;
}

tf2::Matrix3x3 rotation(const usv_map::VesselTrack & track)
{
  const auto & q = track.pose().pose.orientation;
  return tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w));
}

}  // namespace

TEST(VesselTrack, ConvertsRadianLlaToEnuWithReceptionTime)
{
  auto track = make_track();
  const auto message = state(12.0, 0.5, -8.0, 3.0);
  ASSERT_TRUE(track.update(message, stamp(100)));
  EXPECT_EQ(track.pose().header.frame_id, "map");
  EXPECT_EQ(track.pose().header.stamp.sec, 100);
  EXPECT_NEAR(track.pose().pose.position.x, 12.0, 1e-8);
  EXPECT_NEAR(track.pose().pose.position.y, -8.0, 1e-8);
  EXPECT_NEAR(track.pose().pose.position.z, 3.0, 1e-8);
  ASSERT_EQ(track.path().poses.size(), 1u);
  EXPECT_EQ(track.path().poses.front(), track.pose());
  EXPECT_EQ(track.path().header, track.pose().header);

  // INS 航向为 -pi/2 时，RFU 的 +y 指向东，+x 指向南。
  const auto axes = rotation(track);
  EXPECT_NEAR((axes * tf2::Vector3(0, 1, 0)).x(), 1.0, 1e-12);
  EXPECT_NEAR((axes * tf2::Vector3(1, 0, 0)).y(), -1.0, 1e-12);
}

TEST(VesselTrack, MatchesFusionAttitudeRotationOrder)
{
  auto track = make_track();
  auto message = state(0.0, 1.0);
  message.data[0] = kPi / 6.0;
  message.data[1] = kPi / 4.0;
  message.data[2] = kPi / 2.0;
  ASSERT_TRUE(track.update(message, stamp(1)));
  const auto axes = rotation(track);
  const auto forward = axes * tf2::Vector3(0, 1, 0);
  EXPECT_NEAR(forward.x(), -std::sqrt(0.75), 1e-12);
  EXPECT_NEAR(forward.y(), 0.0, 1e-12);
  EXPECT_NEAR(forward.z(), 0.5, 1e-12);
  const auto up = axes * tf2::Vector3(0, 0, 1);
  EXPECT_NEAR(up.x(), std::sqrt(0.125), 1e-12);
  EXPECT_NEAR(up.y(), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(up.z(), std::sqrt(0.375), 1e-12);
  const auto & q = track.pose().pose.orientation;
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
}

TEST(VesselTrack, HeadingMatchesNorthUpMapAndRfuBodyAxes)
{
  struct HeadingCase
  {
    double degrees;
    double east;
    double north;
  };
  const HeadingCase headings[] = {
    {0.0, 0.0, 1.0},        // 北向对应地图上方。
    {-90.0, 1.0, 0.0},      // 东向对应地图右方。
    {90.0, -1.0, 0.0},      // 西向对应地图左方。
    {-180.0, 0.0, -1.0},    // 南向覆盖航向区间的两端。
    {180.0, 0.0, -1.0},
    {-45.0, std::sqrt(0.5), std::sqrt(0.5)},
    {45.0, -std::sqrt(0.5), std::sqrt(0.5)},
  };
  auto track = make_track();
  for (const auto & heading : headings) {
    SCOPED_TRACE(heading.degrees);
    auto message = state(0.0, 1.0);
    message.data[2] = heading.degrees * kPi / 180.0;
    ASSERT_TRUE(track.update(message, stamp(1)));
    const auto axes = rotation(track);
    const auto forward = axes * tf2::Vector3(0.0, 1.0, 0.0);
    EXPECT_NEAR(forward.x(), heading.east, 1e-12);
    EXPECT_NEAR(forward.y(), heading.north, 1e-12);
    EXPECT_NEAR(forward.z(), 0.0, 1e-12);

    const auto right = axes * tf2::Vector3(1.0, 0.0, 0.0);
    EXPECT_NEAR(right.x(), heading.north, 1e-12);
    EXPECT_NEAR(right.y(), -heading.east, 1e-12);
    const auto up = axes * tf2::Vector3(0.0, 0.0, 1.0);
    EXPECT_NEAR(up.z(), 1.0, 1e-12);
  }
}

TEST(VesselTrack, SamplesDistanceWhilePoseKeepsUpdating)
{
  auto track = make_track();
  ASSERT_TRUE(track.update(state(0.0, 0.0), stamp(1)));
  auto message = state(0.3, 1.0);
  message.data[2] = 0.5 - kPi * 0.5;
  ASSERT_TRUE(track.update(message, stamp(2)));
  EXPECT_FALSE(track.path_changed());
  EXPECT_EQ(track.path().poses.size(), 1u);
  EXPECT_NEAR(track.pose().pose.position.x, 0.3, 1e-8);
  EXPECT_EQ(track.pose().header.stamp.sec, 2);
  EXPECT_NEAR((rotation(track) * tf2::Vector3(0, 1, 0)).y(), std::sin(0.5), 1e-12);
  ASSERT_TRUE(track.update(state(0.51, 2.0), stamp(3)));
  EXPECT_TRUE(track.path_changed());
  EXPECT_EQ(track.path().poses.size(), 2u);
  ASSERT_TRUE(track.update(state(0.51, 3.0), stamp(4)));
  EXPECT_FALSE(track.path_changed());
  EXPECT_EQ(track.path().poses.size(), 2u);
}

TEST(VesselTrack, KeepsOnlyRecentPoints)
{
  auto track = make_track(0.5, 2);
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(track.update(state(i, i), stamp(i + 1)));
  }
  ASSERT_EQ(track.path().poses.size(), 2u);
  EXPECT_NEAR(track.path().poses.front().pose.position.x, 1.0, 1e-8);
  EXPECT_NEAR(track.path().poses.back().pose.position.x, 2.0, 1e-8);
}

TEST(VesselTrack, UnalignedHeartbeatUpdatesAttitudeWithoutProjectingZeroLla)
{
  auto track = make_track();
  auto message = state(2.0, 0.0);
  message.data[17] = 0.0;
  message.data[6] = message.data[7] = message.data[8] = 0.0;
  ASSERT_TRUE(track.update(message, stamp(1)));
  EXPECT_DOUBLE_EQ(track.pose().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(track.pose().pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(track.pose().pose.position.z, 0.0);
  EXPECT_TRUE(track.path().poses.empty());
  ASSERT_TRUE(track.update(state(3.0, 1.0), stamp(2)));
  ASSERT_EQ(track.path().poses.size(), 1u);
  for (const double code : {0.0, 1.0, 2.0, 3.0, 6.0, 7.0}) {
    ASSERT_TRUE(track.update(state(3.0, 1.0), stamp(2)));
    message.data[17] = code;
    ASSERT_TRUE(track.update(message, stamp(3)));
    EXPECT_TRUE(track.path_changed());
    EXPECT_TRUE(track.path().poses.empty());
    EXPECT_NEAR(track.pose().pose.position.x, 3.0, 1e-8);
  }
  ASSERT_TRUE(track.update(state(4.0, 3.0), stamp(4)));
  ASSERT_EQ(track.path().poses.size(), 1u);
  EXPECT_NEAR(track.path().poses.front().pose.position.x, 4.0, 1e-8);
}

TEST(VesselTrack, ContinuesInsTrackDuringGnssOutage)
{
  auto track = make_track();
  ASSERT_TRUE(track.update(state(0.0, 1.0), stamp(1)));
  auto message = state(1.0, 2.0);
  message.data[17] = 5.0;
  ASSERT_TRUE(track.update(message, stamp(2)));
  EXPECT_EQ(track.path().poses.size(), 2u);
}

TEST(VesselTrack, SourceRestartBeginsNewTrack)
{
  auto track = make_track();
  ASSERT_TRUE(track.update(state(10.0, 100.0), stamp(1)));
  ASSERT_TRUE(track.update(state(11.0, 101.0), stamp(2)));
  ASSERT_TRUE(track.update(state(0.0, 0.0), stamp(3)));
  EXPECT_TRUE(track.path_changed());
  ASSERT_EQ(track.path().poses.size(), 1u);
  EXPECT_DOUBLE_EQ(track.path().poses.front().pose.position.x, 0.0);
}

TEST(VesselTrack, InvalidStateLeavesPoseAndHistoryIntact)
{
  auto track = make_track();
  ASSERT_TRUE(track.update(state(10.0, 1.0), stamp(1)));
  const auto saved_pose = track.pose();
  const auto saved_path = track.path();
  auto short_message = state(0.0, 0.0);
  short_message.data.resize(18);
  EXPECT_FALSE(track.update(short_message, stamp(2)));
  for (std::size_t index = 0; index < 19; ++index) {
    for (const double invalid : {
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
      auto message = state(0.0, 0.0);
      message.data[index] = invalid;
      EXPECT_FALSE(track.update(message, stamp(2)));
      EXPECT_FALSE(track.path_changed());
      EXPECT_EQ(track.pose(), saved_pose);
      EXPECT_EQ(track.path(), saved_path);
    }
  }
  ASSERT_TRUE(track.update(state(11.0, 2.0), stamp(3)));
  EXPECT_EQ(track.path().poses.size(), 2u);
}

TEST(VesselTrack, RejectsDegreesAndInvalidFusionCodes)
{
  auto track = make_track();
  ASSERT_TRUE(track.update(state(1.0, 1.0), stamp(1)));
  const auto previous = track.path();
  auto message = state(2.0, 2.0);
  message.data[6] = 29.5;
  EXPECT_FALSE(track.update(message, stamp(2)));
  message = state(2.0, 2.0);
  message.data[7] = 106.7;
  EXPECT_FALSE(track.update(message, stamp(2)));
  for (const double code : {-1.0, 4.5, 8.0}) {
    message = state(2.0, 2.0);
    message.data[17] = code;
    EXPECT_FALSE(track.update(message, stamp(2)));
  }
  EXPECT_EQ(track.path(), previous);
}

TEST(VesselTrack, RequiresValidMapOrigin)
{
  usv_map::VesselTrack track;
  EXPECT_FALSE(track.update(state(0.0, 1.0), stamp(1)));
  EXPECT_THROW(track.set_origin(29.5, kLongitude), std::invalid_argument);
  EXPECT_THROW(track.set_origin(kLatitude, 106.7), std::invalid_argument);
  EXPECT_THROW(track.set_origin(kLatitude, kLongitude,
    std::numeric_limits<double>::infinity()), std::invalid_argument);
  track.set_origin(kLatitude, kLongitude, kHeight);
  ASSERT_TRUE(track.update(state(0.0, 1.0), stamp(1)));
  EXPECT_DOUBLE_EQ(track.pose().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(track.pose().pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(track.pose().pose.position.z, 0.0);
}

TEST(VesselTrack, RepeatedOriginPreservesHistoryAndSourceTime)
{
  auto track = make_track();
  ASSERT_TRUE(track.update(state(0.0, 10.0), stamp(1)));
  ASSERT_TRUE(track.update(state(1.0, 11.0), stamp(2)));
  const auto previous = track.path();
  EXPECT_FALSE(track.set_origin(kLatitude, kLongitude, kHeight));
  EXPECT_EQ(track.path(), previous);
  ASSERT_TRUE(track.update(state(2.0, 12.0), stamp(3)));
  EXPECT_EQ(track.path().poses.size(), 3u);
  // 重复设置相同原点应保留对状态源重启的检测能力。
  EXPECT_FALSE(track.set_origin(kLatitude, kLongitude, kHeight));
  ASSERT_TRUE(track.update(state(3.0, 0.0), stamp(4)));
  EXPECT_EQ(track.path().poses.size(), 1u);
}

TEST(VesselTrack, ChangedOriginResetsAndReprojectsHistory)
{
  auto track = make_track();
  const auto message = state(20.0, 10.0, 30.0, 4.0);
  ASSERT_TRUE(track.update(message, stamp(1)));
  EXPECT_TRUE(track.set_origin(kLatitude, kLongitude, kHeight + 2.0));
  EXPECT_TRUE(track.path().poses.empty());
  ASSERT_TRUE(track.update(message, stamp(2)));
  EXPECT_NEAR(track.pose().pose.position.z, 2.0, 1e-8);

  EXPECT_TRUE(track.set_origin(
    kLatitude + 10.0 / kEarthRadiusM, kLongitude, kHeight));
  EXPECT_TRUE(track.path().poses.empty());
  ASSERT_TRUE(track.update(message, stamp(3)));
  EXPECT_NEAR(track.pose().pose.position.y, 20.0, 1e-8);
  EXPECT_NEAR(track.pose().pose.position.x,
    20.0 * std::cos(kLatitude + 10.0 / kEarthRadiusM) / std::cos(kLatitude), 1e-8);
  EXPECT_EQ(track.path().poses.size(), 1u);
}

TEST(VesselTrack, UsesMeasuredPositionRatherThanGroundTruth)
{
  auto track = make_track();
  auto message = state(5.0, 1.0, 8.0, -0.5);
  message.data[13] = kLatitude;
  message.data[14] = kLongitude;
  message.data[15] = kHeight;
  ASSERT_TRUE(track.update(message, stamp(1)));
  EXPECT_NEAR(track.pose().pose.position.x, 5.0, 1e-8);
  EXPECT_NEAR(track.pose().pose.position.y, 8.0, 1e-8);
  EXPECT_NEAR(track.pose().pose.position.z, -0.5, 1e-8);
}

TEST(VesselTrack, PredictsPositionWithEnuVelocityAndCorrectsMeasurementResidual)
{
  usv_map::VesselTrack track(0.5, 10000, 0.2);
  track.set_origin(kLatitude, kLongitude, kHeight);
  auto first = state(0.0, 0.0);
  first.data[3] = 2.0;
  ASSERT_TRUE(track.update(first, stamp(1)));

  auto second = state(0.7, 0.1);
  second.data[3] = 2.0;
  ASSERT_TRUE(track.update(second, stamp(2)));
  EXPECT_NEAR(track.pose().pose.position.x, 0.3, 2e-5);

  auto third = state(-0.1, 0.2);
  third.data[3] = 2.0;
  ASSERT_TRUE(track.update(third, stamp(3)));
  EXPECT_NEAR(track.pose().pose.position.x, 0.38, 2e-5);
}

TEST(VesselTrack, RejectsInvalidHistoryConfiguration)
{
  EXPECT_THROW(usv_map::VesselTrack(0.0, 10), std::invalid_argument);
  EXPECT_THROW(usv_map::VesselTrack(-1.0, 10), std::invalid_argument);
  EXPECT_THROW(usv_map::VesselTrack(std::numeric_limits<double>::quiet_NaN(), 10),
    std::invalid_argument);
  EXPECT_THROW(usv_map::VesselTrack(0.5, 1), std::invalid_argument);
  EXPECT_THROW(usv_map::VesselTrack(0.5, 10, 0.0), std::invalid_argument);
  EXPECT_THROW(usv_map::VesselTrack(0.5, 10, 1.1), std::invalid_argument);
}
