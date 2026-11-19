/**
 * @file vessel_track.cpp
 * @brief 校验船舶状态并生成 RFU 姿态和 ENU 轨迹。
 */

#include "usv_map/vessel_track.hpp"

#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.hpp>

namespace usv_map
{
namespace
{
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

bool valid_coordinates(double latitude, double longitude)
{
  return std::isfinite(latitude) && std::isfinite(longitude) &&
         std::abs(latitude) <= kPi * 0.5 && std::abs(longitude) <= kPi;
}
}  // namespace

VesselTrack::VesselTrack(
  double min_distance_m, std::size_t max_points, double position_measurement_alpha)
: min_distance_m_(min_distance_m), max_points_(max_points),
  position_measurement_alpha_(position_measurement_alpha)
{
  if (!std::isfinite(min_distance_m_) || min_distance_m_ <= 0.0 || max_points_ < 2 ||
    !std::isfinite(position_measurement_alpha_) || position_measurement_alpha_ <= 0.0 ||
    position_measurement_alpha_ > 1.0)
  {
    throw std::invalid_argument(
      "path_min_distance_m must be positive; path_max_points must be >= 2; "
      "position_measurement_alpha must be in (0, 1]");
  }
  pose_.header.frame_id = "map";
  path_.header.frame_id = "map";
}

bool VesselTrack::set_origin(double latitude_rad, double longitude_rad, double height_m)
{
  if (!valid_coordinates(latitude_rad, longitude_rad) || !std::isfinite(height_m)) {
    throw std::invalid_argument("map origin must be valid [latitude rad, longitude rad, height m]");
  }
  const std::array<double, 3> origin{latitude_rad, longitude_rad, height_m};
  if (origin_ && *origin_ == origin) {
    return false;
  }
  origin_ = origin;
  path_.poses.clear();
  path_.header.stamp = builtin_interfaces::msg::Time{};
  pose_.pose.position = geometry_msgs::msg::Point{};
  last_state_time_.reset();
  last_position_filter_time_.reset();
  filtered_position_.reset();
  path_changed_ = true;
  return true;
}

bool VesselTrack::update(
  const std_msgs::msg::Float64MultiArray & state,
  const builtin_interfaces::msg::Time & receive_stamp)
{
  path_changed_ = false;
  const auto & data = state.data;
  if (!origin_ || data.size() != 19) {
    return false;
  }
  for (const double value : data) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  if (!valid_coordinates(data[6], data[7]) || data[17] < 0.0 || data[17] > 7.0 ||
    std::floor(data[17]) != data[17])
  {
    return false;
  }

  pose_.header.stamp = receive_stamp;

  // 姿态按 Rz(航向) * Rx(横摇) * Ry(纵摇) 组合；RFU 的 +y 轴沿船首方向。
  // 航向采用北零、西正约定；水平船首在 ENU 中的方向向量为
  // [-sin(航向), cos(航向), 0]。
  tf2::Quaternion heading, first_angle, second_angle;
  heading.setRPY(0.0, 0.0, data[2]);
  first_angle.setRPY(data[0], 0.0, 0.0);
  second_angle.setRPY(0.0, data[1], 0.0);
  tf2::Quaternion attitude = heading * first_angle * second_angle;
  attitude.normalize();
  pose_.pose.orientation.x = attitude.x();
  pose_.pose.orientation.y = attitude.y();
  pose_.pose.orientation.z = attitude.z();
  pose_.pose.orientation.w = attitude.w();

  // 融合状态 4 表示正常运行，5 表示 GNSS 中断但惯导仍在推算。
  // 状态未对准或仿真时间回退时清空历史轨迹，避免连接无效位置。
  const bool aligned = data[17] == 4.0 || data[17] == 5.0;
  const bool restarted = last_state_time_ && data[18] < *last_state_time_;
  last_state_time_ = data[18];
  if (!aligned || restarted) {
    path_changed_ = !path_.poses.empty();
    path_.poses.clear();
    path_.header.stamp = receive_stamp;
  }
  if (!aligned) {
    last_position_filter_time_.reset();
    filtered_position_.reset();
    // 未对准状态可能以零经纬度发送心跳，不用它更新位置。
    return true;
  }

  const auto & origin = *origin_;
  const double longitude_delta = std::remainder(data[7] - origin[1], 2.0 * kPi);
  // 将相对地理原点的经纬高差近似投影为 ENU：x 向东、y 向北、z 向上。
  pose_.pose.position.x = kEarthRadiusM * std::cos(origin[0]) * longitude_delta;
  pose_.pose.position.y = kEarthRadiusM * (data[6] - origin[0]);
  pose_.pose.position.z = data[8] - origin[2];

  const double state_time = data[18];
  const double dt = last_position_filter_time_ ? state_time - *last_position_filter_time_ : 0.0;
  const auto measured_position = std::array<double, 2>{
    pose_.pose.position.x, pose_.pose.position.y};
  if (!filtered_position_ || restarted || dt <= 0.0 || dt > 0.5) {
    filtered_position_ = measured_position;
  } else {
    const std::array<double, 2> predicted_position{
      (*filtered_position_)[0] + data[3] * dt,
      (*filtered_position_)[1] + data[4] * dt};
    (*filtered_position_)[0] = predicted_position[0] + position_measurement_alpha_ *
      (measured_position[0] - predicted_position[0]);
    (*filtered_position_)[1] = predicted_position[1] + position_measurement_alpha_ *
      (measured_position[1] - predicted_position[1]);
  }
  last_position_filter_time_ = state_time;
  pose_.pose.position.x = (*filtered_position_)[0];
  pose_.pose.position.y = (*filtered_position_)[1];

  if (!path_.poses.empty()) {
    const auto & previous = path_.poses.back().pose.position;
    const auto & current = pose_.pose.position;
    if (std::hypot(current.x - previous.x, current.y - previous.y, current.z - previous.z) <
      min_distance_m_)
    {
      return true;
    }
  }
  if (path_.poses.size() == max_points_) {
    path_.poses.erase(path_.poses.begin());
  }
  path_.poses.push_back(pose_);
  path_.header.stamp = receive_stamp;
  path_changed_ = true;
  return true;
}

}  // namespace usv_map
