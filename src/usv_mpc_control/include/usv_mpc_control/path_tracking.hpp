#pragma once

#include <algorithm>
#include <cmath>

namespace usv_mpc_control
{

// Low-speed maneuvers must not preview 8m into a different steering phase.
inline double speedAdaptivePreview(double speed,double seconds=2.0) {
  return std::clamp(std::max(0.,speed)*std::max(.1,seconds),1.0,8.0);
}

// Bound the low-speed normalization so a tight maneuver is not rewarded for
// parking instead of making progress. It remains a soft cost, never safety.
inline double slowProgressScale(double speed) {
  const double ratio=std::clamp(1./std::max(.5,speed),1.,2.);
  return ratio*ratio;
}

// 一致的终点速度/进度参考；不能一边减速、一边惩罚没有按巡航速度前进。
inline double terminalReferenceSpeed(double remaining, double tolerance,
  double deceleration, double max_speed)
{
  return std::min(max_speed, std::sqrt(2.0 * std::max(0.01, deceleration) *
    std::max(0.0, remaining - tolerance)));
}

inline double terminalReferenceTravel(double remaining, double tolerance,
  double deceleration, double max_speed, double time)
{
  const double a = std::max(0.01, deceleration);
  const double speed = terminalReferenceSpeed(remaining, tolerance, a, max_speed);
  const double cruise_time = std::max(0.0,
    (remaining - tolerance - speed * speed / (2.0 * a)) / std::max(0.01, speed));
  const double cruise = std::min(std::max(0.0, time), cruise_time);
  const double braking = std::min(std::max(0.0, time - cruise_time), speed / a);
  return std::min(std::max(0.0, remaining - tolerance),
    speed * cruise + speed * braking - 0.5 * a * braking * braking);
}

// 低速时atan2(v,u)不稳定，用连续权重消退侧滑修正，不在0.25m/s处跳变。
inline double regularizedSideslip(double u, double v, double fade_speed)
{
  const double speed = std::hypot(u, v);
  const double fade = std::max(0.01, fade_speed);
  return std::atan2(v, std::max(0.05, u)) * speed * speed /
    (speed * speed + fade * fade);
}

inline double normalizeCourseAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double signedCrossTrackError(
  const double path_x, const double path_y, const double path_yaw,
  const double vessel_x, const double vessel_y)
{
  return -std::sin(path_yaw) * (vessel_x - path_x) +
    std::cos(path_yaw) * (vessel_y - path_y);
}

inline double correctedTrackCourse(
  const double path_yaw, const double lateral_error, const double recovery_distance)
{
  const double lookahead = std::max(0.25, recovery_distance);
  return normalizeCourseAngle(path_yaw - std::atan2(lateral_error, lookahead));
}

inline bool isTerminalMissionPath(
  const double active_goal_x, const double active_goal_y,
  const double mission_goal_x, const double mission_goal_y,
  const double path_end_x, const double path_end_y,
  const double tolerance, const double path_end_tolerance = -1.0)
{
  const double allowed = std::max(0.0, tolerance);
  return std::hypot(active_goal_x - mission_goal_x, active_goal_y - mission_goal_y) <= allowed &&
    std::hypot(path_end_x - mission_goal_x, path_end_y - mission_goal_y) <=
      (path_end_tolerance < 0.0 ? allowed : path_end_tolerance);
}

}  // namespace usv_mpc_control
