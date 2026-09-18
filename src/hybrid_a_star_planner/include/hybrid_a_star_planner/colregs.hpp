#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace hybrid_a_star_planner
{

constexpr double kPi = 3.14159265358979323846;

enum class EncounterType
{
  NONE,
  HEAD_ON,
  CROSSING_STARBOARD,
  CROSSING_PORT,
  OVERTAKING,
  BEING_OVERTAKEN
};

enum class TurnPreference
{
  NONE,
  RIGHT,
  LEFT,
  KEEP_COURSE
};

struct VesselState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.0};
};

struct EncounterAssessment
{
  EncounterType type{EncounterType::NONE};
  TurnPreference preference{TurnPreference::NONE};
  bool active{false};
  double relative_bearing{0.0};
  double tcpa{0.0};
  double dcpa{0.0};
  double urgency{0.0};
};

// [功能与联系] 归一化角度，消除跨±pi的跳变；方向比较、运动积分和COLREG约束共同使用。
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// [功能与联系] 将会遇枚举转换为人可读名称，供日志和诊断。
inline std::string toString(EncounterType type)
{
  switch (type) {
    case EncounterType::HEAD_ON:
      return "HEAD_ON";
    case EncounterType::CROSSING_STARBOARD:
      return "CROSSING_STARBOARD";
    case EncounterType::CROSSING_PORT:
      return "CROSSING_PORT_STAND_ON";
    case EncounterType::OVERTAKING:
      return "OVERTAKING";
    case EncounterType::BEING_OVERTAKEN:
      return "BEING_OVERTAKEN";
    default:
      return "NONE";
  }
}

// 实际中心间距停车，独立于CPA和目标航向；等于阈值不触发“小于”条件。
// [功能与联系] 检测两船当前中心距离是否小于紧急停车半径；DWA独立于未来CPA快速调用。
inline bool insideStopDistance(double own_x, double own_y, double target_x,
  double target_y, double stop_distance)
{
  return stop_distance > 0.0 &&
    std::hypot(target_x - own_x, target_y - own_y) < stop_distance;
}

inline EncounterAssessment assessEncounter(
  double own_x, double own_y, double reference_heading, double own_speed,
  const VesselState & target, double time_horizon, double risk_distance,
  double head_on_bearing, double head_on_course_tolerance,
  double crossing_bearing_limit, double overtaking_margin)
{
  EncounterAssessment result;
  const double rx = target.x - own_x;
  const double ry = target.y - own_y;
  const double target_speed = std::hypot(target.vx, target.vy);
  const double own_vx = own_speed * std::cos(reference_heading);
  const double own_vy = own_speed * std::sin(reference_heading);
  const double rvx = target.vx - own_vx;
  const double rvy = target.vy - own_vy;
  const double relative_speed_sq = rvx * rvx + rvy * rvy;

  result.relative_bearing = normalizeAngle(
    std::atan2(ry, rx) - reference_heading);

  if (relative_speed_sq < 1.0e-6) {
    result.tcpa = 0.0;
    result.dcpa = std::hypot(rx, ry);
    return result;
  }

  result.tcpa = -(rx * rvx + ry * rvy) / relative_speed_sq;
  const double cpa_x = rx + rvx * result.tcpa;
  const double cpa_y = ry + rvy * result.tcpa;
  result.dcpa = std::hypot(cpa_x, cpa_y);

  if (result.tcpa < 0.0 || result.tcpa > time_horizon ||
    result.dcpa > risk_distance)
  {
    return result;
  }

  const double abs_bearing = std::abs(result.relative_bearing);
  const double target_heading = target_speed > 0.05 ?
    std::atan2(target.vy, target.vx) : reference_heading;
  const double course_difference = std::abs(
    normalizeAngle(target_heading - reference_heading));

  if (abs_bearing <= head_on_bearing &&
    course_difference >= kPi - head_on_course_tolerance)
  {
    result.type = EncounterType::HEAD_ON;
    result.preference = TurnPreference::RIGHT;
  } else {
    const double target_bearing_to_own = normalizeAngle(
      std::atan2(own_y - target.y, own_x - target.x) - target_heading);
    const bool behind_target = std::abs(target_bearing_to_own) > 112.5 * kPi / 180.0;
    const bool target_ahead = abs_bearing < 67.5 * kPi / 180.0;

    if (target_speed > 0.05 && target_ahead && behind_target &&
      own_speed > target_speed + overtaking_margin)
    {
      result.type = EncounterType::OVERTAKING;
      result.preference = TurnPreference::NONE;
    } else if (abs_bearing > 112.5 * kPi / 180.0 &&
      target_speed > own_speed + overtaking_margin && course_difference < kPi / 2)
    {
      result.type = EncounterType::BEING_OVERTAKEN;
      result.preference = TurnPreference::KEEP_COURSE;
    } else if (abs_bearing <= crossing_bearing_limit) {
      if (result.relative_bearing <= 0.0) {
        result.type = EncounterType::CROSSING_STARBOARD;
        result.preference = TurnPreference::RIGHT;
      } else {
        result.type = EncounterType::CROSSING_PORT;
        result.preference = TurnPreference::KEEP_COURSE;
      }
    }
  }

  result.active = result.type != EncounterType::NONE;
  if (result.active) {
    const double time_urgency = 1.0 - std::clamp(result.tcpa / time_horizon, 0.0, 1.0);
    const double distance_urgency = 1.0 - std::clamp(result.dcpa / risk_distance, 0.0, 1.0);
    result.urgency = std::clamp(0.2 + 0.4 * time_urgency + 0.4 * distance_urgency, 0.0, 1.0);
  }
  return result;
}

// [功能与联系] 计算左右/保向偏好软惩罚；不代替动态硬碰撞判断。
inline double headingPreferenceCost(
  double node_heading, double reference_heading, TurnPreference preference,
  double urgency, double weight, double deadband)
{
  const double deviation = normalizeAngle(node_heading - reference_heading);
  switch (preference) {
    case TurnPreference::RIGHT:
      // ROS yaw is counter-clockwise positive, so a left turn is positive.
      return deviation > deadband ? urgency * weight * (deviation - deadband) : 0.0;
    case TurnPreference::LEFT:
      return deviation < -deadband ? urgency * weight * (-deviation - deadband) : 0.0;
    case TurnPreference::KEEP_COURSE:
      return urgency * weight * std::abs(deviation);
    default:
      return 0.0;
  }
}

}  // namespace hybrid_a_star_planner
