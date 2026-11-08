#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "hybrid_a_star_planner/colregs.hpp"

namespace hybrid_a_star_planner::planning
{
constexpr std::size_t kNoGlobalRouteGoalIndex =
  std::numeric_limits<std::size_t>::max();
constexpr double kDefaultGoalHeadingTolerance = M_PI / 6.0;

inline std::size_t matchGlobalRouteGoalIndex(
  const nav_msgs::msg::Path * route,
  const geometry_msgs::msg::PoseStamped & goal,
  std::size_t previous_goal_index)
{
  if (!route || route->poses.empty() || goal.header.frame_id != route->header.frame_id) {
    return kNoGlobalRouteGoalIndex;
  }
  const std::size_t begin = previous_goal_index == kNoGlobalRouteGoalIndex ?
    0U : std::min(previous_goal_index + 1U, route->poses.size());
  for (std::size_t i = begin; i < route->poses.size(); ++i) {
    const auto & pose = route->poses[i].pose;
    if (std::hypot(pose.position.x - goal.pose.position.x,
        pose.position.y - goal.pose.position.y) <= 0.25)
    {
      return i;
    }
  }
  return kNoGlobalRouteGoalIndex;
}

// Keep a usable local path during an ordinary forward waypoint handoff. A
// cleared path makes the tracker command a stop while the next leg is searched.
inline bool isForwardRouteWaypointAdvance(
  const bool has_valid_path, const std::size_t previous_index,
  const std::size_t next_index)
{
  return has_valid_path && previous_index != kNoGlobalRouteGoalIndex &&
    next_index != kNoGlobalRouteGoalIndex && next_index > previous_index;
}

// 迟到路径只裁掉已航行前缀，不把当前船位硬拼到旧曲线上。接续点位置/艏向
// 必须吻合实测状态，否则拒绝该结果并从最新状态重搜；不进行耗时外推补偿。
inline bool trimPathToMeasuredState(std::vector<geometry_msgs::msg::Pose> & path,
  double x, double y, double yaw, double distance_limit, double heading_limit)
{
  if (path.size() < 2) {return false;}
  std::size_t nearest = 0;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].position.x - x, path[i].position.y - y);
    if (d < distance) {distance = d; nearest = i;}
  }
  if (distance > distance_limit || nearest + 1 >= path.size() ||
    std::abs(hybrid_a_star_planner::normalizeAngle(tf2::getYaw(path[nearest].orientation) - yaw)) >
      heading_limit) {return false;}
  path.erase(path.begin(), path.begin() + static_cast<std::ptrdiff_t>(nearest));
  return true;
}

// 已经满足终点位置与艏向却继续绕很长一段，才是可避免的终点绕圈。
// 近距离反向目标开始时虽靠近位置、艏向却错误，不能把必要转向误删。
inline bool pathHasAvoidableGoalLoop(const std::vector<geometry_msgs::msg::Pose> &path,
  double radius,double minimum_remaining_arc)
{
  if(path.size()<4) {return false;}
  std::vector<double> suffix(path.size(),0.0);
  for(std::size_t i=path.size()-1;i>0;--i) {
    suffix[i-1]=suffix[i]+std::hypot(path[i].position.x-path[i-1].position.x,
      path[i].position.y-path[i-1].position.y);
  }
  const auto &goal=path.back();
  const double goal_yaw=tf2::getYaw(goal.orientation);
  const double reached_radius=std::min(1.0,std::max(0.1,radius));
  for(std::size_t i=0;i+2<path.size();++i) {
    if(std::hypot(path[i].position.x-goal.position.x,path[i].position.y-goal.position.y)<reached_radius &&
      std::abs(hybrid_a_star_planner::normalizeAngle(tf2::getYaw(path[i].orientation)-goal_yaw))<0.22 &&
      suffix[i]>minimum_remaining_arc) {return true;}
  }
  return false;
}

}  // namespace hybrid_a_star_planner::planning
