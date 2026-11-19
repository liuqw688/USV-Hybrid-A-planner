#pragma once

#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace usv_mpc_control
{
// 世界坐标变换作用于地图原点/路径/船位，不能只改frame字符串；网格数据与
// 机体系u/v/r保持不变。当前规划和控制都是平面模型，不接受倾斜的坐标系。
inline void checkPlanarTransform(const geometry_msgs::msg::TransformStamped &tf)
{
  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation,q);
  double roll=0,pitch=0,yaw=0;
  tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
  if (std::abs(roll)>1.0e-3 || std::abs(pitch)>1.0e-3) {
    throw std::runtime_error("MPC requires planar world-frame transforms");
  }
}

inline geometry_msgs::msg::Pose transformWorldPose(const geometry_msgs::msg::Pose &pose,
  const geometry_msgs::msg::TransformStamped &tf)
{
  geometry_msgs::msg::PoseStamped source,out;
  source.pose=pose;
  tf2::doTransform(source,out,tf);
  return out.pose;
}

template<class Message>
Message transformWorldMessage(const Message &raw,const geometry_msgs::msg::TransformStamped &tf);

template<>
inline nav_msgs::msg::OccupancyGrid transformWorldMessage(
  const nav_msgs::msg::OccupancyGrid &raw,const geometry_msgs::msg::TransformStamped &tf)
{
  checkPlanarTransform(tf);
  auto out=raw;
  out.info.origin=transformWorldPose(raw.info.origin,tf);
  out.header.frame_id=tf.header.frame_id;
  return out;
}

template<>
inline nav_msgs::msg::Path transformWorldMessage(
  const nav_msgs::msg::Path &raw,const geometry_msgs::msg::TransformStamped &tf)
{
  checkPlanarTransform(tf);
  auto out=raw;
  out.header.frame_id=tf.header.frame_id;
  for(auto &p:out.poses) {
    if(!p.header.frame_id.empty() && p.header.frame_id!=raw.header.frame_id) {
      throw std::runtime_error("Mixed-frame Path rejected");
    }
    p.pose=transformWorldPose(p.pose,tf);
    p.header.frame_id=tf.header.frame_id;
  }
  return out;
}

template<>
inline geometry_msgs::msg::PoseStamped transformWorldMessage(
  const geometry_msgs::msg::PoseStamped &raw,const geometry_msgs::msg::TransformStamped &tf)
{
  checkPlanarTransform(tf);
  auto out=raw;
  out.pose=transformWorldPose(raw.pose,tf);
  out.header.frame_id=tf.header.frame_id;
  return out;
}

template<>
inline nav_msgs::msg::Odometry transformWorldMessage(
  const nav_msgs::msg::Odometry &raw,const geometry_msgs::msg::TransformStamped &tf)
{
  checkPlanarTransform(tf);
  auto out=raw;
  out.pose.pose=transformWorldPose(raw.pose.pose,tf);
  out.header.frame_id=tf.header.frame_id;
  return out;  // Twist仍是船体坐标，不能随世界frame旋转。
}
}  // namespace usv_mpc_control
