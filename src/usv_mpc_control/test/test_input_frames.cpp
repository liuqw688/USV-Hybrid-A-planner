#include <gtest/gtest.h>
#include "usv_mpc_control/input_frames.hpp"

TEST(InputFrames, CoordinatesAndBodyVelocity)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id="odom";tf.child_frame_id="map";
  tf.transform.translation.x=5;
  tf.transform.rotation.z=std::sqrt(0.5);tf.transform.rotation.w=std::sqrt(0.5);
  nav_msgs::msg::Odometry raw;
  raw.header.frame_id="map";raw.header.stamp.sec=42;
  raw.pose.pose.position.x=2;raw.pose.pose.orientation.w=1;
  raw.twist.twist.linear.x=3;raw.twist.twist.linear.y=0.4;
  auto out=usv_mpc_control::transformWorldMessage(raw,tf);
  EXPECT_NEAR(out.pose.pose.position.x,5,1e-9);
  EXPECT_NEAR(out.pose.pose.position.y,2,1e-9);
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.x,3);
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.y,0.4);
  EXPECT_EQ(out.header.stamp.sec,42);
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.origin=raw.pose.pose;grid.data={0,90};
  const auto transformed=usv_mpc_control::transformWorldMessage(grid,tf);
  EXPECT_EQ(transformed.data,grid.data);
  EXPECT_NEAR(transformed.info.origin.position.y,2,1e-9);
  nav_msgs::msg::Path path;
  path.header=raw.header;
  geometry_msgs::msg::PoseStamped pose;pose.header=raw.header;pose.pose=raw.pose.pose;
  path.poses.push_back(pose);
  const auto transformed_path=usv_mpc_control::transformWorldMessage(path,tf);
  EXPECT_EQ(transformed_path.poses.front().header.frame_id,"odom");
  EXPECT_NEAR(transformed_path.poses.front().pose.position.y,2,1e-9);
  path.poses.front().header.frame_id="different";
  EXPECT_THROW(usv_mpc_control::transformWorldMessage(path,tf),std::runtime_error);
}

TEST(InputFrames, RejectTilt)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.rotation.x=std::sin(.1);tf.transform.rotation.w=std::cos(.1);
  nav_msgs::msg::OccupancyGrid grid;
  EXPECT_THROW(usv_mpc_control::transformWorldMessage(grid,tf),std::runtime_error);
}
