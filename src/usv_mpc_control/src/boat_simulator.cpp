#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "usv_mpc_control/underactuated_model.hpp"

class BoatSimulator : public rclcpp::Node
{
public:
  // [功能与联系] 建立六状态欠驱动模型、执行器限制与控制订阅，启动20Hz运动和数据发布；只能运行一个base_link发布者。
  BoatSimulator()
  : Node("boat_simulator")
  {
    // ==================== 船初始状态 ====================
    x_ = declare_parameter<double>("start_x", 0.0);
    y_ = declare_parameter<double>("start_y", 0.0);
    yaw_ = declare_parameter<double>("start_yaw", 0.0);

    // MPC给出纵向速度及转向指令；实际速度由船模积分。
    max_linear_speed_ = std::max(
      0.0, declare_parameter<double>("max_linear_speed", 1.2));

    // 仅有纵向初速；侧向速度与转艏率由动力学状态自然发展。
    state_.x = x_;
    state_.y = y_;
    state_.yaw = yaw_;
    state_.u = std::clamp(
      declare_parameter<double>("initial_speed", 0.0),
      0.0, max_linear_speed_);
    max_angular_speed_ = std::max(0.0,
      declare_parameter<double>("max_angular_speed", 1.5));
    max_v_cmd_ = std::max(0.0,
      declare_parameter<double>("max_v_cmd", 2.0));
    max_w_cmd_ = std::max(0.0,
      declare_parameter<double>("max_w_cmd", 0.3));
    effort_timeout_ = std::max(0.1, declare_parameter<double>("effort_timeout", 0.5));
    effort_topic_ = declare_parameter<std::string>("effort_topic", "/usv/cmd_velocity");

    // ==================== ROS 订阅 ====================
    effort_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      effort_topic_, rclcpp::QoS(10),
      std::bind(&BoatSimulator::effortCallback, this, std::placeholders::_1));

    // ==================== ROS 发布 ====================
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(20));

    boat_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
      "/sim_boat_marker",
      rclcpp::QoS(10));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 控制频率 20 Hz
    update_period_ = 0.05;

    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(update_period_)),
      std::bind(&BoatSimulator::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Boat simulator ready: MPC velocity-command input, Fast-Planner six-state model");
  }

private:
  // [功能与联系] 接收MPC的速度指令，输入到规划器相同的欠驱动模型。
  void effortCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_cmd_ = std::clamp(msg->linear.x, 0.0, max_v_cmd_);
    w_cmd_ = std::clamp(msg->angular.z, -max_w_cmd_, max_w_cmd_);
    effort_receipt_time_ = now();
    has_effort_command_ = true;
  }

  // 仿真主循环：船模积分、里程计、TF与船体显示。
  // 状态推进与发布使用同一20Hz定时器。
  void update()
  {
    updateBoatMotion();
    publishOdomAndTf();
    publishBoatMarker();
  }

  // 六状态欠驱动模型：接收速度及转向指令，侧滑由船体动力学产生。
  // MPC控制量经RK4积分，结果经/odom反馈给规划与控制。
  void updateBoatMotion()
  {
    usv_mpc_control::VesselControl control;
    if (has_effort_command_ && (now() - effort_receipt_time_).seconds() <= effort_timeout_) {
      control = {v_cmd_, w_cmd_};
    } else {
      // No reverse thrust is available: zero speed command and yaw-rate damping.
      control = {0.0, std::clamp((-0.166118168 * state_.v + 0.544509453 * state_.r
        - 0.000626529) / 0.837536662, -max_w_cmd_, max_w_cmd_)};
    }
    state_ = usv_mpc_control::integrateRk4(state_, control, model_, update_period_);
    state_.u = std::clamp(state_.u, 0.0, max_linear_speed_);
    state_.r = std::clamp(state_.r, -max_angular_speed_, max_angular_speed_);
    x_ = state_.x;
    y_ = state_.y;
    yaw_ = state_.yaw;
  }

  // 发布 /odom 和 odom -> base_link TF
  // [功能与联系] 发布本船/odom及odom→base_link，反馈给航道分类、两级规划器并供RViz定位。
  void publishOdomAndTf()
  {
    const rclcpp::Time stamp = now();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    geometry_msgs::msg::Quaternion orientation =
      tf2::toMsg(q);

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = orientation;

    // nav_msgs/Odometry twist is expressed in child_frame_id (body frame).
    odom.twist.twist.linear.x = state_.u;
    odom.twist.twist.linear.y = state_.v;
    odom.twist.twist.angular.z = state_.r;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  // RViz 显示无人船
  // [功能与联系] 在本船姿态处显示模型；与TF显示重复开启时可能看似有两艘船。
  void publishBoatMarker()
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "odom";
    marker.header.stamp = now();

    marker.ns = "sim_boat";
    marker.id = 0;

    marker.type =
      visualization_msgs::msg::Marker::ARROW;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = 0.15;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = 1.0;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    marker.color.a = 1.0F;
    marker.color.r = 0.0F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;

    boat_marker_pub_->publish(marker);
  }

  // ==================== 船状态 ====================
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  usv_mpc_control::VesselState state_;
  usv_mpc_control::VesselModelParameters model_;
  double v_cmd_{0.0};
  double w_cmd_{0.0};
  double effort_timeout_{0.5};
  bool has_effort_command_{false};
  rclcpp::Time effort_receipt_time_{0, 0, RCL_ROS_TIME};
  std::string effort_topic_{"/usv/cmd_velocity"};

  // ==================== 船体运动约束 ====================
  double max_linear_speed_{1.2};
  double max_angular_speed_{1.5};

  double max_v_cmd_{2.0};
  double max_w_cmd_{0.3};

  double update_period_{0.05};

  // ==================== ROS 通信对象 ====================
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr effort_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_pub_;


  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    boat_marker_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr
    update_timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<BoatSimulator>());

  rclcpp::shutdown();

  return 0;
}
