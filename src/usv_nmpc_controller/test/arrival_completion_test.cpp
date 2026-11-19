/**
 * @file arrival_completion_test.cpp
 * @brief 通过 ROS 输入回归终点到达确认，覆盖海浪漂移与计时中断。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rcl/time.h"
#include "usv_nmpc_controller/usv_nmpc_node.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

class ArrivalCompletionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // 缩短预测窗口以加速接口测试，并明确终点距离、速度和确认时间。
    const char * args[] = {
      "arrival_completion_test", "--ros-args",
      "-p", "nmpc.horizon_steps:=5", "-p", "control_frequency:=20.0",
      "-p", "finish_stable_duration_s:=1.0", "-p", "finish_distance_m:=0.5",
      "-p", "finish_speed_mps:=0.2",
      "-p", "state_timeout_s:=0.2", "-p", "reference_stale_timeout:=2.0"};
    rclcpp::init(static_cast<int>(sizeof(args) / sizeof(args[0])), args);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    controller_ = std::make_shared<UsvNmpcNode>();
    inputs_ = std::make_shared<rclcpp::Node>("arrival_test_inputs");
    odom_pub_ = inputs_->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS());
    spline_pub_ = inputs_->create_publisher<usv_interfaces::msg::UsvBspline>(
      "/planning/bspline", rclcpp::QoS(1).reliable());
    finished_sub_ = inputs_->create_subscription<std_msgs::msg::Bool>(
      "/USV_tracking_finished", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool & message) {finished_ = message.data;});
    diagnostics_sub_ = inputs_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/nmpc_tracking_diagnostics", 10,
      [this](const std_msgs::msg::Float64MultiArray & message) {diagnostics_ = message.data;});
    command_sub_ = inputs_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd", 10,
      [this](const geometry_msgs::msg::Twist & message) {commands_.push_back(message);});
    state_sub_ = inputs_->create_subscription<std_msgs::msg::String>(
      "/nmpc_tracking_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String & message) {
        tracking_state_ = message.data;
        tracking_states_.push_back(message.data);
      });
    reset_pub_ = inputs_->create_publisher<std_msgs::msg::UInt64>(
      "/planning/mission_reset", rclcpp::QoS(1).reliable());
    failed_pub_ = inputs_->create_publisher<std_msgs::msg::Header>(
      "/planning/replan_failed", rclcpp::QoS(10).reliable());
    reset_client_ = inputs_->create_client<std_srvs::srv::Trigger>("/nmpc/reset");
    executor_.add_node(controller_);
    executor_.add_node(inputs_);
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while ((odom_pub_->get_subscription_count() == 0U ||
      spline_pub_->get_subscription_count() == 0U ||
      finished_sub_->get_publisher_count() == 0U) && Clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(odom_pub_->get_subscription_count(), 0U);
    ASSERT_GT(spline_pub_->get_subscription_count(), 0U);
    ASSERT_GT(finished_sub_->get_publisher_count(), 0U);

    configureTerminalSpline();
  }

  void setStartTime(double seconds_before_now)
  {
    spline_.start_time = inputs_->now() - rclcpp::Duration::from_seconds(seconds_before_now);
  }

  void setControllerTime(double seconds)
  {
    auto * clock = controller_->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, static_cast<int64_t>(seconds * 1e9)), RCL_RET_OK);
  }

  void configureTerminalSpline()
  {
    // 夹持三次样条在 x=4 处的位置和航向导数均为零。
    spline_ = usv_interfaces::msg::UsvBspline();
    spline_.order = 3;
    spline_.traj_id = 1;
    for (const double x : {0.0, 0.0, 4.0, 4.0}) {
      usv_interfaces::msg::UsvBsplineControlPoint point;
      point.x = x;
      point.phi = -1.5707963267948966;
      spline_.control_points.push_back(point);
    }
    for (const double knot : {0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 4.0, 4.0}) {
      spline_.knots.push_back(knot);
    }
    setStartTime(5.0);
  }

  void configureLocalStraightSpline(double elapsed_s = 0.0)
  {
    // 均匀三次样条从 x=0 行驶到 x=4，末端速度非零。
    spline_ = usv_interfaces::msg::UsvBspline();
    spline_.order = 3;
    spline_.traj_id = 1;
    for (int i = 0; i < 7; ++i) {
      usv_interfaces::msg::UsvBsplineControlPoint point;
      point.x = i - 1.0;
      point.phi = -1.5707963267948966;
      spline_.control_points.push_back(point);
    }
    for (int i = 0; i < 11; ++i) {
      spline_.knots.push_back(i - 3.0);
    }
    setStartTime(elapsed_s);
  }

  void offsetSpline(double east_m, double north_m)
  {
    for (auto & point : spline_.control_points) {
      point.x += east_m;
      point.y += north_m;
    }
  }

  void TearDown() override
  {
    executor_.remove_node(controller_);
    executor_.remove_node(inputs_);
  }

  // 连续发布反馈与同一条轨迹，不模拟真实动力学；这里只验证完成状态机。
  void runFor(
    double seconds, const std::function<void(double)> & publish_state,
    bool publish_reference = true)
  {
    const auto start = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - start).count() < seconds) {
      publish_state(std::chrono::duration<double>(Clock::now() - start).count());
      if (publish_reference) {spline_pub_->publish(spline_);}
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_.spin_some();
  }

  void primeReferenceAtCurrentProjection()
  {
    usv_nmpc::BsplineReference evaluator;
    usv_nmpc::RefPoint current_reference;
    std::string error;
    ASSERT_TRUE(evaluator.configure(spline_, error)) << error;
    const double elapsed = std::max(
      0.0, (inputs_->now() - rclcpp::Time(spline_.start_time)).seconds());
    ASSERT_TRUE(evaluator.sample(elapsed, current_reference));
    runFor(0.15, [this, current_reference](double) {
        publishOdom(current_reference.x, current_reference.psi);
      });
    ASSERT_FALSE(diagnostics_.empty());
  }

  void publishOdom(
    double x, double yaw = 0.0, double u = 0.0, double v = 0.0,
    double r = 0.0)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "usv_body_rfu";
    odom.header.stamp = inputs_->now();
    odom.pose.pose.position.x = x;
    // /odom RFU 四元数的 yaw 与模型导航角相同；v 在 RFU 的 x 轴向右。
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = u;
    odom.twist.twist.angular.z = r;
    odom_pub_->publish(odom);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<UsvNmpcNode> controller_;
  rclcpp::Node::SharedPtr inputs_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<usv_interfaces::msg::UsvBspline>::SharedPtr spline_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr reset_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr failed_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  std::string tracking_state_;
  std::vector<std::string> tracking_states_;
  usv_interfaces::msg::UsvBspline spline_;
  std::vector<double> diagnostics_;
  std::vector<geometry_msgs::msg::Twist> commands_;
  bool finished_{false};
};

TEST_F(ArrivalCompletionTest, WithdrawnReferenceStopsAndAcceptsNewerTrajectory)
{
  configureLocalStraightSpline();
  primeReferenceAtCurrentProjection();
  std_msgs::msg::Header failure;
  failure.stamp = inputs_->now();
  failed_pub_->publish(failure);
  commands_.clear();
  runFor(0.15, [this](double) {publishOdom(0.0, -1.5707963267948966);});
  EXPECT_EQ(tracking_state_, "WAIT_REFERENCE");
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  setStartTime(0.0);
  spline_.traj_id++;
  runFor(0.25, [this](double) {publishOdom(0.0, -1.5707963267948966);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  // 延迟到达的旧撤销通知不得撤销更新的轨迹。
  failed_pub_->publish(failure);
  runFor(0.1, [this](double) {publishOdom(0.0, -1.5707963267948966);});
  EXPECT_EQ(tracking_state_, "TRACKING");
}

TEST_F(ArrivalCompletionTest, ArrivalUsesDistanceAndSpeedOnly)
{
  primeReferenceAtCurrentProjection();
  // 距离合格但速度高于配置上限时不能 finish。
  runFor(1.2, [this](double) {
      publishOdom(4.0, 0.0, 0.21, 0.0, 0.1);
    });
  EXPECT_FALSE(finished_);
  ASSERT_GE(diagnostics_.size(), 59U);
  EXPECT_DOUBLE_EQ(diagnostics_[57], 0.0);

  // 航向偏差和转艏率不参与到达判定；距离、合速度及持续时间达标即可完成。
  runFor(1.3, [this](double) {publishOdom(4.0, 0.0, 0.2, 0.0, 0.1);});
  EXPECT_TRUE(finished_);
  EXPECT_EQ(tracking_state_, "FINISHED");
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
  // 完成锁定后，即使海浪继续推离终点，也持续输出零指令。
  commands_.clear();
  runFor(0.25, [this](double) {publishOdom(4.8, 1.2, 0.4, 0.2, 0.1);});
  EXPECT_TRUE(finished_);
  ASSERT_FALSE(commands_.empty());
  for (const auto & command : commands_) {
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
}

TEST_F(ArrivalCompletionTest, LeavingArrivalAreaRestartsConfirmation)
{
  primeReferenceAtCurrentProjection();
  runFor(0.6, [this](double) {publishOdom(4.0, -1.5707963267948966, 0.05, 0.0, 0.0);});
  ASSERT_FALSE(finished_);
  ASSERT_GE(diagnostics_.size(), 59U);
  EXPECT_GT(diagnostics_[57], 0.3);
  runFor(0.2, [this](double) {publishOdom(4.8);});
  ASSERT_FALSE(finished_);
  EXPECT_DOUBLE_EQ(diagnostics_[57], 0.0);
  runFor(0.6, [this](double) {publishOdom(4.0, -1.5707963267948966, 0.05, 0.0, 0.0);});
  EXPECT_FALSE(finished_);
  runFor(0.7, [this](double) {publishOdom(4.0, -1.5707963267948966, 0.05, 0.0, 0.0);});
  EXPECT_TRUE(finished_);
}

TEST_F(ArrivalCompletionTest, StateTimeoutRestartsConfirmation)
{
  primeReferenceAtCurrentProjection();
  runFor(0.6, [this](double) {
      publishOdom(4.0, -1.5707963267948966, 0.05, 0.0, 0.0);
    });
  ASSERT_FALSE(finished_);
  runFor(0.5, [](double) {});
  ASSERT_FALSE(finished_);
  EXPECT_EQ(tracking_state_, "SAFE_STOP:ODOM_TIMEOUT");
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
  runFor(0.6, [this](double) {
      publishOdom(4.0, -1.5707963267948966, 0.05, 0.0, 0.0);
    });
  EXPECT_FALSE(finished_);
  EXPECT_EQ(tracking_state_, "ARRIVAL_CONFIRM");
  runFor(0.7, [this](double) {
      publishOdom(4.0, -1.5707963267948966, 0.05, 0.0, 0.0);
    });
  EXPECT_TRUE(finished_);
}

TEST_F(ArrivalCompletionTest, BeforeEndpointDoesNotFinish)
{
  // 末端保持巡航速度的滚动局部段不能触发任务完成。
  configureLocalStraightSpline();
  runFor(1.3, [this](double) {publishOdom(0.0);});
  EXPECT_FALSE(finished_);
  ASSERT_GE(diagnostics_.size(), 59U);
  EXPECT_DOUBLE_EQ(diagnostics_[2], 0.0);
  EXPECT_GT(diagnostics_[4], 0.0);
  EXPECT_LT(diagnostics_[4], 4.0);
}

TEST_F(ArrivalCompletionTest, RejectsSplineStartingMoreThanTwoMetersFromOdom)
{
  offsetSpline(2.01, 0.0);
  setStartTime(0.0);
  runFor(0.5, [this](double) {publishOdom(0.0);});
  EXPECT_TRUE(diagnostics_.empty());
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
}

TEST_F(ArrivalCompletionTest, AcceptsSplineStartingExactlyTwoMetersFromOdom)
{
  offsetSpline(2.0, 0.0);
  setStartTime(-0.2);
  runFor(0.5, [this](double) {publishOdom(0.0);});
  EXPECT_FALSE(diagnostics_.empty());
}

TEST_F(ArrivalCompletionTest, ClosedRouteDoesNotFinishAtItsStart)
{
  setStartTime(0.0);
  spline_.control_points.clear();
  const std::vector<std::pair<double, double>> controls{
    {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {4.0, 0.0}, {4.0, 4.0},
    {0.0, 4.0}, {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}};
  for (const auto & xy : controls) {
    usv_interfaces::msg::UsvBsplineControlPoint point;
    point.x = xy.first;
    point.y = xy.second;
    point.phi = -1.5707963267948966;
    spline_.control_points.push_back(point);
  }
  spline_.knots.clear();
  for (int i = 0; i < 13; ++i) {
    spline_.knots.push_back(i - 3.0);
  }
  runFor(1.3, [this](double) {publishOdom(0.0);});
  EXPECT_FALSE(finished_);
  ASSERT_GE(diagnostics_.size(), 59U);
  EXPECT_DOUBLE_EQ(diagnostics_[2], 0.0);
}

TEST_F(ArrivalCompletionTest, ReferenceSharesVesselArrivalTolerance)
{
  // 终端参考位于 x=4；船位距终点 0.75 m，超出设定容差。
  primeReferenceAtCurrentProjection();
  runFor(0.35, [this](double) {publishOdom(3.25);});
  ASSERT_GE(diagnostics_.size(), 59U);
  EXPECT_NEAR(diagnostics_[4], 4.0, 1e-9);
  EXPECT_DOUBLE_EQ(diagnostics_[2], 1.0);
  EXPECT_NEAR(diagnostics_[54], 0.75, 1e-9);
  EXPECT_DOUBLE_EQ(diagnostics_[56], 0.0);
  EXPECT_FALSE(finished_);
}

TEST_F(ArrivalCompletionTest, NearEndpointPreservesBsplineHeadingAndYawRate)
{
  // 样条位置为 x(t)=t，航向为 psi(t)=base+0.1t；在 x=3 附近采样。
  configureLocalStraightSpline(1.3);
  for (int i = 0; i < 7; ++i) {
    spline_.control_points[i].phi = -1.5707963267948966 + 0.1 * (i - 1);
  }
  primeReferenceAtCurrentProjection();
  runFor(1.7, [this](double) {
      publishOdom(3.0, -1.5707963267948966 + 0.3, 0.5, 0.0, 0.05);
    });
  ASSERT_GE(diagnostics_.size(), 11U);
  EXPECT_NEAR(diagnostics_[6], -1.5707963267948966 + 0.1 * diagnostics_[4], 1e-9);
  EXPECT_NEAR(diagnostics_[9],
    0.1 * std::hypot(diagnostics_[7], diagnostics_[8]), 0.02);
}

TEST_F(ArrivalCompletionTest, StartupAndFutureStartHaveExplicitWaitStates)
{
  runFor(0.15, [](double) {}, false);
  EXPECT_EQ(tracking_state_, "WAIT_ODOM");
  runFor(0.15, [this](double) {publishOdom(0.0);}, false);
  EXPECT_EQ(tracking_state_, "WAIT_REFERENCE");
  configureLocalStraightSpline(-0.7);
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "WAIT_START");
  EXPECT_TRUE(diagnostics_.empty());
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  runFor(0.65, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_DOUBLE_EQ(diagnostics_[67], 3.0);
  EXPECT_DOUBLE_EQ(diagnostics_[68], 0.0);
}

TEST_F(ArrivalCompletionTest, RollingReplacementKeepsCommandsAndRejectedReferenceKeepsOldSegment)
{
  configureLocalStraightSpline();
  runFor(0.5, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  const double revision = diagnostics_[15];
  commands_.clear();
  ++spline_.traj_id;
  runFor(0.25, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], revision + 1.0);
  ASSERT_FALSE(commands_.empty());
  for (const auto & command : commands_) {
    EXPECT_GT(command.linear.x, 0.0);
  }
  ++spline_.traj_id;
  offsetSpline(3.0, 0.0);
  runFor(0.25, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], revision + 1.0);
}

TEST_F(ArrivalCompletionTest, FutureRollingReferenceWaitsWithoutPausingActiveSpline)
{
  configureLocalStraightSpline(0.3);
  runFor(0.3, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  ASSERT_EQ(tracking_state_, "TRACKING");
  const double active_reference_x = diagnostics_[4];
  const double active_reference_psi = diagnostics_[6];
  const double active_revision = diagnostics_[15];
  tracking_states_.clear();
  commands_.clear();

  ++spline_.traj_id;
  setStartTime(-0.6);
  offsetSpline(0.6, 0.0);
  for (auto & point : spline_.control_points) {
    point.phi += 0.2;
  }
  runFor(0.3, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});

  EXPECT_EQ(tracking_state_, "TRACKING");
  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_GT(diagnostics_[4], active_reference_x + 0.1);
  EXPECT_NEAR(diagnostics_[6], active_reference_psi, 0.05);
  EXPECT_DOUBLE_EQ(diagnostics_[15], active_revision);
  EXPECT_EQ(std::find(tracking_states_.begin(), tracking_states_.end(), "WAIT_START"),
    tracking_states_.end());
  ASSERT_FALSE(commands_.empty());
  EXPECT_GT(commands_.back().linear.x, 0.0);

  runFor(0.5, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], active_revision + 1.0);
  EXPECT_NEAR(diagnostics_[6], active_reference_psi + 0.2, 0.05);
}

TEST_F(ArrivalCompletionTest, CurrentReplanSupersedesQueuedFutureSlice)
{
  configureLocalStraightSpline(0.3);
  runFor(0.3, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  const double active_revision = diagnostics_[15];

  ++spline_.traj_id;
  setStartTime(-0.6);
  offsetSpline(0.6, 0.0);
  runFor(0.15, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], active_revision);

  ++spline_.traj_id;
  setStartTime(0.0);
  offsetSpline(-0.6, 1.8);
  for (auto & point : spline_.control_points) {
    point.phi += 2.0;
  }
  runFor(0.15, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});

  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], active_revision + 1.0);
  EXPECT_NEAR(diagnostics_[6], -1.5707963267948966 + 2.0, 0.05);

  runFor(0.5, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], active_revision + 1.0);
  EXPECT_NEAR(diagnostics_[6], -1.5707963267948966 + 2.0, 0.05);
}

TEST_F(ArrivalCompletionTest, NearbyReplanCanChangePositionAndHeadingWithoutContinuityGate)
{
  configureLocalStraightSpline();
  runFor(0.3, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  const double revision = diagnostics_[15];

  ++spline_.traj_id;
  offsetSpline(0.0, 1.8);
  for (auto & point : spline_.control_points) {
    point.phi += 2.0;
  }
  runFor(0.25, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});

  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], revision + 1.0);
  EXPECT_GT(diagnostics_[5], 1.7);
  EXPECT_NEAR(diagnostics_[6], -1.5707963267948966 + 2.0, 0.05);
  ASSERT_FALSE(commands_.empty());
  EXPECT_GT(commands_.back().linear.x, 0.0);
}

TEST_F(ArrivalCompletionTest, DuplicatePublicationRefreshesLivenessWithoutResettingReferenceTime)
{
  configureLocalStraightSpline();
  runFor(0.3, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  ASSERT_EQ(tracking_state_, "TRACKING");
  const double first_reference_x = diagnostics_[4];
  const double accepted_revision = diagnostics_[15];

  runFor(0.25, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});

  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_GT(diagnostics_[4], first_reference_x + 0.1);
  EXPECT_DOUBLE_EQ(diagnostics_[15], accepted_revision);
}

TEST_F(ArrivalCompletionTest, ExpiredMovingSplineReplayCannotKeepReferenceAlive)
{
  configureLocalStraightSpline(4.1);
  usv_nmpc::BsplineReference evaluator;
  usv_nmpc::RefPoint current_reference;
  std::string error;
  ASSERT_TRUE(evaluator.configure(spline_, error)) << error;
  ASSERT_TRUE(evaluator.sample(4.1, current_reference));

  runFor(0.15, [this, current_reference](double) {
      publishOdom(current_reference.x, current_reference.psi,
        current_reference.u, current_reference.v, current_reference.r);
    });
  ASSERT_EQ(tracking_state_, "TRACKING");

  runFor(2.2, [this, current_reference](double) {
      publishOdom(current_reference.x, current_reference.psi,
        current_reference.u, current_reference.v, current_reference.r);
    });
  EXPECT_EQ(tracking_state_, "SAFE_STOP:REFERENCE_TIMEOUT");
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
}

TEST_F(ArrivalCompletionTest, DelayedReferenceChecksItsCurrentTimeProjection)
{
  configureLocalStraightSpline();
  runFor(0.35, [this](double) {
      publishOdom(0.0, -1.5707963267948966, 1.0);
    });
  ASSERT_GE(diagnostics_.size(), 69U);
  ASSERT_EQ(tracking_state_, "TRACKING");
  const double accepted_revision = diagnostics_[15];

  // 样条首点仍在船位，但按该时间戳求得的当前参考点已距船超过 2 m。
  ++spline_.traj_id;
  setStartTime(3.0);
  runFor(0.25, [this](double) {
      publishOdom(0.0, -1.5707963267948966, 1.0);
    });

  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], accepted_revision);
}

TEST_F(ArrivalCompletionTest, NearbyOutOfOrderPublicationCannotReplaceNewerFutureSegment)
{
  configureLocalStraightSpline();
  runFor(0.2, [this](double) {publishOdom(0.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  ASSERT_EQ(tracking_state_, "TRACKING");
  const double accepted_revision = diagnostics_[15];

  auto newer_spline = spline_;
  newer_spline.traj_id++;
  const auto newer_start = inputs_->now() + rclcpp::Duration::from_seconds(0.8);
  newer_spline.start_time = newer_start;
  spline_ = newer_spline;
  runFor(0.12, [this](double) {publishOdom(0.0);});
  ASSERT_EQ(tracking_state_, "TRACKING");

  auto older_spline = newer_spline;
  older_spline.traj_id++;
  older_spline.start_time =
    newer_start - rclcpp::Duration::from_seconds(1.2);
  usv_nmpc::BsplineReference evaluator;
  usv_nmpc::RefPoint projected_reference;
  std::string error;
  ASSERT_TRUE(evaluator.configure(older_spline, error)) << error;
  const double elapsed = std::max(
    0.0, (inputs_->now() - rclcpp::Time(older_spline.start_time)).seconds());
  ASSERT_TRUE(evaluator.sample(elapsed, projected_reference));
  EXPECT_LT(std::hypot(projected_reference.x, projected_reference.y), 2.0);

  spline_ = older_spline;
  runFor(0.12, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");

  spline_ = newer_spline;
  runFor(0.85, [this](double) {publishOdom(0.0);});
  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_EQ(tracking_state_, "TRACKING");
  EXPECT_DOUBLE_EQ(diagnostics_[15], accepted_revision + 1.0);
}

TEST_F(ArrivalCompletionTest, ReferenceTimeoutStopsAndRepeatRestoresTracking)
{
  configureLocalStraightSpline();
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  runFor(2.2, [this](double) {publishOdom(0.0);}, false);
  EXPECT_EQ(tracking_state_, "SAFE_STOP:REFERENCE_TIMEOUT");
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
}

TEST_F(ArrivalCompletionTest, OneHertzReferenceGapKeepsCurrentSplineActive)
{
  configureLocalStraightSpline();
  runFor(0.2, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);});
  ASSERT_EQ(tracking_state_, "TRACKING");
  const double initial_reference_x = diagnostics_[4];

  runFor(1.2, [this](double) {publishOdom(0.0, -1.5707963267948966, 1.0);}, false);

  EXPECT_EQ(tracking_state_, "TRACKING");
  ASSERT_GE(diagnostics_.size(), 69U);
  EXPECT_GT(diagnostics_[4], initial_reference_x + 0.5);
}

TEST_F(ArrivalCompletionTest, InvalidOdometryImmediatelyStopsThenRecovers)
{
  configureLocalStraightSpline();
  runFor(0.25, [this](double) {publishOdom(0.0);});
  runFor(0.15, [this](double) {
      nav_msgs::msg::Odometry invalid;
      odom_pub_->publish(invalid);
    });
  EXPECT_EQ(tracking_state_, "WAIT_ODOM");
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
}

TEST_F(ArrivalCompletionTest, AcceptedNewSegmentUnlocksFinishedAndRepublishesFalse)
{
  primeReferenceAtCurrentProjection();
  runFor(1.25, [this](double) {publishOdom(4.0);});
  ASSERT_TRUE(finished_);
  ASSERT_EQ(tracking_state_, "FINISHED");
  // 无效的新参考不能解除已完成状态。
  ++spline_.traj_id;
  offsetSpline(20.0, 0.0);
  runFor(0.15, [this](double) {publishOdom(4.0);});
  EXPECT_TRUE(finished_);
  EXPECT_EQ(tracking_state_, "FINISHED");
  configureLocalStraightSpline();
  offsetSpline(4.0, 0.0);
  runFor(0.25, [this](double) {publishOdom(4.0);});
  EXPECT_FALSE(finished_);
  EXPECT_EQ(tracking_state_, "TRACKING");
}

TEST_F(ArrivalCompletionTest, MissionResetAndServiceResetClearReferenceAndStop)
{
  configureLocalStraightSpline();
  runFor(0.25, [this](double) {publishOdom(0.0);});
  std_msgs::msg::UInt64 reset;
  reset.data = 42U;
  reset_pub_->publish(reset);
  // 显式复位后，重复收到旧时钟周期的参考仍应被拒绝。
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "WAIT_REFERENCE");
  EXPECT_FALSE(finished_);
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  setStartTime(0.0);
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
  ASSERT_TRUE(reset_client_->wait_for_service(std::chrono::seconds(1)));
  auto future = reset_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(executor_.spin_until_future_complete(future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);
  runFor(0.15, [](double) {}, false);
  EXPECT_EQ(tracking_state_, "WAIT_ODOM");
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
}

TEST_F(ArrivalCompletionTest, PausedRosTimeDoesNotResolveButWallTimeOdomTimeoutStillStops)
{
  setControllerTime(100.0);
  runFor(0.2, [this](double) {publishOdom(0.0);}, false);
  configureLocalStraightSpline();
  spline_.start_time = rclcpp::Time(100, 0, RCL_ROS_TIME);
  runFor(0.2, [this](double) {publishOdom(0.0);});
  ASSERT_EQ(tracking_state_, "TRACKING");
  ASSERT_FALSE(diagnostics_.empty());
  const auto command_count = commands_.size();
  runFor(0.25, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(commands_.size(), command_count);
  EXPECT_FALSE(finished_);
  setControllerTime(100.1);
  runFor(0.15, [this](double) {publishOdom(0.0);});
  EXPECT_GT(commands_.size(), command_count);
  runFor(0.35, [](double) {}, false);
  EXPECT_EQ(tracking_state_, "SAFE_STOP:ODOM_TIMEOUT");
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
}

TEST_F(ArrivalCompletionTest, ClockRewindClearsFutureReferenceBeforeReadinessGate)
{
  setControllerTime(100.0);
  runFor(0.2, [this](double) {publishOdom(0.0);}, false);
  configureLocalStraightSpline();
  spline_.start_time = rclcpp::Time(105, 0, RCL_ROS_TIME);
  runFor(0.15, [this](double) {publishOdom(0.0);});
  ASSERT_EQ(tracking_state_, "WAIT_START");
  tracking_states_.clear();
  setControllerTime(10.0);
  runFor(0.15, [](double) {}, false);
  EXPECT_NE(std::find(tracking_states_.begin(), tracking_states_.end(),
      "SAFE_STOP:CLOCK_REWIND"), tracking_states_.end());
  EXPECT_EQ(tracking_state_, "WAIT_ODOM");
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  runFor(0.15, [this](double) {publishOdom(0.0);}, false);
  EXPECT_EQ(tracking_state_, "WAIT_REFERENCE");
  spline_.start_time = rclcpp::Time(10, 0, RCL_ROS_TIME);
  runFor(0.15, [this](double) {publishOdom(0.0);});
  EXPECT_EQ(tracking_state_, "TRACKING");
}
}  // namespace
