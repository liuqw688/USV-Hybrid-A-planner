/**
 * @file usv_nmpc_node.cpp
 * @brief 无人船 NMPC ROS 节点实现。
 */

#include "usv_nmpc_controller/usv_nmpc_node.hpp"

#include "usv_nmpc_controller/bspline_reference.hpp"
#include "usv_nmpc_controller/coordinate_conventions.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <vector>

namespace
{
namespace coordinates = usv_nmpc::coordinates;

bool sameBspline(
  const usv_interfaces::msg::UsvBspline & a,
  const usv_interfaces::msg::UsvBspline & b)
{
  if (a.order != b.order || a.traj_id != b.traj_id ||
    a.start_time.sec != b.start_time.sec ||
    a.start_time.nanosec != b.start_time.nanosec ||
    a.knots != b.knots || a.control_points.size() != b.control_points.size())
  {
    return false;
  }
  for (std::size_t i = 0; i < a.control_points.size(); ++i) {
    if (a.control_points[i].x != b.control_points[i].x ||
      a.control_points[i].y != b.control_points[i].y ||
      a.control_points[i].phi != b.control_points[i].phi)
    {
      return false;
    }
  }
  return true;
}

double progressAtTime(
  const usv_nmpc::ReferenceTrajectory & trajectory, double elapsed_s)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  if (elapsed_s <= trajectory.points.front().t) {
    return trajectory.points.front().s;
  }
  const auto upper = std::upper_bound(
    trajectory.points.begin(), trajectory.points.end(), elapsed_s,
    [](double value, const usv_nmpc::ReferencePoint & point) {
      return value < point.t;
    });
  if (upper == trajectory.points.end()) {
    return trajectory.points.back().s;
  }
  const auto & b = *upper;
  const auto & a = *(upper - 1);
  const double duration = b.t - a.t;
  if (duration <= 1e-12) {
    return b.s;
  }
  const double ratio = (elapsed_s - a.t) / duration;
  return a.s + ratio * (b.s - a.s);
}

}  // namespace


UsvNmpcNode::UsvNmpcNode()
: Node("usv_nmpc_node")
{
  loadParameters();
  nmpc_config_ = loadNmpcConfig();
  nmpc_ = std::make_unique<usv_nmpc::NMPCController>(nmpc_config_);
  setupRosInterfaces();
  publishTrackingState();
  RCLCPP_INFO(get_logger(), "NMPC 已启动：预测步数=%d，步长=%.2f s；参考按样条时间直接采样",
    nmpc_config_.horizon_steps, nmpc_config_.dt);
}

void UsvNmpcNode::setupRosInterfaces()
{
  state_timeout_s_ = declare_parameter("state_timeout_s", 0.5);
  reference_stale_timeout_s_ = declare_parameter(
    "reference_stale_timeout", reference_stale_timeout_s_);
  if (!std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0 ||
    !std::isfinite(reference_stale_timeout_s_) || reference_stale_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("状态和参考超时必须为有限正数");
  }
  max_reference_start_distance_m_ = declare_parameter(
    "max_reference_start_distance_m", max_reference_start_distance_m_);
  if (!std::isfinite(max_reference_start_distance_m_) ||
    max_reference_start_distance_m_ <= 0.0)
  {
    throw std::invalid_argument("max_reference_start_distance_m 必须为有限正数");
  }
  state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&UsvNmpcNode::stateCallback, this, std::placeholders::_1));
  frame_id_ = declare_parameter("frame_id", std::string("map"));
  bspline_sub_ = create_subscription<Bspline>(
    bspline_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&UsvNmpcNode::bsplineCallback, this, std::placeholders::_1));
  replan_failed_sub_ = create_subscription<std_msgs::msg::Header>(
    "/planning/replan_failed", rclcpp::QoS(10).reliable(),
    std::bind(&UsvNmpcNode::replanFailedCallback, this, std::placeholders::_1));
  mission_reset_sub_ = create_subscription<std_msgs::msg::UInt64>(
    "/planning/mission_reset", rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&UsvNmpcNode::missionResetCallback, this, std::placeholders::_1));
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_,
    rclcpp::QoS(10));
  predicted_path_topic_ = declare_parameter(
    "predicted_path_topic", predicted_path_topic_);
  predicted_poses_topic_ = declare_parameter(
    "predicted_poses_topic", predicted_poses_topic_);
  predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    predicted_path_topic_, rclcpp::QoS(1));
  predicted_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
    predicted_poses_topic_, rclcpp::QoS(1));
  reference_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/nmpc_reference_pose", rclcpp::QoS(1));
  finished_pub_ = create_publisher<std_msgs::msg::Bool>(
    tracking_finished_topic_, rclcpp::QoS(1).reliable().transient_local());
  tracking_state_pub_ = create_publisher<std_msgs::msg::String>(
    "/nmpc_tracking_state", rclcpp::QoS(1).reliable().transient_local());
  diagnostics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    nmpc_diagnostics_topic_, rclcpp::QoS(10));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
    "/nmpc/reset",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      auto reset = std::make_shared<std_msgs::msg::UInt64>();
      reset->data = terminal_mission_id_;
      missionResetCallback(reset);
      std::lock_guard<std::mutex> lock(mutex_);
      accept_reference_after_ = now();
      trajectory_revision_ = 0;
      has_state_ = false;
      state_ = {};
      tracking_fsm_.reset(trackingInputs(now()));
      publishTrackingState();
      // 求解器重建可能耗时；在进入此回调前已先发布零指令。
      try {
        auto controller = std::make_unique<usv_nmpc::NMPCController>(nmpc_config_);
        nmpc_ = std::move(controller);
        has_published_prediction_ = true;
        publishStopCommand();
        response->success = true;
        response->message = "NMPC 已重新初始化；控制输出为零，等待新路线";
      } catch (const std::exception & e) {
        publishStopCommand();
        response->success = false;
        response->message = std::string("初始化失败，保持停车：") + e.what();
      }
    });
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_)),
    std::bind(&UsvNmpcNode::timerCallback, this));
}

void UsvNmpcNode::stateCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const auto & pose = msg->pose.pose;
  const auto & twist = msg->twist.twist;
  const auto & q = pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const bool valid = msg->header.frame_id == frame_id_ &&
    msg->child_frame_id == "usv_body_rfu" && std::isfinite(q_norm) && q_norm > 1e-9 &&
    std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
    std::isfinite(twist.angular.z);
  if (!valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    has_state_ = false;
    tracking_fsm_.updateInputs(trackingInputs(now()));
    publishStopCommand();
    publishTrackingState();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "忽略无效 /odom；要求 frame_id=%s、child_frame_id=usv_body_rfu", frame_id_.c_str());
    return;
  }
  const double measured_yaw = coordinates::rfuOdometryHeading(
    q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm);
  // /odom 位姿使用 ENU，twist 使用 RFU：linear.y 为前向 u，linear.x 为右向 v。
  const double measured_u = twist.linear.y;
  const double measured_v = twist.linear.x;
  const double measured_r = twist.angular.z;
  UsvState measured_state;
  measured_state.x = pose.position.x;
  measured_state.y = pose.position.y;
  measured_state.yaw = measured_yaw;
  measured_state.u = measured_u;
  measured_state.v = measured_v;
  measured_state.r = measured_r;
  measured_state.initialized = true;
  std::lock_guard<std::mutex> lock(mutex_);
  last_state_received_ = std::chrono::steady_clock::now();
  state_ = measured_state;
  has_state_ = true;
}

void UsvNmpcNode::replanFailedCallback(const std_msgs::msg::Header::SharedPtr msg)
{
  if (!msg || (msg->stamp.sec == 0 && msg->stamp.nanosec == 0)) return;
  const rclcpp::Time invalidated_at(msg->stamp);
  std::lock_guard<std::mutex> lock(mutex_);
  if (invalidated_at < accept_reference_after_ ||
    (has_trajectory_ && invalidated_at < trajectory_start_time_)) return;
  accept_reference_after_ = invalidated_at;
  has_trajectory_ = false;
  has_pending_trajectory_ = false;
  has_last_bspline_ = false;
  trajectory_ = {};
  pending_trajectory_ = {};
  nmpc_->resetWarmStart();
  tracking_fsm_.updateInputs(trackingInputs(now()));
  publishStopCommand();
  publishTrackingState();
  RCLCPP_WARN(get_logger(), "规划器撤销失效参考，停车并等待新轨迹");
}

void UsvNmpcNode::bsplineCallback(const Bspline::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const bool zero_start = msg->start_time.sec == 0 && msg->start_time.nanosec == 0;
  const rclcpp::Time received_time = now();
  const rclcpp::Time message_start = zero_start ? received_time : rclcpp::Time(msg->start_time);
  // reset 服务建立接收屏障，避免旧发布器继续重复发送的样条在复位后重新接管控制器。
  // B 样条没有接收时间字段，因此用其绝对 start_time 判断；零时间戳按接收时刻处理。
  if (accept_reference_after_.nanoseconds() != 0 &&
    message_start < accept_reference_after_)
  {
    RCLCPP_DEBUG(get_logger(), "忽略复位前的旧 B 样条参考");
    return;
  }
  bool older_reference = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_last_bspline_ && sameBspline(*msg, last_bspline_)) {
      // 允许重复发布静止终端段；运动段到达末端后不能靠重复消息无限续期。
      const auto & last_trajectory = has_pending_trajectory_ ?
        pending_trajectory_ : trajectory_;
      const auto & last_evaluator = has_pending_trajectory_ ?
        pending_bspline_evaluator_ : bspline_evaluator_;
      const auto & last_start_time = has_pending_trajectory_ ?
        pending_trajectory_start_time_ : trajectory_start_time_;
      const double elapsed = (received_time - last_start_time).seconds();
      if (last_trajectory.terminal || elapsed <= last_evaluator.duration()) {
        last_trajectory_time_ = received_time;
      }
      return;
    }
    older_reference = has_trajectory_ && message_start < trajectory_start_time_;
  }
  if (older_reference) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝乱序 B 样条参考：start_time 早于当前已接受轨迹");
    return;
  }
  usv_nmpc::BsplineReference evaluator;
  std::string error;
  usv_nmpc::ReferenceTrajectory next;
  const double elapsed_at_receipt = std::max(0.0, (received_time - message_start).seconds());
  usv_nmpc::RefPoint reference_at_receipt;
  if (!evaluator.configure(*msg, error) ||
    !evaluator.sample(elapsed_at_receipt, reference_at_receipt))
  {
    RCLCPP_WARN(get_logger(), "拒绝 USV B 样条参考：%s", error.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool state_fresh = has_state_ && state_.initialized &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
      last_state_received_).count() < state_timeout_s_;
    if (!state_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝 USV B 样条参考：没有新鲜的 /odom 状态，无法核对起点距离");
      return;
    }
    // 障碍变化可使局部规划结果跳变，因此只校验新样条当前参考点与船位的距离，
    // 不要求新旧样条连续。
    const double reference_distance = std::hypot(
      reference_at_receipt.x - state_.x, reference_at_receipt.y - state_.y);
    if (!std::isfinite(reference_distance) ||
      reference_distance > max_reference_start_distance_m_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "拒绝 USV B 样条参考：当前时刻参考点与船位偏差 %.3f m，阈值 %.3f m",
        reference_distance, max_reference_start_distance_m_);
      return;
    }
  }
  if (!evaluator.buildTrajectory(bspline_sample_period_s_, next, error)) {
    RCLCPP_WARN(get_logger(), "拒绝 USV B 样条参考：%s", error.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  next.mission_id = 0U;
  const double active_reference_age = (received_time - last_trajectory_time_).seconds();
  const bool active_reference_running = has_trajectory_ &&
    received_time >= trajectory_start_time_ && tracking_fsm_.canSolve() &&
    active_reference_age >= 0.0 && active_reference_age <= reference_stale_timeout_s_;
  const bool queue_for_start = message_start > received_time && active_reference_running;
  bool pending_kept = false;
  if (queue_for_start) {
    // 新切片的绝对起始时刻到达前继续使用当前参考。
    // 多条待生效消息中保留最早的一条，避免滚动更新不断推迟切换时刻。
    if (!has_pending_trajectory_ || message_start <= pending_trajectory_start_time_) {
      pending_trajectory_start_time_ = message_start;
      pending_bspline_evaluator_ = std::move(evaluator);
      pending_trajectory_ = std::move(next);
      has_pending_trajectory_ = true;
      last_bspline_ = *msg;
      has_last_bspline_ = true;
    } else {
      pending_kept = true;
    }
  } else {
    trajectory_start_time_ = message_start;
    bspline_evaluator_ = std::move(evaluator);
    trajectory_ = std::move(next);
    has_trajectory_ = true;
    has_pending_trajectory_ = false;
    ++trajectory_revision_;
    last_bspline_ = *msg;
    has_last_bspline_ = true;
  }
  last_trajectory_time_ = received_time;
  tracking_fsm_.referenceAccepted(trackingInputs(received_time));
  publishTrackingState();
  const auto & reported_trajectory = has_pending_trajectory_ ?
    pending_trajectory_ : trajectory_;
  // 滚动参考不重置控制器；未来切片待其绝对起始时刻到达后再接管。
  RCLCPP_INFO(get_logger(),
    "已接收 B 样条 traj_id=%lld（%s），阶数=%d，状态控制点=%zu，末端速度=%.3f m/s，末端转艏率=%.3f rad/s，诊断采样点=%zu，时长=%.2f s",
    static_cast<long long>(msg->traj_id),
    queue_for_start ? (pending_kept ? "保留更早的待切换片段" : "待绝对起始时刻切换") : "立即生效",
    msg->order, msg->control_points.size(),
    reported_trajectory.points.back().speed, reported_trajectory.points.back().yaw_rate,
    reported_trajectory.points.size(), reported_trajectory.points.back().t);
}

void UsvNmpcNode::missionResetCallback(
  const std_msgs::msg::UInt64::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  has_trajectory_ = false;
  has_pending_trajectory_ = false;
  has_last_bspline_ = false;
  trajectory_ = {};
  pending_trajectory_ = {};
  trajectory_.mission_id = msg->data;
  terminal_mission_id_ = msg->data;
  has_reference_build_time_ = false;
  last_trajectory_time_ = now();
  accept_reference_after_ = last_trajectory_time_;
  nmpc_->resetWarmStart();
  tracking_fsm_.reset(trackingInputs(last_trajectory_time_));
  publishTrackingState();
  publishStopCommand();
  RCLCPP_INFO(get_logger(), "收到任务重置 mission_id=%llu，等待新的 B 样条参考",
    static_cast<unsigned long long>(msg->data));
}

void UsvNmpcNode::publishCommand(const usv_nmpc::Control & control)
{
  if (!std::isfinite(control.speed) || !std::isfinite(control.yaw_rate)) {
    tracking_fsm_.stop(usv_nmpc::StopReason::COMMAND_INVALID);
    publishStopCommand();
    return;
  }
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
    last_state_received_).count() >= state_timeout_s_)
  {
    tracking_fsm_.stop(usv_nmpc::StopReason::ODOM_TIMEOUT);
    publishStopCommand();
    return;
  }
  const rclcpp::Time command_time = now();
  const double command_dt = has_published_command_ ?
    std::max(0.0, (command_time - last_command_time_).seconds()) :
    1.0 / control_frequency_;
  geometry_msgs::msg::Twist command;
  const usv_nmpc::Control previous{
    has_published_command_ ? last_published_speed_ : 0.0,
    has_published_command_ ? last_published_yaw_rate_ : 0.0};
  const auto limited = usv_nmpc::limitCommandChange(control, previous, command_dt, nmpc_config_);
  command.linear.x = limited.speed;
  command.angular.z = limited.yaw_rate;
  command_pub_->publish(command);
  has_published_command_ = true;
  last_command_dt_s_ = command_dt;
  last_command_time_ = command_time;
  last_published_speed_ = command.linear.x;
  last_published_yaw_rate_ = command.angular.z;
  nmpc_->setLastCommand({command.linear.x, command.angular.z});
}

void UsvNmpcNode::publishStopCommand()
{
  // 完成或保护停车当周期直接归零，不经过正常指令的变化率限制。
  geometry_msgs::msg::Twist command;
  command_pub_->publish(command);
  has_published_command_ = true;
  last_published_speed_ = 0.0;
  last_published_yaw_rate_ = 0.0;
  last_command_dt_s_ = 0.0;
  last_command_time_ = now();
  nmpc_->setLastCommand({});
  if (has_published_prediction_) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = now();
    geometry_msgs::msg::PoseArray poses;
    poses.header = path.header;
    predicted_path_pub_->publish(path);
    predicted_poses_pub_->publish(poses);
    has_published_prediction_ = false;
  }
}

void UsvNmpcNode::printTrackingTable(
  const usv_nmpc::State & state, const usv_nmpc::RefPoint & reference,
  double progress_s, double total_distance)
{
  if (!terminal_table_enabled_) {
    return;
  }
  if (!(terminal_table_refresh_period_s_ > 0.0)) {
    return;
  }
  const auto current_time = std::chrono::steady_clock::now();
  if (has_terminal_table_printed_ &&
    std::chrono::duration<double>(current_time - last_terminal_table_print_).count() <
    terminal_table_refresh_period_s_)
  {
    return;
  }
  has_terminal_table_printed_ = true;
  last_terminal_table_print_ = current_time;

  const double progress = total_distance > 1e-9 ?
    100.0 * std::clamp(progress_s / total_distance, 0.0, 1.0) : 100.0;
  const double degrees = 180.0 / usv_nmpc::PI;
  const double heading_error = usv_nmpc::wrapAngle(reference.psi - state.psi) * degrees;
  std::ostringstream table;
  table << std::fixed << std::setprecision(3);
  table << "\nNMPC " << usv_nmpc::trackingStateName(tracking_fsm_.state())
        << " | spline " << std::setprecision(1) << progress_s << "/" << total_distance
        << " m (" << progress << "%)\n"
        << "+-------+------------+------------+------------+-------+\n"
        << "| state | actual     | reference  | error      | unit  |\n"
        << "+-------+------------+------------+------------+-------+\n";
  auto row = [&table](const char * name, double actual, double desired, double error,
    const char * unit) {
      table << "| " << std::left << std::setw(5) << name << " | " << std::right
            << std::setw(10) << actual << " | " << std::setw(10) << desired << " | "
            << std::setw(10) << error << " | " << std::left << std::setw(5) << unit
            << " |\n";
    };
  table << std::setprecision(3);
  row("x", state.x, reference.x, state.x - reference.x, "m");
  row("y", state.y, reference.y, state.y - reference.y, "m");
  row("psi", usv_nmpc::wrapAngle(state.psi) * degrees,
    usv_nmpc::wrapAngle(reference.psi) * degrees, heading_error, "deg");
  row("u", state.u, reference.u, state.u - reference.u, "m/s");
  row("v", state.v, reference.v, state.v - reference.v, "m/s");
  row("r", state.r, reference.r, state.r - reference.r, "rad/s");
  table << "+-------+------------+------------+------------+-------+\n"
        << "cmd: V_cmd=" << last_published_speed_ << " m/s, W_cmd="
        << last_published_yaw_rate_ << " rad/s\n";
  const auto rendered = table.str();
  if (isatty(STDOUT_FILENO)) {
    std::cout << "\033[2J\033[H" << rendered << std::flush;
  } else {
    RCLCPP_INFO(get_logger(), "%s", rendered.c_str());
  }
}

void UsvNmpcNode::publishPredictedTrajectory()
{
  // Path 显示预测轨迹线，PoseArray 显示各点的船首方向。
  // 求解失败或停车时已清空预测，不重复发布失效结果。
  const auto & predicted = nmpc_->predictedTrajectory();
  if (!tracking_fsm_.canSolve() || predicted.empty() || nmpc_->lastSolveFailed() ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() -
    last_state_received_).count() >= state_timeout_s_)
  {
    return;
  }
  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = stamp;
  geometry_msgs::msg::PoseArray poses;
  poses.header = path.header;
  path.poses.reserve(predicted.size());
  poses.poses.reserve(predicted.size());
  for (const auto & s : predicted) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = s.x;
    ps.pose.position.y = s.y;
    ps.pose.position.z = 0.0;
    // nav_msgs/Pose 的标准平面朝向以本体 +x 为前；换算成 ENU yaw = 航向 + pi/2。
    const double map_yaw = s.psi + usv_nmpc::coordinates::kPi * 0.5;
    ps.pose.orientation.z = std::sin(0.5 * map_yaw);
    ps.pose.orientation.w = std::cos(0.5 * map_yaw);
    path.poses.push_back(ps);
    poses.poses.push_back(ps.pose);
  }
  predicted_path_pub_->publish(path);
  predicted_poses_pub_->publish(poses);
  has_published_prediction_ = true;
}

void UsvNmpcNode::publishFinished(bool finished)
{
  std_msgs::msg::Bool message;
  message.data = finished;
  finished_pub_->publish(message);
}

void UsvNmpcNode::publishTrackingDiagnostics(
  std::uint64_t mission_id, bool terminal_hold, double progress_s,
  std::uint64_t trajectory_revision, const usv_nmpc::State & state,
  const usv_nmpc::RefPoint & reference, double endpoint_distance,
  double heading_error, bool inside_finish_area,
  double arrival_duration_s) const
{
  // 诊断字段采用固定下标布局，扩展字段从末尾追加。
  const usv_nmpc::Control feedforward =
    nmpc_->feedforwardForReference(reference);
  const auto & cost = nmpc_->lastCostBreakdown();
  const auto predicted_state = nmpc_->predictedNextState();
  const auto predicted_command = nmpc_->predictedFirstCommand();
  const auto frame_error = usv_nmpc::pathFrameError(state, reference);
  const double course_error = usv_nmpc::referenceHeadingError(state, reference);
  std_msgs::msg::Float64MultiArray message;
  message.data = {
    1.0,
    static_cast<double>(mission_id),
    terminal_hold ? 1.0 : 0.0,
    progress_s,
    reference.x,
    reference.y,
    reference.psi,
    reference.u,
    reference.v,
    reference.r,
    reference.curvature,
    feedforward.speed,
    feedforward.yaw_rate,
    cost.solver_succeeded ? 1.0 : 0.0,
    cost.total,
    static_cast<double>(trajectory_revision),
    state.x, state.y, state.psi, state.u, state.v, state.r,
    predicted_state.x, predicted_state.y, predicted_state.psi,
    predicted_state.u, predicted_state.v, predicted_state.r,
    frame_error.along, frame_error.lateral, course_error,
    state.u - reference.u, state.v - reference.v, state.r - reference.r,
    // d34、d40、d42-d47 为保留槽位，固定为零以保持诊断字段下标稳定。
    0.0, cost.stage_lateral, cost.stage_course,
    cost.stage_u, cost.stage_v, cost.stage_r, 0.0,
    cost.stage_delta_control, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    static_cast<double>(cost.consecutive_failures),
    predicted_command.speed, predicted_command.yaw_rate,
    // 追加诊断字段：51 为参考交接标志，52/53 为实测/参考低速平滑侧滑角（rad）。
    0.0,
    usv_nmpc::courseSideslipAngle(state.u, state.v),
    usv_nmpc::courseSideslipAngle(reference.u, reference.v),
    endpoint_distance,
    heading_error,
    inside_finish_area ? 1.0 : 0.0,
    (tracking_fsm_.confirmingArrival() || tracking_fsm_.finished()) ? 1.0 : 0.0,
    arrival_duration_s,
    // 诊断数组的 59、60 位为零值保留字段，后续指令字段下标保持固定。
    0.0,
    0.0,
    // 后续字段依次为实际限变化率指令、限幅周期和控制器时钟时间戳。
    last_published_speed_,
    last_published_yaw_rate_,
    last_command_dt_s_,
    now().seconds(),
    0.0, 0.0,
    static_cast<double>(tracking_fsm_.state()),
    static_cast<double>(tracking_fsm_.reason())};
  diagnostics_pub_->publish(message);
}

usv_nmpc::TrackingInputs UsvNmpcNode::trackingInputs(const rclcpp::Time & time) const
{
  // 调用方已持有 mutex_；本函数只读取状态，不执行求解或发布操作。
  const double reference_age = (time - last_trajectory_time_).seconds();
  return {
    has_state_ && state_.initialized,
    std::chrono::duration<double>(std::chrono::steady_clock::now() -
      last_state_received_).count() < state_timeout_s_,
    has_trajectory_,
    reference_age >= 0.0 && reference_age <= reference_stale_timeout_s_,
    time >= trajectory_start_time_};
}

void UsvNmpcNode::publishTrackingState()
{
  const auto state = tracking_fsm_.state();
  const auto reason = tracking_fsm_.reason();
  if (has_published_tracking_state_ && state == published_tracking_state_ &&
    reason == published_stop_reason_)
  {
    return;
  }
  std_msgs::msg::String message;
  message.data = usv_nmpc::trackingStateName(state);
  if (reason != usv_nmpc::StopReason::NONE) {
    message.data += std::string(":") + usv_nmpc::stopReasonName(reason);
  }
  tracking_state_pub_->publish(message);
  publishFinished(tracking_fsm_.finished());
  const char * previous = has_published_tracking_state_ ?
    usv_nmpc::trackingStateName(published_tracking_state_) : "INIT";
  RCLCPP_INFO(get_logger(), "跟踪状态 %s -> %s (%s)", previous, message.data.c_str(),
    usv_nmpc::trackingStateDescription(state));
  published_tracking_state_ = state;
  published_stop_reason_ = reason;
  has_published_tracking_state_ = true;
}

void UsvNmpcNode::timerCallback()
{
  const rclcpp::Time cycle_time = now();
  UsvState state;
  usv_nmpc::ReferenceTrajectory trajectory;
  usv_nmpc::BsplineReference evaluator;
  rclcpp::Time trajectory_start_time(0, 0, RCL_ROS_TIME);
  bool clock_rewound = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 先检查时钟是否回拨，再检查数据新鲜度和未来起始时刻，
    // 避免旧时钟周期的样条使控制器一直处于等待状态。
    clock_rewound = has_control_tick_time_ && cycle_time < last_control_tick_time_;
    if (clock_rewound) {
      has_state_ = false;
      state_ = {};
      has_trajectory_ = false;
      has_pending_trajectory_ = false;
      has_last_bspline_ = false;
      trajectory_ = {};
      pending_trajectory_ = {};
      accept_reference_after_ = cycle_time;
      has_reference_build_time_ = false;
      nmpc_->resetWarmStart();
      // 时钟回拨后仍保持已完成任务的锁存状态。
      tracking_fsm_.stop(usv_nmpc::StopReason::CLOCK_REWIND);
    } else {
      if (has_pending_trajectory_ && cycle_time >= pending_trajectory_start_time_) {
        trajectory_start_time_ = pending_trajectory_start_time_;
        trajectory_ = std::move(pending_trajectory_);
        bspline_evaluator_ = std::move(pending_bspline_evaluator_);
        has_pending_trajectory_ = false;
        ++trajectory_revision_;
        RCLCPP_INFO(get_logger(),
          "切换到已到绝对起始时刻的 B 样条 traj_id=%lld",
          static_cast<long long>(last_bspline_.traj_id));
      }
      tracking_fsm_.updateInputs(trackingInputs(cycle_time));
    }
    has_control_tick_time_ = true;
    last_control_tick_time_ = cycle_time;
    if (tracking_fsm_.canSolve()) {
      state = state_;
      trajectory = trajectory_;
      evaluator = bspline_evaluator_;
      trajectory_start_time = trajectory_start_time_;
    }
  }

  switch (tracking_fsm_.state()) {
    case usv_nmpc::TrackingState::TRACKING:
    case usv_nmpc::TrackingState::ARRIVAL_CONFIRM:
      executeTrackingCycle(cycle_time, state, trajectory, evaluator, trajectory_start_time);
      break;
    case usv_nmpc::TrackingState::WAIT_ODOM:
    case usv_nmpc::TrackingState::WAIT_REFERENCE:
    case usv_nmpc::TrackingState::WAIT_START:
    case usv_nmpc::TrackingState::SAFE_STOP:
    case usv_nmpc::TrackingState::FINISHED:
      publishStopCommand();
      break;
  }
  publishTrackingState();
}

void UsvNmpcNode::executeTrackingCycle(
  const rclcpp::Time & cycle_time, const UsvState & state,
  const usv_nmpc::ReferenceTrajectory & trajectory,
  const usv_nmpc::BsplineReference & evaluator,
  const rclcpp::Time & trajectory_start_time)
{
  double control_update_dt = 1.0 / control_frequency_;
  if (has_reference_build_time_) {
    const double elapsed = (cycle_time - last_reference_build_time_).seconds();
    // ROS 时间暂停时不重复求解，也不推进到达确认计时。
    if (elapsed <= 0.0) {return;}
    control_update_dt = elapsed;
  }
  last_reference_build_time_ = cycle_time;
  has_reference_build_time_ = true;
  const double trajectory_elapsed_s = (cycle_time - trajectory_start_time).seconds();
  std::vector<usv_nmpc::RefPoint> references;
  if (trajectory.points.empty() || !evaluator.buildHorizon(
      trajectory_elapsed_s, nmpc_config_.horizon_steps, nmpc_config_.dt, references))
  {
    tracking_fsm_.stop(usv_nmpc::StopReason::HORIZON_INVALID);
    publishStopCommand();
    return;
  }

  const usv_nmpc::State current{state.x, state.y, state.yaw, state.u, state.v, state.r};
  const auto & endpoint = trajectory.points.back();
  const auto & reference = references.front();
  const double progress_s = progressAtTime(trajectory, trajectory_elapsed_s);
  const double reference_distance = std::hypot(reference.x - endpoint.x, reference.y - endpoint.y);
  // 根据样条末端导数判断其是否静止，无需在消息中增加标志位。
  const bool terminal_hold = trajectory.terminal &&
    trajectory_elapsed_s >= endpoint.t - 1e-9 && reference_distance <= 0.05;
  const double endpoint_distance = std::hypot(state.x - endpoint.x, state.y - endpoint.y);
  const double heading_error = std::abs(
    coordinates::shortestAngularDistance(endpoint.psi, state.yaw));
  const bool inside_finish_area = terminal_hold && endpoint_distance <= finish_distance_m_;
  tracking_fsm_.updateArrival(
    inside_finish_area && std::hypot(state.u, state.v) <= finish_speed_mps_,
    cycle_time.seconds(), finish_stable_duration_s_);
  if (tracking_fsm_.finished()) {
    publishStopCommand();
    publishTrackingDiagnostics(
      trajectory.mission_id, terminal_hold, progress_s, trajectory_revision_, current,
      reference, endpoint_distance, heading_error, inside_finish_area,
      tracking_fsm_.arrivalDuration(cycle_time.seconds()));
    RCLCPP_INFO(get_logger(),
      "已满足终点距离与速度条件：位置误差 %.2f m，合速度 %.2f m/s，连续确认 %.2f s",
      endpoint_distance, std::hypot(state.u, state.v),
      tracking_fsm_.arrivalDuration(cycle_time.seconds()));
    return;
  }

  try {
    // 滚动参考切换时保留真实控制周期，避免重置指令变化率约束。
    const auto control = nmpc_->solve(current, references, std::max(1e-3, control_update_dt));
    if (nmpc_->lastSolveFailed()) {
      tracking_fsm_.stop(usv_nmpc::StopReason::SOLVER_FAILED);
      publishStopCommand();
    } else {
      publishCommand(control);
    }
    publishPredictedTrajectory();
    if (tracking_fsm_.canSolve()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id_;
      pose.header.stamp = cycle_time;
      pose.pose.position.x = reference.x;
      pose.pose.position.y = reference.y;
      const double map_yaw = reference.psi + usv_nmpc::coordinates::kPi * 0.5;
      pose.pose.orientation.z = std::sin(0.5 * map_yaw);
      pose.pose.orientation.w = std::cos(0.5 * map_yaw);
      reference_pose_pub_->publish(pose);
    }
    publishTrackingDiagnostics(
      trajectory.mission_id, terminal_hold, progress_s, trajectory_revision_, current,
      reference, endpoint_distance, heading_error, inside_finish_area,
      tracking_fsm_.arrivalDuration(cycle_time.seconds()));
    if (!nmpc_->lastSolveFailed() && tracking_fsm_.canSolve()) {
      printTrackingTable(current, reference, progress_s, endpoint.s);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "NMPC 求解异常：%s", error.what());
    tracking_fsm_.stop(usv_nmpc::StopReason::SOLVER_FAILED);
    publishStopCommand();
  }
}
