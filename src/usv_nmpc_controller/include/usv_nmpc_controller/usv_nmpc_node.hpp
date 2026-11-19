/**
 * @file usv_nmpc_node.hpp
 * @brief 无人船 NMPC ROS 节点声明。
 */

#pragma once

#include <chrono>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "usv_interfaces/msg/usv_bspline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "usv_nmpc_controller/bspline_reference.hpp"
#include "usv_nmpc_controller/common_types.hpp"
#include "usv_nmpc_controller/nmpc_controller.hpp"
#include "usv_nmpc_controller/tracking_state_machine.hpp"

/** @brief 无人船 NMPC ROS 节点。 */
class UsvNmpcNode final : public rclcpp::Node
{
public:
  /** @brief 读取配置，创建控制器、ROS 接口及周期控制定时器。 */
  UsvNmpcNode();

private:
  using Bspline = usv_interfaces::msg::UsvBspline;
  struct UsvState
  {
    double yaw{0.0};
    double u{0.0};
    double v{0.0};
    double r{0.0};
    double x{0.0};
    double y{0.0};
    bool initialized{false};
  };
  /** @brief 加载并校验节点参数；无效配置通过异常终止初始化。 */
  void loadParameters();
  /** @brief 读取控制器参数并构造 NMPC 配置。 */
  usv_nmpc::NMPCConfig loadNmpcConfig();
  /** @brief 创建订阅、发布及任务复位服务。 */
  void setupRosInterfaces();
  /** @brief 接收 ENU 位姿及 RFU 本体速度，更新带接收时刻的状态缓存。 */
  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  /** @brief 校验局部样条并更新参考；未来起始的有效样条暂存至起始时刻。 */
  void bsplineCallback(const Bspline::SharedPtr msg);
  /** @brief 规划器撤销失效参考时立即停车，等待撤销后的新参考。 */
  void replanFailedCallback(const std_msgs::msg::Header::SharedPtr msg);
  /** @brief 按任务标识复位跟踪状态，清除当前参考与求解热启动。 */
  void missionResetCallback(const std_msgs::msg::UInt64::SharedPtr msg);
  /** @brief 执行状态机流转，在输入有效时求解控制量，否则发布停车指令。 */
  void timerCallback();
  /** @brief 按指定 ROS 时刻汇总状态、参考新鲜度及起始条件。 */
  usv_nmpc::TrackingInputs trackingInputs(const rclcpp::Time & time) const;
  /** @brief 状态或停车原因变化时发布跟踪状态。 */
  void publishTrackingState();
  /** @brief 构造预测参考并求解单个跟踪周期，更新到达确认与诊断输出。 */
  void executeTrackingCycle(
    const rclcpp::Time & cycle_time, const UsvState & state,
    const usv_nmpc::ReferenceTrajectory & trajectory,
    const usv_nmpc::BsplineReference & evaluator,
    const rclcpp::Time & trajectory_start_time);
  /** @brief 对指令执行最终限幅并发布，记录实际发送值供下一周期使用。 */
  void publishCommand(const usv_nmpc::Control & control);
  /** @brief 发布零值停车指令。 */
  void publishStopCommand();
  /** @brief 按配置刷新终端跟踪表，显示状态、参考及误差。 */
  void printTrackingTable(
    const usv_nmpc::State & state, const usv_nmpc::RefPoint & reference,
    double progress_s, double total_distance);
  /**
   * @brief 发布预测轨迹线及位姿数组，供 RViz 显示位置和船首方向。
   */
  void publishPredictedTrajectory();
  /** @brief 发布当前任务的完成标志。 */
  void publishFinished(bool finished);
  /** @brief 按固定字段顺序发布参考、误差、约束及求解状态诊断。 */
  void publishTrackingDiagnostics(
    std::uint64_t mission_id, bool terminal_hold, double progress_s,
    std::uint64_t trajectory_revision, const usv_nmpc::State & state,
    const usv_nmpc::RefPoint & reference, double endpoint_distance,
    double heading_error, bool inside_finish_area,
    double arrival_duration_s) const;

  std::string odom_topic_{"/odom"};
  std::string bspline_topic_{"/planning/bspline"};
  std::string cmd_topic_{"/cmd"};
  std::string tracking_finished_topic_{"/USV_tracking_finished"};
  std::string nmpc_diagnostics_topic_{"/nmpc_tracking_diagnostics"};
  /** @brief NMPC 预测轨迹话题，包含位置和航向四元数。 */
  std::string predicted_path_topic_{"/nmpc_predicted_path"};
  /** @brief 预测位姿数组话题，供 RViz 显示各点船首方向。 */
  std::string predicted_poses_topic_{"/nmpc_predicted_poses"};
  bool has_published_prediction_{false};
  /** 预测轨迹与参考量的坐标系。 */
  std::string frame_id_{"map"};
  /** 用于终点诊断和跟踪表的样条离散周期(s)；NMPC 预测直接求值原样条。 */
  double bspline_sample_period_s_{0.1};
  /** 最近一次收到参考消息的时刻，用于诊断和复位处理。 */
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};
  /** B 样条消息指定的绝对起始时刻；零时间戳按接收时刻处理。 */
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time accept_reference_after_{0, 0, RCL_ROS_TIME};
  double control_frequency_{10.0};
  /** 本船到样条终点的平面距离容差(m)。 */
  double finish_distance_m_{2.0};
  /** 终点允许的合速度上限(m/s)。 */
  double finish_speed_mps_{0.2};
  /** 终点距离和合速度条件连续满足的确认时间(s)。 */
  double finish_stable_duration_s_{1.0};
  bool terminal_table_enabled_{true};
  double terminal_table_refresh_period_s_{1.0};
  bool has_terminal_table_printed_{false};
  std::chrono::steady_clock::time_point last_terminal_table_print_;

  usv_nmpc::NMPCConfig nmpc_config_;
  std::unique_ptr<usv_nmpc::NMPCController> nmpc_;
  usv_nmpc::BsplineReference bspline_evaluator_;
  usv_nmpc::BsplineReference pending_bspline_evaluator_;
  UsvState state_;
  usv_nmpc::ReferenceTrajectory trajectory_;
  usv_nmpc::ReferenceTrajectory pending_trajectory_;
  rclcpp::Time pending_trajectory_start_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_state_received_;
  double state_timeout_s_{0.5};
  double reference_stale_timeout_s_{2.0};
  double max_reference_start_distance_m_{2.0};
  bool has_state_{false};
  bool has_trajectory_{false};
  bool has_pending_trajectory_{false};
  usv_nmpc::TrackingStateMachine tracking_fsm_;
  usv_nmpc::TrackingState published_tracking_state_{usv_nmpc::TrackingState::WAIT_ODOM};
  usv_nmpc::StopReason published_stop_reason_{usv_nmpc::StopReason::NONE};
  bool has_published_tracking_state_{false};
  bool has_last_bspline_{false};
  Bspline last_bspline_;
  bool has_published_command_{false};
  double last_published_speed_{0.0};
  double last_published_yaw_rate_{0.0};
  double last_command_dt_s_{0.0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  bool has_reference_build_time_{false};
  rclcpp::Time last_reference_build_time_{0, 0, RCL_ROS_TIME};
  bool has_control_tick_time_{false};
  rclcpp::Time last_control_tick_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t terminal_mission_id_{0U};
  std::uint64_t trajectory_revision_{0U};
  mutable std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr replan_failed_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr mission_reset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr predicted_poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
