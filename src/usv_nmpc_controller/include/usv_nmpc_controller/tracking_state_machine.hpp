/**
 * @file tracking_state_machine.hpp
 * @brief 局部轨迹跟踪状态机，与 ROS 接口和数值求解器解耦。
 */
#pragma once

#include <cstdint>
#include <optional>

namespace usv_nmpc
{
/** @brief 控制器生命周期状态。 */
enum class TrackingState : std::uint8_t
{
  WAIT_ODOM = 0,
  WAIT_REFERENCE = 1,
  WAIT_START = 2,
  TRACKING = 3,
  ARRIVAL_CONFIRM = 4,
  SAFE_STOP = 5,
  FINISHED = 6
};

/** @brief 安全停车原因；用于状态发布和诊断。 */
enum class StopReason : std::uint8_t
{
  NONE = 0,
  ODOM_TIMEOUT = 1,
  REFERENCE_TIMEOUT = 2,
  CLOCK_REWIND = 3,
  HORIZON_INVALID = 4,
  SOLVER_FAILED = 5,
  COMMAND_INVALID = 6
};

/** @brief 单个控制周期的状态、参考有效性和起始时间条件。 */
struct TrackingInputs
{
  bool odom_valid{false};
  bool odom_fresh{false};
  bool has_reference{false};
  bool reference_fresh{false};
  bool reference_started{false};
};

/**
 * @brief 管理等待、跟踪、到达确认、安全停车和完成状态。
 * @details 节点在每个控制周期提供输入有效性；仅 TRACKING 和 ARRIVAL_CONFIRM
 * 状态允许求解。FINISHED 状态保持锁存，直到接受新样条或显式复位。
 * 被拒绝或重复的样条不得触发 referenceAccepted。
 */
class TrackingStateMachine
{
public:
  /** @brief 返回当前状态。 */
  TrackingState state() const {return state_;}
  /** @brief 返回当前停车原因；无停车原因时返回 NONE。 */
  StopReason reason() const {return reason_;}
  /** @brief 当前状态是否允许执行 NMPC 求解。 */
  bool canSolve() const;
  /** @brief 当前是否正在累计到达确认时间。 */
  bool confirmingArrival() const {return state_ == TrackingState::ARRIVAL_CONFIRM;}
  /** @brief 任务是否已完成。 */
  bool finished() const {return state_ == TrackingState::FINISHED;}
  /**
   * @brief 根据本周期输入有效性更新状态；完成状态保持锁存。
   * @param[in] inputs 本周期状态和参考的有效性。
   * @return 更新后的状态允许求解时返回 true。
   */
  bool updateInputs(const TrackingInputs & inputs);
  /**
   * @brief 登记已接受的参考，清除到达计时并解除完成锁存。
   * @param[in] inputs 接受参考后的输入有效性。
   */
  void referenceAccepted(const TrackingInputs & inputs);
  /**
   * @brief 清除到达计时并按当前输入重新进入等待或跟踪状态。
   * @param[in] inputs 复位后的输入有效性。
   */
  void reset(const TrackingInputs & inputs);
  /**
   * @brief 进入安全停车状态，清除到达计时并记录原因。
   * @param[in] reason 本次停车原因。
   */
  void stop(StopReason reason);
  /**
   * @brief 更新终点到达确认计时。
   * @param[in] conditions_met 本周期的完成条件是否全部满足。
   * @param[in] now_s 单调递增的当前时间，单位 s。
   * @param[in] stable_duration_s 条件需持续满足的时长，单位 s。
   */
  void updateArrival(bool conditions_met, double now_s, double stable_duration_s);
  /**
   * @brief 计算连续满足到达条件的时长。
   * @param[in] now_s 当前时刻，须与 updateArrival 使用相同时间基准，单位 s。
   * @return 持续时长，单位 s；尚未开始确认或时刻早于起点时返回 0。
   */
  double arrivalDuration(double now_s) const;

private:
  void transition(TrackingState next, StopReason reason = StopReason::NONE);
  TrackingState state_{TrackingState::WAIT_ODOM};
  StopReason reason_{StopReason::NONE};
  std::optional<double> arrival_since_s_;
};

/** @brief 返回跟踪状态的名称；未知枚举值返回 UNKNOWN。 */
const char * trackingStateName(TrackingState state);
/** @brief 返回跟踪状态的中文说明。 */
const char * trackingStateDescription(TrackingState state);
/** @brief 返回停车原因的名称。 */
const char * stopReasonName(StopReason reason);
}  // namespace usv_nmpc
