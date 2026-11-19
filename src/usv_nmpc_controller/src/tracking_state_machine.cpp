/**
 * @file tracking_state_machine.cpp
 * @brief 局部 B 样条跟踪状态机及到达确认逻辑。
 */
#include "usv_nmpc_controller/tracking_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace usv_nmpc
{
bool TrackingStateMachine::canSolve() const
{
  return state_ == TrackingState::TRACKING || state_ == TrackingState::ARRIVAL_CONFIRM;
}

void TrackingStateMachine::transition(TrackingState next, StopReason reason)
{
  state_ = next;
  reason_ = reason;
  if (next != TrackingState::ARRIVAL_CONFIRM && next != TrackingState::FINISHED) {
    arrival_since_s_.reset();
  }
}

bool TrackingStateMachine::updateInputs(const TrackingInputs & inputs)
{
  if (finished()) {return false;}
  if (!inputs.odom_valid) {
    transition(TrackingState::WAIT_ODOM);
  } else if (!inputs.odom_fresh) {
    stop(StopReason::ODOM_TIMEOUT);
  } else if (!inputs.has_reference) {
    transition(TrackingState::WAIT_REFERENCE);
  } else if (!inputs.reference_fresh) {
    stop(StopReason::REFERENCE_TIMEOUT);
  } else if (!inputs.reference_started) {
    transition(TrackingState::WAIT_START);
  } else if (!confirmingArrival()) {
    transition(TrackingState::TRACKING);
  }
  return canSolve();
}

void TrackingStateMachine::referenceAccepted(const TrackingInputs & inputs)
{
  // 替换滚动局部段仍处于 TRACKING，不重置求解器状态。
  transition(TrackingState::TRACKING);
  updateInputs(inputs);
}

void TrackingStateMachine::reset(const TrackingInputs & inputs)
{
  transition(TrackingState::WAIT_ODOM);
  updateInputs(inputs);
}

void TrackingStateMachine::stop(StopReason reason)
{
  if (!finished()) {transition(TrackingState::SAFE_STOP, reason);}
}

void TrackingStateMachine::updateArrival(
  bool conditions_met, double now_s, double stable_duration_s)
{
  if (!canSolve()) {return;}
  if (!conditions_met || !std::isfinite(now_s) ||
    !std::isfinite(stable_duration_s) || stable_duration_s <= 0.0)
  {
    transition(TrackingState::TRACKING);
    return;
  }
  if (!arrival_since_s_ || now_s < *arrival_since_s_) {
    arrival_since_s_ = now_s;
  }
  transition(arrivalDuration(now_s) >= stable_duration_s ?
    TrackingState::FINISHED : TrackingState::ARRIVAL_CONFIRM);
}

double TrackingStateMachine::arrivalDuration(double now_s) const
{
  return arrival_since_s_ ? std::max(0.0, now_s - *arrival_since_s_) : 0.0;
}

const char * trackingStateName(TrackingState state)
{
  switch (state) {
    case TrackingState::WAIT_ODOM: return "WAIT_ODOM";
    case TrackingState::WAIT_REFERENCE: return "WAIT_REFERENCE";
    case TrackingState::WAIT_START: return "WAIT_START";
    case TrackingState::TRACKING: return "TRACKING";
    case TrackingState::ARRIVAL_CONFIRM: return "ARRIVAL_CONFIRM";
    case TrackingState::SAFE_STOP: return "SAFE_STOP";
    case TrackingState::FINISHED: return "FINISHED";
  }
  return "UNKNOWN";
}

const char * trackingStateDescription(TrackingState state)
{
  switch (state) {
    case TrackingState::WAIT_ODOM: return "等待有效 /odom";
    case TrackingState::WAIT_REFERENCE: return "等待有效 B 样条参考";
    case TrackingState::WAIT_START: return "等待样条绝对起始时刻";
    case TrackingState::TRACKING: return "按样条时间跟踪";
    case TrackingState::ARRIVAL_CONFIRM: return "终点距离与合速度连续确认";
    case TrackingState::SAFE_STOP: return "保护停车，等待输入恢复后重试";
    case TrackingState::FINISHED: return "任务已完成，保持零指令";
  }
  return "未知状态";
}

const char * stopReasonName(StopReason reason)
{
  switch (reason) {
    case StopReason::NONE: return "NONE";
    case StopReason::ODOM_TIMEOUT: return "ODOM_TIMEOUT";
    case StopReason::REFERENCE_TIMEOUT: return "REFERENCE_TIMEOUT";
    case StopReason::CLOCK_REWIND: return "CLOCK_REWIND";
    case StopReason::HORIZON_INVALID: return "HORIZON_INVALID";
    case StopReason::SOLVER_FAILED: return "SOLVER_FAILED";
    case StopReason::COMMAND_INVALID: return "COMMAND_INVALID";
  }
  return "UNKNOWN";
}
}  // namespace usv_nmpc
