/**
 * @file tracking_state_machine_test.cpp
 * @brief 不依赖 ROS 时间和数值求解器的状态机生命周期测试。
 */
#include <limits>

#include "gtest/gtest.h"
#include "usv_nmpc_controller/tracking_state_machine.hpp"

namespace usv_nmpc
{
namespace
{
TrackingInputs ready() {return {true, true, true, true, true};}

TEST(TrackingStateMachine, StartupWaitsForValidInputsInPriorityOrder)
{
  TrackingStateMachine fsm;
  EXPECT_EQ(fsm.state(), TrackingState::WAIT_ODOM);
  EXPECT_FALSE(fsm.updateInputs({}));
  EXPECT_FALSE(fsm.updateInputs({true, true, false, false, false}));
  EXPECT_EQ(fsm.state(), TrackingState::WAIT_REFERENCE);
  EXPECT_FALSE(fsm.updateInputs({true, true, true, true, false}));
  EXPECT_EQ(fsm.state(), TrackingState::WAIT_START);
  EXPECT_TRUE(fsm.updateInputs(ready()));
  EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
}

TEST(TrackingStateMachine, TimeoutClearsArrivalAndAutomaticallyRecovers)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, 10.0, 1.0);
  auto inputs = ready();
  inputs.odom_fresh = false;
  EXPECT_FALSE(fsm.updateInputs(inputs));
  EXPECT_EQ(fsm.reason(), StopReason::ODOM_TIMEOUT);
  EXPECT_EQ(fsm.state(), TrackingState::SAFE_STOP);
  EXPECT_DOUBLE_EQ(fsm.arrivalDuration(11.0), 0.0);
  EXPECT_TRUE(fsm.updateInputs(ready()));
  EXPECT_EQ(fsm.reason(), StopReason::NONE);
  fsm.updateArrival(true, 11.0, 1.0);
  EXPECT_FALSE(fsm.finished());
}

TEST(TrackingStateMachine, StaleReferenceHasPriorityOverFutureStart)
{
  TrackingStateMachine fsm;
  fsm.updateInputs({true, true, true, false, false});
  EXPECT_EQ(fsm.state(), TrackingState::SAFE_STOP);
  EXPECT_EQ(fsm.reason(), StopReason::REFERENCE_TIMEOUT);
}

TEST(TrackingStateMachine, InvalidOdometryWaitsEvenWhenReferenceIsStale)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateInputs({false, false, true, false, false});
  EXPECT_EQ(fsm.state(), TrackingState::WAIT_ODOM);
  EXPECT_EQ(fsm.reason(), StopReason::NONE);
}

TEST(TrackingStateMachine, RollingReferencesKeepTrackingAndRestartArrivalOnly)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  for (int i = 0; i < 100; ++i) {
    fsm.referenceAccepted(ready());
    EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
    EXPECT_TRUE(fsm.canSolve());
  }
  fsm.updateArrival(true, 1.0, 1.0);
  fsm.referenceAccepted(ready());
  EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
  EXPECT_DOUBLE_EQ(fsm.arrivalDuration(3.0), 0.0);
}

TEST(TrackingStateMachine, RepeatedReferenceAndNormalTicksPreserveConfirmation)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, 1.0, 1.0);
  EXPECT_EQ(fsm.state(), TrackingState::ARRIVAL_CONFIRM);
  // 重复或被拒绝的消息不会触发 referenceAccepted 事件。
  fsm.updateInputs(ready());
  fsm.updateArrival(true, 1.99, 1.0);
  EXPECT_TRUE(fsm.confirmingArrival());
  fsm.updateArrival(true, 2.0, 1.0);
  EXPECT_TRUE(fsm.finished());
  EXPECT_FALSE(fsm.canSolve());
}

TEST(TrackingStateMachine, LeavingArrivalConditionsRestartsContinuousTimer)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, 1.0, 1.0);
  fsm.updateArrival(false, 1.9, 1.0);
  EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
  fsm.updateArrival(true, 2.0, 1.0);
  fsm.updateArrival(true, 2.9, 1.0);
  EXPECT_FALSE(fsm.finished());
  fsm.updateArrival(true, 3.0, 1.0);
  EXPECT_TRUE(fsm.finished());
}

TEST(TrackingStateMachine, FinishedLatchSurvivesInvalidInputsAndFailures)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, 1.0, 1.0);
  fsm.updateArrival(true, 2.0, 1.0);
  EXPECT_FALSE(fsm.updateInputs({}));
  fsm.stop(StopReason::CLOCK_REWIND);
  EXPECT_TRUE(fsm.finished());
  fsm.updateArrival(false, 3.0, 1.0);
  EXPECT_TRUE(fsm.finished());
  fsm.referenceAccepted(ready());
  EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
  EXPECT_FALSE(fsm.finished());
}

TEST(TrackingStateMachine, ResetClearsLatchAndSelectsAppropriateWaitState)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, 1.0, 1.0);
  fsm.updateArrival(true, 2.0, 1.0);
  fsm.reset({true, true, false, false, false});
  EXPECT_EQ(fsm.state(), TrackingState::WAIT_REFERENCE);
  EXPECT_DOUBLE_EQ(fsm.arrivalDuration(3.0), 0.0);
  fsm.reset({});
  EXPECT_EQ(fsm.state(), TrackingState::WAIT_ODOM);
}

TEST(TrackingStateMachine, FailuresStopAndRetryWithoutAResetRequest)
{
  for (const auto reason : {StopReason::HORIZON_INVALID, StopReason::SOLVER_FAILED,
      StopReason::COMMAND_INVALID, StopReason::CLOCK_REWIND})
  {
    TrackingStateMachine fsm;
    fsm.referenceAccepted(ready());
    fsm.updateArrival(true, 10.0, 1.0);
    fsm.stop(reason);
    EXPECT_FALSE(fsm.canSolve());
    EXPECT_EQ(fsm.reason(), reason);
    EXPECT_DOUBLE_EQ(fsm.arrivalDuration(12.0), 0.0);
    EXPECT_TRUE(fsm.updateInputs(ready()));
    EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
  }
}

TEST(TrackingStateMachine, ClockRewindAndPausedTimeCannotCompleteEarly)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, 10.0, 1.0);
  fsm.updateArrival(true, 10.0, 1.0);
  EXPECT_DOUBLE_EQ(fsm.arrivalDuration(10.0), 0.0);
  fsm.updateArrival(true, 2.0, 1.0);
  EXPECT_DOUBLE_EQ(fsm.arrivalDuration(2.0), 0.0);
  EXPECT_FALSE(fsm.finished());
  fsm.updateArrival(true, 3.0, 1.0);
  EXPECT_TRUE(fsm.finished());
}

TEST(TrackingStateMachine, InvalidArrivalTimeCannotLatchFinished)
{
  TrackingStateMachine fsm;
  fsm.referenceAccepted(ready());
  fsm.updateArrival(true, std::numeric_limits<double>::quiet_NaN(), 1.0);
  EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
  fsm.updateArrival(true, 1.0, 0.0);
  EXPECT_EQ(fsm.state(), TrackingState::TRACKING);
}
}  // namespace
}  // namespace usv_nmpc
