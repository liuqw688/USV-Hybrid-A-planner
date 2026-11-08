#pragma once

#include "hybrid_a_star_planner/usv_dynamics.hpp"

namespace usv_mpc_control
{
// Planner, MPC and simulator share one set of state equations and RK4 integration.
using VesselState = hybrid_a_star_planner::dynamics::State;
using VesselControl = hybrid_a_star_planner::dynamics::Control;
using VesselModelParameters = hybrid_a_star_planner::dynamics::Parameters;
using hybrid_a_star_planner::dynamics::integrateRk4;
using hybrid_a_star_planner::dynamics::normalizeAngle;

// MPC uses a speed/rate feedback seed as the initial actuator sequence. Its
// response times and actuator limits remain explicit controller parameters.
inline VesselControl trackSetpoint(
  const VesselState &state, double desired_speed, double desired_rate,
  const VesselModelParameters &model, double surge_time, double yaw_time,
  double max_thrust, double max_torque)
{
  auto parameters = model;
  parameters.surge_response_time = surge_time;
  parameters.yaw_response_time = yaw_time;
  parameters.max_tau_y = max_thrust;
  parameters.max_tau_n = max_torque;
  return hybrid_a_star_planner::dynamics::trackSetpoint(
    state, desired_speed, desired_rate, parameters);
}
}  // namespace usv_mpc_control
