#pragma once
#include "usv_mpc_control/underactuated_model.hpp"
#include "usv_mpc_control/identified_dynamics.hpp"

namespace usv_mpc_control {
// Neutral names: units are defined by the selected model, never encoded in Wrench.
struct TrackingControl { double surge{0.0}, yaw{0.0}; };
struct TrackingModelParameters : VesselModelParameters { bool identified{false}; };
inline identified::State identifiedState(const VesselState& s) {
  return {s.x,s.y,s.yaw,s.u,s.v,s.r};
}
inline VesselState predict(const VesselState& s, const TrackingControl& c,
  const TrackingModelParameters& p, double dt) {
  if (!p.identified) return integrateRk4(s,{c.surge,c.yaw},p,dt);
  const auto out=identified::integrate(identifiedState(s),{c.surge,c.yaw},dt);
  return {out[0],out[1],out[2],out[3],out[4],out[5]};
}
inline TrackingControl feedbackControl(const VesselState& s, double speed, double rate,
  const TrackingModelParameters& p, double surge_time, double yaw_time,
  double max_surge, double max_yaw, double min_surge) {
  if (!p.identified) {
    const auto c=trackSetpoint(s,speed,rate,p,surge_time,yaw_time,max_surge,max_yaw);
    return {c.tau_y,c.tau_n};
  }
  const auto c=identified::inverse(identifiedState(s),
    (speed-s.u)/std::max(.05,surge_time), (rate-s.r)/std::max(.05,yaw_time));
  return {std::clamp(c[0],min_surge,max_surge), std::clamp(c[1],-max_yaw,max_yaw)};
}
}  // namespace usv_mpc_control
