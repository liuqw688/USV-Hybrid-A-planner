#pragma once

#include <algorithm>
#include <cmath>
#include "hybrid_a_star_planner/usv_dynamics.hpp"

namespace usv_mpc_control
{
using VesselState = hybrid_a_star_planner::dynamics::State;
using hybrid_a_star_planner::dynamics::normalizeAngle;

// These commands and coefficients match Fast-Planner's usvStateDerivative.
struct VesselControl { double v_cmd{0.0}, w_cmd{0.0}; };
struct VesselModelParameters { double k_v{0.313324916}; };

inline VesselState derivative(const VesselState &s, const VesselControl &c)
{
  return {s.u * std::cos(s.yaw) - s.v * std::sin(s.yaw),
    s.u * std::sin(s.yaw) + s.v * std::cos(s.yaw), s.r,
    -0.433264281 * s.u + 0.400421432 * c.v_cmd + 0.032263988,
    -0.313324916 * s.v - 0.006741061 * s.r - 0.407958500 * c.w_cmd - 0.000993619,
    0.166118168 * s.v - 0.544509453 * s.r + 0.837536662 * c.w_cmd + 0.000626529};
}
inline VesselState offset(const VesselState &s, const VesselState &d, double t)
{
  return {s.x+t*d.x, s.y+t*d.y, s.yaw+t*d.yaw, s.u+t*d.u, s.v+t*d.v, s.r+t*d.r};
}
inline VesselState integrateRk4(const VesselState &s, const VesselControl &c,
                                 const VesselModelParameters &, double dt)
{
  const auto a=derivative(s,c), b=derivative(offset(s,a,dt/2),c);
  const auto d=derivative(offset(s,b,dt/2),c), e=derivative(offset(s,d,dt),c);
  VesselState out{s.x+dt*(a.x+2*b.x+2*d.x+e.x)/6,
    s.y+dt*(a.y+2*b.y+2*d.y+e.y)/6,
    s.yaw+dt*(a.yaw+2*b.yaw+2*d.yaw+e.yaw)/6,
    s.u+dt*(a.u+2*b.u+2*d.u+e.u)/6,
    s.v+dt*(a.v+2*b.v+2*d.v+e.v)/6,
    s.r+dt*(a.r+2*b.r+2*d.r+e.r)/6};
  out.yaw=normalizeAngle(out.yaw);
  return out;
}
inline VesselControl trackSetpoint(const VesselState &s, double speed, double rate,
  const VesselModelParameters &, double surge_time, double yaw_time,
  double max_v_cmd, double max_w_cmd)
{
  const double v_cmd = ((speed-s.u)/std::max(0.05,surge_time)+0.433264281*s.u-0.032263988)/0.400421432;
  const double w_cmd = ((rate-s.r)/std::max(0.05,yaw_time)-0.166118168*s.v+0.544509453*s.r-0.000626529)/0.837536662;
  return {std::clamp(v_cmd,0.0,max_v_cmd),std::clamp(w_cmd,-max_w_cmd,max_w_cmd)};
}
}  // namespace usv_mpc_control
