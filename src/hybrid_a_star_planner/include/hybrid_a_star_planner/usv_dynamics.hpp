#pragma once

#include <algorithm>
#include <cmath>

namespace hybrid_a_star_planner
{
namespace dynamics
{

struct Parameters
{
  double k_u{0.290681};
  double k_t{0.295742};
  double k_turn{2.126739};
  double k_v{0.425383};
  double k_vr{-1.149012};
  double k_vn{0.0};
  double k_r{1.342561};
  double k_r3{0.0};
  double k_n{0.781688};
  double surge_response_time{0.8};
  double yaw_response_time{0.8};
  double max_tau_y{10.0};
  double max_tau_n{5.0};
  double max_yaw_rate{0.45};
  double cruise_speed{4.0};
  double integration_step{0.05};
};

struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double u{0.0};
  double v{0.0};
  double r{0.0};
};

struct Control
{
  double tau_y{0.0};
  double tau_n{0.0};
};

struct Derivative
{
  double x, y, yaw, u, v, r;
};

inline double normalizeAngle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}

inline Derivative derivative(const State & s, const Control & c, const Parameters & p)
{
  return {
    s.u * std::cos(s.yaw) - s.v * std::sin(s.yaw),
    s.u * std::sin(s.yaw) + s.v * std::cos(s.yaw),
    s.r,
    -p.k_u * s.u + p.k_t * c.tau_y - p.k_turn * s.r * s.r,
    -p.k_v * s.v + p.k_vr * s.r - p.k_vn * c.tau_n,
    -p.k_r * s.r - p.k_r3 * s.r * s.r * s.r + p.k_n * c.tau_n};
}

inline State offset(const State & s, const Derivative & d, double scale)
{
  return {s.x + scale * d.x, s.y + scale * d.y, s.yaw + scale * d.yaw,
    s.u + scale * d.u, s.v + scale * d.v, s.r + scale * d.r};
}

inline State integrateRk4(const State & s, const Control & c, const Parameters & p, double dt)
{
  const auto a = derivative(s, c, p);
  const auto b = derivative(offset(s, a, 0.5 * dt), c, p);
  const auto d = derivative(offset(s, b, 0.5 * dt), c, p);
  const auto e = derivative(offset(s, d, dt), c, p);
  State out{
    s.x + dt * (a.x + 2.0 * b.x + 2.0 * d.x + e.x) / 6.0,
    s.y + dt * (a.y + 2.0 * b.y + 2.0 * d.y + e.y) / 6.0,
    s.yaw + dt * (a.yaw + 2.0 * b.yaw + 2.0 * d.yaw + e.yaw) / 6.0,
    s.u + dt * (a.u + 2.0 * b.u + 2.0 * d.u + e.u) / 6.0,
    s.v + dt * (a.v + 2.0 * b.v + 2.0 * d.v + e.v) / 6.0,
    s.r + dt * (a.r + 2.0 * b.r + 2.0 * d.r + e.r) / 6.0};
  out.yaw = normalizeAngle(out.yaw);
  return out;
}

inline Control trackSetpoint(
  const State & s, double desired_speed, double desired_yaw_rate, const Parameters & p)
{
  const double surge_tau = std::max(0.05, p.surge_response_time);
  const double yaw_tau = std::max(0.05, p.yaw_response_time);
  const double tau_y = (p.k_u * s.u + p.k_turn * s.r * s.r +
    (desired_speed - s.u) / surge_tau) / std::max(1.0e-9, p.k_t);
  const double tau_n = (p.k_r * s.r + p.k_r3 * s.r * s.r * s.r +
    (desired_yaw_rate - s.r) / yaw_tau) / std::max(1.0e-9, p.k_n);
  return {std::clamp(tau_y, -std::abs(p.max_tau_y), std::abs(p.max_tau_y)),
    std::clamp(tau_n, -std::abs(p.max_tau_n), std::abs(p.max_tau_n))};
}

inline State stepSetpoint(
  const State & state, double desired_speed, double desired_yaw_rate,
  const Parameters & p, double dt)
{
  const double h = std::max(0.005, p.integration_step);
  const int steps = std::max(1, static_cast<int>(std::ceil(dt / h)));
  const double substep = dt / steps;
  State out = state;
  for (int i = 0; i < steps; ++i) {
    const auto control = trackSetpoint(out, desired_speed, desired_yaw_rate, p);
    out = integrateRk4(out, control, p, substep);
    out.u = std::clamp(out.u, 0.0, p.cruise_speed);
    out.r = std::clamp(out.r, -std::abs(p.max_yaw_rate), std::abs(p.max_yaw_rate));
  }
  return out;
}

}  // namespace dynamics
}  // namespace hybrid_a_star_planner
