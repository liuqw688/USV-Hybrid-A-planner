#ifndef USV_KINODYNAMIC_FRONTEND_MODEL_H
#define USV_KINODYNAMIC_FRONTEND_MODEL_H

#include <Eigen/Core>
#include <usv_mpc_control/identified_dynamics.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace usv_kinodynamic_frontend {
using State = Eigen::Matrix<double, 6, 1>;  // x, y, psi, u, v, r; SI units
using Input = Eigen::Vector2d;             // v_cmd, w_cmd; NOT actual u, r
constexpr double pi = 3.14159265358979323846;
inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Identified coefficients supplied by the user (11 parameters, including biases).
// Keep one source of truth for search, inverse dynamics, validation and simulation.
namespace model {
  using usv_mpc_control::identified::au;
  using usv_mpc_control::identified::bu;
  using usv_mpc_control::identified::cu;
  using usv_mpc_control::identified::av;
  using usv_mpc_control::identified::avr;
  using usv_mpc_control::identified::bv;
  using usv_mpc_control::identified::cv;
  using usv_mpc_control::identified::arv;
  using usv_mpc_control::identified::ar;
  using usv_mpc_control::identified::br;
  using usv_mpc_control::identified::cr;
  inline usv_mpc_control::identified::State arrayState(const State& s) {
    return {s(0),s(1),s(2),s(3),s(4),s(5)};
  }
  inline State eigenState(const usv_mpc_control::identified::State& s) {
    State out; for(int i=0;i<6;++i)out(i)=s[i]; return out;
  }
  inline State derivative(const State& s, const Input& command) {
    return eigenState(usv_mpc_control::identified::derivative(arrayState(s),{command(0),command(1)}));
  }
  inline State integrate(const State& s, const Input& command, double duration, double max_step=0.02) {
    return eigenState(usv_mpc_control::identified::integrate(arrayState(s),{command(0),command(1)},duration,max_step));
  }
  inline Input inverse(const State& s, double u_dot, double r_dot) {
    const auto c=usv_mpc_control::identified::inverse(arrayState(s),u_dot,r_dot);
    return Input(c[0],c[1]);
  }
}  // namespace model

// Engineering defaults for simulation only; identification validity is not known.
struct Limits {
  double min_cmd = -0.4, max_cmd = 1.5, max_turn_cmd = 0.7;
  bool allow_reverse = true;
  double max_u = 1.6, max_v = 0.8, max_r = 0.8;
  bool valid() const {
    return std::isfinite(min_cmd) && std::isfinite(max_cmd) && min_cmd < max_cmd &&
      std::isfinite(max_turn_cmd) && max_turn_cmd > 0 && std::isfinite(max_u) &&
      max_u > 0 && std::isfinite(max_v) && max_v > 0 && std::isfinite(max_r) && max_r > 0;
  }
  bool accepts(const State& s) const {
    return s.allFinite() && (allow_reverse || s(3)>=-1e-8) && std::abs(s(3)) <= max_u &&
           std::abs(s(4)) <= max_v && std::abs(s(5)) <= max_r;
  }
  bool accepts(const Input& c) const {
    return c.allFinite() && c(0) >= min_cmd && c(0) <= max_cmd &&
           std::abs(c(1)) <= max_turn_cmd;
  }
  Input clamp(const Input& c) const {
    return Input(std::max(min_cmd, std::min(max_cmd, c(0))),
                 std::max(-max_turn_cmd, std::min(max_turn_cmd, c(1))));
  }
};
}  // namespace usv_kinodynamic_frontend
#endif
