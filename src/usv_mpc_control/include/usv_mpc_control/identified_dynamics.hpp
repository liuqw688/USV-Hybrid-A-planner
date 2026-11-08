#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace usv_mpc_control::identified {
using State = std::array<double, 6>;  // x, y, psi, u, v, r
using Command = std::array<double, 2>;  // v_cmd [m/s], w_cmd [rad/s]
// User-supplied 11-parameter identification. Shared by search, MPC and plant.
inline constexpr double au=-0.433264281, bu=0.400421432, cu=0.032263988;
inline constexpr double av=-0.313324916, avr=-0.006741061;
inline constexpr double bv=-0.407958500, cv=-0.000993619;
inline constexpr double arv=0.166118168, ar=-0.544509453;
inline constexpr double br=0.837536662, cr=0.000626529;

inline State derivative(const State& s, const Command& c) {
  return {s[3]*std::cos(s[2])-s[4]*std::sin(s[2]),
    s[3]*std::sin(s[2])+s[4]*std::cos(s[2]), s[5],
    au*s[3]+bu*c[0]+cu, av*s[4]+avr*s[5]+bv*c[1]+cv,
    arv*s[4]+ar*s[5]+br*c[1]+cr};
}
inline State offset(State s, const State& d, double h) {
  for (size_t i=0;i<6;++i) s[i]+=h*d[i];
  return s;
}
inline State integrate(State s, const Command& c, double duration, double max_step=0.02) {
  for (double x:s) if (!std::isfinite(x)) throw std::invalid_argument("nonfinite identified state");
  for (double x:c) if (!std::isfinite(x)) throw std::invalid_argument("nonfinite identified command");
  if (!std::isfinite(duration) || duration<0 || !std::isfinite(max_step) || max_step<=0)
    throw std::invalid_argument("invalid identified integration duration/step");
  const int n=std::max(1,static_cast<int>(std::ceil(duration/max_step)));
  const double h=duration/n;
  for (int k=0;k<n;++k) {
    const auto a=derivative(s,c);
    const auto b=derivative(offset(s,a,h/2),c);
    const auto d=derivative(offset(s,b,h/2),c);
    const auto e=derivative(offset(s,d,h),c);
    for (size_t i=0;i<6;++i) s[i]+=h*(a[i]+2*b[i]+2*d[i]+e[i])/6;
  }
  return s;  // Keep yaw continuous, including across +/-pi.
}
inline Command inverse(const State& s, double u_dot, double r_dot) {
  return {(u_dot-au*s[3]-cu)/bu, (r_dot-arv*s[4]-ar*s[5]-cr)/br};
}
}  // namespace usv_mpc_control::identified
