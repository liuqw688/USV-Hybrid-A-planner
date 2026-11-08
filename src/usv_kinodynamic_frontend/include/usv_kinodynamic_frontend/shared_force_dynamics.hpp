#pragma once
#include <usv_kinodynamic_frontend/kinodynamic_astar.hpp>
#include <hybrid_a_star_planner/usv_dynamics.hpp>

namespace usv_kinodynamic_frontend {
// Explicit adapter to the existing 1015v2 plant. It does not reinterpret
// velocity commands as thrust. Here Input means [tau_y,tau_n].
inline Dynamics sharedForceDynamics(hybrid_a_star_planner::dynamics::Parameters parameters = {}) {
  if(parameters.k_t<=0 || parameters.k_n<=0)
    throw std::invalid_argument("shared force model needs positive actuator gains");
  Dynamics out;
  out.name="shared_1015v2_force_model";
  out.inverse=[parameters](const State& s,double udot,double rdot){
    return Input((udot+parameters.k_u*s(3)+parameters.k_turn*s(5)*s(5))/parameters.k_t,
      (rdot+parameters.k_r*s(5)+parameters.k_r3*s(5)*s(5)*s(5))/parameters.k_n);
  };
  out.propagate=[parameters](const State& initial,const Input& command,double duration,double max_step){
    if(!initial.allFinite() || !command.allFinite() || !std::isfinite(duration) || duration<0 ||
      !std::isfinite(max_step) || max_step<=0) throw std::invalid_argument("invalid force-model integration arguments");
    const int n=std::max(1,static_cast<int>(std::ceil(duration/max_step)));
    const double dt=duration/n;
    hybrid_a_star_planner::dynamics::State s{initial(0),initial(1),wrapAngle(initial(2)),initial(3),initial(4),initial(5)};
    double continuous_heading=initial(2);
    for(int i=0;i<n;++i) {
      const double old=s.yaw;
      s=hybrid_a_star_planner::dynamics::integrateRk4(s,{command(0),command(1)},parameters,dt);
      continuous_heading+=wrapAngle(s.yaw-old);
    }
    State result;result<<s.x,s.y,continuous_heading,s.u,s.v,s.r;return result;
  };
  return out;
}
}  // namespace usv_kinodynamic_frontend
