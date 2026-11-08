#include <usv_kinodynamic_frontend/shared_force_dynamics.hpp>
#include <iostream>
#include <stdexcept>
using namespace usv_kinodynamic_frontend;
int main(){try{
  const auto dynamics=sharedForceDynamics();
  State s;s<<1,2,3.12,1.2,-0.1,0.2;Input input(2,0.3);
  const State advanced=dynamics.propagate(s,input,1,0.02);
  hybrid_a_star_planner::dynamics::State oracle{s(0),s(1),wrapAngle(s(2)),s(3),s(4),s(5)};
  hybrid_a_star_planner::dynamics::Parameters p;
  for(int i=0;i<50;++i)oracle=hybrid_a_star_planner::dynamics::integrateRk4(oracle,{input(0),input(1)},p,0.02);
  if(std::hypot(advanced(0)-oracle.x,advanced(1)-oracle.y)>1e-12 ||
      std::abs(wrapAngle(advanced(2)-oracle.yaw))>1e-12 || advanced(2)<=pi)
    throw std::runtime_error("shared model integration or continuous heading mismatch");
  const auto inverse=dynamics.inverse(s,0.1,0.05);
  hybrid_a_star_planner::dynamics::State raw{s(0),s(1),s(2),s(3),s(4),s(5)};
  const auto derivative=hybrid_a_star_planner::dynamics::derivative(raw,{inverse(0),inverse(1)},p);
  if(std::abs(derivative.u-0.1)>1e-12 || std::abs(derivative.r-0.05)>1e-12)
    throw std::runtime_error("force inverse dynamics mismatch");
  const Dynamics identified;
  if((identified.propagate(s,Input(0.8,0.2),0.5,0.02)-model::integrate(s,Input(0.8,0.2),0.5,0.02)).norm()>1e-12)
    throw std::runtime_error("default identified model changed");
  std::cout<<"PASS: existing shared force model, inverse, heading continuity, identified mode retained\n";
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}return 0;}
