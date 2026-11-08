#include <usv_kinodynamic_frontend/kinodynamic_astar.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace usv_kinodynamic_frontend;
static int checks=0;
static void require(bool value,const char* message) {
  ++checks;
  if (!value) throw std::runtime_error(message);
}
int main() {
  try {
    State zero=State::Zero();
    const Input command(1.0,0.3);
    const State d=model::derivative(zero,command);
    require(std::abs(d(3)-(0.400421432+0.032263988))<1e-12,"surge coefficients/bias");
    require(std::abs(d(4)-(-0.407958500*0.3-0.000993619))<1e-12,"sway coefficients/bias");
    require(std::abs(d(5)-(0.837536662*0.3+0.000626529))<1e-12,"yaw coefficients/bias");
    State s; s<<1,2,pi/2,0.8,0.2,0.1;
    const State motion=model::derivative(s,command);
    require(std::abs(motion(0)+0.2)<1e-12 && std::abs(motion(1)-0.8)<1e-12,"body/world transform");
    require(std::abs(motion(4)-(-0.313324916*0.2-0.006741061*0.1-0.407958500*0.3-0.000993619))<1e-12,
            "sway coupling");
    require(std::abs(motion(5)-(0.166118168*0.2-0.544509453*0.1+0.837536662*0.3+0.000626529))<1e-12,
            "yaw coupling");
    const double equilibrium=(model::bu*command(0)+model::cu)/(-model::au);
    const State propagated=model::integrate(zero,command,4.0);
    require(std::abs(propagated(3)-equilibrium*(1-std::exp(model::au*4)))<1e-9,"RK4 vs analytic surge response");
    require(std::abs(propagated(3)-command(0))>0.1,"command is not actual surge speed");
    require((model::integrate(s,command,2,0.02)-model::integrate(s,command,2,0.005)).norm()<1e-8,"RK4 step convergence");
    require((model::inverse(s,motion(3),motion(5))-command).norm()<1e-12,"inverse dynamics");
    require((model::integrate(s,command,0)-s).norm()==0,"zero duration");
    bool rejected=false;
    try { model::integrate(s,command,1,0); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"invalid integration step rejected");
    SearchConfig cfg; cfg.max_search_time=3; cfg.horizon=3;
    KinodynamicSearch search(cfg);
    State left=zero,right=zero; left(2)=pi-1e-8;right(2)=-pi+1e-8;
    require(search.stateIndex(left)==search.stateIndex(right),"periodic heading hash");
    right=left;right(3)+=0.3;
    require(search.stateIndex(left)!=search.stateIndex(right),"velocity must participate in hash");
    require(search.heuristic(zero,zero)==0,"heuristic zero at target");
    State goal=zero; goal(0)=2;
    const auto result=search.search(zero,goal,[](const State&){return true;});
    require(result.status==SearchResult::REACH_END,"stationary start -> stationary terminal connection");
    require(!result.samples.empty() && search.atGoal(result.samples.back().state,goal),"full terminal state");
    for (size_t i=0;i+1<result.samples.size();++i) {
      const auto& a=result.samples[i];const auto& b=result.samples[i+1];
      require(cfg.limits.accepts(a.state) && cfg.limits.accepts(a.command),"all samples respect limits");
      require((model::integrate(a.state,a.command,b.time-a.time)-b.state).norm()<1e-8,"connection follows model throughout");
    }
    State far_goal=zero;far_goal(0)=8;
    const auto horizon=search.search(zero,far_goal,[](const State&){return true;});
    require(horizon.status==SearchResult::REACH_HORIZON,"local horizon is distinct from terminal success");
    State obstacle_goal=zero;obstacle_goal(0)=4;
    const auto obstacle_free=[](const State& s){return std::hypot(s(0)-2,s(1))>0.65;};
    const auto detour=search.search(zero,obstacle_goal,obstacle_free);
    require(detour.status==SearchResult::REACH_END,"find feasible detour around a circular obstacle");
    for (const auto& sample:detour.samples) require(obstacle_free(sample.state),"detour stays outside obstacle");
    const auto blocked=search.search(zero,goal,[](const State& s){return s(0)<0.45 || s(0)>0.55;});
    // If a detour exists every sample must avoid the interior obstacle; a direct
    // numerical shot whose endpoints are free must never cross the obstacle.
    for (const auto& sample:blocked.samples)
      require(sample.state(0)<0.45 || sample.state(0)>0.55,"collision checked inside primitive/connection");
    require(search.search(zero,goal,[](const State&){return false;}).status==SearchResult::NO_PATH,"occupied start rejected");
    SearchConfig tiny=cfg; tiny.max_nodes=2;
    require(KinodynamicSearch(tiny).search(zero,goal,[](const State&){return true;}).status==SearchResult::NO_PATH,"pool exhaustion fails safely");
    tiny=cfg; tiny.max_search_time=1e-12;
    require(KinodynamicSearch(tiny).search(zero,goal,[](const State&){return true;}).status==SearchResult::NO_PATH,"timeout is not success");
    State bad=zero;bad(0)=std::numeric_limits<double>::quiet_NaN();
    require(search.search(bad,goal,[](const State&){return true;}).status==SearchResult::NO_PATH,"non-finite state rejected");
    tiny=cfg;tiny.resolution[2]=0;
    rejected=false;
    try { KinodynamicSearch invalid(tiny); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"invalid discretization rejected");
    std::cout<<"USV model/search: "<<checks<<" checks passed; terminal="<<result.samples.back().state.transpose()<<"\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<"\n";return 1; }
  return 0;
}
