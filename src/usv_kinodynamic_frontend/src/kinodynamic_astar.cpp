#include <usv_kinodynamic_frontend/kinodynamic_astar.hpp>
#include <algorithm>
#include <Eigen/Cholesky>
#include <chrono>
#include <queue>
#include <limits>
#include <unordered_map>

namespace usv_kinodynamic_frontend {
void SearchConfig::validate() const {
  if (!limits.valid() || max_nodes < 2 || durations.empty() || connection_durations.empty())
    throw std::invalid_argument("invalid USV search limits/pool/durations");
  if (!std::isfinite(command_scale) || command_scale<=0 || command_scale>1)
    throw std::invalid_argument("USV command_scale must be in (0,1]");
  for (double d : resolution)
    if (!std::isfinite(d) || d <= 0) throw std::invalid_argument("invalid USV state resolution");
  for (double d : durations)
    if (!std::isfinite(d) || d <= 0) throw std::invalid_argument("invalid USV primitive duration");
  for (double d : connection_durations)
    if (!std::isfinite(d) || d<=0) throw std::invalid_argument("invalid terminal connection duration");
  for (double d : {connection_cruise_speed, connection_range, integration_step, collision_step, output_step, horizon, time_weight, heuristic_weight,
                   max_search_time, goal_position, goal_heading, goal_u, goal_v, goal_r})
    if (!std::isfinite(d) || d <= 0) throw std::invalid_argument("invalid USV search parameter");
  if (!std::isfinite(command_weight) || command_weight < 0)
    throw std::invalid_argument("invalid USV command cost");
}
std::array<int, 6> KinodynamicSearch::stateIndex(const State& s) const {
  std::array<int, 6> key;
  for (int i = 0; i < 6; ++i)
    if (i!=2) key[i] = static_cast<int>(std::floor(s(i)/config_.resolution[i]));
  const int bins = std::max(1, static_cast<int>(std::ceil(2*pi/config_.resolution[2])));
  // Round to nearest circular bin, so +/-pi represent the same neighborhood.
  const int yaw = static_cast<int>(std::floor((wrapAngle(s(2)) + pi)/(2*pi)*bins + 0.5));
  key[2] = yaw % bins;
  return key;
}
bool KinodynamicSearch::atGoal(const State& s, const State& goal) const {
  return (s.head<2>() - goal.head<2>()).norm() <= config_.goal_position &&
    std::abs(wrapAngle(s(2) - goal(2))) <= config_.goal_heading &&
    std::abs(s(3)-goal(3)) <= config_.goal_u &&
    std::abs(s(4)-goal(4)) <= config_.goal_v && std::abs(s(5)-goal(5)) <= config_.goal_r;
}
double KinodynamicSearch::heuristic(const State& s, const State& goal) const {
  // Lower bound on time to the tolerance region using bounded actual speeds.
  // Weighted A* (weight>1) trades optimality for speed; no quartic/double integrator.
  const double t_xy = std::max(0.0, (s.head<2>()-goal.head<2>()).norm()-config_.goal_position) /
    std::hypot(config_.limits.max_u, config_.limits.max_v);
  const double t_psi = std::max(0.0, std::abs(wrapAngle(s(2)-goal(2)))-config_.goal_heading) /
    config_.limits.max_r;
  return config_.time_weight * std::max(t_xy, t_psi);
}
namespace {
struct KeyHash {
  size_t operator()(const std::array<int, 6>& key) const {
    size_t seed = 0;
    for (int i : key) seed ^= std::hash<int>()(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    return seed;
  }
};
struct Node {
  State state;
  Input command;
  int parent;
  double duration, time, g;
};
struct Entry {
  double f;
  int node;
  bool operator<(const Entry& rhs) const { return f > rhs.f; }
};
}
SearchResult KinodynamicSearch::search(const State& start, const State& goal,
                                      const CollisionFree& free, const StateCost& state_cost) const {
  SearchResult result;
  if (!config_.limits.accepts(start) || !config_.limits.accepts(goal) || !free || !free(start) || !free(goal))
    return result;
  const auto penalty=[&](const State& state) {
    const double value=state_cost ? state_cost(state) : 0.0;
    return std::isfinite(value) && value>=0 ? value : std::numeric_limits<double>::infinity();
  };
  if (!std::isfinite(penalty(start)) || !std::isfinite(penalty(goal))) return result;
  std::vector<Node> pool;
  pool.reserve(config_.max_nodes);
  std::unordered_map<std::array<int, 6>, int, KeyHash> best;
  std::priority_queue<Entry> open;
  pool.push_back({start, dynamics_.inverse(start, 0, 0), -1, 0, 0, 0});
  best[stateIndex(start)] = 0;
  open.push({config_.heuristic_weight*heuristic(start, goal), 0});
  const auto begin = std::chrono::steady_clock::now();
  Limits actuation_limits=config_.limits;
  const double command_margin=(actuation_limits.max_cmd-actuation_limits.min_cmd)*(1-config_.command_scale)/2;
  actuation_limits.min_cmd+=command_margin;
  actuation_limits.max_cmd-=command_margin;
  actuation_limits.max_turn_cmd*=config_.command_scale;
  int terminal = -1;
  double last_connection_distance = std::numeric_limits<double>::infinity();
  // Numerical multiple-command shooting replaces the UAV cubic shot. All
  // accepted connections are replayed with model/limits/collision checks.
  auto connect = [&](const Node& cur, std::vector<Node>& connection) {
    constexpr int pieces=4;
    for (double total : config_.connection_durations) {
      const double h=total/pieces;
      Eigen::Matrix<double,8,1> controls;
      State seed=cur.state;
      for (int i=0;i<pieces;++i) {
        const double distance=(goal.head<2>()-seed.head<2>()).norm();
        const double bearing=std::atan2(goal(1)-seed(1),goal(0)-seed(0));
        const double heading=i==pieces-1 ? goal(2) : bearing;
        const double err=wrapAngle(heading-seed(2));
        const double u=i==pieces-1 ? goal(3) : std::min(config_.connection_cruise_speed,distance*0.6)*std::max(0.0,std::cos(err));
        double u_dot=u-seed(3),r_dot=err-seed(5);
        if(config_.duration_scaled_connection_seed) {
          const double target_r=std::max(-config_.limits.max_r*0.8,
            std::min(config_.limits.max_r*0.8,err/std::max(1.0,h)));
          const double response=std::max(1.0,h*0.4);
          u_dot=(u-seed(3))/response;r_dot=(target_r-seed(5))/response;
        }
        const Input command=actuation_limits.clamp(dynamics_.inverse(seed,u_dot,r_dot));
        controls.segment<2>(2*i)=command;
        seed=dynamics_.propagate(seed,command,h,config_.integration_step);
      }
      auto endpoint = [&](const Eigen::Matrix<double,8,1>& commands) {
        State end=cur.state;
        for (int i=0;i<pieces;++i)
          end=dynamics_.propagate(end,commands.segment<2>(2*i),h,config_.integration_step);
        return end;
      };
      auto error = [&](const State& end) {
        State e=end-goal; e(2)=wrapAngle(e(2)); return e;
      };
      for (int iteration=0;iteration<16;++iteration) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>config_.max_search_time)
          return false;
        const State end=endpoint(controls);
        if (atGoal(end,goal)) {
          State s=cur.state;
          connection.clear();
          double g=cur.g;
          bool safe=true;
          for (int i=0;i<pieces && safe;++i) {
            const Input command=controls.segment<2>(2*i);
            const double step=std::min(config_.collision_step,0.5*std::min(config_.resolution[0],
              config_.resolution[1])/std::hypot(config_.limits.max_u,config_.limits.max_v));
            const int n=static_cast<int>(std::ceil(h/step));
            for (int j=0;j<n;++j) {
              s=dynamics_.propagate(s,command,h/n,config_.integration_step);
              if (!config_.limits.accepts(s) || !free(s) || !std::isfinite(penalty(s))) { safe=false; break; }
              g+=h/n*penalty(s);
            }
            const double effort=std::pow(command(0)/std::max(std::abs(config_.limits.min_cmd),
              config_.limits.max_cmd),2)+std::pow(command(1)/config_.limits.max_turn_cmd,2);
            g+=h*(config_.time_weight+config_.command_weight*effort);
            connection.push_back({s,command,-1,h,cur.time+(i+1)*h,g});
          }
          if (safe && atGoal(s,goal)) return true;
          break;
        }
        const State residual=error(end);
        Eigen::Matrix<double,6,8> jacobian;
        for (int j=0;j<8;++j) {
          auto perturbed=controls; perturbed(j)+=1e-3;
          State delta=endpoint(perturbed)-end; delta(2)=wrapAngle(delta(2));
          jacobian.col(j)=delta/1e-3;
        }
        const Eigen::Matrix<double,6,6> normal=jacobian*jacobian.transpose()+
          0.02*Eigen::Matrix<double,6,6>::Identity();
        const Eigen::Matrix<double,8,1> change=-jacobian.transpose()*normal.ldlt().solve(residual);
        bool improved=false;
        for (double alpha : {1.0,0.5,0.25}) {
          Eigen::Matrix<double,8,1> candidate=controls+alpha*change;
          for (int i=0;i<pieces;++i) candidate.segment<2>(2*i)=actuation_limits.clamp(candidate.segment<2>(2*i));
          if (error(endpoint(candidate)).squaredNorm()<residual.squaredNorm()) {
            controls=candidate; improved=true; break;
          }
        }
        if (!improved) break;
      }
    }
    return false;
  };
  while (!open.empty()) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() > config_.max_search_time)
      break;
    const int id = open.top().node;
    open.pop();
    const Node cur = pool[id];
    if (best.at(stateIndex(cur.state)) != id) continue; // stale immutable queue entry
    ++result.expanded;
    if (atGoal(cur.state, goal)) {
      terminal = id; result.status = SearchResult::REACH_END; result.origin=SearchResult::SEARCH_ONLY; break;
    }
    const double goal_distance=(cur.state.head<2>()-goal.head<2>()).norm();
    if (goal_distance<config_.connection_range && goal_distance<last_connection_distance-0.3 &&
        static_cast<int>(pool.size())+4<=config_.max_nodes) {
      last_connection_distance=goal_distance;
      std::vector<Node> connection;
      if (connect(cur,connection)) {
        result.origin=(id==0 ? SearchResult::DIRECT_CONNECTION : SearchResult::SEARCH_THEN_CONNECTION);
        int parent=id;
        for (auto node:connection) {
          node.parent=parent;
          parent=static_cast<int>(pool.size()); pool.push_back(node);
        }
        terminal=parent; result.status=SearchResult::REACH_END; break;
      }
    }
    if ((cur.state.head<2>() - start.head<2>()).norm() >= config_.horizon) {
      terminal = id; result.status = SearchResult::REACH_HORIZON; break;
    }
    std::vector<Input> commands;
    const auto& lim = actuation_limits;
    for (double v : {lim.min_cmd, 0.0, 0.5*lim.max_cmd, lim.max_cmd})
      for (int j = -2; j <= 2; ++j) commands.emplace_back(v, j*lim.max_turn_cmd/2);
    commands.push_back(lim.clamp(dynamics_.inverse(cur.state, 0, 0)));
    // Goal-directed primitive supplements the lattice. It is still integrated
    // with exactly the same identified model, never a geometric shortcut.
    const double distance = (goal.head<2>() - cur.state.head<2>()).norm();
    const double bearing = std::atan2(goal(1)-cur.state(1), goal(0)-cur.state(0));
    const double heading = distance < config_.goal_position*1.5 ? goal(2) : bearing;
    const double err = wrapAngle(heading-cur.state(2));
    const double target_u = std::min(0.85*lim.max_u, distance*0.65) * std::max(0.0, std::cos(err));
    const double target_r = std::max(-lim.max_r*0.8, std::min(lim.max_r*0.8, err*1.2));
    commands.push_back(lim.clamp(dynamics_.inverse(cur.state, (target_u-cur.state(3))*0.8,
                                               (target_r-cur.state(5))*1.2)));
    for (const Input& command : commands) {
      if (!lim.accepts(command)) continue;
      for (double duration : config_.durations) {
        State next = cur.state;
        const double sample_step = std::min(config_.collision_step,
          0.5*std::min(config_.resolution[0], config_.resolution[1])/std::hypot(lim.max_u, lim.max_v));
        const int steps = std::max(1, static_cast<int>(std::ceil(duration/sample_step)));
        const double dt = duration/steps;
        bool safe = true;
        double environment_cost=0;
        for (int k = 0; k < steps; ++k) {
          next = dynamics_.propagate(next, command, dt, config_.integration_step);
          const double extra=penalty(next);
          if (!lim.accepts(next) || !free(next) || !std::isfinite(extra)) { safe = false; break; }
          environment_cost+=dt*extra;
        }
        if (!safe) continue;
        const auto key = stateIndex(next);
        if (key == stateIndex(cur.state)) continue;
        const double effort = std::pow(command(0)/std::max(std::abs(config_.limits.min_cmd),config_.limits.max_cmd),2) +
                              std::pow(command(1)/config_.limits.max_turn_cmd,2);
        const double g = cur.g + duration*(config_.time_weight + config_.command_weight*effort) + environment_cost;
        const auto found = best.find(key);
        if (found != best.end() && pool[found->second].g <= g) continue;
        if (static_cast<int>(pool.size()) >= config_.max_nodes) continue;
        const int child = static_cast<int>(pool.size());
        pool.push_back({next, command, id, duration, cur.time+duration, g});
        best[key] = child;
        open.push({g + config_.heuristic_weight*heuristic(next, goal), child});
      }
    }
  }
  if (terminal < 0) return result; // timeout/pool exhaustion never disguises failure as success
  result.cost = pool[terminal].g;
  std::vector<int> chain;
  for (int id = terminal; id >= 0; id = pool[id].parent) chain.push_back(id);
  std::reverse(chain.begin(), chain.end());
  result.samples.push_back({0, start, chain.size()>1 ? pool[chain[1]].command : pool[0].command});
  for (size_t i = 1; i < chain.size(); ++i) {
    const Node& child = pool[chain[i]];
    const Node& parent = pool[child.parent];
    const int n = std::max(1, static_cast<int>(std::ceil(child.duration/config_.output_step)));
    State s = parent.state;
    for (int k = 1; k <= n; ++k) {
      s = dynamics_.propagate(s, child.command, child.duration/n, config_.integration_step);
      const Input cmd = (k == n && i+1 < chain.size()) ? pool[chain[i+1]].command : child.command;
      result.samples.push_back({parent.time + k*child.duration/n, s, cmd});
    }
  }
  return result;
}

}  // namespace usv_kinodynamic_frontend
