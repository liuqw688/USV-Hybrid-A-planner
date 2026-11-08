#ifndef USV_KINODYNAMIC_FRONTEND_KINODYNAMIC_ASTAR_H
#define USV_KINODYNAMIC_FRONTEND_KINODYNAMIC_ASTAR_H

#include <usv_kinodynamic_frontend/usv_model.hpp>
#include <array>
#include <functional>
#include <vector>
#include <string>
#include <utility>

namespace usv_kinodynamic_frontend {
struct SearchConfig {
  Limits limits;
  std::array<double, 6> resolution{{0.25, 0.25, pi/18, 0.15, 0.10, 0.10}};
  std::vector<double> durations{0.5, 1.0};
  double connection_range = 2.5;
  double connection_cruise_speed = 0.8;
  bool duration_scaled_connection_seed = false;
  std::vector<double> connection_durations{4.0, 6.0, 8.0};
  double integration_step = 0.1, collision_step = 0.04;
  double output_step = 0.1; // Output sampling is independent of RK4 maximum.
  double horizon = 5.0, time_weight = 1.0, command_weight = 0.05;
  double heuristic_weight = 5.0, max_search_time = 0.8;
  double command_scale = 0.85; // reserve command headroom for spline fitting
  double goal_position = 0.35, goal_heading = 0.25;
  double goal_u = 0.15, goal_v = 0.10, goal_r = 0.12;
  int max_nodes = 60000;
  void validate() const;
};
struct Sample {
  double time;
  State state;
  Input command;  // right-continuous command at this time
};
struct SearchResult {
  enum Status { NO_PATH, REACH_HORIZON, REACH_END } status = NO_PATH;
  enum Origin { UNKNOWN, DIRECT_CONNECTION, SEARCH_THEN_CONNECTION, SEARCH_ONLY } origin = UNKNOWN;
  std::vector<Sample> samples;
  int expanded = 0;
  double cost = 0;
};
// Reuses the Kinodynamic A* structure (node pool, hash, priority queue), with
// explicit vessel state/input types. Static obstacles only in this first version.
// The default remains the user's identified velocity-command model. A caller
// may explicitly supply a plant model with different, clearly tagged inputs.
struct Dynamics {
  using Propagate = std::function<State(const State&,const Input&,double,double)>;
  using Inverse = std::function<Input(const State&,double,double)>;
  Propagate propagate = model::integrate;
  Inverse inverse = model::inverse;
  std::string name = "identified_velocity_commands";
  void validate() const {
    if (!propagate || !inverse || name.empty()) throw std::invalid_argument("invalid dynamics provider");
  }
};
class KinodynamicSearch {
 public:
  explicit KinodynamicSearch(const SearchConfig& config, Dynamics dynamics = {})
    : config_(config),dynamics_(std::move(dynamics)) { config_.validate();dynamics_.validate(); }
  using CollisionFree = std::function<bool(const State&)>;
  using StateCost = std::function<double(const State&)>;
  SearchResult search(const State& start, const State& goal, const CollisionFree& free,
                      const StateCost& state_cost = {}) const;
  std::array<int, 6> stateIndex(const State& s) const;
  double heuristic(const State& s, const State& goal) const;
  bool atGoal(const State& s, const State& goal) const;
 private:
  SearchConfig config_;
  Dynamics dynamics_;
};
}  // namespace usv_kinodynamic_frontend
#endif
