#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <path_searching/matrix_hash.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <set>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  bool is_shot_segment = false;  // Uses shot integration step for replay.
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) const {
    if (node1->f_score != node2->f_score) return node1->f_score < node2->f_score;
    return std::less<PathNodePtr>{}(node1, node2);
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar {
 private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  // Ordered set permits removing a node before changing its f-score.
  std::set<PathNodePtr, NodeComparator> open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_;
  Eigen::Vector3d start_cmd_;  // [v_cmd, w_cmd, unused]
  // shared_ptr<SDFMap> sdf_map;
  EDTEnvironment::Ptr edt_environment_;
  bool is_shot_succ_ = false;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double max_u_, max_v_, max_r_;  // Body-frame speeds and yaw rate.
  double min_v_cmd_, max_v_cmd_, max_w_cmd_;  // Ordinary search commands.
  // Bounds on actual ground speed (m/s) and yaw rate (rad/s), not commands.
  // Assigned by setParam(), like the other search parameters.
  double max_ground_speed_, max_yaw_rate_;
  double w_time_, horizon_, lambda_heu_;
  double goal_pos_tolerance_, goal_yaw_tolerance_;  // meters, radians
  int allocate_num_, check_num_;
  int rk4_steps_;
  // Terminal shooting parameters; set through the search/shot_* parameters.
  std::vector<double> shot_durations_;
  double shot_cruise_speed_, shot_integration_step_, shot_collision_step_;
  double shot_pos_tolerance_, shot_yaw_tolerance_, shot_u_tolerance_, shot_v_tolerance_, shot_r_tolerance_;
  double shot_min_v_cmd_, shot_max_v_cmd_, shot_max_w_cmd_, shot_command_scale_;
  double shot_max_u_, shot_max_v_, shot_max_r_, shot_map_height_;
  double shot_fd_epsilon_, shot_damping_, shot_time_limit_;
  int shot_max_iterations_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, yaw_resolution_, time_resolution_, inv_time_resolution_;
  int yaw_bins_;  // Number of equal heading bins covering one full turn.
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;  // Dynamic search start time, in seconds.

  /* helper */
  // pose=[x,y,phi] -> index=[x cell,y cell,periodic heading bin].
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  // Dynamic time in seconds -> bin relative to time_origin_.
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  void propagateShot(const Eigen::Matrix<double, 6, 1>& start, const Eigen::Vector3d& command,
                     double duration, Eigen::Matrix<double, 6, 1>& result);
  Eigen::Vector3d poseAcceleration(const Eigen::Matrix<double, 6, 1>& state,
                                   const Eigen::Vector3d& command) const;
  // USV states [x,y,psi,u,v,r]; optimal_time is now a time lower bound.
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);

  /* state propagation */
  // USV state: [x, y, psi, u, v, r]. Command: [v_cmd, w_cmd, unused].
  // Configured through search/rk4_steps; each primitive uses h = tau / rk4_steps_.
  Eigen::Matrix<double, 6, 1> usvStateDerivative(
      const Eigen::Matrix<double, 6, 1>& state, const Eigen::Vector3d& um) const;
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);

 public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  /* ---------- USV dynamics conversions (stateless) ---------- */
  // pose=[x,y,phi]; pose_vel=[dot_x,dot_y,r] in the reference frame.
  static Eigen::Vector3d computeBodyVelocity(const Eigen::Vector3d& pose,
                                             const Eigen::Vector3d& pose_vel);
  // body_vel=[u,v,r] -> reference-frame [vx,vy,r].
  // The third velocity component is yaw rate r, not heading phi.
  static Eigen::Vector3d computeGlobalVelocity(const Eigen::Vector3d& pose,
                                               const Eigen::Vector3d& body_vel);
  // pose_acc=[ddot_x,ddot_y,dot_r]; outputs [u,v,r] and [v_cmd,w_cmd,0].
  static void computeUsvStateAndCommand(const Eigen::Vector3d& pose,
                                        const Eigen::Vector3d& pose_vel,
                                        const Eigen::Vector3d& pose_acc,
                                        Eigen::Vector3d& body_vel,
                                        Eigen::Vector3d& command);

  /* main API */
  void setParam(rclcpp::Node& nh);
  void init();
  void reset();
  // Poses: [x,y,phi]; start/end_body_vel: body-frame [u,v,r].
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_body_vel,
             Eigen::Vector3d start_cmd, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_body_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  std::pair<int, int> getSegmentCounts() const {
    int search = 0, shot = 0;
    for (size_t i = 1; i < path_nodes_.size(); ++i) {
      if (path_nodes_[i]->is_shot_segment) ++shot; else ++search;
    }
    return {search, shot};
  }

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
