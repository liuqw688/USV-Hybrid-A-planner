#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <path_searching/matrix_hash.h>
#include <Eigen/Eigen>
#include <array>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <memory>
#include "path_searching/kinodynamic_astar_config.hpp"
#include "path_searching/search_environment.hpp"
#include "path_searching/usv_search_state.hpp"
#include "path_searching/planar_distance_field.hpp"

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

class PathNode {
 public:
  /* -------------------- */
  SearchStateIndex index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  bool is_shot_segment = false;  // Uses shot integration step for replay.
  double time = 0.0;
  int time_idx = 0;
  int open_index = -1;
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

class OpenSet {
 private:
  std::vector<PathNodePtr> heap_;

  static bool less(PathNodePtr lhs, PathNodePtr rhs) {
    if (lhs->f_score != rhs->f_score) return lhs->f_score < rhs->f_score;
    return std::less<PathNodePtr>{}(lhs, rhs);
  }

  void swapNodes(std::size_t lhs, std::size_t rhs) {
    std::swap(heap_[lhs], heap_[rhs]);
    heap_[lhs]->open_index = static_cast<int>(lhs);
    heap_[rhs]->open_index = static_cast<int>(rhs);
  }

  void siftUp(std::size_t index) {
    while (index > 0) {
      const std::size_t parent = (index - 1) / 2;
      if (!less(heap_[index], heap_[parent])) break;
      swapNodes(index, parent);
      index = parent;
    }
  }

  void siftDown(std::size_t index) {
    while (true) {
      const std::size_t left = 2 * index + 1;
      if (left >= heap_.size()) break;
      const std::size_t right = left + 1;
      const std::size_t child = right < heap_.size() && less(heap_[right], heap_[left]) ? right : left;
      if (!less(heap_[child], heap_[index])) break;
      swapNodes(index, child);
      index = child;
    }
  }

 public:
  void reserve(std::size_t capacity) { heap_.reserve(capacity); }
  bool empty() const { return heap_.empty(); }
  PathNodePtr top() const { return heap_.front(); }

  void insert(PathNodePtr node) {
    node->open_index = static_cast<int>(heap_.size());
    heap_.push_back(node);
    siftUp(heap_.size() - 1);
  }

  void erase(PathNodePtr node) {
    const int index = node->open_index;
    if (index < 0 || static_cast<std::size_t>(index) >= heap_.size() || heap_[index] != node) return;
    const std::size_t last = heap_.size() - 1;
    node->open_index = -1;
    if (static_cast<std::size_t>(index) == last) {
      heap_.pop_back();
      return;
    }
    heap_[index] = heap_[last];
    heap_[index]->open_index = index;
    heap_.pop_back();
    if (index > 0 && less(heap_[index], heap_[(index - 1) / 2]))
      siftUp(index);
    else
      siftDown(index);
  }

  void pop() { erase(top()); }

  void clear() {
    for (PathNodePtr node : heap_) node->open_index = -1;
    heap_.clear();
  }
};

class NodeHashTable {
 private:
  using Key = std::array<int, 7>;
  struct Entry {
    Key key{};
    PathNodePtr node = nullptr;
    std::uint32_t generation = 0;
  };
  std::vector<Entry> entries_;
  std::size_t mask_ = 0;
  std::uint32_t generation_ = 1;

  static Key key(const SearchStateIndex& index, int time_index) {
    return {index(0), index(1), index(2), index(3), index(4), index(5), time_index};
  }

  static std::size_t hash(const Key& value) {
    std::size_t seed = 0;
    for (int element : value)
      seed ^= std::hash<int>{}(element) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }

  void insert(const Key& value, PathNodePtr node) {
    std::size_t slot = hash(value) & mask_;
    while (entries_[slot].generation == generation_ && entries_[slot].key != value)
      slot = (slot + 1) & mask_;
    entries_[slot].key = value;
    entries_[slot].node = node;
    entries_[slot].generation = generation_;
  }

 public:
  void reserve(std::size_t expected_size) {
    std::size_t capacity = 16;
    while (capacity < expected_size * 2) capacity <<= 1;
    if (capacity <= entries_.size()) return;

    std::vector<Entry> previous = std::move(entries_);
    const std::uint32_t previous_generation = generation_;
    entries_.resize(capacity);
    mask_ = capacity - 1;
    generation_ = 1;
    for (const auto& entry : previous)
      if (entry.generation == previous_generation && entry.node != nullptr)
        insert(entry.key, entry.node);
  }

  void insert(const SearchStateIndex& idx, PathNodePtr node) {
    insert(idx, 0, node);
  }
  void insert(const SearchStateIndex& idx, int time_idx, PathNodePtr node) {
    if (entries_.empty()) reserve(8);
    insert(key(idx, time_idx), node);
  }

  PathNodePtr find(const SearchStateIndex& idx) {
    return find(idx, 0);
  }
  PathNodePtr find(const SearchStateIndex& idx, int time_idx) {
    if (entries_.empty()) return nullptr;
    const Key value = key(idx, time_idx);
    std::size_t slot = hash(value) & mask_;
    while (entries_[slot].generation == generation_) {
      if (entries_[slot].key == value) return entries_[slot].node;
      slot = (slot + 1) & mask_;
    }
    return nullptr;
  }

  void clear() {
    if (++generation_ == 0) {
      for (auto& entry : entries_) entry.generation = 0;
      generation_ = 1;
    }
  }
};

class KinodynamicAstar {
  friend class KinodynamicAstarTest;
 private:
  static constexpr int kShotPieces = 6;
  /* ---------- main data structure ---------- */
  std::vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  PlanarDistanceField obstacle_heuristic_;
  OpenSet open_set_;
  std::vector<PathNodePtr> path_nodes_;
  struct CachedSegment {
    Eigen::Matrix<double, 6, 1> start;
    Eigen::Vector3d command;
    double duration;
    bool shot;
  };
  std::vector<CachedSegment> previous_path_;  ///< 已发布路径的输入副本，不持有节点池指针。
  Eigen::Matrix<double, 6, 1> previous_goal_, current_goal_;
  PathNodePtr replay_end_ = nullptr;  ///< 本次完整重放的末节点，仅在当前搜索内有效。
  size_t replay_begin_segment_ = 0;
  double replay_begin_time_ = 0.0;

  /** @brief 从本次搜索初态重放旧路径剩余输入，将验碰通过的前缀作为搜索初值。 */
  PathNodePtr seedPreviousPath(PathNodePtr root);
  /** @brief 判断六状态是否同时满足现有终端容差。 */
  bool terminalState(const Eigen::Matrix<double, 6, 1>& state) const;
  /** @brief 根据当前速度、被动制动和终端转向需求计算 shot 触发距离，单位 m。 */
  double shotTriggerDistance(const Eigen::Matrix<double, 6, 1>& state) const;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_;
  Eigen::Vector3d start_cmd_;  // [v_cmd, w_cmd, unused]
  std::shared_ptr<SearchEnvironment> environment_;
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
  double min_turn_radius_{0.0};  ///< 与样条发布校验共用的最小半径，单位 m。
  double w_time_, horizon_, lambda_heu_;
  double obstacle_slowdown_distance_{5.0}, obstacle_max_speed_{1.0};
  double search_clearance_;  // 与执行期碰撞检查共用已膨胀地图上的距离门限。
  double shot_trigger_distance_;  // Maximum range for terminal shooting, meters.
  int allocate_num_, check_num_;
  int rk4_steps_;
  // Terminal shooting parameters; set through the search/shot_* parameters.
  std::vector<double> shot_durations_;
  double shot_cruise_speed_, shot_integration_step_, shot_collision_step_;
  double shot_pos_tolerance_, shot_yaw_tolerance_, shot_u_tolerance_, shot_v_tolerance_, shot_r_tolerance_;
  double shot_min_v_cmd_, shot_max_v_cmd_, shot_max_w_cmd_, shot_command_scale_;
  double shot_max_u_, shot_max_v_, shot_max_r_, shot_map_height_;
  double shot_fd_epsilon_, shot_damping_, shot_time_limit_;
  double search_time_limit_ = 0.4;  ///< 规划软预算；管理器为后端保留 20%，单位 s。
  std::chrono::steady_clock::time_point search_deadline_ =
    std::chrono::steady_clock::time_point::max();
  int shot_max_iterations_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, yaw_resolution_, time_resolution_, inv_time_resolution_;
  int yaw_bins_;  // Number of equal heading bins covering one full turn.
  Eigen::Vector3d velocity_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;  // Dynamic search start time, in seconds.

  /* helper */
  SearchStateIndex stateToIndex(const Eigen::Matrix<double, 6, 1>& state) const;
  // Dynamic time in seconds -> bin relative to time_origin_.
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(PathNodePtr start_node,
                       const Eigen::Matrix<double, 6, 1>& goal_state,
                       double time_to_goal);
  void propagateShot(const Eigen::Matrix<double, 6, 1>& start, const Eigen::Vector3d& command,
                     double duration, Eigen::Matrix<double, 6, 1>& result);
  // 六状态加权搜索的时间引导值，含航向引导，不保证可采纳性。
  double estimateHeuristic(const Eigen::Matrix<double, 6, 1>& x1,
                           const Eigen::Matrix<double, 6, 1>& x2,
                           double& optimal_time);

  /* state propagation */
  // USV state: [x_E, y_N, phi, u_forward, v_starboard, r].
  // Command: [V_cmd, W_cmd, unused]. The model implementation is in usv_dynamics.
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);

 public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4, TIMEOUT = 5 };

  /* ---------- USV dynamics conversions (stateless) ---------- */
  // World: ENU. Body: RFU; phi is north-zero and west-positive.
  // pose=[x_E,y_N,phi]; pose_vel=[x_dot_E,y_dot_N,r] in the world frame.
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
  void setConfig(const KinodynamicAstarConfig& config);
  void init();
  void reset();
  /**
   * @brief 按完整船模搜索局部轨迹，并在制动区尝试终端连接。
   * @param start_pt 起始 [x_E,y_N,phi]，单位 m、rad。
   * @param start_body_vel 起始船体速度 [u,v,r]，单位 m/s、rad/s。
   * @param start_cmd 起始控制指令 [V_cmd,W_cmd,unused]。
   * @param end_pt 目标 [x_E,y_N,phi]，单位 m、rad。
   * @param end_body_vel 目标船体速度 [u,v,r]。
   * @param init 是否将首段限制为当前指令。
   * @param dynamic 是否将时间索引纳入状态去重。
   * @param time_start 动态搜索的起始时间，单位 s。
   * @param deadline 调用方共享的稳态时钟截止时刻；默认仅使用本次搜索预算。
   * @return REACH_END 表示终态通过；REACH_HORIZON 可为预算耗尽的安全局部前缀；TIMEOUT 无可用路径。
   */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_body_vel,
             Eigen::Vector3d start_cmd, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_body_vel, bool init, bool dynamic = false,
             double time_start = -1.0,
             std::chrono::steady_clock::time_point deadline =
               std::chrono::steady_clock::time_point::max());

  double getSearchTimeLimit() const { return search_time_limit_; }
  /** @brief 返回样条终点相对当前目标的船首航向容差，单位 rad。 */
  double getShotYawTolerance() const { return shot_yaw_tolerance_; }

  /** @brief 仅在后端通过发布校验后缓存本次路径，供下次重规划初始化。 */
  void cacheSuccessfulPath();

  void setEnvironment(std::shared_ptr<SearchEnvironment> environment);

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  std::pair<int, int> getSegmentCounts() const {
    int search = 0, shot = 0;
    for (size_t i = 1; i < path_nodes_.size(); ++i) {
      if (path_nodes_[i]->is_shot_segment) ++shot; else ++search;
    }
    return {search, shot};
  }

  void getSamples(double& ts, std::vector<Eigen::Vector3d>& point_set,
                  std::vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  using Ptr = std::shared_ptr<KinodynamicAstar>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
