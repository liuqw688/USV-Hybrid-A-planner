/**
 * @file nmpc_controller.hpp
 * @brief NMPC 求解器接口和控制参数。
 */

#pragma once

#include <casadi/casadi.hpp>

#include <vector>

#include "usv_nmpc_controller/common_types.hpp"

namespace usv_nmpc
{

/** @brief 预测时域、状态与指令约束、目标权重和求解器参数。 */
struct NMPCConfig
{
  int horizon_steps{150};  ///< 预测区间数量，必须为正数。
  double dt{0.1};  ///< 相邻预测状态的时间间隔，单位 s。

  double weight_lateral{18.0};
  double weight_course{10.0};
  double weight_u{2.0};
  /** 预测状态和参考的最大倒车速度幅值(m/s)，与指令下限分别约束。 */
  double max_reverse_speed_mps{0.0};
  /** 预测状态正向速度 u 的上限(m/s)。 */
  double max_surge_state_mps{5.0};
  /** 预测状态横向速度 v 的对称幅值上限(m/s)。 */
  double max_sway_state_mps{2.0};
  /** 预测状态转艏角速度 r 的对称幅值上限(rad/s)。 */
  double max_yaw_rate_state_radps{0.3};
  double weight_v{1.0};
  double weight_r{2.0};
  /**
   * 速度指令和角速度指令的变化代价权重。
   * 两个通道分别按各自的最大指令归一化，避免 m/s 与 rad/s 混用。
   */
  double weight_surge_delta{3.0};
  double weight_yaw_rate_delta{3.0};
  double min_speed_command_mps{0.0};
  /** 前进速度指令上限(m/s)，运行时由 nmpc.max_speed_command_mps 配置。 */
  double max_speed_command_mps{5.0};
  /** 转向角速度指令范围(rad/s)，默认 ±0.3 rad/s。 */
  double max_yaw_rate_command_radps{0.3};
  /** 速度指令加速/制动变化率硬约束；第一步按实际周期，后续按预测步长。 */
  double max_speed_command_accel_mps2{0.12};  // m/s²。
  double max_speed_command_decel_mps2{1.25};  // m/s²。
  double max_yaw_rate_command_accel_radps2{0.03490658503988659};  // 2°/s²，rad/s²。
  /** 转艏率包络：|r| <= allowance + |u| / radius；不是严格的几何转弯半径。 */
  double yaw_rate_envelope_radius_m{30.0};
  double yaw_rate_envelope_allowance_radps{0.15};
  int solver_max_iterations{100};
  double solver_tolerance{1e-4};
};

/**
 * @brief 将两个指令通道限制在配置的幅值范围内。
 * @param[in] desired 期望的速度和转艏角速度指令，单位分别为 m/s 和 rad/s。
 * @param[in] config 指令幅值约束。
 * @return 限幅后的指令。
 */
Control clampCommand(const Control & desired, const NMPCConfig & config);
/**
 * @brief 按幅值和变化率约束限制指令。
 * @param[in] desired 期望指令。
 * @param[in] previous 上次实际执行的指令，须已满足幅值约束。
 * @param[in] dt 两次指令间隔，调用方保证为有限正数，单位 s。
 * @param[in] config 指令幅值和加速、制动及转向变化率约束。
 * @return 满足本周期变化率约束的指令。
 */
Control limitCommandChange(
  const Control & desired, const Control & previous, double dt, const NMPCConfig & config);

/** @brief 最近一次求解的状态和各目标项累计代价，仅用于诊断。 */
struct NmpcCostBreakdown
{
  bool solver_succeeded{false};
  int consecutive_failures{0};
  double stage_lateral{0.0};
  double stage_course{0.0};
  double stage_u{0.0};
  double stage_v{0.0};
  double stage_r{0.0};
  double stage_delta_control{0.0};
  double total{0.0};
};

/** @brief 带动力学约束、指令限幅和热启动的非线性模型预测控制器。 */
class NMPCController
{
public:
  /**
   * @brief 校验配置并构建 CasADi 求解器。
   * @param[in] config 预测时域、约束及权重参数。
   * @throws std::invalid_argument 配置参数无效。
   * @throws std::exception 求解器构建失败。
   */
  explicit NMPCController(const NMPCConfig & config);

  /**
   * @brief 求解当前预测时域，并返回第一个控制指令。
   * @param[in] initial_state 当前六维状态，采用 ENU 位置和北零西正航向。
   * @param[in] references 等时间间隔参考；不足时重复末点，超过预测时域的点被忽略。
   * @param[in] command_dt_s 首个指令的变化率计算周期，单位 s；非正数时使用配置 dt。
   * @return 首个最优指令；求解失败时返回零指令并清空预测，由节点执行停车。
   * @throws std::invalid_argument 参考序列为空。
   */
  Control solve(
    const State & initial_state,
    const std::vector<RefPoint> & references,
    double command_dt_s = -1.0);
  /** @brief 清除热启动、预测和失败计数，保留上次实际执行的指令。 */
  void resetWarmStart();
  /**
   * @brief 记录实际执行指令，作为下一周期变化率约束的基准。
   * @param[in] command 实际指令；存储前按配置限幅。
   */
  void setLastCommand(const Control & command);
  /**
   * @brief 根据模型稳态方程计算参考点的前馈指令，不作限幅。
   * @param[in] reference 参考前向、侧向速度及转艏角速度。
   * @return 前向速度指令和转艏角速度指令，单位分别为 m/s 和 rad/s。
   */
  Control feedforwardForReference(const RefPoint & reference) const;
  /** @brief 返回最近一次求解是否失败。 */
  bool lastSolveFailed() const;
  /** @brief 返回当前连续求解失败次数。 */
  int consecutiveFailures() const;
  /** @brief 返回最近一次成功求解的预测状态序列；无有效预测时为空。 */
  const std::vector<State> & predictedTrajectory() const;
  /** @brief 返回下一步预测状态；仅有初始状态时返回该状态，无预测时返回零状态。 */
  State predictedNextState() const;
  /** @brief 返回预测的首个指令；无有效预测时返回零指令。 */
  Control predictedFirstCommand() const;
  /** @brief 返回最近一次求解的分项代价与失败诊断。 */
  const NmpcCostBreakdown & lastCostBreakdown() const;

private:
  void validateConfig() const;
  void buildSolver();
  /** 用上次解移位或模型前馈生成初值，并按动力学滚动状态。 */
  std::vector<double> buildInitialGuess(
    const State & state, const std::vector<RefPoint> & references,
    const Control & previous_command, double command_dt_s) const;
  /** 直接求值 NLP 的分项代价表达式，避免另写一套诊断公式。 */
  void updateCostBreakdown(const casadi::DM & solution, const std::vector<double> & parameters);

  NMPCConfig config_;
  casadi::Function solver_;
  casadi::Function cost_function_;
  int constraint_count_{0};
  bool has_last_command_{false};
  double last_speed_command_{0.0};
  double last_yaw_rate_command_{0.0};
  bool last_solve_failed_{false};
  int consecutive_failures_{0};
  std::vector<double> last_solution_;
  std::vector<State> predicted_trajectory_;
  std::vector<Control> predicted_controls_;
  NmpcCostBreakdown last_cost_breakdown_;
};

}  // namespace usv_nmpc
