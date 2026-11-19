/**
 * @file reference_trajectory.hpp
 * @brief B 样条参考轨迹的离散状态和任务元数据。
 */

#pragma once

#include <cstdint>
#include <vector>

#include "usv_nmpc_controller/nmpc_controller.hpp"

namespace usv_nmpc
{

/** @brief 离散参考点，含相对时间、位姿、速度比例和诊断量。 */
struct ReferencePoint
{
  /** 轨迹相对起始时刻，单位 s。 */
  double t{0.0};
  /** 沿轨迹累计距离，单位 m。 */
  double s{0.0};
  /** ENU 位置，单位 m。 */
  double x{0.0};
  double y{0.0};
  /** 北零西正船首航向，单位 rad。 */
  double psi{0.0};
  /** 按前向速度符号定义的有符号合速度，倒车为负，单位 m/s。 */
  double speed{0.0};
  /** 转艏角速度与地速的比值，单位 1/m；忽略侧滑角变化率。 */
  double curvature{0.0};
  /** 参考转艏角速度，单位 rad/s。 */
  double yaw_rate{0.0};
  /** 地速切线航向，北零西正，单位 rad。 */
  double course{0.0};
  /** 前向速度占合速度的比例。 */
  double surge_ratio{1.0};
  /** 右向速度占合速度的比例。 */
  double sway_ratio{0.0};
};

/** @brief 一条离散轨迹及其任务标识。 */
struct ReferenceTrajectory
{
  /** 任务编号；同一任务的滚动局部段保持不变。 */
  std::uint64_t mission_id{0U};
  /** 是否为任务终端轨迹段。 */
  bool terminal{false};
  /** 按时间递增排列的轨迹采样点。 */
  std::vector<ReferencePoint> points;
};

}  // namespace usv_nmpc
