/**
 * @file bspline_reference.hpp
 * @brief 将 USV B 样条消息采样为 NMPC 可用的时空参考。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "usv_interfaces/msg/usv_bspline.hpp"
#include "usv_nmpc_controller/reference_trajectory.hpp"

namespace usv_nmpc
{

/**
 * @brief 已校验的时标化位置与航向样条。
 *
 * 样条数据不可变，可在 NMPC 预测时刻直接求值。端点导数取自样条本身，
 * 不要求末端速度为零。
 */
class BsplineReference
{
public:
  BsplineReference() = default;

  /**
   * @brief 校验并装载 B 样条消息。
   * @param[in] message 待装载的样条消息；次数为 1..5，节点向量单调不减。
   * @param[out] error 输出校验失败原因；成功时清空。
   * @return 消息有效时返回 true；失败时清除已装载的样条。
   */
  bool configure(
    const usv_interfaces::msg::UsvBspline & message, std::string & error);

  /**
   * @brief 按相对起始时间采样一个参考状态。
   * @param[in] elapsed_s 相对样条起始时刻的时间，单位为 s；负值按起点处理。
   * @param[out] reference 输出的位置、航向及其导数。
   * @return 样条有效且计算结果有限时返回 true。
   * @note 超过样条时长时，位置和航向沿末端一阶导数线性外推。
   */
  bool sample(double elapsed_s, RefPoint & reference) const;

  /**
   * @brief 为 NMPC 预测时域生成等时间间隔参考状态。
   * @param[in] elapsed_s 当前相对样条起始时间，单位为 s。
   * @param[in] horizon_steps 预测步数。
   * @param[in] dt 相邻预测点的时间间隔，单位为 s。
   * @param[out] references 输出参考状态序列。
   * @return 参数有效且所有预测时刻均可求值时返回 true；失败时清空输出。
   * @note 输出 horizon_steps + 1 个状态，超过样条末端时采用 sample 的外推规则。
   */
  bool buildHorizon(
    double elapsed_s, int horizon_steps, double dt,
    std::vector<RefPoint> & references) const;

  /**
   * @brief 将样条离散为供诊断和可视化使用的轨迹。
   * @param[in] sample_period_s 离散时间间隔，单位为 s。
   * @param[out] trajectory 输出的离散轨迹。
   * @param[out] error 输出构造失败原因；成功时清空。
   * @return 样条有效且采样成功时返回 true。
   */
  bool buildTrajectory(
    double sample_period_s, ReferenceTrajectory & trajectory,
    std::string & error) const;

  /** @brief 返回样条有效时长，单位为 s；未配置时返回 0。 */
  double duration() const;

  /** @brief 返回当前对象是否已装载有效样条。 */
  bool valid() const {return static_cast<bool>(data_);}

private:
  struct Data;
  std::shared_ptr<const Data> data_;
};

/**
 * @brief 用 de Boor 算法采样位置和航向样条。
 *
 * UsvBspline 消息没有 frame_id；调用方需确保控制点与 /odom 同属 ENU
 * 米制坐标系。失败时返回 false，并将原因写入 error。
 * @param[in] message 输入的 B 样条消息。
 * @param[in] sample_period_s 离散时间间隔，单位为 s。
 * @param[out] trajectory 输出的离散参考轨迹。
 * @param[out] error 输出失败原因；成功时清空。
 * @return 消息有效且构造成功时返回 true。
 */
bool buildReferenceFromBspline(
  const usv_interfaces::msg::UsvBspline & message,
  double sample_period_s,
  ReferenceTrajectory & trajectory,
  std::string & error);

}  // namespace usv_nmpc
