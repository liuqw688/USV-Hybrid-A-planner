/**
 * @file coordinate_conventions.hpp
 * @brief 坐标系、航向角和里程计数据的转换约定。
 *
 * 世界坐标采用 ENU：x 向东、y 向北、z 向上。船体坐标采用 RFU：x 向右、
 * y 向前、z 向上。导航航向以北为零，向西为正。
 */

#pragma once

#include <cmath>

namespace usv_nmpc::coordinates
{

/** @brief 圆周率，单位为 rad。 */
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 将角度归一化到 [-pi, pi]。
 * @param[in] angle 输入角度，单位为 rad。
 * @return 等价的归一化角度，单位为 rad。
 */
inline double wrapAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

/**
 * @brief 计算从 from 转到 to 的最短有符号角度差。
 * @param[in] from 起始角，单位为 rad。
 * @param[in] to 目标角，单位为 rad。
 * @return 最短有符号角度差，单位为 rad。
 */
inline double shortestAngularDistance(double from, double to)
{
  return wrapAngle(to - from);
}

/**
 * @brief 将角度展开为最接近 reference 的等价角。
 * @param[in] angle 待展开角度，单位为 rad。
 * @param[in] reference 参考角，单位为 rad。
 * @return 与 angle 等价且最接近 reference 的角度，单位为 rad。
 */
inline double unwrapNear(double angle, double reference)
{
  return reference + shortestAngularDistance(reference, angle);
}

/**
 * @brief 沿最短角度方向插值。
 * @param[in] from 起始角，单位为 rad。
 * @param[in] to 目标角，单位为 rad。
 * @param[in] ratio 插值比例。
 * @return 插值角度，单位为 rad。
 */
inline double interpolateHeading(double from, double to, double ratio)
{
  return from + ratio * shortestAngularDistance(from, to);
}

/**
 * @brief 归一化 /USV_state 中与 map/odom 相同定义的导航航向。
 * @param[in] heading 北向为零、向西为正的航向，单位为 rad。
 * @return 归一化航向，单位为 rad。
 */
inline double usvStateHeadingToMap(double heading)
{
  return wrapAngle(heading);
}

/**
 * @brief 将 RFU 的前向速度映射为 NMPC 的 u。
 * @param[in] rfu_forward 船体前向速度，单位为 m/s。
 * @return NMPC 纵向速度，单位为 m/s。
 */
inline double usvStateForwardVelocityToBody(double rfu_forward)
{
  return rfu_forward;
}

/**
 * @brief 将 RFU 的右向速度映射为 NMPC 的 v。
 * @param[in] rfu_starboard 船体右向速度，单位为 m/s。
 * @return NMPC 横向速度，单位为 m/s。
 */
inline double usvStateLateralVelocityToStarboard(double rfu_starboard)
{
  return rfu_starboard;
}

/**
 * @brief 将 RFU 状态中的偏航角速度映射到世界坐标约定。
 * @param[in] yaw_rate 绕船体 z 轴的角速度，单位为 rad/s。
 * @return 按北零、西正航向定义的角速度，单位为 rad/s。
 */
inline double usvStateYawRateToMap(double yaw_rate)
{
  return yaw_rate;
}

/**
 * @brief 从平面 RFU 四元数中读取船首导航航向。
 * @param[in] qz 四元数 z 分量。
 * @param[in] qw 四元数 w 分量。
 * @return 北向为零、向西为正的航向，单位为 rad。
 */
inline double planarQuaternionToMapHeading(double qz, double qw)
{
  const double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
  return wrapAngle(yaw);
}

/**
 * @brief 从四元数中提取绕竖直轴的 yaw。
 * @param[in] x 四元数 x 分量。
 * @param[in] y 四元数 y 分量。
 * @param[in] z 四元数 z 分量。
 * @param[in] w 四元数 w 分量。
 * @return 按标准 ENU yaw 定义的角度，单位为 rad。
 */
inline double quaternionYaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/**
 * @brief 从 /odom 的 RFU 姿态四元数中读取船首导航航向。
 * @param[in] qx 四元数 x 分量。
 * @param[in] qy 四元数 y 分量。
 * @param[in] qz 四元数 z 分量。
 * @param[in] qw 四元数 w 分量。
 * @return 北向为零、向西为正的船首航向，单位为 rad。
 */
inline double rfuOdometryHeading(double qx, double qy, double qz, double qw)
{
  return wrapAngle(quaternionYaw(qx, qy, qz, qw));
}

}  // namespace usv_nmpc::coordinates
