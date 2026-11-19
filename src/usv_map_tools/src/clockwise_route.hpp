/**
 * @file clockwise_route.hpp
 * @brief 闭合平滑航线、安全距离校验与双向靠右航道生成。
 */

#pragma once

#include <vector>

namespace usv_map {struct Chart;}

namespace usv_map_tools
{
/** @brief map 坐标系平面位置，x 向东、y 向北，单位 m。 */
struct RoutePoint
{
  double x;
  double y;
};

/** @brief 航线位置及切向姿态；yaw 以东为零、逆时针为正，单位 rad。 */
struct RoutePose
{
  RoutePoint position;
  double yaw;
};

/**
 * @brief 在参考点凸包内生成顺时针闭合 C2 样条并采样。
 * @param[in] reference 局部米制参考点，允许包含重复记录。
 * @param[in] spacing_m 目标采样间距，必须为有限正数，单位 m。
 * @return 沿顺时针排列的位姿，末点与首点重合。
 * @throws std::invalid_argument 间距无效或参考点无法构成有效凸包。
 */
std::vector<RoutePose> make_clockwise_route(
  const std::vector<RoutePoint> & reference, double spacing_m);

/** @brief 已完成连续曲线和采样折线净距校验的航线。 */
struct SafeRoute
{
  std::vector<RoutePose> poses;
  // 连续样条与发布折线的净距下界，单位米。
  double minimum_clearance_m;
  // 保留周期三次 B 样条控制点，用于计算规则法向偏移。
  std::vector<RoutePoint> controls;
};

/** @brief 单段通航区域及其切向姿态，yaw 遵循 RoutePose 的约定。 */
struct FairwaySection
{
  std::vector<RoutePoint> polygon;
  double yaw;
  bool clockwise;
};

/** @brief 双向靠右航道及其边界、中心线和安全净距，距离单位 m。 */
struct Fairway
{
  // 各断面数组沿顺时针中心线排列，含闭合点。
  std::vector<RoutePose> center;
  std::vector<RoutePoint> inner, outer;
  std::vector<RoutePose> clockwise, counterclockwise;
  std::vector<FairwaySection> sections;
  double lane_width_m;
  double centerline_clearance_m;
  double minimum_clearance_m;
};

/**
 * @brief 生成内圈顺时针、外圈逆时针的双向靠右航道。
 * @param[in] route 含周期三次样条控制点的安全中心航线。
 * @param[in] lane_width_m 单向航道宽度，单位 m。
 * @param[in] spacing_m 目标几何采样间距，单位 m。
 * @param[in] chart 用于安全校验的局部米制海图。
 * @param[in] edge_clearance_m 航道边缘要求的最小净距，单位 m。
 * @param[in] centerline_inset_m 中心线内缩量，正值朝环内，单位 m。
 * @return 含中心线、边界和分段多边形的航道。
 * @throws std::exception 参数无效、偏移产生尖点或完整航道不满足安全净距。
 */
Fairway make_fairway(
  const SafeRoute & route, double lane_width_m, double spacing_m,
  const usv_map::Chart & chart, double edge_clearance_m, double centerline_inset_m = 0.0);

/**
 * @brief 在已知水域内拟合 GNSS 包络，生成顺时针凸闭合 C2 航线。
 * @param[in] reference GNSS 记录转换后的局部米制参考点。
 * @param[in] spacing_m 目标采样间距，单位 m。
 * @param[in] chart 用于水域覆盖和障碍净距校验的海图。
 * @param[in] clearance_m 航线要求的最小净距，单位 m。
 * @param[in] inward_bow_m 东侧控制点的内弯量，单位 m。
 * @param[in] bottom_rounding_m 南侧控制点向环心的移动量，单位 m。
 * @return 含采样位姿、控制点和净距下界的安全航线。
 * @throws std::exception 输入无效或有限次形状搜索后仍无安全航线。
 */
SafeRoute make_safe_clockwise_route(
  const std::vector<RoutePoint> & reference, double spacing_m,
  const usv_map::Chart & chart, double clearance_m,
  double inward_bow_m = 0.0, double bottom_rounding_m = 0.0);
}  // namespace usv_map_tools
