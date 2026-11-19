/**
 * @file vessel_track.hpp
 * @brief 将船舶状态转换为 RFU 姿态和有界 ENU 轨迹。
 */

#ifndef USV_MAP__VESSEL_TRACK_HPP_
#define USV_MAP__VESSEL_TRACK_HPP_

#include <array>
#include <cstddef>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace usv_map
{

/** @brief 校验船舶融合状态，维护地图位姿和限长历史轨迹。 */
class VesselTrack
{
public:
  /**
   * @brief 配置历史轨迹抽样和位置预测校正。
   * @param[in] min_distance_m 轨迹相邻保留点的最小距离，须为有限正数，单位 m。
   * @param[in] max_points 历史轨迹点数上限，至少为 2。
   * @param[in] position_measurement_alpha 位置测量校正系数，范围 (0, 1]；1 表示直接采用测量值。
   * @throws std::invalid_argument 配置不满足范围要求。
   */
  explicit VesselTrack(
    double min_distance_m = 0.5, std::size_t max_points = 10000,
    double position_measurement_alpha = 1.0);

  /**
   * @brief 设置局部 ENU 坐标原点。
   * @param[in] latitude_rad 原点纬度，单位为 rad。
   * @param[in] longitude_rad 原点经度，单位为 rad。
   * @param[in] height_m 原点高度，单位为 m。
   * @return 原点变化并清空轨迹时返回 true；原点未变化时返回 false。
   * @throws std::invalid_argument 经纬度越界或任一参数非有限。
   */
  bool set_origin(double latitude_rad, double longitude_rad, double height_m = 0.0);

  /**
   * @brief 消费一个 /USV_state 消息并更新里程计姿态和历史轨迹。
   *
   * 状态固定包含 19 个字段：姿态 [0..2]、ENU 速度 [3..5]、测量纬经高 [6..8]、
   * RFU 速度 [9..11]、偏航角速度 [12]、真值纬经高 [13..15]、真值航向 [16]、
   * 融合状态 [17] 和累计时间 [18]。经纬度及角度单位为 rad，距离单位为 m。
   * @param[in] state 输入状态消息。
   * @param[in] receive_stamp 输出 ROS 消息使用的时间戳。
   * @return 更新成功时返回 true；原点未设置或状态非法时返回 false。
   */
  bool update(
    const std_msgs::msg::Float64MultiArray & state,
    const builtin_interfaces::msg::Time & receive_stamp);

  /** @brief 返回 map 坐标系中的最近位姿；设置原点及接收有效状态后使用。 */
  const geometry_msgs::msg::PoseStamped & pose() const {return pose_;}
  /** @brief 返回 map 坐标系中的限长历史轨迹。 */
  const nav_msgs::msg::Path & path() const {return path_;}
  /** @brief 返回最近一次更新是否改变了历史轨迹。 */
  bool path_changed() const {return path_changed_;}

private:
  double min_distance_m_;
  std::size_t max_points_;
  double position_measurement_alpha_;
  std::optional<double> last_state_time_;
  std::optional<double> last_position_filter_time_;
  std::optional<std::array<double, 2>> filtered_position_;
  std::optional<std::array<double, 3>> origin_;
  geometry_msgs::msg::PoseStamped pose_;
  nav_msgs::msg::Path path_;
  bool path_changed_{false};
};

}  // namespace usv_map

#endif  // USV_MAP__VESSEL_TRACK_HPP_
