/**
 * @file berth_chart.hpp
 * @brief 泊位矩形生成、历史泊位恢复与海图保存。
 */

#ifndef USV_MAP_TOOLS__BERTH_CHART_HPP_
#define USV_MAP_TOOLS__BERTH_CHART_HPP_

#include <array>
#include <filesystem>
#include <vector>

#include <tinyxml2.h>

namespace usv_map_tools
{
/** @brief 平面坐标点；泊位几何使用局部东、北坐标，单位 m。 */
struct BerthPoint
{
  double x;
  double y;
};

/** @brief 矩形泊位，长轴方向由 yaw 指定。 */
struct Berth
{
  int id{0};
  BerthPoint center;
  double yaw;  ///< 长轴方向，以东为零、逆时针为正，单位 rad。
  double length;  ///< 长轴尺寸，必须为正，单位 m。
  double width;  ///< 短轴尺寸，必须为正，单位 m。
  /**
   * @brief 计算闭合矩形轮廓。
   * @return 五个局部米制顶点，末点重复首点。
   * @throws std::invalid_argument 位姿非有限或尺寸非正。
   */
  std::array<BerthPoint, 5> corners() const;
};

/** @brief 在 XML 海图中维护泊位，并以原子替换方式保存。 */
class BerthChart
{
public:
  /**
   * @brief 读取源图；可将已有输出中的泊位合并到基准海图。
   * @param[in] input 输入海图路径。
   * @param[in] base 基准海图路径；为空时直接使用输入海图。
   * @throws std::exception 文件或泊位数据无效。
   */
  explicit BerthChart(
    const std::filesystem::path & input, const std::filesystem::path & base = {});
  /**
   * @brief 分配要素编号并保存泊位，写入失败时回滚文档变更。
   * @param[in] berth 待添加的局部米制泊位；id 由本函数分配。
   * @param[in] output 输出 XML 路径。
   * @return 已分配编号的泊位。
   * @throws std::exception 泊位越界、参数无效、编号耗尽或保存失败。
   */
  Berth add(Berth berth, const std::filesystem::path & output);
  /** @brief 返回已加载及已保存的泊位列表。 */
  const std::vector<Berth> & berths() const {return berths_;}
  /** @brief 返回局部原点经度，单位 °。 */
  double longitude() const {return longitude_;}
  /** @brief 返回局部原点纬度，单位 °。 */
  double latitude() const {return latitude_;}

private:
  BerthPoint to_geographic(BerthPoint point) const;
  BerthPoint to_source(BerthPoint point) const;
  void save(const std::filesystem::path & output);

  tinyxml2::XMLDocument document_;
  tinyxml2::XMLElement * features_{nullptr};
  std::array<double, 4> bounds_{};
  double longitude_{0.0};
  double latitude_{0.0};
  bool projected_{false};
  int next_id_{1};
  std::vector<Berth> berths_;
};
}  // namespace usv_map_tools

#endif  // USV_MAP_TOOLS__BERTH_CHART_HPP_
