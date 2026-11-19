/**
 * @file reef_chart.hpp
 * @brief 生成不规则光滑凸暗礁并保存为海图障碍物。
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include <usv_map/parser.hpp>
#include <tinyxml2.h>

namespace usv_map_tools
{
/** @brief 局部东、北坐标系中的暗礁轮廓，位置单位 m。 */
struct Reef
{
  int id{0};
  usv_map::Point center;
  // 逆时针密集采样的光滑轮廓；末点重复首点以闭合。
  std::vector<usv_map::Point> boundary;
};

/**
 * @brief 生成以指定点为面积重心的光滑凸暗礁。
 * @param[in] center 面积重心，局部东、北坐标，单位 m。
 * @param[in] seed 随机种子；相同参数生成相同轮廓。
 * @param[in] min_radius_m 边界距重心的最小允许半径，单位 m。
 * @param[in] max_radius_m 边界距重心的最大允许半径，单位 m。
 * @return 闭合暗礁轮廓；半径约束覆盖整条边界线段。
 * @throws std::invalid_argument 中心无效或半径范围无效。
 */
Reef make_reef(
  usv_map::Point center, uint64_t seed, double min_radius_m = 3.0, double max_radius_m = 5.0);

/** @brief 加载已有暗礁，并在 XML 海图中维护障碍物要素。 */
class ReefChart
{
public:
  /**
   * @brief 读取海图并恢复本工具保存的暗礁。
   * @param[in] input 源 XML 路径。
   * @throws std::exception 海图或已保存暗礁数据无效。
   */
  explicit ReefChart(const std::filesystem::path & input);
  /**
   * @brief 生成暗礁并分配编号，保存失败时回滚文档变更。
   * @param[in] center 面积重心，局部东、北坐标，单位 m。
   * @param[in] seed 轮廓生成的随机种子。
   * @param[in] min_radius_m 最小允许半径，单位 m。
   * @param[in] max_radius_m 最大允许半径，单位 m。
   * @param[in] output 输出 XML 路径。
   * @return 已分配编号的暗礁。
   * @throws std::exception 参数无效、轮廓超出海图边界、编号耗尽或保存失败。
   */
  Reef add(
    usv_map::Point center, uint64_t seed, double min_radius_m, double max_radius_m,
    const std::filesystem::path & output);
  /**
   * @brief 以临时文件和原子替换方式保存当前海图。
   * @param[in] output 输出 XML 路径。
   * @throws std::exception 创建目录、写入或替换文件失败。
   */
  void save(const std::filesystem::path & output);
  /** @brief 返回已加载及已保存的暗礁列表。 */
  const std::vector<Reef> & reefs() const {return reefs_;}
  /** @brief 返回局部原点经度，单位 °。 */
  double longitude() const {return chart_.origin_longitude;}
  /** @brief 返回局部原点纬度，单位 °。 */
  double latitude() const {return chart_.origin_latitude;}

private:
  usv_map::Point geographic(usv_map::Point point) const;
  usv_map::Point source(usv_map::Point point) const;

  usv_map::Chart chart_;
  tinyxml2::XMLDocument document_;
  tinyxml2::XMLElement * features_{nullptr};
  int next_id_{1};
  std::vector<Reef> reefs_;
};
}  // namespace usv_map_tools
