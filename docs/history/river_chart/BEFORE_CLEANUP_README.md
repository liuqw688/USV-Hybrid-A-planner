# river_chart

## 详细功能与参数文档

- [FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)：节点启动、功能/函数流程图、话题与包间关系、逐函数职责、当前实现边界。
- [PARAMETERS.md](PARAMETERS.md)：配置值、参数作用及增大/减小的影响。
- [工作空间总索引](../../PACKAGE_DOCUMENTATION_INDEX.md)：整条运行链、操作与验证。



这是一个 ROS 2 Jazzy `ament_python` 功能包，用于读取 GDAL/S-57 XML 导出文件，并在 RViz2 中显示长江电子海图元素。

推荐使用 `visualization_msgs/msg/MarkerArray` 作为地图表达：它保留了航道边界、等深线、锚地、泊位、航标、桥梁等 S-57 要素及其分组语义。程序另外支持发布 `nav_msgs/msg/OccupancyGrid`，但该栅格只是几何诊断结果，不能直接当作可航行性或自主导航地图。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/river_chart_ros2
colcon build --symlink-install --packages-select river_chart
source install/setup.bash
```

## 在 RViz2 中运行

```bash
ros2 launch river_chart river_chart.launch.py
```

默认输入文件随功能包安装在 `share/river_chart/data/ecdis_data.xml`，不依赖工作空间绝对路径；也可以指定其他 XML：

```bash
ros2 launch river_chart river_chart.launch.py \
  input_path:=/path/to/ecdis_data.xml
```

坐标会转换为以 `map` 为坐标系的局部东-北平面，单位为米。默认原点是 XML 全部 Bounds 的中心；为多个海图单元保持一致时可显式指定：

```bash
ros2 launch river_chart river_chart.launch.py \
  origin_longitude:=108.4425 origin_latitude:=30.8250
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_path` | 包内安装的 `data/ecdis_data.xml` | XML 文件路径 |
| `frame_id` | `map` | Marker 和 RViz 固定坐标系 |
| `origin_longitude` / `origin_latitude` | 自动 | 局部坐标原点（WGS-84 度） |
| `scale` | `1.0` | 坐标缩放，保持 1.0 即米 |
| `line_width` | `3.0` | 线宽（米） |
| `point_size` | `2.0` | 点要素尺寸（米） |
| `fill_polygons` | `false` | 对简单区域增加半透明三角扇填充 |
| `show_land_boundaries` | `false` | 是否显示 `LNDARE`/`LNDRGN` 陆地边界 |
| `focus_navigation_features` | `true` | 显示航道、浮标、桥梁、泊位和等深线，隐藏其他辅助图层 |
| `planning_resolution` | `10.0` | 规划栅格分辨率（米） |
| `planning_max_cells` | `600000` | 规划栅格最大单元数，超出时自动降低分辨率 |
| `obstacle_inflation` | `10.0` | 陆地边界的安全膨胀半径（米） |
| `channel_corridor_width` | `300.0` | 航道线两侧的可通行走廊半径（米） |
| `publish_channel_direction` | `false` | 是否发布方向箭头；默认关闭 |
| `channel_outside_cost` | `60` | 航道外低速通行带代价；陆地和动态障碍物固定为 `100` |
| `publish_labels` | `false` | 发布要素名称文字 |
| `publish_occupancy_grid` | `false` | 发布几何栅格诊断图 |
| `grid_resolution` | `5.0` | 栅格分辨率（米） |
| `reload_period` | `0.0` | 定期重载秒数，0 表示只加载一次 |
| `rviz` | `true` | 是否同时启动 RViz2 |

## ROS 话题

- `/chart_markers`：`visualization_msgs/msg/MarkerArray`，主要矢量海图显示。
- `/chart_metadata`：`std_msgs/msg/String`，JSON 格式的来源、范围、原点和要素统计。
- `/chart_occupancy`：`nav_msgs/msg/OccupancyGrid`，可选的几何栅格诊断结果。
- `/chart_static_obstacles`：`nav_msgs/msg/OccupancyGrid`，陆地多边形及安全膨胀后的静态障碍栅格；桥梁、泊位、浮标保持可通行。
- `/chart_costmap`：`nav_msgs/msg/OccupancyGrid`，航道内低代价、航道外低速高代价、陆地不可通行的规划栅格。
- `/chart_channel_direction`：`visualization_msgs/msg/MarkerArray`，可选的航道方向箭头，默认不发布。
- `/chart_lane_data`：`std_msgs/msg/String`，一次性发布的轻量级 TSSLPT 多边形和
  ORIENT 数据，供 `channel_navigation_manager` 动态判断本船航道；使用
  Reliable + Transient Local，新启动的订阅者也能收到。
- `/future_obstacles`：`geometry_msgs/msg/PoseArray` 输入，未来雷达/AIS/感知模块将障碍物位置发布到此话题后，会叠加到 `/chart_costmap`。

默认不绘制 `M_NPUB`、`M_NSYS`、`M_QUAL` 制图元数据产生的瓦片矩形，也不绘制导出器未分类的 `Generic` 几何和锚地 `ACHARE`；这些要素仍保留在解析统计中。
`TSSBND`、`TSELNE`、`TSSLPT` 航道/交通分道标识统一使用绿色。

默认聚焦显示的分组为：绿色航道/交通分道 `TSSBND`、`TSELNE`、`TSSLPT`，蓝色等深线 `DEPCNT`，黄色浮标 `BOYSPP`，红色桥梁 `BRIDGE`，洋红色泊位 `BERTHS`。代价地图中航道走廊为 `0`（可通行），航道外为 `60`（低速通行），不会使用 `-1` 未知值。

发布器使用 `Reliable + Transient Local` QoS，RViz2 后启动也能收到最近一次地图。

## 数据和导航限制

当前 XML 共包含 4 个地图单元、757 个带几何的要素。另有 27 个 `SOUNDG` 要素没有坐标，无法从该 XML 恢复水深点。

若要用于真实船舶导航，还需要依据 S-57 属性解释 `DEPARE`/`DEPCNT` 水深、陆地和禁限航区，结合船舶吃水、富裕水深（UKC）、坐标基准和 GPS 到 `map` 的 TF。当前包的局部坐标主要用于 RViz 可视化；生产系统建议采用 ENU/UTM 或完整地理坐标转换链路。

## 测试

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select river_chart
colcon test-result --verbose
```

当前测试结果：11 tests，0 errors，0 failures。
