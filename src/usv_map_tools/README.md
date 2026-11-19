# usv_map_tools

维护者：susheng（963338707@qq.com）  
文档更新：2026-10-31

## 功能

`usv_map_tools` 是 RViz2 海图编辑和测试工具包：

- 用 RViz2 两个点框选矩形禁行区；
- 在 RViz2 中指定中心和朝向设计泊位；
- 用 RViz2 单击位置生成不规则凸形暗礁；
- 读取 GNSS 路径，生成顺时针/逆时针双向航道；
- 将 XML 海图转换为 S-57 `*.000` 文件。

## 目录和配置

```text
usv_map_tools/
├── config/                    # 编辑器和航道参数
├── launch/                    # 三个 RViz2 启动入口
├── rviz/area_editor.rviz
├── src/                       # 编辑器、航道和 S-57 实现
├── matlab_nv_lse_pe_gnss.txt  # GNSS 测试路径
└── test/                      # 编辑器和航道测试
```

主要配置：

- `config/area_editor.yaml`：禁行区输入/输出 XML；
- `config/berth_designer.yaml`：泊位长、宽和输出 XML；
- `config/reef_designer.yaml`：暗礁最小/最大半径和输出 XML；
- `config/gnss_path.yaml`：GNSS 文件、航道宽度、岸边净距和曲线参数。

## RViz2 编辑

### 禁行区

```bash
ros2 launch usv_map_tools area_editor.launch.py
```

在 RViz2 固定坐标系为 `map` 时：

1. 使用 **2D Pose Estimate** 发布第一个角点；
2. 使用 **2D Goal Pose** 发布对角点；
3. 工具自动保存矩形到 `reservoir_test_blocked.xml`。

可用参数：

```bash
ros2 launch usv_map_tools area_editor.launch.py \
  input_chart:=/绝对路径/输入.xml \
  output_chart:=/绝对路径/输出.xml
```

### 泊位

```bash
ros2 launch usv_map_tools berth_designer.launch.py
```

在 RViz2 使用 **2D Nav Goal**，从泊位中心向船首方向拖动；每次拖动保存一个泊位。
默认尺寸为 10 m × 3 m，可在 `berth_designer.yaml` 修改。

### 暗礁

```bash
ros2 launch usv_map_tools reef_designer.launch.py
```

在 RViz2 使用 **Publish Point** 单击暗礁重心；工具生成封闭、不规则、凸形边界并保存 XML。
默认边界半径为 3～5 m，可在 `reef_designer.yaml` 修改。暗礁编辑器会实时刷新
`/chart_markers` 和 `/chart_costmap`；航道管理器是否把它加入动态代价由感知节点决定。

三个启动文件都支持 `input_chart:=...`、`output_chart:=...` 和 `rviz:=false`。

## GNSS 航道

三个编辑启动文件都会同时启动 GNSS 路径发布器。独立使用时，修改 `config/gnss_path.yaml`
后启动对应编辑器即可。

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/test_tools/gnss_path` | `nav_msgs/Path` | 原始 GNSS 路径 |
| `/test_tools/clockwise_route` | `nav_msgs/Path` | 平滑闭合中心线 |
| `/test_tools/fairway_clockwise` | `nav_msgs/Path` | 顺时针单侧航道 |
| `/test_tools/fairway_counterclockwise` | `nav_msgs/Path` | 逆时针单侧航道 |
| `/test_tools/fairway` | `visualization_msgs/MarkerArray` | 航道边界和方向标记 |

当前默认单侧宽度为 15 m，外沿净距为 5 m，中心线向闭合环内收窄 5 m；
`inward_bow_m` 和 `bottom_rounding_m` 用于控制弧度。生成结果按海图 XML 的
`TSSLPT/TSSBND/TSELNE` 结构追加到输出文件。

## 其他接口

| 话题 | 用途 |
| --- | --- |
| `/test_tools/no_go_regions` | 框选禁行区 Marker |
| `/test_tools/berths` | 泊位 Marker |
| `/test_tools/berth_pose` | 泊位位置和朝向输入 |
| `/test_tools/reefs` | 暗礁 Marker |
| `/test_tools/reef_point` | 暗礁重心输入 |
| `/fix` | 编辑器发布的海图原点 |

## XML 转 S-57

```bash
ros2 run usv_map_tools chart_to_s57 \
  src/usv_map/resource/suiku.xml \
  /绝对路径/output.000
```

## 测试

```bash
colcon test --packages-select usv_map_tools
colcon test-result --verbose
```
