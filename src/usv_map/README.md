# usv_map

维护者：susheng（963338707@qq.com）  
文档更新：2026-11-09  

## 功能

`usv_map` 负责 XML 格式 S-57 海图和地图坐标转换：

- 读取 `resource/` 中的 XML/S-57 海图；
- 发布海图 Marker、静态代价地图、航道几何数据和地图原点；
- 将 `/USV_state` 的纬经度状态转换为 ENU 地图坐标；
- 发布 `header.frame_id=map` 的 `/odom`、`/USV_path` 及 `map -> usv_body_rfu` TF；
- 提供 WAM-V 船体外观模型，并通过 `robot_state_publisher` 绑定到里程计 TF；
- 启动 RViz2，并按配置加载本地或在线天地图瓦片。

本包不启动仿真或硬件状态源。仿真时需另行启动 `usv_model_sim`；接入实船时由
`usv_hardware` 提供 `/USV_state` 和 `/cmd`，地图与控制器不依赖具体状态来源。

暗礁只在海图 Marker 中显示，不写入静态 `/chart_costmap`；感知到的暗礁由
`channel_navigation_manager` 叠加到动态航道代价地图。

## 目录

```text
usv_map/
├── config/                  # usv_map.yaml、water_map.yaml
├── urdf/                    # RViz 船体外观模型
├── meshes/wam_v/            # 船体网格与材质
├── include/usv_map/         # 解析、渲染、轨迹和参数接口
├── src/                     # usv_map_node、usv_odometry_node 和核心库
├── launch/usv_map.launch.py
├── rviz/usv_map.rviz
├── resource/                # suiku.xml、changjiang.xml、generated/
├── cache/tianditu/          # 本地天地图瓦片
├── tools/                   # 矢量制图和资源合并脚本
└── test/                    # 解析、渲染、轨迹测试
```

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
ros2 launch usv_map usv_map.launch.py \
  rviz:=true channel_navigation:=false simulate_perception:=false
```

这条命令适用于纯轨迹跟踪测试，关闭动态航道模块，保留静态地图与 RViz。
地图启动入口始终启动下列节点：

| 节点 | 职责 |
| --- | --- |
| `usv_map_node` | 发布静态 `/chart_costmap`、海图 Marker、航道几何及原点 `/fix` |
| `usv_odometry_node` | 接收 `/fix`、`/USV_state`，发布 `/odom`、`/USV_path` 和船体 TF |
| `map_origin_frame`、`map_gps_sensor_frame` | 发布 `map -> map_origin` 和 `map -> gps_sensor` 静态 TF |
| `usv_robot_state_publisher` | 加载船体 URDF 并发布显示模型 TF；不计算船体动力学 |

`rviz=true` 时另启动 `usv_map_rviz`；`channel_navigation=true` 时另启动
`channel_navigation_node`，并按 `simulate_perception` 决定是否启动
`obstacle_perception_sim_node`。船体动态显示和 `/odom` 更新需要外部 `/USV_state`；
本入口不会启动动力学仿真器、NMPC 或测试路径发布器。

仿真状态源在另一个终端单独启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
ros2 launch usv_model_sim usv_model.launch.py
```

### 启动参数

使用 `参数名:=值` 传入；下表为不传参数时的默认值，不代表上述纯跟踪命令的配置。

| 参数 | 默认值 | 取值与作用 |
| --- | --- | --- |
| `input_path` | `""`（空字符串） | 海图 XML 路径，建议使用绝对路径；非空时覆盖主 YAML 的 `input_path`，空时沿用 YAML。YAML 也为空才加载包内 `data/suiku.xml` |
| `rviz` | `true` | `true` 启动 RViz；`false` 不启动窗口，地图、里程计、船体显示 TF 仍运行 |
| `channel_navigation` | `true` | `true` 启动整个动态航道模块；`false` 不启动航道节点及其模拟感知，也不发布 `/channel_costmap`，但保留静态 `/chart_costmap` |
| `simulate_perception` | `true` | 仅在 `channel_navigation=true` 时有效；`true` 启动模拟障碍感知；`false` 关闭模拟感知，航道节点仍可接收外部 `/obstacle_detections` |
| `output` | `screen` | `screen` 将节点输出显示在终端；`log` 写入 ROS 日志文件；也传给航道模块 |

三个开关的常用组合如下，`simulate_perception` 不控制船体仿真器：

| 场景 | `rviz` | `channel_navigation` | `simulate_perception` |
| --- | --- | --- | --- |
| 纯轨迹跟踪，有界面 | `true` | `false` | `false`（航道关闭时不生效） |
| 纯轨迹跟踪，无界面 | `false` | `false` | `false`（航道关闭时不生效） |
| 动态航道 + 模拟感知 | `true` | `true` | `true` |
| 动态航道 + 外部感知 | `true` | `true` | `false` |

仅指定 `simulate_perception:=false` 属于最后一种配置，**不会关闭航道模块**。
外部感知通过 `/obstacle_detections` 发布 `usv_interfaces/msg/ObstacleDetectionArray`，
详见 [航道模块说明](../channel_navigation_manager/README.md)。不带参数启动时，三个开关均为 `true`。

完整命令示例：

```bash
# 无界面纯跟踪测试。
ros2 launch usv_map usv_map.launch.py \
  rviz:=false channel_navigation:=false simulate_perception:=false

# 航道代价与模拟障碍感知。
ros2 launch usv_map usv_map.launch.py \
  rviz:=true channel_navigation:=true simulate_perception:=true

# 航道代价与外部感知。
ros2 launch usv_map usv_map.launch.py \
  rviz:=true channel_navigation:=true simulate_perception:=false

# 指定其他海图，关闭航道模块，并将节点输出写入日志。
ros2 launch usv_map usv_map.launch.py \
  input_path:=/绝对路径/海图.xml rviz:=true \
  channel_navigation:=false simulate_perception:=false output:=log

# 仅查看启动参数，不启动节点。
ros2 launch usv_map usv_map.launch.py --show-args
```

## 配置

### 配置文件与生效方式

| 文件 | 用途及加载条件 |
| --- | --- |
| [config/usv_map.yaml](config/usv_map.yaml) | 始终加载；分别配置 `usv_map_node` 和 `usv_odometry_node` |
| [config/water_map.yaml](config/water_map.yaml) | `rviz=true` 时读取其中的卫星图层配置；同文件的其他字段供制图工具使用 |
| [rviz/usv_map.rviz](rviz/usv_map.rviz) | `rviz=true` 时加载显示配置，并按卫星图层配置生成临时 RViz 文件 |
| [channel_navigation.yaml](../channel_navigation_manager/config/channel_navigation.yaml) | `channel_navigation=true` 时由航道子启动文件加载，配置动态航道及模拟感知 |

启动入口从已安装包的 `share` 目录读取上述文件，目前没有 `config_file` 启动参数。
修改源码配置后重新构建对应包、重新加载环境，再重启地图入口：

```bash
cd /home/sus/USV_systems
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select usv_map
source install/setup.bash
```

若修改航道模块配置，将构建命令的包名改为 `channel_navigation_manager`。
`--symlink-install` 安装的配置若已链接到源码，修改后重启即可读取新值；不要依赖运行时
`ros2 param set` 更新地图或定时器。查询安装配置目录可用 `ros2 pkg prefix --share usv_map`。

`input_path` 的优先级为：**非空启动参数 > YAML 非空值 > 包内默认海图**。
其余 YAML 项不是启动参数，例如 `planning_resolution:=1.0` 不会覆盖节点的栅格分辨率，
应修改下述 YAML 字段。节点名和 `ros__parameters` 层级需要保留。

### 海图节点参数

以下为 `usv_map.yaml` 中 `usv_map_node.ros__parameters` 的当前配置值，单位和含义如下。

| 参数 | 当前值 | 单位、取值及作用 |
| --- | --- | --- |
| `input_path` | `""` | 海图 XML 文件路径；空值时使用默认海图，非空启动参数可覆盖它 |
| `origin_longitude` | `106.716974974` | 十进制度；地图原点经度，用于海图 ENU 转换和 `/fix` |
| `origin_latitude` | `29.506912001` | 十进制度；地图原点纬度，与经度共同定义 ENU 原点 |
| `line_width` | `0.5` | m；RViz 矢量线条宽度，不表示航道实际宽度 |
| `point_size` | `0.5` | m；RViz 点要素显示大小 |
| `focus_navigation_features` | `true` | 显示航道、泊位等导航要素及陆地、禁行区、暗礁轮廓；`false` 时额外显示其他海图要素 |
| `planning_resolution` | `2.0` | m/格，需大于 0；静态代价地图栅格边长，越小内存开销越大 |
| `planning_max_cells` | `20000000` | 栅格数量预算，需为正整数；预计超出时自动增大栅格边长，实际分辨率可能大于配置值 |
| `planning_crop_width_fraction` | `1.0` | `(0, 1]`；保留地图东西方向范围的比例，`1.0` 为完整范围 |
| `planning_crop_height_fraction` | `1.0` | `(0, 1]`；保留地图南北方向范围的比例；两项均从海图左下角起裁切，只改变栅格范围 |
| `obstacle_inflation` | `4.0` | m；静态陆地和禁行区膨胀半径，暗礁不写入静态代价地图 |
| `channel_corridor_width` | `300.0` | m；航道线向两侧扩展的**半径**，两侧总宽约 600 m；`0` 仅栅格化线本身。只用于静态低代价走廊，动态通行宽度由 XML 和航道模块定义 |
| `channel_outside_cost` | `60` | 栅格默认代价，建议 `0～99`（`100` 留给障碍）；实现将输入截在 `[-1, 99]`，`-1` 表示未知。水域和航道可覆盖为 `0`，陆地等覆盖为 `100` |

关闭 `channel_navigation` 不影响上述静态地图参数；`channel_corridor_width` 也不控制动态航道模块是否启动。

### 里程计节点参数

以下为 `usv_map.yaml` 中 `usv_odometry_node.ros__parameters` 的当前配置值。

| 参数 | 当前值 | 单位、取值及作用 |
| --- | --- | --- |
| `odom_publish_period_sec` | `0.05` | s，需大于 0；处理最新船体状态并发布 `/odom` 与动态 TF 的周期，当前最多 20 Hz，收到新状态才更新 |
| `publish_period_sec` | `1.0` | s，需大于 0；发布 `/USV_path` 历史轨迹的周期，当前 1 Hz，不决定 `/odom` 频率 |
| `path_min_distance_m` | `0.5` | m，需大于 0；新位置距上一个保存点至少达到此距离才加入历史轨迹 |
| `path_max_points` | `10000` | 点，整数且至少为 `2`；历史轨迹容量，超出后移除最早的点 |
| `position_measurement_alpha` | `0.2` | `(0, 1]`；ENU 速度外推后对位置测量残差的校正系数，越小越平滑，`1.0` 表示直接采用测量位置 |

这组参数不改变坐标约定：世界系 `x` 向东、`y` 向北；RFU 本体系 `x` 向右、`y` 向前。
前向速度 `u` 对应 `/odom.twist.twist.linear.y`，右向速度 `v` 对应
`/odom.twist.twist.linear.x`。航向为北零、西正，东向为 `-π/2`。

### RViz 卫星图层

修改 `water_map.yaml` 中对应的顶层字段；这些选项只影响显示。

| 配置项 | 当前值或回退值 | 作用 |
| --- | --- | --- |
| `rviz_satellite_mode` | `local` | `local` 读取本地瓦片；`online` 在线请求；`disabled` 关闭卫星图层，保留海图矢量显示 |
| `tile_cache` | `cache/tianditu` | 本地瓦片目录；相对路径优先查找源码包和工作空间，再回退安装包目录，也可填写绝对路径 |
| `rviz_satellite_layer` | `img_w` | 本地瓦片文件前缀；文件名格式为 `img_w_{z}_{x}_{y}.tile` |
| `zoom` | `16` | 本地瓦片缩放级别；若显式设置 `rviz_satellite_zoom`，优先采用该值 |
| `tianditu_api_key` | 按部署设置 | `online` 模式的天地图密钥；此项为空时才读取环境变量 `TIANDITU_TK` |

`local` 模式未找到匹配瓦片时关闭卫星图层，静态海图仍显示；`rviz=false` 时不读取这些显示配置。

### 查询生效值

地图节点启动后，在已加载工程环境的另一终端查询，例如：

```bash
ros2 param get /usv_map_node planning_resolution
ros2 param get /usv_map_node input_path
ros2 param get /usv_odometry_node odom_publish_period_sec
```

参数查询中的 `input_path` 为空时表示选择内置默认路径，不代表没有加载海图。
实际栅格分辨率以 `/chart_costmap.info.resolution` 为准，可能受栅格数量预算影响。

## ROS 接口

| 话题/TF | 类型 | 说明 |
| --- | --- | --- |
| `/chart_markers` | `visualization_msgs/MarkerArray` | 海图要素、陆地、障碍和航道方向 |
| `/chart_costmap` | `nav_msgs/OccupancyGrid` | 静态代价地图 |
| `/chart_lane_data` | `std_msgs/String` | 航道和暗礁的 JSON 几何数据 |
| `/chart_metadata` | `std_msgs/String` | 海图元数据 |
| `/fix` | `sensor_msgs/NavSatFix` | 地图原点，纬度/经度为度、高程为米 |
| `/USV_state` | `std_msgs/Float64MultiArray` | 无人船状态输入 |
| `/odom` | `nav_msgs/Odometry` | ENU 地图位姿和 RFU 本体速度 |
| `/USV_path` | `nav_msgs/Path` | 地图坐标系轨迹 |
| `map -> odom` | 静态 TF | 地图和里程计原点重合 |
| `odom -> usv_body_rfu` | 动态 TF | 船体姿态 |

`/USV_state` 必须有 19 个有限值，顺序为：姿态角 3、导航系速度 3、测量纬度/经度/高度
3、本体右/前/上速度 3、偏航角速度 1、真值纬度/经度/高度 3、真值航向 1、状态位 1、时间 1。
角度和状态中的经纬度使用弧度；`/fix` 使用度。

航向约定为 `[-π, π]`：0 指北，`-π/2` 指东，`+π/2` 指西，±π 指南。RViz2 中北向上、
南向下、西向左、东向右。

## 制图工具

```bash
python3 src/usv_map/tools/water_map_maker.py --help
python3 src/usv_map/tools/merge_generated_chart.py --help
```

矢量图用于水体/陆地分割，卫星图仅用于 RViz2 显示；生成的 XML 放在
`resource/generated/`。地图比例元数据不会对坐标进行缩放。

## 测试

```bash
colcon test --packages-select usv_map
colcon test-result --verbose
```
