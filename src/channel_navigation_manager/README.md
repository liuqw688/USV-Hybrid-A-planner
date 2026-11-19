# channel_navigation_manager

维护者：susheng（963338707@qq.com）  
文档更新：2026-11-15

## 功能

本包把静态海图和航道方向转换为动态航道代价地图，并把感知障碍物单独叠加：

- 复用 `/chart_costmap` 的尺寸、分辨率和原点；
- 同向航道代价从中心 0 递增到侧边 8，航道外水域代价 20，对向航道为 20～28；
- 接收 `/path` 航点时由首个有效航段选择 CW/CCW，并保持该通行方向；收到新 `/goal_pose` 后切回单点目标方向判断。
- 陆地和静态障碍代价 100；
- 仿真感知以本船为中心 60 m 探测暗礁；
- 感知障碍按“消息半径 + 2 m”生成不可通行圆，代价为 100；
- 感知消息为空或超时后，恢复航道/水域的基础代价；
- 用 RViz2 Marker 显示感知圆和方向箭头。

感知节点只发布标签、方位、距离和大小，不发布障碍物点集。未来接入真实传感器时，
只需让传感器适配器发布 `usv_interfaces/msg/ObstacleDetectionArray`。

## 目录

```text
channel_navigation_manager/
├── config/channel_navigation.yaml
├── launch/river_navigation.launch.py
├── include/channel_navigation_manager/ # 航道和感知几何接口
├── src/
│   ├── channel_navigation_node.cpp     # 代价地图节点
│   └── obstacle_perception_sim_node.cpp # 仿真感知节点
└── test/                                # 几何和感知流水线测试
```

## 启动

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py
```

默认同时启动：

- `channel_navigation_node`：生成 `/channel_costmap`；
- `obstacle_perception_sim_node`：读取海图暗礁真值，模拟传感器探测。

接入真实感知时关闭仿真节点：

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py simulate_perception:=false
```

地图、里程计和航道管理器通常由 `ros2 launch usv_map usv_map.launch.py` 一起启动。

## 配置

配置文件：[config/channel_navigation.yaml](config/channel_navigation.yaml)

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `same_direction_cost` | 0 | 同向航道 |
| `opposite_direction_cost` | 20 | 对向航道 |
| `lane_center_cost` | 8 | 航道中心至两侧边缘的附加代价，0 关闭居中偏好 |
| `open_water_cost` | 20 | 航道外水域，与对向航道基础代价相同 |
| `obstacle_cost` | 100 | 陆地和障碍 |
| `perception_radius_m` | 60.0 | 仿真感知半径 |
| `detection_inflation_m` | 2.0 | 感知障碍额外膨胀 |
| `detection_timeout_s` | 3.0 | 感知帧超时恢复基础代价 |
| `direction_tolerance_deg` | 90.0 | 航道方向选择阈值 |
| `direction_arrow_length_m` | 12.0 | 方向箭头长度 |
| `publish_rate_hz` | 5.0 | 输出刷新频率上限 |

航道内代价为 `方向基础代价 + round(lane_center_cost × 横断面归一化偏心量)`。
偏心量在中心为 0、侧边为 1，沿 S-57 航道方向的法线求当前横断面；凹多边形使用包含
查询点的交点区间，航段端盖不作为侧岸。中心指所属单向航道的中心。栅格为整数代价，
中心附近存在低代价平带；这是可绕障的软偏好，不保证每个轨迹点严格压在中心线上。
重叠航段取较低代价，陆地与障碍始终最后覆盖。
参数必须满足 `same_direction_cost + lane_center_cost < open_water_cost` 且
`opposite_direction_cost + lane_center_cost < 75`，避免破坏通行优先级或进入规划器禁行区。
修改配置后重启航道节点生效。

## ROS 接口

输入：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/chart_costmap` | `nav_msgs/OccupancyGrid` | 静态地图尺寸、原点和基础障碍 |
| `/chart_lane_data` | `std_msgs/String` | 航道方向和暗礁几何 JSON |
| `/odom` | `nav_msgs/Odometry` | 本船位置和姿态 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 目标方向；没有目标时参考船体朝向 |
| `/obstacle_detections` | `usv_interfaces/ObstacleDetectionArray` | 感知障碍完整快照 |

输出：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/channel_costmap` | `nav_msgs/OccupancyGrid` | 航道方向和感知障碍叠加后的代价 |
| `/channel_navigation_markers` | `visualization_msgs/MarkerArray` | 航道轮廓和方向箭头 |
| `/perception_markers` | `visualization_msgs/MarkerArray` | 仿真感知圆和已感知障碍 |

方向箭头使用黄色表示低代价方向，红色表示高代价方向。船舶主航道靠右航行，左侧为对向航道；
箭头方向由本船到目标点的方向（无目标时由船体朝向）选择，并沿当前闭合航道保持一致。

## 感知消息规则

`ObstacleDetectionArray` 的方位相对 `usv_body_rfu`：船首为 0，右侧为 `+π/2`，左侧为
`-π/2`。航道节点根据观测时刻 TF 计算障碍物圆心，最终禁行半径为：

```text
禁行半径 = radius_m + detection_inflation_m
```

例如障碍半径 5 m 时，代价圆半径为 7 m。收到空数组立即清除临时障碍；静态陆地和静态障碍
始终保持代价 100。仿真感知没有建模遮挡、漏检和测距噪声。

## 测试

```bash
colcon test --packages-select channel_navigation_manager
colcon test-result --verbose
```

几何测试可单独运行构建出的 `test_geometry` 和 `test_perception_geometry`；Python 流水线测试
需要 ROS 2 环境和已构建的消息包。
