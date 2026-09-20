# 实际河道动态航道规划使用说明

## 详细功能与参数文档

- [FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)：节点启动、功能/函数流程图、话题与包间关系、逐函数职责、当前实现边界。
- [PARAMETERS.md](PARAMETERS.md)：配置值、参数作用及增大/减小的影响。
- [工作空间总索引](../../PACKAGE_DOCUMENTATION_INDEX.md)：整条运行链、操作与验证。



`channel_navigation_manager` 当前为 C++17、`ament_cmake` 功能包。节点、几何
算法和测试均为 C++；仅一键启动入口使用 ROS 2 Python launch。

## 1. 功能与优先原则

该包把静态海图和动态航道角色分开，整体决策顺序为：

1. 安全：陆地、地图外、动态船硬碰撞和航道障碍物优先处理；
2. COLREG：有目标船时遵守避碰方向与责任；
3. 右行：通常选择与本船实际航行方向一致的 TSSLPT 分道；
4. 效率：在安全且合规的候选中选择较短、较平滑的路径。

主航道、航道外、对向航道始终同时参与 Hybrid A* 搜索。默认软代价为
主航道 0、航道外 0.65/m、对向航道 1.10/m，因此通常保持右行，但当对向
路径明显更短且安全时仍可被选择。

主航道内部还带有中心软梯度：几何中心区域代价最低，接近左右边沿时逐渐
升高到 20，再由 Hybrid A* 的 `channel.main_center_weight` 转换为路径代价。
它不是硬约束，避障、COLREG 或终点需要时仍可驶近边沿。

## 2. 地图和障碍物如何融合

- `/chart_lane_data`：`river_chart` 一次发布的 TSSLPT 多边形和 ORIENT；
- `/channel/lane_costmap`：根据 `/odom` 航迹向动态生成；主航道中心到边沿默认
  为 0–20 的软梯度，45=航道外，75=对向；
- `/local_costmap`：河岸和航道外安全代价；
- `/chart_static_obstacles`：实际陆地核心，任何时候都不可通行；
- `/channel/obstacle_costmap`：为后续感知障碍物预留的独立膨胀层。

TSSLPT 内只忽略 `/local_costmap` 中侵入航道的岸边软膨胀，不会忽略
`/channel/obstacle_costmap`。未来在航道中加入障碍物时，应把其膨胀结果发布到
`/channel/obstacle_costmap`，数值 1–84 为软代价，默认大于等于 85 为硬碰撞风险。

## 3. 航向切换与后方目标

正常情况下，主航道角色跟随 `/odom` 的实际航迹向 COG。设置新目标时会额外
检查一次任务意图：

- 北航时目标在东北或西北等前半平面，不翻转航道角色；
- 目标明确位于后方超过 `reverse_enter_angle` 时，在 Hybrid A* 搜索前就按
  目标方向预选反向主航道；
- 因此南向目标会优先朝西侧的新主航道掉头，不再先向东再横跨；
- 实际航迹向接近新方向后结束预选，继续按 COG 动态分类。

这一判断使用线速度方向和目标任务方向，不使用角速度或累计转向角。

## 4. 轨迹稳定与大惯性控制

Hybrid A* 默认采用事件搜索、持续安全监测：

- 新目标、剩余路径预测碰撞/COLREG冲突、静态阻断或明显偏航时才重新搜索；
- 仅检测到附近船只但路径不会相撞时继续执行缓存路径，不触发搜索；
- 安全缓存路径按timer频率原样重发，DWA不会因3秒超时停止；
- 真正重规划时上一条有效航线作为软参考距离层，避免无意义的大幅跳变；
- 相邻运动基元增加舵角变化代价，减少左右交替的小圆弧；
- 对持续大舵角增加平方软代价，减少尺度较大的 S 弯，必要避障转向仍可覆盖；
- 搜索结果统一按 1 m 间距重采样，再以原路径约束进行整体平滑，消除因
  2 m 搜索基元和 0.15 m Dubins 尾段密度不同造成的曲折；
- 每轮平滑都重新检查河岸、航道障碍物、动态船预测安全域和 COLREG 航向，
  不安全时退回原始安全路线或上一轮安全平滑结果，因此不会为了外观平滑牺牲避碰能力；
- 障碍物或目标船使剩余路径不再安全时在下一个监测周期重新搜索，连续性代价可以被安全规则覆盖。

平滑功能仅修改Hybrid A*；默认tracking_only的DWA只执行路径跟踪、静态可行性和终点收敛，动态避碰停车由Hybrid通过空Path统一触发。
平滑相关参数位于 `hybrid_a_star_params.yaml` 的
`stability.*` 和 `smoothing.*` 段。

一键启动带有进程级单实例锁。若旧的 `river_navigation.launch.py` 尚未退出，
再次启动会直接提示先结束旧实例，不会再产生两套 `/odom`、`base_link`、规划器
和控制器。RViz 默认关闭 TF 坐标轴，只用 `/sim_boat_marker` 显示一艘本船；需要
排查坐标变换时可在 Displays 中手动开启 TF。

## 5. 编译

```bash
cd /home/l/work_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 6. 一键启动

当前默认不启动虚拟船，也不发布固定目标：

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py
```

不打开 RViz：

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py use_rviz:=false
```

需要恢复虚拟船做 COLREG 联调：

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py use_virtual_boat:=true
```

## 7. 在 RViz 中操作

1. 等待海图、绿色主航道边框和船体出现；
2. 点击顶部 `2D Goal Pose`；
3. 在可航区域点击并拖动，拖动方向同时设定终点艏向；
4. 新目标发布到 `/goal_pose`，Channel A* 更新全局路线并在
   `/goal_pose_from_astar` 发布当前活动航点，Hybrid A* 随后重规划；
5. 绿色边框是当前航行方向的主航道，红色边框是对向航道；
6. `Hybrid A Star Path` 是全局轨迹，`DWA Candidates` 与
   `DWA Optimal Trajectory` 是局部候选和最终轨迹。

起点为 `[-270,-270]`，位于 `ORIENT=327°` 的 TSSLPT 分道内。初始 yaw 已换算为
ROS ENU 下约 123°，保证初始航迹向与该分道一致。

## 8. 常用参数

- 航道区域偏好：`hybrid_a_star_params.yaml` 中
  `channel.outside_lane_weight`、`channel.opposite_lane_weight`；
- 航道外碰撞阈值：`channel.shoreline_cost_threshold`；
- 航道障碍物碰撞阈值：`channel.obstacle_cost_threshold`；
- 航道障碍物膨胀权重：`channel.obstacle_inflation_weight`；
- 航行方向重新分类灵敏度：`channel_navigation.yaml` 中 `reclassify_angle`；
- 航道中心吸引范围：`center_clearance` 和 `channel.main_center_weight`；
- 后方目标提前翻转阈值：`reverse_enter_angle`、`reverse_complete_angle`；
- 船速：现按要求使用 `4 m/s`（约7.78节）。本项目必须同步设置
  `dwa_params.yaml/max_vel_x`、`hybrid_a_star_params.yaml/cruise_speed`，以及
  `sim_params.yaml/initial_speed` 和 `max_linear_speed`。当前四项均为 4.0。
  DWA 的 `speed_weight=30.0` 使安全直航候选明确优先达到该速度；
  `max_lookahead_distance=10.0` 为高速航行提供足够前视。急转弯、临近终点或
  避碰时仍会自动降速，不能为了保持最高速度绕过安全约束。
  `goal_deceleration=0.5` 为终点制动速度上限提供依据，防止高速绕终点。

小型无人船测试避碰域现在为：两船等效半径均0.5m、冗余1m，中心间普通硬净空
2m；追越硬净空3m；有效会遇的青色软规划域7m；默认DWA不再根据当前目标船距离独立停车。
目标船显示长1m、宽1.5m，显示外形与0.5m等效规划半径相互独立。
风险圆层说明见根目录 `COLREGS_GUIDE.md`，各包操作见 `NAVIGATION_PACKAGES_GUIDE.md`。

### 8.1 Hybrid A* 平滑参数

以下参数都在 `hybrid_a_star_params.yaml` 中。安全检查始终生效，平滑参数不能
让轨迹穿越河岸、障碍物或动态船硬安全域。

| 参数 | 当前值 | 增大后的效果 | 减小后的效果 |
|---|---:|---|---|
| `stability.reference_path_weight` | 0.5 | 相邻周期更稳定，但可能更依赖上一条路线 | 更容易响应新环境，但周期间跳动可能增加 |
| `stability.reference_path_max_distance` | 12 m | 旧路线在更远范围仍有影响 | 旧路线影响更局部 |
| `stability.steering_change_weight` | 1.0 | 更少左右反打舵和短小圆弧 | 转向选择更灵活，但容易出现细碎摆动 |
| `stability.steering_magnitude_weight` | 1.0 | 更少持续大舵角和宽大 S 弯 | 更容易采用大曲率绕行 |
| `smoothing.resample_spacing` | 1.0 m | 点更稀疏、计算更快，但显示可能不够细 | 点更密、曲线显示更细，计算量增加 |
| `smoothing.iterations` | 120 | 更接近平滑收敛结果，超过一定值收益变小 | 计算更快，但可能残留弯折 |
| `smoothing.smooth_weight` | 0.26 | 加强相邻点局部平滑 | 更多保留局部弯折 |
| `smoothing.data_weight` | 0.015 | 更贴近原始 Hybrid A* 路线，实际会降低平滑程度 | 允许更大修正，轨迹更加平滑 |
| `smoothing.long_range_points` | 6 | 观察范围更长，更能消除宽大 S 弯 | 主要处理较短尺度弯折 |
| `smoothing.long_range_weight` | 0.12 | 更强地拉直整体蛇形 | 更多保留航线原有大尺度形状 |
| `smoothing.max_deviation` | 6 m | 平滑器有更大调整空间 | 更严格贴近原始安全路线 |

推荐每次只修改一类参数：

1. 整体仍有宽大 S 弯：先把 `long_range_weight` 从 0.12 调到 0.14 或 0.16；
   仍不够时把 `long_range_points` 从 6 调到 8。
2. 只有局部小波浪：把 `smooth_weight` 从 0.26 调到 0.28，或把
   `iterations` 从 120 调到 150。
3. 路线仍过度贴着原始折线：把 `data_weight` 从 0.015 降到 0.010。
4. 平滑后切弯过多：降低 `long_range_weight`，或把 `max_deviation` 从 6 m
   降到 4 m。
5. 相邻周期轨迹跳动：略微提高 `reference_path_weight`，建议每次增加 0.2；
   不建议超过 1.5，以免妨碍突发障碍物响应。

修改 YAML 后需要重新启动节点。若使用普通 `colcon build` 而非
`--symlink-install`，还需要重新编译才能把 YAML 复制到 `install`。

## 9. 状态检查

```bash
ros2 topic echo /channel/status
ros2 topic echo /channel/lane_costmap --once
ros2 topic echo /hybrid_a_star_trajectory --once
ros2 topic echo /optimal_path --once
```

`/channel/status` 会显示实际航迹向、用于规划的策略航向、`reverse_intent`、
所在 lane_id 和 MAIN/OPPOSITE/OUTSIDE。

## 10. 本版验证结果

自动测试命令：

```bash
cd /home/l/work_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
colcon test --packages-select channel_navigation_manager hybrid_a_star_planner dwa_deep river_chart
colcon test-result --verbose
```

当前共有 47 项自动测试，0 error、0 failure。除几何、海图解析和既有 COLREG
用例外，新增测试覆盖以下行为：

- 主航道中心栅格代价低于边沿，边沿仍保持可通行；
- 北航时东北、西北目标不误触发反向航道，南向后方目标触发提前翻转；
- Hybrid A* 全局平滑显著缩短人工锯齿路线，同时保持起终点不变；
- 50 m 波长的整体蛇形路线波峰至少降低 35%；
- 平滑后的每一段都不能切入动态船的时间同步硬安全域；
- Hybrid A* 仍能通过对遇、右舷交叉、左舷交叉（含 Rule 17 接管）、追越和开阔水域用例。

实际整链路远目标测试从 `[-270,-270]` 规划到约 `[-324,-101]`，首条轨迹在
0.36 秒内生成，共 918 个点。首条轨迹 100% 位于当时的主航道，进入边沿高代价
带的比例为 0。该数据来自旧连续搜索版本，用于验证航道和平滑能力；当前事件模式
首次成功后会重复发布同一条安全路径，因此RViz同样不会周期性闪烁。

轨迹稳定采用两层保护：静态场景不创建重复时间状态，并使用二维水域引导搜索
穿过弯曲河道；新目标只清空旧路径一次。事件模式不重复搜索，但会刷新同一条
安全路径的时间戳，因此DWA的 `path_stale_timeout` 不会把正常缓存路径判为失效。

后方目标测试中，船体实际航迹向仍为 123° 时，策略航向会立即切换为 -57°并
报告 `reverse_intent=true`，证明航道角色在船体完成掉头前切换。

`planner_frequency` 现在是风险监测和缓存路径刷新频率。轨迹平滑测试仍比较原始与
后处理轨迹的长度、累计转向波动和安全性；事件测试另外验证无冲突目标不触发搜索、
对遇触发搜索、右舷安全路径保持执行以及新静态障碍触发搜索。

加入长尺度约束后的同一路段连续测试仍以 1.00 s 平均间隔规划。单段最大航向变化
约 0.016 rad（0.92°），每条路径累计转向波动约 0.28–0.51 rad；最初版本同一路线
约为 2.96 rad。重复执行一键启动的第二个进程会在创建节点前退出。

以下为此前8节版本的历史速度联调数据，不是当前4m/s配置：
四个运行参数均读取为 `4.115552 m/s`。在获得有效全局轨迹后，
实测 `/odom` 最高线速度为 `4.115319 m/s`（数值离散误差约 0.006%），最后 2 秒
平均约 `4.0735 m/s`；路线弯曲时允许小幅自动降速。
