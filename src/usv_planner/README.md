# usv_planner

ROS 2 Jazzy 无人船局部规划包。算法由 Fast-Planner 的动力学搜索和 B 样条优化改造，
使用 `usv_model_sim` 中与仿真、NMPC 共用的船体模型，并通过 `usv_interfaces/UsvBspline`
发布三维位姿曲线。

## 坐标与状态

内部六维状态顺序为 `[x_E, y_N, phi, u, v, r]`。`x_E` 向东、`y_N` 向北，位置采用地图
ENU 坐标；`phi` 是北为零、西转为正的导航航向，单位 rad，东为 `-pi/2`、西为 `pi/2`；
船体速度采用 RFU，`u` 向前、`v` 向右，`r` 为转艏角速度。运动学和动力学直接调用
`usv_model_sim` 的模型，不在规划器内使用另一组系数或重解释状态数值。

输入 `/odom` 的位置必须已经在规划地图坐标系内，`header.frame_id` 必须与 `planning_frame`
相同。规划器不做 TF 查询或坐标变换；twist 使用 `child_frame_id=usv_body_rfu` 的 RFU 约定，
映射到模型的 `u`、`v`。地图、目标和里程计的平面位置均是东、北坐标。

## 输入与输出

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| `/channel_costmap` | `nav_msgs/OccupancyGrid` | 同向/航道外/对向/障碍代价图；由 `grid_map_topic` 配置 |
| `/local_map` | `nav_msgs/OccupancyGrid` | 感知局部栅格；未知格默认按障碍处理 |
| `/odom` | `nav_msgs/Odometry` | 实船状态；位置必须在规划地图坐标系，速度为 RFU |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 单个位姿目标；RViz Pose Goal 的局部 +x 方向作为目标船首方向 |
| `/path` | `nav_msgs/Path` | 航点序列；目标航向由相邻位置切线计算 |
| `/planning/bspline` | `usv_interfaces/UsvBspline` | 输出的时间参数化 `[x,y,phi]` B 样条 |
| `/planning/bspline_path` | `nav_msgs/Path` | 与已发布样条一致的平面采样曲线，供 RViz 显示 |

`goal_input_mode=1` 时只接收单个位姿，`goal_input_mode=2` 时只接收完整任务 Path。
Path 按航点顺序推进，用实船位置投影保持单调进度，每次规划向前取最多 `search_horizon`
米的折线弧长作为局部目标；横向偏离占用前视距离，目标不越出以实船为中心的感知圆。
局部目标按 `path_speed_mps` 通过，只有覆盖任务末航点时才要求零速。目标 Pose 也按静止终态规划。输出控制点
是 `[x_E, y_N, phi]`，不是六维状态或 B 样条曲线上的采样值；航向单位 rad，位置单位 m。
消息使用 `order`、`traj_id`、`start_time`、`knots` 和 `control_points` 字段，与 NMPC
订阅的 `UsvBspline.msg` 一致。

规划器按需生成新轨迹，在执行期间以 10 Hz 重发当前有效轨迹。默认搜索前视半径为 `60 m`，
局部障碍栅格裁切半宽随 `search_horizon` 使用相同数值，覆盖搜索圆形范围。模拟感知的
`channel_navigation_manager/config/channel_navigation.yaml` 中 `perception_radius_m` 也默认为
`60 m`；调整感知范围时须同步设置 `search_horizon`，启动参数不会自动修改传感器。
该值是前视上限，弯道的目标直线距离、剩余任务长度及实际样条长度不一定等于 60 m。重发保留原始
`traj_id` 和 `start_time`，控制器沿原时间轴继续跟踪；收到新的有效轨迹后切换。
发布前检查轨迹剩余部分的碰撞、有效期和船位偏差；里程计超过 0.5 s 未更新、
地图未就绪或轨迹无法通过检查时停止重发，并保留 NMPC 的参考超时保护。
地图就绪不表示传感器数据实时有效，系统仍需提供可靠的地图更新与失效监测。

## 分终端启动

先在各终端加载 Jazzy 环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
```

1. 启动地图、原点、里程计转换和 RViz：

```bash
ros2 launch usv_map usv_map.launch.py
```

2. 启动无人船动力学仿真：

```bash
ros2 launch usv_model_sim usv_model.launch.py
```

3. 启动规划器。默认使用单个位姿模式；规划器不会启动地图、船模、控制器或目标发布器：

```bash
ros2 launch usv_planner usv_planner.launch.py
```

4. 单个位姿可在 RViz 使用 Pose Goal 工具；使用 Path 航点模式时，在第 3 步启动时设置
`goal_input_mode:=2`：

```bash
ros2 launch usv_planner usv_planner.launch.py goal_input_mode:=2
```

5. 启动 NMPC 后，在另一个终端发布默认 `suiku.xml` 顺时针航道一周：

```bash
ros2 run usv_planner publish_clockwise_waypoints.py
```

脚本从 `/chart_lane_data` 读取 CW 航道各段的面积质心，验证闭环顺序，从距离 `/odom`
最近航点沿顺时针前移约 30 m 选择入口，再沿航道一周回到入口；`--entry-lookahead` 可调整
入口前移距离，话题默认 `/path`。脚本等待地图、里程计和
Path 订阅者，超时或航道几何不满足生成条件时不发送；可通过 `--timeout 60` 修改等待秒数，
通过 `--topic /其他话题` 修改发布话题，并同时用 `goal_path_topic` 配置规划器。航点消息
只发送一次；再次运行脚本会作为新任务重新开始。切勿用高频循环重复发布同一任务。
本脚本适用于航段编号与行驶顺序一致的闭环海图；更换地图时需先确认航道编号、障碍及可通航性，
脚本的几何检查不能替代碰撞检查或保证规划成功。
发布器等待名为 `usv_planner_node` 的订阅者；规划节点改名时用 `--planner-node` 指定新名称。

航点用于定义按顺序推进的折线，不再逐点到达后才切换。默认前视 `60 m`，常规每前进
`8 m` 重新规划时更新局部目标，碰撞可提前触发重规划；重发当前样条不重置进度。
实船投影只在已有进度之后、相邻里程计位移两倍以内搜索，避免闭环及交叉处跳到末段。
该机制提供局部目标引导，不要求生成的避障曲线严格穿过每个航点。轨迹能否连续通过还取决于
船模、障碍、安全约束和规划是否成功。
选点查询融合后的占用图和距离场，要求目标在膨胀障碍外且额外净距大于 `0.5 m`。
航点或中间折线落在障碍中不作为必须到达的点；前视目标安全时，直接由动力学搜索绕过中间障碍。
若前视目标受阻，则沿折线回查该受阻段的入口，从入口前约 `10 m` 开始，按不超过
`0.5 m` 的弧长间隔向前寻找安全目标。障碍后方尚未进入前视范围时先使用障碍前目标；
实船推进后继续滚动选点，前视目标越过障碍且安全后交给搜索绕障通过。修正目标不推进实船
投影进度，也不改变最终任务位置；只有原始任务末点才要求停车。范围内无安全候选时等待更新，
不会将占用点送入搜索。通过目标位于既有 `5 m` 障碍限速区时，末速同步限制为 `1 m/s`。
航道管理器也订阅 `/path`，按首个有效航段确定 CW/CCW 并在任务中保持，新的 `/goal_pose`
会切回单点方向判断；如果修改 Path 话题，也应同步修改航道管理器的 `goal_path_topic`。

跟踪仿真时，单独启动 `usv_nmpc_controller`，由本规划器发布 `/planning/bspline`；
不要同时运行 Python 样条参考发布器。完整分步命令及实际运行绘图见
[仿真测试快速启动](../../README_SIMULATION.md)。

地图自带的 RViz 配置已启用绿色 `Mission waypoints` 图层，订阅 `/path`；它显示输入的
完整任务航点。青色 `Local planner B-spline` 图层订阅 `/planning/bspline_path`，显示规划器
滚动生成的局部曲线；Fixed Frame 为 `map`。B 样条话题使用 Reliable、Transient Local，
任务 Path 使用 Reliable、Volatile。
任务只发送一次，请先启动地图 RViz，再运行航点发布脚本；若 RViz 在任务发送后启动，
需重新发送任务才能显示 `/path`，重新发送也会重启规划任务。
规划器按不超过 0.1 s 的间隔对实际发布的 B 样条求值，包含首尾点，位置只使用 `x_E、y_N`，
高度为零，姿态直接保存原生 `phi`；显示的是曲线而不是控制点折线。该可视化话题保留最近一条
消息，RViz 晚启动也能显示。NMPC 仍订阅原始 `/planning/bspline`。

## Launch 参数

不传参数时使用 `config/usv_planner.yaml`。常用覆盖项如下；完整默认值由该 YAML 文件统一维护。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `params_file` | 包内 `config/usv_planner.yaml` | 规划器 ROS 参数 YAML，可替换整套规划配置 |
| `grid_map_topic` | `/channel_costmap` | 航道方向代价地图；无航道代价节点时可覆盖为 `/chart_costmap` |
| `local_grid_topic` | `/local_map` | 局部障碍 OccupancyGrid |
| `planning_frame` | `map` | 地图、目标和里程计必须一致的 frame；不触发坐标转换 |
| `odom_topic` | `/odom` | 无人船里程计输入 |
| `goal_input_mode` | `1` | `1` 为 PoseStamped，`2` 为 Path |
| `goal_pose_topic` | `/goal_pose` | 单个位姿目标输入 |
| `goal_path_topic` | `/path` | 多航点 Path 输入；旧发布器可配置为 `/goal_path` |
| `bspline_topic` | `/planning/bspline` | `UsvBspline` 输出 |
| `path_speed_mps` | `3.0` | Path 滚动局部目标的通过速度，m/s；任务末点为零速 |
| `shot_trigger_distance` | `30.0` | 尝试动力学终端连接的最大距离，m |
| `search_time_limit` | `0.4` | 前端搜索软预算，s；样条优化另有预算 |
| `search_horizon` | `60.0` | 搜索前视半径及局部障碍栅格裁切半宽，m |
| `search_resolution` | `1.0` | 搜索位置离散分辨率，m |
| `search_heuristic_weight` | `5.0` | 加权 A* 启发式系数 |
| `replan_distance` | `8.0` | 常规滚动重规划距离阈值，m |
| `safety_replan_distance` | `0.5` | 已膨胀障碍边界外额外安全距离，m |
| `max_vel` | `3.0` | B 样条各分量速度界，m/s |
| `max_acc` | `2.0` | B 样条各分量加速度界，m/s^2 |

默认直线巡航目标为 `3 m/s`，转弯和任务终点制动时允许减速。搜索前向指令按共享船模的
稳态关系配置为约 `2.9643`；终端连接保留 `5` 的指令上限，状态速度上限仍为 `3 m/s`。

每条发布样条的终点船首航向与当前目标航向的最短角差小于 `search/shot_yaw_tolerance`，
默认 10°，滚动前缀也适用；Path 模式的当前目标航向由对应航段方向确定。绕障或掉头时若无法
在预算内同时满足该条件和动力学、避碰约束，规划器不会发布不合格前缀。

默认距离输入占用栅格边界不超过 `manager/obstacle_slowdown_distance=5 m` 时，
规划参考的对地合速度限制为 `manager/obstacle_max_speed=1 m/s`。距离从船舶定位点计算，
包含栅格分辨率的保守余量；`/channel_costmap` 中的感知障碍和岸线可能已预先膨胀，因而该
距离不一定是到真实障碍表面的距离。样条每 0.025 s 采样验收，不是连续时间的严格证明。
旧参考因地图变化而不再安全时，规划器通过 `/planning/replan_failed` 通知控制器撤销并停车；
障碍出现过晚或船舶已处于限速区且超速时，实际船速受动力学制动能力限制，不能瞬时达标。
样条实际时间调整使用 `manager/max_global_vel=3.0` 和 `manager/max_global_acc=2.0`。
`search/w_time=80.0` 平衡行驶时间和推力代价；`manager/max_yaw_acc=0.5 rad/s²`
覆盖所设状态及指令边界下的船模响应，避免时间调整将正常转向过度拉慢。
常规重规划距离为 `8 m`；距目标不足 `fsm/thresh_no_replan=5 m` 且航向满足阈值时，
暂缓常规重规划。碰撞检查仍可提前触发重规划。

查看实际可覆盖参数：

```bash
ros2 launch usv_planner usv_planner.launch.py --show-args
```

默认代价图由 `channel_navigation_manager` 发布：同向航道从中心 `0` 递增到侧边 `8`，
航道外为 `20`，与对向航道基础代价相同；对向航道从中心 `20` 递增到侧边 `28`，陆地和障碍物为 `100`。
航道中心是所属同向航道的横断面中心，不是双向水域的几何中心。
搜索将可通行代价按 `1 + cost / 20` 换算成路程倍率，终端连接和样条优化也查询同一张图。
样条优化对实际曲线上的通行代价积分，并使用双线性插值梯度，避免仅优化障碍距离或控制点。
居中属于软偏好，允许绕障偏离中心；`100` 仍作为障碍硬拒绝。
局部地图的空闲观测只清除局部障碍，不覆盖底图中的航道代价。

转弯使用 `manager/min_turn_radius`（默认 `8.0 m`，`0` 关闭），同时约束对地曲率和
艏摇率与平面速度的比值。对地曲率使用位置的一、二阶导数计算，包含侧滑变化的影响，
不能用 `r / speed` 替代。搜索保留拟合余量；启用半径约束时 shot 使用六段分段恒定
指令连接，关闭约束时使用四段。shot 根据
半径和航向差提前触发；优化后对实际 B 样条每不超过 `0.025 s` 密集采样核验，超限时
检查搜索拟合备用曲线，两者均不合格则不发布。数值核验允许 `0.5%` 的倍率误差。
低于 `0.3 m/s` 时采用正则化速度，处理停车处的零除，因此该速度以下不保证严格几何半径。
`8 m` 是可调工程初值，需要根据实船与可用水域确定；时间拉长不能修复过小的几何半径。

相关配置：

| 文件及参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `usv_planner.yaml`：`manager/min_turn_radius` | `8.0` | 搜索、终端连接和最终样条共用的最小转弯半径，m |
| `usv_planner.yaml`：`optimization/channel_weight` | `2.0` | 样条通行代价积分权重；0 关闭后端通行偏好 |
| `channel_navigation.yaml`：`lane_center_cost` | `8` | 中心至侧边的附加代价；0 恢复均匀航道代价 |

只有搜索实际到达完整任务终态时才强制末端零速；滚动前缀保留搜索末态速度。
最终目标的终端连接按当前船速预留被动制动时间；搜索超时回退的前缀也保留直线
制动距离余量。该余量不保证所有障碍或转向场景都存在可行停车轨迹。
完整终端轨迹仍安全且时间参考与实船位置相距不超过 `2 m` 时保持原曲线及起始时间；
偏差超过 `2 m` 时从实测状态恢复规划。轨迹结束后继续发布有效的零速参考，最终目标
的状态机还要求实测平面合速度不大于 `0.2 m/s`，并与终端轨迹末点相距不超过 `2 m`。
到达后仅在里程计、地图和终点安全检查有效时保持参考；新任务接管原参考。

YAML 中其余项按功能分为状态机到达/重规划门限、栅格代价图、动力学状态与控制输入边界、
shot 终端容差、样条优化代价及样条可行性边界。障碍物膨胀默认 `3.0 m`，在膨胀边界外
另留 `0.5 m` 搜索余量。修改 YAML 后重启规划器生效。

## 构建与测试

```bash
cd /home/sus/USV_systems
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to usv_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select usv_planner --ctest-args --output-on-failure
colcon test-result --verbose
python3 src/usv_planner/test/test_clockwise_waypoints.py
```

规划器使用 Eigen、PCL、OpenCV 和 NLopt；NLopt 开发包须提供 C++ 头文件及链接库。
核心测试覆盖船体模型、样条参数化、碰撞安全、动力学 A*、转弯半径、航道梯度和曲线居中。
规划耗时与构建类型有关；实时评估使用 Release。测试通过不代表任意障碍和终态均可规划，
发布前应检查本机 `colcon test-result --verbose`，不能只以编译成功判断规划成功率。

持续参考发布的独立联调测试：

```bash
python3 /home/sus/USV_systems/src/usv_planner/test/test_reference_stream.py
```

脚本在独立 DDS 域 `91` 启动规划器和 NMPC，验证重复消息不重置参考时间、新轨迹连续
接管，以及障碍使旧轨迹失效后停止重发、通知 NMPC 停车并等待新参考。测试需要 `rclpy` 和
`python3-scipy`；反馈直接取样条真值，检验通信与切换行为，不评价船体闭环跟踪精度。

Path 前沿及中间航点落入障碍的独立回归测试：

```bash
python3 /home/sus/USV_systems/src/usv_planner/test/test_blocked_path.py
```

脚本使用独立 DDS 域 `193`，运行真实规划器，并以已发布样条反馈里程计。测试检查受阻前沿
的安全替代目标、后续绕障和单调进度，同时每 `0.02 s` 采样核验发布曲线与障碍膨胀区的
关系；不运行 NMPC 或船模。约需一分钟，日志和曲线保存在 `/tmp/usv_blocked_path_test`，
可用 `--domain-id` 和 `--output` 更改测试域及输出目录。

## 运行中只读监测

记录规划节点日志中的成功/失败调用耗时，并订阅当前样条及 NMPC 诊断；脚本不启动、
停止节点或发布目标及控制指令。先加载本工作空间环境，再指定当前规划节点的日志文件：

```bash
python3 /home/sus/USV_systems/src/usv_planner/test/record_planner_timing.py \
  /tmp/usv_planner_monitor --duration 60 \
  --planner-log /home/sus/.ros/log/usv_planner_node_<进程号>_<时间戳>.log
```

输出包含消息间隔、计算耗时、样条原始控制点、NMPC 参考/实际速度和图表。
`reference_switches.csv` 在同一接收时刻计算新旧样条的 `u、v、r` 差值，避免把正常
时间推进当成换轨突变；`bspline_messages.csv` 另外记录同一条曲线在发布及接收时刻
的速度差，反映传输延迟期间的参考变化。两项均保持本船 RFU 速度及导航航向定义。
脚本的发布/接收延迟与同一时刻对比要求节点使用系统时间（`use_sim_time=false`），
跨主机还需时钟同步；NMPC 诊断另取实际节点使用的参考速度作对照。

## 源码结构

`usv_planner_search_core` 是不依赖 ROS 2 API 的 C++ 库，包含六状态动力学搜索、船体模型
适配、搜索参数结构、地图查询接口、二维距离启发和状态离散。调用方实现
`SearchEnvironment` 并传入 `KinodynamicAstarConfig` 即可复用 A*。`usv_planner_search_ros`
负责 ROS 参数读取、动力学 A* 的地图环境适配，以及仍使用 ROS 参数接口的几何/拓扑搜索。
`modules/plan_env` 负责 ROS 地图输入和二维距离查询；`modules/bspline` 的曲线数学与
`modules/poly_traj` 不依赖 ROS 2 API。`usv_planner_optimizer` 只依赖 Eigen、NLopt 和
`SearchEnvironment`，由 `usv_planner_optimizer_ros` 读取 ROS 参数后注入配置；
`modules/plan_manage` 组合规划器并运行目标输入状态机；`modules/traj_utils` 发布 RViz 标记；
`src/main.cpp` 是 ROS 2 节点入口。算法代码来自 Fast-Planner 并为本船模和海图接口作了改造，
上游版权与许可证见本包 `LICENSE`。
