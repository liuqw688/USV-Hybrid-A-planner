# 无人船项目：功能包与操作说明

## 1. 当前参数与系统关系

当前仍为仿真调试版，追越终点减速场景未稳定通过，不得将自动测试通过
解读为全部避碰合格；具体失败与通过记录见根目录 `TEST_RESULTS_4MPS.md`。

本船最高4m/s、半径1.5m；目标船速度2m/s、保守避碰半径2m、显示船长1m宽1.5m；buffer=4m。基础动态硬安全距离7.5m，8m以内发布停车命令，一般软避让范围9m。圆圈和CPA详细解释见 [COLREGS_GUIDE.md](COLREGS_GUIDE.md)。平滑调参见 [航道管理README](src/channel_navigation_manager/README.md)。

实际河道链路：

```text
river_chart → /chart_costmap → local_costmap_generator → /local_costmap
           → /chart_static_obstacles ───────────────────────┐
           → /chart_lane_data → channel_navigation_manager ─┤
                              → /channel/lane_costmap      │
本船/目标船odom ─────────────────────────────────────────────┤
/goal_pose ─────────────────────────────────────────────────┤
                                                         Hybrid A*
                                      /hybrid_a_star_trajectory + /colregs/policies
                                                         ↓
                        目标船odom → DWA（8m独立停车/短时动态避碰）
                                                         ↓ /cmd_vel
                                                  boat_simulator
                                                         ↓ /odom、TF
```

## 2. 各功能包的职责

| 源码包目录 / ROS包名 | 节点及职责 | 主要输入/输出 | 启动入口 |
|---|---|---|---|
| river_chart | Python海图解析、矢量显示、河岸陆地和航道原始语义发布；静态数据通常只发布一次 | XML → /chart_markers、/chart_costmap、/chart_static_obstacles、/chart_lane_data | river_chart.launch.py |
| local_costmap_generator | C++河岸地图膨胀；不是动态航道划分器 | /chart_costmap → /local_costmap | local_costmap.launch.py |
| channel_navigation_manager | C++航道角色和中心代价生成，按实际航迹向与后方目标意图动态区分主/对向航道 | /chart_lane_data、/odom、/goal_pose → /channel/lane_costmap、/channel/navigation_markers、/channel/status | river_navigation.launch.py（一键整链路） |
| hybrid_a_star_planner | C++时空Hybrid A*、静态/动态安全检查、COLREG策略、全局长尺度平滑 | /local_costmap、航道层、本船/目标odom、/goal_pose → 全局Path、策略、风险Marker | hybrid_a_star.launch.py |
| dwa_and_ship_sim / dwa_deep | C++DWA追踪全局轨迹并生成Twist；boat_simulator受加速度限制执行命令并发布本船状态 | Path、地图、odom、策略 → /cmd_vel；仿真 → /odom、/scan、/sim_boat_marker、odom→base_link | dwa_sim.launch.py（仿真+DWA） |
| virtual_boat_simulator | C++虚拟船运动、状态和船体显示；默认匀速2m/s，可通过控制话题操纵 | /target_boat/command → /target_boat/odom、status、markers、target_boat/base_link TF | virtual_boat.launch.py |
| free_water_map | 独立开阔水域测试图和一次性目标发布，不用于实际河道地图 | → /local_costmap、/goal_pose | colregs_demo.launch.py |

不要把Marker显示当作碰撞层；未来航道内感知障碍物应发布独立膨胀层 /channel/obstacle_costmap，而不是修改船体Marker。陆地核心和独立障碍物始终不可被“航道畅通”覆盖。

## 3. 编译和日常启动

```bash
cd /home/l/work_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch channel_navigation_manager river_navigation.launch.py
```

此入口已经包含海图、膨胀、航道管理、本船仿真、Hybrid A*、DWA和RViz，不要再另开dwa_sim或再次启动同一入口。入口使用单实例锁，Ctrl-C会释放。TF默认隐藏，只用一艘本船Marker显示本船；目标船使用独立target_boat/base_link，不共享base_link。

启动后等待海图出现，再点击RViz顶部2D Goal Pose，在水域点选并拖动设置终点艏向。发布到/goal_pose，可随时重新指定。系统默认没有固定目标，也默认不启动虚拟船。没有有效路径/策略时DWA停车。

无图形环境：

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py use_rviz:=false
```

带虚拟船：

```bash
ros2 launch channel_navigation_manager river_navigation.launch.py use_virtual_boat:=true target_x:=-310.0 target_y:=-210.0 target_yaw:=-0.99483767
```

这是故意模拟靠近本船航迹的目标，初始位置不自动按航道约束目标船运动。目标船到地图外也不会自动回航，测试前要自行选择合适位置。RViz的Set Initial Pose目前用于虚拟船重设位置，不是本船重定位工具；在有本船惯性的情况下不要把目标直接放入8m停车圈并认为能保证瞬时停止。

## 4. 分开调试时怎么启动

仅在没有一键系统运行的情况下，依次启动：

```bash
ros2 launch river_chart river_chart.launch.py frame_id:=odom rviz:=false
ros2 launch local_costmap_generator local_costmap.launch.py
ros2 run channel_navigation_manager channel_navigation_node --ros-args --params-file /home/l/work_ws/src/channel_navigation_manager/config/channel_navigation.yaml
ros2 launch dwa_deep dwa_sim.launch.py
ros2 launch hybrid_a_star_planner hybrid_a_star.launch.py
```

virtual_boat.launch.py会另外发布map→odom静态TF；使用河道一键入口时不要再单独启动它。需要给现有河道增加目标时优先使用一键入口use_virtual_boat=true，或仅启动virtual_boat_node并加载YAML，不重复启动静态TF。

独立开阔水域测试：

```bash
ros2 launch free_water_map colregs_demo.launch.py scenario:=head_on
```

此入口自身包含地图、仿真、目标、Hybrid、DWA及RViz，显式关闭航道语义并将本船起点设为[-40,0]、初向0。不要同时运行河道系统。场景参数可选head_on、starboard、port_cooperative、port_takeover、overtaking、open_water。

## 5. 配置文件和参数同步

| 参数用途 | 文件 |
|---|---|
| 航道角色、中心梯度、反向意图 | channel_navigation_manager/config/channel_navigation.yaml |
| 搜索、全局平滑、目标半径、预测安全域、风险门槛 | hybrid_a_star_planner/config/hybrid_a_star_params.yaml |
| 最高速度、目标odom独立停车、DWA预测与评分 | dwa_and_ship_sim/config/dwa_params.yaml |
| 本船起点、初速、仿真限速 | dwa_and_ship_sim/config/sim_params.yaml |
| 虚拟船显示尺寸、速度、状态话题 | virtual_boat_simulator/config/virtual_boat.yaml |
| 岸边膨胀参数 | local_costmap_generator/config/local_costmap.yaml |

最高速度同步DWA max_vel_x、Hybrid cruise_speed、仿真max_linear_speed；initial_speed决定初始实际速度。本船半径与buffer同步Hybrid和DWA。目标半径由Hybrid target_ship_radii列表决定；虚拟船length/width只决定显示尺寸，不能替代半径。停车colregs_risk_distance和追越overtaking_safe_distance均在Hybrid、DWA配置为相同值。

参数修改后重启；symlink-install让YAML直接生效，普通安装需重新build复制。不要为更快测试增加min_vel_x强制最小速度，否则会妨碍停车和必要减速。

## 6. 排查常见问题

- 没有路径：检查是否设置目标、是否收到一次性静态图（Transient Local）、目标是否在可航水域，检查/channel/status和Hybrid日志。
- 有Path却不走：检查策略valid、/cmd_vel、里程计和路径新鲜度；8m停车或目标数据过期会停车。
- 看到两个本船或船位乱跳：检查是否重复运行boat_simulator/dwa_sim/一键入口；TF与Marker同时显示也会有坐标轴叠加，但真正双TF必须停止重复发布者。
- 进入红圈却没停：红色20m是风险门槛，不是8m停车；进入圈仍须DCPA/TCPA条件。
- 航道内有岸边膨胀：只忽略岸边源；独立航道障碍物和陆地核心依然生效。
- 航速不到4m/s：急转弯、避障及终点减速是允许的；没有有效路径也会停车，不能绕过硬安全筛选。

```bash
ros2 node list
ros2 topic info /tf --verbose
ros2 topic echo /channel/status
ros2 topic echo /colregs/encounter
ros2 topic echo /cmd_vel
ros2 topic echo /odom --field twist.twist.linear.x
```

## 7. 测试和使用边界

先停止已运行导航，再一键执行 `bash scripts/test_4mps_colregs.sh`。
入口自动运行47项自动测试、六类真实闭环和河道圆层/停车注入检查，返回非零
即有失败；每次生成独立结果目录，不覆盖历史失败记录。
本版详细结果与已知边界见 [TEST_RESULTS_4MPS.md](TEST_RESULTS_4MPS.md)。

```bash
colcon test --packages-select hybrid_a_star_planner channel_navigation_manager river_chart dwa_deep
colcon test-result --verbose
python3 src/free_water_map/test/v4_closed_loop.py head_on --duration 90 --output test_results/current
ROS_DOMAIN_ID=65 python3 src/free_water_map/test/current_safety_checks.py --output test_results/current
```

测试程序会启动真实ROS节点，测量实际本船和目标里程计、CPA分类、转向和最近距离，finally中停止所启动进程；不要在已有系统运行时再运行测试。结果保存在指定目录，不用旧配置历史数据当作本版结果。

此系统是仿真级行为实现和工程回归测试，不是完整COLREG合规认证。物理停车不是收到Twist零速度就瞬时实现；尤其本船4m/s、目标2m/s时，必须提前规避并验证制动裕度。实船需额外考虑感知/定位误差、船体动力学、流风、失联、人工接管与受控水域测试。
