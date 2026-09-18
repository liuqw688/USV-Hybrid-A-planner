# Channel A* Global Planner：一次任务、固定航点表

## 详细功能与参数文档

- [FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)：节点启动、功能/函数流程图、话题与包间关系、逐函数职责、当前实现边界。
- [PARAMETERS.md](PARAMETERS.md)：配置值、参数作用及增大/减小的影响。
- [工作空间总索引](../../PACKAGE_DOCUMENTATION_INDEX.md)：整条运行链、操作与验证。



## 功能与接线

主节点是C++ `channel_astar_node`。接收最终目标与完整航道语义栅格，
使用8邻域A*生成路线，按沿路线累计弧长约100m采样。
YAML的`via_points`途经点按顺序必经，精确保留，不被100m采样删除。

```text
river_chart静态信息 → channel_navigation_manager → /channel/lane_costmap
/goal_pose + /odom → Channel A* → /astar_channel_waypoints（完整固定Path）
                               → /goal_pose_from_astar（依次一个活动目标）
                               → Hybrid A* → DWA → 仿真
```

不能同时向Hybrid发布一批PoseStamped，否则它只保存最后一个目标。
本包一次发布完整Path，再根据船位顺序切换活动目标。
你更新后的航道方向判定保持不变，本包直接使用其0..20主航道编码，
不会另行用每段航道和本船艏向夹角重新划分左右。

## 一次规划与例外

- 每个新最终目标规划一次；船运动、重复图、0..20内部代价变化不重建当前任务。
- 主航道0..20与对向航道75互换时自动纠偏一次，成功或失败任务均适用；保留已完成途经点。
- 角色变化立即撤销旧路线，等待map_settle_time后重建；连续变化合并为一次最新快照搜索。
- 每条目标消息都是新任务；再次点击相同位置也重新搜索一次。
- Path和活动目标使用Reliable＋Transient Local；晚启动订阅者仍能收到一次发布的内容。
- 新任务先发布空Path撤销旧任务，无解时不继续执行旧目标。
- 首次输入不齐或TF缺失会等待，由地图/船位回调补齐这一次规划；没有规划定时器。
- 输入就绪后的A*搜索成功或失败均只执行一次；失败后地图更新不会自动重试。
- 重新规划入口为新目标、主/对向角色互换，以及明确调用人工重规划服务；没有固定频率搜索。
- 显式服务从当前位置经剩余途经点重新规划，已完成途经点不会要求返回。
- 动态避障交给Hybrid/DWA；实际航道封闭或静态图重大变化时，应人工更新任务或显式重规划。

## 配置

文件：`config/channel_astar.yaml`。顶层必须为实际节点名`channel_astar_node`。
原来的包名与节点不匹配，可能导致参数不生效，已更正。

| 参数 | 默认 | 说明 |
|---|---:|---|
| waypoint_spacing | 100m | 必经目标之间按累计弧长采样；不足100m的终点保留 |
| goal_switch_distance | 8m | 船距普通采样点小于8m时切换下一个点，必须小于采样间隔 |
| mandatory_goal_distance | 1.5m | 途经点到达半径，不能用普通8m阈值跳过 |
| via_points | 默认空列表 | 数字数组[x1,y1,x2,y2]，按顺序必经，须与图同坐标系；无途经点时注释该项，不写空数组[]（Humble解析限制） |
| map_settle_time | 0.5s | 新目标点击后等待航道角色更新的时间，由输入回调触发，无规划定时器 |
| minimum_replan_period | 0.5s | 两次任务搜索最小间隔，不是固定重规划周期 |
| replan_on_lane_role_change | true | 主/对向航道角色互换时纠偏一次；关闭恢复仅目标/人工服务触发 |
| right_channel_only | true | 主体限制在语义0..20主航道；左航道最终目标允许末段连接 |
| opposite_lane_value | 75 | 左/对向航道编码，与航道管理器一致 |
| terminal_crossing_distance | 100m | 左航道目标附近可跨入对向航道的半径，宽河道需适当增大 |
| opposite_lane_penalty | 4.0 | 左航道额外行程代价，越大越偏好延后跨入；仍累计原语义代价 |
| outside_lane_value | 45 | 航道外语义编码，与航道管理器一致 |
| outside_lane_penalty | 1.0 | 航道外额外行程代价，默认低于左航道 |
| safety_costmap_topic | /local_costmap | 独立物理安全地图，须与航道图同坐标系/原点/尺寸/分辨率 |
| safety_cost_threshold | 88 | 航道外>=88或未知栅格拒绝，须与Hybrid岸边阈值一致 |
| center_cost_weight | 0.08 | 增大更偏好主航道中心 |

每个途经点把路线分段，各段从0重新累计100m采样。
当前源码没有找到海图途经点话题或现成读取逻辑，故提供明确YAML入口。
若途经点来自其他话题，需要确认消息名/类型再接入，Marker不会自动当途经点。
修改参数重启加载；新任务重新经过全部配置点，显式服务只走当前任务剩余点。

## 操作

```bash
cd /home/l/work_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch channel_navigation_manager river_navigation.launch.py
```

RViz用2D Goal Pose设置`/goal_pose`。配置途经点须在连通主航道。
船若已经位于左航道或安全航道外，新任务可在起点100m附近连接返回主航道；危险航道外起点拒绝。
最终目标可在右主航道、左/对向航道或安全航道外水域：主体在右侧，
末段进入非主航道限制在终点100m附近；航道外代价高于右航道、低于左航道。
普通采样间隔100m；正常窄分道下通常前面的稀疏点在右侧、最后目标在左侧，
但不强制九比一数量。最后一段仍由Hybrid生成安全曲线，不直接跟踪黄色直线。
末段没有连通路径时拒绝任务。航道外45不是危险值，而是语义标签；
必须同时检查/local_costmap，未知或>=88的航道外格不放开，危险终点也拒绝。
当前叠加检查按栅格中心、只对航道外执行（保留航道内岸边膨胀处理原则）；
最终船体半径、真实陆地、航道内障碍物及动态船碰撞仍由Hybrid/DWA验证。
RViz保持Fixed Frame=map即可；节点通过TF将最终目标位置和姿态转换到航道图坐标系。
一键启动已提供map→odom静态TF，并启用黄色`Channel A Star Global Route`显示。
黄色路线为稀疏全局点连线，绿色路线才是Hybrid安全控制曲线。
Hybrid订阅`/goal_pose_from_astar`，仍按原频率完整重规划。
最终目标位置/朝向精确保留；中间点朝向来自下一段路线。
最终活动目标不提前跳走，由Hybrid/DWA的终点容差停车。

仅启动本包（须已有航道图、同网格/local_costmap和连续odom，不能与一键入口重复启动）：

```bash
ros2 launch channel_astar_global_planner channel_astar.launch.py
ros2 topic echo /astar_channel_waypoints
ros2 topic echo /goal_pose_from_astar
ros2 service call /channel_astar/replan std_srvs/srv/Trigger '{}'
```

服务success表示请求已排队，实际成功看非空Path及日志。

## 边界与其他问题

1. 起点/终点不再无条件豁免不可通行检查。目标通过最新可用TF转换到航道图坐标系；
   TF未到达时等待并重试，绝不将不同坐标系直接视为相同。
   odom仍须与航道图同坐标系；途经点配置也使用航道图坐标系，不跟随RViz坐标系。
2. 搜索拒绝斜向穿障碍角，但稀疏点之间直线可能穿出弯曲航道，
   不能直接作为控制曲线，必须由Hybrid重新搜索安全航道曲线。
3. 路线语义输入是`/channel/lane_costmap`，航道外叠加`/local_costmap`危险值检查。
   主航道掩码不能代替独立陆地/障碍层；Hybrid仍执行完整安全检查。
   本次不修改你的river_chart或航道分类算法。
4. 一键入口为Hybrid开启`global_route_topic=/astar_channel_waypoints`：
   空Path清空局部轨迹；旧任务缓存目标不能驱动新任务。
   独立接线也应开启该参数，避免无解新任务沿旧目标航行。
5. 精确包含途经点不等于零误差经过，实际允许1.5m到达半径。
   实船惯性、曲率与航道宽度仍需另外验证。
6. 当前固定路线不会自动适应任务途中地图封闭；需要显式重规划，
   不能把一次生成的航点表作为后续地图变化的安全认证。

## 测试

```bash
colcon test --packages-select channel_astar_global_planner hybrid_a_star_planner channel_navigation_manager river_chart
colcon test-result --verbose
ros2 run channel_astar_global_planner validate_pipeline.py
# 以下脚本自行启动/关闭整链路，先关闭已有的一键实例：
ROS_DOMAIN_ID=74 ROS_LOG_DIR=/dev/shm/river-fixed-logs ros2 run channel_astar_global_planner test_river_once.py
```

自动测试启动真实C++节点（隔离ROS域73），检查采样、途经点、运动/重复图不重规划、
晚订阅、顺序切换、必经点不跳过、剩余点服务重规划、新目标/同位置重新点击各一次更新与无解任务撤销。
实际海图结果见`INTEGRATED_GUIDE.md`；测试不是完整实船安全认证。

工作空间当前磁盘几乎已满；本次构建使用内存临时目录解决编译临时文件不足，
后续仍建议清理或扩容磁盘。无需删除源码/地图；临时内存目录不适合保存长期测试报告。
