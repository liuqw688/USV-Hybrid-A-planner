# 航道管理器：功能、调用关系与使用说明

本包把静态分道几何变成当前航行方向下的全图主/对向语义，不负责A*搜索或速度控制。
参数详表见 PARAMETERS.md；现有README保留历史操作说明。

## 1. 文件与职责

src/channel_navigation_node.cpp是唯一运行节点，include/channel_navigation_manager/geometry.hpp实现几何与传播。
launch/river_navigation.launch.py是整链路一键入口，使用进程文件锁避免重复本船/TF实例。
test/test_geometry.cpp验证方向换算、填充、中心梯度、后方目标和弯河传播。
一键默认不发布虚拟船；use_virtual_boat:=true用于显式测试，目标由用户RViz设置。

## 2. 输入输出与包关系

| 方向 | 话题/类型 | 联系 | 用途 |
|---|---|---|---|
| 输入 | /chart_lane_data / String(JSON) | river_chart | TSSLPT多边形、lane_id、ORIENT，静态一次发布并锁存 |
| 输入 | /local_costmap / OccupancyGrid | 局部地图包 | 复制网格元数据，保证语义图对齐 |
| 输入 | /odom / Odometry | 本船仿真 | 实际运动方向COG及附近种子位置 |
| 输入 | /goal_pose / PoseStamped | RViz最终任务 | 判断是否准备反向，不用中间活动目标翻转任务 |
| 输出 | /channel/lane_costmap / OccupancyGrid | 全局A*、Hybrid、DWA | 主0..20、外45、对向75；不是物理占用概率 |
| 输出 | /channel/navigation_markers / MarkerArray | RViz | 角色边界、颜色与通航箭头 |
| 输出 | /channel/status / String(JSON) | 诊断 | COG、策略方向、是否反向、当前区域/lane_id |

绿色/chart_markers只是海图显示。实际分类读取/chart_lane_data，不依赖Marker颜色。
不修改/local_costmap物理地图，原始陆地/独立感知障碍层由规划器另行融合。

## 3. 启动与角色判定流程

~~~mermaid
flowchart TD
 A["main初始化/构造参数与订阅"] --> B["缓存静态lane JSON和地图"]
 C["odom: yaw + atan2(body_vy,body_vx)得到COG"] --> D["updatePolicyCourse"]
 E["目标明显在后方:距离>=10m、方位差>=110度"] --> D
 D --> F["低速保持方向;反向意图预选新方向"]
 B --> G["publishIfReady"]
 F --> G
 G --> H["连续主航道传播 + 0..20中心梯度"]
 H --> I["发布语义图/角色Marker/status"]
~~~

速度低于0.1m/s时保留旧有效方向；第一次低速用yaw初始化。
后方任务先用目标方位，COG接近新方向35度后退出提前反向状态。
COG策略变化至少12度才重建全图，避免20Hz船位回调每次重新刷图。
所有loaded多边形都参与生成全图，但只有从种子传播到的组为主，其他航道暂归75。

## 4. 函数调用流程

~~~mermaid
flowchart LR
 A["laneCallback/costmapCallback"] --> D["publishIfReady"]
 B["odomCallback/goalCallback"] --> C["updatePolicyCourse / isReverseTask"]
 C --> D
 D --> E["continuousMainLaneMask"]
 E --> F["isSameNavigationDirection + pointPolygonDistance选择种子"]
 E --> G["polygonDistance + 局部ORIENT差传播"]
 D --> H["fillPolygon + applyMainLaneCenterGradient"]
 H --> I["publishMarkers / publishStatus"]
~~~

种子仍以策略方向与ORIENT约90度以内同向筛选，再找离船最近者；没有同向候选时回退最近航段。
之后不是所有航段与船头单独比较，而是沿160m邻接、80度局部方向连续性传播；可以经过累计大弯。
这是近似交通流传播，不是严格几何左右/拓扑验证。断开的同向段可能被归75，阈值太宽也可能误连支流。

## 5. 一键节点启动关系

~~~mermaid
flowchart TD
 L["river_navigation.launch.py取得单实例锁"] --> R["river_chart + map到odom静态TF"]
 L --> M["local_costmap_generator"]
 L --> C["channel_navigation_node"]
 L --> S["boat_simulator"]
 L --> A["channel_astar_node"]
 L --> H["hybrid_a_star_node"]
 L --> D["dwa_planner"]
 L --> V["可选虚拟船及RViz"]
~~~

launch动作列出顺序不代表数据同步顺序；各节点通过输入就绪检查等待，不应依赖启动后固定sleep来保证地图已加载。
全局Path空值使Hybrid撤销任务，DWA收到空局部Path后零速保护；全局角色纠偏可能产生短暂停车。

## 6. 调整与边界

更居中：先适量增大center_clearance/main_lane_edge_value，再协调Hybrid与DWA中心权重；编码必须保持一致。
降低角色抖动：增大reclassify_angle/minimum_course_speed，但会延迟更新。
急弯漏传播：检查ORIENT和邻接几何，再调连续角/邻接距，不宜无条件扩大。
当前goalCallback直接使用目标数值与odom比较，不进行TF转换；一键map→odom是单位变换所以相同。
若以后采用非单位map→odom，管理器也需要统一目标坐标，这不是本次注释改造已解决的功能。


## 源码函数逐项索引

以下按文件列出已注释函数。`[功能与联系]`代码注释可直接定位职责；同名重载/不同类函数分列。函数内局部计算与分支说明保留原有注释。流程图展示主要调用链，表格覆盖辅助函数、入口和测试。

### src/channel_navigation_node.cpp

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `ChannelNavigationNode` | 创建海图几何、地图、船位和目标订阅；加载分类阈值，等待输入后生成全图航道语义供两级规划使用。 |
| `laneCallback` | 解析river_chart的航道JSON，保存多边形、lane_id和ORIENT；通过publishIfReady强制重建语义图，不搜索或清空全局路线。 |
| `costmapCallback` | 缓存物理地图及其网格元数据；航道节点用于对齐语义图，规划/控制节点用于碰撞和数据就绪检查。 |
| `odomCallback` | 缓存里程计并计算COG；低速保持方向，经updatePolicyCourse与publishIfReady按角度阈值重建语义图。 |
| `goalCallback` | 缓存最终任务目标，更新后方任务的反向意图，再按需重建航道分类；不管理全局航点。 |
| `updatePolicyCourse` | 通常采用实际COG；目标明确在后方时提前采用反向任务方位，转向完成后回到COG，供种子航段选择。 |
| `publishIfReady` | 检查静态几何、地图及策略方向，按角度阈值节流分类；传播主航道、绘制语义中心梯度并发布图/可视化/状态。 |
| `publishMarkers` | 绘制主/对向航段和策略方向供RViz检查分类；不修改物理地图或控制命令。 |
| `publishStatus` | 发布COG、策略方向、反向状态、所在区域和lane_id的JSON诊断，帮助检查主航道选择。 |
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |

### launch/river_navigation.launch.py

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `_acquire_single_instance_lock` | 持有进程级文件锁拒绝重复一键启动；防止多个本船仿真器发布相同base_link。 |
| `generate_launch_description` | 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。 |

### include/channel_navigation_manager/geometry.hpp

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `normalizeAngle` | 归一化角度，消除跨±pi的跳变；方向比较、运动积分和COLREG约束共同使用。 |
| `s57OrientationToYaw` | 将真北为0顺时针的S-57 ORIENT换算为ROS东为0逆时针yaw；避免海图与运动学方向定义混淆。 |
| `isSameNavigationDirection` | 以方向余弦是否非负判断同向候选（约90度以内）；当前连续算法仅用它筛选种子航段。 |
| `laneValue` | 直接方向分类的辅助函数；当前主流程采用continuousMainLaneMask，不应误认为所有航段仍独立比较船头角。 |
| `pointSegmentDistance` | 计算点到线段的最近距离；支撑多边形距离和邻接传播，不是动态船碰撞检查。 |
| `pointPolygonDistance` | 计算点到航段多边形的距离；种子选择用它寻找本船附近同向航段。 |
| `polygonDistance` | 估计两个航段多边形的空间距离；与连续方向阈值联合决定传播邻接。 |
| `continuousMainLaneMask` | 从最近同向种子以队列沿空间邻接和局部ORIENT连续性传播主航道；避免累计大弯时被初始船向误分类。 |
| `bearingAndDistance` | 由两点求方位与中心距离；反向任务判定调用它，要求坐标系一致。 |
| `isReverseTask` | 联合目标方位差及最小目标距离判断反向任务；正常左右前方目标不会直接翻转全河道角色。 |
| `pointInPolygon` | 射线交点法判断点在多边形内部；用于区域状态和距离计算。 |
| `fillPolygon` | 扫描线填充航道语义区域；仅绘制区域类别，不覆盖物理陆地或独立障碍层。 |
| `applyMainLaneCenterGradient` | 在主航道范围内形成0..边缘值的软代价梯度；保留全部主航道可通行，规划器用权重偏向中心。 |
