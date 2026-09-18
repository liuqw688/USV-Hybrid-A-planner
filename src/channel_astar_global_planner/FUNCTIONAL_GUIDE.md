# 全局航点规划：功能、调用关系与使用说明

本文件说明当前源码行为，参数详表见同目录 PARAMETERS.md，历史测试见 INTEGRATED_GUIDE.md。
本包只产生任务级航点，不直接输出速度；黄色全局连线不是碰撞验证过的控制曲线。

## 1. 文件和节点职责

src/channel_astar_node.cpp 为C++主程序，ChannelAStarNode为ROS节点。launch/channel_astar.launch.py加载YAML。
scripts/test_river_once.py自行启动整条河道链路做短时联动；test/test_fixed_mission.py以真实C++节点做隔离输入测试。
scripts/validate_pipeline.py是历史右航道校验器：它要求所有点0..20，且不转换目标frame；左/外终点及非单位TF可能误报，不能作为当前全功能验收器。

## 2. 包间接口

| 方向 | 话题/类型 | 来源或去向 | 作用 |
|---|---|---|---|
| 输入 | /goal_pose / PoseStamped | RViz | 每次发布均是新任务，同位置重新点击也重新规划 |
| 输入 | /odom / Odometry | dwa_deep本船仿真 | 起点和航点切换，不因移动自动搜索 |
| 输入 | /channel/lane_costmap / OccupancyGrid | 航道管理器 | 主0..20、外45、左75及中心偏好 |
| 输入 | /local_costmap / OccupancyGrid | 局部地图包 | 航道外物理危险值检查，须与语义图同网格 |
| 输入 | /tf、/tf_static | TF发布者 | 将map目标位置及姿态变换到语义图frame |
| 输出 | /astar_channel_waypoints / Path | RViz、Hybrid任务保护 | 锁存完整航点表，黄色显示；空表撤销旧任务 |
| 输出 | /goal_pose_from_astar / PoseStamped | Hybrid | 当前一个活动目标，顺序更新 |
| 人工服务 | /channel_astar/replan / Trigger | 运维 | 重新搜索剩余任务；success仅表示排队 |

实际话题名称可由YAML更改。Path和活动目标为Reliable、Transient Local，晚订阅也能取到最近值。
只配置Path话题不足以驱动Hybrid；它需要当前PoseStamped，两者不能混成同名不同类型。

## 3. 节点启动过程

~~~mermaid
flowchart TD
 A["main: rclcpp初始化"] --> B["构造节点: 参数/TF/订阅/发布/人工服务"]
 B --> C["spin派发回调: 没有规划timer"]
 C --> D["等待地图、安全图、odom及目标"]
 D --> E["待任务满足稳定等待与最小间隔"]
 E --> F["TF就绪: plan一次"]
~~~

目标先于地图也不会丢失；TF缺失时待任务保持，由输入回调再尝试。当前依赖持续odom回调来推进等待。
安全图与语义图必须同frame、尺寸、分辨率和完整origin；不能按数组下标混用两幅不同网格。

## 4. 功能流程

~~~mermaid
flowchart TD
 G["新目标: 重置途经点进度"] --> Q["撤销旧Path并排队"]
 L["主/对向角色互换"] --> P["保留完成进度并排队纠偏"]
 P --> Q
 Q --> W["输入就绪 + 稳定等待"]
 W --> T["转换目标到航道图frame"]
 T --> S["逐段A*: 剩余途经点 + 最终目标"]
 S --> K{"全部段成功?"}
 K -- 否 --> N["保持空路线;不周期重试"]
 K -- 是 --> R["弧长100m采样 + 精确保留必经点"]
 R --> O["一次发布全表 + 当前目标"]
 O --> V["odom距离判断:普通8m/必经1.5m切换"]
~~~

连续角色变化重新计稳定等待，合并到最新快照一次搜索；重复图、0..20梯度变化和普通运动不触发。
对错误分类的纠偏只是跟随最新分类，不能证明分类算法正确。频繁角色翻转时先撤销路线可能短时STOP。

## 5. 函数调用流程

~~~mermaid
flowchart LR
 A["goalCallback/laneCallback/odomCallback/安全图回调"] --> B["tryPendingGoal"]
 B --> C["plan"]
 C --> D["searchCells"]
 D --> E["worldToGrid / traversable / outsideSafe / index"]
 C --> F["gridToWorld + 航点采样"]
 F --> G["publishActiveGoal"]
 H["odomCallback"] --> I["switchWaypoint"]
 I --> G
~~~

plan只有所有段成功才发布；失败不发布半条任务。中间朝向指向下一段，最终姿态保留用户要求。
起末段可以连接左/外水域，主体仍限制主航道；区域代价与物理危险分开。
安全图检查按航道外格中心，并不验证整船、静态陆地核心、独立航道障碍或动态船轨迹；这些由Hybrid/DWA补充。
稀疏连线可穿出河弯，必须由Hybrid重新生成可跟踪曲线，不能直接将黄色Path用于速度控制。

## 6. 操作、调参和故障定位

一键运行使用 ros2 launch channel_navigation_manager river_navigation.launch.py，RViz保持map并用2D Goal Pose发目标。
初始普通航点100m、切换8m、必经点1.5m；left/outside末段连接默认100m。
配置途经点须位于主航道且使用航道图frame；无点时注释via_points，Humble不能解析空数组。
航道外目标>=88或未知拒绝；增大末段半径可连接更远非主航道，但也扩大允许非主行程。
没有路径时依次检查节点名/YAML根、话题发现、两幅图对齐、TF、起终点值、连通和迭代上限。
参数启动时读取，修改后重启；人工replan保留完成点，新目标重新经过全部配置点。


## 源码函数逐项索引

以下按文件列出已注释函数。`[功能与联系]`代码注释可直接定位职责；同名重载/不同类函数分列。函数内局部计算与分支说明保留原有注释。流程图展示主要调用链，表格覆盖辅助函数、入口和测试。

### launch/channel_astar.launch.py

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `generate_launch_description` | 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。 |

### test/test_fixed_mission.py

测试/诊断程序，不属于生产控制循环。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `test_fixed_mission` | 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。 |
| `spin_for` | 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。 |
| `nonempty` | 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。 |
| `current_x` | 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。 |

### scripts/test_river_once.py

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |
| `spin` | 诊断脚本限时派发回调并检查条件；等待期间不改变正常规划频率，超时用于判定测试失败。 |
| `location` | 按真实栅格原点姿态和分辨率求格中心世界坐标；用于选取左/航道外测试终点。 |
| `score` | 计算候选格到标准测试终点的平方距离；仅选择测试场景，不参与生产A*代价。 |

### scripts/validate_pipeline.py

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `__init__` | 读取本节点参数并创建ROS输入输出/调度；后续回调与主循环关系见包内功能说明。 |
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |
| `_lane` | 缓存最新航道图供诊断校验，未执行生产航道分类。 |
| `_goal` | 缓存请求终点供诊断对比；本历史校验器不做跨frame转换，map非单位TF场景可能误报。 |
| `_astar` | 缓存全局稀疏Path供诊断检查；生产航点切换由全局C++节点完成。 |
| `_active` | 缓存当前Hybrid活动目标供接口存在性核验。 |
| `_hybrid` | 缓存最近非空Hybrid路径供诊断；此脚本不能独立证明控制安全或最终到达。 |
| `lane_value` | 把世界点变换到语义格并读取类别，用于历史右航道校验；不是碰撞验证。 |
| `validate` | 检查话题存在、全局点在右航道及末点相等；旧规则不支持新左/外终点与跨frame比较，可能误报，详见文档边界。 |

### src/channel_astar_node.cpp

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `ChannelAStarNode` | 读取全局航点/语义/安全参数，建立TF、输入回调及锁存输出；不创建规划timer，搜索由目标或航道角色变化触发。 |
| `index` | 将二维栅格坐标映射为行优先下标；供安全查询、A*邻居扩展及父节点数组共用。 |
| `worldToGrid` | 按航道图原点、分辨率和原点姿态转换世界坐标并检查边界；搜索前验证起点和目标。 |
| `gridToWorld` | 将航道格中心还原为世界坐标；用于把A*父链转换为供Hybrid接收的稀疏航点。 |
| `traversable` | 综合区域类别、末段/起段连接半径与航道外安全阈值筛选格子；searchCells同时用它检查斜向切角。 |
| `outsideSafe` | 查询对齐的物理安全图，拒绝航道外未知或高代价格；仅是中心栅格筛选，最终曲线由Hybrid验证。 |
| `laneCallback` | 缓存航道输入并识别主/对向角色互换；纠偏清空旧路线、保留已完成途经点，随后尝试待处理任务。 |
| `goalCallback` | 每次RViz目标都启动新任务，撤销旧路线并重置途经点进度；输入就绪后执行一次全局A*。 |
| `odomCallback` | 缓存本船位姿，按到达半径推进活动航点并尝试待处理任务；普通移动不重复已完成搜索。 |
| `tryPendingGoal` | 无timer的待任务入口：确认地图对齐、船位及稳定等待后调用plan；没有待任务时立即返回。 |
| `plan` | 将目标通过TF变换到规划图，逐段经过剩余途经点搜索并按弧长采样；全部成功后一次发布完整Path和当前活动目标。 |
| `searchCells` | 8邻域A*累计中心与区域代价，禁止斜穿障碍角；主体走主航道，起末段允许受限非主航道连接。 |
| `switchWaypoint` | 依据船位顺序推进普通8m/必经1.5m目标，并记录完成的途经点；最终点交给DWA终点停车。 |
| `publishActiveGoal` | 发布当前一个PoseStamped给Hybrid；不能把整表同时当目标发送，否则只剩最后一个目标。 |
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |
