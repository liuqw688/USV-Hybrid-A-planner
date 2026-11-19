# 参数作用及调节说明

本说明对应当前源码与配置，不承诺每个参数可在线更新。多数参数仅在构造时读取：编辑YAML后请停止旧实例并重新启动。launch内追加参数可能覆盖YAML；可用 `ros2 param dump /节点名` 核对运行值。

数值“增大/减小”指其他条件不变时的趋势，不是无限调大都有效。布尔值按开/关解释；话题和frame必须与生产者一致。不要同时降低安全半径、冗余及急停距离来追求直线效率。


## config/dwa_params.yaml

当前配置值不是节点内置默认值，其他launch也可能覆盖它。

| 参数 | 当前YAML值 | 作用、关联及调大/调小影响 |
|---|---|---|
| `path_topic` | `/hybrid_a_star_trajectory` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `costmap_topic` | `/local_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `odom_topic` | `/odom` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `cmd_vel_topic` | `/cmd_vel` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `tracking_only` | `true` | 默认只跟踪Hybrid A*路径；不读取目标船停车圈、DCPA/TCPA或COLREG策略。false仅用于兼容旧版DWA双重动态安全检查。 |
| `control_frame` | `odom` | 坐标系/TF frame名称。修改须与消息头、TF及接收节点一致；不是数值大小参数，缺失或失配会拒绝输入。 |
| `control_frequency` | `20.0` | DWA控制频率(Hz)。增大：响应更快、动态窗口单步更小、算力更高；减小：控制粗、保护响应更迟；与本船20Hz仿真配套。 |
| `max_vel_x` | `4.0` | DWA最大候选前进速度(m/s)。增大：速度偏好更高、需更长制动/预测；减小：巡航更慢；与Hybrid cruise_speed及仿真上限同步。 |
| `goal_deceleration` | `0.5` | DWA终点制动加速度(m/s²)。增大：可更晚制动；减小：更早减速；实际还受acc_lim_x限制，与Hybrid预测模型同步。 |
| `min_vel_x` | `0.0` | DWA候选线速度下界(m/s)。增大：缺少慢速/零候选可能妨碍安全；减小到负数会允许倒车候选，但模型/规则需另验证。 |
| `max_vel_theta` | `0.45` | 最大候选艏摇率(rad/s)，与仿真和Hybrid上限一致。增大：允许更急转但易摆动；减小：转弯更缓但可能跟不上曲率。 |
| `model_mode` | `HIGH_SPEED` | 欠驱动船体模型标签。当前实现按HIGH_SPEED方程计算；修改标签不会自动切换另一套系数。 |
| `model.k_u/k_t/k_turn` | `0.290681/0.295742/2.126739` | 纵向阻尼、推力增益、转弯掉速系数；须与仿真器YAML一致，按实船辨识结果调整。 |
| `model.k_v/k_vr/k_vn` | `0.425383/-1.149012/0.0` | 侧滑阻尼、转艏诱导侧滑、偏航力矩侧向耦合。特别是k_vr符号影响转弯时侧滑方向。 |
| `model.k_r/k_r3/k_n` | `1.342561/0.0/0.781688` | 线性/三次转艏阻尼与偏航力矩增益；影响转艏建立和衰减。 |
| `surge_response_time/yaw_response_time` | `0.8/0.8` | 低层期望速度与转艏率响应时间(s)。减小响应更快但执行器需求更大，增大则更迟缓；DWA与仿真器必须一致。 |
| `max_tau_y/max_tau_n` | `10.0/5.0` | 纵向推力和偏航力矩模型输入限幅；过小会导致跟踪能力不足，过大则可能造成强烈机动。输入单位沿用模型标定约定，用户给出的系数说明未提供物理单位。 |
| `acc_lim_x` | `0.7` | DWA动态窗口线加速度(m/s²)。增大：指令响应快、变化更大；减小：变化慢/制动窗口更受限；应匹配仿真惯性。 |
| `acc_lim_theta` | `1.8` | DWA动态窗口角加速度(rad/s²)。增大：转向更灵敏易摆动；减小：转向缓慢，急避让能力也下降。 |
| `sim_time` | `3.0` | DWA常速候选预测时长(s)。增大：看得远但常速模型误差/计算变大；减小：近视，可能太晚避障；追越近终点另有10s检查。 |
| `sim_granularity` | `0.05` | DWA积分时间步长(s)。增大：点少但碰撞检查粗；减小：预测细、计算/显示开销上升。 |
| `v_samples` | `21` | 线速度候选数。增大：速度选择细、候选数量与w_samples相乘增长；减小：算力低但可能漏安全控制。 |
| `w_samples` | `31` | 角速度候选数。增大：转向选择细、算力增加；减小：转向粗更可能摇摆或无可行候选。 |
| `lookahead_distance` | `5.0` | 基础前视弧长(m)，随速度额外增大。增大：提前转弯、跟踪更平缓但可能切角；减小：贴路径细节、易晚转摆动。 |
| `max_lookahead_distance` | `10.0` | 速度自适应前视上限(m)。增大：高速更远前视/更早转向；减小：近端响应更细但高速可能转弯太迟。 |
| `goal_tolerance` | `0.4` | DWA最终任务点中心位置容差(m)。只在全局活动目标和当前局部路径终点都匹配全局路线末点时停车；中间航点不停车。 |
| `min_cruise_speed` | `0.12` | 转弯限速公式的最低前进参考(m/s)。增大：急弯保持更多前进；减小：更慢转弯；不能绕过安全STOP。 |
| `own_ship_radius` | `0.5` | 本船规划圆形半径(m)。增大：硬净空更大、绕行更宽；减小：更贴近且碰撞裕度减小；Hybrid/DWA须一致。 |
| `safety_buffer` | `1.0` | 两船半径之外的安全冗余(m)。增大：提前/更宽绕行、可行空间减少；减小：更靠近目标、抗跟踪误差更差；两级须一致。 |
| `colregs_risk_distance` | `2.0` | 兼容旧模式的当前两船中心STOP半径(m)；tracking_only=true时由Hybrid A*统一处理，DWA不使用该参数。 |
| `target_odom_topics` | `[/target_boat/odom]` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `target_state_timeout` | `2.0` | 兼容旧模式目标消息有效时间(s)；tracking_only=true时DWA不订阅目标船。 |
| `overtaking_safe_distance` | `3.0` | 追越中心硬净空(m)；普通硬净空为2m。 |
| `lethal_cost_threshold` | `88` | 静态硬危险代价门槛。增大：允许更高代价、可能更贴岸；减小：更保守易无解；Hybrid启用channel时以分源阈值为准。 |
| `unknown_is_obstacle` | `true` | DWA为true拒绝未知、false允许未知参与评分，后者风险大；局部地图活动实现中此项声明但未使用，始终保留-1。 |
| `channel.enabled` | `true` | true：启用分源航道语义融合；false：退回原地图单层判定/评分。开阔水域演示明确关闭，河道模式保持开启。 |
| `channel.lane_costmap_topic` | `/channel/lane_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `channel.obstacle_costmap_topic` | `/channel/obstacle_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `channel.main_lane_value_max` | `20` | 主航道语义上界，与管理器编码一致；修改不是调整船距，失配会把边沿误分类。 |
| `channel.opposite_lane_value_min` | `65` | 对向语义下界，与75编码配套；增大可能漏判左航道，减小可能把航道外误判为航道内；非物理硬阈值。 |
| `channel.obstacle_cost_threshold` | `85` | 独立/channel/obstacle_costmap禁止阈值。增大：允许更高障碍膨胀；减小：更保守更早绕开；航道内也生效。 |
| `channel.center_cost_scale` | `2.5` | DWA把主航道边沿值转换为评分代价倍率。增大：更偏中心；减小：更贴边；过大会逼近硬阈值而使边沿成为禁止候选。 |
| `path_weight` | `3.5` | DWA路径贴合分数权重。增大：更贴Hybrid路径；减小：更愿局部偏离；其他分项与硬约束共同决定。 |
| `progress_weight` | `4.5` | DWA沿路径前进分数权重。增大：更偏向推进；减小：允许保守/侧向机动；不能消除动态硬约束。 |
| `heading_weight` | `1.2` | DWA路径切向一致分数权重。增大：更对齐路径方向；减小：接受更多横向朝向差。 |
| `obstacle_weight` | `4.0` | DWA低代价避障分数权重。增大：更远离高代价；减小：更偏效率；危险候选仍硬淘汰。 |
| `speed_weight` | `30.0` | DWA速度偏好权重。增大：安全候选更偏高速；减小：跟踪/避障更容易选慢速；终点限速仍生效。 |
| `smooth_weight` | `0.45` | DWA命令连续性分数权重。增大：变化小、可能响应慢；减小：响应快但角速度更易来回切。 |
| `target_weight` | `1.5` | DWA靠近前视点分数权重。增大：更追前视点、可能削角；减小：路径整体贴合等分项更占主导。 |
| `path_stale_timeout` | `3.0` | Hybrid有效路径接收过期时间(s)。增大：可继续跟较旧路径；减小：更快停车但更容易因搜索延迟停船。 |
| `costmap_stale_timeout` | `0.0` | 地图接收过期时间(s)，<=0禁用时间过期但保留frame检查。增大：容忍旧图；减小正数：更快保护；静态一次地图用0。 |

## config/sim_params.yaml

当前配置值不是节点内置默认值，其他launch也可能覆盖它。

| 参数 | 当前YAML值 | 作用、关联及调大/调小影响 |
|---|---|---|
| `start_x` | `-270.0` | 本船起点东向坐标(m)。增大向东、减小向西；必须在地图有效安全水域，重启才加载。 |
| `start_y` | `-270.0` | 本船起点北向坐标(m)。增大向北、减小向南；必须与海图坐标/scale一致。 |
| `start_yaw` | `2.14675498` | 本船初始ROS yaw(rad)，0东、正逆时针。增大逆时针转、减小顺时针；低速首次COG会采用该方向。 |
| `initial_speed` | `4.0` | 启动实际/目标线速度(m/s)。增大：初始运动更快；减小：更慢；本船受限速和惯性、虚拟船直接匀速积分。 |
| `max_linear_speed` | `4.0` | 本船仿真线速度硬上限(m/s)。增大：允许更快命令；减小：截断DWA速度、预测到达时间可能失配；三处航速同步。 |
| `model_mode` | `HIGH_SPEED` | 欠驱动六状态船体模型。 |
| `model.*` | 与DWA参数相同 | HIGH_SPEED动力学系数。仿真与DWA预测必须相同，否则局部轨迹预测会与仿真运动不符。 |
| `surge_response_time/yaw_response_time` | `0.8/0.8` | 速度/转艏率参考响应时间(s)，并需与DWA配置一致。 |
| `max_tau_y/max_tau_n` | `10.0/5.0` | 合推力/偏航力矩输入限幅，影响船体加速与转艏能力；输入单位依模型标定。 |
| `max_angular_speed` | `0.45` | 艏摇率硬限幅(rad/s)，应与MPC `max_yaw_rate`及Hybrid `usv.max_yaw_rate`一致。 |

## config/mpc_params.yaml

一键河道启动默认`use_mpc=true`，由MPC替代DWA输出控制；它发布τ_y/τ_N，不发布Twist。参数改完需重启节点。

| 参数 | 当前值 | 作用、关联及调节影响 |
|---|---:|---|
| `control_frequency` | `10.0` Hz | 滚动优化频率。增大控制更及时、计算更密；受CPU和ROS执行负荷限制。 |
| `prediction_step` / `prediction_horizon` | `0.10` s / `5.0` s | 预测积分间隔与时域。减小步长或加长时域能更细/看更远，但优化计算量成比例增加；船模误差也会累积。 |
| `control_knots` | `5` | 时域内独立推力片段数。增大可表达更复杂的动作，但优化变量与局部极值增多；减少动作更平缓、控制分辨率降低。 |
| `optimization_sweeps` | `2` | 每周期坐标下降扫描轮数。增大更可能找到低代价序列但会增加CPU时延；建议先确认10Hz预算足够。 |
| `max_speed` / `max_yaw_rate` | `4.0` m/s / `0.45` rad/s | 必须与Hybrid和仿真限值一致。设得高于仿真器会使预测过于乐观；调低则整体更保守。 |
| `max_tau_y` / `max_tau_n` | `10.0` / `5.0` | 纵向合推力和偏航力矩限幅，须与模拟/船体模型一致。过小会转不动或加速不足，过大动作更激烈。单位遵从水动力标定输入。 |
| `model.k_*` | 与船模参数一致 | 六状态欠驱动模型阻尼、耦合、推力增益；k_vr控制转艏侧滑。MPC、仿真、Hybrid需同步。 |
| `surge_response_time` / `yaw_response_time` | `0.8` s / `0.8` s | 影响参考控制序列的响应惯性和COLREG可达转向包络。增大更迟缓，减小更急；应按真实船响应辨识。 |
| `own_ship_radius` / `safety_buffer` | `0.5` m / `1.0` m | 地图船体九点包络半径与目标船净空冗余；Hybrid同参数必须一致。调大更安全、可行水域变窄。 |
| `channel_enabled` | `true` | true时与Hybrid一致：航道内忽略岸边膨胀，但仍检查`/chart_static_obstacles`和`/channel/obstacle_costmap`；没有这些分层地图不可启用直控。 |
| `shoreline_cost_threshold` / `channel_obstacle_threshold` | `88` / `85` | 航道外岸边硬阈值和独立障碍膨胀硬阈值，须与Hybrid配置同步。增大更贴近高代价格，减小更保守。 |
| `dynamic_hard_distance` / `dynamic_soft_distance` | `4.0` m / `15.0` m | MPC动态船硬净空与软代价外缘，硬值取大者：配置值或本船半径+目标半径+buffer；软范围越大越早偏好远离。必须和Hybrid安全参数一起评估。 |
| `emergency_stop_distance` | `2.0` m | 当前船与目标船中心距急停阈值；不是规划软域。危险路径通常会先被动态硬净空淘汰。 |
| `goal_deceleration` / `intermediate_goal_speed` | `0.5` m/s² / `2.0` m/s | 最终点使用停车包络；中间点可按前方曲率减速，但不强制停车。下项约束弯道参考速度，并至少保持经过点速度下限。 |
| `max_lateral_acceleration/curvature_preview_distance` | `0.6/12.0` | 前方曲率速度偏好与预览长度(m)。减小加速度尺度更稳更慢，增大预览长度更早减速；受速度下限影响，不是严格物理约束。 |
| `max_torque_rate/terminal_heading_fade_speed` | `2.0/0.35` | 力矩变化率上限与终点低速航向消退尺度(m/s)。降低前项更平顺，增大后项更早弱化终点纠艏。 |
| `terminal_hold_radius` | `0.5m` | 低速且剩余侧滑漂移可接受时进入稳定终点停车；中间航点不进入。调大位置容差更宽松。 |
| `cross_track_recovery_distance` | `8.0` m | MPC横向偏差转向回正的虚拟前视距离。减小：更积极回到轨迹、响应更强；增大：回正更平缓但横向误差收敛更慢；不改变障碍硬检查。 |
| `weight.cross_track` / `weight.course` | `12.0` / `12.0` | 横向偏差和路径切向代价。增大贴线更强；过大可能让控制更激进，障碍安全仍优先。 |
| `weight.progress` / `weight.speed` | `1.4` / `2.0` | 路径前进和速度跟踪偏好，两者使用一致的弯道/终点减速参考。增大更积极追踪参考。 |
| `weight.sideslip` / `weight.yaw_rate` | `3.0` / `2.0` | 抑制侧滑和过大的持续转艏率。增大更温和但转向可能滞后；障碍需要时由硬安全约束迫使绕行。 |
| `weight.torque_slew` / `weight.reverse_turn` | `0.6` / `2.5` | 惩罚本周期与上次实际命令、预测内相邻力矩跳变及换向，增大可抑制左右反舵“S”形；过大可能减慢避让响应。 |
| `/mpc/predicted_trajectory` | 青蓝 Marker | RViz控制预测，不是全局路径；Marker被删除时表示安全序列不存在或输入不可用。 |

## MPC与低速掉头规划的连接

`speed_limit_topic: /hybrid_a_star/speed_limit` 接收与局部Path配对的TwistStamped消息；
linear.x限制参考速度，掉头回退默认为2m/s，普通路径恢复4m/s。惯性使实际速度只能逐渐减小，
并非直接截断仿真速度。`require_speed_profile: true`要求新几何路径有对应限速再驱动；
同形路径心跳不会重复等待。独立旧规划器不发布该消息时才关闭此开关。

近距机动还可能发布1.2m/s参考上限。`projection_forward_distance=6m`限定实测船位
匹配的沿程窗口，至少覆盖当前2秒航程，向后2m；避免掉头曲线在起终点附近交叠时
提前跳到末段刹车。增大更易重捕获但跳段风险增加；减小更稳但严重偏离时恢复较慢。
终点停车保持要求已走到路径末段，而不是初始位置近终点就停车。

## 跨包一致性检查（原参数）

- 当前本船最高速度4m/s；小型无人船等效规划半径均为0.5m、安全冗余1m，因此普通中心硬净空为2m。Marker长1m宽1.5m只是显示外形。
- Hybrid在7m软域提前规划、2m硬净空由Hybrid负责；tracking_only=true时DWA只跟踪输入Path，目标船距离不会在DWA中触发停车。
- AIS式远程监视保持80m/40s不变；行动层24m/16s、紧急层8m/6s。风险圈不是感知截止范围，也不等于软/硬避让域。
- 地图尺寸、原点、分辨率与frame应一致；地图膨胀和独立障碍层不得混淆。
- 主航道、外部、对向代价是偏好，安全硬过滤优先；降低偏好不解除危险栅格阻挡。
