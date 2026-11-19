# 参数作用及调节说明

本说明对应当前源码与配置，不承诺每个参数可在线更新。多数参数仅在构造时读取：编辑YAML后请停止旧实例并重新启动。launch内追加参数可能覆盖YAML；可用 `ros2 param dump /节点名` 核对运行值。

数值“增大/减小”指其他条件不变时的趋势，不是无限调大都有效。布尔值按开/关解释；话题和frame必须与生产者一致。不要同时降低安全半径、冗余及急停距离来追求直线效率。


## config/hybrid_a_star_params.yaml

当前配置值不是节点内置默认值，其他launch也可能覆盖它。

| 参数 | 当前YAML值 | 作用、关联及调大/调小影响 |
|---|---|---|
| `planner_frequency` | `5.0` | 事件安全监测与缓存路径刷新频率(Hz)，不是固定搜索频率。增大可更快发现新风险；只有触发条件成立才运行Hybrid A*。 |
| `event_driven.enabled` | `true` | true：新目标、预测碰撞/规则冲突、静态阻断或明显偏航时才搜索；false：每个timer周期搜索。 |
| `event_driven.collision_trigger_distance` | `2.0` | 同一会遇已成功规划后，剩余路径重新进入2m预测硬域才再次搜索；首次会遇由DCPA/TCPA提前触发。 |
| `event_driven.cross_track_limit` | `8.0` | 本船偏离缓存路径超过此值时重规划。增大更愿继续旧路；减小则偏航后更快改线。 |
| `event_driven.target_course_change` | `0.0873` | 目标船相对上次成功规划航向变化超过5度时视为新机动并重新规划。 |
| `event_driven.target_speed_change` | `0.3` | 目标船相对上次成功规划速度变化超过0.3m/s时重新规划。 |
| `event_driven.search_attempts` | `2` | 最大候选次数(1~3)。第一条满足长度与不绕圈质量门槛时立即发布，只有第一条异常才搜索备用候选；增大只会增加异常场景的重试上限。 |
| `event_driven.max_path_length_ratio` / `max_path_extra_distance` | `3.0` / `40.0` | 路线长度质量门槛，超过则不发布路径，等待下一次规划，避免终点附近大绕圈。 |
| `event_driven.goal_loop_radius` / `goal_loop_min_arc` | `3.0` / `8.0` | 终点附近回到目标半径内后仍绕行超过该弧长即判为绕圈并丢弃。 |
| `path_failure_hold_time` | `3.0` | 最近路径有效性标志保留时间(s)。增大：更晚标为无效；减小：更早失效；当前不直接清除RViz路径，停车由DWA路径过期负责。 |
| `own_odom_timeout` | `5.0` | Hybrid船位消息时间容忍(s)。增大：容忍搜索阻塞但允许较旧船位；减小：更快失联保护、可能误判计算延迟。 |
| `trajectory_topic` | `/hybrid_a_star_trajectory` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `goal_topic` | `/goal_pose_from_astar` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `costmap_topic` | `/local_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `global_frame` | `odom` | 坐标系/TF frame名称。修改须与消息头、TF及接收节点一致；不是数值大小参数，缺失或失配会拒绝输入。 |
| `base_frame` | `base_link` | 坐标系/TF frame名称。修改须与消息头、TF及接收节点一致；不是数值大小参数，缺失或失配会拒绝输入。 |
| `motion_model` | `USV_DYNAMIC` | 搜索运动模型。默认六状态欠驱动模型积分纵向速度、侧滑和艏摇惯性；`USV_KINEMATIC`保留定速近似、`CAR_LIKE`保留自行车模型。更改后重启。 |
| `usv.max_yaw_rate` | `0.45` | USV最大艏摇角速度(rad/s)；动态参考还受 `√(u²+v²) / min_turning_radius` 限制，低速时随合速度缩小，静止时不允许规划原地急转。增大允许更急转，减小轨迹更宽缓但窄河弯更难通过。 |
| `usv.gentle_turn_sampling` | `true` | 航道普通缓弯增加±1/16、±1/8最大转向量候选并细分零附近艏摇率状态，避免缓转被直航合并；同时开启搜索后的船模宽弧优化。开启时该场景每节点11个候选，关闭恢复7个且关闭宽弧优化；开阔水域/掉头采样、最大转向和最小半径保持原值。 |
| `usv.model.k_u/k_t/k_turn` | `0.290681/0.295742/2.126739` | 动态船模纵向阻尼、推力增益和转弯掉速项；必须与仿真`model.*`及MPC一致。 |
| `usv.model.k_v/k_vr/k_vn` | `0.425383/-1.149012/0.0` | 侧滑阻尼、艏摇诱导侧滑和偏航耦合。`k_vr`符号决定左右转时侧向速度方向，须按实际辨识。 |
| `usv.model.k_r/k_r3/k_n` | `1.342561/0.0/0.781688` | 线性/三次艏摇阻尼及偏航力矩增益；决定转艏建立速度，必须与MPC/仿真一致。 |
| `usv.surge_response_time/yaw_response_time` | `0.8/0.8` s | 将搜索动作换算为纵向推力/偏航力矩的参考响应常数。减小动作更敏捷但更容易急转；增大响应滞后、曲线更缓；同步修改MPC与仿真。 |
| `usv.max_tau_y/max_tau_n` | `10.0/5.0` | 动态外推时推力/力矩输入限幅，需与MPC及仿真配置一致。调低将缩小加速/转向能力；调高会扩大可达控制范围。单位遵从船模标定。 |
| `usv.integration_step` | `0.05` s | RK4最大子步。减小能更细检查动态船和局部曲线、搜索耗时增加；增大会降低精度。仿真当前步长也是0.05s。 |
| `step_size` | `2.0` | Hybrid每次运动基元长度(m)。增大：扩展少、细节粗且更难窄处绕行；减小：搜索细但状态/检查更多。 |
| `min_turning_radius` | `8.0` | USV动态模式下限制参考艏摇率：`|r_ref| ≤ min(usv.max_yaw_rate, √(u²+v²)/min_turning_radius)`；当前物理艏摇率上限仍为0.45rad/s，模型约束不变。汽车模式下仍作为启发估计半径。增大更宽缓，减小允许更急弯。 |
| `wheelbase` | `5.0` | 仅 `CAR_LIKE` 使用的轴距(m)。增大：相同转角转弯半径更大；USV模式忽略。 |
| `max_steering_angle` | `0.5` | 仅 `CAR_LIKE` 使用的最大前轮转角(rad)，半径约为wheelbase/tan(angle)。USV模式改用艏摇角速度，不读取此参数。 |
| `d_theta` | `0.10` | 姿态离散角步长(rad)。增大：状态少但航向粗；减小：航向细、状态和搜索代价增加。 |
| `max_iterations` | `250000` | Hybrid最大扩展/出队次数。增大：长路线更易完成、延迟/算力增加；减小：易搜索失败并触发DWA路径过期停车。 |
| `lethal_cost_threshold` | `88` | 静态硬危险代价门槛。增大：允许更高代价、可能更贴岸；减小：更保守易无解；Hybrid启用channel时以分源阈值为准。 |
| `stability.reference_path_weight` | `0.5` | 偏离上次安全路径的软代价倍率。增大：跨周期路线更稳定但可能保留绕路；减小：更追求当前短路但更易跳线；不锁死旧路线。 |
| `stability.near_field_reference_weight` | `12.0` | 本船近前方跨周期稳定平方代价。增大：路径两侧跳变更少；减小：改线更灵活但DWA前视点更易跳动。硬碰撞与COLREG约束优先。 |
| `stability.near_field_reference_horizon` | `45.0` | 近场稳定项从本船到前方该距离(m)线性衰减为0。增大：更长区段稳定；减小：只保护船首近段。 |
| `stability.near_field_deviation_scale` | `3.0` | 偏离达到该米数时近场代价约等于near_field_reference_weight。增大：约束变柔；减小：小偏离就受强惩罚；不是硬走廊。 |
| `stability.recovery_weight` | `4.0` | 恢复方向仍不安全时的额外近端连续性代价。增大：减少避让中跳线；减小：更灵活但易摇摆；恢复安全时门控关闭。 |
| `stability.recovery_horizon` | `6.0` | 风险恢复软偏好的近端时间尺度(s)。增大：影响更长前方范围；减小：只影响船前短段；不是强制直航或解锁等待时间。 |
| `prediction.goal_braking` | `true` | true：动态到达时间考虑终点减速；false：更接近全程巡航外推，终点附近CPA同步可能偏乐观。 |
| `prediction.goal_deceleration` | `0.5` | 终点到达模型制动加速度(m/s²)。增大：预测可更晚减速、到达更快；减小：更早减速、到达更慢；与DWA终点制动匹配。 |
| `prediction.goal_tolerance` | `0.4` | 动态预测终点剩余距离扣除容差(m)。增大：更早视为终点邻域；减小：更接近精确目标；与DWA goal_tolerance匹配。 |
| `prediction.minimum_speed` | `0.3` | 预测积分的速度下限(m/s)。增大：估计到达更快；减小：估计更慢、更保守且时间更大；不是实际船速下限。 |
| `stability.reference_path_max_distance` | `12.0` | 旧路线偏离代价饱和距离(m)。增大：远离旧路仍受约束；减小：更早饱和、允许必要大绕行更容易。 |
| `stability.steering_change_weight` | `1.5` | 相邻基元转角改变惩罚。增大：减少左右反复短弧；减小：搜索更灵活、整体更波浪。 |
| `stability.steering_magnitude_weight` | `12.0` | 持续转向控制量平方代价，动态船模提供±0.25低艏摇率采样。增大：更愿意用更长、更大半径的弯；减小：可在较短距离内急转。12.0偏大半径，窄河弯可降到6–10；硬避障/COLREG仍优先。 |
| `stability.turnaround_heading_threshold` | `2.0944` rad | 起终航向变化达到120°后触发掉头偏好；阈值减小会让更多转弯使用紧凑转向，增大则只影响更接近掉头的路线。 |
| `stability.turnaround_steering_scale` | `0.25` | 掉头时把持续艏摇率/舵幅软代价乘此比例，让搜索优先使用模型允许的较紧转向。减小更紧、增大更宽；不突破艏摇率/船模/碰撞/COLREG约束。 |
| `stability.turn_reversal_weight` | `24.0` | 记住最近一次非零转向，直线基元不会清除这段历史；反向转弯时增加一次软代价。增大：开阔水域更偏向单向C弯；减小：更容易规划S弯。只影响偏好，不禁止避障/COLREG所需反向动作。建议12–36逐步调节。 |
| `stability.sideslip_weight` | `6.0` | USV_DYNAMIC中累计侧滑角平方代价。增大：偏好较小侧滑角、转弯趋于宽缓；减小：机动更灵活但横向漂移可能增加。其他硬安全检查不受影响。 |
| `smoothing.enabled` | `true` | 仅CAR_LIKE/USV_KINEMATIC执行几何平滑和安全复查。USV_DYNAMIC输出船模积分坐标，其平滑由低转向采样和曲率软代价实现，不能靠移动坐标破坏动力学一致性。 |
| `smoothing.resample_spacing` | `1.0` | 后处理弧长采样间距(m)，代码至少0.25m。增大：点少/计算少但曲线细节粗；减小：更细、每轮安全检查更贵。 |
| `smoothing.iterations` | `120` | 平滑最大轮数。增大：可能更平顺、耗时更长；减小：后处理轻但原始曲折残留；安全门槛不变。 |
| `smoothing.smooth_weight` | `0.26` | 相邻点拉直权重。增大：短波弯更平但大步易被安全复查拒绝；减小：保持原始短圆弧更多。 |
| `smoothing.data_weight` | `0.015` | 回拉原安全路线的权重。增大：偏移小、更保留原弯；减小：更能拉直，仍受最大偏移和安全检查限制。 |
| `smoothing.long_range_points` | `6` | 长尺度每侧跨越的采样点数。增大：抑制更长波S弯、也可能拉直真实河弯；减小：只处理短波；尺度随采样间距变化。 |
| `smoothing.long_range_weight` | `0.12` | 长尺度弦中点拉直权重。增大：更压低宽大S弯、步长可能被安全拒绝；减小：更保留长弯。 |
| `smoothing.max_deviation` | `6.0` | 平滑点距重采样原路径最大偏移(m)。增大：更有空间拉直但可能贴危险边界；减小：保持原路线更多；0关闭此软限幅，安全检查仍在。 |
| `channel.enabled` | `true` | true：启用分源航道语义融合；false：退回原地图单层判定/评分。开阔水域演示明确关闭，河道模式保持开启。 |
| `channel.lane_costmap_topic` | `/channel/lane_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `channel.static_land_topic` | `/chart_static_obstacles` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `channel.obstacle_costmap_topic` | `/channel/obstacle_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `channel.shoreline_cost_threshold` | `88` | 航道外岸边膨胀硬阈值。增大：可更贴岸；减小：更保守易无解；航道内仍优先陆地核心及独立障碍检查。 |
| `channel.obstacle_cost_threshold` | `85` | 独立/channel/obstacle_costmap禁止阈值。增大：允许更高障碍膨胀；减小：更保守更早绕开；航道内也生效。 |
| `channel.main_lane_value_max` | `20` | 主航道语义上界，与管理器编码一致；修改不是调整船距，失配会把边沿误分类。 |
| `channel.opposite_lane_value_min` | `65` | 对向语义下界，与75编码配套；增大可能漏判左航道，减小可能把航道外误判为航道内；非物理硬阈值。 |
| `channel.outside_lane_weight` | `12.0` | Hybrid航道外每米软代价。增大：减少离开主航道；减小：更愿安全航道外绕行；建议低于对向权重。 |
| `channel.opposite_lane_weight` | `18.0` | Hybrid对向航道每米软代价。增大：更少借用左航道、更快回主航道；减小：可选择更短对向绕行；末段目标仍可达到。 |
| `channel.main_center_weight` | `2.0` | Hybrid主航道中心梯度权重。增大：远离边沿、更居中；减小：更可能贴边短路线；不产生硬禁入边界。 |
| `channel.broad_curve_radius` | `60.0` | 航道缓弯曲率的软尺度(m)，不是硬性最小半径。每米代价为`broad_curve_weight × (radius × 实际地面曲率)²`，实际曲率含横向侧滑。增大鼓励提前、分散转向；减小允许更集中转向。仅USV_DYNAMIC有航道地图且不是大角度掉头时生效。 |
| `channel.broad_curve_weight` | `1.0` | 上述曲率代价倍率，0关闭。增大减少曲率尖峰，过大可导致额外绕行/搜索量增加；减小更接近原行为。建议0.5~2.0；窄弯和避障仍允许使用原最小半径8m。 |
| `channel.broad_curve_center_fraction` | `0.6` | 中部起终点的宽弧候选允许的最高中心梯度占比，当前上限12/20。减小候选更居中，过小可放弃宽弧；增大允许更靠边，1允许全部主航道。仅约束优化候选，原搜索仍可为避障靠边/借道；边沿起终点不启用。 |
| `channel.outside_map_weight` | `2.0` | Hybrid航道外岸边渐变代价倍率。增大：远离岸边；减小：可更贴低代价岸区；硬阈值仍生效。 |
| `channel.obstacle_inflation_weight` | `5.0` | 独立障碍软膨胀代价倍率。增大：提前更宽避让；减小：更靠近障碍但不得越硬阈值。 |
| `channel.analytic_expansion_distance` | `18.0` | 允许近端Dubins尾连接的目标距离(m)。增大：更早连接/更快结束、可能少累计航道软代价；减小：更多正常搜索/算力增加。 |
| `cruise_speed` | `4.0` | 动态时间同步典型航速(m/s)。增大：预测到达更早；减小：更晚；不是独立控制指令，应与DWA/仿真4m/s一致。 |
| `own_ship_radius` | `0.5` | 本船规划圆形半径(m)。增大：硬净空更大、绕行更宽；减小：更贴近且碰撞裕度减小；Hybrid/DWA须一致。 |
| `safety_buffer` | `1.0` | 两船半径之外的安全冗余(m)。增大：提前/更宽绕行、可行空间减少；减小：更靠近目标、抗跟踪误差更差；两级须一致。 |
| `overtaking_safe_distance` | `3.0` | 追越中心硬净空(m)；普通硬净空为2m，调到2m以下不再降低普通净空。 |
| `dynamic_prediction_horizon` | `60.0` | 动态软代价累计时域(s)。增大：更关注远期目标轨迹、绕行可能更早；减小：远期软影响弱；超时仍做硬碰撞。 |
| `dynamic_hard_distance` | `3.0` | 动态目标中心距小于该值禁止进入；这是独立于船体半径的最低硬禁入距离。 |
| `dynamic_soft_distance_factor` | `3.5` | 软避让距离相对基础硬净空的倍率，与avoidance_radius共同形成7m软域。 |
| `avoidance_radius` | `7.0` | 有实际会遇风险时的动态软避让域及RViz青圈(m)，3m硬域外到7m逐渐增加代价。 |
| `dynamic_cost_weight` | `16.0` | 目标船软邻近代价权重，只对有效会遇累加；提高后更早完成绕行，硬安全仍独立。 |
| `target_state_timeout` | `2.0` | 目标消息有效时间(s)。增大：容忍通信抖动但预测更旧；减小：更快失联停车、也更易误停。 |
| `require_target_states` | `false` | true：配置目标从未出现也使输入失效；false：允许无虚拟船的静态河道规划；已出现目标失联仍需保护。 |
| `target_odom_topics` | `[/target_boat/odom]` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `target_ship_radii` | `[0.5]` | 与目标话题对应的规划半径数组(m)。增大：目标硬域更大；减小：裕度减少；显示长宽不会自动更新它。 |
| `risk.monitor_dcpa` | `10.0` | 对应风险层的DCPA上限(m)，还需满足当前距离或TCPA条件。增大：更多未来近遇算风险、提前行动；减小：预警更紧但可能更晚避让；保持监测>=行动>=紧急。 |
| `risk.action_dcpa` | `5.0` | 对应风险层的DCPA上限(m)，还需满足当前距离或TCPA条件。增大：更多未来近遇算风险、提前行动；减小：预警更紧但可能更晚避让；保持监测>=行动>=紧急。 |
| `risk.emergency_dcpa` | `2.5` | 对应风险层的DCPA上限(m)，还需满足当前距离或TCPA条件。增大：更多未来近遇算风险、提前行动；减小：预警更紧但可能更晚避让；保持监测>=行动>=紧急。 |
| `risk.monitor_range` | `80.0` | AIS式远程监视层，保持不变；还需满足DCPA，不是感知截止或停车圈。 |
| `risk.action_range` | `24.0` | 行动层距离，与16s TCPA为OR且还需满足DCPA。 |
| `risk.emergency_range` | `8.0` | 紧急层距离，与6s TCPA为OR且还需满足DCPA；不是2m停车线。 |
| `risk.monitor_tcpa` | `40.0` | 对应风险层TCPA上限(s)，还需满足DCPA门槛。增大：提前预警/行动；减小：更晚、更紧迫；负TCPA通常表示最近会遇已过去，仍有当前距离独立保护。 |
| `risk.action_tcpa` | `16.0` | 对应风险层TCPA上限(s)，还需满足DCPA门槛。增大：提前预警/行动；减小：更晚、更紧迫；负TCPA通常表示最近会遇已过去，仍有当前距离独立保护。 |
| `risk.emergency_tcpa` | `6.0` | 对应风险层TCPA上限(s)，还需满足DCPA门槛。增大：提前预警/行动；减小：更晚、更紧迫；负TCPA通常表示最近会遇已过去，仍有当前距离独立保护。 |
| `risk.release_tcpa` | `-2.0` | 历史会遇释放要求TCPA小于此值(s)。数值增大接近0：更早释放；更负：更晚；安全recovering动作退出还看通过几何。 |
| `risk.release_range` | `6.0` | 历史释放最小中心距离(m)。增大：更晚解锁；减小：更早解锁；不等于2m当前距离停车。 |
| `risk.release_observations` | `2` | 满足释放条件的连续观察次数。增大：更抗抖动但记录更久；减小：解除更快；恢复动作不必等记录解除。 |
| `risk.manoeuvre_grace` | `6.0` | 已观察到对方机动后允许DCPA改善的等待尺度(s)。增大：更容忍机动、接管可能更迟；减小：更快判定无效。 |
| `visualization.enabled` | `true` | true：发布风险Marker；false：关闭风险显示，风险判断/安全检查仍执行。 |
| `colregs_weight` | `8.0` | 规则转向偏好软代价。增大：更偏向规则方向；减小：规则软偏好变弱；headingAllowed等硬约束仍执行。 |
| `colregs_heading_deadband` | `0.05` | 方向软代价死区(rad)。增大：允许更多微小相反偏转不罚；减小：小偏差也受罚，可能更敏感。 |
| `colregs_risk_distance` | `2.0` | 兼容assessEncounter风险距离(m)。增大/减小影响兼容评估；当前主分层使用risk.*；DWA同名参数才执行2m当前距离STOP。 |
| `colregs_time_horizon` | `60.0` | 兼容assessEncounter外推时域(s)。增大：更多远期会遇进入评估；减小：兼容评估更近；当前主风险使用risk.*。 |
| `head_on_bearing` | `0.3490658504` | assessEncounter对遇相对方位范围(rad)。增大：更多侧向目标算对遇；减小：更严格；Policy首次锁定还含固定工程门槛，非全部由此项控制。 |
| `head_on_course_tolerance` | `0.3490658504` | assessEncounter对向航向容差(rad)。增大：更多非严格对向被算对遇；减小：更严格；Policy锁定内部另有固定门槛。 |
| `crossing_bearing_limit` | `1.963495` | 兼容交叉方位边界(rad)。增大/减小改变交叉与其他类型边界；Policy锁定内部固定112.5度，不应只调此项预期全系统变化。 |
| `overtaking_speed_margin` | `0.2` | 兼容追越速度差门槛(m/s)。增大：更难判追越；减小：更易判追越；Policy锁定内部另有固定门槛。 |

## 大航向差低速回退

新增 `large_turn.enabled: true`、`large_turn.heading_threshold: 1.57`（约90°）、
`large_turn.speed: 2.0`（m/s）、`large_turn.min_radius: 5.0`（m）。正常8m转向尺度保持不变。
常速连接失败才减速收紧转向；引导失败继续用低速模式搜索。降低回退速度能减少角速度限制
对应的半径，但侧滑比例可能增大；缩小回退半径不能突破船模0.45rad/s的艏摇率能力。
具体函数联系与操作见 [大航向差修复说明](LARGE_HEADING_TURN_FIX.md)。

`speed_limit_topic` 两端均为 `/hybrid_a_star/speed_limit`，TwistStamped的linear.x是参考速度上限，
header与Path配对。MPC实际速度需按惯性逐渐减小，不能瞬时变成参考值。

## 跨包一致性检查（原参数）

近距追加 `close_turn.range=15m`、`heading_threshold=0.7854rad`、`speed=1.2m/s`、
`min_radius=3m`；正常8m及普通掉头5m保留。较小艏向差仅在近距才启用精细连接。
长度检查也计入转向尺度，绕圈检查同时要求位置和艏向已满足，避免误删必要转向。
函数联系、参数增减和测试操作见 [近距姿态修复](CLOSE_GOAL_TURN_FIX.md)。

- 当前本船最高速度4m/s；小型无人船等效规划半径均为0.5m、安全冗余1m，因此普通中心硬净空为2m。Marker长1m宽1.5m只是显示外形。
- Hybrid在7m软域提前规划、3m动态硬禁入由Hybrid负责；tracking_only=true的DWA只跟踪Path，不使用目标船距离停车。
- RViz默认仅显示黄色/橙色/红色距离预警圈、青色软代价圈和洋红色硬禁入圈；DCPA独立圈及旧STOP圈已隐藏，DCPA/TCPA仍显示在文字和预测线上。
- AIS式远程监视保持80m/40s不变；行动层24m/16s、紧急层8m/6s。风险圈不是感知截止范围，也不等于软/硬避让域。
- 地图尺寸、原点、分辨率与frame应一致；地图膨胀和独立障碍层不得混淆。
- 主航道、外部、对向代价是偏好，安全硬过滤优先；降低偏好不解除危险栅格阻挡。
# 最新自适应C弯参数

使用 `turning.prefer_c_curve=true` 和 `speed_sampling.samples=10`；旧的独立转向速度列表已被统一等间隔速度采样替代。
这是规划参考速度，不瞬间改变实测初始速度；所有C候选重新执行船模和原安全/软代价。
参考速度降低后转向尺度在正常8m、普通5m、近距3m之间插值，安全优先，多弧回退保留。
详细调参、函数和测试见 [任务主航道与C弯更新](TASK_LANE_C_TURN_UPDATE.md)。


## 最大速度优先与低速恢复（2026-10-10）

最新特殊机动参数见 [终点容差与特殊转向](MANEUVER_TOLERANCE_UPDATE.md)：
`goal_heading_tolerance=0.5235987756`（30°）、`maneuver.max_initial_straight=6m`、
`maneuver.max_detour_ratio=1.7`、`maneuver.compact_guidance_distance=30m`。
这些是末端方向/机动质量参数，不改变避碰距离；正常航行仍优先主航道。

| 参数 | 默认配置 | 作用 |
|---|---|---|
| `speed_sampling.samples` | 10 | 含最大速度和 `close_turn.speed` 的10档等间隔采样。4至1.2m/s时每档约0.311m/s；设1时只搜索最大速度。 |
| `speed_sampling.stage_time_limit` | 0.12s | 非最低速阶段的搜索上限。减小：更快进入低速但可能错过高速格点解；增大：高速搜索更充分、延迟更大。二维引导图构建也受当前阶段上限限制；单次搜索最多构建一次，未完成时清空半成品并用几何引导继续降速。完整缓存仍复用。最低速保留剩余预算。 |
| `speed_sampling.recovery_interval` | 8.0s | 低速路径上转艏率/侧滑已稳定时，按此间隔从最大速度尝试新轨迹；持续低速达到两倍间隔也触发。 |
| `speed_sampling.settled_yaw_rate` | 0.04rad/s | 提速恢复的转向稳定门槛，另要求实测横向速度小于0.15m/s。 |

普通航行且起终点都在主航道时，先分配65%搜索预算尝试留在主航道；失败后剩余35%允许带软代价的安全借道。大角度转向直接允许安全借道。所有速度档保留实测u/v/r及硬障碍、动态交通和COLREG检查。首次可行速度优先，受实时预算限制，不代表数学上证明的全局最大可行速度。

节点的一次后台工作（含规则回退）共享 `search_time_limit`；正常最大速度连接成功后不再遍历低速档。提速结果必须更快并通过实时安全/接续复验，否则继续发布旧安全路径。终点15m内不额外触发巡航提速，以免干扰终点减速。


### 对向航道短机动与朝目标分道掉头

`close_turn.range`与`maneuver.compact_guidance_distance`取较大值作为精细姿态连接范围，默认30m，覆盖10m/20m左、右、后目标艏向。普通缓弯的宽曲率偏好不施加到这些特殊机动；实际初始u/v/r、船模积分、最大艏摇率与逐步碰撞检查仍保留。

| 参数 | 默认值 | 调节效果 |
|---|---|---|
| `channel.turn_toward_goal_weight` | 30 | 后方另一分道目标的起转侧偏好。初始向远离目标分道的横移增加代价，反侧候选增加30m等效长度代价；首侧无解时仍保留安全反侧候选，0关闭。 |
| `channel.compact_opposite_weight_scale` | 0.1 | 30m内起终点均在对向语义区时，对向软权重倍率。避免短姿态调整被18倍对向代价吸向区外；1恢复原罚，0移除该短机动软罚。远程对向航行维持原权重。 |

`Speed stages [...]`日志列出实际进入的速度阶段；高速失败后顺序尝试到最低1.2m/s，有高速合格解则提前完成。总任务截止时间、取消令牌和静态/动态/COLREG硬约束保持有效。测试说明见`OPPOSITE_LANE_MANEUVER_FIX.md`。
