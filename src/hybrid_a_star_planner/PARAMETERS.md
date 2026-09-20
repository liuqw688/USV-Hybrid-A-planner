# 参数作用及调节说明

本说明对应当前源码与配置，不承诺每个参数可在线更新。多数参数仅在构造时读取：编辑YAML后请停止旧实例并重新启动。launch内追加参数可能覆盖YAML；可用 `ros2 param dump /节点名` 核对运行值。

数值“增大/减小”指其他条件不变时的趋势，不是无限调大都有效。布尔值按开/关解释；话题和frame必须与生产者一致。不要同时降低安全半径、冗余及急停距离来追求直线效率。


## config/hybrid_a_star_params.yaml

当前配置值不是节点内置默认值，其他launch也可能覆盖它。

| 参数 | 当前YAML值 | 作用、关联及调大/调小影响 |
|---|---|---|
| `planner_frequency` | `1.0` | 事件安全监测与缓存路径刷新频率(Hz)，不是固定搜索频率。增大可更快发现新风险；只有触发条件成立才运行Hybrid A*。 |
| `event_driven.enabled` | `true` | true：新目标、预测碰撞/规则冲突、静态阻断或明显偏航时才搜索；false：每个timer周期搜索。 |
| `event_driven.collision_trigger_distance` | `2.0` | 同一会遇已成功规划后，剩余路径重新进入2m预测硬域才再次搜索；首次会遇由DCPA/TCPA提前触发。 |
| `event_driven.cross_track_limit` | `8.0` | 本船偏离缓存路径超过此值时重规划。增大更愿继续旧路；减小则偏航后更快改线。 |
| `event_driven.target_course_change` | `0.0873` | 目标船相对上次成功规划航向变化超过5度时视为新机动并重新规划。 |
| `event_driven.target_speed_change` | `0.3` | 目标船相对上次成功规划速度变化超过0.3m/s时重新规划。 |
| `event_driven.search_attempts` | `2` | 每次事件规划生成的候选次数(1~3)，选择不绕圈且较短的安全方案；增大算力和延迟。 |
| `event_driven.max_path_length_ratio` / `max_path_extra_distance` | `3.0` / `40.0` | 路线长度质量门槛，超过则不发布路径，等待下一次规划，避免终点附近大绕圈。 |
| `event_driven.goal_loop_radius` / `goal_loop_min_arc` | `3.0` / `8.0` | 终点附近回到目标半径内后仍绕行超过该弧长即判为绕圈并丢弃。 |
| `path_failure_hold_time` | `3.0` | 最近路径有效性标志保留时间(s)。增大：更晚标为无效；减小：更早失效；当前不直接清除RViz路径，停车由DWA路径过期负责。 |
| `own_odom_timeout` | `5.0` | Hybrid船位消息时间容忍(s)。增大：容忍搜索阻塞但允许较旧船位；减小：更快失联保护、可能误判计算延迟。 |
| `trajectory_topic` | `/hybrid_a_star_trajectory` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `goal_topic` | `/goal_pose_from_astar` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `costmap_topic` | `/local_costmap` | ROS消息接口名称（列表按目标对应）。修改需同时匹配发布方/订阅方及RViz；名称大小没有调参含义，失配会收不到数据并等待/停车。 |
| `global_frame` | `odom` | 坐标系/TF frame名称。修改须与消息头、TF及接收节点一致；不是数值大小参数，缺失或失配会拒绝输入。 |
| `base_frame` | `base_link` | 坐标系/TF frame名称。修改须与消息头、TF及接收节点一致；不是数值大小参数，缺失或失配会拒绝输入。 |
| `step_size` | `2.0` | Hybrid每次运动基元长度(m)。增大：扩展少、细节粗且更难窄处绕行；减小：搜索细但状态/检查更多。 |
| `min_turning_radius` | `9.0` | Hybrid启发估计用的最小转弯半径(m)。增大：估计更不愿急转；减小：启发更灵活；实际基元/Dubins由wheelbase与舵角决定，须保持一致。 |
| `wheelbase` | `5.0` | 简化转向模型轴距(m)。增大：相同舵角转弯半径更大；减小：可更急转；需反映船舶可实现转弯能力。 |
| `max_steering_angle` | `0.5` | 基元最大转角(rad)，R约wheelbase/tan(angle)。增大：转弯更紧、急弯更多；减小：更平缓但窄处无解；不是Twist角速度。 |
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
| `stability.steering_magnitude_weight` | `1.0` | 大转角平方惩罚。增大：减少大S弯、转向更温和；减小：急转更容易；硬避障仍优先。 |
| `smoothing.enabled` | `true` | true：搜索后重采样/平滑并安全复查；false：输出原搜索轨迹，安全检查仍在搜索中执行。 |
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
| `channel.outside_lane_weight` | `2.5` | Hybrid航道外每米软代价。增大：减少离开主航道；减小：更愿安全航道外绕行；建议低于对向权重。 |
| `channel.opposite_lane_weight` | `4.0` | Hybrid对向航道每米软代价。增大：更少借用左航道、更快回主航道；减小：可选择更短对向绕行；末段目标仍可达到。 |
| `channel.main_center_weight` | `2.0` | Hybrid主航道中心梯度权重。增大：远离边沿、更居中；减小：更可能贴边短路线；不产生硬禁入边界。 |
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

## 跨包一致性检查

- 当前本船最高速度4m/s；小型无人船等效规划半径均为0.5m、安全冗余1m，因此普通中心硬净空为2m。Marker长1m宽1.5m只是显示外形。
- Hybrid在7m软域提前规划、3m动态硬禁入由Hybrid负责；tracking_only=true的DWA只跟踪Path，不使用目标船距离停车。
- RViz默认仅显示黄色/橙色/红色距离预警圈、青色软代价圈和洋红色硬禁入圈；DCPA独立圈及旧STOP圈已隐藏，DCPA/TCPA仍显示在文字和预测线上。
- AIS式远程监视保持80m/40s不变；行动层24m/16s、紧急层8m/6s。风险圈不是感知截止范围，也不等于软/硬避让域。
- 地图尺寸、原点、分辨率与frame应一致；地图膨胀和独立障碍层不得混淆。
- 主航道、外部、对向代价是偏好，安全硬过滤优先；降低偏好不解除危险栅格阻挡。
