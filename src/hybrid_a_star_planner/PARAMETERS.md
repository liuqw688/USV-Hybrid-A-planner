# 船模轨迹规划与动态避碰：当前参数

以下默认值直接对应当前YAML；调节目的与单位见YAML中的相邻注释。修改后重启节点。

## hybrid_a_star_params.yaml

| 参数 | 当前值 |
|---|---|
| `planning_status_topic` | `/hybrid_a_star/planning_status` |
| `planner_frequency` | `5.0` |
| `event_driven.enabled` | `True` |
| `event_driven.collision_trigger_distance` | `2.0` |
| `event_driven.cross_track_limit` | `8.0` |
| `event_driven.target_course_change` | `0.0872664626` |
| `event_driven.target_speed_change` | `0.3` |
| `event_driven.search_attempts` | `2` |
| `event_driven.max_path_length_ratio` | `3.0` |
| `event_driven.max_path_extra_distance` | `40.0` |
| `event_driven.goal_loop_radius` | `3.0` |
| `event_driven.goal_loop_min_arc` | `8.0` |
| `own_odom_timeout` | `5.0` |
| `trajectory_topic` | `/hybrid_a_star_trajectory` |
| `goal_topic` | `/goal_pose_from_astar` |
| `costmap_topic` | `/local_costmap` |
| `global_frame` | `odom` |
| `base_frame` | `base_link` |
| `motion_model` | `USV_DYNAMIC` |
| `usv.max_yaw_rate` | `0.45` |
| `usv.gentle_turn_sampling` | `True` |
| `usv.model.k_u` | `0.290681` |
| `usv.model.k_t` | `0.295742` |
| `usv.model.k_turn` | `2.126739` |
| `usv.model.k_v` | `0.425383` |
| `usv.model.k_vr` | `-1.149012` |
| `usv.model.k_vn` | `0.0` |
| `usv.model.k_r` | `1.342561` |
| `usv.model.k_r3` | `0.0` |
| `usv.model.k_n` | `0.781688` |
| `usv.surge_response_time` | `0.8` |
| `usv.yaw_response_time` | `0.8` |
| `usv.max_tau_y` | `10.0` |
| `usv.max_tau_n` | `5.0` |
| `usv.integration_step` | `0.05` |
| `step_size` | `2.0` |
| `min_turning_radius` | `8.0` |
| `large_turn.enabled` | `True` |
| `large_turn.heading_threshold` | `1.57` |
| `large_turn.speed` | `2.0` |
| `large_turn.min_radius` | `5.0` |
| `turning.prefer_c_curve` | `True` |
| `speed_sampling.samples` | `10` |
| `speed_sampling.stage_time_limit` | `0.12` |
| `goal_heading_tolerance` | `0.5235987756` |
| `maneuver.max_initial_straight` | `6.0` |
| `maneuver.max_detour_ratio` | `1.7` |
| `maneuver.compact_guidance_distance` | `30.0` |
| `speed_sampling.recovery_interval` | `8.0` |
| `speed_sampling.settled_yaw_rate` | `0.04` |
| `close_turn.range` | `15.0` |
| `close_turn.heading_threshold` | `0.7853981634` |
| `close_turn.speed` | `1.2` |
| `close_turn.min_radius` | `3.0` |
| `speed_limit_topic` | `/hybrid_a_star/speed_limit` |
| `wheelbase` | `5.0` |
| `max_steering_angle` | `0.5` |
| `d_theta` | `0.1` |
| `max_iterations` | `250000` |
| `search_time_limit` | `3.0` |
| `fast_guided_connection` | `True` |
| `result_join_distance` | `1.5` |
| `result_join_heading` | `0.35` |
| `lethal_cost_threshold` | `88` |
| `stability.reference_path_weight` | `0.5` |
| `stability.near_field_reference_weight` | `12.0` |
| `stability.near_field_reference_horizon` | `45.0` |
| `stability.near_field_deviation_scale` | `3.0` |
| `stability.recovery_weight` | `4.0` |
| `stability.recovery_horizon` | `6.0` |
| `prediction.goal_braking` | `True` |
| `prediction.goal_deceleration` | `0.5` |
| `prediction.goal_tolerance` | `0.4` |
| `prediction.minimum_speed` | `0.3` |
| `stability.reference_path_max_distance` | `12.0` |
| `stability.steering_change_weight` | `1.5` |
| `stability.steering_magnitude_weight` | `12.0` |
| `stability.turnaround_heading_threshold` | `2.0943951024` |
| `stability.turnaround_steering_scale` | `0.25` |
| `stability.turn_reversal_weight` | `24.0` |
| `stability.sideslip_weight` | `6.0` |
| `smoothing.enabled` | `True` |
| `smoothing.resample_spacing` | `1.0` |
| `smoothing.iterations` | `120` |
| `smoothing.smooth_weight` | `0.26` |
| `smoothing.data_weight` | `0.015` |
| `smoothing.long_range_points` | `6` |
| `smoothing.long_range_weight` | `0.12` |
| `smoothing.max_deviation` | `6.0` |
| `channel.enabled` | `True` |
| `channel.lane_costmap_topic` | `/channel/lane_costmap` |
| `channel.static_land_topic` | `/chart_static_obstacles` |
| `channel.obstacle_costmap_topic` | `/channel/obstacle_costmap` |
| `channel.shoreline_cost_threshold` | `88` |
| `channel.obstacle_cost_threshold` | `85` |
| `channel.main_lane_value_max` | `20` |
| `channel.opposite_lane_value_min` | `65` |
| `channel.outside_lane_weight` | `12.0` |
| `channel.opposite_lane_weight` | `18.0` |
| `channel.turn_toward_goal_weight` | `30.0` |
| `channel.compact_opposite_weight_scale` | `0.1` |
| `channel.main_center_weight` | `4.0` |
| `channel.broad_curve_radius` | `60.0` |
| `channel.broad_curve_weight` | `1.0` |
| `channel.broad_curve_center_fraction` | `0.6` |
| `channel.outside_map_weight` | `2.0` |
| `channel.obstacle_inflation_weight` | `5.0` |
| `channel.analytic_expansion_distance` | `18.0` |
| `cruise_speed` | `4.0` |
| `own_ship_radius` | `0.5` |
| `safety_buffer` | `1.0` |
| `overtaking_safe_distance` | `3.0` |
| `dynamic_prediction_horizon` | `60.0` |
| `dynamic_soft_distance_factor` | `3.5` |
| `avoidance_radius` | `7.0` |
| `dynamic_hard_distance` | `4.0` |
| `dynamic_cost_weight` | `16.0` |
| `target_state_timeout` | `2.0` |
| `require_target_states` | `False` |
| `target_odom_topics` | `['/target_boat/odom']` |
| `target_ship_radii` | `[0.5]` |
| `risk.monitor_dcpa` | `10.0` |
| `risk.action_dcpa` | `5.0` |
| `risk.emergency_dcpa` | `2.5` |
| `risk.monitor_range` | `80.0` |
| `risk.action_range` | `24.0` |
| `risk.emergency_range` | `8.0` |
| `risk.monitor_tcpa` | `40.0` |
| `risk.action_tcpa` | `16.0` |
| `risk.emergency_tcpa` | `6.0` |
| `risk.release_tcpa` | `-2.0` |
| `risk.release_range` | `6.0` |
| `risk.release_observations` | `2` |
| `risk.manoeuvre_grace` | `6.0` |
| `visualization.enabled` | `True` |
| `colregs_weight` | `8.0` |
| `colregs_heading_deadband` | `0.05` |
| `colregs_risk_distance` | `2.0` |
| `colregs_time_horizon` | `60.0` |
| `head_on_bearing` | `0.3490658504` |
| `head_on_course_tolerance` | `0.3490658504` |
| `crossing_bearing_limit` | `1.963495` |
| `overtaking_speed_margin` | `0.2` |

终点艏向容差统一默认30°（`goal_heading_tolerance = 0.5235987756rad`）：ROS节点、规划器默认值、USV_DYNAMIC/USV_KINEMATIC/CAR_LIKE的终点验收均使用同一参数。该设置仅作用于终点方向；当前位置与目标位置、碰撞约束、当前A*航点及其参考艏向不变。此次未启用后续航点方向引导。

| 几何参考参数 | 默认值 |
|---|---|
| `geometric_reference.enabled` | `true` |
| `geometric_reference.preferred_radius` | `8.0` m |
| `geometric_reference.max_deviation` | `12.0` m |
| `geometric_reference.time_limit` | `0.15` s |

几何参考独立于旧`smoothing`，仅用于USV_DYNAMIC搜索后的参考曲线优化；允许暂时偏离完整动力学。候选仍须通过曲率不增、原终点位置和30°方向容差、地图/交通/COLREG与实时衔接检查。无可接受改进时保留原积分轨迹；关闭enabled可恢复仅船模连接输出。8m是软期望而非硬半径下限。


## 2026-10-14 主航道入道软偏好

| 参数 | 默认 | 作用 |
|---|---:|---|
| channel.main_entry_weight | 30.0 | 进入任务MAIN的有限软权重；0关闭本次新增偏好以便对照 |
| channel.direct_crossing_distance | 30.0m | 目标在OPPOSITE且短于该距离时保留直接跨道；目标本身在MAIN仍优先入道 |
| channel.main_entry_horizon | 30.0m | 衡量起段偏离MAIN、进入时机的弧长范围 |

原`channel.turn_toward_goal_weight`仍用于后方另一分道掉头。新偏好还覆盖前/侧方以及较小航向差，并按语义MAIN的局部方向决定左右；它不更改任务航道角色。
连续连接在同一速度阶段内限时试入道候选，失败时保留原搜索；软候选比较与原搜索回退共用3s总预算。后优化同时比较曲率、长度和入道代价，而不只比较半径。新的二维代价缓存包含入道开关及起始船位；改变入道配置也会失效缓存。
较早入道与更大转弯半径可能冲突：部分案例需要一个S或更紧的弯。每次几何后优化仍不增加对应积分轨迹的最大曲率，但不同版本最终轨迹的峰值不保证都下降。
矩阵运行说明见本包AGENTS.md，报告及所有图片在工作空间artifacts/lane_entry_1014v2。


## 近距离动态射击补救（2026-10-15）

| 参数 | 默认 |
|---|---|
| `shooting_rescue.enabled` | `true` |
| `shooting_rescue.time_budget` | `1.2 s`，包含在总预算内 |
| `shooting_rescue.minimum_speed` | `0.6 m/s` |
| `shooting_rescue.minimum_radius` | `1.5 m`，低速候选仍受艏摇与船模限幅 |

原十档4→1.2m/s不变，最后一档先试原快速单弧，再补充动态射击；最终仍对原始目标按原位置/30度艏向验收。补救输出保持实际积分，不改为纯几何后优化。
