# MPC跟踪与无人船仿真：当前参数

以下默认值直接对应当前YAML；调节目的与单位见YAML中的相邻注释。修改后重启节点。

## mpc_params.yaml

| 参数 | 当前值 |
|---|---|
| `path_topic` | `/hybrid_a_star_trajectory` |
| `costmap_topic` | `/local_costmap` |
| `lane_costmap_topic` | `/channel/lane_costmap` |
| `static_land_topic` | `/chart_static_obstacles` |
| `channel_obstacle_topic` | `/channel/obstacle_costmap` |
| `odom_topic` | `/odom` |
| `effort_topic` | `/cmd_wrench` |
| `policy_topic` | `/colregs/policies` |
| `global_route_topic` | `/astar_channel_waypoints` |
| `active_goal_topic` | `/goal_pose_from_astar` |
| `planning_status_topic` | `/hybrid_a_star/planning_status` |
| `speed_limit_topic` | `/hybrid_a_star/speed_limit` |
| `require_speed_profile` | `True` |
| `control_frame` | `odom` |
| `control_frequency` | `10.0` |
| `prediction_step` | `0.1` |
| `prediction_horizon` | `5.0` |
| `control_knots` | `5` |
| `optimization_sweeps` | `2` |
| `max_speed` | `4.0` |
| `max_yaw_rate` | `0.45` |
| `max_tau_y` | `10.0` |
| `max_tau_n` | `5.0` |
| `surge_response_time` | `0.8` |
| `yaw_response_time` | `0.8` |
| `model.k_u` | `0.290681` |
| `model.k_t` | `0.295742` |
| `model.k_turn` | `2.126739` |
| `model.k_v` | `0.425383` |
| `model.k_vr` | `-1.149012` |
| `model.k_vn` | `0.0` |
| `model.k_r` | `1.342561` |
| `model.k_r3` | `0.0` |
| `model.k_n` | `0.781688` |
| `channel_enabled` | `True` |
| `shoreline_cost_threshold` | `88` |
| `channel_obstacle_threshold` | `85` |
| `main_lane_value_max` | `20` |
| `opposite_lane_value_min` | `65` |
| `lethal_cost_threshold` | `88` |
| `own_ship_radius` | `0.5` |
| `safety_buffer` | `1.0` |
| `map_timeout` | `0.0` |
| `path_timeout` | `3.0` |
| `odom_timeout` | `1.0` |
| `dynamic_hard_distance` | `4.0` |
| `dynamic_soft_distance` | `15.0` |
| `emergency_stop_distance` | `2.0` |
| `goal_deceleration` | `0.5` |
| `goal_tolerance` | `0.5` |
| `intermediate_goal_speed` | `2.0` |
| `cross_track_recovery_distance` | `8.0` |
| `projection_forward_distance` | `6.0` |
| `minimum_tracking_speed` | `0.25` |
| `terminal_heading_fade_speed` | `0.35` |
| `max_torque_rate` | `2.0` |
| `max_lateral_acceleration` | `0.6` |
| `curvature_preview_distance` | `12.0` |
| `terminal_hold_radius` | `0.5` |
| `weight.cross_track` | `12.0` |
| `weight.course` | `12.0` |
| `weight.progress` | `1.4` |
| `weight.speed` | `2.0` |
| `weight.sideslip` | `3.0` |
| `weight.yaw_rate` | `2.0` |
| `weight.effort` | `0.01` |
| `weight.torque_effort` | `0.008` |
| `weight.torque_slew` | `0.6` |
| `weight.reverse_turn` | `2.5` |
| `weight.terminal` | `4.0` |
| `weight.dynamic_clearance` | `12.0` |

## sim_params.yaml

| 参数 | 当前值 |
|---|---|
| `start_x` | `-270.0` |
| `start_y` | `-270.0` |
| `start_yaw` | `2.14675498` |
| `initial_speed` | `4.0` |
| `max_linear_speed` | `4.0` |
| `model.k_u` | `0.290681` |
| `model.k_t` | `0.295742` |
| `model.k_turn` | `2.126739` |
| `model.k_v` | `0.425383` |
| `model.k_vr` | `-1.149012` |
| `model.k_vn` | `0.0` |
| `model.k_r` | `1.342561` |
| `model.k_r3` | `0.0` |
| `model.k_n` | `0.781688` |
| `max_tau_y` | `10.0` |
| `max_tau_n` | `5.0` |
| `max_angular_speed` | `0.45` |

MPC、仿真与Hybrid的模型系数/最高速度/艏摇率须同步。`require_speed_profile=true`要求新轨迹与限速时间戳配对；空路径、失联或不安全时制动。仿真输入仅为`/cmd_wrench`，不存在速度命令兼容模式。


## 2026-10-15 跟踪补充

`course_preview_time=2.0`：额外前馈候选的速度相关预览，范围1..8m；原反馈候选保留。`weight.terminal_heading=50.0`：末段4m独立考虑最终船艏，24度内不罚；停车保持须满足30度。低速进度软代价有上限归一化，不覆盖硬安全。

默认预测仍是5s、0.1s、5控制结点。10s/0.2s/8结点只做过独立参数试验；低速自适应默认窗口的写入未获批准，未加入源码/配置。细节见项目实验报告。
