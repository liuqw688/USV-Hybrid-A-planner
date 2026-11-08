# 全局航点规划：当前参数

以下默认值直接对应当前YAML；调节目的与单位见YAML中的相邻注释。修改后重启节点。

## channel_astar.yaml

| 参数 | 当前值 |
|---|---|
| `lane_costmap_topic` | `/channel/lane_costmap` |
| `odom_topic` | `/odom` |
| `goal_topic` | `/goal_pose` |
| `output_topic` | `/astar_channel_waypoints` |
| `active_goal_topic` | `/goal_pose_from_astar` |
| `waypoint_spacing` | `100.0` |
| `terminal_min_waypoint_spacing` | `20.0` |
| `goal_switch_distance` | `8.0` |
| `mandatory_goal_distance` | `1.5` |
| `right_channel_only` | `True` |
| `main_lane_value_max` | `20` |
| `opposite_lane_value` | `75` |
| `outside_lane_value` | `45` |
| `outside_lane_penalty` | `1.0` |
| `safety_costmap_topic` | `/local_costmap` |
| `safety_cost_threshold` | `88` |
| `terminal_crossing_distance` | `100.0` |
| `opposite_lane_penalty` | `4.0` |
| `center_cost_weight` | `0.12` |
| `max_search_iterations` | `200000` |
| `map_settle_time` | `0.5` |
| `minimum_replan_period` | `0.5` |
| `replan_on_lane_role_change` | `True` |

`compact_direct_distance: 30.0`：超过30m且起终点不在主航道时，分解为起点接入、主航道主体、终点接入。主体限定 MAIN；接入段保留既有100m连接限制与航道外安全检查。该阈值不改变通航方向。

`obstacle_cost_weight: 0.02`：叠加局部物理膨胀软代价，使独立障碍附近保留绕行偏好。安全图所有区域的100/99/未知均拒绝，包括起终点，不允许航道语义覆盖硬安全。
