# 任务航道管理与整链路启动：当前参数

以下默认值直接对应当前YAML；调节目的与单位见YAML中的相邻注释。修改后重启节点。

## channel_navigation.yaml

| 参数 | 当前值 |
|---|---|
| `lane_data_topic` | `/chart_lane_data` |
| `costmap_topic` | `/local_costmap` |
| `odom_topic` | `/odom` |
| `goal_topic` | `/goal_pose` |
| `lane_costmap_topic` | `/channel/lane_costmap` |
| `marker_topic` | `/channel/navigation_markers` |
| `status_topic` | `/channel/status` |
| `outside_value` | `45` |
| `opposite_value` | `75` |
| `main_lane_edge_value` | `20` |
| `center_clearance` | `60.0` |
| `minimum_course_speed` | `0.1` |
| `reclassify_angle` | `0.2094395102` |
| `lane_adjacency_distance` | `160.0` |
| `lane_continuity_angle` | `1.3962634016` |
| `reverse_enter_angle` | `1.9198621772` |
| `minimum_reverse_goal_distance` | `10.0` |
| `compact_mission_range` | `30.0`（保留兼容，不再强制当前分道） |
| `lane_topology_gap` | `5.0 m` |

实际拓扑连接与同向传播距离取 `min(lane_adjacency_distance, lane_topology_gap)`，避免160m邻接阈值串联不相邻支流。`/channel/status` 中 `neutral_corridor_lane_ids` 是确定方向的中性走廊，不是执行路线；`classification_valid=false` 表示不可确定任务分道。
