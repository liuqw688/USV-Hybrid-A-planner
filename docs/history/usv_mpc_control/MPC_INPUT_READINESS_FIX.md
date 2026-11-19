# MPC等待路径/地图/船位问题修复

原日志 `waiting for a same-frame path, map, and odometry` 使用同一个判断汇总多种情况，不能据此断定是搜索慢还是坐标系不一致。本次检查时没有运行中的仿真节点，所以没有直接复现用户那一次日志；下列代码缺陷已通过隔离测试修复。

## 功能变化

MPC过去仅允许所有消息的世界坐标系名称完全一致，收到map地图、odom船位时会一直制动。现在通过TF将地图原点、路径点、全局航点、活动目标和里程计位姿转换到 `control_frame`。栅格数值、消息时间戳及船体坐标速度u/v/r保持不变。静态地图转换有缓存，不在10Hz控制周期重复复制整张地图。支持平移和绕Z轴旋转；拒绝倾斜TF和混合坐标系Path。

无TF时保留原始消息并安全制动，TF到达后自动转换并恢复，不要求重新发布静态地图。未改变真实船位，也没有用速度/角速度推算来补偿搜索时间。

Hybrid发布 `/hybrid_a_star/planning_status`（String，可靠、Transient Local）：`idle`未接任务，`planning`正在求解，`tracking`已有路径，`blocked`当前无安全路径，`cancelled`任务已撤销。MPC对近期明确的正常规划等待使用INFO说明，对缺地图、缺船位、缺TF或blocked继续使用WARN并打印具体原因。空路径始终制动，绝不是通过取消安全检查来消除警告。

已有安全轨迹时后台重规划继续刷新路径；新任务首次规划没有可用路径时，MPC等待是正常且必要的。无安全解也必须等待，不能强行驱动。状态话题只辅助诊断，不授权运动。

## 使用与排查

重启一键启动进程以加载新编译节点；终端执行 `source /home/l/work_ws/install/setup.bash`。保持MPC的 `control_frame: odom`；海图允许使用map，但必须有真实正确的map到odom TF，不能任意伪造变换。

仍停车时，看新日志：`missing costmap/lane map/land map/odometry` 表示对应发布节点或话题没有输入；`TF unavailable/invalid` 会指出来源坐标系和目标坐标系；`planner withdrew path (blocked)` 表示规划器没有安全轨迹，应查看Hybrid搜索/碰撞日志。`MPC waiting safely` 是首次求解或取消任务时的安全等待，不意味着控制器故障。

可查看 `ros2 topic echo /hybrid_a_star/planning_status` 与 `ros2 run tf2_ros tf2_echo odom map`。两节点的 `planning_status_topic` 要一致。安全超时参数未放宽。

## 回归测试

测试在隔离ROS域运行，不连接真实无人船：

- C++输入坐标变换：平移旋转、地图数据保持、船体系速度和时间戳保持、混合路径/倾斜TF拒绝。
- `test_mpc_terminal_ros.py --map-frame`：map地图带5m平移和0.3rad旋转、odom路径与船位，六状态船模闭环28秒；终点距离0.272m，横向误差0.174m，终点转矩符号切换3次，单次转矩变化上限0.2。
- `test_mpc_readiness_ros.py`：TF缺失制动、TF迟到恢复、首次规划空路径制动、有效路径恢复、blocked空路径仍制动，并检查具体日志。
- `test_path_freshness_ros.py`：后台末段搜索期间安全旧路径持续刷新，以及任务撤销清空路径和发布cancelled状态。

这些是节点级与合成船模回归，不等同于已在用户的实际河图/RViz现场复现全部工况。
