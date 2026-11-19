# 对向分道掉头、10/20m短机动与低速预算修正

日期：2026-10-10。修改航道任务分类与Hybrid A*；参数和测试同步更新。

## 行为

- 当前向前航行，目标在后方另一条分道且末端艏向反向：按船体坐标中的目标横向位置确定优先起转侧。目标在左后方优先左转进入目标分道，右后方镜像右转。±π附近的目标艏向不再仅靠角度归一化决定起转侧；允许沿目标侧转过180°，端点仍按原目标艏向验收。
- 起终点在同一个明确物理航段、相距不超过30m时，该航段作为任务主航道种子。左/右/后朝向的短姿态调整不会仅因目标方位在后方而交换两条分道。跨分道与更远任务保持原任务方向分类；角色仍锁存到下一次新目标。
- 10m/20m姿态机动统一使用精细连接。对向语义区的直航连接也启用；短机动暂时降低对向区域软罚，避免被软代价吸向航道外。特殊转向不施加普通航道缓弯的宽曲率偏好。
- 按原10档从4m/s降到1.2m/s。二维引导图构建受当前速度阶段上限约束，单次search最多构建一次；未完成时丢弃半成品，以几何启发继续后续速度。完整启发缓存仍复用，任务软代价状态变化时失效。`Speed stages [...]`列出实际尝试过的速度。

掉头侧是软偏好：对远离目标分道的初始横移加代价，安全的另一侧候选会保存，并按路径长度加有限反侧代价比较；首侧无解或代价过高时可使用另一侧。任何候选仍执行船模积分、原位置/艏向验收、岸线/障碍/动态船/COLREG硬检查。所有速度尝试都从实测u/v/r启动，低速参考不意味着将高速实船瞬间变为低速。

## 新参数

| 配置 | 参数 | 默认值 | 含义 |
|---|---|---:|---|
| channel_navigation.yaml | compact_mission_range | 30m | 同一明确物理航段内的短任务种子锁定范围，0关闭 |
| hybrid_a_star_params.yaml | channel.turn_toward_goal_weight | 30 | 起转侧软权重及反侧候选等效长度代价，0关闭 |
| hybrid_a_star_params.yaml | channel.compact_opposite_weight_scale | 0.1 | 起终点均为对向语义区的短任务对向软罚倍率，1恢复原罚 |

精细连接范围取`max(close_turn.range, maneuver.compact_guidance_distance)`，默认30m；不需要增大全程普通转弯能力或修改控制器。

## 验证

- 单元测试78项全部通过：Hybrid 64项、COLREG 5项、航道几何9项。
- 新规划回归涵盖主/对向语义、10/20m前方目标的左右/向后艏向、0/4m/s初始速度、28m窄水域、镜像后方分道掉头及±0.35rad末端艏向偏差。每条路径检查终点位置/艏向、地图安全和起点连续性。
- 新增缓存回归通过：搜索worker的短机动引导缓存能由下一worker复用；同一目标由20m短任务变为40m较远任务时，软代价上下文变化会重新构建。
- 180万格地图加贯穿硬墙的1秒预算测试：没有安全路径，实际进入全部10档，包括1.2m/s；不能由第一档整图扫描把低速尝试耗掉。
- 隔离域187运行真实航道管理与Hybrid节点：12个同分道短机动、3个左后方跨分道目标均通过，目标分道正确成为任务主航道，跨道案例初次转向均为左转。Path和速度消息按时间戳配对，且路径必须新于当前局部目标；不接受上一任务的缓存结果。硬墙任务仍输出blocked/空Path。
- 原航道锁存ROS回归通过（旋转TF、迟到TF、跨道/转向/越过目标、地图重建/海图重排）；原C弯ROS回归通过（10个机动及硬墙）。
- 真实XML河图、全局A*、Hybrid、原MPC与原船模的20m前方右向目标通过规划链路：参考速度1.822m/s，路径长22.33m，起始直段1.23m，末端艏向误差0.00201rad；收到新路径约0.29s，随后运行8秒闭环仿真。此项验证路径交付与短时跟踪，不代表完成整个终点停车测试。

初步修改前的开阔测试12个短机动本身也可成功，但部分对向场景多次耗尽高速阶段；不能把这些基线测试称作“修改前全部失败”。修正依据还包括入口不对称、20m场景未进入原15m精细范围，以及整图构建覆盖速度阶段截止时间的代码检查。

本次新增场景为合成水域；实际河图只验证了上述20m右向案例。低速与偏好不能保证所有水域目标都可达；无安全路径仍必须失败。

## 复测与产物

```bash
source /home/l/work_ws/install/setup.bash
ctest --test-dir /home/l/work_ws/build/hybrid_a_star_planner --output-on-failure
ctest --test-dir /home/l/work_ws/build/channel_navigation_manager --output-on-failure
python3 /home/l/work_ws/src/hybrid_a_star_planner/test/test_opposite_lane_ros.py
python3 /home/l/work_ws/src/channel_navigation_manager/test/test_mission_lock_ros.py
python3 /home/l/work_ws/src/hybrid_a_star_planner/test/test_adaptive_c_turn_ros.py
RIVER_SPECIAL_GOAL=right RIVER_HANDOFF_OUTPUT=/home/l/work_ws/artifacts/opposite_lane_update/actual_river \
  python3 /home/l/work_ws/src/hybrid_a_star_planner/test/test_river_handoff_ros.py
```

永久结果位于`/home/l/work_ws/artifacts/opposite_lane_update/`：场景指标JSON、单元测试XML、测试日志；`actual_river/`保存实河图轨迹、指标、节点日志及绘图。运行中的旧节点需要重启才能加载新二进制和参数。
