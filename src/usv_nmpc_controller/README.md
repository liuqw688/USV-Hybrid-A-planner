# usv_nmpc_controller

基于 CasADi 的无人船 NMPC 跟踪控制器。控制器直接读取里程计和工程内定义的 B 样条消息。

## 功能结构

```text
/odom (nav_msgs/Odometry) ─┐
                           ├─> usv_nmpc_node ──> /cmd (geometry_msgs/Twist)
/planning/bspline         ┘          ├─────────> /nmpc_predicted_path
  (usv_interfaces/UsvBspline)         ├─────────> /nmpc_predicted_poses
                                      └─────────> /nmpc_tracking_diagnostics
```

主要代码按职责放置：`nmpc_controller.*` 和 `nmpc_problem.cpp` 负责优化，`bspline_reference.*` 负责样条校验与按轨迹时间求值，`usv_nmpc_node.*` 负责 ROS 接口及控制循环，`tracking_state_machine.*` 统一管理跟踪生命周期。状态转移图、恢复规则与观测方法见 [STATE_MACHINE.md](STATE_MACHINE.md)。

终端状态表默认开启，每秒显示一次实测状态、当前参考、两者误差、样条段进度和实际发布的控制量。可在 `config/usv_nmpc.yaml` 中设置 `terminal_table.enabled: false` 关闭，或调整 `terminal_table.refresh_period_s`；输出节流复用控制循环，不增加定时器。

## 接口约定

| 话题 | 类型 | 约定 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | `header.frame_id=map`；ENU 平面位置，`child_frame_id=usv_body_rfu`；本体速度 x=右、y=前；RFU 四元数的 y 轴为船首。 |
| `/planning/bspline` | `usv_interfaces/msg/UsvBspline` | 控制点为位姿 `[x,y,phi]`，三者共用节点向量。`x` 东、`y` 北，`phi` 北为 0、东为 -90°、西为 +90°。消息没有坐标系字段，位置必须与 `/odom` 同属 ENU 米制坐标；`start_time` 使用 ROS 绝对时间。 |
| `/planning/replan_failed` | `std_msgs/msg/Header` | 规划器撤销当前失效参考；控制器清除参考并输出零指令，等待撤销时间之后生成的新样条。 |
| `/planning/bspline_path` | `nav_msgs/msg/Path` | 发布器将同一条 B 样条采样后发布，坐标系为 `odom`，供 RViz2 显示。 |
| `/cmd` | `geometry_msgs/msg/Twist` | `linear.x` 为前向速度指令 m/s，`angular.z` 为偏航角速度指令 rad/s；正角速度为逆时针。 |
| `/nmpc_tracking_state` | `std_msgs/msg/String` | 只读、可靠且瞬态本地的状态快照，如 `TRACKING`、`SAFE_STOP:ODOM_TIMEOUT`；仅状态或保护原因变化时发布。 |

`/odom` RFU 与控制器使用相同导航角：船首向北时 `psi=0`，向东时 `psi=-pi/2`。模型速度为前向 `u=twist.linear.y`、右向 `v=twist.linear.x`；正 `r` 表示船首向西转。B 样条接收端从位置与船首角的时间导数按同一运动学反算本体速度。

模型运动学为 `xdot=-u*sin(psi)+v*cos(psi)`、`ydot=u*cos(psi)+v*sin(psi)`、`psidot=r`。动力学系数与 `usv_model_sim` 共用，具体方程见 `src/usv_nmpc_controller/src/usv_model.cpp` 与 `src/usv_model_sim/src/dynamics.cpp`。

NMPC 用 `now - start_time + k * dt` 直接评估未来样条，不按实测位置重投影，也不重算或平滑规划器给出的速度。滚动片段必须沿用同一个全局 ROS 时间原点：`start_time` 表示该片段首个样条参数时刻对应的绝对时间，不能每次发布都重置为当前时刻。这样发布间隔变化时，控制器仍从同一条时间参数化路线采样；消息间隔中继续使用当前有效样条。若新片段的起始时刻在未来，控制器保持当前样条，等最早的待切片到达其绝对起始时间再切换，避免收到消息就停船等待或不断推迟切换。参考默认 2 s 未更新才停车，容忍 1 Hz 消息更新及一定抖动。测试中的 `--publish-jitter-hz 1` 和 `5` 表示 10 Hz 名义发布间隔上的 1 Hz、5 Hz 调制频率，不代表实际以 1 Hz 或 5 Hz 发布；1 Hz 实际更新间隔由 1.2 s ROS 回归测试覆盖。规划器仍需在当前片段结束前更新，并使新片段覆盖当前时刻与预测窗。

样条段可以在非零速度下结束，预测窗超出段尾时统一沿段尾 ENU 位置速度和艏向率短时外推；规划器负责生成动力学可行的时间参数化轨迹。控制器不要求样条末端速度为零，也不在消息中设置终点标记；只有样条末端导数近零、时间到达末端且实船到点停稳时才发布任务完成。

只有时间到达样条末端且末端导数近零时，控制器才检查任务完成。完成条件为本船距离终点不大于 `finish_distance_m`（默认 2.0 m）、合速度不高于 `finish_speed_mps`（默认 0.2 m/s），并连续满足 `finish_stable_duration_s`（默认 1 s）。确认完成后发布 `/USV_tracking_finished=true` 并持续输出零指令。控制器保留状态/轨迹超时停车、指令边界及求解失败保护；它不替规划器补造终点停车段。

控制器在本包内维护 NMPC 所需的船体预测方程和辨识系数，不依赖仿真状态源。轨迹参考从 `x/y/phi` 样条导数按给定运动学反算 `u/v/r`，保留几何轨迹要求的侧向速度；仿真状态源可由硬件驱动替换，控制器接口保持不变。NMPC 以 0.1 s 步长预测未来 10 s（100 步），并以 10 Hz 发布速度与偏航角速度指令。前向速度指令上限为 5 m/s，角速度指令范围为 ±0.3 rad/s，禁止倒车；预测状态 `u/v/r` 分别受配置硬边界限制。首次求解用模型前馈、后续用旧解移位；求解失败直接停车并清空预测。
优化器使用六状态参考 `x, y, psi, u, v, r`；横向误差投影到参考地速切线 `chi_ref = psi_ref - beta_ref`，航向代价分别补偿实际与参考侧滑。代价只包含横向、航向、`u/v/r` 跟踪和分通道指令变化量，不再增加独立侧滑上限惩罚。
速度和角速度的限幅、限变化率实现由初值与 ROS 发布共用；角速度仅配置一个对称幅值上限。
数值参数见 [PARAMETERS.md](PARAMETERS.md)。

## 编译

ROS 2 Jazzy 环境需要安装 CasADi；`usv_interfaces` 由本工程构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/sus/USV_systems
colcon build --symlink-install --packages-up-to usv_nmpc_controller
source install/setup.bash
```

此消息与 Fast-Planner 的 `quadrotor_msgs/msg/Bspline` 字段对应，但 ROS 消息类型不同，旧发布器需改用 `usv_interfaces/msg/UsvBspline`。

## 独立启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
ros2 launch usv_nmpc_controller usv_nmpc.launch.py
```

此入口只启动正式控制节点，不启动地图、仿真或参考发布器。控制配置可用 `config_file:=/绝对路径/usv_nmpc.yaml` 指定。地图、仿真、控制器和 Python 局部路径发布器的四终端启动步骤见 [工作空间说明](../../README.md)。组合测试入口和路线回归脚本在 [usv_tracking_test](../usv_tracking_test/README.md) 包。

## 状态检查

```bash
ros2 topic echo /nmpc_tracking_state --qos-durability transient_local
ros2 topic echo /nmpc_tracking_diagnostics
ros2 topic echo /cmd
```

诊断数组保留原有下标：d34/d40/d42～d47 为已退役代价槽，固定为零；56 为末端到点区域条件，57 为到达确认计时是否启用，58 为连续确认秒数；59、60 为已退役停稳字段，固定为零；61～64 为实际指令、发布间隔和时间戳；65、66 为已退役侧滑代价槽，固定为零；67 为有限状态机状态编号，68 为保护原因编号，详见 [STATE_MACHINE.md](STATE_MACHINE.md)。
