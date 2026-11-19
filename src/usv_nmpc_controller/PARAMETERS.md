# NMPC 参数说明（当前速度接口）

对应配置：`config/usv_nmpc.yaml`。以下数值取自本次配置，修改配置后需重启导航。
`/**` 表示将参数应用于匹配的 ROS 节点，`ros__parameters` 内才是节点参数。
`reference:`、`nmpc:` 是分组，例如 `nmpc.dt` 是实际 ROS 参数名。

状态为 `[x, y, psi, u, v, r]`：世界坐标 ENU，船体速度为 RFU，`u` 向前、`v` 向右，
航向北为零且向西为正，`r>0` 时航向增加。输入为前向速度指令 `speed` (m/s) 和角速度指令
`yaw_rate` (rad/s)。NMPC 与仿真器使用相同的参数，NMPC 同时构建数值模型和 CasADi 符号模型。
B 样条控制点包含 `[x,y,phi]`，控制器按其导数反算完整本体参考速度。

## 话题、频率与超时

| 参数 | 配置值 | 意义 |
| --- | --- | --- |
| odom_topic | /odom | 订阅 ENU 位姿和 RFU 本体速度的里程计。 |
| bspline_topic | /planning/bspline | 订阅 `usv_interfaces/msg/UsvBspline` 作为轨迹参考。 |
| frame_id | map | 预测轨迹和参考位姿使用的坐标系；必须与 `/odom.header.frame_id` 一致。 |
| cmd_topic | /cmd | 发布 Twist：linear.x=v_cmd，angular.z=w_cmd。 |
| tracking_finished_topic | /USV_tracking_finished | 发布任务是否已经满足完成判据。 |
| nmpc_diagnostics_topic | /nmpc_tracking_diagnostics | 发布跟踪误差、状态及优化相关诊断。 |
| predicted_path_topic | /nmpc_predicted_path | 发布优化预测的位置轨迹，供 RViz2/Web 显示。 |
| predicted_poses_topic | /nmpc_predicted_poses | 发布优化预测的位姿序列，供显示船头方向。 |
| control_frequency | 10.0 Hz | 控制定时器期望每 0.1 s 执行一次；实际频率受求解耗时影响。 |
| state_timeout_s | 0.5 s | 超过该时间未收到新状态，发布零控制指令。 |
| reference_stale_timeout | 2.0 s | 局部参考超过该时间未更新，触发停车保护；允许 1 Hz 更新并容忍一定发布抖动。 |
| terminal_table.enabled | true | 是否在终端显示实测/参考/误差及控制量表格。 |
| terminal_table.refresh_period_s | 1.0 s | 表格最短刷新间隔；只节流输出，不改变 NMPC 控制频率。 |

`/planning/bspline` 消息没有 `frame_id` 字段，因此位置控制点必须与 `/odom` 使用相同 ENU 米制坐标。
`/cmd.angular.z` 的正值表示逆时针左转；取负仅在硬件网络发送端执行。

## 到点与完成判定

| 参数 | 配置值 | 意义 |
| --- | --- | --- |
| finish_distance_m | 2.0 m | 本船到样条终点的平面距离容差；仅在到达末端样条后检查。 |
| finish_speed_mps | 0.2 m/s | 完成任务允许的本船合速度上限。 |
| finish_stable_duration_s | 1.0 s | 终点距离和合速度条件连续满足的时间；越界或输入失效会重置计时。 |

完成表示船已到达样条末端，且终点距离和合速度条件持续满足。输入失效或样条末端导数不近零时，不会完成任务。

航向、转艏率和位置都按 `start_time` 的轨迹时刻直接从 B 样条及其导数读取。
B 样条末端允许非零速度，预测窗越过段尾时统一沿末端导数外推；控制器不会因此拒收轨迹。
完成判定根据样条末端导数是否近零、时间是否到达末端及实船是否停稳推断，不增加消息标记。
规划器负责局部段之间的时间对齐、起点连续和最终制动，NMPC 不对参考速度做二次重定时。

## reference：样条时间采样

| 参数 | 配置值 | 意义与调整效果 |
| --- | --- | --- |
| bspline_sample_period_s | 0.1 s | 仅用于跟踪表和终点诊断的离散周期，不是控制周期或 NMPC 预测步长；NMPC 预测点直接按 `dt` 评估样条。 |

NMPC 的第 `k` 个参考点位于 `now - start_time + k * nmpc.dt`，按 `xdot/ydot` 与 `dphi/dt` 反算 `u/v/r`。
预测窗超过样条末端后，沿末端位置导数和航向导数外推；外推只是短时覆盖重规划间隔，
规划器需及时发布新段，并保证新段起点和时间连续。若任务要求停车，减速与停车需包含在规划器输出中。
控制器的指令变化率仍由 `nmpc.max_speed_command_accel_mps2`、`nmpc.max_speed_command_decel_mps2`
和 `nmpc.max_yaw_rate_command_accel_radps2` 限制；这些约束不改变送入 NMPC 的参考轨迹。

## nmpc：模型、时间窗口与控制边界

| 参数 | 配置值 | 意义 |
| --- | --- | --- |
| horizon_steps | 100 | 每次优化未来 100 个区间，并预测 101 个状态。 |
| dt | 0.1 s | 预测积分步长；预测时长为 100×0.1=10 s。实际控制目标为 10 Hz。 |
| max_speed_command_accel_mps2 | 0.65 m/s² | 预测和发布速度指令的加速变化率上限。按 10 Hz 理想周期，每次最多增加 0.065 m/s。 |
| max_speed_command_decel_mps2 | 2.5 m/s² | 预测和发布速度指令的制动变化率上限。按 10 Hz 理想周期，每次最多减少 0.25 m/s；保护停车直接归零。 |
| max_yaw_rate_command_accel_radps2 | 0.12 rad/s² | 预测和发布 w_cmd 的最大变化率，约 6.9°/s²。10 Hz 理想周期最多变化 0.012 rad/s（约 0.69°/s）；指令幅值仍限 ±0.3 rad/s。 |
| min_speed_command_mps | 0.0 m/s | 前进速度指令下限，禁止倒车。 |
| max_speed_command_mps | 5.0 m/s | 前进速度指令硬上限；内置 B 样条测试轨迹的巡航参考为 4.0 m/s。 |
| max_yaw_rate_command_radps | 0.3 rad/s | 转向指令的对称幅值上限；正值左转、负值右转。 |
| max_reverse_speed_mps | 0.0 m/s | 禁用倒车参考，并限制优化预测中的实际纵向速度 u 不低于 0 m/s。 |
| max_surge_state_mps | 5.0 m/s | 优化预测状态 u 的正向硬上限，与速度指令上限分开约束。 |
| max_sway_state_mps | 2.0 m/s | 优化预测状态 v 的对称硬边界，即 −2～2 m/s。 |
| max_yaw_rate_state_radps | 0.3 rad/s | 优化预测状态 r 的对称硬边界，即 −0.3～0.3 rad/s。 |

优化器第一步变化量按连续两次控制回调的完整间隔计算，后续按预测步长 dt 计算；
发布端按实际发布间隔保留限幅保护。求解前距上次发布的时间不包含上次求解耗时，
不能用它代替完整控制周期，否则预测和实际换舵能力会出现偏差。
表中每周期变化量是理想周期下的例子。
停车保护、状态失效和完成后的零指令会立即归零，不受正常变化率限制。
10 Hz 是目标调度频率，`solver_max_iterations` 并不保证求解在 100 ms 内完成。

## nmpc：跟踪代价权重

权重通常乘误差平方。增大某项会让优化器更重视它，但不同误差量纲不同，
不能只比较权重数字判断实际作用；这些权重也不是 PID 增益。

| 参数 | 配置值 | 惩罚的内容 |
| --- | --- | --- |
| weight_lateral | 0.5 | 相对参考地速切线方向的 Frenet 横向位置误差。参考切线为 `chi_ref = ref.psi - beta_ref`。 |
| weight_course | 12.0 | 分别对实际和参考船首航向进行侧滑补偿后的角度误差，不含横向回线修正。 |
| weight_u | 2.0 | 实际纵向速度 u 与参考 u 的误差。 |
| weight_v | 0.5 | 实际横向速度 v 与参考 v 的误差。 |
| weight_r | 240.0 | 实际转艏率 r 与参考 r 的误差；提高后更贴合平滑曲率参考。与约 6.9°/s² 的角速度指令变化率限制配合。 |
| weight_surge_delta | 3.0 | 速度指令 `V_cmd` 相邻预测步的变化代价；按 `max_speed_command_mps` 归一化。 |
| weight_yaw_rate_delta | 3.0 | 角速度指令 `W_cmd` 相邻预测步的变化代价；按 `max_yaw_rate_command_radps` 归一化。 |

阶段代价包含横向、侧滑补偿后的航向及 u/v/r 跟踪；
预测窗口末端状态没有独立代价，仍受到动力学和状态约束。
优化参考为 `x, y, psi, u, v, r` 六个状态。前进时地速方向近似为
`psi - beta`，侧滑 `beta` 由 `u, v` 计算并在低速平滑淡出；
实际和参考都采用相同补偿，避免参考本身有侧滑时产生额外误差。
世界位置 `x=东、y=北`，船体系 `u=前、v=右`；航向北为零、东为 −90°。
横向误差使用参考地速切线方向：`e_lat = cos(chi_ref)*(x-ref.x) + sin(chi_ref)*(y-ref.y)`，
其中 `chi_ref = ref.psi - beta_ref`，`beta_ref` 由参考 `u,v` 的侧滑角计算；向参考地速右侧偏离为正。
无侧滑时参考朝北为 `dx`，朝东时为 `-dy`。
具体采用 `beta = (u²+v²)/(u²+v²+0.3²) * atan2(v, sqrt(u²+0.3²))`，
角度误差为 `wrap(ref.psi - psi + beta_actual - beta_ref)`。
终端表和诊断中的船首航向误差仍指 `wrap(ref.psi - psi)`，
航向成本字段则使用上述侧滑补偿后的误差。
取消沿程位置代价后，静止参考下的前后位置偏差不会驱动纵向纠偏，
运动中的纵向进度主要依靠参考速度跟踪；不能据此保证时间同步的位置误差。
航向和横向成本用侧滑补偿；不再有侧滑阈值或独立侧滑惩罚项。优化目标与诊断分项共用 CasADi 表达式。
任务完成独立使用 `finish_distance_m`、`finish_speed_mps` 与 `finish_stable_duration_s`。

## nmpc：实际转艏率和侧滑

| 参数 | 配置值 | 意义 |
| --- | --- | --- |
| yaw_rate_envelope_radius_m | 15.0 m | 用于实际转艏率包络：abs(r) <= yaw_rate_envelope_allowance_radps + abs(u)/15。由于有附加余量，不能把它解释为严格最小几何半径 15 m。 |
| yaw_rate_envelope_allowance_radps | 0.28 rad/s | 预测转艏率包络余量；为低速转向惯性保留空间，避免包络与指令变化率约束冲突；不改变 w_cmd ±0.3 rad/s 幅值及约 6.9°/s² 变化率。 |

## nmpc：初值、求解失败与求解精度

| 参数 | 配置值 | 意义 |
| --- | --- | --- |
| solver_max_iterations | 1000 | IPOPT 每次求解的最大迭代次数，不是控制周期数，也不是毫秒限制。 |
| solver_tolerance | 0.0001 | IPOPT 收敛容差；更小一般更严格，但可能增加求解耗时。 |

首次求解使用模型稳态前馈，已有解则移位；初值指令先限幅、限变化率，再按动力学滚动状态。
求解失败时清空旧解和预测并返回零指令，ROS 节点立即停车，不执行备用反馈控制律。
任务完成的当周期立即输出零指令，完成锁定后持续为零；零指令不等于物理刹车或闭环定点保持。

## 动力学模型与坐标校验

数值预测直接调用公共动力学求导函数；CasADi 符号动力学读取同一组 `kModelParameters`：

```text
u_dot = -0.431930955*u + 0.437133189*V_cmd
v_dot = -0.313324916*v + 0.006741061*r + 0.407958500*W_cmd + 0.000993619
r_dot = -0.166118168*v - 0.544509453*r + 0.837536662*W_cmd + 0.000626529
```

`/USV_state`、`/odom` 与 NMPC 使用同一导航航向：北=0、东=-π/2、西=+π/2。
本体速度使用 RFU 方向，`u` 前向为正、`v` 右向为正；位置运动学按北向为零的 `psi`
直接计算，`r>0` 时航向向西增加。

当前预测窗口为 10 s，前向速度指令上限为 5 m/s，角速度指令上限为 ±0.3 rad/s。
预测状态 `u/v/r` 的硬边界分别由 `max_surge_state_mps`、`max_sway_state_mps`、
`max_yaw_rate_state_radps` 单独配置；它们与输入指令限幅分开，允许状态动力学短时响应但不超出设定边界。
预测与发布使用相同的指令变化率约束，当前角速度指令变化率上限为 0.12 rad/s²。
直接使用原始融合反馈，不追加航向/速度低通；`/odom` 和 `/USV_path` 包含设定的定位噪声。
直接使用原始融合反馈，不追加航向/速度低通；`/odom` 和 `/USV_path` 包含设定的定位噪声。
