# usv_tracking_test

独立的仿真测试包，包含测试参考发布器、组合启动文件和闭环路线回归。正式控制器仅依赖自身接口与预测模型。测试包单向依赖控制器，控制器不依赖测试包、地图或仿真包。

使用 `usv_planner` 进行真实规划与跟踪闭环，直接按 [仿真测试快速启动](../../README_SIMULATION.md) 的五终端步骤操作；下述四终端方式使用测试参考发布器。

## 四终端独立启动

每个终端先执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
```

终端 1：地图、原点、里程计转换和 RViz。

```bash
ros2 launch usv_map usv_map.launch.py simulate_perception:=false
```

终端 2：USV 动力学仿真，等待 `/fix` 后发布 `/USV_state`。

```bash
ros2 launch usv_model_sim usv_model.launch.py
```

终端 3：正式 NMPC 控制器，接收 `/odom` 和 `/planning/bspline`。

```bash
ros2 launch usv_nmpc_controller usv_nmpc.launch.py
```

待控制器获得 `/odom` 后，终端 4：Python 局部路径发布脚本，按船位裁出全局时间样条的前方最多 50 m，约 10 Hz 发布。

```bash
USV_NMPC_PUBLISH_RATE_MODE=fixed python3 /home/sus/USV_systems/src/usv_tracking_test/test/numeric_routes/route_shapes.py
```

脚本首次收到 `/odom` 后计算完整动力学参考，然后开始滚动发布。它只发布参考，不拉起其他节点。末段不足 50 m 时按剩余长度发送；`[x,y,phi]` 控制点包含连续展开的船首航向，时间导数携带速度信息。1～10 Hz 随机更新的命令见 [工作空间 README](../../README.md)。地图、仿真初始状态和控制配置亦在该文档分别说明。

## 组合测试入口

```bash
ros2 launch usv_tracking_test usv_tracking.launch.py
```

此入口默认只启动地图和动力学仿真，替代独立启动中的终端 1、2；随后启动 NMPC，再运行 Python 参考脚本。可用选项：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `start_controller` | `false` | 设为 `true`，同时启动 NMPC |
| `publish_reference` | `false` | 设为 `true`，启用简易整条几何参考；50 m 滚动测试保持 `false` |
| `rviz` | `true` | 是否显示 RViz |
| `simulate_perception` | `true` | 是否启动模拟障碍感知 |
| `config_file` | 正式控制器默认配置 | 控制器参数文件绝对路径 |

例如同时启动仿真环境与 NMPC，局部参考仍由 Python 脚本单独发布：

```bash
ros2 launch usv_tracking_test usv_tracking.launch.py start_controller:=true
```

组合入口与四终端方式二选一，避免重复启动同名节点或多个参考发布器。参考脚本应最后启动；控制器晚启动时，已经按时间推进的旧参考可能超过 2 m 接受范围。重启测试时先停止参考脚本，待控制器就绪后重新启动脚本。

## 简易几何参考发布器

```bash
python3 /home/sus/USV_systems/src/usv_tracking_test/scripts/publish_bspline_reference.py
```

也可通过包执行或 launch 启动：

```bash
ros2 run usv_tracking_test publish_bspline_reference.py
# 与上一条命令二选一。
ros2 launch usv_tracking_test publish_bspline_reference.launch.py
```

它使用首次 `/odom` 生成完整几何样条，预留 3 s 启动等待，以 1 Hz 重发同一消息，供消息接收、样条求值及 RViz 显示检查；它不是 50 m 滚动发布器，也不保证满足船体动力学。默认割草路线为 5 条 300 m 航段，间距 60 m，巡航参数 4 m/s。环境变量 `USV_NMPC_ROUTE_MODE` 支持 `straight`、`reverse_straight`、`arc_left`、`arc_right`、`s_curve`、`mowing`；`USV_NMPC_ROUTE_LENGTH_M` 和 `USV_NMPC_ROUTE_SPEED_MPS` 可覆盖长度与速度。

可视化路径 `/planning/bspline_path` 采用瞬态本地 QoS；简易发布器生成新轨迹时发送，滚动发布器约每秒刷新。地图里的 `/odom` 默认 20 Hz，`/USV_path` 默认 1 Hz。

## 检查与测试

真实规划器闭环测试需分别启动地图、船模、NMPC 和 `usv_planner`，不要同时运行上述
Python 参考发布器。待节点就绪后，以下命令向同向航道发布 `(50 m, 0 m)`、航向
`−90°` 的单点目标，并记录最多 100 秒：

```bash
ros2 launch usv_planner usv_planner.launch.py
# 在另一个已 source 的终端执行记录器。
python3 /home/sus/USV_systems/src/usv_tracking_test/test/record_planner_tracking.py \
  /tmp/usv_systems_runs/channel_tracking --east 50 --north 0 --phi -90 --duration 100
```

记录器仅发布一次目标，不发布样条，不修改节点参数。输出 `tracking_overview.png`、
`summary.json`、原始 CSV、完整样条和代价图快照；以船模真值评估实际船迹，并使用同一条
NMPC 诊断内的状态与参考计算同步误差。`--plot-only` 可根据已有数据重新绘图。
若规划器按事件发布、更新间隔超过 NMPC 的 `reference_stale_timeout`，控制器会超时停车；
应先检查两者发布协议与超时配置，不能将此情况解释为正常连续跟踪结果。

```bash
ros2 topic hz /odom
ros2 topic hz /planning/bspline
ros2 topic echo /nmpc_tracking_state --qos-durability transient_local
ros2 topic echo /cmd
colcon test --packages-select usv_nmpc_controller usv_tracking_test
colcon test-result --verbose
```

`reference_publisher_test` 检查六类几何路线的节点向量、末端停车导数及完整 Path 采样，并用 SciPy 独立比对求值；`global_spline_slice_test` 检查全局曲线裁剪、投影进度及发布时序。NMPC 诊断说明见 [控制器 README](../usv_nmpc_controller/README.md)。测试并行于其他仿真时，在同一组终端使用独立且一致的 `ROS_DOMAIN_ID`。

## 复合路线滚动 B 样条测试

`run_suite.py` 使用数值船模、里程计和实际 NMPC 节点测试固定路线：R7 圆、直线 200 m、R10 圆、直线 200 m、R15 圆、直线 200 m。测试初始 `phi` 为 0°、30°、−60°、−90°、180°；默认船模初始位置为 ENU `(2 m, 2 m)`，全局路线仍从 `(0 m, 0 m)` 开始。仿真器也可单独用 `initial_east_m`、`initial_north_m` 参数设置初始位置。

复测直接读取 `usv_nmpc_controller/config/usv_nmpc.yaml` 的预测窗、指令范围和完成判定配置；默认预测窗为 100 步 × 0.1 s，不再由脚本固定覆盖为 120 步。每组的 `test_setup.json` 记录实际生效的预测窗、两通道增量权重和定位噪声，便于核对复现条件。

测试发布器先沿 R7 圆、直线 200 m、R10 圆、直线 200 m、R15 圆、直线 200 m 的基准几何，用识别出的六状态船体模型积分一条完整动力学轨迹，再对该轨迹拟合一次全局时间参数化三次 B 样条。每 0.1 s，发布器把当前 `/odom` 位置投影到这条全局样条，并精确裁剪当前位置起、沿曲线弧长向前最多 50 m 的区间；局部消息的样条控制点因此始终来自同一全局曲线，不会逐次另行生成或改变路线。末段按剩余长度缩短，终点片段保持发布以供控制器完成判定。控制点只包含 `x、y、phi`，速度和转艏率由时间导数获得。新局部样条起点距当前 `/odom` 超过 2 m 时 NMPC 拒收，并保留已接受的参考。速度廓线计入油门指令下降斜率，以便在设计终点附近停稳；直线巡航目标为 4 m/s，参考转艏率限制为 6°/s。

该复合路线是固定的测试基准，用来验证动力学可行的全局样条及其实时局部片段发布；它不是针对环境地图或障碍代价求解出的数学意义全局最优路径。部署时应将全局规划器输出的可行路径交给同一套动力学时间参数化与局部样条裁剪流程。

NMPC 终端表的“样条段进度”是当前单条 B 样条内的局部进度。滚动发布器每 0.1 s 更新参考后该数值会重新从零附近计数，不能用来判断全局路线进度或是否长时间未收敛；全局路线误差应查看滚动测试生成的 `tracking_paths.png`。

```bash
cd /home/sus/USV_systems
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT_DIR=/tmp/usv_systems_runs/rolling_composite
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 src/usv_tracking_test/test/numeric_routes/run_suite.py "$OUT_DIR" --parallel 2
python3 src/usv_tracking_test/test/numeric_routes/analyze_rolling_suite.py "$OUT_DIR"
```

需要做可重复的里程计滤波对照时，可增加 `--noise-seed 731 --odom-position-alpha 0.1`；横向权重试验使用 `--weight-lateral`。这些参数只覆盖本次测试配置，不改生产配置。建议把每次结果写到 `/tmp/usv_systems_runs/` 下的新目录，避免实验数据混入源码。分析器写入 `tracking_paths.png`、`control_channels.png`、`state_timeline.png` 和 `rolling_summary.json`；轨迹图叠加设计路线、拟合前的动力学积分参考、完整全局 B 样条及 NMPC 实际船迹，误差子图显示实际船位到全局 B 样条的最近几何距离。控制图显示 `u/v/r` 反馈和参考、`V_cmd/W_cmd` 及同一诊断内的 `/odom` 到参考点距离。汇总分别列出滤波后 `/odom` 位置误差和原始位置测量噪声，并报告全局样条动力学残差、实际航迹对全局样条的误差、局部发布频率、片段长度、样条起点偏差和求解成功率。`true_to_local_reference` 使用诊断接收时刻插值真值，包含求解和传输延迟；同步参考点误差应查看 `odom_to_local_reference` 字段。

`state_timeline.png` 根据实际收到的 `/nmpc_tracking_state` 事件绘制状态机时间线，并放大启动阶段和相对完成时刻的到达确认阶段。正常滚动替换样条应持续处于 `TRACKING`；保护状态会标注原因。汇总同时记录状态序列和保护次数。轨迹误差图会标注超过 0.3 m 的峰值；单轮全部完成及求解成功，不代表全程精度达标，也不代表多轮精度已经稳定。
