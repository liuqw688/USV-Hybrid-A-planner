# INS 采集轨迹回放与 Hybrid A* / MPC 闭环

输入 `data_ins_20261019_153413.csv`，读取弧度制 `INS_LAT`、`INS_LON` 和米制
`ins_altitude`，以首条记录为 WGS84 ENU 原点（东 x、北 y），按二维累计弧长等距插值
为 30 点，包含首末点，共 29 个等弧长间隔。每点朝向下一点，末点沿用最后一段朝向。
保留原始数据，不做平滑或跳点剔除。不是按CSV行号或时间平均分段。

本包代替全局 A*，发布 `/astar_channel_waypoints` (`nav_msgs/Path`) 和
`/goal_pose_from_astar` (`geometry_msgs/PoseStamped`)；不直接代替 Hybrid 的局部路径。
现有 Hybrid 接收目标后发布 `/hybrid_a_star_trajectory` 和配对限速，原 MPC 控制
`/cmd_wrench`，原六状态欠驱动船模通过 `/odom` 反馈。
不开启海图、航道、全局 A* 或目标船。本次 launch 关闭航道，并默认将 Hybrid
`cruise_speed`、MPC `max_speed`、船模 `max_linear_speed` 同步覆盖为 **1.2m/s**。
大转向参考速度和MPC中间点速度下限也限制到该上限以内。覆盖只在本次启动生效，
不改 Hybrid/MPC/船模原包的配置文件和源码，其余参数沿用原包。

## 一键启动

```bash
cd /home/l/1015v2
./run_ins_replay.sh
```

脚本自动构建三个所需功能包并加载本工作空间。打开 RViz，显示原始轨迹（灰）、
30点路线（橙）、航点编号/朝向、当前目标（黄）、Hybrid局部路径（绿）和实际轨迹（蓝）。
开放水域用网格和边界线显示，避免当前显卡的栅格纹理着色器兼容问题；
`/local_costmap` 仍正常发布供规划与控制使用。
船从原点静止起步，初始朝向为第1→2点方向。首点作为已在起点到达的目标交接，
随后依次推进；中间点距离≤3m时交接，末点要求距离≤0.6m、速度≤0.15m/s、
艏向误差≤30°持续2秒。最终仍保留末点目标和整条路线以维持停车，不循环回起点。
若已经在末点附近停稳，但超过0.6m严格位置判据，状态会分别标出
`terminal_settled=true` 和 `completed=false`，不会放宽判据宣称通过。
结束按 Ctrl-C；再次启动会从头开始。不要同时运行另一套本船仿真。

只运行无显示闭环：

```bash
./run_ins_replay.sh use_rviz:=false
```

自行覆盖 `csv_path:=/absolute/path.csv`、`point_count:=30`、`angle_unit:=radians`、
`advance_radius:=3.0`、`final_radius:=0.6`、`max_speed:=1.2` 或 `output_dir:=/absolute/fresh/directory`。
经纬度单位必须明确指定；默认是这份文件采用的弧度。

已经构建时也可直接运行：

```bash
source /opt/ros/humble/setup.bash
source /home/l/1015v2/install/setup.bash
ros2 launch ins_path_replay ins_replay.launch.py
```

## 闭环测试和指标

```bash
source /opt/ros/humble/setup.bash
source /home/l/1015v2/install/setup.bash
ros2 run ins_path_replay run_evaluation.py \
  --output /home/l/1015v2/artifacts/ins_replay/run_speed_1p2 --max-speed 1.2 --timeout 1800
```

测试使用独立 DDS 域186，启动同一个 launch，保留节点日志、全部目标事件、变化后的
局部路径、船模实测状态、原始ENU和30点数据。输出 `REPORT.md`、`evaluation.json`、
`tracking_overview.png`、`speed_and_progress.png`。只清理测试脚本启动的进程。
同时统计距原始数据折线、30点折线和当前Hybrid路径的距离，避免把稀疏化误差与MPC误差混淆。
原始折线取全局最近距离，交叉/近邻路段需结合有序航点完成情况判断。
指标区间截止末点附近停稳，不用之后长时间静止等待降低误差统计；完整记录仍保留。
采集的时间和速度不作为控制指令，实际速度由原规划和MPC在本次1.2m/s上限内决定。
仿真结果只代表当前六状态模型，不能代替实船测试。

常规 RViz 回放也会在输出目录记录数据。默认一键脚本使用带时间戳的新目录；
原始输入CSV不会被修改。运行中可查看 `/ins_replay/status`。
根目录一键脚本默认直接读取工作空间根目录的原CSV；单独使用 ros2 launch 时
默认读取包内保存的同内容数据副本，也可以用 `csv_path` 明确指定。

## 本次实测结果

此前4m/s配置闭环已走到最后一个目标并停车，原始严格0.6m位置验收未通过，
终点误差约0.713m、艏向误差约2.53°；没有为让结果通过而修改Hybrid或MPC。
完整测试记录与图表见工作空间 `artifacts/ins_replay/closed_loop_01/REPORT.md`。

1.2m/s启动覆盖复测已执行至第30点停稳，耗时约797秒：Hybrid路径跟踪RMS约0.094m，
原始数据轨迹距离RMS约1.624m；终点误差约0.733m、艏向误差约6.74°，
仍未通过0.6m严格终点位置判据。最高实测纵向速度1.200m/s，
含侧滑的地面合速度峰值1.260m/s（原有max_speed约束的是纵向速度）。
报告见 `artifacts/ins_replay/closed_loop_speed_1p2_01/REPORT.md`，
与4m/s对比见同目录 `COMPARISON.md`。
