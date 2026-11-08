# 对齐1015v2接口并在空旷水域闭环测试

> 本文保留此前力模型闭环的历史结果。当前默认入口已按用户要求切换为11参数速度命令模型，并增加逐节点船体航向显示，当前使用和验证见 [IDENTIFIED_MODEL_TRACKING.md](IDENTIFIED_MODEL_TRACKING.md)。

日期：2026-10-31。本轮按用户要求，将移植前端接入1015v2现有MPC与本船仿真，使用较远目标验证实际跟踪与停车。在独立ROS域82、仅本机通信环境运行，未启动其他船只或避让场景，原河道总启动入口未替换。

## 当前启动与交互

本轮结束时RViz和本船闭环保持运行，船已停车在第二个目标附近。可直接使用 **2D Goal Pose** 继续设置目标位置和船体航向。该模式不提供预览模式的起点拖动功能：规划起点始终来自真实`/odom`，不能在界面中伪造船位。

一键启动：

```bash
bash /home/l/1015v2/run_usv_tracking.sh
```

脚本自动加载ROS和工作空间环境，默认使用域82及仅本机通信，启动地图、规划、原MPC、本船仿真与RViz。默认自动目标`(40,10,0.25rad)`；追加`auto_goal:=false`则等待在RViz中使用 **2D Goal Pose** 按住并拖动设置位置和航向。青色为参考轨迹，黄色为实测航迹。启动终端按 **Ctrl+C** 结束，再次启动重新初始化本船仿真。同域实例运行时脚本只提示使用现有RViz，不重复创建节点。`--check`只检查环境和是否已有实例。

本次启动便利性修改前后记录：此前每次需手动切换目录、加载两套环境并设置ROS域；新增工作空间根目录`run_usv_tracking.sh`完成这些步骤，支持原launch参数透传、环境检查与重复启动提示。规划、控制和仿真算法未修改。

后续修正：此前后台实例占用时仅提示在原终端按Ctrl+C，用户无法从新终端重启；现新增`--stop`与`--restart`。launch在获得锁后记录自身PID，脚本核对启动命令与ROS域后发送正常退出信号，并等待锁释放，再按需重启。旧测试实例已清理；例如`bash /home/l/1015v2/run_usv_tracking.sh --restart auto_goal:=false`可从新终端重新开始。未确认身份时不会结束进程。

等价手动启动方式（同一ROS域只启动一次）：

```bash
cd /home/l/1015v2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=82
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_kinodynamic_frontend open_water_tracking.launch.py
# 默认自动目标(40,10,0.25rad)，初速0。
# 也可加 auto_goal:=false，等待RViz给目标。
```

独立ROS域用于隔离仿真测试；在其他终端查看话题或发布目标也要使用域82。此入口启动：空旷地图、本船仿真、原MPC、新前端、RViz。没有启动旧Hybrid节点、目标船、河道管理器或全局航点节点。

## 修改前 → 修改后

| 环节/文件 | 上轮状态 | 本轮状态 |
|---|---|---|
| `tracking_node.cpp` | 只有`open_water_node`的静止起点+模型预览 | 新增真实里程计驱动的规划/执行接口节点，使用现有地图和目标输入 |
| 目标 | `/usv_frontend/goal`的预览目标 | 接收原`/goal_pose_from_astar`及RViz`/goal_pose`；归一化后向MPC提供锁存活动目标 |
| 实际起点 | 预览配置/手动静止起点 | 从`/odom`提取真实`x,y,psi,u,v,r`，保留船体侧滑与偏航率 |
| 世界坐标 | 预览固定map | 控制坐标系odom；不同世界frame通过实际平面TF转换，船体速度不随世界frame旋转 |
| 动力学 | 11参数速度命令模型固定在搜索内部 | 默认模式保持该模型；新增显式Dynamics适配器，闭环模式调用既有共享力输入船模 |
| `shared_force_dynamics.hpp` | 无 | 直接复用`hybrid_a_star_planner/usv_dynamics.hpp`的积分；输入明确为`tau_y/tau_n`，不把速度命令伪装为推力 |
| `kinodynamic_astar.hpp/.cpp` | 固定调用识别模型传播与逆动力学 | 传播/逆动力学由明确的Dynamics对象提供；原模型仍为默认；闭环种子按连接时长初始化，支持较远目标 |
| 船模配置 | 两套模型语义不同，未接入 | 闭环launch从现有`mpc_params.yaml`读取`model.*`，同一组参数传入前端、MPC、本船仿真；不修改原参数文件 |
| 轨迹输出 | 预览Path | 输出到原`/hybrid_a_star_trajectory`，保留船体航向；空间约0.2m抽样用于原Path跟踪接口 |
| 限速配对 | 无 | 同一帧号、同一时间戳的`TwistStamped`配对，5Hz刷新路径/限速/规划状态，避免原MPC因路径过期停车 |
| 重规划时序 | 预览搜索 | 后台搜索；新目标撤回旧路径；过时请求不发布；完成后若初态已变化则从最新实测状态重算，不伪造补偿起点 |
| 输入失效/取消/无完整路径 | 不涉及执行 | 发布空路径并沿用原MPC制动机制；局部视野完成不冒充完整路径供MPC执行 |
| `open_water_tracking.launch.py` | 无 | 独立闭环入口；仅本次launch覆盖MPC的低速弯道参考下限0.7m/s，原YAML与控制器源码均未修改 |
| RViz | 模型预览箭头 | 青色参考、黄色实测航迹、实际仿真船、MPC预测、真实目标距离，无预览箭头冒充实际船位 |
| CMake/package | 原移植包 | 新闭环节点、共享模型适配测试、C++17（与原共享头文件要求一致）及必要依赖 |

原有142个其他包活动源码文件的哈希复核全部一致。本轮修改集中在`src/usv_kinodynamic_frontend`。构建时补齐了`free_water_map`及其声明的安装依赖；没有运行目标船节点。

## 与现有接口对齐

| 方向 | 话题 | 消息/约定 |
|---|---|---|
| 输入 | `/odom` | `nav_msgs/Odometry`；位姿在odom或可转换的世界frame，`u/v/r`在child_frame_id船体坐标 |
| 输入 | `/local_costmap` | 原`nav_msgs/OccupancyGrid`；本轮全0开放水域图，仍保留边界和原硬占据检查 |
| 输入 | `/goal_pose_from_astar` | 原当前航点`PoseStamped`接口 |
| 输入 | `/goal_pose` | RViz当前试验目标；本次试验将其视作最终目标 |
| 输出 | `/hybrid_a_star_trajectory` | 原`nav_msgs/Path`接口，本次由新前端发布，不同时运行旧Hybrid发布者 |
| 输出 | `/hybrid_a_star/speed_limit` | 原`TwistStamped`限速配对接口，linear.x为巡航参考上限 |
| 输出 | `/hybrid_a_star/planning_status` | 原字符串状态idle/planning/tracking/blocked/cancelled |
| 输出 | `/usv_frontend/active_goal` | 锁存的同坐标系目标；本次launch将原MPC的active_goal_topic指向这里，解决RViz目标与锁存订阅差异 |
| 诊断 | `/usv_frontend/state_samples` | 每行`time,x,y,psi,u,v,r,tau_y,tau_n`，明确力输入语义 |
| 诊断 | `/usv_frontend/status`、`actual_path`、`markers` | 模型标签、真实六状态、请求状态、实测航迹及显示 |
| 执行 | `/cmd_wrench` | **仍仅由原MPC发布**纵向推力/偏航力矩，由原本船仿真接收；新前端不直接发布执行器命令 |

本次不是完整河道链路替换：航道语义、其他船只、COLREG、中间航点高速交接未接入新前端。原总启动文件保持原状。

## 验证方法与实际结果

先执行独立模型适配测试，验证与原共享模型的积分相同、逆动力学正确、航向跨±pi连续，并确认默认11参数模式没有被替换：

```bash
ctest --test-dir build/usv_kinodynamic_frontend \
  -R '^usv_frontend_shared_force$' --output-on-failure
```

再启动上述闭环入口，使用`auto_goal:=false`，运行真实接口测试：

```bash
python3 src/usv_kinodynamic_frontend/test/closed_loop_ros_test.py \
  --output artifacts/usv_frontend_closed_loop_20261031/results.json --timeout 100
```

脚本仅通过目标接口给点，记录实际`/odom`和`/cmd_wrench`，未移动仿真船位。两段连续使用同一艘船；第二段保留第一段结束时的实际位置、残余u/v/r。首段实测起点约(-0.327,0)，等待地图与规划时原仿真制动离散行为产生的偏移也没有被改写成(0,0)。

验收要求：目标距离≤1m、实际平面速度≤0.12m/s、航向误差≤30°并持续2秒；必须观测到正推力和匹配的路径/限速；全程到参考折线的最大横向偏差<2m。实际结果显著小于横向偏差门槛：

| 案例 | 实际初始位置 | 请求目标 | 到达停车耗时 | 横向RMS / 最大 | 最终目标距离 | 最终速度 | 最终航向误差 |
|---|---|---|---|---|---|---|---|
| 30m直线 | -0.327,0 | 30,0,0rad | 24.15s | 0.0017m / 0.0110m | 0.294m | 0.0062m/s | 0.00056rad |
| 弯曲段 | 29.706,-0.003 | 60,15,0.4rad | 29.34s | 0.1543m / 0.3742m | 0.510m | 0.0071m/s | 0.01266rad |

两项均通过。第二段实际航行约34m，请求目标相对原点位于(60,15)，不能把它写成从起点航行了60m。

本次接口是**空间Path跟踪**：MPC按自己的限速、曲率和终点减速机制执行，不按每个原始运动基元的时间戳逐点跟踪。因此规划模型时长（首段48秒、第二段20秒）不等于实际到达时间，本轮不能宣称时间参数化轨迹被精确执行。完整原始时间/状态/输入仍作为诊断输出保留。

1.5m/s是纵向巡航参考上限，不是包含侧滑的世界平面速度硬上限；弯曲段实测合速度会因侧滑和响应过程高于它。原物理限幅与控制器保护未绕过。

## 记录与文件

`/home/l/1015v2/artifacts/usv_frontend_closed_loop_20261031/` 包含：

- `frontend_before.tar.gz`、修改前后哈希：可核对修改前后的源文件；其他142个旧包文件未变。
- `build.log`、`model_test.log`、`launch.log`、`tracking_test.log`：构建与运行证据。
- `results.json`：完整目标、实际航迹、参考路径、推力/力矩与验收结果。首条路径到达时的异步诊断可能还显示planning，不应将它误解为MPC未执行；实际轨迹、后续日志与最终状态均单独保留。
- `summary.json`：直接从原始记录计算的误差、速度与停车统计。
- `final_frontend_status.json`：结束后实际状态为arrived，包含最终参考/实测状态。
- `tracking_comparison.png`：参考与实测航迹及横向误差曲线。
- `rviz_tracking_in_progress.png`、`rviz_tracking_final.png`：实际RViz窗口。

这两例证明当前配置下空旷水域仿真可以搜索、发布、跟踪并停车，不构成任意起终姿态、动态障碍、风浪流或实船性能保证。原11参数速度命令模式保留在预览/默认库入口；本次闭环结论只属于明确标记的共享力输入船模模式。
