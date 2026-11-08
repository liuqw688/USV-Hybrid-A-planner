# 空旷开放水域 / RViz 交互测试记录

> 本文件记录原11参数模式的搜索/预览；真实MPC跟踪及共享力模型入口见 [CLOSED_LOOP_TRACKING.md](CLOSED_LOOP_TRACKING.md)。

日期：2026-10-31。按用户本轮要求，仅运行移植后的前端包，在空旷开放水域验证完整轨迹搜索和RViz显示。**本轮不修改或测试其他船只避让，不运行旧的障碍/水域代价图测试，不接入Hybrid、MPC或执行器。** 用户已授权在新包调整测试接口。

## 使用方式

```bash
cd /home/l/1015v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usv_kinodynamic_frontend open_water_rviz.launch.py
```

本轮结束时该入口与RViz保持运行。初次启动自动搜索 `(0,0,0,0,0,0)` 到 `(2,1,0.5,0,0,0)`；本次测试结束后界面保留最后一个目标 `(6,2,0.4,0,0,0)`。

1. **2D Pose Estimate**：按住鼠标并拖动设置起点位置和船体航向，默认实际 `u/v/r=0`。
2. **2D Goal Pose**：按住并拖动设置目标位置和船体航向，目标实际 `u/v/r=0`。
3. 青色曲线是前端积分轨迹；绿色/红色大箭头是起点/目标；橙色小箭头是船体航向；移动青色箭头是沿积分轨迹的模型预览。

所有显示使用`map`固定坐标系。每格1米。视图已设置合适缩放，可以用RViz Move Camera调整。

“模型预览”只回放搜索结果，不是MPC闭环或实船仿真。`GOAL REACHED (MODEL)` 表示终端六状态满足容差；`PARTIAL HORIZON`不能当作完成目标；`NO PATH`表示预算内无解，不能据此推断物理不可达。试验坐标范围±40m，船体半径仍计入边界判断。

## 修改前 → 修改后

| 文件/环节 | 修改前 | 修改后 |
|---|---|---|
| `src/open_water_node.cpp` | 无ROS/RViz入口，只有离线演示 | 新增专用交互节点；起点/目标输入、轨迹/完整状态采样/诊断输出、船体航向及模型预览 |
| 节点调度 | 无交互调度 | 后台搜索，不阻塞起点和目标输入；新请求清除旧显示，旧请求结果不覆盖新请求 |
| `SearchConfig`、`kinodynamic_astar.cpp` | 终端连接范围硬编码2.5m，候选总时长硬编码4/6/8秒 | 新增可配置`connection_range/connection_durations`；库默认保持原值；开放水域入口单独采用30m与4/6/8/12/16/24/36秒 |
| 搜索目标 | 离线默认5m局部视野 | 开放水域入口视野100m，要求搜索到最终目标；不将局部段冒充终点 |
| 模型/数值积分 | 11参数模型、RK4 | 保持；长连接仍由模型传播并检查全部六状态与输入限值，没有改成几何直线拼接 |
| `config/open_water.yaml` | 无 | 新增空旷环境、静止起点、目标姿态、运行时间及连接时长配置 |
| `launch/open_water_rviz.launch.py` | 无 | 一条命令启动新包节点和RViz，可设`use_rviz:=false`只启动节点 |
| `rviz/open_water.rviz` | 无 | 固定map、1m栅格、轨迹、起终点、船体航向、交互工具；为ROS2 MarkerArray使用实际有效的`Topic`属性 |
| 坐标显示 | 未提供TF | 专用`map → usv_open_water_origin`静态单位变换，使RViz固定坐标系状态正常，不转换或假定旧导航链路TF |
| CMake/package.xml | 仅库与离线程序 | 增加rclcpp/tf2_ros/标准几何、状态、显示消息依赖及新节点，安装launch/rviz配置 |
| `test/open_water_ros_test.py` | 无 | 新增仅开放水域的6个接口测试，核对完整终端状态、状态采样/RK4及RViz订阅 |
| 原水域/避让与旧导航包 | 已存在的相关代码 | 本轮未修改；未运行相关测试 |

首轮测试2米目标可解，但两个约6米目标在3秒预算内无解，结果保存在`first_pass_results.json`。问题出现在测试仍沿用近距离、短时长连接参数。将连接参数显式开放给试验入口后，6米案例均由12秒的分段船模命令连接完成。首轮失败保留，没有删除失败样例或将其当成成功。

## 本轮专用接口

全部位于新命名空间，不复用旧Hybrid/MPC控制话题：

| 话题 | 消息 | 含义 |
|---|---|---|
| `/usv_frontend/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz起点，位置和船体航向；重置实际速度为零 |
| `/usv_frontend/goal` | `geometry_msgs/PoseStamped` | RViz目标位置和船体航向 |
| `/usv_frontend/path` | `nav_msgs/Path` | 前端积分轨迹，姿态为船体航向，transient local |
| `/usv_frontend/state_samples` | `std_msgs/Float64MultiArray` | 每行9列：`time,x,y,psi,u,v,r,v_cmd,w_cmd`，保留前端完整信息 |
| `/usv_frontend/status` | `std_msgs/String` | JSON：请求编号、真实搜索状态、起终点、采样数、耗时、扩展数 |
| `/usv_frontend/markers` | `visualization_msgs/MarkerArray` | 试验显示，不是控制/避让数据 |

起点和目标必须是有效的平面姿态且frame为`map`，本节点不会默认将其他frame当作相同frame。输出状态采样中的命令仍为v_cmd/w_cmd，不是推力、力矩或即时实际速度。

## 最终实际结果

Release构建成功。仅运行以下开放水域脚本：

```bash
python3 src/usv_kinodynamic_frontend/test/open_water_ros_test.py \
  --output artifacts/usv_open_water_rviz_20261031/results.json
```

| 案例 | 起点(x,y,psi) | 目标(x,y,psi) | 最终状态 | 位置误差(m) | 采样数 |
|---|---|---|---|---|---|
| 2m直线 | 0,0,0 | 2,0,0 | reach_end | 0.169 | 301 |
| 左转 | 0,0,0 | 2,1,0.5 | reach_end | 0.227 | 301 |
| 右转 | 0,0,0 | 2,-1,-0.5 | reach_end | 0.217 | 301 |
| 超过原5m局部视野 | 0,0,0 | 6,0,0 | reach_end | 0.128 | 601 |
| 平移/旋转起点 | 1,-1,0.4 | 3,0.2,0.6 | reach_end | 0.078 | 301 |
| 6m开放水域弯曲轨迹 | 0,0,0 | 6,2,0.4 | reach_end | 0.274 | 601 |

六项全部满足：位置误差≤0.35m、最短航向误差≤0.25rad、终端`|u|≤0.15m/s`、`|v|≤0.10m/s`、`|r|≤0.12rad/s`。整段相邻采样的独立RK4复核最大状态差小于`1.7e-11`。搜索实测约0.000～0.004秒，仅代表这6个空旷场景，不代表任意起终点或预算下都能成功。

RViz实际窗口已检查：固定坐标系与轨迹显示均为OK，起点、目标、轨迹、航向和预览可见；自动测试确认RViz实际订阅轨迹与MarkerArray。该测试通过RViz同名输入话题送入姿态，未声称已自动执行鼠标拖拽。

产物位置：`/home/l/1015v2/artifacts/usv_open_water_rviz_20261031/`，包括构建/启动/测试日志、完整6场景状态与命令、首轮失败结果、RViz窗口截图、修改前后源文件清单。原有142个活动源码再次逐项哈希核对，未被修改。
