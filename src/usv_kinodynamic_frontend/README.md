# usv_kinodynamic_frontend

闭环前端终端连接范围恢复为120米，搜索代价恢复为命令幅值惩罚；普通搜索控制段为0.5/1.0秒。修改与回退记录见 [COMMAND_CHANGE_COST.md](docs/COMMAND_CHANGE_COST.md)。

分段搜索试验已按用户要求回退，恢复原有搜索流程；试验记录见 [SEGMENTED_CONNECTION.md](docs/SEGMENTED_CONNECTION.md)。

RViz用紫色粗线高亮参考轨迹中最大几何曲率附近的轨迹，用圆点标出最大曲率位置，旁边显示曲率与半径。计算时排除沿参考轨迹距起点和终点各2米的区域，并使用发布的Path连续三个点的外接圆估算，跳过小于0.1米的短弦；近直线时半径为无穷大。轨迹太短或没有有效采样时不显示最大曲率。这是采样几何估计，不是船体航向变化率除以速度，也不是实际航迹的曲率。

白色状态文字包含轨迹来源：起点直接数值连接、Kinodynamic A*搜索后连接，或仅通过搜索到达终点；分类依据实际搜索节点的终止方式，不根据距离推测。

2026-11-01已按用户要求恢复稀疏化之前的控制段、离散、终端连接和显示，将候选RK4最大步长改为0.1秒，时间预算/局部范围/节点池为8秒/500米/20万。用户随后要求增大输出状态间隔，当前为0.1秒，可通过启动参数`output_step:=...`设置；其余配置保持回退版。修改记录见 [ROLLBACK_RK4_ONLY.md](docs/ROLLBACK_RK4_ONLY.md)。

从 Fast-Planner 本轮无人船适配代码移植的**独立前端规划包**。状态 `[x,y,psi,u,v,r]`，输入 `[v_cmd,w_cmd]`，采用用户提供的11参数辨识模型、RK4运动基元、六状态离散、船舶时间启发式和数值终端连接。

默认识别模型的搜索内核不依赖Fast-Planner、三维SDF、B样条、NLopt或PCL。11参数方程现由MPC包导出的共享头文件提供；旧力模式适配器仍依赖原Hybrid共享船模。首次移植提供库和离线演示，现已接入空旷开放水域的ROS/RViz闭环。

**当前默认闭环使用用户提供的11参数`v_cmd/w_cmd`模型**：前端、MPC预测和本船仿真调用同源方程，通过`/cmd_vel`传递速度命令。旧力模式可用`dynamics_model:=force`对照，仍使用`/cmd_wrench`。原河道总入口未切换。当前修改前后说明见 [IDENTIFIED_MODEL_TRACKING.md](docs/IDENTIFIED_MODEL_TRACKING.md)。

## 在RViz进行实际MPC跟踪测试（新增）

在独立ROS域82运行，可在RViz窗口用 **2D Goal Pose** 指定目标。青色为参考轨迹，橙色箭头为每个发布节点的船体航向，黄色为真实`/odom`航迹，蓝色船体为MPC控制的仿真船。节点密集时可滚轮放大查看箭头。

一键启动（自动加载环境；同域已有实例时提示直接使用，避免重复启动）：

```bash
bash /home/l/1015v2/run_usv_tracking.sh
```

默认自动驶向`(40,10,0.25rad)`。仅手动选点可追加`auto_goal:=false`；自定较远目标可追加`goal_x:=60.0 goal_y:=15.0 goal_yaw:=0.4`。在RViz用 **2D Goal Pose** 按住并拖动设置目标位置和船艏方向，观察青色参考轨迹与黄色实测航迹。启动终端按 **Ctrl+C** 结束，再启动会重新初始化仿真。`--check`只检查环境，不启动节点。

如果原测试仍在后台，可用`bash /home/l/1015v2/run_usv_tracking.sh --stop`关闭；直接重置并等待手动选点可用`bash /home/l/1015v2/run_usv_tracking.sh --restart auto_goal:=false`。停止/重启只处理当前ROS域中经身份核对的本闭环启动进程，使用正常退出信号。

等价的手动启动方式（先结束同域旧实例）：

```bash
cd /home/l/1015v2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=82
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_kinodynamic_frontend open_water_tracking.launch.py
```

默认目标`(40,10,0.25rad)`，从静止起步；用`auto_goal:=false`等待RViz目标。起点来自实际里程计，不能拖动重置。原`/odom`、`/local_costmap`、目标Pose、Path、匹配时间戳限速及规划状态接口已接入原MPC。

此前力模型已实测30m直线和接续弯曲段，历史记录见 [CLOSED_LOOP_TRACKING.md](docs/CLOSED_LOOP_TRACKING.md)。本轮11参数闭环结果单独记录，见 [IDENTIFIED_MODEL_TRACKING.md](docs/IDENTIFIED_MODEL_TRACKING.md)。均为空旷仿真空间Path跟踪，未据此宣称逐时刻轨迹跟踪或实船性能。

## 在RViz测试空旷开放水域

```bash
cd /home/l/1015v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usv_kinodynamic_frontend open_water_rviz.launch.py
```

启动后自动搜索初始 `(0,0,0)` 到目标 `(2,1,0.5rad)` 的轨迹，两端实际速度默认为零。

- 点击工具栏 **2D Pose Estimate**，在水面按住并拖动，设置静止起点与船艏方向。
- 点击 **2D Goal Pose**，按住并拖动，设置目标位置与船艏方向；松开后自动搜索。
- 绿色为起点，红色为目标，青色曲线为轨迹，橙色小箭头为船体航向。移动的大青色箭头是沿前端轨迹的模型预览，不是控制器或实船运动。
- `GOAL REACHED (MODEL)` 表示位置、航向与 `u/v/r` 全部满足终端容差；`PARTIAL HORIZON` 仅为局部段，`NO PATH` 表示当前预算下没有解。新任务开始时先清空旧轨迹，过时结果不覆盖新目标。

本入口使用空旷、无障碍的开放水域，不接收或测试其他船只，不涉及避让。配置见 `config/open_water.yaml`，范围为以原点为中心±40m；不把地图边界外当作可通航水域。当前终端位置容差0.35m，航向容差0.25rad，实际速度容差沿用搜索配置。

本轮仅运行开放水域测试，不重跑旧障碍/水域代价图测试：

```bash
# 先保持上面的RViz入口运行，再开一个终端：
python3 src/usv_kinodynamic_frontend/test/open_water_ros_test.py \
  --output /tmp/usv_open_water_results.json
```

本轮接口与修改前后记录见 [OPEN_WATER_RVIZ.md](docs/OPEN_WATER_RVIZ.md)。

## 构建与原始离线演示

```bash
cd /home/l/1015v2
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --packages-select usv_kinodynamic_frontend \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ros2 run usv_kinodynamic_frontend usv_frontend_demo --output /tmp/usv_frontend_result.json
```

演示从静止到 `(2,0,0,0,0,0)`，在已知开阔二维水域中搜索。输出每个时刻的六状态、两命令、真实规划状态和代价。`reach_horizon` 是局部段，不能当成已到最终目标；`no_path` 时退出码2，不能解释为物理不可达。

可用 `--goal-x/--goal-y/--goal-yaw/--max-search-time/--hull-radius` 更改演示条件，航向单位rad。该程序用于离线算法验证，不连接运行中的船舶。

## 作为库使用

```cpp
#include <usv_kinodynamic_frontend/occupancy_grid.hpp>
using namespace usv_kinodynamic_frontend;
WaterwayMap map = fromOccupancyGrid(existing_grid_message);
State start, goal; // 必须完整填入x,y,psi,u,v,r，同一个地图坐标系
start << 0,0,0,0,0,0;
goal << 2,0,0,0,0,0;
auto result = WaterwayPlanner().plan(map, start, goal);
// result.samples包含time/state/command；不要丢掉时间、实际横荡速度和偏航率。
```

调用方 `find_package(usv_kinodynamic_frontend REQUIRED)` 后使用 `ament_target_dependencies(your_target usv_kinodynamic_frontend)` 或导出的CMake库目标。全局TF与消息时效性由未来接入端负责，本包不会将不同frame默认为相同frame。

## 无人船水域适配

- 世界坐标平面地图与船体航向分开，地图没有“psi作为z”的用法。
- 完整圆形船体足迹与格子做相交检查，包括船体内部；加可配置净空，不能只检查中心或圆周。
- 地图外、未知 `-1`、原环境硬缓冲 `99` 与陆地 `100` 默认不可通行；默认硬阈值88，已有膨胀层会增加保守性。
- 支持地图原点的二维旋转和平移，非平面地图拒绝。
- 岸边软代价作为非负沿时间积分加入g代价；不能覆盖硬占据。不会将不同语义地图层偷偷合并，航道任务锁存、对向偏好与COLREG尚未移植。
- 根据地图分辨率及最大实际船速限制碰撞采样间隔。
- 前端通过模型传播只能保证模型和采样容差内的轨迹；后端改变位置、航向或时间后仍需独立验证。Fast-Planner侧已实现B样条验收，新包本轮只移植前端，不伪造后端已接入。

代码来源、修改前后对照、模型差异和验收记录见 [PORTING.md](docs/PORTING.md)。全部参数通过 `SearchConfig/WaterwayConfig` 设置；`config/engineering_defaults.yaml` 为说明性对照，不会被无节点的库自动加载。
