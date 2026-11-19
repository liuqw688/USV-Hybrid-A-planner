# 河道无人船导航：MPC版本

ROS 2 Humble仿真项目。主链路为海图与航道语义 → 全局A*航点 → Hybrid A*船模轨迹 → MPC执行器控制 → 本船状态反馈。

当前只使用MPC跟踪。控制与仿真包为`usv_mpc_control`，本船接收`/cmd_wrench`的纵向推力与偏航力矩。
海图包实际使用Python/ament_python；未接入运行链路的C++副本已清理。

## 编译与启动

```bash
cd /home/l/work_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch channel_navigation_manager river_navigation.launch.py
```

RViz中用2D Goal Pose指定最终位置与艏向。无显示运行加`use_rviz:=false`，需要目标船加`use_virtual_boat:=true`。
入口有单实例锁，不要同时启动另一套本船仿真。

开阔水域演示：

```bash
ros2 launch free_water_map colregs_demo.launch.py scenario:=head_on use_rviz:=false
```

## 目录与职责

| 包 | 职责 |
|---|---|
| river_chart | Python解析S-57 XML，发布陆地、航道与显示 |
| local_costmap_generator | 岸边代价与距离膨胀 |
| channel_navigation_manager | 任务航道角色、中心代价、整链路启动 |
| channel_astar_global_planner | 全局稀疏航点与交接 |
| hybrid_a_star_planner | 欠驱动轨迹、降速机动与动态避碰策略 |
| usv_mpc_control | MPC推力/力矩优化、本船仿真 |
| virtual_boat_simulator | 目标船运动与状态 |
| free_water_map | 独立地图和会遇演示 |

核心文件与话题关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。当前配置说明保留在各包`PARAMETERS.md`，参数调节目的与单位在对应YAML注释中。

## 维护与验证

规划、MPC与仿真共用`hybrid_a_star_planner/usv_dynamics.hpp`中的六状态模型。
地图硬安全、航道软偏好和显示图层分开；船体Marker不代表碰撞域。
Path和限速需配对；新任务、取消、无解或输入失效时制动。低速参考不会瞬间消除实船惯性。

```bash
colcon test --packages-select hybrid_a_star_planner usv_mpc_control channel_navigation_manager river_chart
colcon test-result --verbose
python3 src/usv_mpc_control/test/test_mpc_readiness_ros.py
python3 src/usv_mpc_control/test/test_simulator_timeout_ros.py
python3 src/hybrid_a_star_planner/test/test_opposite_lane_ros.py
```

结构精简记录见[docs/CODE_CLEANUP.md](/home/l/work_ws/docs/CODE_CLEANUP.md)。原修复说明和旧测试配置保存在`docs/history/`，清理前源码快照保存在`artifacts/code_cleanup/source_before_cleanup.tar.gz`。
仿真与记录的回归结果只覆盖对应场景，不代表所有实船工况已验证。
