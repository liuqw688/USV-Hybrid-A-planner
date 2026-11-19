# 开阔水域仿真入口

在 `fast_planner_test` 工作区执行：

```bash
./run_fast_planner_open_water.sh
```

脚本加载 ROS Humble、工作区 `install` 和工作区内 `.test_deps/root` 的 NLopt，然后启动规划器、开阔地图、MPC、船舶仿真器与 RViz。RViz 固定坐标系为 `map`；使用工具栏 **2D Goal Pose** 设置位置和箭头方向。绿色线是规划器输出 B 样条的二维采样路径，紫色点是最近一次样条控制点的 x/y 投影；控制点消息中的第三分量是航向角，不是高度。规划器、地图及 RViz 的帧为 `map`，仿真器使用 `odom`，该测试明确假设 `map` 与 `odom` 重合。

本入口依赖工作区已构建产物和 `.test_deps/root`；换机器时需根据 ROS 版本和架构重新安装或编译依赖、构建工作区。该场景中的采样适配器将 B 样条转为 MPC 的几何 Path，MPC 不按样条时间执行；轨迹过期会撤回 Path。控制器/仿真器船模与规划器船模不同，不能把测试结果当成实际船舶验证。每次测试的条件、结果与问题见 `docs/测试记录.md`，原始数据见 `docs/test_data/`。

开阔水域演示目前试用射击触发距离 40 m、搜索启发式权重 2；规划器通用入口默认分别为 25 m、5。可通过启动参数覆盖，例如：

```bash
./run_fast_planner_open_water.sh shot_trigger_distance:=25.0 search_heuristic_weight:=5.0
```

此试验配置可能让射击延伸到船周围 50 m 局部更新窗口之外；只应用于地图和感知覆盖满足规划需求的场景。反向目标的闭环表现仍未解决，见测试记录。实际到达条件在 `fast_planner/plan_manage/launch/kino_replan.launch.py` 设置，为 `fsm/arrival_pos_tolerance`（米）和 `fsm/arrival_yaw_tolerance`（弧度）；修改后需重新构建 `plan_manage` 并重启节点。
