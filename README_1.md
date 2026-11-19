# 规划与跟踪仿真：分步启动

ROS 2 Jazzy；运行链路：地图 → 船模 → 规划器 → NMPC → 船模。
以下测试由真实 `usv_planner` 生成 B 样条，记录器只发布目标和绘图。

## 0. 首次编译

```bash
cd /home/sus/USV_systems
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

按顺序打开 5 个终端，**每个终端都先执行**：

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
```

## 1. 终端一：地图与 RViz

```bash
ros2 launch usv_map usv_map.launch.py \
  rviz:=true channel_navigation:=true simulate_perception:=true
```

开启航道代价 `/channel_costmap`、地图原点 `/fix`、里程计转换和模拟感知。
无需模拟感知时改为 `simulate_perception:=false`。
没有图形界面时改为 `rviz:=false`。船模启动后才有 `/odom`。
RViz 的青色 `Local planner B-spline` 图层订阅 `/planning/bspline_path`，显示规划器实际发布
样条的采样曲线；绿色 `Mission waypoints` 订阅 `/path` 显示完整任务航点，橙色 `USV path`
是实际船迹。Fixed Frame 保持为 `map`。先启动 RViz，再发送一次性航点任务。

## 2. 终端二：无人船仿真

```bash
ros2 launch usv_model_sim usv_model.launch.py
```

默认从东向位置 `0 m`、北向位置 `0 m` 附近开始，船首向东；订阅 `/cmd`，发布 `/USV_state`。

## 3. 终端三：NMPC

```bash
ros2 launch usv_nmpc_controller usv_nmpc.launch.py
```

规划器按需重规划，并以 10 Hz 发布当前有效轨迹；NMPC 默认在 2 秒未收到有效
参考消息后保护停车。已有进程需要重启才能加载新编译的节点。

## 4. 终端四：局部规划器

```bash
ros2 launch usv_planner usv_planner.launch.py goal_input_mode:=1
```

接收 `/goal_pose`，结合 `/odom` 和 `/channel_costmap`，向 `/planning/bspline` 发布轨迹。
此测试不要同时运行 Python 样条参考发布器，否则会与规划器争用同一话题。

如需沿默认海图顺时针航行一周，将本步骤改为
`ros2 launch usv_planner usv_planner.launch.py goal_input_mode:=2`，然后在终端五执行：

```bash
ros2 run usv_planner publish_clockwise_waypoints.py
```

发布器等待地图、里程计和规划器，向 `/path` 发送一次 53 个闭环航点；中途航点不停船，
仅任务末点停车。模式 2 沿航点折线滚动前视，默认 `search_horizon=60 m`，与模拟感知半径
一致，局部栅格裁切半宽同步使用该值；默认前进 `8 m` 触发常规重规划。海图及入口选择说明
见 `src/usv_planner/README.md`。
此模式不要同时运行下面的单点目标记录器。


## 手动目标与停止

停止当前用户启动的本工程地图、船模、控制器、规划器和测试脚本：

```bash
/home/sus/USV_systems/kill_usv_systems.sh --dry-run  # 预览进程
/home/sus/USV_systems/kill_usv_systems.sh            # 停止进程
```

脚本也会停止本工程启动器创建的 RViz 等子进程。`ros2 launch/run` 按包名匹配，
同一用户从其他工作空间启动同名包时也会被列入；执行前可先预览。

手动测试时可用 RViz 的 **2D Goal Pose / Pose Goal** 工具，话题设为 `/goal_pose`、
Fixed Frame 设为 `map`，拖动箭头指定船首方向。不要同时让记录器再发另一目标。
上述记录器会自动发送自己的目标，只记录手动目标的模式尚未提供。

查看控制状态：

```bash
ros2 topic echo /nmpc_tracking_state --qos-durability transient_local
```

正常为 `WAIT_REFERENCE → TRACKING → ARRIVAL_CONFIRM → FINISHED`。
结束测试时在各终端按 `Ctrl+C`；复测先重启船模、NMPC 和规划器，再运行终端五。
