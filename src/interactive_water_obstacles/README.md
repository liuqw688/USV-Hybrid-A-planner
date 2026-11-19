# 交互式开阔水域障碍物

开阔水域启动后，在 RViz 使用 **2D Pose Estimate** 点击并拖动，可在地图上添加一个圆形障碍；朝向被忽略。每次点击追加，障碍叠加在 `/free_water/base_map` 后发布为 `/map`，规划器、MPC 和 RViz 使用同一张地图；障碍在节点运行期间保留，重启节点后清空。格子与圆有交集时标为 100，实际栅格边缘受分辨率影响。

启动开阔水域并将障碍半径设为 3 m：

```bash
ros2 launch free_water_map fast_planner_open_water.launch.py obstacle_radius:=3.0
```

默认半径为 2 m，参数必须大于 0。独立运行可用：

```bash
ros2 launch interactive_water_obstacles circular_obstacles.launch.py radius:=2.0
```

独立运行时需要已有 `/free_water/base_map` 的可靠 transient-local `nav_msgs/OccupancyGrid`。默认监听 `/initialpose`、发布 `/map`；地图和点击消息必须同一坐标系，不会执行 TF 变换。重复发布的底图不会清除障碍；没有底图、坐标系不匹配或坐标无效时点击被拒绝。不要同时在 `/map` 发布另一张底图，否则消费者会收到相互覆盖的地图。2D Pose Estimate 也可能被定位系统订阅，在真实定位环境使用时应注意该话题的其他订阅者。
