# 任务航道管理与整链路启动：代码导航

按点击时起点锁存任务，用中性航道邻接关系确定河道前进方向，匹配海图 ORIENT 并传播主航道；短任务不改变规定通航方向。拓扑断开或没有匹配流向时发布 UNKNOWN。

src/channel_navigation_node.cpp：任务输入与发布；include/channel_navigation_manager/geometry.hpp：分类、传播与栅格化；launch/river_navigation.launch.py：启动完整MPC导航链路。

消息关系：`/chart_lane_data、/odom、/goal_pose → /channel/lane_costmap`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/orangepi/1014v1/PROJECT_CONTEXT.md)。
