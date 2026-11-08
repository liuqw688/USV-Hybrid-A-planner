# 全局航点规划：代码导航

二维A*生成稀疏航点，根据船位顺序交接当前目标。新任务撤销旧路线；末段保留用户目标艏向。

src/channel_astar_node.cpp：plan/searchCells搜索，switchWaypoint交接，publishActiveGoal发布。

消息关系：`/goal_pose、/channel/lane_costmap、/local_costmap、/odom → /astar_channel_waypoints、/goal_pose_from_astar`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。
