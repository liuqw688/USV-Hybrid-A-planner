# 全局航点规划

二维A*生成稀疏航点，根据船位顺序交接当前目标。新任务撤销旧路线；末段保留用户目标艏向。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：src/channel_astar_node.cpp：plan/searchCells搜索，switchWaypoint交接，publishActiveGoal发布。

主要消息关系：`/goal_pose、/channel/lane_costmap、/local_costmap、/odom → /astar_channel_waypoints、/goal_pose_from_astar`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/channel_astar_global_planner/`。
