# 任务航道管理与整链路启动

按点击时起点锁存任务，用中性航道邻接关系确定河道前进方向，匹配海图 ORIENT 并传播主航道；短任务不改变规定通航方向。拓扑断开或没有匹配流向时发布 UNKNOWN。

入口、模块关系和当前运行方式见[项目说明](/home/orangepi/1014v1/README.md)。

核心实现：src/channel_navigation_node.cpp：任务输入与发布；include/channel_navigation_manager/geometry.hpp：分类、传播与栅格化；launch/river_navigation.launch.py：启动完整MPC导航链路。

主要消息关系：`/chart_lane_data、/odom、/goal_pose → /channel/lane_costmap`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/channel_navigation_manager/`。
