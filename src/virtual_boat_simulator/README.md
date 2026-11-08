# 目标船仿真

产生独立目标船里程计、船体显示与可控运动，作为会遇测试输入。目标船的Twist控制话题与本船执行器输入独立。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：src/virtual_boat_simulator.cpp：目标运动、重定位、状态及Marker。

主要消息关系：`/target_boat/command → /target_boat/odom`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/virtual_boat_simulator/`。
