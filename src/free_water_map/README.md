# 开阔水域演示

提供独立地图和会遇初始条件；演示入口同样只启动MPC和力控制船模。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：src/free_water_costmap.py：地图；src/demo_goal_publisher.py：一次性目标；launch/colregs_demo.launch.py：会遇场景。

主要消息关系：`/local_costmap、/goal_pose、目标船状态`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/free_water_map/`。
