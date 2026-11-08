# 开阔水域演示：代码导航

提供独立地图和会遇初始条件；演示入口同样只启动MPC和力控制船模。

src/free_water_costmap.py：地图；src/demo_goal_publisher.py：一次性目标；launch/colregs_demo.launch.py：会遇场景。

消息关系：`/local_costmap、/goal_pose、目标船状态`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。
