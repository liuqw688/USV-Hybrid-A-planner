# 目标船仿真：代码导航

产生独立目标船里程计、船体显示与可控运动，作为会遇测试输入。目标船的Twist控制话题与本船执行器输入独立。

src/virtual_boat_simulator.cpp：目标运动、重定位、状态及Marker。

消息关系：`/target_boat/command → /target_boat/odom`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。
