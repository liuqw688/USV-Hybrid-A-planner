# MPC跟踪与无人船仿真：代码导航

唯一跟踪控制器为MPC，直接输出纵向推力和偏航力矩。仿真器仅接受力命令，缺失或超时制动。

src/mpc_controller.cpp：输入转换、路径参考、滚动优化、制动；src/boat_simulator.cpp：船模积分与状态发布；include/usv_mpc_control/：坐标转换、路径辅助、共享船模适配。

消息关系：`轨迹、限速、地图、/odom、策略 → /cmd_wrench → /odom、TF`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。
