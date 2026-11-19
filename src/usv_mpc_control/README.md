> 当前版本使用 Fast-Planner 六状态船模；控制话题为 `/usv/cmd_velocity`（Twist：linear.x 为 v_cmd，angular.z 为 w_cmd），范围分别为 [0, 2]、[-0.3, 0.3]。以下历史推力/力矩及 Hybrid 船模说明不适用于当前控制器和仿真器。零指令不能保证静止。

# MPC跟踪与无人船仿真

唯一跟踪控制器为MPC，直接输出纵向推力和偏航力矩。仿真器仅接受力命令，缺失或超时制动。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：src/mpc_controller.cpp：输入转换、路径参考、滚动优化、制动；src/boat_simulator.cpp：船模积分与状态发布；include/usv_mpc_control/：坐标转换、路径辅助、共享船模适配。

主要消息关系：`轨迹、限速、地图、/odom、策略 → /cmd_wrench → /odom、TF`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/usv_mpc_control/`。
