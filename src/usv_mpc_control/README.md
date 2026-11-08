# MPC跟踪与无人船仿真

唯一跟踪控制器为MPC。默认`dynamics_model=force`沿用纵向推力和偏航力矩，通过`/cmd_wrench`控制原船模。

空旷前端测试入口现选择`dynamics_model=identified_velocity_commands`：使用用户提供的六状态11参数辨识方程，MPC与仿真通过`/cmd_vel`传递`v_cmd/w_cmd`，实际`u/v/r`由RK4积分，不直接赋成命令。模型系数及积分实现在`include/usv_mpc_control/identified_dynamics.hpp`，同时供移植前端调用。等待、停车和超时命令包含偏置补偿。该模式与原力输入模式的执行话题分开。

一键使用及本轮修改前后记录见[辨识模型闭环说明](/home/l/1015v2/src/usv_kinodynamic_frontend/docs/IDENTIFIED_MODEL_TRACKING.md)。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：src/mpc_controller.cpp：输入转换、路径参考、滚动优化、制动；src/boat_simulator.cpp：船模积分与状态发布；include/usv_mpc_control/：坐标转换、路径辅助、共享船模适配。

主要消息关系：`轨迹、限速、地图、/odom、策略 → /cmd_wrench → /odom、TF`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/usv_mpc_control/`。
