# 船模轨迹规划与动态避碰

根据当前航点生成欠驱动船模轨迹。按速度阶段降速、尝试C弯/宽弧，检查静态/动态安全与COLREG约束；事件触发后台搜索。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：include/hybrid_a_star_planner/planner.hpp：搜索、代价与连接；path_utils.hpp：交接和迟到路径核验；path_geometry.hpp：地面曲率与几何辅助；policy.hpp/colregs.hpp：会遇策略；usv_dynamics.hpp：共享船模；src/hybrid_a_star_node.cpp：ROS输入、后台调度与发布。

主要消息关系：`/goal_pose_from_astar、地图、/odom、目标船状态 → /hybrid_a_star_trajectory、限速、策略`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/hybrid_a_star_planner/`。

### 曲率反向与短距离姿态调整验证

2026-10-13的[控制变量调查](../../../artifacts/curvature_investigation/REPORT.md)记录了开阔水域、主/对向航道和原始海图裁剪中的前后对照。C形连接在原终点艏向容差内补充候选；后方偏侧短机动增加局部反向软代价；10～30m的开阔/对向普通姿态任务增加连续船模连接。仍保留必要多弧机动和原硬安全检查。

2026-10-14新增独立几何跟踪参考优化（`geometric_reference.enabled`默认true）：搜索仍使用共享船模，后处理尝试更大半径C弯或减少尾段反向。允许参考形状偏离动力学积分，但最大曲率、被修改尾段曲率不增加，保留终点位置/30°方向容差及安全复验；失败可退回原积分路径。详细前后对照和MPC未完成到达的限制见[测试报告](../../../artifacts/curvature_c_optimization/REPORT.md)。
