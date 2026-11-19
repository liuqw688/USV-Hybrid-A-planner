# 船模轨迹规划与动态避碰：代码导航

根据当前航点生成欠驱动船模轨迹。按速度阶段降速、尝试C弯/宽弧，检查静态/动态安全与COLREG约束；事件触发后台搜索。

include/hybrid_a_star_planner/planner.hpp：搜索、代价与连接；path_utils.hpp：交接和迟到路径核验；path_geometry.hpp：地面曲率与几何辅助；policy.hpp/colregs.hpp：会遇策略；usv_dynamics.hpp：共享船模；src/hybrid_a_star_node.cpp：ROS输入、后台调度与发布。

消息关系：`/goal_pose_from_astar、地图、/odom、目标船状态 → /hybrid_a_star_trajectory、限速、策略`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。

2026-10-14新增独立几何跟踪参考优化（`geometric_reference.enabled`默认true）：搜索仍使用共享船模，后处理尝试更大半径C弯或减少尾段反向。允许参考形状偏离动力学积分，但最大曲率、被修改尾段曲率不增加，保留终点位置/30°方向容差及安全复验；失败可退回原积分路径。详细前后对照和MPC未完成到达的限制见[测试报告](../../../artifacts/curvature_c_optimization/REPORT.md)。
