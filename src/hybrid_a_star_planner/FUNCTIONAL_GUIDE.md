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


## 主航道入道与全姿态矩阵回归

`lane_guidance.hpp`只把已锁存的MAIN语义栅格变成空间距离与最近方向，不参与航道角色分类，也不替代安全地图。
`planner.search`根据起终点角色启用入道软偏好，限时试连续入道候选，并保留相同总预算内的原始安全搜索竞争候选。短主→对向目标不强制多绕主航道。
`optimizeTrackingPath`以同一入道空间指标比较几何候选，原位置、30°方向、曲率和安全验收保持；必要的S仍允许存在。
二维启发缓存记录MAIN入道状态和起始船位，防止同一目标在不同船位重规划时复用旧区域代价。节点增加debug级Planning snapshot日志，记录实际起点/目标艏向及u/v/r，便于复现场景。
新增手动`pose_matrix_probe`和`test/run_pose_matrix.py`：固定20米、每15°测试目标方位与目标艏向，三环境各576格，输出每格原/最终CSV与SVG、24×24矩阵PNG、每环境24页PDF和HTML索引。完整矩阵不加入快速ctest，轨迹修改后必须单独运行。

测试夹具按ROS TF规范角度并消除正侧方的数学残差；终点箭头显示输出参考姿态，不是闭环实船结果。新入道候选在原30°容差内按请求转向和MAIN侧别安排姿态样本，保持左右镜像。


`include/hybrid_a_star_planner/maneuver_shooting.hpp`：双弧及Dubins种子、三段时长/转向幅度求解、六状态积分。`planner.hpp::rescueShooting`：同预算调度和原primitiveIsSafe复核；地图、动态船与规则仍优先。矩阵探针额外保存 *_states.csv 和 *_controls.json。
