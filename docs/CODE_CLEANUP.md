# MPC结构精简与验证

日期：2026-10-11。当前一键入口保持`river_navigation.launch.py`，本船只使用MPC。

## 活动代码变化

| 清理前 | 当前 |
|---|---|
| dwa_and_ship_sim目录、dwa_deep包/命名空间 | usv_mpc_control目录、包与命名空间 |
| DWA与MPC互斥启动开关 | 唯一MPC节点，无use_mpc兼容选择 |
| DWA速度参考与MPC力命令双接口 | 本船只接收cmd_wrench，缺失/超时制动 |
| 仿真器空scan输出、旧拼写起点参数、无实际选择作用的model_mode | 移除占位输出和未使用参数 |
| 两份重复船体状态方程/RK4 | 规划、MPC、仿真共用usv_dynamics.hpp；控制头文件只保留适配 |
| Hybrid算法与ROS主函数混在约3700行cpp中 | planner.hpp存算法，path_utils.hpp存路径核验，约1200行节点cpp负责接线与后台事件 |
| 永远false的旧轨迹拼接分支及无效失败保留参数 | 删除；每段从实测状态起步，保留真正的航点交接连续性 |
| 旧代价地图831备份 | 从活动源码删除 |
| 海图Python包与未被构建的C++副本并存 | 保留实际运行的Python包，删除未接入C++副本与对应未注册测试 |
| 重复map→odom静态TF、DWA RViz图层 | 单一坐标桥接与MPC显示 |
| 活动目录中的历史修复/对照报告工具 | docs/history归档；当前说明和配置索引重新整理 |

旧控制包的build/install、失效安装链接和旧算法测试二进制也已清理，ROS包索引只保留usv_mpc_control。
生产源码、YAML、launch中无DWA控制入口；测试中的cmd_vel只用于证明旧接口不存在。

活动C++/头文件/Python源码由68个文件、17880行变为54个文件、12975行，净减少4905行（27.4%）。
数值按清理前快照和当前src统计；历史归档与压缩备份不计入活动源码。
保留了正在用于规则/几何回归的数学对照模型、有效船模、避碰检查与任务测试；它们不是本船跟踪器分支。

## 额外接线修正

开阔水域演示从旧跟踪器改接MPC后，补齐了：

- Python地图与一次性目标脚本的执行权限。
- MPC的直接最终目标模式：global_route_topic为空时不创建无效订阅，活动目标就是最终停车目标。
- 河道模式仍通过完整航点路线区分经过点和最终停车点。
- 旧依赖cmd_vel的固定路线检查改为检查MPC制动推力。

未更改船模系数、硬安全距离或规则约束来迎合测试。共享模型用清理前的转向/制动轨迹数值验证，误差门槛1e-12。

## 验证结果与范围

- 全工作区8个包编译通过；当前Python、YAML与RViz配置语法检查通过。
- 新鲜注册测试97项通过：规划62、COLREG 5、航道9、MPC路径辅助6、坐标输入2、共享模型1、Python海图12。数字按本次生成的测试XML统计，不将CTest外层汇总或其他历史结果重复计入。
- MPC输入/TF回归通过：缺TF制动、迟到TF恢复、空planning/blocked路径制动、有效轨迹恢复。
- 实际船模接口回归通过：推力命令驱动、无旧速度订阅、无空激光输出、缺失/超时命令制动。
- 对向航道ROS回归通过：12个10/20m短任务、3个后方换道目标、Path/限速配对及硬墙拒绝。
- 无近邻冲突的开阔水域启动回归通过：MPC唯一跟踪器、Path/限速配对、实际运动与执行器限幅。
- 直接目标闭环停车通过：终点误差约0.318m，末速度接近零，终点明显反向力矩切换1次，单次力矩变化不超过0.20。
- 实际河图全局A*→Hybrid→MPC→船模的20m右向机动通过：最后复测参考速度1.2m/s，轨迹约20.07m，起始直段约0.94m，艏向误差约0.465rad，符合当前0.524rad容差；后续短时闭环验证正常。

### 明确保留的未通过项

默认head_on演示能够启动MPC链路并产生初始轨迹，但12秒持续推进检查未通过：实际位移仅约1.38m，随后出现无安全MPC序列/规划blocked并制动。
此项不作为完整对遇避碰任务通过，不降低安全检查或放宽推进断言来掩盖它。
对应启动与blocked制动单独验证，结果记录在测试日志中。原始推进失败日志永久保留，完整对遇性能仍需另行分析。

## 产物与复测

`artifacts/code_cleanup/`保存清理前源码快照、源码规模统计、删除/生成物清单、单元结果、ROS日志及实际河图轨迹。
当前项目结构见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)，旧修复资料见[docs/history](/home/l/work_ws/docs/history)。

```bash
source /home/l/work_ws/install/setup.bash
colcon test --packages-select hybrid_a_star_planner usv_mpc_control channel_navigation_manager river_chart
python3 src/usv_mpc_control/test/test_mpc_readiness_ros.py
python3 src/usv_mpc_control/test/test_simulator_timeout_ros.py
python3 src/usv_mpc_control/test/test_mpc_terminal_ros.py --direct-goal
python3 src/hybrid_a_star_planner/test/test_opposite_lane_ros.py
python3 src/free_water_map/test/test_mpc_demo_ros.py
python3 src/free_water_map/test/test_mpc_demo_ros.py --scenario head_on
```

上述ROS脚本使用各自隔离域；已有导航实例需要重启才能加载新包与二进制。
