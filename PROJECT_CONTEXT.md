# 项目上下文：当前MPC结构

更新日期：2026-10-15。依据活动源码、包声明、launch和YAML整理；历史修复资料保存在docs/history。

## 实际运行链路

```text
river_chart（Python XML/WKT解析）
  ├─ chart_costmap → local_costmap_generator → local_costmap
  ├─ chart_static_obstacles（原始陆地）
  └─ chart_lane_data → channel_navigation_manager → channel/lane_costmap

RViz goal_pose → 航道任务分类 + channel_astar_global_planner
全局A* → astar_channel_waypoints（完整航点）+ goal_pose_from_astar（当前航点）
当前航点/地图/odom/目标船状态 → Hybrid A* → 船模Path + 限速 + 会遇策略
Path/限速/地图/odom/策略 → MPC → cmd_wrench → boat_simulator → odom与TF
```

唯一跟踪器为MPC。包名、目录与头文件命名空间均为`usv_mpc_control`。
旧DWA节点、速度命令分支、空激光输出、旧包与RViz图层已删除；本船只有推力/偏航力矩接口。
目标船仍使用独立Twist控制话题；其接口与本船无关。

## 从哪里读代码

| 内容 | 文件 |
|---|---|
| 一键河道启动与节点连接 | src/channel_navigation_manager/launch/river_navigation.launch.py |
| 海图实际Python实现 | src/river_chart/river_chart/node.py、parser.py |
| 河岸地图膨胀 | src/local_costmap_generator/src/local_costmap_node.cpp |
| 任务航道锁存、中心代价 | src/channel_navigation_manager/src/channel_navigation_node.cpp、include/channel_navigation_manager/geometry.hpp |
| 全局搜索、稀疏航点、交接 | src/channel_astar_global_planner/src/channel_astar_node.cpp |
| Hybrid纯算法 | src/hybrid_a_star_planner/include/hybrid_a_star_planner/planner.hpp |
| 交接、迟到结果与绕圈核验 | src/hybrid_a_star_planner/include/hybrid_a_star_planner/path_utils.hpp |
| Hybrid ROS订阅、事件监测、后台worker与发布 | src/hybrid_a_star_planner/src/hybrid_a_star_node.cpp |
| 会遇分类与策略状态 | src/hybrid_a_star_planner/include/hybrid_a_star_planner/colregs.hpp、policy.hpp |
| 唯一共享船模 | src/hybrid_a_star_planner/include/hybrid_a_star_planner/usv_dynamics.hpp |
| MPC输入、预测优化与制动 | src/usv_mpc_control/src/mpc_controller.cpp |
| 力命令接收、超时制动、船模积分与odom | src/usv_mpc_control/src/boat_simulator.cpp |

`planner.hpp`的核心入口为`search → searchAtSpeed`；C弯连接为`connectMonotoneTurn`，宽弧为`refineBroadChannelCurve`，逐步安全检查为`primitiveIsSafe`。
算法测试直接包含该头文件，不再编译整个ROS节点入口。
MPC与仿真的`underactuated_model.hpp`仅适配共享模型名称和反馈控制接口，不重复实现状态方程。

## 必须保持的行为

- 航道角色按任务锁存，不随转艏、侧滑、跨道、越过目标反复改变。
- 同物理航段内30m短任务保持本分道为任务主航道；跨分道目标按任务方向分类。
- 后方另一分道目标优先向目标侧掉头；软偏好不能替代静态/动态硬安全。
- 高速失败后按默认10档从4m/s降至1.2m/s，保留真实初始u/v/r；二维引导构建不能耗尽后续速度预算。
- C弯是偏好，必要时允许安全的多弧机动；六状态搜索仍保留积分轨迹。2026-10-14用户明确授权新增几何参考优化，默认启用geometric_reference，允许偏离动力学积分形状；仍须通过曲率、位置/方向、衔接与安全检查，可关闭恢复仅船模输出。
- 新轨迹从实测船位开始，不拼接已航行旧前缀。航点交接时只保留实时复验过的安全旧路径。
- 后台规划不阻塞里程计；迟到结果必须接上当前实测状态，并重新检查安全。
- Path与速度限值按时间戳及坐标系匹配；缺输入/空路径/无解/取消/超时均制动。
- 中间航点经过与最终停车分开，低速机动后允许安全提速恢复。
- map/odom经有效平面TF转换；u/v/r保持船体坐标速度，不能伪造起点补偿求解耗时。

## 参数与运行

正常最高速度4m/s，最大艏摇率0.45rad/s；普通/低速/最紧转向尺度8/5/3m。
Hybrid总搜索预算3s、非最低速阶段0.12s；全局航点间距100m、末段最小间距20m、交接距离8m。
具体有效值以各包YAML为准；模型系数与上限需在规划、MPC和仿真配置中同步。

一键入口仍为`ros2 launch channel_navigation_manager river_navigation.launch.py`，无控制器选择开关。
已有运行实例须重启才加载新二进制。src为活动源码；build/install/log为生成产物；artifacts为验证结果。

## 历史与接续

上一版上下文误根据CMake副本将海图判断为C++；已核对实际package.xml和安装入口，活动实现为Python。
未接入运行的C++海图副本和旧代价地图备份已清理；模型对照实现仍被规则/几何回归使用，不作为本船跟踪分支。
清理前源文件可从`artifacts/code_cleanup/source_before_cleanup.tar.gz`恢复。历史说明、旧参数文档与已退役DWA测试配置归档在docs/history。
当前大量源码修改尚未提交Git；不要以旧提交或旧报告替代当前文件。已删除聊天原文无法从这些文件完整还原。
新版本清理与验证结果见[CODE_CLEANUP.md](/home/l/work_ws/docs/CODE_CLEANUP.md)。

## 2026-10-13 曲率反向与短机动调查

详见 [控制变量报告](artifacts/curvature_investigation/REPORT.md)。后20m左5m反向目标在开阔/航道/海图中复现3～5次地面曲率反向，原因包含C连接未充分使用现有30°容差、低速缩半径侧滑及局部控制目标缺少反向项。生产修改：C偏置覆盖现有容差；后方偏侧短机动的局部反向软惩罚；10～30m开阔/对向普通姿态任务增加连续船模连接。原2～10m近同轴机动保留原局部选择，避免惩罚影响可达性。带残余u/v/r=4/.6/.1的前20m同向15°任务，对向原3s超时，新增连接成功；其末端误差仍按现有1m/30°验收，并非精确艏向。158对照任务无新增失败，30°组131/131成功，70项单元测试、30条ROS节点路径与5项硬墙拒绝通过。5°人为严格容差仍有2条短机动失败。固定12m半径时不同速度稳态侧滑均13°，降速与缩半径必须分开解释。未测试本次完整MPC闭环或实船。

## 2026-10-14 仅统一Hybrid终点艏向容差

用户明确取消后续航点方向引导，此次仅统一终点艏向容差为30°（π/6rad）。配置原本已为30°，现补齐规划器内部默认值（原0.22rad）和USV_KINEMATIC/CAR_LIKE格点验收（原硬编码0.15rad），ROS默认值引用同一常量。未增加后续航点预览、未调整当前航点参考姿态或位置约束。终点验收专项覆盖所有模型、29°接受/31°拒绝、±180°角度环绕；规划器66项及COLREG5项回归全通过。已构建并安装；正在运行的旧节点须重启加载新可执行文件。日志见artifacts/heading_tolerance_30。

## 2026-10-14 扩大半径与几何尾段优化、TF显示

用户明确允许几何优化暂时不符合完整动力学，目标为减少多次S摆动、减小曲率。新增path_geometry.hpp（统一空间采样、地面曲率指标、主转弯尾端识别）和planner.optimizeTrackingPath；ROS后台搜索后默认启用几何参考优化：更大半径Dubins，必要时保留主弯仅整形尾段。整体/被修改尾段最大曲率不增，均方曲率降低，位置与30°终点方向检查保留；无候选或无法实时安全接续可退回同次原积分轨迹。参数geometric_reference.enabled/preferred_radius/max_deviation/time_limit默认true/8m/12m/0.15s；后续A*点方向预览仍关闭。河道一键RViz TF已Enabled、Marker Scale=20、显示坐标轴和名字。96对照无新增失败，原多次反向降至1次；78项单元回归、40条ROS发布/限速和5项硬墙拒绝通过。MPC对照两组70s都未完成到达，优化后RMS略增，不能宣称整体闭环跟踪已改善；本次未改控制器或船模。详见artifacts/curvature_c_optimization/REPORT.md。


## 2026-10-14 1014v2 主航道入道与姿态矩阵

本轮仅修改/home/l/1014v2活动源码；/home/l/work_ws保留实验暂存文件，不能混用两工作空间install。
新MainLaneDistanceField提供MAIN局部最近方向与距离；新channel.main_entry_weight/direct_crossing_distance/main_entry_horizon默认30/30m/30m。搜索覆盖前方/侧方、小航向差入道，左右镜像切线偏置，限时入道连接及原搜索软竞争；后优化加入MAIN起段距离代价。安全检查、任务角色锁存、30°终点容差、MPC与船模保持。
二维缓存必须包含入道状态和起始位置；配置改变也失效，避免重规划沿用旧偏好。节点新增debug级Planning snapshot，记录实际船位/艏向/目标/u/v/r。
用户明确要求以后所有轨迹优化重复20m、目标方位×目标姿态每15°的24×24矩阵，开阔/主/对向共1728格，每格必须有图，失败必须保留。持久要求见src/hybrid_a_star_planner/AGENTS.md；可复用工具为test/run_pose_matrix.py及BUILD_TESTING目标pose_matrix_probe。
本轮数据、对照、全部矩阵和图见artifacts/lane_entry_1014v2/REPORT.md与pose_matrix/index.html。部分更早入道路线的曲率比旧的大外绕更大，不能声称所有指标都下降；后优化的“不增曲率”是相对同次积分原轨迹。仅规划/ROS发布验证，未完成MPC全程闭环或实船验证。

最终矩阵已校正测试夹具：姿态输入按ROS TF规范到[-π,π]，正侧方坐标余量归零；列名仍0..345°，30°容差未变。新入道终点容差候选顺序按请求转向与MAIN侧别镜像。
