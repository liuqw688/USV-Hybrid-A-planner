# 前端移植与无人船环境适配记录

> 本文件保留首次独立移植阶段记录；后续新增了与现有MPC/本船仿真的闭环入口，当前接口和依赖见 [CLOSED_LOOP_TRACKING.md](CLOSED_LOOP_TRACKING.md)。

日期：2026-10-31。来源工作空间 `/home/l/Fast-Planner-ROS2`，目标工作空间 `/home/l/1015v2`，新增包 `usv_kinodynamic_frontend`。用户要求“接口暂时不用修改”，本轮保留所有已有导航包、话题、消息、MPC、仿真与启动链路；新包提供独立库及离线演示，未自动切换运行中的规划器。

## 代码来源

`PORT_SOURCES.json` 记录移植源路径、原文件 SHA256、完成名称/命名空间转换后的初始 SHA256。随后目标包独立增加水域适配，故最终哈希可能与初始移植哈希不同；本文件末尾记录最终文件哈希所在位置。

| Fast-Planner源文件 | 新包文件 | 移植处理 |
|---|---|---|
| `path_searching/include/path_searching/usv_model.h` | `include/usv_kinodynamic_frontend/usv_model.hpp` | 保持全部11参数和公式；移除原包路径/命名空间依赖 |
| `path_searching/include/path_searching/usv_kinodynamic_astar.h` | `include/usv_kinodynamic_frontend/kinodynamic_astar.hpp` | 运动基元/六状态类型、分辨率、代价、终端容差和结果保持；新增可选环境代价函数 |
| `path_searching/src/usv_kinodynamic_astar.cpp` | `src/kinodynamic_astar.cpp` | 保留节点池/哈希/不可变优先队列、RK4与数值终端连接；新增沿时间积分的非负环境代价 |
| `path_searching/test/usv_search_test.cpp` | `test/model_search_test.cpp` | 保留模型与搜索回归，并在目标包重新编译执行 |
| 无 | `waterway_map.hpp`、`occupancy_grid.hpp` | 新增二维水域地图、船体相交检测与原OccupancyGrid转换助手 |
| 无 | `src/frontend_demo.cpp` | 新增离线算法入口，输出含时间、六状态与两命令的JSON；不访问ROS话题/执行器 |
| 无 | `test/waterway_map_test.cpp` | 新增水域碰撞、坐标、地图转换和环境代价回归 |

新包不引用原工作空间的绝对路径，也不依赖 `path_searching/plan_env/bspline/bspline_opt/traj_utils/plan_manage`。纯算法仅依赖Eigen；已有OccupancyGrid转换助手使用ROS标准nav_msgs。包内LICENSE沿用来源仓库的GPLv3文件，原项目LICENSE文件未修改。

## 从无人机前端到无人船前端：修改前 → 修改后

| 方面 | 原Fast-Planner | 本轮移植版本 |
|---|---|---|
| 六维状态 | 三维位置+世界速度 | x/y/船体航向+船体u/v/r；相同维度不等于相同物理意义 |
| 控制 | 三轴加速度 | 两路v_cmd/w_cmd；不等于实际u/r |
| 传播 | 三轴双积分器闭式解 | 用户给出的带偏置、横荡/偏航耦合模型与固定最大步长RK4 |
| 离散 | 位置体素主导 | 六个状态全部入哈希；航向周期分桶 |
| 启发式 | 双积分器四次方程 | 平面距离和最短航向差的时间下界；加权A*不保证最优 |
| 终端连接 | 三次多项式shot | 多段恒定船舶命令的数值射击，整段重积分验算；检查全部六状态 |
| g代价 | 加速度能量+时间 | 归一化命令能量+时间+可选非负岸边软代价积分 |
| 地图 | 三维SDF | 二维栅格，支持旋转原点；psi与地图高度彻底分开 |
| 船体安全 | 点/膨胀点查询 | 完整圆形足迹与格子相交，包含船体内部，增加净空；未知与地图外默认拒绝 |
| 输出 | 原搜索路径与内部节点 | 时间+六状态+两命令完整采样；失败/局部视野/终点状态明确区分 |
| 后端 | UAV B样条优化 | 本包仅前端；Fast-Planner侧另有统一模型下的B样条验证，没有冒充已移植后端 |

## 与1015v2现有模型的边界

现有 `hybrid_a_star_planner/usv_dynamics.hpp` 使用 `tau_y/tau_n` 力输入，纵向包含 `r²` 项，系数与这次用户给出的速度命令辨识模型不同。MPC和本船仿真共用现有力输入模型；当前执行器接口为 `/cmd_wrench`。

新包的11参数模型是用户本轮明确要求，按原值移植，**没有改名后将它当作原力模型使用**。当前3条边界明确保留：

1. 不替换原共享船模，不修改MPC或仿真参数，不把v_cmd/w_cmd写成力/力矩。
2. 不在原导航启动链路插入新规划器，不向原控制器发布新路径/命令。
3. 原环境常用起点u=4m/s；新包默认max_u=1.6m/s是未标定工程值。范围外起点返回失败，不静默裁剪。后续接入必须先统一辨识工作范围和控制语义。

用户要求先不改接口，故本轮不增加状态/控制消息、不建立订阅发布节点。已有OccupancyGrid转换仅为库助手；调用方仍必须保证规划状态与地图同一frame、有效平面TF、及时里程计。旧包的航道角色锁存、动态船、COLREG和几何参考优化继续保留在旧包，尚未移植到新包。不能由本包离线规划成功推断原MPC闭环已可用。

## 水域环境约定

- 按现有局部代价图习惯，100为陆地/障碍，99为最低硬缓冲，-1为未知；新包默认lethal_threshold=88，不清除这些硬层。
- 可通过WaterwayConfig调整阈值，但软代价不能覆盖硬安全。直接使用已膨胀的local_costmap再加船体足迹会较保守，这是明确的净空叠加，不应通过清除陆地“修复”。
- 原地图岸边/航道软层有独立语义，新包不猜测或重新分类；只使用调用方给出的单张栅格及非负代价。未读取任务主/对向航道语义。
- 固定二维平面，不处理潮汐、吃水、风浪流或动态障碍；安全检查是按步长采样的模型内验证，不是连续时间/实船证明。
- 默认命令预留余量command_scale=0.85，来自Fast-Planner侧发现的样条切换超限问题；本包输出仍是完整的可积分前端轨迹。

## 验证及生成产物

仅构建新包，不重编译或替换原Hybrid/MPC。

- 模型/搜索：11参数、解析响应、RK4收敛、航向周期、六状态终端连接、逐段模型一致性、障碍绕行、局部视野、超时与节点池耗尽。
- 水域：未知/99/100硬拒绝、软代价保留、中心之外的船体碰撞、地图边界、旋转地图、OccupancyGrid转换与非平面拒绝、4m/s范围外起点拒绝、环境代价计入g。
- 离线演示：完整采样保存JSON；无后台ROS节点、无控制器或执行器访问。

原有活动源码的移植前清单、测试日志、演示结果和本包最终哈希放在：

`/home/l/1015v2/artifacts/usv_kinodynamic_frontend_20261031/`

其中 `existing_sources_before.sha256` 用来确认所有旧包活动文件未被修改。`new_package_sources.sha256` 记录新包最终源文件（不包括此清单自身）。本轮未修改Hybrid包，因此没有将它的历史20米姿态矩阵结果当成新包的验证；新模型不同，未声称已经覆盖1728格姿态矩阵。

## 本轮实际验收结果

- 仅新包Release构建成功，不依赖Fast-Planner工作空间。
- CTest：2项测试全部通过；模型/搜索1325项断言，水域地图/规划318项断言。
- 离线开阔水域示例：`reach_end`，6秒、301个六状态/两命令采样点，数据见`open_water_result.json`。
- 原有142个活动源码文件的SHA256复核全部一致。
- 未进行原MPC闭环、实船、动态船会遇或全姿态矩阵验证。
- 独立调用方验证：另建最小CMake工程，通过安装后的`find_package(usv_kinodynamic_frontend)`成功编译、链接和执行；日志见`independent_consumer.log`。
