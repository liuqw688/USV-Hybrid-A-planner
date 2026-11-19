# Fast-Planner 无人船改造记录

## 项目原则与当前决策

- 保留 Fast-Planner 整体框架，在该框架上逐模块、逐功能改造。
- 用户决定具体修改路线；先检查逻辑与接口衔接，发现问题及时提醒并给出建议。
- 每次改完追加记录，并向用户解释每个改动的位置、原因、做法和与此前改动的关系。
- 不在用户本地电脑编译此工程；采用源码核查、静态检查或不编译工程的数值检查。
- 原始参考方案见 [USV_DESIGN_REFERENCE.md](USV_DESIGN_REFERENCE.md)，其建议不等于已确定决策。
- 船模参考：状态 `[x,y,psi,u,v,r]`，输入 `[v_cmd,w_cmd]`；11 个辨识系数见参考方案。原始 PDF 与辨识数据尚未在本轮独立核验。
- 后端总体 A/B 路线、B 中的状态曲线约束或控制输入 B 样条仍待用户确定；终端连接按记录 021 改为四段恒定命令数值射击。
- 已完成传播函数的第一步改造：保留 `Eigen::Matrix<double, 6, 1>`，在传播函数中解释为 `[x,y,psi,u,v,r]`；保留 `Eigen::Vector3d` 输入，仅使用 `[v_cmd,w_cmd]` 两项，第三项预留；保留 `stateTransit()` 名称。按记录 005，RK4 步数由 `search/rk4_steps` 读取，声明默认 -1，启动配置为 10，非正步数在传播时显式报错。详见记录 002、005。
- 启发式已采用 `w_time * max(d / V_max, abs(wrap(psi_goal-psi)) / r_max)`，其中 `d` 只取二维位置距离。用户先前指定 `V_max=1.5 m/s`、`r_max=0.3 rad/s`，按记录 026 将 `V_max` 改为 `2.0 m/s`；实际数值由启动配置提供；按记录 004，`declare_parameter()` 的默认值统一为 `-1.0`，非法参数时回退 `h=0`。按记录 006，`max_ground_speed_`、`max_yaw_rate_` 和 `rk4_steps_` 在头文件中只声明，不提供成员初值，统一由 `setParam()` 赋值，使用前必须先调用该函数。详见记录 003、004、006。
- 搜索与后端尚未整体接通；普通扩展的碰撞与速度约束、目标输入及后端等仍需后续修改；控制采样按记录 023 改为无人船两路命令。射击已按记录 021 改造，但未运行验证。此说明不是对工作区全部既有修改的审计。
- 按记录 007，管理器与搜索器的第三输入已改为 `start_cmd`，含义 `[v_cmd,w_cmd,unused]`。按记录 013，使用 `computeUsvStateAndCommand()` 一次接收参考系位姿及其一、二阶导数，同时输出船体速度和反算命令，管理器第三实参为局部 `start_cmd`。旧的 `computeUsvCommand()` 已合并替换；横向一致性与命令限幅仍待处理。
- 用户在记录 009 明确决定将位姿三分量解释为 `[x,y,phi]`；此决定优先于原参考方案中的分离表示建议。但地图、混合单位代价、目标输入、普通采样和整个后端并未因此自动完成适配，仍按用户指定逐模块推进。
- 用户已确认里程计线速度输入为世界坐标。按记录 013，`odom_vel_` 恢复保存 `[dot_x,dot_y,r]`，第三项仍取 `angular.z`；首次和重规划的 `start_vel_` 统一为参考系位姿一阶导数，`start_acc_` 为参考系位姿二阶导数（首次为零假设），转换函数不再依赖 `GEN_NEW_TRAJ/REPLAN_TRAJ` 分支。Kino 独立偏航路径已移除，消息定义和其他消费者暂不修改。
- 规划入口使用参考系位姿 `[x,y,phi]`、转换后的船体速度 `[u,v,r]` 与反算命令 `[v_cmd,w_cmd,0]`，匹配现有传播器；动力学方程和 RK4 保持不变。原始状态机成员保持参考系导数含义；按记录 015，转换结果保存到同一数据区中的 `start_body_vel_`、`start_cmd_`、`end_body_vel_` 成员，每次转换赋值后再传给管理器。
- 按记录 014，`kinodynamicReplan()` 与 `search()` 的起终点速度形参统一命名为 `start_body_vel`、`end_body_vel`，明确都是船体 `[u,v,r]`，类型仍为 `Eigen::Vector3d`。
- 按记录 016，`computeUsvStateAndCommand()`、`computeBodyVelocity()` 已从 Kino 状态机迁入船模所在的 `kinodynamic_astar.h/.cpp`，作为 `KinodynamicAstar` 公共静态函数；新增 `computeGlobalVelocity()` 输出 `[vx,vy,r]`。速度向量第三项始终是角速度 r，航向 phi 只从位姿第三项读取参与旋转。
- 记录 017 曾将 `search()` 保存的 `start_vel_` 改为参考系速度；记录 018 时源码变回船体系。按记录 021，因采样衔接需要已再次使用 `computeGlobalVelocity()` 保存参考系速度。
- 按记录 018，`posToIndex([x,y,phi])` 使用 0.25 米位置分辨率和每 5° 一个的 72 个周期航向区间，`search/yaw_bins` 为可配置数量。按记录 020，局部范围和近终点判断已改为连续二维位置和周期航向误差；普通扩展的地图碰撞查询等其余第三维逻辑仍需后续适配。
- 记录 019 的时间索引衔接修改已由用户改回；当前源码仍存在动态哈希插入连续时间、静态搜索读取未初始化时间字段的问题，时间分辨率仍为 0.8 秒。记录 019 作为历史保留，不能将其描述为当前代码状态。
- 按记录 020，ROS 2 启动配置中 `horizon=50 m`，近终点位置容差 `25 m`，航向容差 `60°`；`near_end` 触发条件保持不变；按记录 021，Shot 已改为四段恒定命令数值射击，但整体规划链尚未验证。

- 按记录 056，ROS2 Kino 默认接收 `/map` 的 OccupancyGrid，地图、里程计和目标暂统一为 map 坐标系，不启用 TF 转换。SDFMap 保留原查询接口，静态模式使用真实二维存储、占据膨胀、带符号距离和二维梯度；尺寸/分辨率/原点来自地图消息，可接收 500m x 500m 地图。未知默认障碍，地图外不可通行；先收到地图并生成距离场再规划。详情见 [USV_GRID_MAP.md](USV_GRID_MAP.md)。原点云/深度三维分支保留，但关闭 grid 模式后仍需核对无人船第三项语义。

- 按记录059，增加/map静态底图与/local_map传感器栅格融合，均在固定map坐标系，无TF。以实际船位为中心，x/y各±50m（100m方形）局部写回占据/膨胀/距离，窗口外保留历史；距离采用用户确认的±10m饱和。静态障碍保留，局部空闲可清旧局部障碍，未知不清旧值。首次/底图变化全图初始化，后续局部更新；输入地图、输出融合地图和更新补丁分开。新增map_msgs依赖，接入说明见USV_GRID_MAP.md。

## 开始仿真测试时提醒

- 前端、后端改造完成并准备仿真时，先核对记录 056 的静态栅格接入：地图/里程计/目标实际均在 map、QoS、原点与分辨率、未知/越界、地图就绪以及占据/距离梯度一致性；测量大范围地图的 C++ 内存、首次全图和后续100m窗口更新耗时，核对局部图融合、清除、时间戳和全图快照/补丁显示。静态模式查询已忽略 phi，旧三维模式仍有第三项语义待办。船体长宽与转向扫掠尚未实现，当前膨胀按记录057设为3m，仍不是按船体航向轮廓计算的精确安全尺寸。
- 提醒评估 B 样条优化求解预算：当前 Kino 使用 `max_num_id=1`、`max_time_id=1`，对应 `optimization/max_iteration_num2=300`、`optimization/max_iteration_time2=0.005 s`。当前无实际测试依据，保持参数。仿真时记录目标函数计算次数、优化耗时、求解器停止原因、控制点数量及最终可行性，再决定调整次数和时间；不要预先假定 5 ms 一定够用或一定不足。

## 换电脑后继续工作的方式

1. 携带整个工程，包括根目录 `AGENTS.md`、本记录和原始方案副本。
2. 先阅读本记录，核对实际代码、分支及本地未提交修改，再继续工作；文档不能代替源码核查。
3. 构建环境参考根目录 `BUILD.md` 和 `README.md`；本轮未验证在其他电脑上的构建。
4. 每次新增决策同步更新本节的当前决策，并在下面追加对应修改记录。

## 2026-11-05 · 001 · 建立改造记录与交接规则

### 用户要求

每次修改后记录内容，以便更换电脑继续；整体沿用 Fast-Planner 框架，只在其上做功能性修改。

### 文件与修改原因

- `AGENTS.md`：新增项目协作约定，让后续进入项目的助手先读取记录并遵守逐项修改要求。
- `docs/USV_MODIFICATION_LOG.md`：新增本记录，保存当前决策、交接步骤和后续追加格式。
- `docs/USV_DESIGN_REFERENCE.md`：原样复制用户提供的方案附件，避免换电脑后丢失对话附件路径。保留原文中的建议、引用和待定内容，不表示这些内容已独立核验或已决定实施。

### 与此前工作的衔接

此前已解释 ROS 2 前端和后端，并阅读用户改造方案；本次将协作要求和参考材料保存在项目内，未修改算法、参数、接口或运行逻辑。

### 验证结果

文档创建后检查文件存在、协作规则引用路径有效、方案副本与附件字节一致。此次仅改文档，不运行算法测试或编译。

### 待定与下一步

等待用户指定第一个具体功能；不预先选定后端或终端连接路线。

## 后续记录格式

每次追加独立条目：日期与编号、用户要求、涉及文件/函数、具体修改与原因、与此前改动及上下游的衔接、验证命令或方法与结果、剩余问题、用户已确定及仍待确定的决策。必要时附参数、公式和兼容性说明。

## 2026-11-05 · 002 · 船舶模型与固定 10 步 RK4 状态传播

### 用户要求

保持状态类型 `Eigen::Matrix<double, 6, 1>`，将物理含义改为 `[x,y,psi,u,v,r]`；输入保持 `Eigen::Vector3d`，前两项为 `[v_cmd,w_cmd]`，第三项暂不用；保持 `stateTransit()` 函数名，以 RK4 将每段时间分成 10 份。

### 文件与具体改动

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：增加私有常量 `rk4_steps_ = 10`，声明 `usvStateDerivative()`，注明状态和输入分量含义；`stateTransit()` 签名不变。移除不再使用的 `phi_` 成员。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：显式包含 `<cmath>`；增加 `usvStateDerivative()`，采用原方案全部 11 个系数；`stateTransit()` 改为 `h=tau/10`、恒定输入下执行 10 次四阶 RK 积分，先用局部状态计算，最后写入输出。`init()` 中移除旧 `phi_` 单位矩阵初始化。

### 使用的船模与积分公式

```text
dx   = u*cos(psi) - v*sin(psi)
dy   = u*sin(psi) + v*cos(psi)
dpsi = r
du   = -0.433264281*u + 0.400421432*v_cmd + 0.032263988
dv   = -0.313324916*v - 0.006741061*r - 0.407958500*w_cmd - 0.000993619
dr   =  0.166118168*v - 0.544509453*r + 0.837536662*w_cmd + 0.000626529

h = tau / 10
k1 = f(X,U)
k2 = f(X+h*k1/2,U)
k3 = f(X+h*k2/2,U)
k4 = f(X+h*k3,U)
X_next = X + h*(k1+2*k2+2*k3+k4)/6
```

第三输入分量不读取，不做控制限幅；航向保留连续值，周期角处理留给索引与误差计算。每次调用固定 10 步，不是固定 0.05 秒步长；时长越大，单步越大，精度需按实际时长评估。

### 与此前工作的衔接和未完成项

这只替换传播器。现有 `search()` 仍按无人机语义组装起终点并生成三轴加速度，`head(3)`、`tail(3)` 的地图/速度/采样逻辑尚未转换；当前不能直接作为完整无人船规划器使用。积分内部 10 个小步不等于已对这些中间状态做船体碰撞检查，碰撞逻辑仍需后续接通。

### 实际验证结果

用户提出“本地不编译”之前，在临时目录编译了从源码提取出的两个计算函数并做独立数值核对：零时长、输入第三项不影响结果、输入输出同一对象、航向跨 pi 保持连续均通过。使用仿射速度/航向子系统的矩阵指数解以及对应世界速度的 Simpson 积分作为参考，测试状态 `[1,-2,3.1,0.5,-0.1,0.2]`、命令前两项 `[0.7,0.3]`：

- tau=0.6 秒：航向及速度最大绝对误差约 `1.38e-9`，位置误差约 `2.01e-9` 米。
- tau=2.0 秒：航向及速度最大绝对误差约 `2.76e-7`，位置误差约 `4.28e-7` 米。

曾在临时目录启动 `colcon` 构建，收到用户禁止本地编译的新要求后发送停止信号，进程检查中已无相关构建进程；未取得完整工程构建通过结果。后续不再启动本地构建。上述数值结果只验证特定输入下传播计算，不代表模型实船精度或完整规划通过。

## 2026-11-05 · 003 · 最短行驶时间与航向约束启发式

### 用户要求

不在本地电脑编译工程；按方案改为 `h(X)=w_time*max(d/V_max, abs(e_psi)/r_max)`。

### 文件与具体改动

- `AGENTS.md`：保存“不在本地编译”的持久协作要求。
- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：新增 `max_ground_speed_=1.5`、`max_yaw_rate_=0.3`；注明 `optimal_time` 现在是时间下界；删除旧 `tie_breaker_` 成员。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：`setParam()` 声明独立参数 `search/max_ground_speed`（实际对地速度模长上界，m/s）和 `search/max_yaw_rate`（实际偏航角速度绝对值上界，rad/s）。无效上界或无效时间权重会给出提示，启发式回退零。
- 同一文件 `estimateHeuristic()`：只使用 `x,y` 计算二维欧氏距离；用 `atan2(sin(delta_psi),cos(delta_psi))` 计算最短角度误差；将两项时间的最大值写入 `optimal_time`，返回 `w_time_*optimal_time`。移除本函数旧双积分器四次方程、速度半值和额外 tie-break 放大；函数签名保留。`cubic()/quartic()` 原定义暂保留，新的启发式不再调用它们。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：显式增加上述两个参数。最初询问用户时暂留 -1；用户随后明确回复 `max_speed: 1.5, max_turn: 0.3`，据此将头文件、`setParam()` 默认值和启动配置统一设为 `1.5 m/s` 和 `0.3 rad/s`。这是用户指定的实际状态上界，不复用命令上界或原无人机速度参数。
- 本记录同步更新当前决策并追加 002、003 两项。

### 与 002 及上下游的衔接

启发式与传播器统一解释第三状态为航向 psi，不再将其当作高度计算距离。`u,v,r` 在这一时间下界中不直接参与；其动态影响由传播器处理。

保留调用方 `f=g+lambda_heu*h`，因此现有 `lambda_heu=5` 仍使整体成为加权搜索；不能把加权后的项当作严格不高估的标准 A* 启发项。

本式按精确位置和航向目标计算。未来若目标是允许位置/航向误差的区域，需在距离与角度中扣除相应容差，并让终止条件一致；若目标不约束航向，也需由用户确定是否关闭航向项。

`optimal_time` 已不是旧双积分器最优时间。现有 `computeShotTraj()` 仍将其用于三次多项式连接，模型不一致，且同状态或上界未配置时可能为零；Shot 的关闭或重写待后续用户指定。本次没有修改 Shot，也没有宣称完整搜索已可用。

### 检查方式和待定事项

不启动本地构建。使用 Python 对 launch 文件做语法解析，核对头文件/参数声明/launch 参数名称一致、二维距离和角度环绕表达式、旧启发式求根调用及 tie-break 已移除；公式数值检查覆盖距离主导、179 度至 -179 度的最短 2 度角差、同状态零时间。数值示例仅验证公式，不等同于运行 C++ 工程。

实际 `V_max` 与 `r_max` 的配置值已由用户确定为 `1.5` 和 `0.3`。后续仍需把相同实际状态上界接入运动基元有效性检查；当前只用于启发式，传播器本身不会限幅。若传播结果或初始状态超出这些界，时间下界的前提也需重新检查。

## 2026-11-05 · 004 · 统一启发式参数声明默认值

### 用户要求

`max_ground_speed_`、`max_yaw_rate_` 的 `declare_parameter()` 按现有 `search/max_vel`、`search/max_acc` 的方式书写，使用相同的默认值 `-1.0`。

### 修改位置与原因

- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()`：将 `search/max_ground_speed` 的声明默认值从 1.5 改为 -1.0，将 `search/max_yaw_rate` 的声明默认值从 0.3 改为 -1.0，使参数读取风格统一。
- 本记录更新当前决策并追加本项；保留 003 作为历史。

### 与之前修改的衔接

启动文件仍提供实际数值 1.5 m/s 和 0.3 rad/s，正常使用该启动配置时按这两个值计算启发式。如果没有提供参数覆盖，`setParam()` 读取 -1.0 并覆盖成员初值，触发已有提示和 h=0 回退。头文件成员初值、本次启发式公式和其他算法逻辑均未修改。

### 验证结果与待定事项

源码核对两条声明默认值均为 -1.0，并核对启动配置仍为 1.5、0.3；不编译工程。本项没有新增待定决策，之前记录的上下游衔接问题仍待后续修改。

## 2026-11-05 · 005 · RK4 步数改为 ROS 2 参数

### 用户要求

将 `static constexpr int rk4_steps_ = 10` 也改为与其他搜索参数相同的读取方式。

### 文件、改法与原因

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：将编译期常量改为普通成员 `int rk4_steps_ = -1`，使其能够接收参数读取结果；更新注释。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：`setParam()` 增加 `rk4_steps_ = nh.declare_parameter<int>("search/rk4_steps", -1)`。步数是整数，默认值因此使用 -1，与现有整数参数一致。
- 同一文件 `stateTransit()`：除法前检查 `rk4_steps_ > 0`；非法值抛出 `std::invalid_argument`，避免除零或负数导致循环不执行、静默返回原状态。显式包含 `<stdexcept>`。不自动替换用户配置。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：新增 `'search/rk4_steps': 10`，保持用户最初指定的每段 10 小步。
- 本记录更新当前决策并追加本项，保留此前记录作为历史。

### 与此前修改的衔接

船模、状态/输入类型、函数名和 RK4 公式沿用记录 002，仅将步数来源改为参数。每次传播仍按 `h=tau/rk4_steps_` 划分时间；用当前启动文件时为 `tau/10`。若其他启动方式没有配置该参数，默认 -1 会在调用传播函数时报错，需要显式配置正整数。

### 验证结果与待定事项

仅做源码核查和 Python AST 启动文件语法检查：确认声明为整数参数、默认 -1、启动配置为整数 10、除法前有非正值检查，以及积分循环使用同一成员变量。未编译或运行 C++ 工程。本项无新增待定决策，前后端尚未接通的问题沿用此前记录。

## 2026-11-05 · 006 · 参数成员统一为无初值声明

### 用户要求

`max_ground_speed_`、`max_yaw_rate_`、`rk4_steps_` 与原有 `max_tau_`、`init_max_tau_`、`max_vel_`、`max_acc_` 一样处理：头文件只声明，参数读取时赋值。

### 修改位置与原因

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：改为 `double max_ground_speed_, max_yaw_rate_;` 和 `int rk4_steps_;`，去除成员初值；将 `rk4_steps_` 声明移到搜索参数区，与其他整数参数放在一起；更新注释，明确由 `setParam()` 赋值。
- 本记录同步更新当前决策并追加本项，保留此前记录作为历史。

### 与此前修改的衔接

`setParam()` 继续使用默认值 -1.0、-1.0、-1 读取三个参数；启动文件继续提供 1.5、0.3、10。现有管理器按 `setParam()`、`setEnvironment()`、`init()` 的顺序初始化搜索器，符合统一赋值方式。

去除成员初值后，调用 `setParam()` 前这三个值尚未初始化，不能提前调用依赖它们的传播或启发式函数；已有非法数值检查也不能代替先初始化的要求。

### 验证结果与待定事项

源码核对：三个成员均只有一处无初值声明，仍在 `setParam()` 中赋值；管理器初始化顺序符合要求。未编译工程；本项无新增待定决策。

## 2026-11-06 · 007 · 搜索第三输入改为无人船初始命令（仅管理器与搜索接口）

### 用户要求

将管理器调用 `search(...,start_acc,...)` 的第三输入改为无人船命令 `[v_cmd,w_cmd,unused]`，并将对应参数命名改为适合无人船的名称；保持 `Eigen::Vector3d` 类型。

### 已修改文件与原因

- `fast_planner/plan_manage/include/plan_manage/planner_manager.h`、`fast_planner/plan_manage/src/planner_manager.cpp`：`kinodynamicReplan()` 的第三参数统一命名 `start_cmd`；两次 `search()` 调用均传该参数；日志标注为命令，原未使用的局部副本 `init_acc` 同步改为 `init_cmd`；重试注释说明不再限定第一轮使用起始命令。
- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`、`fast_planner/path_searching/src/kinodynamic_astar.cpp`：`search()` 第三参数统一为 `start_cmd`，存储成员从 `start_acc_` 改为 `start_cmd_`，注明分量含义；输入第三项置零，避免预留分量进入首段代价。`init=true` 首次扩展改为使用 `start_cmd_`。
- 本记录同步更新当前决策，追加本项。

### 与已有改动的衔接

首次扩展命令经节点 `input` 传给记录 002 的 RK4 传播器，后者只使用前两项，符合统一输入语义。`init=false` 重试仍不固定首段输入。普通三轴加速度采样尚未改成两维船舶命令采样，本次不擅自更改采样范围。

### 尚未接通的上游与后端

Kino 状态机原先首次规划将 `start_acc_` 置零，重规划则从 `acceleration_traj_` 求值，并将其传给管理器。代码检查未找到该状态机已有的无人船命令订阅或命令输入接口，不能直接把轨迹加速度解释为命令。

已向用户询问来源，用户明确选择“本次只改管理器和 search 接口，命令来源后续再接”。据此保留状态机现有来源逻辑，本次授权范围内的接口修改已完成。当前接口改名不代表整条调用链已经可用，状态机仍传旧加速度的调用必须在后续接入时修正。

`getSamples()` 中 `start_acc`、`end_acc` 表示后端位置曲线需要的加速度，不能简单重命名为命令。该函数仍有将节点 `input` 当作加速度的原逻辑，需后续依据船模转换为世界坐标导数，不能宣称后端边界已兼容船模。

### 检查结果

仅做源码静态检查：管理器/搜索器的声明和定义参数名一致，两次搜索调用均传 `start_cmd`；搜索函数内不再使用 `start_acc`，首次输入使用 `start_cmd_`，第三分量置零。未编译或运行 C++ 工程。上游来源和后端导数转换由用户后续指定再修改。

## 2026-11-06 · 008 · 状态机调用前用船模反算初始命令

### 用户要求

在 `KinoReplanFSM::callKinodynamicReplan()` 调用管理器前，根据船舶动力学将起始加速度等状态转换成 `start_cmd=[v_cmd,w_cmd,0]`，再参与后续规划。

### 文件、修改内容与原因

- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：新增 `<cmath>`；在 `callKinodynamicReplan()` 的 `bool plan_success` 之前计算局部 `start_cmd`，并将管理器第三实参从 `start_acc_` 改为 `start_cmd`。
- 原有 `start_acc_` 保持世界坐标位置加速度含义，不覆盖为命令；没有把其第三项当作偏航角加速度。
- 读取 `start_yaw_=[psi,r,dot_r]`。使用当前代码约定的世界速度 `start_vel_(0:1)`、世界加速度 `start_acc_(0:1)` 计算船体系状态和导数；首次规划使用现有零加速度、零偏航角速度及零偏航角加速度假设，重规划使用旧轨迹对应导数。没有修改这些来源。
- 同步更新本记录的当前状态并追加本项。

### 公式及与传播器的衔接

```text
u = cos(psi)*vx + sin(psi)*vy
v = -sin(psi)*vx + cos(psi)*vy
dot_u = cos(psi)*ax + sin(psi)*ay + r*v

v_cmd = (dot_u + 0.433264281*u - 0.032263988) / 0.400421432
w_cmd = (dot_r - 0.166118168*v + 0.544509453*r - 0.000626529) / 0.837536662
start_cmd = [v_cmd,w_cmd,0]
```

`r*v` 来自旋转坐标系求导，不可省略。反算使用的纵向与偏航系数与 `usvStateDerivative()` 一致；在当前最小修改实现中，相关系数分别出现在正向和反向代码中，未来改模型必须同步更新。

命令随后经 `kinodynamicReplan(...,start_cmd,...)`、`search(...,start_cmd,...)` 和首次扩展的 `start_cmd_` 进入 RK4 船模，接续记录 002 和 007。

### 静态与数值检查（不编译）

通过 Python 读取两处源码的实际标量表达式，使用船模正向导数构造世界速度、加速度及偏航导数，再反算命令。覆盖 4 种航向和 3 组速度/命令，共 12 例，最大命令恢复绝对误差约 `2.78e-17`。检查管理器第三实参为 `start_cmd`，局部第三项在调用前设为零。未编译工程、未运行 ROS 节点或 C++ 程序。

### 限制与后续待接通部分

- 反算纵向与偏航两式并不保证任意给定世界加速度、航向导数满足横向方程；需要后续计算 `dot_v=-sin(psi)*ax+cos(psi)*ay-r*u`，与船模横向方程比较。此次不自行选择残差容差或失败处理。
- 未配置实际命令上下界，未做命令限幅；实际速度上界 1.5 和偏航角速度上界 0.3 不能直接当作命令边界。用户需后续决定。
- 静止且期望所有输入导数为零时，模型偏置会使反算值约为 `v_cmd=-0.080575078`、`w_cmd=-0.000748062`；不等同于已验证可发送的实船命令。
- 本次仅完成初始命令反算。传入搜索的其他状态仍需统一为 `[x,y,psi]` 和 `[u,v,r]`，当前状态机仍保留原位置/世界速度传法，不能将整个规划流程视为已完成船模接入。
- 普通控制采样、状态哈希、船体碰撞、终端条件、Shot 和后端导数等未接通问题沿用此前记录。

## 2026-11-06 · 009 · 三个位姿导数向量反算命令并封装函数

### 用户要求与当前表示

不再依赖 `start_yaw_`，将 `start_pt_` 解释为 `[x,y,phi]`，`start_vel_` 为 `[dot_x,dot_y,dot_phi]`，`start_acc_` 为 `[ddot_x,ddot_y,ddot_phi]`；重规划从位置轨迹及其导数获得三个向量，命令计算封装成函数。

### 修改文件与原因

- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：声明私有辅助函数 `computeUsvCommand(const Eigen::Vector3d& pose, const Eigen::Vector3d& pose_vel, const Eigen::Vector3d& pose_acc) const`；移除 Kino 状态机的 `start_yaw_` 成员，补充三个向量与里程计数据分量的含义注释。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：把记录 008 的反算代码迁入该函数，航向改读 `pose(2)`，偏航角速度改读 `pose_vel(2)`，偏航角加速度改读 `pose_acc(2)`；世界速度/加速度转换及辨识系数不变，返回 `[v_cmd,w_cmd,0]`。
- `callKinodynamicReplan()` 仅调用 `computeUsvCommand(start_pt_,start_vel_,start_acc_)` 生成局部命令，再传给管理器，避免计算逻辑与规划发布流程混在一起。
- `GEN_NEW_TRAJ`：保持现有里程计位置和速度赋值、零加速度假设，但将 `start_pt_(2)` 用姿态四元数导出的航向覆盖；`odometryCallback()` 中 `odom_vel_(2)` 改读 `twist.angular.z`，不再使用竖直速度作为航向角速度。
- `REPLAN_TRAJ`：保留用户指定的 `position_traj_`、`velocity_traj_`、`acceleration_traj_` 三条求值语句，删除独立 yaw/yawdot/yawdotdot 的起始状态求值。
- 旧 `planYaw()` 和消息中的偏航字段仍保留兼容路径：临时用 `yaw_boundary=[start_pt_(2),start_vel_(2),start_acc_(2)]` 传入 `planYaw()`，不再使用 `start_yaw_`。此调用不代表独立偏航后端已经合并为第三维位姿曲线。
- 本记录更新当前决策，并追加本项。

### 公式与上一步的衔接

```text
psi = pose(2)
r = pose_vel(2)
dot_r = pose_acc(2)
u = cos(psi)*pose_vel(0) + sin(psi)*pose_vel(1)
v = -sin(psi)*pose_vel(0) + cos(psi)*pose_vel(1)
dot_u = cos(psi)*pose_acc(0) + sin(psi)*pose_acc(1) + r*v
```

再使用记录 008 的逆动力学计算两项命令。输入三个向量是世界位置和航向的导数，不是 `[u,v,r]`；辅助函数内部才将平移导数转换为船体速度。搜索器所需六状态为 `[x,y,phi,u,v,r]`，因此在管理器/搜索边界仍需后续明确世界速度与船体速度的转换。

### 检查与限制

仅做源码及 Python 标量数值检查，不编译。辅助函数声明/定义/调用与输入分量对应检查通过；Kino 头文件和实现中不再出现 `start_yaw_`；首次航向和角速度源、重规划仅从三个轨迹对象提取边界的源码检查通过。数值检查沿用船模正算与逆算 12 个不同航向/速度/命令组合，最大命令恢复误差约 `2.78e-17`。这些检查不代表完整工程运行或轨迹动力学可行性通过。

当前 `position_traj_` 仍由未全面适配的搜索和 B 样条后端生成，不能仅因本次修改就保证其第三维实际已是正确航向。人工目标仍带原高度设置、预设航点仍有 `_z` 参数、地图/可视化仍按三维空间点解释、独立 yaw 后端仍存在，需逐项处理。不同单位的 `[x,y,phi]` 不能直接共用未经修改的欧氏距离和统一速度/加速度限制。

里程计平移分量沿用现有代码的世界坐标假设；若实际消息的线速度在船体系，需后续按发布者约定转换。首次加速度仍为零假设，并未新增加速度测量；横向动力学残差和命令限幅尚未实现。Topo 状态机属于另一类，本次未改其独立 `start_yaw_`。

## 2026-11-06 · 010 · Kino 移除独立偏航路径并转换里程计船体速度

### 用户要求与补充确认

1. 不再保留 Kino 流程中的独立 `planYaw()` 与偏航消息字段赋值。
2. 将里程计转换为无人船状态，速度为 `[u,v,r]`；后续逻辑暂不修改，按运行流程逐项推进。
3. 用户确认 `/state_ukf/odom` 的 `twist.linear.x/y` 在世界坐标系，需要按航向旋转为船体速度。
4. 用户明确选择：本次仅移除 Kino 独立偏航规划及字段赋值，消息定义稍后改；不扩展修改 Topo 或 `traj_server`。

### 开始修改时发现的源码差异

当前 `kino_replan_fsm.cpp` 与记录 009 的完成状态不一致：原始 `start_yaw_` 读写和独立偏航流程重新出现；`computeUsvCommand()` 定义及其调用不在当前 `.cpp` 中，管理器第三实参为 `start_acc_`。头文件仍保留 `computeUsvCommand()` 声明，且没有 `start_yaw_` 成员。已向用户说明，以当前磁盘文件为基准执行本次明确范围，不自动覆盖或恢复之前版本。

### 修改文件与逐项原因

- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：显式包含 `<cmath>`。`odometryCallback()` 先保存姿态四元数，计算平面航向 `phi`，再保存 `odom_pos_=[x,y,phi]`；不再把高度放在第三项。
- 同一回调读取世界速度 `vx,vy`，保存 `odom_vel_(0)=cos(phi)*vx+sin(phi)*vy`、`odom_vel_(1)=-sin(phi)*vx+cos(phi)*vy`、`odom_vel_(2)=twist.angular.z`，与船模 `[u,v,r]` 一致。
- `GEN_NEW_TRAJ`：直接复制 `odom_pos_`、`odom_vel_`；移除旧 `start_yaw_` 初始化及重复航向计算。起始加速度仍沿用原先置零逻辑。
- `REPLAN_TRAJ`：移除 `start_yaw_` 对独立 yaw/yawdot/yawdotdot 轨迹的读取；用户指定的三个位置/速度/加速度轨迹求值语句保持原样。
- `callKinodynamicReplan()`：删除 `planYaw()` 调用，删除 `yaw_traj_` 控制点读取、`bspline.yaw_pts` 填充和 `bspline.yaw_dt` 赋值。仅继续现有位姿样条发布流程，不恢复命令反算或改动管理器调用参数。
- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：更新里程计成员注释，明确初始规划 `start_vel_` 是船体速度，但重规划仍取轨迹导数、转换待做；`start_acc_` 仍是零假设/轨迹二阶导数，不是命令。
- 更新本记录的当前状态并追加本项，历史记录保留。

### 与已有船模的衔接

初始规划中 `start_pt_=[x,y,phi]`、`start_vel_=[u,v,r]`，搜索器若按 `head(3)`、`tail(3)` 拼接，可对应已修改的六状态传播器。

用户确认世界线速度输入后，使用平面旋转矩阵的逆变换得到船体系分量，不能再将该 `[u,v,r]` 当成世界速度重复旋转。后续若重新接入记录 009 的 `computeUsvCommand()`，需注意其 `pose_vel` 输入约定是世界位姿导数，与现在首次规划的 `start_vel_` 不同，必须先明确接口，不能直接照搬调用。

### 检查结果（不编译）

源码检查通过：Kino `.h/.cpp` 无 `start_yaw_` 引用，Kino `.cpp` 无独立 `planYaw()`、独立 yaw 轨迹读取或偏航字段赋值；消息定义中的字段仍在，符合本次明确范围；重规划三条样条求值语句保留。

从实际源码提取速度变换表达式，用 Python 检查 6 种航向、3 组速度，共 18 例世界/船体往返转换，最大误差约 `1.11e-16`。检查第三项使用 `angular.z`，不使用 `linear.z`。未编译工程或运行 ROS 节点。

### 后续待处理

- `Bspline.msg` 仍含 `yaw_pts/yaw_dt`，Kino 不再填充，默认值为空/零；旧 `traj_server` 仍依赖独立偏航数据，目前不能视为已适配新的 Kino 消息。共享消息定义、消费者与 Topo 修改由用户后续指定。
- 重规划仍使用 `[dot_x,dot_y,dot_phi]` 的样条导数，尚未转换为船体 `[u,v,r]`；本次按用户要求不改。
- 当前磁盘版本的命令反算实现缺失、管理器仍接收 `start_acc_` 实参，不能宣称命令链路接通。
- 人工/预设目标、地图、可视化、控制采样、终端条件、Shot 和后端适配沿用此前待办，均不在本次范围内。

## 2026-11-06 · 011 · 补齐当前源码中的命令反算定义和调用

### 用户反馈与修改前核查

用户指出代码中没有此前说明的状态转命令封装函数和调用。检查确认：头文件有声明，当前 `kino_replan_fsm.cpp` 中缺少定义，`callKinodynamicReplan()` 仍把 `start_acc_` 直接传给管理器。此次根据用户要求补齐实际实现与调用。

### 文件、改法与原因

- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：在 `callKinodynamicReplan()` 前增加 `computeUsvCommand()` 的实际定义，输入 `pose=[x,y,phi]`、`pose_vel=[dot_x,dot_y,r]`、`pose_acc=[ddot_x,ddot_y,dot_r]`，返回 `[v_cmd,w_cmd,0]`。
- 同一文件 `callKinodynamicReplan()`：创建局部 `pose_vel`。`GEN_NEW_TRAJ` 时现有 `start_vel_` 已是船体系 `[u,v,r]`，先将局部副本前两项正向旋转成世界速度；`REPLAN_TRAJ` 的 `start_vel_` 仍是轨迹一阶导数，直接使用。第三项 r 保留，成员 `start_vel_` 不被修改。
- 通过 `computeUsvCommand(start_pt_,pose_vel,start_acc_)` 生成局部 `start_cmd`，管理器调用第三实参改为该命令，不再传加速度。
- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：为已有声明补充输入坐标系和输出命令说明，保持函数声明签名一致。
- 本记录更新当前状态，并追加本项。

### 公式与已有实现的衔接

```text
# 首次规划：只对局部速度副本做船体到世界转换
dot_x = cos(phi)*u - sin(phi)*v
dot_y = sin(phi)*u + cos(phi)*v

# 反算函数：统一由世界位姿导数计算
psi = pose(2)
r = pose_vel(2)
dot_r = pose_acc(2)
u = cos(psi)*pose_vel(0) + sin(psi)*pose_vel(1)
v = -sin(psi)*pose_vel(0) + cos(psi)*pose_vel(1)
dot_u = cos(psi)*pose_acc(0) + sin(psi)*pose_acc(1) + r*v
v_cmd = (dot_u + 0.433264281*u - 0.032263988) / 0.400421432
w_cmd = (dot_r - 0.166118168*v + 0.544509453*r - 0.000626529) / 0.837536662
```

系数与正向 `usvStateDerivative()` 保持一致。局部输入适配避免将记录 010 已转换为船体速度的值错误地再次按世界速度解释，同时保留记录 009 的三个世界位姿向量输入约定。所有当前调用来自 `GEN_NEW_TRAJ` 或 `REPLAN_TRAJ`，该分支选择依赖现有状态机；未来新增其他调用路径时必须明确输入坐标系。

### 验证结果（不编译）

检查当前文件恰有一个辅助函数定义、头文件恰有一个声明，且实际调用位于管理器调用之前；管理器第三参数为 `start_cmd`；输出第三项为零；Kino 中未恢复任何独立偏航规划或 `start_yaw_` 引用。

通过 Python 提取实际 C++ 标量表达式，使用正向船模生成世界位姿导数，分别检查首次规划的输入适配路径和重规划路径。覆盖 4 种航向、3 组速度/命令、2 条路径，共 24 例，最大命令恢复误差约 `2.78e-17`。未编译、未运行 C++ 程序或 ROS 节点；此检查不能替代完整工程运行验证。

### 仍未修改的部分

本次未改动重规划传给搜索器的速度语义，仍存在世界导数与船体状态的待转换接口；没有修改控制采样、目标、地图、后端或消息消费者。反算仍只匹配纵向与偏航方程，没有横向残差约束或命令限幅。首次 `start_acc_` 仍为零假设；此反算结果是期望导数对应的模型命令，不是测量得到的当前执行命令。

## 2026-11-06 · 012 · 规划入口统一船体速度与船模命令

### 用户要求

在调用 `planner_manager_->kinodynamicReplan()` 前，使输入速度和输入命令符合已有无人船动力学模型；不再修改动力学模型。传播器的状态为 `[x,y,phi,u,v,r]`，其中位置和航向相对参考系，两个平移速度在船体系，r 为偏航角速度。

### 文件与修改内容

- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：声明 `computeBodyVelocity(pose,pose_vel)`，输入为参考系位姿及其一阶导数，输出船体速度；更新成员来源注释，明确在管理器边界适配。目标速度 `end_vel_` 约定为世界位姿导数，当前实际赋值为零。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：实现 `computeBodyVelocity()`，按航向把 `[dot_x,dot_y]` 转成 `[u,v]`，保留第三项 r。
- 同一文件 `computeUsvCommand()`：复用该函数取得 u、v，保留记录 011 的逆动力学公式和世界加速度输入约定，避免两处反算使用不同的速度转换。
- 同一文件 `callKinodynamicReplan()`：新增局部 `start_body_vel`。首次规划速度已经在船体系，直接沿用；重规划速度为世界导数，调用辅助函数转换。保留局部世界速度 `pose_vel` 用于逆动力学输入。
- 目标速度通过 `computeBodyVelocity(end_pt_,end_vel_)` 得到局部 `end_body_vel`；当前零目标速度转换后仍为零。如果后续改为非零目标速度，来源必须遵守上述世界导数输入约定。
- 管理器调用改为 `kinodynamicReplan(start_pt_,start_body_vel,start_cmd,end_pt_,end_body_vel)`。没有覆盖状态机原成员，避免重复调用时重复转换。
- `fast_planner/plan_manage/include/plan_manage/planner_manager.h`：补充接口注释，明确输入位姿为参考系 `[x,y,phi]`、速度为船体 `[u,v,r]`，命令仍是 `[v_cmd,w_cmd,0]`。
- 更新本记录并追加本项。

### 公式与衔接

```text
u = cos(phi)*dot_x + sin(phi)*dot_y
v = -sin(phi)*dot_x + cos(phi)*dot_y
r = dot_phi
```

逆动力学继续使用世界加速度转换后的 `dot_u=cos(phi)*ddot_x+sin(phi)*ddot_y+r*v` 以及 `dot_r=ddot_phi` 计算命令。命令是纵向和转向控制通道的标量，通过船模反算得到，不把 `[v_cmd,w_cmd]` 当成世界平面向量做旋转。

管理器沿原调用链将船体速度送入 `search()`，后者用 `head(3)=pose`、`tail(3)=body_velocity` 组装六状态，匹配已有 `usvStateDerivative()` 和 RK4。传播输出是经过 tau 时间后的预测 `[x,y,phi,u,v,r]`，不等同于传感器测量的当前状态。

### 验证结果（不编译）

检查辅助函数声明、定义、命令反算复用及管理器实参一致，首次和重规划两条路径均传船体速度；检查零目标速度转换保持为零。

用实际源码标量表达式进行 24 组首次/重规划输入路径验证，船体速度最大恢复误差约 `5.55e-17`，命令最大恢复误差约 `2.78e-17`。未编译或运行 C++/ROS 程序。

对修改前后 `path_searching/src/kinodynamic_astar.cpp` 做 SHA256 核对，两者均为 `e58528e1347b8534e9c676ca07d48dfb313eafdee5b89d2386ed30a474eb28f3`，确认该文件及其中的船模、RK4 没有改动。

### 待后续处理

起终点位姿来源是否完整符合 `[x,y,phi]`，尤其是旧目标高度参数、航向连续化、地图查询、控制采样、终端检查、Shot、B 样条与消息消费者，仍按用户逐步指令推进。逆动力学没有新增横向残差约束或命令限幅，初始加速度仍采用零假设；本轮不能代表完整规划器可运行或全轨迹动力学可行性已经验证。

## 2026-11-06 · 013 · 统一参考系导数输入并合并船体速度与命令转换

### 用户要求

将 `odom_vel_` 恢复为参考系速度，使首次规划与重规划的 `start_pt_`、`start_vel_`、`start_acc_` 均为位姿及其一、二阶导数；不再判断 `GEN_NEW_TRAJ/REPLAN_TRAJ` 做坐标转换。封装一个函数完成船体速度与船舶输入命令的计算。

### 修改文件、内容与原因

- `fast_planner/plan_manage/src/kino_replan_fsm.cpp` 的 `odometryCallback()`：前两项直接保存用户确认的世界线速度 `vx,vy`，删除回调中的速度旋转和不再使用的正余弦变量；第三项仍读取 `angular.z`，表示航向角速度，不恢复为原无人机的 `linear.z`。
- 同一文件将 `computeUsvCommand()` 合并替换为 `void computeUsvStateAndCommand(pose,pose_vel,pose_acc,body_vel,command) const`；前 3 个参数为输入，后 2 个引用参数分别输出 `[u,v,r]` 和 `[v_cmd,w_cmd,0]`。函数内部复用 `computeBodyVelocity()` 完成速度旋转，再用原逆动力学计算命令，公式与辨识系数不变。
- `callKinodynamicReplan()` 删除按 `exec_state_` 分支的转换、局部 `pose_vel` 以及初始速度的反向旋转，改为无分支调用 `computeUsvStateAndCommand(start_pt_,start_vel_,start_acc_,start_body_vel,start_cmd)`；两项输出用于管理器调用。
- 目标速度仍通过 `computeBodyVelocity(end_pt_,end_vel_)` 转换；当前目标速度为零，本次不新增目标加速度或目标命令假设。
- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：更新里程计与起始速度的统一坐标系说明；同步替换函数声明，注明双输出含义；保留低层旋转函数供组合转换与目标速度复用。
- 本记录同步更新当前状态并追加本项，历史 010–012 的实现过程保留。

### 统一接口和数学关系

```text
输入：pose=[x,y,phi]
      pose_vel=[dot_x,dot_y,r]
      pose_acc=[ddot_x,ddot_y,dot_r]

u = cos(phi)*dot_x + sin(phi)*dot_y
v = -sin(phi)*dot_x + cos(phi)*dot_y
dot_u = cos(phi)*ddot_x + sin(phi)*ddot_y + r*v

输出：body_vel=[u,v,r]
      command=[(dot_u+0.433264281*u-0.032263988)/0.400421432,
               (dot_r-0.166118168*v+0.544509453*r-0.000626529)/0.837536662,
               0]
```

原始 `start_vel_`、`start_acc_` 不覆盖为船体速度和命令，以保持与 `start_pt_` 的微分关系。局部 `start_body_vel`、`start_cmd` 才传入管理器；首次和重规划共用相同转换路径，消除变量随状态改变坐标系的问题。

### 验证与未完成项

不编译工程。源码检查通过：声明/定义/调用一致、里程计直接保留参考系平移速度及角速度、转换调用无状态分支、旧函数名无残留、双输出全部赋值。船模源码 SHA256 仍为 `e58528e1347b8534e9c676ca07d48dfb313eafdee5b89d2386ed30a474eb28f3`，确认未修改。

提取实际源码表达式进行正向船模与统一转换的数值核对，覆盖 4 种航向和 3 组速度/命令，共 12 例。船体速度最大恢复误差约 `5.55e-17`，命令最大恢复误差约 `2.78e-17`。这仅检查数学转换，不代表完整 C++/ROS 工程运行通过。

首次加速度仍为零假设，并非实测；重规划的三个样条求值语句保持原样，要求上游第三维实际表示连续航向。横向残差、命令限幅、目标来源、地图、后续搜索/后端与消息消费者适配仍按后续指令推进，本次未改动力学模型。

## 2026-11-06 · 014 · 管理器与搜索器统一船体速度形参名称

### 用户要求

将 `kinodynamicReplan()` 和 `search()` 的起始、终止速度输入明确命名为 `start_body_vel`、`end_body_vel`。

### 修改位置与原因

- `fast_planner/plan_manage/include/plan_manage/planner_manager.h`：同步修改 `kinodynamicReplan()` 声明中的两个速度形参名，保持已有船体系接口注释。
- `fast_planner/plan_manage/src/planner_manager.cpp`：修改函数定义、日志、局部副本及两次 `search()` 调用；原局部 `init_vel` 同步改为 `init_body_vel`。
- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：同步修改 `search()` 声明中的起终点速度名称，补充 `[u,v,r]` 船体速度注释。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：定义形参从 `start_v/end_v` 改为 `start_body_vel/end_body_vel`，内部保存起始速度与组装起终点六状态的引用全部同步。
- 本记录更新当前接口约定并追加本项。

### 与此前修改的衔接

记录 013 的状态机已将局部船体速度传给管理器，本次统一管理器到搜索器的名称，明确整个入口链路的坐标语义。类型和参数顺序不变，不增加旋转或修改数值。

`search()` 内部存储成员 `start_vel_` 保持原名，赋值来自 `start_body_vel`；其他函数的局部速度变量与状态机原始参考系导数成员不做全局替换，避免扩大本次范围。动力学与 RK4 算法未修改。

### 验证结果

源码核查通过：两个函数的声明/定义命名一致；管理器两次搜索均传 `start_body_vel/end_body_vel`；起终点 `state.tail(3)` 分别取对应船体速度；两个函数内无旧速度形参名残留。未编译或运行 C++/ROS 工程。本项无新增待定决策。

## 2026-11-06 · 015 · 船体速度与命令输出改为状态机成员

### 用户要求

将 `start_body_vel_`、`start_cmd_`、`end_body_vel_` 与 `start_pt_`、`start_vel_`、`start_acc_` 一起声明在类数据区，转换和调用处不再局部定义。

### 文件与改动原因

- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：在起点数据后新增 `Eigen::Vector3d start_body_vel_, start_cmd_;`，在终点数据后新增 `Eigen::Vector3d end_body_vel_;`，注明含义。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：删除 `callKinodynamicReplan()` 中两个起点输出的局部声明，将终点输出从局部 `const` 定义改为成员赋值。管理器调用沿用当前文件中的带下划线变量名。
- 本记录同步更新当前状态并追加本项。

### 与已有转换的衔接

每次调用先由 `computeUsvStateAndCommand()` 完整写入两个起点成员，再由 `computeBodyVelocity()` 写入终点成员，然后传给管理器；计算公式与坐标系不变。`end_body_vel_` 需要随目标和规划调用更新，因此成员不声明为 `const`。这些成员按原有变量风格仅声明，使用前必须完成上述赋值。

### 验证结果

源码核对三个成员均在头文件有唯一声明，函数内没有同名局部声明遮蔽；三个输出赋值均位于管理器调用前，调用使用对应成员。未编译工程。本项无新增待定决策。

## 2026-11-06 · 016 · 转换函数归并至船模文件，增加船体到全局速度转换

### 用户要求及补充说明

将 `computeUsvStateAndCommand()`、`computeBodyVelocity()` 的定义与声明移到无人船动力学所在文件，并在同处新增船体速度转全局速度函数。用户特别明确：输出为 `[vx,vy,r]`，最后一项是角速度，不是航向角。

### 文件与具体修改

- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：移除两个转换函数的状态机成员声明；记录 015 的输出成员继续保留。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：移除两个函数的状态机定义，调用改为 `KinodynamicAstar::computeUsvStateAndCommand(...)` 和 `KinodynamicAstar::computeBodyVelocity(...)`；仍在规划前写入三个输出成员，再传入管理器。
- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：新增三个转换函数的公共静态声明，位于船模所在的同一头文件；这些函数不依赖搜索器实例或 ROS 参数，采用静态接口使状态机可以直接调用，无需暴露管理器私有搜索器。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：在 `usvStateDerivative()` 前放置两个迁移函数及新增的 `computeGlobalVelocity(pose,body_vel)`。两个迁移函数只改所属类及静态接口对应的限定，不改计算体。新函数中 `pose=[x,y,phi]`，输入 `body_vel=[u,v,r]`，输出 `[vx,vy,r]`。
- 更新本记录当前状态并追加本项。

### 新函数公式与使用方式

```text
vx = cos(phi)*u - sin(phi)*v
vy = sin(phi)*u + cos(phi)*v
global_vel(2) = r

global_vel = KinodynamicAstar::computeGlobalVelocity(pose, body_vel)
```

航向 phi 是旋转所需输入角度，来自 `pose(2)`。速度第三分量原样保留 `body_vel(2)`，不输出 phi。在平面船模下 r=dot_phi，仍是 rad/s；新函数不转换加速度、不做积分，也不主动接入其他后续模块。

### 与现有动力学及调用链的衔接

统一转换与正向模型现在处于同一文件，便于共同检查系数与坐标定义。逆动力学输出仍为 `[v_cmd,w_cmd,0]`，与 RK4 输入保持一致；`stateTransit()` 和 `usvStateDerivative()` 实现未改动。公共静态转换无需创建未初始化的搜索器对象，也不需要读取步数或速度上界参数。

### 验证结果（不编译）

保存迁移前快照并逐函数比较：两个迁移函数的函数体去除空白后与迁移前一致；`usvStateDerivative()` 至文件结尾（含 RK4）的源码原文一致。检查声明均为公共静态成员、每个函数唯一定义、状态机不再有旧声明/定义且调用已限定为 `KinodynamicAstar::`。

从实际源码提取正反速度转换表达式，覆盖 6 种航向（含超过一周的连续角度）、3 组速度，共 18 例往返转换，最大误差约 `1.11e-16`。每例同时验证输出第三项严格保持 r。未编译工程或运行 C++/ROS 程序。

### 后续待处理

此前记录的横向动力学一致性、命令限幅、目标来源、地图、搜索与后端/消息消费者适配继续待用户逐步指定。本次只归并文件与增加速度转换接口，未扩大实现范围。

## 2026-11-06 · 017 · 搜索器起始导数保存为参考系速度

### 用户要求

在 `KinodynamicAstar::search()` 中，将原 `start_vel_ = start_body_vel` 改为调用 `KinodynamicAstar::computeGlobalVelocity(pose,body_vel)`。

### 修改位置与原因

- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：搜索函数开头改为 `start_vel_ = KinodynamicAstar::computeGlobalVelocity(start_pt, start_body_vel);`，注释为 `[vx,vy,r]`。`start_pt` 提供 `phi`，`start_body_vel` 提供 `[u,v,r]`。
- 本记录更新当前约定并追加本项。

### 与此前代码的衔接

`getSamples()` 将 `start_vel_` 放进 `start_end_derivatives`，因此这一项现在采用位置位姿曲线所需的参考系导数 `[vx,vy,r]`。搜索节点的 `cur_node->state.tail(3)` 仍为 `start_body_vel=[u,v,r]`，与 RK4 船模状态一致；角速度 r 不被旋转或替换为航向角。

此次不处理终点速度与起终加速度：`getSamples()` 其他导数及其坐标系仍需依照后续后端改造逐项检查，不能仅凭本次起点速度转换就认为整个 B 样条接口已经完成。

### 验证结果

源码检查 `start_vel_` 使用指定函数和两个起始参数、节点尾部仍为船体速度、`getSamples()` 仍输出该成员；核对 `computeGlobalVelocity()` 第三分量保留 r，并做角度 90 度的公式数值检查。未编译或运行 C++/ROS 工程。

## 2026-11-06 · 018 · 适配无人船位置与周期航向索引

### 用户要求及确认

修改 `posToIndex()`，输入状态按 `[x,y,phi]` 处理并调整分辨率；用户明确指定位置分辨率 `0.25 m`，航向分辨率 `5°`。

### 修改前的源码差异

检查发现当前 `search()` 中 `start_vel_ = start_body_vel`，与记录 017 的转换版本不一致。已向用户说明；本次仅修改索引及其必要参数，不自动覆盖该赋值。记录 017 保留为历史，不表示当前源码仍是该版本。

### 文件、修改内容与原因

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：增加 `int yaw_bins_` 和 `double yaw_resolution_` 成员，按项目参数风格不在头文件赋默认值；注释 `posToIndex()` 输入是 `[x,y,phi]`，输出是 x、y 栅格索引和航向索引。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()`：增加 `search/yaw_bins` 整数参数，声明默认值为 -1，保持现有参数风格。
- 同文件 `init()`：验证位置分辨率有限且大于零、航向区间数大于零；根据 `2*pi/yaw_bins_` 计算实际航向分辨率。参数无效抛出异常，避免除零或使用无效索引。直接以区间数设置，保证整圆等分且闭合。
- 同文件 `posToIndex()`：x、y 仍减各自的地图原点并按位置分辨率向下取整；phi 不再减地图 z 原点，先按 `2*pi` 周期归一到 `[0,2*pi)`，再用角度分辨率离散。对浮点舍入造成的上界越界做钳制。返回类型仍为 `Eigen::Vector3i`。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：`search/resolution_astar` 从 0.1 改为 0.25 米，新增 `search/yaw_bins=72`，对应 `360/72=5°`。其他启动方式若未提供新的正整数参数，初始化会报错，需对应配置。
- 本记录同步更新当前状态并追加本项。

### 数学关系与原代码衔接

```text
i_x = floor((x - origin_x) / 0.25)
i_y = floor((y - origin_y) / 0.25)
phi_normalized = phi mod 2*pi，取值 [0,2*pi)
delta_phi = 2*pi / 72 = 5°
i_phi = floor(phi_normalized / delta_phi)，取值 0..71
```

示例：`phi=0°` 与 `phi=360°` 均为索引 0；`phi=-0.1°` 与 `phi=359.9°` 均为索引 71；`phi=90°` 为索引 18。角度跨界两侧虽然物理上相近，仍可能处于索引 71 和 0；相邻性或角差比较必须明确做周期处理。

起点、目标和候选节点仍复用同一个 `posToIndex()`，哈希键仍为三维整数，因此此次只改索引语义和分辨率，不扩展到按 `u,v,r` 去重；同一位置与航向区间中的不同速度仍可能合并。

### 检查结果（不编译）

Python AST 检查启动文件和参数值；源码检查参数声明、初始化的正值验证、x/y 原点以及 phi 周期计算路径。按同一数学公式检查负 x/y、90°、0°/±360°、负小角度以及多个整圈偏移。结果通过；这是公式检查和源码核对，不等同于 C++/ROS 运行或实际搜索验证。

### 需要后续适配

- `near_end` 仍直接按整数差比较三个索引，第三维使用位置的 `tolerance` 且没有环绕，不能视为合理的无人船终端航向条件；应在用户后续指定的终端改造中处理。
- `reach_horizon` 仍对 `[x,y,phi]` 计算三维欧氏范数，混合米和弧度；原占据地图查询仍将 `head(3)` 第三项作为空间高度处理。此次未扩大修改范围，这些条件未适配前不能认为完整规划安全可用。
- 起始导数的当前源码与记录 017 不一致，见本项“修改前源码差异”。

## 2026-11-06 · 019 · 无人船搜索时间索引及动态哈希衔接

### 用户要求

针对无人船修改 `timeToIndex(time_start)` 的时间索引，使之适配无人船搜索。

### 实际语义与设计

时间本身不因船体运动学而改变单位或需要航向变换。此处保留 `i_t=floor((time-time_origin_)/time_resolution_)`，明确用于动态搜索：`time_origin_` 是本次搜索起点 `time_start`，`time` 是相同时间基准的节点到达时刻，单位秒。起点 `timeToIndex(time_start)=0`。每段恒定命令运动基元的实际持续时间 `tau` 累加到父节点时间；RK4 数值小步只影响积分精度，不代替 `tau`。

### 文件与具体改动

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：为 `time_origin_` 与 `timeToIndex()` 增加相对动态搜索起点、以秒计的注释；函数签名保持不变。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `init()`：检查 `search/time_resolution` 有限且大于零，求倒数后也检查有限性；无效值显式报错，避免除零和无效时间分箱。
- 同文件 `timeToIndex()`：保持已有相对时间公式，用 `std::floor` 明确向下取整，先验证输入时间、时间原点有限，再验证计算结果有限且在 `int` 范围内，最后显式转换整数。允许负的相对时间按数学 floor 分箱；不强加未经用户确定的时间戳来源或运动基元限制。
- 同文件 `search()` 候选扩展：只有 `dynamic=true` 时才读取 `cur_node->time`、调用 `timeToIndex()` 和计算 `time_idx` 差；静态模式使用不会读取这些未初始化字段的占位值。物理传播仍始终使用原 `tau`。
- 同文件动态哈希插入：由原来的 `expanded_nodes_.insert(pro_id, pro_node->time, pro_node)` 改为 `expanded_nodes_.insert(pro_id, pro_node->time_idx, pro_node)`，使插入与 `find(pro_id,pro_t_id)` 使用相同的离散时间索引；否则连续 double 会被当作整数键且与查找键不一致。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：给原 `search/time_resolution=0.8` 添加“动态时间分箱，单位秒”注释，未改变数值；当前没有足够依据擅自调整。
- 本记录同步当前状态并追加本项。

### 示例与与已有船模的衔接

```text
time_start = 100.0 s，time_resolution = 0.8 s
time = 100.0 s -> i_t = 0
time = 100.4 s -> i_t = 0
time = 101.0 s -> i_t = 1
time = 101.7 s -> i_t = 2
```

动态哈希键为 `[i_x,i_y,i_phi,i_t]`；空间/航向索引由记录 018 的 `posToIndex()` 负责，时间索引单独处理。静态搜索仍仅按 `[i_x,i_y,i_phi]` 查重。无论静态还是动态，船模状态传播都保留 `[x,y,phi,u,v,r]` 和 RK4 对 `tau` 的积分，不修改动力学模型。

### 检查结果（不编译）

源码核对时间原点和起始时间设置；动态计算、哈希插入与查找均使用整数时间索引；静态路径不再求值动态时间字段；时间分辨率和数值范围检查存在。Python 对源码所示公式检查起点、不同时间区间、负偏移及动态键插入/查找；启动文件 `0.8` 参数及语法核对通过。未编译、未运行 C++/ROS 节点，未验证动态避障。

### 后续待处理

- 规划管理器现有调用仍使用 `dynamic=false` 默认值；若未来启用动态搜索，需要由调用方提供与动态障碍预测一致的 `time_start` 和时间基准。
- 碰撞检测目前仍使用不带时间的 `getInflateOccupancy(pos)`；时间分箱和四维哈希并不自动构成动态障碍物预测或避碰能力。
- 0.8 秒只是已有配置，尚未根据实际命令周期、运动基元和障碍物预测精度验证；若需调整应由用户确定。
- 同一 `[x,y,phi,time]` 栅格中的不同 `[u,v,r]` 仍可能合并；近终点航向环绕、地图第三维、Shot、后端等未适配问题沿用此前待办。

## 2026-11-06 · 020 · 无人船二维局部范围与近终点容差

### 用户要求

沿此前讨论的 `[x,y,phi]` 判断方式设置 `horizon=50 m`、位置容差 `25 m`、航向容差 `60°`。

### 文件与逐项修改

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：增加 `goal_pos_tolerance_` 和 `goal_yaw_tolerance_`，分别为米和弧度，保持其余参数的无成员初值声明方式。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()`：新增 `search/goal_pos_tolerance`、`search/goal_yaw_tolerance` 声明，默认均为 -1.0，实际值由启动配置提供。
- 同文件 `init()`：检查 `horizon_` 为正且有限、两个容差非负且有限；无效或未配置时显式报错，避免用 -1 造成错误终止行为。
- 同文件 `search()`：局部搜索范围改用当前位置与搜索起点的连续二维欧氏距离；近终点判定改用到目标的二维欧氏距离和 `atan2(sin(delta_phi),cos(delta_phi))` 得到的最短航向角误差。两个条件均使用 `<=`；范围使用 `>=`。删除已无用途的 `end_index` 及原共用索引容差。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：将 `search/horizon` 从 7.0 改为 50.0，新增 `search/goal_pos_tolerance=25.0` 米与 `search/goal_yaw_tolerance=1.0471975511965976` 弧度（60°）。其他启动方式须提供这两个非负容差参数。
- 更新本记录当前状态并追加本项；根据用户已明确说明撤回了记录 019 的代码修改，纠正文件顶部对当前时间索引状态的描述，保留历史记录原文。

### 数学关系与已有流程的衔接

```text
distance_from_start = sqrt((x-x_start)^2+(y-y_start)^2)
distance_to_goal    = sqrt((x-x_goal)^2+(y-y_goal)^2)
heading_error       = atan2(sin(phi_goal-phi),cos(phi_goal-phi))

reach_horizon = distance_from_start >= 50 m
near_end = distance_to_goal <= 25 m AND abs(heading_error) <= pi/3 rad
```

范围是相对本次起点的平面直线距离，不是累计航程。近终点判断使用连续状态，不依赖 0.25 m/5° 索引分辨率；359° 到 1° 的角误差为 2°。本次没有增加终端 `u,v,r` 的容差或改变返回状态。

### 静态与数学检查

Python AST 检查启动配置分别为 50、25、pi/3（误差小于 `1e-15`）；源码检查参数声明、初始化校验、二维距离、航向环绕及旧的索引容差移除；数学示例检查 50 m 范围边界、25 m 位置边界、359°/1° 环绕、59.9° 和 60.1° 航向。未编译或运行 C++/ROS 工程。

### 待后续处理

- `near_end` 满足后仍进入原 `computeShotTraj()`，同时位置容差现在达到 25 m；该 Shot 仍是原多项式模型，不能保证船舶动力学可行，且为零的 `time_to_goal` 会导致其中零除及采样循环问题。本次按范围未更改 Shot 或返回状态。
- 手动目标 `end_pt_(2)` 当前仍硬编码 1.0（历史高度），需要后续确保该值确为任务期望航向；否则 60° 阈值会相对错误的目标航向判断。
- `estimateHeuristic()` 仍以精确位置和航向计算时间，尚未扣除目标区域容差；外层 `lambda_heu` 仍为加权搜索，不能把当前估计视为到目标区域的严格 A* 下界。
- 碰撞查询仍将第三维视为地图高度，位置/时间/速度去重与后端适配沿用此前待办；不能仅凭判断修改认为整条船舶规划链已就绪。


## 2026-11-06 · 021 · 四段无人船控制输入终端射击

### 用户要求与决策

参照用户提供的旧航道跟踪射击实现，将 `computeShotTraj()` 从三次多项式改为四段恒定 `[v_cmd,w_cmd,0]` 控制的船模数值射击；可调参数按照已有搜索参数方式集中设置。用户最终选择试验命令范围 `v_cmd ∈ [-0.4,1.5]`、`|w_cmd|≤0.3`；三维占据地图查询暂用可配置固定高度 `z=0 m`。这些值不代表已验证的实船极限。历史消息中先选择过 `|w_cmd|≤0.7`，但后续明确改为 0.3，本次以最新决定为准。

### 修改文件和内容

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：声明 `shot_*` 参数、`propagateShot()` 和 `poseAcceleration()`；`PathNode::is_shot_segment` 标识需要按射击积分步长回放的段。删除不再使用的多项式系数、射击时间及旧终点速度成员。普通节点和四个射击节点共用原节点结构及父节点链。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()`：读取候选时间、数值求解、控制限幅、实际状态限幅、终点容差和地图切片等参数；非法或缺失的参数使射击返回失败并报错，不把实际速度上界当成命令边界。
- 同文件 `computeShotTraj()`：保留原签名，候选时间按配置依次尝试，每个时间分四段。逆动力学生成初值，使用 6×8 有限差分雅可比和阻尼 LDLT 求解更新，再按 1、0.5、0.25 尝试步长、逐段裁剪命令。终点五项容差全部满足后，以更密时间间隔重新积分，逐样本检查 `u,v,r`、膨胀占据地图并重新检查终点；成功时把四段作为节点连接到当前搜索节点并调用 `retrievePath()`。求解是局部方法，碰撞只在求解之后检查，不主动进行绕障优化。
- 同文件 `propagateShot()`：长射击段先按 `shot_integration_step` 分段，每段仍调用既有 `stateTransit()`，内部仍使用原有 `search/rk4_steps` 次 RK4；优化、检查及采样均使用同一船模，未改变动力学方程。
- 同文件 `search()`：原有的 25 m / 60° `near_end` 仍控制何时尝试射击；成功返回 `REACH_END`，失败后继续普通节点扩展；若已到 `horizon` 则回溯当前路径并返回 `REACH_HORIZON`。射击前检查节点池至少有四个空位。未采用旧独立工程 120 m 的连接范围或距离减小 0.3 m 的重复尝试抑制，避免覆盖用户已设置的近终点判断。
- 同文件 `getKinoTraj()`、`getSamples()`：从统一父节点链重建普通段和射击段，射击段调用 `propagateShot()`，不再读取旧多项式；将起终边界速度转换为 `[vx,vy,r]`。新 `poseAcceleration()` 由船模求 `[ddot x,ddot y,dot r]`，代替将输入命令直接当作位姿加速度；这是射击与原 B 样条采样接口衔接所需，不能保证后端其余部分已适配。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：加入参数：时长 `[24,36,48,64,80,100] s`，巡航种子 `1.1 m/s`，积分步长 `0.1 s`，碰撞采样 `0.04 s`；目标位置/航向/u/v/r 容差分别 `0.15 m/0.12 rad/0.08 m/s/0.06 m/s/0.04 rad/s`；命令范围 `[-0.4,1.5]` 和 `±0.3`，命令余量系数 0.85；实际状态上限 `|u|≤1.6, |v|≤0.8, |r|≤0.8`，地图高度 `0 m`；差分 `0.001`、阻尼 `0.02`、每个时长最多 16 次、每次射击总体预算 8 秒。除用户明确指定的命令范围和地图高度外，以上均为参考旧工程的可调整初值，非实船验证结论。由于 0.85 的余量裁剪，实际射击命令范围为 `v_cmd ∈ [-0.2575,1.3575]`、`w_cmd ∈ [-0.255,0.255]`。

### 验证

未编译、未启动 ROS 或本工程。使用 Python AST 检查启动文件和 21 个参数声明及最终命令上界；静态核对多项式成员移除、LDLT、节点回接、采样和地图查询。独立 Python 重写相同动力学与 RK4 对一个示例分别按 0.1 s 和 0.04 s 回放 12 s，最大状态差约 `5.4e-9`；航向跨 0° 包络和世界速度公式的示例通过。该独立数学检查没有执行 C++ 实现，不能作为工程编译、连接成功率、实时性或避碰的验证。

### 尚待处理

- 普通搜索扩展仍把 `[x,y,phi]` 当成地图三维位置查询，仍有历史静态搜索读取未初始化时间、动态哈希时间键等问题；射击失败回到普通搜索并不会自动修复这些问题。
- 碰撞仅查 `z=0` 切片的膨胀地图，是否覆盖实际船体和障碍物需后续验证；动态障碍预测及环境代价没有接入。优化和密集检查的积分切分不完全相同，末端重新检查降低误差影响，但不能保证采样之间无碰撞。
- 单次射击预算是 8 秒，并非整个 A* 搜索的总预算；多个候选节点可能多次尝试，实时性须测试。终点求解残差未按单位归一化。
- 原来 25 m/60° 是尝试连接的范围，新的精确终点容差是另一组值；仅在连接成功时返回 `REACH_END`。接近目标且未连接成功时继续搜索，和原 `NEAR_END` 提前返回行为不同。
- 搜索的三维轨迹后端仍有高度/航向和物理约束尚未适配，不能据此认为整个无人船系统可以安全运行。实际状态限值、步长、终点容差、候选时间及对接执行器都需结合船和地图确认。


## 2026-11-06 · 022 · 恢复搜索终止判断框架

### 用户要求

将 `search()` 中 `reach_horizon`、`near_end` 和找不到轨迹的返回判断恢复为原 Fast-Planner 框架，同时衔接记录 021 已改造的四段无人船终端射击。

### 文件、修改与衔接

- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：恢复 `if (reach_horizon || near_end)`、随后分别判断 `reach_horizon` 和 `near_end` 的结构。当 `near_end` 且节点池有四个空位时，重新计算时间估计并尝试射击；射击失败或者节点池空间不足时，回溯到当前普通搜索节点。
- 保留 `computeShotTraj()` 成功时自行将四段节点接到当前节点并回溯完整路径的设计。成功时不再重复调用 `retrievePath(cur_node)`；否则会在已有路径向量中重复追加、破坏节点顺序。射击失败才回溯普通节点路径。
- `reach_horizon` 优先判断：成功射击返回 `REACH_END`，否则返回 `REACH_HORIZON`。没有达到 horizon 而满足 near_end 时：成功射击返回 `REACH_END`；射击失败且当前节点有父节点，返回 `NEAR_END`；射击失败且当前节点是起点，返回 `NO_PATH`。当两者均不满足时继续扩展；开放集耗尽及节点池耗尽仍返回 `NO_PATH`。
- 近终点仍沿用已有二维位置和周期航向条件；船模、四段射击、容差及参数未变。原代码在首次搜索循环即尝试射击时输出错误日志；由于当前起点直接射击是合法情况，此次未恢复该误导性日志。
- `fast_planner/plan_manage/src/planner_manager.cpp` 未修改：它会对 `NO_PATH` 重试，`NEAR_END` 与 `REACH_HORIZON` 则继续进行轨迹采样和后端处理；这与现有管理器逻辑一致，但 `NEAR_END` 并不代表精确终点已达到。

### 检查

未在本机编译或运行工程。通过源码静态检查核对返回分支、射击成功与失败时只回溯一次，以及管理器对各状态的处理；使用独立真值表检查 horizon、near_end、射击结果和父节点组合。真值表检查是逻辑验证，不代表 C++/ROS 运行测试。

### 待后续处理

恢复原框架后，在 near_end 时射击失败不再继续扩展寻找其他连接起点；特别是起点已经位于 25 米、60° 范围内但射击失败时会返回 `NO_PATH` 并由管理器重试。普通扩展地图、时间索引、后端以及地图切片等未适配事项沿用记录 021。


## 2026-11-06 · 023 · 普通搜索扩展的无人船命令采样

### 用户要求

将 `search()` 中首次及普通扩展的输入生成改为无人船状态与输入语义，衔接现有 `[x,y,phi,u,v,r]` 船模和 `[v_cmd,w_cmd,0]` 输入。

### 逐项修改及原因

- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：保留首次扩展 `start_cmd_` 及原来的 `init_max_tau/20` 到 `init_max_tau` 的 20 个时长；第三项仍在函数入口归零。首次命令若非有限数或超出配置的命令范围，返回 `NO_PATH`，不偷偷裁剪反算命令；管理器现有逻辑随后可采用 `init=false` 重试。
- 同一函数普通扩展：删除三层 `ax/ay/az` 加速度循环。纵向与转向各分四个区间、五个等间距采样，生成 `[v_cmd,w_cmd,0]`。若纵向范围跨越零而等间距采样未包含零，额外加入 `v_cmd=0`。这样当前配置产生 6×5=30 个命令，第三项不再造成重复计算。普通扩展时长仍由 `max_tau` 及原有 `time_res=1` 生成，目前为 0.6 秒。
- 搜索入口检查命令上下限及两个时长有限且有效，避免无效参数或零步长导致错误搜索或无穷循环。
- 不新增第二套命令边界，普通扩展复用记录 021 的 `search/shot_min_v_cmd`、`search/shot_max_v_cmd`、`search/shot_max_w_cmd`。参数虽然名为 shot，但当前同时约束普通命令采样；普通扩展使用完整配置范围，射击仍单独使用 `shot_command_scale=0.85` 预留余量。

### 参数、计算与衔接

当前 `v_cmd` 样本为 `[-0.4, 0.075, 0.55, 1.025, 1.5, 0]`；`w_cmd` 样本为 `[-0.3,-0.15,0,0.15,0.3]`。每组固定命令传入既有 `stateTransit()`，传播状态 `[x,y,phi,u,v,r]`，其中前两项是控制命令，不是 `u` 和 `r` 本身。0 命令不是保证静止，因为动力学包含偏置和耦合。

### 检查

不编译、不运行 ROS。使用 Python AST 读取现有启动参数，并按源码公式独立检查样本数、边界、零值和第三项；静态检查初始命令路径保留、普通采样不再使用 `max_acc_`。未执行 C++ 搜索或验证轨迹质量。

### 未完成事项

- 普通扩展之后的速度检查仍把 `max_vel_` 同时用于 `u,v,r`，尚未与船体状态的独立限值统一；控制代价仍混合不同单位。按逐模块原则本次未修改。
- 普通扩展碰撞检查仍将航向当作地图第三维；静态时间索引等既有问题仍未处理。不能据此认为整条规划流程已适合实船。
- 目前首次反算命令超限时直接失败而非限幅；首次失败时管理器的重试可能丢失初始命令连续性。实际执行器命令范围与模型有效区间仍需验证。


## 2026-11-06 · 024 · 简化普通命令采样并与射击参数分开

### 用户要求

普通扩展使用基础、简洁且接近原 Fast-Planner 的代码，不依赖 `shot_traj` 的变量；解释此前写法与原始射击轨迹变量的关系。

### 修改内容

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：与其他搜索参数一样声明 `min_v_cmd_`、`max_v_cmd_` 和 `max_w_cmd_`，分别表示普通扩展的纵向命令下限、上限和转向命令绝对值上限。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()`：用 `declare_parameter` 读取 `search/min_v_cmd`、`search/max_v_cmd` 和 `search/max_w_cmd`，默认 -1.0，与现有参数声明风格一致。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：普通扩展新增独立参数，配置值仍为 `-0.4、1.5、0.3`；射击的 `search/shot_*` 参数不改变。两组值当前相同，但独立配置，不会自动同步。
- 同一 `.cpp` 的 `search()`：参数检查和初始命令检查由此前的 `shot_*` 成员改用新普通扩展成员。普通采样改成两层普通 `for` 循环，每项 5 个均匀样本，直接生成 `[v_cmd,w_cmd,0]`，去掉上一版临时向量、`std::none_of` 和 lambda。首次扩展和时间采样不变。
- 源代码旧 `coef_shot_`、`t_shot_` 是原三次多项式射击的结果，不是普通扩展的命令参数；上一版借用 `shot_*` 范围是为避免重复配置，但容易产生不必要的依赖，按用户要求取消。当前四段射击实现不使用原多项式系数，`is_shot_succ_` 仍用于记录射击成功。

### 行为变化及衔接

普通采样现在为 `v_cmd=[-0.4,0.075,0.55,1.025,1.5]`，`w_cmd=[-0.3,-0.15,0,0.15,0.3]`，共 25 组；与记录 023 相比，简化后没有额外补充 `v_cmd=0`。等间距采样对非对称范围不一定包含零；零输入也不保证船静止。是否需要单独加零值可后续由用户决定。输入仍交给 `stateTransit()`；未改变船模、四段射击或时间采样。

### 检查与待办

未编译或运行工程。Python AST 核对启动参数、声明，静态检查普通采样中无 `shot_`、无 lambda，公式检查 25 组采样。两组命令限值应共同维护；否则普通扩展与射击可使用不同限制。记录 023 的速度检查、碰撞及时间索引待办仍然存在。


## 2026-11-06 · 025 · 删除普通搜索入口的新增检查

### 用户要求

保留普通搜索命令范围及参数位置，删除 `search()` 入口此前新增的命令范围、时长、初始命令检查，使入口更接近原有框架。

### 修改及衔接

- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：删除普通命令范围及 `max_tau_`、`init_max_tau_` 的有效性检查，以及 `init=true` 时对 `start_cmd_` 有限性与范围的检查。保留 `start_cmd_(2)=0`，首次扩展仍直接使用 `start_cmd_`。
- 普通搜索的 `min_v_cmd_`、`max_v_cmd_`、`max_w_cmd_` 声明、读取、配置和采样不变；四段终端射击内部原有参数检查及其他算法不变。
- 此后首次扩展不会因为初始命令超出配置范围而在入口返回 `NO_PATH`，也不会自动限幅。普通采样的范围仍由配置决定；若未来修改参数，应确保命令边界和 `max_tau_`、`init_max_tau_` 有效，否则时长采样可能不正确或不终止。

### 检查

未编译或运行工程。静态检查确认两段检查已移除，命令范围配置、参数读取、第三项归零、首次输入及普通两层采样仍存在。记录 024 的其他说明仍适用，但入口检查部分以本记录为准。


## 2026-11-06 · 026 · 无人船普通扩展、终端射击与启发式速度参数

### 用户要求

将启发式 `max_ground_speed_` 改为 2.0 m/s；射击状态上限 `shot_max_u_`、`shot_max_v_`、`shot_max_r_` 分别改为 2.0 m/s、0.5 m/s、0.3 rad/s；普通扩展分别使用 `max_u_`、`max_v_`、`max_r_`，数值同为 2.0、0.5、0.3。

### 文件与修改内容

- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：和原搜索参数一起声明 `max_u_`、`max_v_`、`max_r_`；仅声明，不设置成员初值。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()`：分别通过 `search/max_u`、`search/max_v`、`search/max_r` 赋值，默认值均为 -1.0，保持既有声明风格。
- 同文件 `search()`：保留原先超限 `continue` 判断结构，`pro_state.tail(3)` 按 `[u,v,r]` 分别与 `max_u_`、`max_v_`、`max_r_` 绝对值上界比较，不再将 `max_vel_` 同时用于这三个不同物理量。该判断仅检查扩展段的终点，段中间是否超速尚未检查。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：为三个普通扩展参数配置 2.0、0.5、0.3；将 `search/shot_max_u/v/r` 调为同样的数值；将 `search/max_ground_speed` 改为 2.0。`search/max_yaw_rate` 保持 0.3。保留原有 `search/max_vel` 及相关参数，以免影响其他仍在使用的代码。

### 衔接及重要区别

普通扩展和射击分别使用各自的三个状态限值，虽然当前数值相同，但不是同一个参数，修改时需要分别调整。射击还按 `0.8 * shot_max_r_` 生成初值参考，新的初值参考限为 0.24 rad/s，最终射击轨迹限值仍为 0.3 rad/s。命令 `v_cmd/w_cmd` 的限制与实际速度 `u/v/r` 是不同参数。

启发式用的是实际平面速度模长的上界，而逐分量约束 `|u|≤2.0`、`|v|≤0.5` 在数学上只保证 `sqrt(u²+v²)≤sqrt(4.25)≈2.062 m/s`，不能单独保证上界为 2.0。若允许两个分量同时达到上限，当前启发式的 2.0 并非由这些约束保证的严格上界；具体可达速度需结合动力学、命令限值和轨迹验证。是否调整上界或额外约束实际平面速度由用户后续决定。

### 检查及待办

没有编译、启动 ROS 或运行工程。使用 Python AST 检查启动文件及七个目标参数值，静态核对成员声明、参数读取、普通扩展分别比较三个分量以及射击仍使用自身三个上限。其他启动方式若使用 KinodynamicAstar，需要提供新参数；未知环境和后端适配等待办不因本次修改而解决。

## 2026-11-08 · 027 · 调整首次扩展最长时长

### 用户要求与修改

- 将 `search/init_max_tau` 从 0.8 秒改为 2.0 秒，修改 `fast_planner/plan_manage/launch/kino_replan.launch.py` 的对应启动参数。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()` 仍用 `time_res_init=1/20` 生成首次扩展时长，因此候选为 0.1、0.2、…、2.0 秒，共 20 个；搜索从同一起点使用初始命令分别传播，最终选用的段时长取决于搜索结果。普通扩展 `search/max_tau=0.6` 不变。
- 用户说明纵向命令上限的手动修改以实际配置为准：当前普通扩展及射击的上限均为 2.0；当前普通扩展循环两个方向各五个样本，公式分母为 5，不包含各自正向上限，历史条目保留。

### 衔接、验证与待定事项

- 保持现有 `search/rk4_steps=10`；对最长 2.0 秒段整段传播时的 RK4 小步为 0.2 秒。碰撞检查 `search/check_num=5`，最长段的检查时刻相隔 0.4 秒；普通扩展碰撞检查仍将航向作为地图第三维，且速度约束只检查段终点。更长时长的积分精度和采样间碰撞覆盖尚未验证。
- Python AST 解析启动文件，检查首次和普通扩展时长、RK4 步数及碰撞检查数；按源码循环公式核对 20 个候选、首尾及间隔，均通过。未编译、未运行 C++ 或 ROS，未验证轨迹质量和实际船模精度。
- 无新增方案决定；地图、时间索引、后端及其他既有待办继续保留。是否调整积分或碰撞检查密度留待用户决定。

## 2026-11-08 · 028 · 调整搜索路径输出采样间隔

### 用户要求、文件与修改

- 按用户要求，将 `fast_planner/plan_manage/src/planner_manager.cpp` 中 `kinodynamicReplan()` 的 `getKinoTraj(0.01)` 改为 `getKinoTraj(0.1)`，将 `plan_data_.kino_path_` 的段内时间采样间隔改为 0.1 秒。
- 搜索器的 `getKinoTraj()` 仍保留起点及每段终点，段内状态沿既有船模传播；0.1 秒不是 RK4 内部积分步长。记录 027 的首次扩展候选时长及普通扩展时长不变。
- `kino_replan_fsm.cpp` 使用该路径绘制搜索结果；后端 `getSamples(ts, ...)` 单独采样，本次不改变。较稀疏的输出会影响路径显示细节；当前显示仍把第三分量航向作为空间高度。

### 验证与待定事项

- 静态检查确认调用参数为 0.1、原 0.01 调用已移除，`getSamples()` 调用和逐段终点加入逻辑保留。未编译或运行工程，也未验证可视化效果。
- 本项无新增方案决定；已有地图、后端和验证待办继续保留。

## 2026-11-08 · 029 · 管理器统一读取无人船速度上限

### 用户要求与修改

- 使用 `pp_.max_u_`、`pp_.max_v_`、`pp_.max_r_` 保存 `search/max_u`、`search/max_v`、`search/max_r`；三项参数在 `pp_.max_vel_` 同一初始化区读取，并按用户要求排在 `pp_.max_jerk_` 之后。
- `fast_planner/plan_manage/include/plan_manage/plan_container.hpp` 的 `PlanParameters` 声明三个成员，分别表示纵向速度、横向速度和偏航角速度上限。
- `fast_planner/plan_manage/src/planner_manager.cpp` 的 `FastPlannerManager::initPlanModules()` 在 `max_jerk_` 之后集中声明参数并赋值。当前 launch 值为 `max_u=2.0 m/s`、`max_v=0.5 m/s`、`max_r=0.3 rad/s`。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `KinodynamicAstar::setParam()` 用 `nh.get_parameter()` 读取管理器已经声明的三项参数，避免重复声明。`kinodynamicReplan()` 的初始采样间隔使用 `pp_.ctrl_pt_dist / pp_.max_u_`；`max_v_` 和 `max_r_` 暂作为管理器成员保存。

### 衔接、验证与待定事项

- `pp_.max_vel_` 仍由 `manager/max_vel` 提供，并用于原有 B 样条物理限值及其他路径；本项没有全局替换该参数。三个 `search/max_*` 仍用于 Kinodynamic A* 的逐分量状态约束。
- 静态检查确认参数集中声明位置、排列在 `max_jerk_` 之后、搜索器读取已声明值且不存在重复声明；当前 `ctrl_pt_dist=0.5`、`max_u=2.0` 时初始 `ts=0.25 s`。未编译、未运行 ROS 或验证后端轨迹效果。
- 启用 Kinodynamic A* 时需提供有效的正 `search/max_u`；当前 ROS 参数默认值为 -1.0，不适用于该除法。其他规划模式对新增成员的初始化需求及既有地图、后端适配事项仍待后续处理。

## 2026-11-08 · 030 · B 样条拟合航向采样连续化

### 用户要求与修改

- 按用户指定，在 `fast_planner/bspline/src/non_uniform_bspline.cpp` 的 `NonUniformBspline::parameterizeToBspline()` 中，先为 `bx/by/bz` 赋第 0 个采样值，再从 `i=1` 到 `K-1` 逐点赋值；在循环前加入航向展开注释。
- 每次读取新的 `bz(i)` 后，以前一个已展开的航向为参考，计算 `delta=bz(i)-bz(i-1)`，再赋值 `bz(i)=bz(i-1)+atan2(sin(delta),cos(delta))`；显式加入 `<cmath>`。只展开前 K 个位姿采样点，后四项起终速度及加速度保持原赋值方式。

### 衔接、验证与待定事项

- Kino 流程的第三维按用户决定为航向，现有 RK4 传播保持连续角；此处理使输入采样出现 ±pi 跳变时按相邻最短角差展开。该方法假定相邻样本的真实转角小于 pi；否则仅凭采样角度无法恢复真实转向和圈数。拟合及后端优化仍不保证船模动力学一致性。
- `parameterizeToBspline()` 还被其他规划调用复用；这些调用如果将第三分量当高度，相邻第三分量差达到或超过 pi 时会被按角度转换，可能改变拟合结果。当前按用户明确指定在此函数修改，其他流程是否改用独立接口留待后续决定。
- 源码核查确认第 0 点先赋值、循环仅覆盖其余采样点、边界导数循环保留；独立 Python 数学示例检查正反方向跨 ±180°、一周转向及连续输入，结果符合预期。未执行 C++，未编译或运行 ROS。


## 2026-11-09 · 031 · 调整采样分支代码格式

### 用户要求与修改

将 `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `getSamples()` 中，判断段终点、射击段和普通搜索段的 `if / else if / else` 拆分为多行，并为三个分支分别添加大括号，便于阅读。条件、执行语句及其顺序保持不变，与此前船模传播和射击回放的衔接不变。

### 检查及待办

源码核对三个条件分支及调用未变；没有编译或运行工程。此前地图、后端及轨迹数值验证等待办继续保留。


## 2026-11-09 · 032 · B 样条逐分量速度与加速度限制

### 用户要求

对 Kino 后端 `setPhysicalLimits()` 使用的全局位姿导数限值，分别设置 `|vx|,|vy|≤2.0 m/s`、`|r|≤0.3 rad/s`、`|ax|,|ay|≤0.5 m/s²`、`|dot_r|≤0.1 rad/s²`。

### 修改文件、函数与原因

- `fast_planner/plan_manage/include/plan_manage/plan_container.hpp`：新增四个管理器参数成员 `max_global_vel_`、`max_yaw_rate_`、`max_global_acc_`、`max_yaw_acc_`，用于全局位姿曲线的平面速度、偏航速度、平面加速度及偏航加速度限制，与船体 `max_u/v/r` 区分。
- `fast_planner/plan_manage/src/planner_manager.cpp` 的 `initPlanModules()`：读取四个 `manager/max_global_vel`、`manager/max_yaw_rate`、`manager/max_global_acc`、`manager/max_yaw_acc`，默认 -1.0。`kinodynamicReplan()` 构造速度与加速度限值向量，调用 `pos.setPhysicalLimits(velocity_limits, acceleration_limits)`。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：配置上述参数为 2.0、0.3、0.5、0.1。
- `fast_planner/bspline/include/bspline/non_uniform_bspline.h` 与 `src/non_uniform_bspline.cpp`：限值成员由单个 double 改为向量，新增两个 Vector3d 参数的 `setPhysicalLimits()` 重载。保留原标量接口，调用时将标量填充到所有曲线分量，以兼容既有其他流程。
- `checkFeasibility()`：三个导数分量分别比较对应上限。`checkRatio()` 与 `reallocateTime()`：按各分量 `|velocity_j|/velocity_limit_j`、`|acceleration_j|/acceleration_limit_j` 取最大比例，时间放大比例取速度比与加速度比平方根的较大者。时间节点调整框架、每步 1.1 的比例上限保持原有方式。比例日志同步说明输出为无量纲的超限比例。

### 与已有工作的衔接及限制

B 样条表示 `[x,y,phi]`，其一阶和二阶导数分别为 `[vx,vy,r]` 和 `[ax,ay,dot_r]`。本次限制的是各分量绝对值，不是二维速度/加速度模长，也不是船体 `dot_u/dot_v`。前端船体状态与命令限值不变。

本次适配的是优化后检查与时间调整；`BsplineOptimizer::calcFeasibilityCost()` 仍使用原 `optimization/max_vel/max_acc` 单一标量代价，尚未同步为这组分量限值。管理器仍最多尝试三次时间调整，达到次数上限后未拒绝未通过的曲线，不能声称最终轨迹一定符合限制。延长时间会改变起终导数和反算命令，仍需后续进行船模一致性与边界验证。其他启动方式若走 Kino 流程需要提供这四个新参数。

### 验证

未编译、未运行 C++ 或 ROS。Python AST 检查启动配置的四个参数；静态核对重载接口、调用、逐分量比较和无残留标量除法。独立数学例子检查只有偏航分量超限时的速度比 4/3、加速度比 2 以及理论统一放大比例 sqrt(2)，并检查相同分量限值时比例运算与原标量方式等价；不代表实际非均匀时间调整或规划轨迹已经运行验证。

## 2026-11-09 · 033 · 可行性代价复用无人船全局导数限值

### 用户要求

先将 `calcFeasibilityCost()` 的速度与加速度阈值改为此前确定的无人船全局参数，与 `setPhysicalLimits()` 对齐。暂不增加船体速度转换、命令约束或动力学残差。

### 文件、函数与修改原因

- `fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`：新增四个标量成员 `max_global_vel_`、`max_yaw_rate_`、`max_global_acc_`、`max_yaw_acc_`。与现有搜索参数一样只声明，避免在算法中写死限值。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()`：通过 `nh.get_parameter(...).as_double()` 读取已有 `manager/max_global_vel`、`manager/max_yaw_rate`、`manager/max_global_acc`、`manager/max_yaw_acc`。管理器在初始化优化器前已经声明这些参数，因此直接读取，避免重复声明，也不增加第二套启动参数。
- 同文件 `calcFeasibilityCost()`：保留原来的控制点一阶、二阶差分和循环。速度循环中 `j=0/1` 使用 `max_global_vel_`，`j=2` 使用 `max_yaw_rate_`；加速度循环中 `j=0/1` 使用 `max_global_acc_`，`j=2` 使用 `max_yaw_acc_`。仅当平方超限量大于零时加入原平方惩罚，原解析梯度公式与累加顺序不变。

### 与现有流程的衔接

当前速度分量上限为 `[2.0,2.0,0.3]`，加速度分量上限为 `[0.5,0.5,0.1]`，对应全局位姿曲线 `[x,y,phi]` 的导数；优化代价与记录 032 的检查和时间调整复用相同 ROS 参数。旧 `optimization/max_vel/max_acc` 的读取和成员暂保留，但本函数不再使用它们作为阈值；权重 `lambda3_` 未变。

### 实际验证

遵照要求没有编译或运行 C++/ROS。Python AST 核对启动参数；源码核对四项读取、分量选择和旧阈值从函数中移除。独立 Python 数学实现按新阈值计算代价及梯度，与中心有限差分对照，最大相对误差约 `9.94e-9`；零轨迹代价为零、恒定偏航速度 0.2 不超限而 0.4 超过 0.3 的示例通过。这些是数学与静态检查，不代表 C++ 实现运行、求解收敛或轨迹已验证。

### 后续事项

本项仅改变全局分量超限软代价，不保证优化结果严格满足上限。船体速度约束、输入命令限值、横向动力学一致性按用户要求暂不改；时间调整后的边界和可行性验证、地图适配等继续待后续处理。该优化器函数被不同流程共用，当前阈值都来自管理器的全局位姿参数；其他规划语义如需不同限值应后续另行区分。


## 2026-11-09 · 034 · 统一优化器参数读取写法

### 用户要求与修改

将 `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()` 中四项无人船全局限值读取，改为与相邻 `max_vel_`、`max_acc_` 一致的 `try_declare("参数名", -1.0)` 写法。四个参数名继续使用 `manager/max_global_vel`、`manager/max_yaw_rate`、`manager/max_global_acc`、`manager/max_yaw_acc`，复用既有配置，不增加另一套限值。

### 衔接、检查及待办

既有 `try_declare` 在参数未声明时声明参数，在已声明时捕获对应异常并读取已有值；当前管理器会先声明这些参数，多优化器实例也复用相同值。默认 -1.0 只是未配置占位值，仍需有效启动配置。源码核对四项读取方式、参数名和原有辅助函数，数值及可行性代价逻辑不变。未编译或运行工程；此前船模、地图及验证待办不变。

## 2026-11-09 · 035 · 偏航可行性代价权重参数

### 用户要求

保留当前未归一化的超限代价，为偏航角速度添加 1975.3 的权重，为偏航角加速度添加 625.0 的权重；通过参数配置，并在代价计算行后注释当前权重。

### 文件、函数与修改

- `fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`：增加标量成员 `yaw_rate_weight_`、`yaw_acc_weight_`。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()`：按既有 `try_declare` 写法读取 `optimization/yaw_rate_weight` 和 `optimization/yaw_acc_weight`，默认 -1.0 为未配置占位值，实际使用须提供正配置值。
- 同文件 `calcFeasibilityCost()`：沿用基础 if/else 和原循环；每个分量初始化局部权重为 1.0，仅 j=2 时使用偏航权重。速度代价为 `weight * vd²`，加速度代价为 `weight * ad²`；对应 `temp_v`、`temp_a` 也乘相同权重，使梯度与目标函数一致。两条代价累加语句后添加当前配置 1975.3、625.0 的注释，并注明实际以参数为准。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：增加上述两个参数，分别配置为 1975.3、625.0。其他限值、外层 lambda3_、船模与地图逻辑保持原样。

### 与已有参数的衔接

当前阈值速度为平移 2.0、偏航 0.3，加速度为平移 0.5、偏航 0.1。原超限平方代价在相同比例超限时随阈值的四次方变化，因此速度尺度补偿约为 `(2/0.3)^4≈1975.3086`，按用户决定取 1975.3；加速度补偿为 `(0.5/0.1)^4=625`。这些权重只平衡当前阈值下的同百分比超限代价，并非物理安全系数；阈值改变后需重新评估权重。仍属于软代价，船体速度和动力学一致性暂未加入。

### 检查

未编译或运行 C++/ROS。Python AST 核对参数值；独立数学实现将加权解析梯度与中心有限差分比较，最大相对误差约 3.11e-7。相同 10% 超限示例中，平移/偏航速度代价分别约 0.705600、0.705597，加速度代价均约 0.00275625。该检查仅验证数学公式与配置，不代表优化器运行效果或整条轨迹已满足约束。


## 2026-11-09 · 036 · 取消速度项加权并移动权重注释

### 用户要求与修改

用户明确要求 `temp_v` 与速度代价 `cost += weight * pow(vd, 2)` 不再乘权重，并将当前权重值的注释放在给实际计算权重赋值的行。

- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `calcFeasibilityCost()`：速度代价恢复 `cost += pow(vd,2)`，对应梯度系数恢复 `temp_v=4*vd*ts_inv2`，同步取消权重，保持速度目标函数与梯度对应。
- 将速度和加速度的当前权重注释分别移到 `weight=yaw_rate_weight_` 和 `weight=yaw_acc_weight_` 赋值行。偏航角速度参数及赋值按用户提到的注释位置保留，但当前不再参与速度代价或梯度，注释中明确说明。
- 本次按用户明确指到的速度项处理，未修改加速度代价和梯度中的权重：角加速度仍使用 625.0。两项参数和限值配置不变。

### 检查及后续事项

仅静态核对速度代价与梯度同时取消权重、注释已移动及加速度权重仍生效；未编译或运行工程。速度循环的局部 weight 当前赋值后未使用，可能产生编译器未使用变量告警；是否移除该保留赋值、或者重新启用角速度权重需用户后续决定。此前地图、船体速度和动力学一致性待办不变。


## 2026-11-09 · 037 · 撤销记录 036 的代码修改

### 用户要求与修改

用户要求撤销上次修改。将 `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `calcFeasibilityCost()` 恢复为记录 035 对应的实现：速度代价和 `temp_v` 同时乘以 weight；移除权重赋值行后新增的注释，将当前速度与加速度权重说明恢复到各自代价累加行。加速度代价、梯度与权重参数均保持记录 035 的设置。

### 检查及衔接

源码核对速度代价与梯度均恢复加权，注释位置恢复；未编译或运行工程。当前偏航角速度权重 1975.3、偏航角加速度权重 625.0 均参与可行性代价和对应梯度，记录 036 作为历史保留，以本条为当前状态。其他既有待办不变。

## 2026-11-09 · 038 · 航向平滑权重参数

### 用户要求与修改

增加航向平滑权重参数，当前取值 2.0。

- `fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`：声明标量成员 `yaw_smoothness_weight_`。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()`：按既有写法读取 `optimization/yaw_smoothness_weight`，未配置默认占位值为 -1.0。
- 同文件 `calcSmoothnessCost()`：保留三阶控制点差分及循环，平移分量平方项权重为 1.0，航向平方项乘 `yaw_smoothness_weight_`；对应 `temp_j(2)` 同步乘该权重，之后四个控制点的梯度分配不变。在代价计算处注明当前权重及实际以参数为准。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：配置 `optimization/yaw_smoothness_weight=2.0`。

### 衔接、验证及待办

整体平滑权重 `lambda1_=10.0` 继续作用于整项代价和梯度；新增参数仅调整航向相对平面位置的平滑惩罚，不是角速度或角加速度上限。其他已有可行性权重、船模和地图逻辑不变。优化器为共用类，第三分量当前按航向处理，其他仍以第三维为高度的流程须后续区分。

未编译或运行 C++/ROS。Python AST 核对参数配置为 2.0；独立数学实现将加权三阶差分的解析梯度与中心有限差分比较，最大绝对误差约 6.85e-10。该检查不代表求解收敛或实际轨迹效果已验证。权重的实际效果后续测试时再评估。


## 2026-11-09 · 039 · 航向平滑梯度注释

按用户要求，在 `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `calcSmoothnessCost()` 中，给 `temp_j(2) *= yaw_smoothness_weight_` 添加行尾注释，说明航向损失加权后梯度应乘相同权重，并非再次累加损失。仅增加注释，算法与既有参数衔接不变。已核对源码，未编译或运行工程；此前待办不变。

## 2026-11-09 · 040 · 障碍距离代价暂按平面位置计算

### 用户要求

`calcDistanceCost()` 暂不考虑航向，仅计算 x、y 方向的障碍代价，并注明航向后续再处理；安全距离 `dist0` 改为 2.0 m。

### 文件、修改及原因

- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `calcDistanceCost()`：将原 `q[i]=[x,y,phi]` 直接查询三维地图的方式改为构造 `map_pos=[x,y,0]`，固定 z=0，与用户此前接受的临时地图切片一致，避免将航向当高度。查询后先将距离梯度第三项清零，再沿用原梯度归一化方式；仅向控制点 x、y 分量累加梯度，第三项保持初始化的零值。相关行添加固定高度、航向暂不参与、船体轮廓与航向避障影响后续处理的注释。仅此障碍代价忽略航向，航向变量和平滑及可行性项仍保留。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：`optimization/dist0` 从 0.4 改为 2.0 m。代价仍为距离低于阈值时的 `(dist-dist0)^2`，外层 lambda2 不变。

### 衔接、检查及待办

未编译或运行 C++/ROS。Python AST 核对 dist0 配置为 2.0，静态检查固定高度查询、归一化前清零第三梯度以及仅累加 x/y 梯度。当前仍调用三维距离场在 z=0 的切片，非完整二维障碍投影，不能保证覆盖所有高度的障碍物。原梯度归一化是保留的启发式处理，不是对任意距离场插值的精确梯度保证。控制点代价也不能代替整条曲线碰撞检查；船体尺寸、航向影响、地图完整适配和测试仍待后续按用户决定进行。普通前端扩展的地图查询未因本项自动适配，开始测试前仍需提醒处理地图。

## 2026-11-10 · 041 · 终点最短航向误差与独立权重

### 用户要求

临近终点只要求位置与最终朝向一致，不希望因为航向数值相差整圈而要求额外转动；路径过程中因其他原因需要的转动可以接受。按用户确认修改终点航向误差并添加权重参数。用户未指定数值，本次先配置权重 1.0 作为可调初值，不沿用速度/加速度的尺度补偿权重。

### 文件、函数与修改

- `fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`：新增标量成员 `yaw_endpoint_weight_`。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()`：通过既有 `try_declare` 读取 `optimization/yaw_endpoint_weight`，默认 -1.0 为未配置占位值。
- 同文件 `calcEndpointCost()`：保留最后三个控制点的 1/6、4/6、1/6 组合及终点参考来源，计算误差后将第三项赋为 `atan2(sin(dq(2)),cos(dq(2)))`。位置误差权重 1.0，航向误差平方乘新权重。对应梯度先计算 `2*dq`，只给第三项乘航向权重，再按原系数分配给最后三个控制点。增加整圈等价及梯度说明注释，显式包含 `<cmath>`。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：增加 `optimization/yaw_endpoint_weight=1.0`，整体终点权重 `lambda4=0.01` 不变。

### 衔接与边界

已有 B 样条采样航向展开保持不变，曲线仍保存连续航向；仅终点代价比较最终朝向，0、2pi、4pi 等价。未添加过程转圈惩罚，也未改动控制点固定方式。终点代价仍只在 ENDPOINT 启用时计算，当前 Kino 搜索未返回 REACH_END 时才启用；REACH_END 时最后三个控制点仍固定。此修改防止该终点代价按整圈差异施加额外惩罚，不保证由其他代价、轨迹初始化或执行器造成的所有转圈都消失。平方最短角差在相差 pi 的分界不可导，该处梯度存在方向歧义，不能声称处处光滑或全局收敛。

### 验证

未编译或运行 C++/ROS。Python AST 核对新参数及配置；独立数学示例检查 720°/0°、359°/0°、-179°/179° 的最短误差，分别为约 0、-1°、2°。在远离 pi 分界的样本上，采用非单位权重检查解析梯度与中心有限差分，最大绝对误差约 7.78e-10。仅验证公式与配置，不代表实际末端轨迹或规划已运行验证。地图、船体约束及动力学一致性等待办继续保留。


## 2026-11-10 · 042 · 终点权重对应 1 米与 30 度误差

### 用户要求、文件与修改

用户要求终点 1 m 的位置误差与 30° 的航向误差产生相同代价。`fast_planner/plan_manage/launch/kino_replan.launch.py` 将 `optimization/yaw_endpoint_weight` 从 1.0 改为 `1/(pi/6)^2=36/pi²≈3.6475626111241605`；`fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `calcEndpointCost()` 中代价行尾注释同步说明该对应关系。参数读取、最短角差、梯度与外层 lambda4 不变。

### 衔接和验证

平面位置代价为 `ex²+ey²`，因此此处 1 m 指平面位置误差模长为 1 m；若 x、y 各误差 1 m，则位置代价为 2。原权重 1.0 时，1 m 对应 1 rad≈57.3°。Python AST 检查参数，并验证新权重乘 `(pi/6)^2` 约为 1，未编译或运行工程。实际收敛、终点精度及其他既有待办仍需后续验证。

## 2026-11-10 · 043 · 引导与途经点代价的无人船航向适配

### 用户要求与判断

用户要求 `calcGuideCost()`、`calcWaypointsCost()` 按终点代价进行类似的无人船适配。在参考数据明确为 `[x,y,phi]` 时，该处理合理；若只有平面路径或第三项仍为高度，不应直接作为航向参考。本次保持两项当前 Kino 未启用的状态，仅修改共用函数与参数。

### 文件、函数与修改

- `fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`：新增标量 `yaw_guide_weight_`、`yaw_waypoint_weight_`。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()`：按既有 `try_declare` 写法读取 `optimization/yaw_guide_weight`、`optimization/yaw_waypoint_weight`，默认 -1.0 为未配置占位值。
- 同文件 `calcGuideCost()`：保留控制点与对应引导点的比较方式，误差第三项采用 `atan2(sin(delta),cos(delta))`；代价改为 x/y 误差平方加独立权重乘航向误差平方，梯度第三项同步乘权重。注释说明参考点第三项必须是航向及当前权重含义。
- 同文件 `calcWaypointsCost()`：保留三个控制点的 1/6、4/6、1/6 曲线采样关系；对参考途经点采用最短航向误差、独立航向权重和相应梯度，位置分量权重仍为 1.0。
- 同文件 `isQuadratic()`：三维且含 GUIDE 或 WAYPOINTS 时返回 false，避免在加入周期角差后仍把相应组合认作全局二次目标；沿用现有的一般代价求解器配置，低维原判断保留。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：两个新权重均配置为 3.6475626111241605，沿用用户终点的“1 m 平面位置误差与 30° 航向误差同代价”关系。两项参数独立于终点权重，可后续分别调整。外层 lambda5、lambda7 和原启用标志不变。

### 衔接及限制

当前 Kino 正常优化不启用 GUIDE/WAYPOINTS，因此本项不会自动增加其代价项。其他 Topo/偏航等流程存在调用共用函数的路径，其中三维数据来源尚未全面适配：后续启用时必须确认参考点第三项为 phi，不是 z，也不能在缺少航向时擅自补零。引导代价仍作用于控制点，不是直接约束曲线在同一时刻经过引导点；途经点代价仍为软约束。最短角差在 pi 分界不可导、存在多解，不保证所有转圈或收敛问题自动消失。地图、船体速度与动力学一致性等既有待办继续保留。

### 验证

没有编译或运行 C++/ROS。Python AST 核对两项参数和数值；独立数学实现检查整圈角度偏移前后代价一致，并在远离 pi 分界的样本上用中心有限差分核对加权梯度：引导项最大绝对误差约 1.73e-9，途经点项约 9.12e-10。仅验证公式与配置，未验证求解器或实际轨迹。

## 2026-11-10 · 044 · 航向控制点调整范围设为 ±60°

### 用户要求

将 `BsplineOptimizer::optimize()` 的航向变量调整范围与位置分开，用户确定先采用相对初始航向控制点 ±60°。

### 文件、函数与修改

- `fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`：声明标量 `yaw_bound_`，单位 rad。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `setParam()`：按既有写法读取 `optimization/yaw_bound`，默认 -1.0 为未配置占位值。
- 同文件 `optimize()`：保留原 `dim_!=1` 边界设置和基础 for 循环。展开变量在三维时依次为 x、y、phi，使用 `dim_==3 && i%dim_==2` 识别航向分量；这些分量采用 `q[i]±yaw_bound_`，其余仍为 `q[i]±10.0`。增加单位与相对初值的注释。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：配置 `optimization/yaw_bound=1.0471975511965976`，即 pi/3 rad。

### 衔接、验证及待办

该参数限制的是每个参与优化的航向控制点相对本次初始值的变化，不是航向绝对范围、角速度限制、相邻控制点角差或过程转圈限制；已有连续航向不强行截回 [-pi,pi]。位置变量仍允许 ±10 m，固定端点控制点的规则及所有代价不变；一维原来不设置 box 边界的行为保留，二维仍使用 ±10。

未编译或运行 C++/ROS。Python AST 检查配置值与 pi/3 一致，独立检查展开变量索引和边界宽度，并验证初始航向为 4pi 时仍围绕 4pi 设置边界。该检查不代表此范围足以解决所有场景，也不保证轨迹满足船模；实际效果、地图适配及动力学验证等仍待后续处理。其他启动方式若调用三维优化器，需要配置有效的正 `optimization/yaw_bound`。


## 2026-11-10 · 045 · 仿真阶段再评估优化求解预算

用户要求记住：当前尚未测试，暂不调整 B 样条求解时间和目标函数计算次数；后续开始仿真时提醒评估参数。已在本文件顶部加入仿真测试提醒，与既有地图适配提醒放在一起，便于换电脑后继续。

当前已知配置为 Kino 调用 `BsplineOptimizeTraj(...,1,1)`，最多 300 次目标函数计算，求解时间预算 0.005 s。测试时结合实际耗时、次数、停止原因及轨迹可行性调整，不预先选择未经验证的新数值。本次只更新记录，未修改算法或启动参数，也未编译、运行或进行仿真。

## 2026-11-10 · 046 · 轨迹接收回调适配统一无人船位姿

### 用户要求

先将 `traj_server.cpp` 的 `bsplineCallback()` 适配无人船统一 `[x,y,phi]` B 样条；显示用的 `odomCallback()` 和 `traj_real_` 暂不修改。

### 文件、函数及修改

- `fast_planner/plan_manage/src/traj_server.cpp` 的 `bsplineCallback()`：控制点前三项解释为 x、y、phi，明确注释消息 `pos_pts.z` 暂携带航向（rad）而非高度。保留现有三维矩阵、时间节点读取、开始时间、编号和持续时间；显式使用消息 knots 重建曲线以保留时间重分配。删除独立 `yaw_pts/yaw_dt` 解析及独立 yaw 样条构造，避免 Kino 已不提供这些字段时仍构造空曲线。`traj_` 仅保存三条曲线：位姿 `[x,y,phi]`、其导数 `[vx,vy,r]`、二阶导数 `[ax,ay,dot_r]`。
- 同文件 `cmdCallback()`：为衔接缩减为三项的 `traj_`，将正常段和轨迹结束分支中原先读取 `traj_[3]/traj_[4]` 改为从 `pos(2)`、`vel(2)` 提取航向和偏航角速度，避免越界。轨迹结束分支沿用原零速度、零加速度参考，故该分支 yawdot 现在也为零；这不是已实现的船舶物理停车策略。
- `odomCallback()`、`traj_real_`、`replanCallback()`、`newCallback()` 及消息定义没有修改。消息发布端当前已将航向写在 pos_pts 第三项，此改动与其一致。其他仍发送传统 `[x,y,z]` 加独立 yaw 的流程，不能直接使用新的接收语义，后续需另行区分。

### 检查及后续工作

没有编译或运行 C++/ROS。静态检查确认回调只保存三条曲线、保持消息 knots、无 `msg->yaw_pts/yaw_dt` 读取以及无 `traj_[3]/traj_[4]` 访问；修改前后对照确认 `odomCallback()` 内容相同。

`cmdCallback()` 的实际输出消息仍为原 `quadrotor_msgs::PositionCommand`，其空间 z、velocity.z、acceleration.z 目前仍会携带 phi、r、dot_r；显示也仍可能将航向作为高度。控制接口、显示映射、轨迹结束/重规划处理、开始时间与实际起始状态对齐均需按后续步骤适配，不能据此认定 traj_server 已能正确控制无人船。船体速度与输入反算未在此处接入；地图测试及优化预算提醒继续保留。


## 2026-11-10 · 047 · 重规划旧轨迹延续时间设为 3 秒

### 用户要求与修改

用户在讨论旧轨迹延续窗口后要求先按 3 处理，本次将其理解为 3.0 秒。`fast_planner/plan_manage/src/traj_server.cpp` 的 `replanCallback()` 将 `time_out` 从 0.01 改为 3.0，增加注释说明这是重规划通知后旧参考轨迹最多继续的时间，不是停车时间。其余时间截断公式和控制流程保持原样。

### 衔接、检查及待办

新轨迹提前到达时按原逻辑切换，旧轨迹剩余不足 3 秒时不延长原终点。该值不限制规划器计算时间，也不生成无人船减速轨迹。正常滚动重规划和障碍触发目前仍使用同一空消息和回调，因此障碍触发也会采用该窗口，尚未根据碰撞风险区别处理。窗口结束后的原零速度参考不能代替实际停车策略，此部分仍待用户后续决定。

静态核对回调数值、min 截断及注释；未编译、未运行或仿真验证，3 秒是用户指定的暂定值，不代表已验证安全。

## 2026-11-10 · 048 · 轨迹参考输出与显示的无人船适配

### 用户要求与实施范围

根据上一轮对 `cmdCallback()` 的分析实施适配：先接通现有统一位姿曲线的参考输出和显示，处理重复发布及负时间分支；实际无人船控制器消息、命令接口和停车策略尚未确定，本次保留现有 PositionCommand 参考接口，不擅自增加执行器控制方案。

### 文件、函数与修改

- `fast_planner/plan_manage/src/traj_server.cpp` 的 `cmdCallback()`：正常段仍从三条曲线读取 `[x,y,phi]`、`[vx,vy,r]`、`[ax,ay,dot_r]`，phi 与 r 分别用于 yaw、yaw_dot。保持连续航向，不改变为船体速度。
- 输出 PositionCommand 时，空间位置 z、速度 z、加速度 z 分别设为 0，避免将 phi、r、dot_r 当作竖直运动。x/y 位置及全局导数照常输出；现有消息没有对应角加速度字段，dot_r 未在该消息中单独发布，后续控制器接口再处理。
- 单独构造 `display_pos=[x,y,0]` 用于箭头和期望路径显示；方向仍由 phi 计算。位姿 pos 本身保持 [x,y,phi]，不修改实际里程计的显示记录。
- 删除重复发布，只保留一次 `pos_cmd_pub->publish(cmd)`。时间不在有效分支时记录日志后立即返回，避免继续使用未赋值局部变量。所有新增分支使用大括号。
- 移除本回调中已不参与输出的前瞻点、pos_err 及注释掉的“朝前瞻位置确定 yaw”旧逻辑，避免与直接跟踪 phi 混淆。轨迹结束仍沿用固定末端与零导数参考，明确注释它不是船模减速策略。

### 衔接、检查与待办

本项与记录 046 的三条统一曲线一致。源码对照确认 `odomCallback()`、`replanCallback()` 未改变，3 秒窗口不变，消息定义和订阅话题不变。静态检查一次回调仅一处发布、三个空间 z 字段为零、yaw/yaw_dot 映射、显示位置分离、异常时间返回及无旧独立 yaw 索引。没有编译、运行 C++/ROS 或仿真验证。

输出仍依赖 HAVE_QUADROTOR_MSGS，未提供该依赖时仍不会发布控制参考；最终是否采用无人船专用参考消息、是否发送船体速度及角加速度、如何实现闭环控制由用户后续决定。当前未直接产生 [v_cmd,w_cmd]。轨迹开始时间与切换状态、障碍触发重规划策略、实际停车、地图及仿真预算等待办继续保留。该节点现在按无人船平面语义输出，不再与旧三维无人机消息消费方式完全等价。


## 2026-11-10 · 049 · 保留上次参考航向赋值

按用户要求，在 `fast_planner/plan_manage/src/traj_server.cpp` 的 `cmdCallback()` 中恢复 `last_yaw_ = yaw`，并注释保留当前参考航向供后续使用；不恢复此前已注释掉的前瞻点航向计算或重复发布。`pos`、`yaw` 后续仍用于显示，因此不需要在无 quadrotor_msgs 的分支中通过 `(void)` 抑制未使用警告；现有 `(void)` 语句保持不变。

静态核对赋值位于航向求值之后和显示之前，未编译或运行工程。其他既有待办不变。

## 2026-11-10 · 050 · EXEC_TRAJ 普通重规划阈值适配无人船

### 用户要求

接近目标时，位置误差小于 5.0 m 且航向误差小于 15°，暂不触发普通重规划；离当前轨迹起点的平面距离小于 4.0 m 时，暂不触发普通重规划。

### 文件、函数与修改

- `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：`EXEC_TRAJ` 中的目标距离、起点距离改为只计算 x/y；目标航向误差使用 `atan2(sin(delta_phi), cos(delta_phi))`，按最短角差判断，避免米与弧度混算及跨 ±pi/整圈误差。近目标条件使用位置与航向同时满足；近起点仅检查平面距离。新分支使用大括号。
- 同文件 `init()`：新增 `fsm/thresh_no_replan_yaw` 参数读取，声明默认值保持既有风格 -1.0。
- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：声明 `no_replan_yaw_thresh_`，注明 rad；既有距离成员注明 m。
- `fast_planner/plan_manage/launch/kino_replan.launch.py` 与兼容启动配置 `kino_algorithm.xml`：`fsm/thresh_replan=4.0`、`fsm/thresh_no_replan=5.0`、`fsm/thresh_no_replan_yaw=0.2617993877991494`（15°）。不修改 Topo 配置。

### 与此前改动的衔接及待办

沿用 `[x,y,phi]` 轨迹，仍对当前参考轨迹采样位置进行判断，不改为里程计实际状态。距离和航向阈值用于抑制普通重规划，不是任务到达判据，也不替换搜索器 25 m/60° 的终端射击触发阈值。两个抑制条件依次独立生效：即使目标航向未满足，仍可能因离轨迹起点小于 4 m 而暂不重规划。轨迹到时直接清除目标的分支、REPLAN_TRAJ、碰撞检查、3 秒旧轨迹窗口及规划时间信息提交逻辑保持原样，其前次检查所指出的问题仍待后续决定。

### 实际检查

通过 Python AST 解析启动文件、XML 解析并核对两处参数一致；静态核对成员声明、参数读取、二维距离与周期角条件。独立数学检查覆盖 4.9 m/14° 满足、4.9 m/16° 不满足、5.1 m/0° 不满足、整圈角等价、179°/-179° 最短差 2° 及近起点 3.9/4.1 m 两侧。保持原严格小于运算，阈值等号不满足对应抑制条件。未编译工程、未运行 C++/ROS 或仿真；数值检查不能代替实际状态机测试。地图适配和仿真优化预算提醒继续保留。

## 2026-11-10 · 051 · 目标附近航向阈值改为 30°，旧轨迹延续窗口改为 1 秒

### 用户要求与修改

- `fast_planner/plan_manage/launch/kino_replan.launch.py` 和 `kino_algorithm.xml`：`fsm/thresh_no_replan_yaw` 从 15° 改为 30°，实际配置值为 0.5235987755982988 rad；同步 `src/kino_replan_fsm.cpp` 的 EXEC_TRAJ 注释。位置阈值仍为目标附近 5 m、起点附近 4 m，条件逻辑与记录 050 相同。
- `fast_planner/plan_manage/src/traj_server.cpp` 的 `replanCallback()`：`time_out` 从 3.0 改为 1.0 s，同步注释。仍通过 min 截断旧轨迹，不延长原持续时间；新轨迹提前到达时仍可提前替换。该窗口不是规划计算超时或动力学停车时间，正常/障碍重规划仍共用。

### 检查与衔接

通过 Python AST 解析启动文件、XML 解析及数值一致性检查；独立数学核对 29°/31°、整圈等价及 1 秒截断算例；源码检查 time_out 和 min 公式。未编译、未运行 C++/ROS 或仿真。

用户同时询问“成功后提交轨迹，失败时保留旧信息”的实施方法，本次仅分析，未改管理器。当前可用最小修改方案是在 kinodynamicReplan() 开始处用局部变量保存规划起始时间，取消提前写 local_data_.start_time_，并在函数末尾与新 position_traj_ 一起提交该时间、调用 updateTrajInfo()；现有返回 false 的路径应不改变旧 local_data_。保存的时间应为原规划开始时刻，不能简单替换为规划结束时刻；轨迹实际切换状态和时间对齐仍需另行设计。当前函数也没有因三次时间调整后仍不可行而返回 false，此成功判据不在本次改动范围。地图与仿真预算待办继续保留。

## 2026-11-10 · 052 · 新轨迹成功后再提交起始时间

### 用户要求

按记录 051 讨论的最小方案实施：规划开始时间暂存在局部变量中，成功后与新轨迹一起提交；失败时保留旧轨迹及其时间信息。

### 文件、函数与修改

仅修改 `fast_planner/plan_manage/src/planner_manager.cpp` 的 `FastPlannerManager::kinodynamicReplan()`：

- 原搜索前的 `local_data_.start_time_ = clock_->now()` 改为局部 `rclcpp::Time plan_start_time = clock_->now()`，添加中文注释解释失败时保留旧时间。
- 函数末尾保存 `local_data_.position_traj_ = pos` 后，再写入 `local_data_.start_time_ = plan_start_time`；继续使用既有 `updateTrajInfo()` 更新导数、起点、持续时间及编号。

### 衔接与实际验证

源码差异核对确认只改以上两处，函数外代码未改变。静态检查确认函数两处显式 `return false` 都在 local_data_ 写入之前，局部时间在搜索前取得，成功分支依次提交位置曲线、起始时间，调用 updateTrajInfo() 后返回 true。未编译、未运行 C++/ROS 或仿真。

保留的是规划开始时刻，不是规划完成时刻；不改 updateTrajInfo() 接口、搜索/优化/时间调整、失败状态机跳转或既有成功判据。此项修复失败时旧轨迹起始时间被提前覆盖的问题，不代表多线程原子提交，也不处理异常回滚；实际新旧轨迹切换连续性、1 秒窗口后规划未完成时的处置、轨迹到时与实际到达的区分仍待后续确定。目标附近 5 m/30°、起点附近 4 m、time_out=1.0 s 均保持记录 051 的设置，地图和仿真预算提醒继续保留。

## 2026-11-11 · 053 · 普通命令采样覆盖两端与平面安全检查适配

### 用户要求及范围

根据用户四项要求，修改普通搜索命令采样、状态机目标碰撞查询、目标附近重新选点和管理器轨迹碰撞查询。用户另选中了 PositionCommand/发布宏说明，但未明确其与四项编号的对应，已询问输出接口范围；本次未改 traj_server、消息或 CMake，不自行选择控制输出方案。

### 文件、函数与修改

- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 search()：保留每路 5 点、共 25 组普通命令。采样除数由 res 改为 res-1，覆盖上下限；当前 res=5，j=res/2 时显式将 w_cmd 置零。现有配置得到 v_cmd=[-0.4,0.2,0.8,1.4,2.0]、w_cmd=[-0.3,-0.15,0,0.15,0.3]。初次 start_cmd 扩展、时长采样及参数上下限不变。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp` 的 checkCollisionCallback()：目标查询改为临时地图点 [end_x,end_y,0]。目标附近候选点删除 dz/nz/new_z 遍历，仅保留原 x/y 环形采样；goal 从 end_pt_ 复制，只更新前两项，保留原目标航向。所有候选地图点使用 [x,y,0]，不把 phi 作为高度。
- `fast_planner/plan_manage/src/planner_manager.cpp` 的 checkTrajCollision()：仍从 [x,y,phi] 轨迹取样，另构造 [x,y,0] 查询 EDT；前视半径改为 x/y 差的范数；碰撞日志只输出 x/y，避免将 phi 标成空间坐标。6 m 前视半径、0.02 s 检查间隔、0.1 m 碰撞阈值不变。

### 衔接与实际验证

仍沿用三维地图接口，因此 z=0 是固定切片，并非已将地图改为二维或对全部高度投影；与此前后端障碍代价和当前 shot_map_height=0 设置一致。状态向量第三项及目标航向均保留，仅不参与本次平面碰撞位置和距离计算。

源码差异检查确认只改上述三份源文件的指定区域。静态检查采样端点除数与零转向分支、目标查询点、候选航向保留、无高度遍历及平面前视范数；独立数学检查两端、转向对称与零值、25 个组合，以及改变 phi 不改变地图查询位置/平面距离。未编译、未运行 C++/ROS 或仿真，检查不代替实际碰撞测试。

普通搜索扩展中仍将 state.head(3) 直接送三维占据查询，地图越界策略、船体轮廓、首次有效轨迹保护、动态时间语义、最终可行性验收等既有待办不在本次范围。后续完整地图适配及仿真求解预算提醒继续保留。

## 2026-11-11 · 054 · 安全轨迹检查参数按用户指定调整

### 用户要求与修改

用户指定前视距离先取 10 m、沿轨迹采样时间间隔 0.5 s、危险距离触发阈值 1.5 m。

- `fast_planner/plan_manage/src/planner_manager.cpp` 的 checkTrajCollision()：前视半径从 6.0 改为 10.0；fut_t 初值及循环增量均从 0.02 改为 0.5；危险距离条件由 dist<0.1 改为 dist<1.5。保留平面距离、z=0 查询及原判断框架。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp` 的 checkCollisionCallback()：结合上轮关于统一安全距离的讨论，将目标调整触发 dist<=0.3 和候选接受 max_dist>0.3 同步为 1.5 m，避免轨迹检查拒绝但目标选点仍接受同一近障碍位置。目标平面选点步长、范围、航向保留和安全定时器 50 ms 不变。

### 衔接、检查与未完成事项

10 m 为当前参考位置到未来轨迹采样点的平面直线距离阈值，与激光雷达量程、地图有效范围、search/horizon=50 m 相互独立，不会扩大传感器或地图的已知区域。0.5 s 是一次检查中未来轨迹的采样间隔，不是安全回调周期；按 2 m/s 估算点距约 1 m，较原采样粗，可能漏掉采样点之间的局部风险，不能保证连续曲线安全。当前循环仍先检查点再更新 radius，保留约 10 m 范围语义；实际速度模长上界尚未由后端严格统一约束。

静态核对初值和增量一致、前视阈值、三处障碍距离阈值；未编译、未运行 C++/ROS 或仿真。这些是用户指定的暂定值，非实测安全结论。地图与船体轮廓、越界策略、最终连续轨迹验收、首次有效轨迹保护及计算超时等既有待办继续保留；优化预算提醒不变。

## 2026-11-11 · 055 · 轨迹碰撞前视距离改为 25 m

### 用户要求与修改

`fast_planner/plan_manage/src/planner_manager.cpp` 的 checkTrajCollision()：仅将循环前视半径阈值从 10.0 改为 25.0 m。沿轨迹采样初值/步长继续为 0.5 s，危险距离继续为 1.5 m，search/horizon 仍为 50 m，其他函数未修改。

### 验证、衔接与待办

静态检查确认半径阈值已为 25.0，0.5 s 和 1.5 m 保留；未编译、未运行 C++/ROS 或仿真。25 m 为参考轨迹点相对当前参考点的平面距离，不代表传感器或地图有效覆盖范围。当前 launch 的点云局部更新范围 x/y 为 5.5 m，地图总大小默认 40x20x5 m；部分查询会超出已有效更新范围或地图边界，现有 getDistance() 对越界索引会夹到边界，不能将其结果当作实际外部空间距离。

用户同时询问地图接口关系、话题和静态栅格接入方式。本次仅阅读并解释：setEnvironment/setMap 传递共享对象，evaluateCoarseEDT 查询已有距离场；标准地图由点云/深度更新，当前无 OccupancyGrid 订阅。静态地图后端、分辨率/原点/坐标系、未知/越界策略、占据与距离梯度一致性、地图就绪门控等实施方案尚未批准，未接入静态地图或修改地图模块。完整地图适配和仿真预算提醒继续保留。

## 2026-11-11 · 056 · 接入面向大范围无人船航行的静态二维栅格地图

### 用户要求与已确定决策

在 Fast-Planner 框架上按原点云接收/更新模式增加 OccupancyGrid 订阅及回调，适配约 500m x 500m 地图，生成二维距离场和梯度供前端碰撞、后端优化及安全检查使用，减少上层调用改动。用户先回复地图 map/里程计 odom-world 需 TF，随后明确改为“规划里程计先在 map 中，后续再改”；本次按最新决定统一 map，不实现 TF。未知视为障碍、地图外不可通行。

### 文件、函数与修改原因

1. `fast_planner/plan_env/include/plan_env/sdf_map.h`：在原 SDFMap 增加静态模式/就绪标志、OccupancyGrid 订阅和消息缓存、二维膨胀图与 float 距离图、新方法声明；原占据/距离/索引/边界接口增加二维分支。Vector3d 类型保留，地图分支只用 x/y；行主序索引、原点 yaw、越界占据/非正距离一并适配。旧三维分支保留。
2. 新增 `fast_planner/plan_env/src/sdf_map_grid.cpp`（同一个 SDFMap 类的实现）：
   - initGridMap() 读取静态地图参数，创建 Reliable/Transient Local 的 `/map` 默认订阅、50ms ESDF 更新定时器及显示发布器，不创建传感器融合/清图流程，不预分配三维体素。
   - gridMapCallback() 校验坐标系、尺寸/数据长度、分辨率、原点四元数和平面性及占据值；保存共享消息，按消息更新尺寸/分辨率/平移/旋转与包围范围，标记距离场待更新。支持完整地图替换，不实现增量 map_updates。
   - updateESDF2d() 基于占据>=65及未知策略分类，地图外补障碍圈；使用 OpenCV L2 精确格心距离变换膨胀并计算带符号米制距离。格心到障碍边界使用半格近似，全占据图返回有限负距离。更新完成后才置就绪。
   - gridCoordinates()/gridDistanceWithGrad() 完成坐标-栅格旋转、双线性距离插值与一致梯度（旋转回 map，第三项0）；地图外非正距离、向内梯度，不沿用夹到边界的旧距离查询策略。
   - publishGridMap() 发布 `/sdf_map/grid_inflate` 的 OccupancyGrid；可选抽样 `/sdf_map/esdf` 点云（距离存 intensity）。仅重算完成后发布，默认关闭 ESDF 点云，避免反复发送大图。
3. `fast_planner/plan_env/src/sdf_map.cpp`：initMap() 在旧参数与三维缓存分配前分流到静态模式；原 updateESDFCallback() 调用二维更新，维持“接收后标记、定时器生成距离场”的流程；静态模式关闭旧清图/融合与不适用的发布调用，范围、环绕点和就绪辅助接口接入二维分支。
4. `fast_planner/plan_env/src/edt_environment.cpp`：静态模式 evaluateEDTWithGrad() 直接使用二维插值，不使用地图到来前缓存的旧分辨率；evaluateCoarseEDT() 静态模式只查询静态场，不访问未配置的动态预测对象。setMap()/setEnvironment() 接口不改。
5. `fast_planner/plan_env/CMakeLists.txt`：将新实现源文件加入已有库；nav_msgs 与 OpenCV 已是原依赖，没有新增 TF 或其他依赖。
6. `fast_planner/path_searching/src/kinodynamic_astar.cpp`：搜索入口增加地图就绪检查；静态模式每次刷新实际地图原点/包围范围，拒绝膨胀占据的起终点（含未知、地图外）。普通扩展现有 getInflateOccupancy() 在静态模式自动只使用 x/y，第三项不再当高度；动力学、控制采样和搜索终止框架未改。
7. `fast_planner/plan_manage/src/planner_manager.cpp`：初始化 duration_=0.0；kinodynamicReplan() 开始处等待地图就绪，避免对未生成距离场规划。
8. `fast_planner/plan_manage/src/kino_replan_fsm.cpp`：状态机与安全回调增加地图就绪保护；静态模式检查里程计/目标 frame_id 与配置一致，拒绝空目标，不做假坐标变换；首次没有旧轨迹且目标附近不可用时回 GEN_NEW_TRAJ，避免对空样条 REPLAN。配套修复 trigger_ 未初始化和 waypoint_num_ 作为自身未初始化默认值的问题；不改原固定目标航向来源。
9. `fast_planner/plan_manage/launch/kino_replan.launch.py`：Kino 默认开启静态模式，grid_map_topic=/map、planning_frame=map，未知障碍、阈值65，ESDF点云默认关闭/步长10；原3D参数保留但静态模式不用于分配。实际范围与分辨率严格以地图消息为准，不假造500m地图，也不将旧40x20x5参数扩大成巨型三维缓存。
10. `fast_planner/plan_manage/src/traj_server.cpp`：增加 traj_server/frame_id 参数；`fast_planner/traj_utils/include/traj_utils/planning_visualization.h` 和 `src/planning_visualization.cpp` 增加 visualization/frame_id 参数。Kino launch 将两者配置为 map，使参考输出与显示坐标系匹配；其他调用默认仍为 world。PositionCommand 发布宏和控制接口未改。
11. 新增 `docs/USV_GRID_MAP.md`：记录接口、参数、实际接入方式、话题/QoS、RViz、资源与局限；新增 `tests/check_usv_grid_math.py`：非编译的独立算法/源码接线检查脚本。

### 衔接与实际验证结果

- 本次没有编译工程、运行 C++/ROS、启动地图服务器或仿真。Python 独立数值脚本共8项通过：与小图穷举最近格心距离对照，未知/阈值/膨胀，全空/全占据/单格，行主序和旋转原点，世界坐标梯度与有限差分，航向独立性/地图外，以及500m x 500m、0.25m分辨率（2000x2000）数组规模，另核查源码入口和QoS。
- 检查新增声明/定义、源列表、ROS消息头、共享对象公共访问路径，Python启动文件语法和参数键无重复。
- cppcheck 扩展扫描相关源文件报告134条warning、4条performance和7条error级报告；7条error均在原 planning_visualization.cpp 的旧容器访问/未初始化 lambda 代码中，对修改前备份复扫得到相同7条（按id与消息核对）。没有新增error级报告，但不能将整个工程描述为静态检查无问题。其余包括原有构造成员初始化告警和 Eigen 逗号初始化的可疑运算告警；新参数在 initGridMap() 中读取赋值也被列为构造成员告警。未擅自修改无关旧显示逻辑，不能据此宣称 C++ 编译/运行通过。
- 核心二维持久数据约每格6字节（消息+膨胀+float距离）：500m/0.25m约24MB，500m/0.1m约150MB，不含ROS发布缓存、OpenCV临时矩阵和规划器，峰值更高。实际 C++ 大图耗时与内存待测试；全图更新在原单线程回调中同步执行，没有声称实时性。

### 未完成事项与后续决定

本次仅静态完整地图，不融合雷达动态障碍，不做TF或坐标重命名替代变换，不启动map_server，不配置具体地图文件。需由外部发布有效 `/map`，提供真实 map 下的里程计和目标。消息尺度、分辨率和原点自动生效；若更换地图分辨率，二维查询不使用旧 EDT 分辨率缓存。

膨胀沿用0.099m，尚未按船体长宽确定；1.5m危险距离与2m后端dist0均相对膨胀障碍，应后续统一船体/安全裕度，避免误以为已实现整船轮廓碰撞。距离是栅格近似并采用双线性插值，不等价于连续船体安全证明；沿轨迹0.5s采样、25m前视、50m搜索horizon均不改变。旧3D模式普通扩展第三维、动态搜索时间字段、实际目标航向、局部结束与到达区分、最终可行性验收、超时/停车、控制输出宏、曲线显示将phi作为高度及仿真优化预算等既有待办仍在。不要将接入框架理解为已完成无人船仿真或实船验证。

## 2026-11-11 · 057 · 明确地图元数据来源，障碍膨胀设为 3 m

### 用户要求与修改

用户强调范围不固定为500m，尺寸、分辨率和原点均应取自收到的地图；膨胀半径指定3m，同时询问距离场定时器开销与原无人机地图更新方式。

- 核对 gridMapCallback() 已从 info.resolution、width、height、origin 赋值，不需要改尺寸算法。保留500m测试仅作为大范围示例，不是运行限制或固定尺寸。
- `fast_planner/plan_manage/launch/kino_replan.launch.py` 的 sdf_map/obstacles_inflation 从0.099改为3.0 m，按原参数注入方式生效；静态初始化中的声明默认0.0不改，由当前启动配置覆盖。兼容旧XML/其他三维启动方式未改。
- `fast_planner/plan_env/src/sdf_map_grid.cpp` 仅将初始化注释改为“不分配大范围三维缓存”，避免把500m误解为固定配置。
- `docs/USV_GRID_MAP.md` 明确元数据决定范围、3m膨胀及定时器只检查更新标志。每条有效地图消息都会触发重算，当前未比较相同地图消息内容；没有消息更新则复用距离缓存。
- `tests/check_usv_grid_math.py` 新增3m膨胀与边界膨胀算例。当前决策提醒同步3m，历史记录保持原值作为改动历史。

### 实际验证与衔接

Python启动文件AST确认当前膨胀配置唯一且为3.0，核对尺寸/分辨率取消息、回调置待更新标志、距离场计算后清除标志。9项独立数学/源码契约检查全部通过，包括新增3m膨胀；未编译、未运行C++/ROS、地图服务器或仿真。

50ms仅是更新回调调度周期，不意味着每50ms重算全图。静态地图只收到一次有效消息则算一次；反复发布相同消息仍可能重复全图计算，本次不增加去重或取消定时器。原三维模式同样由 esdf_need_update_ 控制，但点云/深度观测可持续触发局部距离场更新，原实现使用传感器更新地图而非直接订阅静态OccupancyGrid。

距离场基于膨胀后的占据图，危险阈值1.5m、优化dist0=2m、25m前视、0.5s采样保持不变。3m膨胀会压缩狭窄航道，并与后续距离裕度叠加；在直障碍附近粗略相当于原障碍外约4.5m触发监测，实际受栅格化/插值影响，不能作为精确船体安全证明。地图外、未知规则及map坐标系不变，TF和其他既有待办保留。

## 2026-11-11 · 058 · 二维距离场更新检查改为100ms，占据阈值改为75

### 用户要求及最终决定

用户要求距离场定时器50ms改100ms，并将障碍物分类改为栅格值>=75。随后明确“frame_id不改，保持上次”，因此保留记录056的固定map配置及坐标系一致性检查，不从地图消息自动更改frame_id；消息定义和原坐标系传递方式均恢复为本次修改前版本。

### 文件、函数及修改

- `fast_planner/plan_env/src/sdf_map_grid.cpp`：initGridMap() 中二维ESDF定时器周期改为100ms；grid_occupied_threshold参数声明默认从65改为75。仍由地图回调置esdf_need_update_，距离场生成后清除标志，不变成周期无条件重算。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：sdf_map/grid_occupied_threshold配置改为75；膨胀继续3m，其他规划阈值不变。
- `docs/USV_GRID_MAP.md`：同步100ms、75及最终固定map决定，解释0..74在初始分类中无差别视为空闲，并非按占据值大小增加风险代价；仍可能被邻近障碍/边界膨胀覆盖。
- `tests/check_usv_grid_math.py`：将阈值边界算例改为74/75，新增低于75的1/60/74与0产生相同障碍种子/距离场的对照；新增固定坐标系、100ms及消息定义未改的源码契约核查。

### 与此前实现的衔接及含义

尺寸、分辨率和原点继续取地图消息；frame_id按用户最新决定仍取配置map，消息必须匹配，无TF。原三维地图的定时器和安全检查50ms不改，本次100ms仅用于二维静态地图ESDF更新调度。静态地图只收到一次则算一次；相同内容重复发布仍触发重算，未增加去重。

updateESDF2d先将>=75、未知（按现配置）及地图外归为障碍，再以3m半径膨胀，对膨胀后的二值图计算带符号米制距离，负值表示障碍内部、正值表示外部，采用格心及半格近似。占据值不是距离，0..74不单独形成障碍或概率代价，因此若实际希望避开低占据概率区域，需要后续单独决定阈值或风险代价，不在本次自动增加。

### 实际检查

11项独立数学/源码契约检查通过；启动文件AST核对阈值75、膨胀3m。按本次前备份逐字比较确认frame_id相关源文件及Bspline.msg已恢复，sdf_map_grid.cpp只保留100ms与75两处指定差异。此前撤回前的局部静态扫描未报告error级问题，但不代表工程已编译；本次未编译、未运行C++/ROS或仿真。

地图外、未知、船体轮廓、0.5s离散检查、动态字段、停车和最终控制接口等既有待办不变。原始三维分支仍保留历史配置，不应将本条静态地图参数描述为所有启动方式已统一。

## 2026-11-11 · 059 · 静态底图与传感器局部栅格融合，随船更新二维距离场

### 用户要求与已确认决定

保留原SDFMap查询接口，在静态全局地图上接收传感器局部栅格，按实际船位更新100m x 100m窗口，窗口外保留历史缓存。用户确认输入/local_map、frame_id=map；静态已知障碍保留、局部已知空闲清除旧局部障碍、局部未知不清除旧值、有效观测可补充静态未知；另外确认距离场采用可配置的正负10m饱和。全局地图几何仍取/map消息，固定map坐标系、无TF；阈值75、膨胀3m、二维更新检查100ms保持记录058决定。

### 文件、函数与原因

1. `fast_planner/plan_env/include/plan_env/sdf_map.h`：在原SDFMap增加局部历史层、融合层、船位/更新窗口/队列、输入订阅和输出发布成员及辅助函数声明；二维isUnknown/getOccupancy读取融合层，避免占据查询仍读旧静态底图。原三维分支接口保留。
2. `fast_planner/plan_env/src/sdf_map_grid.cpp`：initGridMap新增局部地图与里程计订阅、50m半宽、10m饱和和5s快照周期参数。gridMapCallback保留相同几何下的局部观测，几何变化重置局部层；相同底图消息不触发距离变换，但比较数据仍有全图读取开销。updateESDF2d首次或底图实质变化时全图初始化，之后按随船窗口局部更新；读取窗口外ceil((膨胀+饱和距离)/分辨率)+2格余量，只写回窗口内结果。人工计算框边缘补空闲，仅真实全局边界补障碍，避免出现随窗口移动的假墙。距离值饱和至[-10,10]，地图外负距离也饱和，远场梯度为0。
3. 新增 `fast_planner/plan_env/src/sdf_map_grid_local.cpp`：gridOdometryCallback记录map下实际位置，移入新中心栅格时标记刷新；localGridCallback校验消息并进入最多10帧队列，拒绝倒序非零时间戳，队列满丢弃最旧帧并警告。fusedGridValue/fuseLocalGrid落实用户确定的融合规则。相同分辨率/朝向/格线直接映射；否则按格子重叠面积保守融合，避免细障碍被格心采样漏掉，只有有效空闲完整覆盖全局格才清除旧局部障碍。gridWindow/insideGridWindow支持旋转原点下map轴向100m方形。
4. 同一新文件publishGridMap/checkGridPublication发布融合/膨胀完整快照与map_msgs局部补丁。输出/sdf_map/grid_fused、/sdf_map/grid_fused_updates、/sdf_map/grid_inflate、/sdf_map/grid_inflate_updates，不回发输入/map。有全图订阅者时工程默认每5s发送快照，新订阅者出现也发送，避免晚加入/丢补丁导致显示长期过期；快照不触发ESDF重算，但序列化有全图开销。没有补丁订阅者时跳过补丁构造。
5. `fast_planner/plan_env/src/sdf_map.cpp`：二维定时分支处理待更新标志或观测队列，然后检查快照发布；旧三维流程不变。
6. `fast_planner/plan_env/CMakeLists.txt`、`package.xml`：增加新源文件和标准map_msgs依赖（OccupancyGridUpdate）；当前机器有对应消息头，未安装新依赖。
7. `fast_planner/plan_manage/launch/kino_replan.launch.py`：新增local_grid_topic启动参数默认/local_map，配置grid_local_range=50.0、grid_distance_cap=10.0、grid_snapshot_period=5.0；地图里程计沿用/sdf_map/odom到状态机同一odom_topic的映射。未修改其他规划阈值、动力学、Bspline消息或控制输出接口。
8. `docs/USV_GRID_MAP.md`重写为融合接入说明，包含输入输出、融合/未知规则、更新窗口、读取余量、饱和、资源和限制；`tests/check_usv_grid_math.py`补饱和检查；新增`tests/check_usv_local_grid_math.py`独立检查局部融合与距离数学。本文顶部当前决定和仿真提醒同步本条，历史记录不覆盖。

### 与前后端衔接

EDTEnvironment/search/optimizer/checkTrajCollision继续查询原getInflateOccupancy/getDistance/evaluateEDTWithGrad等接口，共享融合后的缓存；无需改规划调用链。后端dist0=2m、安全阈值1.5m均小于10m饱和上限，数值不变。±10m不是膨胀半径、地图大小或视场；窗口外距离是历史值，不承诺全局距离始终对应最新障碍，船进入后刷新。窗口外读取余量约13m加两格，属于计算辅助区，不扩大观测或写回范围。

### 实际验证

未编译工程、未运行C++/ROS或仿真。两组Python独立数学及源码契约检查12+11共23项通过，包括局部与全图重算对照、新增/清除障碍、窗口外邻近障碍、人工窗口无假墙、真实边界、旋转窗口外缓存不变、细障碍/部分未知覆盖、旋转原点、直接映射和补丁接线；这些不是C++集成或实测性能验证。Python启动文件AST及package.xml解析通过，声明/定义/源列表和本机map_msgs消息字段已核对。cppcheck扫描三个地图实现文件报告43条warning、无error级报告，包含原有成员初始化与Eigen逗号初始化告警；不代表全仓静态无问题，也不能替代编译。

### 未完成事项与换环境注意

实际传感器生成/local_map由上游负责；必须在map坐标系，未观测/遮挡格用-1，不能用0冒充空闲。没有TF、严格观测时间同步、过期检测、动态物体预测或时间衰减；未知/视场外保留旧动态障碍直到再次观测为空闲。静态已知障碍按已确认策略不能被局部空闲删除。

100ms只是调度检查周期，原节点单线程规划仍可阻塞地图/里程计回调；初始全图重算、保守分辨率转换、局部距离变换及全图快照开销待仿真测量。核心持久数据约8字节/格，500m/0.25m约32MB、500m/0.1m约200MB，不含消息缓存、队列和临时矩阵；尺寸完全由实际输入决定。换电脑携带整个工程、文档与测试，按package.xml配置新增map_msgs依赖。船体轮廓/航向扫掠、0.5s离散碰撞采样、最终可行性验收、动态搜索字段、停车和控制器等此前待办不在本条解决。

## 2026-11-11 · 060 · 手动目标航向接入

### 用户要求
在测试工作区中接通 RViz 目标、无人船规划和控制，优先保留 Fast-Planner 接口。

### 项目代码修改
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp` 的 `waypointCallback()`：手动目标航向由固定的 1 rad 改为目标四元数计算的 yaw；四元数归一化并拒绝无效值。原 Path 话题、消息和预设航点处理保持不变。

## 2026-11-12 · 061 · 实际船位重规划与到达判断

### 用户要求及决定
修复新轨迹与实际船位不衔接的问题；用户确定实际到达容差为平面距离 3 m、航向误差 30°。保留 50 m 搜索范围和原 B 样条消息接口。

### 项目代码修改及衔接
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp` 的 `REPLAN_TRAJ`：使用最新规划坐标系里程计位姿和世界系速度作为重新规划起点；里程计没有加速度，使用零加速度假设，然后复用原船体速度与命令转换函数。
- 同文件 `GEN_NEW_TRAJ`、`EXEC_TRAJ`：规划前和参考轨迹到期时，按实际位置与最短航向误差判定是否到达。到期未到达时保留目标并从实际状态继续规划；未配置有效到达容差时不判定到达。
- 同文件 `callKinodynamicReplan()`：规划失败时新增 `/planning/replan_failed` 的带时间戳 Header 通知，允许适配器撤回旧路径；失败后的 `GEN_NEW_TRAJ` 重试间隔为 0.5 s，原 `/planning/bspline` 消息不变。
- `fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h`：新增到达容差、失败通知和失败重试时间成员。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：配置 `fsm/arrival_pos_tolerance=3.0` m、`fsm/arrival_yaw_tolerance=0.5235987755982988` rad。

### 现有边界
新失败通知需要控制侧订阅；未提供加速度测量，重规划仍采用零加速度假设。规划是单线程，规划过程中的实际运动会带来起点时差；实际状态超出搜索限值时可能无法立刻生成新轨迹。

## 2026-11-12 · 限制终端射击的额外整圈

### 用户要求与决定
用户要求排查开阔水域轨迹绕远；确认同意保留现有四段恒定命令射击，增加对额外整圈转向的限制。

### 项目代码修改及衔接
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `computeShotTraj()`：射击目标航向取相对于射击起点最短角差对应的连续分支；末态航向残差和容差检查使用该连续分支，避免相差整圈的末态被视为同一解。
- 同一函数的射击回放：累计射击四段的绝对航向变化，拒绝相对于最短净航向变化额外达到或超过一整圈的候选；仍保留既有船模、四段常值命令、碰撞及速度检查。
- 同一函数在接受射击时输出普通搜索段数、射击段数、射击时长、起末航向及累计转角，以区分纯射击与搜索后射击。

该限制只覆盖射击段，不改变普通搜索阶段的转向约束；在障碍或特殊机动中可能排除原先可用的候选，搜索失败时仍沿用原有回退。


## 2026-11-12 · 暴露末端射击的触发距离

### 用户要求
用户要求排查开阔水域局部轨迹偏离目标方向，并调整相关参数；保留已有终端连接结构和用户确认的实际到达容差。

### 项目代码修改
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：将 `search/goal_pos_tolerance` 的值改为启动参数 `shot_trigger_distance`，通用默认值仍为 25 m；该值控制何时尝试射击，并不是实际到达容差或目标方位角。
- `search/goal_yaw_tolerance` 仍为 60°，比较当前状态航向与目标姿态航向，不是当前位置指向目标的方位角。50 m 搜索 horizon 也保持不变。

### 衔接限制
在其他启动入口覆盖触发距离后，射击可能延伸到局部传感器更新窗口以外；地图和感知的有效范围必须与运行场景匹配。实际到达判断仍使用 `fsm/arrival_pos_tolerance` 和 `fsm/arrival_yaw_tolerance`，两组参数作用不同。

## 2026-11-12 · 修复搜索开放集合的代价更新顺序

### 用户要求
排查开阔水域搜索到局部范围边界时偏离目标方向的问题，保留现有动力学搜索框架。

### 项目代码修改及衔接
- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h`：将开放集合从以可变节点指针为键的优先队列改为按 `f_score` 升序排列的有序集合；代价相同时以节点地址稳定区分，不合并节点。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp`：取最小代价节点、添加节点和重置时相应使用集合操作；两处更新已入集合节点代价的路径均先移除节点、修改后重新插入，保证下次选择反映当前代价。射击仍从当前被选中的搜索节点接续。

此项只修复开放集合排序维护，不改变代价公式、控制采样、局部范围或终端射击的四段结构。相同代价节点的先后顺序可能与原优先队列不同，后续规划结果可能随之变化。

## 2026-11-12 · 暴露搜索启发式权重

### 用户要求
用户要求调整开阔水域规划方向，并保留现有搜索框架。

### 项目代码修改
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：将 `search/lambda_heu` 配置改为启动参数 `search_heuristic_weight`，通用默认仍为 5.0。该参数是调用方对原 `estimateHeuristic()` 返回值的加权系数，未修改距离与航向时间下界公式。

### 衔接限制
其他启动入口可以覆盖此系数；较小权重可能显著增加开放集合规模，节点池仍由原 `search/allocate_num` 限制。与射击触发距离、目标航向及船模共同影响结果，不能保证所有目标的局部轨迹都朝向目标或全局最优。

## 2026-11-12 · 优化预算、来源日志和禁止倒车指令

### 用户要求
优化预算设为 600 次、0.06 秒；显示优化停止原因和搜索/射击来源，FSM 状态间隔 5 秒；搜索与射击的纵向速度命令不允许倒车。空轨迹暂不修改。用户确认控制侧沿用本规划器船模原公式，接受零指令时仍可能缓慢前移。

### 本项目代码修改及衔接
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：普通搜索和四段射击的 `min_v_cmd` 均为 0，最大值仍为 2；`optimization/max_iteration_num2` 为 600、`optimization/max_iteration_time2` 为 0.06。射击纵向上限缩放余量仍生效；下限不再额外加余量，才能实际使用零指令。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `setParam()` 和 `search()`：参数下限不能小于 0，初始命令也按普通搜索上下限裁剪；避免逆模型从零加速度推得负初始指令。`computeShotTraj()` 纵向下限不叠加缩放余量，上限余量不变，使射击允许零指令减速。原船模系数与动力学公式未改变。
- `fast_planner/path_searching/include/path_searching/kinodynamic_astar.h` 的 `getSegmentCounts()`：按实际最终路径节点标记统计普通搜索及射击段数；`fast_planner/plan_manage/src/planner_manager.cpp` 在规划时间日志中显示来源及段数，不改变路径或样条话题。
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `optimize()`：记录 NLopt 返回的停止原因、目标函数评价次数和预算，异常仍按原回退机制处理；保留 `xtol_rel=1e-5`，故可能提前收敛，不强制耗尽预算。
- `fast_planner/plan_manage/src/kino_replan_fsm.cpp` 的 `execFSMCallback()`：仅状态提示按稳定时钟每 5 秒显示，状态机回调和重规划判断周期不变。

### 待定与限制
控制器、仿真器和接口适配位于本项目外；零指令不等于物理制动，原船模纵向零指令平衡速度约 0.0745 m/s。优化边界及代价未修改，完成优化预算不等于保证全局最优或动力学可行。空轨迹撤回逻辑保留，后续需结合路径有效性和碰撞风险决定保留旧轨迹的策略。

## 2026-11-12 · 控制点调整范围与射击失败后继续搜索

### 用户要求
将可变控制点相对初值的 x、y 范围改为各 ±5 m、航向改为 ±30°；接近目标时不因进入射击范围而提前截断普通搜索，射击失败后继续搜索，保留普通搜索与四段射击的连接。

### 项目代码修改及衔接
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `optimize()`：x、y 分量的控制点边界从 ±10 m 改为 ±5 m，仍相对各控制点初始值设置；其他固定控制点与目标函数不变。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：`optimization/yaw_bound` 从 π/3 改为 π/6，即 ±30°。实际到达容差、搜索中的射击触发航向容差及 50 m horizon 均未随之改变。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：进入射击触发范围只触发一次连接尝试；失败后继续扩展普通搜索，后续距离目标比上一次尝试减少至少 5 m 时再次尝试，防止对每个节点都运行射击。射击成功保持原搜索节点加四段射击的路径；达到 horizon 时返回局部搜索路径。开放集耗尽或节点池用尽时，若已有比起点更靠近目标的已搜索节点，返回其中距离目标最近的节点构成的搜索段，否则仍返回无路径。新增射击失败与继续搜索的日志。

### 衔接及限制
5 m 重试间隔是搜索距离进展门限，不是到达容差、避障距离或控制点边界；可能错过相近位置但不同姿态的其他射击候选。前端回退的局部段不代表到达目标，也不保证总能进展；射击的时间限制仍按每次求解生效，多次尝试可能增加单线程规划延迟。此前用户要求不修改空路径撤回机制，本次不改该机制。原船模、零速漂移特性、碰撞检测方式和射击末态约束不变。

## 2026-11-12 · 恢复首次无人船测试时的搜索终止逻辑

### 用户要求
用户澄清“之前的逻辑”指首次无人船开阔水域测试时的搜索逻辑，horizon 为 50 m，其他机制按当时的版本；保留上一轮要求的可变控制点 x/y 各 ±5 m、航向 ±30°。

### 本项目代码修改及衔接
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：根据首次测试前相同的无人船版本恢复 horizon 或 near_end 时结束搜索的分支。near_end 为目标距离和目标航向误差同时满足参数时尝试四段射击；成功返回 REACH_END，失败时普通节点路径返回 NEAR_END，根节点失败返回 NO_PATH。到 horizon 且未成功射击返回 REACH_HORIZON。撤销前一条记录中新加的 5 m 重试间隔和节点池耗尽时返回最佳前进节点；保留开放集合排序修复、当前无人船动力学、禁止倒车和样条接口。
- 开阔水域入口位于本项目以外；其首次测试搜索参数与当前范围不同，按当时 horizon 50 m、射击触发距离 25 m、航向 60°、启发式权重 5 对齐。用户的实际到达容差 3 m/30°不变。

### 限制
首次版本的 near_end 不等同实际到达：射击失败后局部段会在触发范围内提前结束，根节点失败可直接无路径；因此本次按用户指定恢复后，以前报告的短轨迹和卡住问题可能再次出现。若要同时保证射击失败后继续搜索，需要另行明确采用不同于首次版本的分支。

## 2026-11-12 · 恢复用户标注的测试版本配置

### 用户要求
用户要求返回标注的禁倒车、600 次/0.06 s 优化预算版本，接受后续普通搜索段可能变短。

### 项目代码修改与衔接
- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `optimize()`：将可变控制点 x/y 相对初值范围从各 ±5 m 恢复为标注版本的各 ±10 m。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：将 `optimization/yaw_bound` 从 π/6 恢复为 π/3，即 ±60°。优化预算 600 次/0.06 s、50 m horizon、3 m/30°实际到达容差保持标注版本配置。
- 搜索分支已与标注版本一致：near_end 射击失败返回已有搜索段，根节点失败返回无路径，本次不再改动。禁倒车、射击命令零下限、动力学模型、状态日志和来源日志保持标注版本；空路径撤回机制不变。
- 本项目外的开阔水域入口恢复射击触发距离 40 m、启发式权重 2，以对齐标注版本的测试配置。

### 限制
控制点边界是相对各初始控制点的分量范围，非轨迹整体位移限制。进入射击触发范围后搜索段可变短，射击失败仍可能导致根节点无路径；不意味着闭环到达已保证。

## 2026-11-12 · 调整控制点范围与开阔水域射击距离

### 用户要求及修改
用户要求射击触发距离 25 m、控制点 x/y 各 ±5 m、航向 ±30°，并询问射击失败后能否继续普通搜索到 horizon；后者先说明建议，不自行改变搜索策略。

- `fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `optimize()`：可变控制点 x、y 分量相对初值边界从各 ±10 m 改为各 ±5 m。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：`optimization/yaw_bound` 改为 π/6，即 ±30°。
- 开阔水域启动入口位于本项目外，其 `shot_trigger_distance` 默认值改为 25 m；规划器通用启动入口默认也为 25 m。horizon 仍为 50 m，启发式权重开阔水域仍为 2，射击航向触发范围仍为 60°。

### 衔接与限制
搜索终止逻辑不变：符合触发条件后射击失败时，有非根搜索段则返回该段；根节点失败则无有效搜索段并返回 NO_PATH。没有实现射击失败后继续扩展、周期性重试或改变空路径撤回策略。25 m 是触发门限，不保证搜索走过的路程正好为目标距离减 25 m，也不保证射击成功；目标实际到达容差与控制点航向调整范围均为不同参数。

## 2026-11-12 · 将 shot 尝试的航向误差改为 30°

### 用户要求和代码修改
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：`search/goal_yaw_tolerance` 从 π/3 改为 π/6，即搜索状态与目标航向的最短角误差不超过 30°时才满足 shot 触发的航向条件；位置触发条件仍为 25 m，horizon 仍为 50 m。

### 衔接与待定
该参数不是 `optimization/yaw_bound`、实际到达航向容差、shot 末态误差 `shot_yaw_tolerance` 或航向角速度限制，以上参数此次未改。当前根节点在 25 m/30° 内且 shot 失败时，搜索会直接返回 NO_PATH；管理器尝试第二遍但相同根节点仍先射击，因此状态变化不充分时可能反复失败。用户正在选择只修复根节点还是所有 shot 失败时继续扩展；未授权的重试间隔、次数或回退策略不在此条自行确定。

## 2026-11-12 · 调整 shot 时间上限与实际到达容差

### 用户要求及代码修改
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：将单次 `search/shot_time_limit` 从 8 秒改为 4 秒。该值限制每次 shot 求解时长，不是 shot 轨迹持续时间。
- 同一启动文件：用户要求实际到达位置和航向误差稍微调大，采用 `fsm/arrival_pos_tolerance=4.0` m、`fsm/arrival_yaw_tolerance=0.6981317007977318` rad（40°），此前为 3 m、30°。实际船位和最短航向误差同时满足才触发到达。

### 衔接与限制
shot 尝试触发航向误差仍为 30°，shot 本身终态位置和航向容差仍为 0.15 m 和 0.12 rad；它们与状态机实际到达容差用途不同。缩短单次 shot 求解时间可能降低某些连接的成功率；当前根节点 shot 失败仍可能返回无路径，此次不改变搜索回退、空路径撤回、控制器停车或船模。

## 2026-11-12 · 放宽 shot 末态位置和航向容差

### 用户要求及修改
用户要求将 shot 计算结果的末态位置和航向误差适当调大，并了解配置位置。`fast_planner/plan_manage/launch/kino_replan.launch.py`：`search/shot_pos_tolerance` 从 0.15 m 改为 0.30 m，`search/shot_yaw_tolerance` 从 0.12 rad 改为 0.20 rad（约 11.46°）。

### 衔接与限制
`fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `computeShotTraj()` 仍须同时满足位置、连续航向、u/v/r 末态容差，并通过速度、累计转角和碰撞复查；仅放宽前两项，未改求解方法、shot 触发门限、状态机到达容差或禁倒车模型。放宽可能接受偏离目标更大的 shot 终点，不能保证 shot 成功或 B 样条优化后的最终端点满足相同容差。

## 2026-11-12 · shot 三次失败后普通搜索继续

### 用户要求及代码修改
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：将 `search/shot_pos_tolerance` 从用户手动设置的 2.0 m 调整为 2.5 m；`search/shot_yaw_tolerance` 仍为 0.4 rad，修正其旁的角度注释为约 22.92°。
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：一趟搜索内 shot 最多失败三次；在符合原 25 m/30°触发条件的不同展开节点尝试，失败时继续普通搜索，第三次失败后本次搜索不再尝试 shot。搜索再次调用（包括管理器的第二次搜索或后续重规划）时计数重置。新增失败次数日志。
- 第三次失败后，普通搜索抵达符合配置的 2.5 m 位置、0.4 rad 航向及与 shot 一样的 u/v/r 末态容差的非根节点时，直接回溯普通搜索段并返回 REACH_END；该节点来自原运动学传播和碰撞检查，不添加指向目标的未经检查直线。到达 horizon 仍返回局部搜索段。保留原 B 样条消息接口。

### 衔接与限制
三次失败是在三个展开状态上的求解尝试，前两次失败后需要普通扩展取得新状态；搜索耗尽、资源耗尽或已到 horizon 时可能不足三次。每次 shot 上限仍为 4 s，一趟搜索最多约 12 s 的 shot 求解预算，管理器重试会重置计数；实际单线程规划延迟需实测。直接普通搜索终点属于容差内节点，不保证坐标与目标完全重合；它的末态 u/v/r 在原容差内，且只有规划轨迹到期时状态机按实际船位检查 4 m/40°到达。船模零指令漂移、碰撞离散采样、后端优化/时间调整及空路径撤回机制均未改，仍需实际航行验证。

补充：`search()` 在三次 shot 失败后，如果开放集耗尽或节点池耗尽，但已展开经原碰撞检查且比搜索起点更接近目标（至少一个搜索分辨率）的节点，则回溯这些节点中已记录的最佳前进节点并返回 `NEAR_END` 局部段，而不是报告已到达；没有有效前进段时仍返回无路径。该策略不能保证每次重规划均有可行进展或能最终到达，局部段还会经过原 B 样条优化链。

## 2026-11-13 · 控制点航向调整范围改为 ±15°

### 用户要求与修改
用户要求可变控制点相对初值的 x/y 范围各 ±5 m、航向范围 ±15°，并查看航向变化约束；MPC 仅跟踪接收到的轨迹。`fast_planner/plan_manage/launch/kino_replan.launch.py` 的 `optimization/yaw_bound` 从 π/6 改为 π/12（±15°）；`fast_planner/bspline_opt/src/bspline_optimizer.cpp` 同步更新说明，x/y 范围原本已是各 ±5 m，无数值修改。

### 衔接与限制
该边界约束可变航向控制点相对初值的变化，不是整段轨迹相邻航向差或艏摇率上限。现有 `manager/max_yaw_rate`、`manager/max_yaw_acc`、`search/max_r`、`search/shot_max_r` 以及后端航向平滑/可行性权重未修改；后端可行性代价为软约束，时间重分配最多三次后仍可能未满足，不能将配置值视为已证明的最终硬约束。MPC 修改位于本项目外，不改变规划器轨迹与消息接口。

## 2026-11-13 · 普通搜索转向命令代价与航向平滑权重

### 用户要求及修改
- `fast_planner/path_searching/src/kinodynamic_astar.cpp` 的 `search()`：普通扩展的 `tmp_g_score` 从 `(v_cmd²+w_cmd²+um(2)²+w_time)*tau+g` 改为 `(v_cmd²+3*w_cmd²+um(2)²+w_time)*tau+g`，仅提高转向命令二次代价；预留 `um(2)` 仍为零。运动学传播、候选采样、终端 shot 求解器与启发式不变。
- `fast_planner/plan_manage/launch/kino_replan.launch.py`：`optimization/yaw_smoothness_weight` 从 2.0 改为 4.0；`fast_planner/bspline_opt/src/bspline_optimizer.cpp` 的 `calcSmoothnessCost()` 同步说明，既有航向三阶差分代价及梯度均使用该参数，无公式改动。

### 衔接与限制
权重 3 用于普通搜索代价，不是对 w_cmd 施加三倍命令或改变上限；shot 节点的 g_score 仅作路径记账，仍用原公式、不会进入普通开放集排序。权重 4 是航向三阶控制点差分的软惩罚，不能保证曲率、艏摇率或轨迹长度更小。用户只询问指数采样，未改变既有线性采样。
