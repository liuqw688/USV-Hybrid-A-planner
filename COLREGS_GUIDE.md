# COLREG 风险圆圈、DCPA/TCPA 与安全域说明

当前配置：本船最高速度 4 m/s、避碰半径 1.5 m；虚拟船速度 2 m/s、避碰半径 2 m、显示模型长 1 m 宽 1.5 m；safety_buffer=4 m。模型显示尺寸与保守避碰半径互不替代。

测试尚未全部合格：追越后紧接着在短距离目标停车的净空存在重复失败，
见 [TEST_RESULTS_4MPS.md](TEST_RESULTS_4MPS.md)。圆层显示和停车命令通过，
不等于物理避碰已经安全。

## 1. 先分清“感知、风险、软避让、禁止进入、停车”

目前虚拟船直接发布 /target_boat/odom，Hybrid A* 和 DWA 订阅该状态。因此不存在一个“进入某个圆才感知到虚拟船”的雷达范围：只要收到坐标系正确、时间戳有效的消息，就会处理目标。所有圆都是计算或显示层，不是传感器。

仿真器 /scan 只扫描其内部配置的静态障碍物，不会自动把虚拟船 Marker 变成激光障碍物。真正雷达/AIS感知距离、遮挡、噪声尚未建模。

没有目标消息时，默认静态河道演示可运行（require_target_states=false）。DWA一旦直接收到目标odom，后续该目标超过2秒未更新会停车；目标坐标系不是odom也会停车。严格目标感知测试可设置 require_target_states=true。

## 2. RViz 中每个圆具体是什么

所有下表数值都是两船中心距离或对应DCPA，不是船壳间净空。通过 Displays → COLREG Risk Layers → Namespaces 可以单独开关。

| 外观/命名空间 | 圆心与当前半径 | 实际作用 |
|---|---|---|
| 黄色细线 range_monitor | 目标船当前位置；80 m | 监测距离门槛；须满足DCPA≤10m且TCPA≥0，不是感知范围 |
| 橙色细线 range_action | 目标当前位置；48 m | 行动距离门槛；须满足DCPA≤9m且TCPA≥0，并看让路责任 |
| 红色细线 range_emergency | 目标当前位置；20 m | 紧急距离门槛；须满足DCPA≤8m且TCPA≥0，不是8m停车圈 |
| 黄/橙/红较粗圈 dcpa_monitor/action/emergency | 目标船在max(TCPA,0)时的预测位置；10/9/8 m | 未来最近会遇距离分级参考；以本船同一时刻预测位置是否落入来理解 |
| 青色 avoidance_range | 目标当前位置；一般9 m | Hybrid A*软避让范围：靠近目标预测位置增加代价；必要且满足硬约束时允许进入 |
| 品红色 hard_separation | 目标当前位置参考圈；一般7.5 m、追越9 m | 禁止本船中心进入同一到达时刻的目标预测硬安全域；不是只避开当前静态圆 |
| 紫色 current_stop | 目标当前位置；8 m | DWA直接订阅目标odom，20Hz检查当前中心间距，小于8m发布线/角速度为零的停车命令 |
| 青色宽带 Avoidance Envelope | 沿本船计划轨迹；名义半宽9 m | 显示计划绕行走廊的参考范围；不额外提供硬安全认证，通常有锁定风险时显示，解除后消失 |

基础硬安全距离 = 本船半径1.5 + 目标半径2 + 冗余4 = 7.5 m。
基础软避让距离 = max(硬安全距离, avoidance_radius, 半径之和与buffer乘倍率)
= max(7.5,9,7.5×1.2) = 9 m。
追越硬安全距离 = max(7.5,overtaking_safe_distance=9) = 9 m。
追越硬软边界都为9m时，两者之间没有额外软代价带；不能为缩短绕行突破9m追越硬约束。

不安全分支会被Hybrid A*淘汰；DWA在预测候选轨迹中同样检查动态硬域，并约束避碰航向。全局平滑也重新检查这些条件。

## 3. DCPA/TCPA 到底是什么

CPA是Closest Point of Approach，即两船按当前速度保持匀速航行时的最近会遇点。

设相对位置 r = 目标位置−本船位置，相对速度 v = 目标速度−本船速度：
TCPA = −(r·v)/|v|²，单位秒；
DCPA = |r + v×TCPA|，单位米。
相对速度接近0时程序取TCPA=0，并用当前距离作为DCPA，避免除零。

- TCPA>0：最近会遇在未来；数值越小，剩余行动时间越少。
- TCPA<0：最近会遇已经过去；瞬时风险判为CLEAR，但已锁定的会遇仍要满足解除余量。
- DCPA越小，预测擦碰/碰撞风险越高；它不是当前两船距离。
- 即使当前距离很近，如果正安全远离、DCPA不危险，也不必触发会遇右转；但小于8m的独立停车条件仍覆盖。
- 即使当前距离很远，若DCPA很小且TCPA已进入时间门槛，也会提前避让，不能等到驶进橙色圆再行动。

距离圈按虚拟船速度2m/s换算：40/24/10s对应80/48/20m，不叠加本船速度。
这些是YAML固定距离门槛，目标船速度改变时需要手动同步修改；不是传感器范围。
DCPA/TCPA仍根据两船相对运动计算，不能只用虚拟船航速替代。
例如同一直线相距120m时，TCPA约20s、DCPA约0，进入ACTION；相距60m时
TCPA约10s，虽在20m紧急距离圈外，仍因时间门槛进入EMERGENCY。预警圈不是硬障碍。

TCPA不是空间半径，不会产生一个“秒数圆”。青色预测线上的T+10s、T+24s、T+40s文字，分别标出目标船沿当前速度10/24/40秒后的位置；不是本船必经点，也不是三个停车位置。

DCPA圈圆心取目标在最近会遇时刻的预测位置；TCPA会随两船状态变化，圆心也会移动。TCPA<0时显示用当前位置，不向过去外推画圈。

## 4. 分级判定及进入后行为

程序先要求TCPA≥0，再从EMERGENCY向MONITOR检查：

| 等级 | DCPA条件 | 必须同时满足“距离或时间”之一 |
|---|---|---|
| EMERGENCY | ≤8m | 当前距离≤20m 或 TCPA≤10s |
| ACTION | ≤9m | 当前距离≤48m 或 TCPA≤24s |
| MONITOR | ≤10m | 当前距离≤80m 或 TCPA≤40s |
| CLEAR | 不满足以上组合，或TCPA<0 | 无新会遇锁定 |

实际判定是“DCPA小 AND（距离小 OR TCPA小）”，不是随便进入一个圆就触发。
例如DCPA=15m时，进入黄色80m距离圈也不一定有会遇风险。

锁定会遇后的行为：
- 对遇：右转；监测/行动/紧急级转向里程碑分别为5°/12°/20°。
- 右舷交叉：本船让路，优先右转，从目标船船尾后方通过。
- 左舷交叉或被追越：通常保持航向航速；不等于忽视目标。
- 左舷交叉目标没有有效避让：可能触发Rule17接管并右转，紧急风险会加快接管。
- 有效避让判据随监测DCPA门槛调整，不再固定要求20m净会遇距离；有效避让时不会仅因原风险级别触发紧急接管。
- 追越：本船负避让责任，左右均可选择，但须保持9m追越安全域。

注意：MONITOR在当前实现中已经可能锁定会遇并施加5°机动里程碑，不是仅打印日志。
会遇一旦锁定，不因本船转弯暂时使DCPA变大就立即取消；实际TCPA<-2s且距离>12m，连续2次有效观测后才解除。一般恢复主航道和任务路径。

## 5. 8m停车参数与大惯性边界

### 当前防摇摆方案：有风险才增强软连续性

不使用已撤回的“12秒禁止回左”或“必须等待反事实CPA过去”条件。
Hybrid A*每周期将当前位置到目标的恢复方向作为风险探针，分段计算本船
名义到达时间和目标同步位置。探针不是可执行轨迹，也不会直接禁止一条安全
候选路线；真正候选的运动基元、Dubins连接和平滑结果仍独立检查动态硬域。

恢复探针仍有碰撞风险时，增强对上一安全轨迹的软距离代价；仅在近端
航速×`stability.recovery_horizon`范围作用，并随空间距离减弱。不要求直航，
不锁转向，必要时可以为新障碍改变路线。探针安全时额外代价同周期归零，
即使会遇尚有观测解除余量，也不因此额外拖延回归。普通平滑软代价仍保留。

`stability.recovery_weight=2.0`：增大更稳定，过大可能保留多余绕路；0关闭新增项。
`stability.recovery_horizon=6.0`秒：增大影响更远，减小更局部；不是强制直航时间。
`prediction.goal_braking=true`：所有动态时间积分使用终点减速名义速度；
`goal_deceleration=0.5`与`goal_tolerance=0.4`应与DWA一致。
`prediction.minimum_speed=0.3`仅用于防止预测时间除零，不强制实际最低速度。
该模型未完整模拟控制跟踪误差、流风或未来突然转向，仍须闭环验证。

### 安全通过后不再等会遇记录解除

之前-2秒TCPA、12m距离、两次观测属于历史记录的解除滞回；曾连带保留
航向/保速约束、方向代价和旧避让轨迹吸引，导致安全后恢复偏慢。
现在新增`/colregs/policies`中`recovering`字段：恢复方向无同步碰撞风险，
实际TCPA<0且当前距离大于硬安全距离，并且满足安全通过位置时即设为true。
对遇确认对方已在原航向后方安全域外；交叉确认本船处于对方船尾安全域后方。
这些是位置/净空检查，不是额外保持直航的秒数。

recovering=true时立即释放行动航向/保速约束、方向软代价和旧轨迹距离代价，
DWA收到同一状态继续追踪新的恢复路径，不等locked变false。
动态硬域、交叉船尾通过约束、8m停车、传感器失联保护始终保留。
轨迹仍有曲率平滑、船仍有惯性，恢复是下一次规划开始平滑转回，不是瞬间掉头。
发生新的恢复风险时会重新检查，不能将“允许恢复”理解成忽略其他船。

两个YAML中均配置 colregs_risk_distance: 8.0：
- Hybrid A*中保留兼容的CPA评估距离参数（最终风险策略主要由risk.*决定）。
- DWA中同名参数现在真正负责“当前中心间距<8m发布STOP”，独立于CPA、会遇类别和目标从哪侧经过。

STOP表示发布Twist零速度，不是把船瞬间传送成静止。仿真线加速度限制0.8m/s²时，4m/s减至0的静止目标制动距离约10m、耗时约5s，尚未计算目标船自身靠近。因此8m停车圈是最后一道命令保护，不是足以覆盖高速紧急制动的距离保证。动态预测必须提前避让。实船部署须另外验证完整制动轨迹、流风扰动及感知误差，不应把本仿真结果当作海事合规认证。

## 6. 参数在哪里改、如何显示和检查

Hybrid：src/hybrid_a_star_planner/config/hybrid_a_star_params.yaml
DWA：src/dwa_and_ship_sim/config/dwa_params.yaml
目标模型和速度：src/virtual_boat_simulator/config/virtual_boat.yaml

修改后重启。当前软圈可调avoidance_radius和dynamic_soft_distance_factor，但实际取上述max，不可压小硬安全域。
缩小risk.monitor/action/emergency_*会延迟预警；最高相对接近速度约6m/s，过度缩短时间门槛可能来不及操纵。
修改船体半径、buffer、追越距离时，Hybrid和DWA必须保持一致；target_ship_radii按target_odom_topics列表顺序对应目标。

终点接近时DWA使用goal_deceleration=0.5m/s²推导制动速度上限，再取
0.5×剩余距离的低速收敛上限；10m内增加位置收敛软评分，避免4m/s高速绕终点。
锁定追越且距终点20m内时，DWA增加10秒动态前视，提前检查后船追近风险；
普通显示轨迹仍为3秒。该前视仍按候选匀速转弯预测，不保证停车驻留安全。
左舷交叉演示给直航船约20秒CPA提前量（本船[-80,0]、目标[0,40]）；常规测试通过
不代表10秒近距离突现也能保持安全冗余。当前模型为匀速预测，目标突然转向、
持续撞向已经停车的本船等情况仍可能破坏净空，详见测试报告。

河道使用：
```bash
cd /home/l/work_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch channel_navigation_manager river_navigation.launch.py use_virtual_boat:=true
```
默认虚拟船在[-310,-210]，朝向-0.99483767rad，速度2m/s。本船起点[-270,-270]。用2D Goal Pose设置目标；没有目标时不会生成风险图层。
可通过target_x、target_y、target_yaw参数改变初始位置和朝向。

```bash
ros2 topic echo /colregs/encounter
ros2 topic echo /colregs/policies
ros2 topic echo /cmd_vel
ros2 topic echo /target_boat/odom
```

仅作独立COLREG测试，不与河道系统同时启动：
```bash
ros2 launch free_water_map colregs_demo.launch.py scenario:=head_on
python3 src/free_water_map/test/v4_closed_loop.py head_on --duration 90 --output test_results/current
```
scenario可选head_on、starboard、port_cooperative、port_takeover、overtaking、open_water。
实际最新测试结果见test_results目录，本说明不复用旧尺寸/旧航速的历史PASS数据。
