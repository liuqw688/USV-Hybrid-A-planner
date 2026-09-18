# 虚拟船：功能、调用关系与使用说明

本包提供可控的目标船运动真值，用于动态碰撞与会遇测试，不会自行遵守COLREG避让。
参数详表见 PARAMETERS.md；不应把演示通过当作所有实船场景认证。

## 1. 节点与模型

src/virtual_boat_simulator.cpp包含VirtualBoatSimulator。构造内部名称virtual_boat_simulator，launch覆盖为virtual_boat_node以匹配YAML根。
运动直接使用cmd_v/cmd_w，默认2m/s、零角速度匀速前进；没有本船加速度惯性，也没有命令过期停车。
显示长1m宽1.5m，规划碰撞半径2m来自Hybrid target_ship_radii，不由显示长宽自动推导。
msg/BoatStatus.msg用于身份和COG诊断，不是规划器主要目标输入。

## 2. 包间接口

| 方向 | 话题/类型 | 联系与作用 |
|---|---|---|
| 输入 | /target_boat/command / Twist | 改变目标船速率，与本船/cmd_vel分离 |
| 输入 | /initialpose / PoseWithCovarianceStamped | RViz 2D Pose Estimate重设目标位置/航向 |
| 输出 | /target_boat/odom / Odometry | Hybrid预测目标、DWA独立8m保护 |
| 输出 | /target_boat/status / BoatStatus | 身份、显示尺寸、速度和航海COG诊断 |
| 输出 | /target_boat/markers / MarkerArray | RViz船体、黄色速度箭头和名称 |
| 输出 | /tf: odom→target_boat/base_link | 定位目标模型，不能复用本船base_link |
| 输出 | /boat/imu、/boat/gps | 简化传感演示，当前规划链路不依赖 |

initialpose直接取数值，不转换frame；一键map/odom单位TF时一致，非单位TF需另行修正。
GPS以赤道原点近似，不与river_chart经纬原点严格对应。IMU简化重力/角速率，无噪声模型。

## 3. 启动及功能流程

~~~mermaid
flowchart TD
 A["main/构造读取参数"] --> B["建立目标控制/重设订阅及输出"]
 B --> T["update_rate定时器"]
 C["cmdCallback缓存v/w"] --> T
 P["initialPoseCallback重设x/y/yaw"] --> T
 T --> I["updateLoop常速积分并归一化yaw"]
 I --> O["发布TF/odom/status/IMU/GPS/Marker"]
 O --> H["Hybrid与DWA动态预测"]
~~~

## 4. 函数调用流程

~~~mermaid
flowchart LR
 A["updateLoop"] --> B["publishTf"]
 A --> C["publishOdometry"]
 A --> D["publishStatus"]
 A --> E["publishImu"]
 A --> F["publishGps"]
 A --> G["publishMarkers"]
~~~

控制指令只缓存，不立刻发布位置；积分下一周期统一输出。位置重设不自动清除原速度指令。
当前初速为2，20Hz更新；初始船位/yaw可由河道一键target_x/target_y/target_yaw覆盖YAML。

## 5. 操作与调节

独立运行 ros2 launch virtual_boat_simulator virtual_boat.launch.py；河道入口默认关闭，显式use_virtual_boat:=true才启动。
改变initial_speed或发Twist会改变CPA/会遇，风险距离圈不会自动随目标速率更新；需同步Hybrid risk.*_range。
更新频率太低会触发2s目标过期。改变base_frame必须保持独立，避免重现两个本船模型问题。
模拟目标真值话题不等于传感器探测范围，彩色风险圈只是Hybrid诊断，不限制目标odom接收距离。


## 源码函数逐项索引

以下按文件列出已注释函数。`[功能与联系]`代码注释可直接定位职责；同名重载/不同类函数分列。函数内局部计算与分支说明保留原有注释。流程图展示主要调用链，表格覆盖辅助函数、入口和测试。

### src/virtual_boat_simulator.cpp

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `VirtualBoatSimulator` | 加载目标船身份、位置、速度与显示尺寸，订阅独立指令，启动更新循环并发布目标odom/状态/TF。 |
| `cmdCallback` | 缓存目标船独立Twist供updateLoop直接积分；不修改本船命令，也没有本船惯性模型。 |
| `initialPoseCallback` | 接收RViz初始位姿重设虚拟船位置航向；当前不做TF转换，需使用与odom一致的数值坐标。 |
| `updateLoop` | 将虚拟船cmd_v/cmd_w直接作为实际速率积分，随后发布TF、odom、状态和显示；没有本船惯性模型。 |
| `publishTf` | 广播odom到目标船专用base_frame，避免与本船base_link冲突。 |
| `publishOdometry` | 发布目标运动状态供Hybrid/DWA进行匀速预测；船体显示长宽不决定规划半径。 |
| `publishStatus` | 发布目标船身份、显示尺寸、速度和COG诊断；规划预测主要使用目标Odometry。 |
| `publishImu` | 发布简单姿态/角速度IMU样本；不包含真实传感噪声或完整动力学。 |
| `publishGps` | 将平面坐标近似转换为GPS演示数据，假设赤道原点；不能用作真实河图地理定位。 |
| `publishMarkers` | 发布目标船船体、速度箭头及名称；显示长宽不改变规划配置的目标碰撞半径。 |
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |

### launch/virtual_boat.launch.py

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `generate_launch_description` | 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。 |
