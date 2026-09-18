# 局部代价地图：功能、调用关系与使用说明

本包对输入物理地图执行距离传播和膨胀，供语义图对齐与两级规划使用。参数详表见 PARAMETERS.md。
“local”是话题名；当前不按船位裁剪滚动窗口，输出仍保持输入整图尺寸和原点。

## 1. 活动源码与历史文件

CMake编译src/local_costmap_node.cpp。src/local_costmap_node_831.cpp是未编译备份，其参数/未知处理与活动版本不同。
不要在备份中改参数后期待一键启动生效。主节点名local_costmap_node，launch/local_costmap.launch.py加载YAML。
注释记录两份文件的差异，本次不删除、不切换实现。

## 2. 包间接口

| 方向 | 话题/类型 | 联系与作用 |
|---|---|---|
| 输入 | /chart_costmap / OccupancyGrid | river_chart物理规划底图，不是左右航道语义 |
| 输出 | /local_costmap / OccupancyGrid | 管理器用其几何元数据；全局A*检查航道外危险；Hybrid/DWA进行物理代价融合 |

输出Reliable+Transient Local；启动晚于海图也能取得一次静态输入，不需要不断重发大地图。
仅收到输入才buildCostmap，没有独立构图timer，也不订阅本船位置。
当前没有从LaserScan构图的节点，/scan不会自动转为本包输入。

## 3. 节点启动和功能流程

~~~mermaid
flowchart TD
 A["main初始化"] --> B["构造:参数/锁存订阅与发布"]
 B --> C["等待原图"]
 C --> D["mapCallback + isValidMap"]
 D --> E["复制header/info;初始化输出"]
 E --> F["buildCostmap"]
 F --> G["发布/local_costmap"]
~~~

~~~mermaid
flowchart TD
 A["按输入分类"] --> U["未知:保留-1"]
 A --> O[">=occupied_threshold:障碍种子"]
 A --> H[">=high_cost_threshold:高代价种子"]
 O --> DO["8邻域Dijkstra:robot_radius+inflation_radius"]
 H --> DH["8邻域Dijkstra:inflation_radius"]
 DO --> C["半径内99;半径外指数衰减"]
 DH --> E["源高代价指数衰减"]
 C --> M["取两源较大代价合并"]
 E --> M
 U --> M
~~~

## 4. 函数调用流程

~~~mermaid
flowchart LR
 A["mapCallback"] --> B["isValidMap"]
 A --> C["buildCostmap"]
 C --> D["isUnknown / indexOf"]
 C --> E["两组距离队列"]
 E --> F["合并输出"]
~~~

障碍半径内输出99、半径外接近98再衰减；当前活动最终合并会使原100障碍格也成为99。
因此不能根据本包输出“100仍全保留”理解安全；Hybrid还读取river_chart原始陆地核心层。
unknown_is_obstacle当前声明但未被buildCostmap使用，负格始终-1。本次仅注明，不修改行为。
minimum_cost是指数基准不是“所有小于该值统一置0”阈值；传播范围之外才由合并逻辑得到0。

## 5. 调参与操作

ros2 launch local_costmap_generator local_costmap.launch.py。
半径/膨胀增大使高代价带变宽，减小可减少航道被岸边膨胀覆盖但也降低裕度。
decay_factor增大使同距离值降得更快，不缩小总传播半径。
阈值occupied降低使更多高代价源变成障碍源；high_cost降低使第二传播种子变多。
阈值与船实际半径不是同一参数；当前robot_radius=3m，而规划半径为1.5m。
后续独立感知障碍应发布到/channel/obstacle_costmap并保留来源，不把它当岸边膨胀清零。


## 源码函数逐项索引

以下按文件列出已注释函数。`[功能与联系]`代码注释可直接定位职责；同名重载/不同类函数分列。函数内局部计算与分支说明保留原有注释。流程图展示主要调用链，表格覆盖辅助函数、入口和测试。

### src/local_costmap_node_831.cpp

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `LocalCostmapNode` | 加载膨胀阈值和距离参数，订阅原图并锁存发布/local_costmap；输入到达才构图，不靠频率反复重建。 |
| `mapCallback` | 验证输入栅格，保持坐标元数据，调用buildCostmap并发布；给航道管理器和两级规划器提供对齐底图。 |
| `isValidMap` | 核对非零尺寸、正分辨率及data长度，避免距离变换越界；由mapCallback前置调用。 |
| `isObstacle` | 历史实现中按占用阈值/未知策略识别膨胀种子；此备份文件不参与当前CMake编译。 |
| `indexOf` | 将二维格坐标变为数组下标；构图的距离场、源代价和最终输出共用。 |
| `buildCostmap` | 分别为硬障碍与高代价种子执行8邻域距离传播，再按半径及指数衰减合并；输出梯度不是左右航道标签。 |
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |

### src/local_costmap_node.cpp

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `LocalCostmapNode` | 加载膨胀阈值和距离参数，订阅原图并锁存发布/local_costmap；输入到达才构图，不靠频率反复重建。 |
| `mapCallback` | 验证输入栅格，保持坐标元数据，调用buildCostmap并发布；给航道管理器和两级规划器提供对齐底图。 |
| `isValidMap` | 核对非零尺寸、正分辨率及data长度，避免距离变换越界；由mapCallback前置调用。 |
| `isUnknown` | 识别负栅格为未知；当前活动实现始终保留-1，unknown_is_obstacle参数未参与此判定。 |
| `indexOf` | 将二维格坐标变为数组下标；构图的距离场、源代价和最终输出共用。 |
| `buildCostmap` | 分别为硬障碍与高代价种子执行8邻域距离传播，再按半径及指数衰减合并；输出梯度不是左右航道标签。 |
| `main` | 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。 |

### launch/local_costmap.launch.py

生产或启动辅助源码。

| 函数 | 主要职责、调用联系与作用 |
|---|---|
| `generate_launch_description` | 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。 |

