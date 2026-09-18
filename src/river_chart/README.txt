river_chart ROS 2 海图与无人船规划功能包（C++ 版本）
作者：susheng
更新时间：2026-08-28

一、工程结构
本目录就是功能包根目录（package.xml + CMakeLists.txt），
可以整体复制或重命名，也可以放入任意工作空间的 src/ 目录，
例如：<工作空间>/src/river_chart/。
include/river_chart/   C++ 头文件（parser / renderer / node）
src/                   C++ 实现
launch/                ROS 2 启动文件
rviz/                  RViz2 配置
resource/ecdis_data.xml  随功能包安装的默认 S-57 XML 海图
test/                  GTest 单元测试与启动文件测试

二、自动路径配置
默认海图不使用 /home/sus 或其他用户目录的固定绝对路径。
编译安装后，节点和启动文件按功能包路径自动解析海图，顺序为：
1) <install>/river_chart/share/river_chart/data/ecdis_data.xml（ament_index 解析）
2) 直接运行 install 内可执行文件时，由自身路径反推同一 share/river_chart/data/
3) 未安装、直接运行源码构建产物时，回退到源码包 resource/ecdis_data.xml
因此整个工作空间复制、改名或移动目录后，只要重新编译，默认输入路径仍然正确，
无需修改任何路径配置；节点日志中的 chart=... 会显示实际读取的文件。
需要使用其他海图时，在启动命令中覆盖 input_path 参数。

三、编译
cd <工作空间根目录>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select river_chart
source install/setup.bash

四、启动
启动节点和 RViz2：
ros2 launch river_chart river_chart.launch.py

仅启动节点：
ros2 launch river_chart river_chart.launch.py rviz:=false

指定外部海图：
ros2 launch river_chart river_chart.launch.py \
  input_path:=/absolute/path/to/ecdis_data.xml

五、主要输出话题
/chart_markers             MarkerArray，海图要素
/chart_metadata            String，海图统计与坐标元数据
/chart_static_obstacles    OccupancyGrid，陆地不可通行栅格
/chart_costmap             OccupancyGrid，规划代价栅格
/chart_channel_direction   MarkerArray，可选航道方向箭头，默认关闭
/chart_lane_data           String，轻量 TSSLPT 多边形与 ORIENT 数据
/chart_occupancy           OccupancyGrid，可选几何栅格诊断图

六、动态障碍物输入
/future_obstacles          geometry_msgs/PoseArray
坐标必须使用 map 坐标系。每个 Pose 的 position.x 和 position.y 表示动态障碍物位置。

七、默认地图规则
绿色：航道/交通分道 TSSBND、TSELNE、TSSLPT
蓝色：等深线 DEPCNT
黄色：浮标 BOYSPP
红色：桥梁 BRIDGE（可通行）
洋红色：泊位 BERTHS（可通行）

规划代价：
航道内：0，可通行且优先。
航道外：60，低速通行带。
陆地 LNDARE：100，不可通行。
未来动态障碍物：100，不可通行。

八、常用参数（括号内为节点默认值）
planning_resolution (0.2)         规划栅格分辨率（米）
planning_max_cells (20000000)     规划栅格最大单元数，超出时自动降低分辨率
grid_resolution (0.2)             诊断栅格分辨率（米）
max_grid_cells (20000000)         诊断栅格最大单元数
channel_corridor_width (20.0)     航道可通行走廊半径（米）
channel_outside_cost (60)         航道外低速通行代价
obstacle_inflation (1.0)          陆地安全膨胀半径（米）
dynamic_obstacle_radius (1.0)     动态障碍物膨胀半径（米）
publish_channel_direction (false) 不显示航道方向箭头
scale (15.0)                      坐标缩放，设为 1.0 时单位为米

九、测试
cd <工作空间根目录>
source /opt/ros/jazzy/setup.bash
colcon test --packages-select river_chart
colcon test-result --verbose
