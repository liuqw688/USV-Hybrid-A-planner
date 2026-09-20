river_chart_1 ROS 2 海图与无人船规划功能包
作者：susheng
更新时间：2026-08-28

一、工程结构
/home/sus/river_chart_1 是 ROS 2 工作空间根目录。
src/ 是 river_chart 功能包根目录，不需要把它移动或重命名。
src/resource/ecdis_data.xml 是随功能包安装的默认 S-57 XML 海图。

二、自动路径配置
默认海图不使用 /home/sus 或其他用户目录的固定绝对路径。
编译安装后，节点和启动文件自动从以下位置读取海图：
<install>/river_chart/share/river_chart/data/ecdis_data.xml

因此整个工作空间复制到其他计算机或改名后，只要重新编译，默认输入路径仍然正确。
需要使用其他海图时，在启动命令中覆盖 input_path 参数。

三、编译
cd /home/sus/river_chart_1
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

八、常用参数
planning_resolution:=10.0       规划栅格分辨率（米）
planning_max_cells:=600000      规划栅格最大单元数，超出时自动降低分辨率
channel_corridor_width:=300.0   航道可通行走廊半径（米）
channel_outside_cost:=60        航道外低速通行代价
obstacle_inflation:=10.0        陆地安全膨胀半径（米）
dynamic_obstacle_radius:=15.0   动态障碍物膨胀半径（米）
publish_channel_direction:=false 不显示航道方向箭头

九、测试
cd /home/sus/river_chart_1
source /opt/ros/jazzy/setup.bash
colcon test --packages-select river_chart
colcon test-result --verbose
