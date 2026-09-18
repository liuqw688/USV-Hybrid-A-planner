cmake_minimum_required(VERSION 3.8)
project(dwa_deep)

# 使用 C++17 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(hybrid_a_star_planner REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)             # 用于角度归一化

# ===== DWA 规划器节点 =====
add_executable(dwa_planner src/dwa_planner.cpp)
ament_target_dependencies(dwa_planner
  hybrid_a_star_planner
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

# ===== 无人船仿真器节点 =====
add_executable(boat_simulator src/boat_simulator.cpp)
ament_target_dependencies(boat_simulator
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# 安装可执行文件
install(TARGETS
  dwa_planner
  boat_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch、config rviz目录

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
ament_package()
