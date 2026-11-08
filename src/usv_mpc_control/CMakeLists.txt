cmake_minimum_required(VERSION 3.8)
project(usv_mpc_control)

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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# MPC滚动优化与六状态无人船仿真。
add_executable(mpc_controller src/mpc_controller.cpp)
target_include_directories(mpc_controller PRIVATE include)
ament_target_dependencies(mpc_controller
  std_msgs
  tf2_ros
  hybrid_a_star_planner
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# ===== 无人船仿真器节点 =====
add_executable(boat_simulator src/boat_simulator.cpp)
target_include_directories(boat_simulator PRIVATE include)
ament_target_dependencies(boat_simulator
  hybrid_a_star_planner
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vessel_model test/test_vessel_model.cpp)
  target_include_directories(test_vessel_model PRIVATE include)
  ament_target_dependencies(test_vessel_model hybrid_a_star_planner)
  ament_add_gtest(test_identified_model test/test_identified_model.cpp)
  target_include_directories(test_identified_model PRIVATE include)
  ament_target_dependencies(test_identified_model hybrid_a_star_planner)
  ament_add_gtest(test_path_tracking test/test_path_tracking.cpp)
  target_include_directories(test_path_tracking PRIVATE include)
  ament_add_gtest(test_input_frames test/test_input_frames.cpp)
  target_include_directories(test_input_frames PRIVATE include)
  ament_target_dependencies(test_input_frames geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
endif()

# 安装可执行文件
install(TARGETS
  mpc_controller
  boat_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch、config rviz目录

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(hybrid_a_star_planner)
ament_package()
