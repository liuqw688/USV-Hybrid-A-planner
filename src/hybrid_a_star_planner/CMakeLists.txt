cmake_minimum_required(VERSION 3.8)
project(hybrid_a_star_planner)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Encounter.msg" "msg/EncounterArray.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)

# 包含头文件目录
include_directories(include)

# 编译节点
add_executable(hybrid_a_star_node src/hybrid_a_star_node.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hybrid_a_star_node "${cpp_typesupport_target}")
target_link_libraries(hybrid_a_star_node ${OMPL_LIBRARIES})
ament_target_dependencies(hybrid_a_star_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_colregs test/test_colregs.cpp)
  target_include_directories(test_colregs PRIVATE include)
  ament_add_gtest(test_v4 test/test_v4.cpp TIMEOUT 120)
  ament_target_dependencies(test_v4 rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs visualization_msgs)
  target_link_libraries(test_v4 "${cpp_typesupport_target}")
  target_link_libraries(test_v4 ${OMPL_LIBRARIES})
endif()

# 安装目标
install(TARGETS
  hybrid_a_star_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 和 config 目录
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
ament_package()
