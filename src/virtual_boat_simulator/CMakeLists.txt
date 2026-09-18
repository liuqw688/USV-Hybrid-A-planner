cmake_minimum_required(VERSION 3.8)
project(virtual_boat_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoatStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(virtual_boat_node src/virtual_boat_simulator.cpp)
ament_target_dependencies(virtual_boat_node
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

# 链接自定义消息
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(virtual_boat_node "${cpp_typesupport_target}")

install(TARGETS virtual_boat_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
ament_package()
