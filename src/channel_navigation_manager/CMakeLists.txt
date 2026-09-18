cmake_minimum_required(VERSION 3.8)
project(channel_navigation_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(channel_navigation_node src/channel_navigation_node.cpp)
target_include_directories(channel_navigation_node PRIVATE include)
target_link_libraries(channel_navigation_node nlohmann_json::nlohmann_json)
ament_target_dependencies(channel_navigation_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs visualization_msgs)

install(TARGETS channel_navigation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_include_directories(test_geometry PRIVATE include)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
ament_package()
