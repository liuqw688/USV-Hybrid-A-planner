cmake_minimum_required(VERSION 3.8)
project(usv_map LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 核心库：海图解析、消息渲染和状态坐标转换，节点与测试共用。
add_library(usv_map_core SHARED
  src/parser.cpp
  src/renderer.cpp
  src/vessel_track.cpp
)
target_include_directories(usv_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(usv_map_core PUBLIC
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tinyxml2::tinyxml2
  tf2::tf2
)

# 海图发布节点。
add_executable(usv_map_node src/node.cpp)
target_link_libraries(usv_map_node PRIVATE
  usv_map_core
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
)

add_executable(usv_odometry_node src/odometry_node.cpp)
target_link_libraries(usv_odometry_node PRIVATE
  usv_map_core
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
)
install(TARGETS usv_map_core usv_map_node usv_odometry_node
  EXPORT export_usv_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz config urdf meshes DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES resource/suiku.xml resource/changjiang.xml resource/ZZ5SUIKU.000
  DESTINATION share/${PROJECT_NAME}/data)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cache")
  install(DIRECTORY cache DESTINATION share/${PROJECT_NAME})
endif()
# 矢量海图由本地工具生成；首次构建时可能不存在，按可选资源安装。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource/generated/reservoir_test_vec.xml")
  install(FILES resource/generated/reservoir_test_vec.xml
    DESTINATION share/${PROJECT_NAME}/data)
endif()
install(FILES requirements.txt DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_parser test/test_parser.cpp)
  target_link_libraries(test_parser usv_map_core)

  ament_add_gtest(test_renderer test/test_renderer.cpp)
  target_link_libraries(test_renderer usv_map_core)

  ament_add_gtest(test_vessel_track test/test_vessel_track.cpp)
  target_link_libraries(test_vessel_track usv_map_core)
endif()

ament_export_targets(export_usv_map HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs
  tinyxml2_vendor TinyXML2 tf2 tf2_ros
)
ament_package()
