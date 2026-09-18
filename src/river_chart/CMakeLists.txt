cmake_minimum_required(VERSION 3.8)
project(river_chart)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# 核心库：S-57 解析 + 消息渲染，节点和单元测试共用。
add_library(river_chart_lib SHARED
  src/parser.cpp
  src/renderer.cpp
)
target_include_directories(river_chart_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(river_chart_lib PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tinyxml2::tinyxml2
)

# 海图发布节点。
add_executable(river_chart_node src/node.cpp)
target_link_libraries(river_chart_node PRIVATE river_chart_lib ament_index_cpp::ament_index_cpp)
target_compile_definitions(river_chart_node PRIVATE
  RIVER_CHART_SOURCE_DATA="${CMAKE_CURRENT_SOURCE_DIR}/resource/ecdis_data.xml"
)

install(TARGETS river_chart_lib river_chart_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES resource/ecdis_data.xml DESTINATION share/${PROJECT_NAME}/data)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_parser test/test_parser.cpp)
  target_link_libraries(test_parser river_chart_lib)

  ament_add_gtest(test_renderer test/test_renderer.cpp)
  target_link_libraries(test_renderer river_chart_lib)

  # 启动文件仍是 Python，沿用原测试验证其可加载。
  ament_add_pytest_test(test_launch test/test_launch.py)
endif()

ament_package()
