cmake_minimum_required(VERSION 3.8)
project(usv_map_tools LANGUAGES CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(usv_map REQUIRED)
find_package(GEOS 3.8 REQUIRED)
find_package(GDAL REQUIRED)

add_library(s57_chart STATIC src/s57_chart.cpp)
target_include_directories(s57_chart PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(s57_chart PUBLIC usv_map::usv_map_core GDAL::GDAL)
add_executable(chart_to_s57 src/chart_to_s57.cpp)
target_link_libraries(chart_to_s57 PRIVATE s57_chart)

add_library(berth_chart STATIC src/berth_chart.cpp)
target_include_directories(berth_chart PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(berth_chart PUBLIC tinyxml2::tinyxml2)

add_executable(berth_designer src/berth_designer.cpp)
target_link_libraries(berth_designer PRIVATE
  berth_chart
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

add_library(reef_chart STATIC src/reef_chart.cpp)
target_include_directories(reef_chart PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(reef_chart PUBLIC usv_map::usv_map_core tinyxml2::tinyxml2)

add_executable(reef_designer src/reef_designer.cpp)
target_link_libraries(reef_designer PRIVATE
  reef_chart
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

add_executable(no_go_area_editor src/no_go_area_editor.cpp)
target_link_libraries(no_go_area_editor PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tinyxml2::tinyxml2
)

add_library(clockwise_route STATIC src/clockwise_route.cpp src/route_water.cpp src/fairway_chart.cpp)
target_include_directories(clockwise_route PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(clockwise_route PUBLIC usv_map::usv_map_core GEOS::geos_c tinyxml2::tinyxml2)

add_executable(gnss_path_publisher src/gnss_path_publisher.cpp)
target_link_libraries(gnss_path_publisher PRIVATE
  clockwise_route
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS no_go_area_editor gnss_path_publisher berth_designer reef_designer chart_to_s57
  DESTINATION lib/${PROJECT_NAME})
install(FILES matlab_nv_lse_pe_gnss.txt DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_s57_chart test/test_s57_chart.cpp)
  target_link_libraries(test_s57_chart s57_chart)
  ament_add_gtest(test_berth_chart test/test_berth_chart.cpp)
  target_link_libraries(test_berth_chart berth_chart)
  ament_add_gtest(test_reef_chart test/test_reef_chart.cpp)
  target_link_libraries(test_reef_chart reef_chart GEOS::geos_c)
  ament_add_gtest(test_clockwise_route test/test_clockwise_route.cpp)
  target_link_libraries(test_clockwise_route clockwise_route)
  ament_add_gtest(test_route_clearance test/test_route_clearance.cpp)
  target_link_libraries(test_route_clearance clockwise_route)
  ament_add_gtest(test_fairway test/test_fairway.cpp)
  target_link_libraries(test_fairway clockwise_route)
endif()

ament_package()
