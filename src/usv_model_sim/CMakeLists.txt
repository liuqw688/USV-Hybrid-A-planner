cmake_minimum_required(VERSION 3.8)
project(usv_model_sim LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(usv_model_sim_dynamics SHARED src/dynamics.cpp)
target_include_directories(usv_model_sim_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(usv_model_node src/model_node.cpp)
target_include_directories(usv_model_node PRIVATE include)
target_link_libraries(usv_model_node usv_model_sim_dynamics)
ament_target_dependencies(usv_model_node rclcpp geometry_msgs sensor_msgs std_msgs)

install(TARGETS usv_model_sim_dynamics
  EXPORT export_usv_model_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS usv_model_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/keyboard_control.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamics test/test_dynamics.cpp)
  target_link_libraries(test_dynamics usv_model_sim_dynamics)
endif()

ament_export_targets(export_usv_model_sim HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs std_msgs)
ament_package()
