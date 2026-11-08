cmake_minimum_required(VERSION 3.8)
project(local_costmap_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
include_directories(include)

add_executable(local_costmap_node
  src/local_costmap_node.cpp
)

ament_target_dependencies(local_costmap_node
  rclcpp
  nav_msgs
  std_msgs
)
target_link_libraries(local_costmap_node nlohmann_json::nlohmann_json)

install(TARGETS local_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_inflation test/test_inflation.cpp)
endif()
ament_package()
