cmake_minimum_required(VERSION 3.8)
project(free_water_map)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/free_water_costmap.py
  src/demo_goal_publisher.py
  test/v4_closed_loop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES FUNCTIONAL_GUIDE.md PARAMETERS.md DESTINATION share/${PROJECT_NAME})
ament_package()
