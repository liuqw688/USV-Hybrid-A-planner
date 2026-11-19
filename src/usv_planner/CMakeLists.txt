cmake_minimum_required(VERSION 3.8)

# 使用当前 CMake 支持的包根目录查找规则，并兼容较旧版本。
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  # PCL 将内部策略版本设为 3.10，需为其未设置的策略提供默认值。
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

project(usv_planner LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(usv_model_sim REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)

find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
find_library(NLOPT_LIBRARY nlopt)
if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "NLopt C++ headers and library are required (libnlopt-cxx-dev).")
endif()

set(PLANNER_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/path_searching/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/plan_env/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/bspline/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/bspline_opt/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/poly_traj/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/traj_utils/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/plan_manage/include
)

add_library(usv_planner_map
  modules/plan_env/src/sdf_map.cpp
  modules/plan_env/src/sdf_map_grid.cpp
  modules/plan_env/src/sdf_map_grid_local.cpp
  modules/plan_env/src/obj_predictor.cpp
  modules/plan_env/src/edt_environment.cpp
  modules/plan_env/src/raycast.cpp
)
target_include_directories(usv_planner_map PUBLIC ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(usv_planner_map
  rclcpp sensor_msgs geometry_msgs nav_msgs map_msgs visualization_msgs
  cv_bridge message_filters pcl_conversions)
target_link_libraries(usv_planner_map Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(usv_planner_spline modules/bspline/src/non_uniform_bspline.cpp)
target_include_directories(usv_planner_spline PUBLIC ${PLANNER_INCLUDE_DIRS})
target_link_libraries(usv_planner_spline Eigen3::Eigen)

add_library(usv_planner_polynomial modules/poly_traj/src/polynomial_traj.cpp)
target_include_directories(usv_planner_polynomial PUBLIC ${PLANNER_INCLUDE_DIRS})
target_link_libraries(usv_planner_polynomial Eigen3::Eigen)

add_library(usv_planner_search_core
  modules/path_searching/src/kinodynamic_astar.cpp
  modules/path_searching/src/usv_dynamics.cpp
  modules/path_searching/src/planar_distance_field.cpp
)
target_include_directories(usv_planner_search_core PUBLIC ${PLANNER_INCLUDE_DIRS})
target_link_libraries(usv_planner_search_core
  Eigen3::Eigen usv_model_sim::usv_model_sim_dynamics)

add_library(usv_planner_search_ros
  modules/path_searching/src/astar.cpp
  modules/path_searching/src/topo_prm.cpp
  modules/path_searching/src/kinodynamic_astar_ros.cpp
)
target_include_directories(usv_planner_search_ros PUBLIC
  ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(usv_planner_search_ros
  rclcpp geometry_msgs nav_msgs map_msgs sensor_msgs visualization_msgs
  cv_bridge message_filters pcl_conversions)
target_link_libraries(usv_planner_search_ros
  Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS}
  usv_planner_search_core usv_planner_map)

add_library(usv_planner_optimizer modules/bspline_opt/src/bspline_optimizer.cpp)
target_include_directories(usv_planner_optimizer PUBLIC ${PLANNER_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIR})
target_link_libraries(usv_planner_optimizer
  Eigen3::Eigen ${NLOPT_LIBRARY})

add_library(usv_planner_optimizer_ros
  modules/bspline_opt/src/bspline_optimizer_ros.cpp)
target_include_directories(usv_planner_optimizer_ros PUBLIC ${PLANNER_INCLUDE_DIRS})
ament_target_dependencies(usv_planner_optimizer_ros rclcpp)
target_link_libraries(usv_planner_optimizer_ros usv_planner_optimizer)

add_library(usv_planner_visualization modules/traj_utils/src/planning_visualization.cpp)
target_include_directories(usv_planner_visualization PUBLIC
  ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(usv_planner_visualization
  rclcpp visualization_msgs geometry_msgs nav_msgs map_msgs sensor_msgs
  cv_bridge message_filters pcl_conversions)
target_link_libraries(usv_planner_visualization
  Eigen3::Eigen ${PCL_LIBRARIES} usv_planner_spline usv_planner_search_core usv_planner_polynomial)

add_library(usv_planner_engine modules/plan_manage/src/planner_manager.cpp)
target_include_directories(usv_planner_engine PUBLIC
  ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(usv_planner_engine
  rclcpp geometry_msgs nav_msgs map_msgs sensor_msgs visualization_msgs
  cv_bridge message_filters pcl_conversions)
target_link_libraries(usv_planner_engine
  Eigen3::Eigen usv_planner_map usv_planner_search_ros usv_planner_spline
  usv_planner_optimizer usv_planner_optimizer_ros usv_planner_polynomial)

add_executable(usv_planner_node
  src/main.cpp
  modules/plan_manage/src/kino_replan_fsm.cpp
)
target_include_directories(usv_planner_node PRIVATE
  ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(usv_planner_node
  rclcpp std_msgs geometry_msgs nav_msgs map_msgs sensor_msgs visualization_msgs
  cv_bridge message_filters pcl_conversions usv_interfaces)
target_link_libraries(usv_planner_node
  Eigen3::Eigen usv_planner_engine usv_planner_visualization)

install(TARGETS
  usv_planner_map usv_planner_spline usv_planner_polynomial usv_planner_search_core
  usv_planner_search_ros usv_planner_optimizer usv_planner_optimizer_ros usv_planner_visualization
  usv_planner_engine usv_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
  modules/path_searching/include/
  modules/plan_env/include/
  modules/bspline/include/
  modules/bspline_opt/include/
  modules/poly_traj/include/
  modules/traj_utils/include/
  modules/plan_manage/include/
  DESTINATION include
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/publish_clockwise_waypoints.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_route test/test_waypoint_route.cpp)
  if(TARGET test_waypoint_route)
    target_include_directories(test_waypoint_route PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_waypoint_route Eigen3::Eigen)
  endif()

  ament_add_gtest(test_usv_dynamics
    modules/path_searching/test/test_usv_dynamics.cpp
    modules/path_searching/test/test_usv_search_state.cpp
    modules/path_searching/test/test_kinodynamic_shot.cpp
    modules/path_searching/test/test_grid_map_costs.cpp)
  if(TARGET test_usv_dynamics)
    target_include_directories(test_usv_dynamics PRIVATE
      ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(test_usv_dynamics
      rclcpp geometry_msgs nav_msgs map_msgs sensor_msgs visualization_msgs
      cv_bridge message_filters pcl_conversions)
    target_link_libraries(test_usv_dynamics
      Eigen3::Eigen usv_planner_search_core usv_planner_map
      usv_model_sim::usv_model_sim_dynamics)
  endif()

  ament_add_gtest(test_bspline_parameterization modules/bspline/test/test_parameterization.cpp)
  if(TARGET test_bspline_parameterization)
    target_include_directories(test_bspline_parameterization PRIVATE ${PLANNER_INCLUDE_DIRS})
    ament_target_dependencies(test_bspline_parameterization rclcpp)
    target_link_libraries(test_bspline_parameterization Eigen3::Eigen usv_planner_spline)
  endif()

  ament_add_gtest(test_trajectory_safety
    modules/plan_manage/test_trajectory_safety.cpp
    modules/bspline_opt/test_navigation_cost.cpp)
  if(TARGET test_trajectory_safety)
    target_compile_definitions(test_trajectory_safety PRIVATE
      USV_PLANNER_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/usv_planner.yaml")
    target_include_directories(test_trajectory_safety PRIVATE
      ${PLANNER_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(test_trajectory_safety
      rclcpp std_msgs geometry_msgs nav_msgs map_msgs sensor_msgs visualization_msgs
      cv_bridge message_filters pcl_conversions)
    target_link_libraries(test_trajectory_safety
      Eigen3::Eigen usv_planner_engine usv_planner_search_core usv_planner_spline
      usv_planner_optimizer usv_planner_map usv_planner_polynomial)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  usv_planner_map usv_planner_spline usv_planner_polynomial usv_planner_search_core
  usv_planner_search_ros usv_planner_optimizer usv_planner_optimizer_ros
  usv_planner_visualization usv_planner_engine)
ament_export_dependencies(usv_interfaces usv_model_sim)
ament_package()
