cmake_minimum_required(VERSION 3.8)

project(usv_nmpc_controller)

# install(CODE) 使用目标文件生成器表达式；CMake 3.14 及以上需启用此策略。
if(POLICY CMP0087)
  cmake_policy(SET CMP0087 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(casadi QUIET CONFIG)

# 优先使用 CasADi 导出的 CMake 目标，未找到时搜索头文件和库。
if(casadi_FOUND)
  if(TARGET casadi::casadi)
    set(CASADI_TARGET casadi::casadi)
  elseif(TARGET casadi)
    set(CASADI_TARGET casadi)
  else()
    message(FATAL_ERROR "CasADi 未导出可用的 CMake 目标。")
  endif()
else()
  find_path(CASADI_INCLUDE_DIR NAMES casadi/casadi.hpp
    PATHS /usr/local/include /usr/include /opt/casadi/include)
  find_library(CASADI_LIBRARY NAMES casadi
    PATHS /usr/local/lib /usr/local/lib64 /usr/lib /usr/lib/x86_64-linux-gnu
    /opt/casadi/lib)
  if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
    message(FATAL_ERROR "未找到 CasADi 头文件或库文件。")
  endif()
  add_library(casadi_external UNKNOWN IMPORTED)
  set_target_properties(casadi_external PROPERTIES
    IMPORTED_LOCATION "${CASADI_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CASADI_INCLUDE_DIR}")
  set(CASADI_TARGET casadi_external)
endif()

set(USV_NMPC_SOURCES
  src/usv_model.cpp
  src/bspline_reference.cpp
  src/nmpc_problem.cpp
  src/nmpc_controller.cpp
  src/usv_nmpc_parameters.cpp
  src/tracking_state_machine.cpp
  src/usv_nmpc_node.cpp)

add_library(${PROJECT_NAME}_lib ${USV_NMPC_SOURCES})
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs usv_interfaces)

# ament 使用 plain 形式链接，以下保持同一形式。
target_link_libraries(${PROJECT_NAME}_lib ${CASADI_TARGET})

add_executable(usv_nmpc_node src/main.cpp)
target_include_directories(usv_nmpc_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(usv_nmpc_node ${PROJECT_NAME}_lib)
ament_target_dependencies(usv_nmpc_node
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs usv_interfaces)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)
install(TARGETS usv_nmpc_node RUNTIME DESTINATION lib/${PROJECT_NAME})
# 时间戳相同的文件可能具有不同内容；按内容复制，确保安装程序与构建产物一致。
install(CODE [[
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
      "$<TARGET_FILE:usv_nmpc_node>"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/usv_nmpc_controller/usv_nmpc_node"
    RESULT_VARIABLE usv_nmpc_runtime_sync_result)
  if(NOT usv_nmpc_runtime_sync_result EQUAL 0)
    message(FATAL_ERROR "Unable to synchronize usv_nmpc_node into the install tree")
  endif()
]])
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES PARAMETERS.md DESTINATION share/${PROJECT_NAME})
install(FILES README.md STATE_MACHINE.md DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs std_srvs geometry_msgs nav_msgs usv_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(tracking_state_machine_test test/tracking_state_machine_test.cpp)
  target_link_libraries(tracking_state_machine_test ${PROJECT_NAME}_lib)
  ament_add_gtest(controller_behavior_test test/controller_behavior_test.cpp)
  target_link_libraries(controller_behavior_test ${PROJECT_NAME}_lib)
  ament_add_gtest(bspline_reference_test test/bspline_reference_test.cpp)
  target_link_libraries(bspline_reference_test ${PROJECT_NAME}_lib)
  ament_add_gtest(arrival_completion_test test/arrival_completion_test.cpp)
  target_link_libraries(arrival_completion_test ${PROJECT_NAME}_lib)
  # 与正在运行的仿真隔离，避免回归输入和 /cmd 干扰实际闭环。
  set_tests_properties(arrival_completion_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=187;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  # 校验公共动力学与 CasADi 符号预测的一致性。
  ament_add_gtest(dynamics_consistency_test test/dynamics_consistency_test.cpp)
  target_link_libraries(dynamics_consistency_test ${PROJECT_NAME}_lib)
endif()

ament_package(CONFIG_EXTRAS cmake/casadi_dependency.cmake)
