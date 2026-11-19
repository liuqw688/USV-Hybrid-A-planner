cmake_minimum_required(VERSION 3.8)
project(usv_tracking_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS scripts/publish_bspline_reference.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(reference_publisher_test test/test_reference_publisher.py TIMEOUT 30)
  ament_add_pytest_test(global_spline_slice_test
    test/numeric_routes/test_global_spline_slice.py TIMEOUT 120)
endif()

ament_package()
