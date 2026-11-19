cmake_minimum_required(VERSION 3.8)
project(usv_interfaces)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleDetection.msg"
  "msg/ObstacleDetectionArray.msg"
  "msg/UsvBsplineControlPoint.msg"
  "msg/UsvBspline.msg"
  DEPENDENCIES std_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
