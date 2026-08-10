cmake_minimum_required(VERSION 3.8)
project(whycode_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  "msg/Discovery.msg"
  "srv/SetCalibMethod.srv"
  "srv/SetCalibPath.srv"
  "srv/SelectMarker.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
