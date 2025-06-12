cmake_minimum_required(VERSION 3.5)
project(cone_detector_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedCone.msg"
  "msg/DetectedConeArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
