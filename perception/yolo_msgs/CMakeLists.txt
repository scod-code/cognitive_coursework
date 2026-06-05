cmake_minimum_required(VERSION 3.5)
project(yolo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "srv/StoreLandmark.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
