cmake_minimum_required(VERSION 3.8)
project(simple_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
